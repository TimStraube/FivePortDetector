/**
 * Five-Port Detector – STM32F411RE / EK-TM4C129EXL
 *
 * Kalibrierungsalgorithmus nach:
 *   Neveux et al., "Wide-Band RF Receiver Using the Five-Port Technology",
 *   IEEE Trans. Veh. Technol., 53(5):1441-1451, 2004  [Gleichungen 15–24]
 *
 * Ablauf:
 *   1. Bekannte QPSK-Trainingssymbole per SPI an IQ-Generator senden
 *   2. Drei ADC-Kanäle lesen (Detektorausgänge v_o1, v_o2, v_o3)
 *   3. DC-Offset entfernen, Kleinste-Quadrate-Lösung → rg[], ig[]
 *   4. Demodulation: I = Σ rg[k]·ṽ_k,  Q = Σ ig[k]·ṽ_k
 *
 * Pin-Belegung STM32F411RE Nucleo-64:
 *   SPI1 (IQ-Generator):  SCK=PA5  MOSI=PA7  CS=PA4
 *   ADC Detektor 1:  PA0  (ADC1_IN0, Nucleo A0)
 *   ADC Detektor 2:  PA1  (ADC1_IN1, Nucleo A1)
 *   ADC Detektor 3:  PB0  (ADC1_IN8, Nucleo A3)
 *
 * Pin-Belegung EK-TM4C129EXL (BoosterPack 1):
 *   SPI3/SSI3 (IQ-Generator):  SCK=J1-7(PQ0)  MOSI=J1-15(PQ2)  CS=J1-19(PH2)
 *   ADC Detektor 1:  A0 = J1-2  (PE3 / AIN0)
 *   ADC Detektor 2:  A1 = J1-6  (PE2 / AIN1)
 *   ADC Detektor 3:  A2 = J1-5  (PE1 / AIN2)
 * 
 *   Adc Detektor 4: A3          (PE0 / AIN3)
 *   Adc Detektor 4: A9          (PE4 / AIN9)
 *   Adc Detektor 4: A8          (PE5 / AIN8)
 */

#include <Arduino.h>
#include <math.h>
#include <stdio.h>

// ───────── TM4C Low-Level ─────────
extern "C" {
  #include "inc/hw_memmap.h"
  #include "inc/hw_ints.h"
  #include "driverlib/sysctl.h"
  #include "driverlib/adc.h"
  #include "driverlib/gpio.h"
  #include "driverlib/timer.h"
  #include "driverlib/interrupt.h"
}

// ───────── Settings ─────────
#define VREF 3.3f
#define ADC_MAXVAL 4095.0f


#define D_ANT 0.01f //anpassen auf richtigen abstand
#define LAMBDA 0.1237f


#define SERIAL_SHOW_ADC 1
#define SERIAL_SHOW_IQ  1

#define FS 1000   // 1 kHz

// ───────── Pins ─────────
#define IQ_I_PIN PC_4
#define IQ_Q_PIN PC_5

// ───────── Globals ─────────
volatile uint32_t adc_raw[6];
volatile bool new_sample_ready = false;
volatile int sym_idx = 0;
volatile uint8_t I_sent = 0, Q_sent = 0;


// Button
volatile bool calibrate_request = false;


static float rg1[3], ig1[3], dc1[3];
static float rg2[3], ig2[3], dc2[3];
static bool calibrated = false;

// ───────── Training ─────────
#define N_CAL 16

const uint8_t I_TRAIN[N_CAL] = {
  1, 1, 0, 0,  1, 1, 0, 0,  1, 1, 0, 0,  1, 1, 0, 0
};

const uint8_t Q_TRAIN[N_CAL] = {
  1, 0, 1, 0,  1, 0, 1, 0,  1, 0, 1, 0,  1, 0, 1, 0
};

// ───────── Timer ISR ─────────
void Timer0IntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    I_sent = I_TRAIN[sym_idx];
    Q_sent = Q_TRAIN[sym_idx];

    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, I_sent ? GPIO_PIN_4 : 0);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_5, Q_sent ? GPIO_PIN_5 : 0);

    sym_idx = (sym_idx + 1) % N_CAL;

    ADCProcessorTrigger(ADC0_BASE, 0);

}

// ───────── ADC ISR ─────────
void ADC0SS0IntHandler(void)
{
    ADCIntClear(ADC0_BASE, 0);

    // cast wegen volatile
    ADCSequenceDataGet(ADC0_BASE, 0, (uint32_t*)adc_raw);

    new_sample_ready = true;
}


// ───────── ADC → Float ─────────
void read_detectors(float v1[3], float v2[3])
{
    for(int i=0;i<3;i++){
        v1[i] = adc_raw[i] * VREF / ADC_MAXVAL;
        v2[i] = adc_raw[i+3] * VREF / ADC_MAXVAL;
    }
}


// ───────── Button ISR ─────────
void ButtonIntHandler(void)
{
    GPIOIntClear(GPIO_PORTJ_BASE, GPIO_PIN_0);
    calibrate_request = true;
}

// ───────── Matrix Inversion ─────────
static bool mat3_inv(const float M[3][3], float R[3][3])
{
    float det =
        M[0][0]*(M[1][1]*M[2][2]-M[1][2]*M[2][1])
      - M[0][1]*(M[1][0]*M[2][2]-M[1][2]*M[2][0])
      + M[0][2]*(M[1][0]*M[2][1]-M[1][1]*M[2][0]);

    if (fabsf(det) < 1e-30f) return false;

    float d = 1.0f/det;

    R[0][0]=(M[1][1]*M[2][2]-M[1][2]*M[2][1])*d;
    R[0][1]=-(M[0][1]*M[2][2]-M[0][2]*M[2][1])*d;
    R[0][2]=(M[0][1]*M[1][2]-M[0][2]*M[1][1])*d;
    R[1][0]=-(M[1][0]*M[2][2]-M[1][2]*M[2][0])*d;
    R[1][1]=(M[0][0]*M[2][2]-M[0][2]*M[2][0])*d;
    R[1][2]=-(M[0][0]*M[1][2]-M[0][2]*M[1][0])*d;
    R[2][0]=(M[1][0]*M[2][1]-M[1][1]*M[2][0])*d;
    R[2][1]=-(M[0][0]*M[2][1]-M[0][1]*M[2][0])*d;
    R[2][2]=(M[0][0]*M[1][1]-M[0][1]*M[1][0])*d;

    return true;
}

// ───────── Least Squares ─────────
static bool lstsq3(const float A[][3], const float b[], int N, float c[3])
{
    float ATA[3][3]={};
    float ATb[3]={};

    for(int n=0;n<N;n++){
        for(int i=0;i<3;i++){
            ATb[i]+=A[n][i]*b[n];
            for(int j=0;j<3;j++)
                ATA[i][j]+=A[n][i]*A[n][j];
        }
    }

    float inv[3][3];
    if(!mat3_inv(ATA,inv)) return false;

    for(int i=0;i<3;i++){
        c[i]=0;
        for(int j=0;j<3;j++)
            c[i]+=inv[i][j]*ATb[j];
    }
    return true;
}

// ───────── Calibration ─────────
static void calibrate()
{
    static float V1[N_CAL][3];
    static float A1[N_CAL][3];
    static float Ivec[N_CAL], Qvec[N_CAL];

    static float V2[N_CAL][3];
    static float A2[N_CAL][3];

    Serial.println("Calibrating...");

    for(int n=0;n<N_CAL;n++){
        while(!new_sample_ready);
        new_sample_ready=false;

        read_detectors(V1[n], V2[n]);

        // Zentrieren: 0→-0.5, 1→+0.5  (macht mean=0, lstsq korrekt)
        Ivec[n] = (float)I_sent - 0.5f;
        Qvec[n] = (float)Q_sent - 0.5f;
    }

    // dc = Mittelwert der Detektoren
    for(int k=0;k<3;k++){
        dc1[k]=0;
        dc2[k]=0;
        for(int n=0;n<N_CAL;n++){
         dc1[k]+=V1[n][k];
         dc2[k]+=V2[n][k];
        }
        dc1[k]/=N_CAL;
        dc2[k]/=N_CAL;
    }

    // Residuen (DC-frei)
    for(int n=0;n<N_CAL;n++)
        for(int k=0;k<3;k++){
            A1[n][k]=V1[n][k]-dc1[k];
            A2[n][k]=V2[n][k]-dc2[k];
        }

    // Inverses Modell: finde rg,ig s.d. Σ rg[k]*ṽ[k] ≈ I-0.5
    lstsq3(A1,Ivec,N_CAL,rg1);
    lstsq3(A1,Qvec,N_CAL,ig1);
    lstsq3(A2,Ivec,N_CAL,rg2);
    lstsq3(A2,Qvec,N_CAL,ig2);

    calibrated=true;
    Serial.println("Calibration done");
    Serial.print("dc1:  "); Serial.print(dc1[0],4); Serial.print("  "); Serial.print(dc1[1],4); Serial.print("  "); Serial.println(dc1[2],4);
    Serial.print("rg1:  "); Serial.print(rg1[0],4); Serial.print("  "); Serial.print(rg1[1],4); Serial.print("  "); Serial.println(rg1[2],4);
    Serial.print("ig1:  "); Serial.print(ig1[0],4); Serial.print("  "); Serial.print(ig1[1],4); Serial.print("  "); Serial.println(ig1[2],4);
    Serial.print("dc2:  "); Serial.print(dc2[0],4); Serial.print("  "); Serial.print(dc2[1],4); Serial.print("  "); Serial.println(dc2[2],4);
    Serial.print("rg2:  "); Serial.print(rg2[0],4); Serial.print("  "); Serial.print(rg2[1],4); Serial.print("  "); Serial.println(rg2[2],4);
    Serial.print("ig2:  "); Serial.print(ig2[0],4); Serial.print("  "); Serial.print(ig2[1],4); Serial.print("  "); Serial.println(ig2[2],4);
}

// ───────── Demod ─────────
static void demodulate(const float v[3], float rg[3], float ig[3], float dc[3], float* I, float* Q)
{
    *I = 0.5f;
    *Q = 0.5f;
    for(int k=0;k<3;k++){
        float vt=v[k]-dc[k];
        *I+=rg[k]*vt;
        *Q+=ig[k]*vt;
    }
}


// ───────── Phase ─────────
float compute_phase(float I, float Q)
{
    return atan2f(Q - 0.5f, I - 0.5f);
}

// ───────── DOA ─────────
float compute_doa(float dphi)
{
    // clamp
    float x = dphi * LAMBDA / (2*M_PI*D_ANT);
    if(x > 1) x = 1;
    if(x < -1) x = -1;

    return asinf(x) * 180.0f / M_PI;
}

// ───────── Setup ─────────
void setup()
{
    uint32_t freq = SysCtlClockFreqSet(
        SYSCTL_XTAL_25MHZ |
        SYSCTL_OSC_MAIN |
        SYSCTL_USE_PLL |
        SYSCTL_CFG_VCO_480,
        120000000);

    Serial.begin(921600);

    // GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // ADC Pins PE1/PE2/PE3 als Analogeingang
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3
                                  | GPIO_PIN_0 | GPIO_PIN_4 | GPIO_PIN_5);

    // ADC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));

    ADCSequenceConfigure(ADC0_BASE,0,ADC_TRIGGER_PROCESSOR,0);
    ADCSequenceStepConfigure(ADC0_BASE,0,0,ADC_CTL_CH0); // PE3
    ADCSequenceStepConfigure(ADC0_BASE,0,1,ADC_CTL_CH1); // PE2
    ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH2); // PE1
    ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH3); // PE0
    ADCSequenceStepConfigure(ADC0_BASE, 0, 4, ADC_CTL_CH9); // PE4
    ADCSequenceStepConfigure(ADC0_BASE, 0, 5,
        ADC_CTL_CH8 | ADC_CTL_IE | ADC_CTL_END);// PE5

    // Button PJ0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    
    GPIOIntTypeSet(GPIO_PORTJ_BASE,
                GPIO_PIN_0,
                GPIO_FALLING_EDGE);

    GPIOIntEnable(GPIO_PORTJ_BASE, GPIO_PIN_0);
    IntEnable(INT_GPIOJ);
    GPIOIntRegister(GPIO_PORTJ_BASE, ButtonIntHandler);


    ADCHardwareOversampleConfigure(ADC0_BASE, 64);
    ADCSequenceEnable(ADC0_BASE,0);
    ADCIntEnable(ADC0_BASE,0);
    ADCIntClear(ADC0_BASE,0);

    // Timer
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0));

    TimerConfigure(TIMER0_BASE,TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE,TIMER_A,(freq/FS)-1);

    
    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0IntHandler);
    ADCIntRegister(ADC0_BASE, 0, ADC0SS0IntHandler);
    IntEnable(INT_TIMER0A);
    IntEnable(INT_ADC0SS0);
    TimerIntEnable(TIMER0_BASE,TIMER_TIMA_TIMEOUT);


    IntMasterEnable();

    TimerEnable(TIMER0_BASE,TIMER_A);

    delay(500);

    calibrate();
}

// ───────── Loop ─────────
void loop()
{
    
    TimerEnable(TIMER0_BASE, TIMER_A);
    
    if(calibrate_request){
        TimerDisable(TIMER0_BASE, TIMER_A);
        calibrate_request = false;
        sym_idx = 0;
        Serial.print("Calibrating both FP...\n");
        calibrate();
        TimerEnable(TIMER0_BASE, TIMER_A);
    }


    Serial.println("--- burst ---");
    float I_avg = 0, Q_avg = 0;
    while(!new_sample_ready);
    new_sample_ready = false;

    float v1[3];
    float v2[3];
    read_detectors(v1, v2);
    float I1,Q1,I2,Q2;
    demodulate(v1, rg1, ig1, dc1, &I1, &Q1);
    demodulate(v2, rg2, ig2, dc2, &I2, &Q2);


    float phi1 = compute_phase(I1,Q1);
    float phi2 = compute_phase(I2,Q2);

    float dphi = phi1 - phi2;

    // unwrap
    if(dphi > M_PI) dphi -= 2*M_PI;
    if(dphi < -M_PI) dphi += 2*M_PI;

    float doa = compute_doa(dphi);

    float y = dphi * 180 / M_PI;
    Serial.print(y);
    Serial.print(doa);


    TimerDisable(TIMER0_BASE, TIMER_A);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0);

    delay(100);
}