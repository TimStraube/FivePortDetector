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
 */

#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <SPI.h>
#include "LCD_SharpBoosterPack_SPI.h"

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

#define SERIAL_SHOW_ADC 1
#define SERIAL_SHOW_IQ  1

#define FS 1000   // 1 kHz

// ───────── Pins ─────────
#define IQ_I_PIN PC_4
#define IQ_Q_PIN PC_5

// ───────── Globals ─────────
volatile uint32_t adc_raw[3];
volatile bool new_sample_ready = false;
volatile int sym_idx = 0;
volatile uint8_t I_sent = 0, Q_sent = 0;

static float rg[3], ig[3], dc[3];
static bool calibrated = false;

static float disp_Iavg = 0, disp_Qavg = 0;

// CS, DISP, VCC; autoVCOM=false (OneMsTaskTimer hat keinen TM4C1294-Zweig -> Linkerfehler)
LCD_SharpBoosterPack_SPI myScreen(6, 5, 2, false);

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

// ───────── ADC Read ─────────
static void read_detectors(float v[3])
{
    v[0] = adc_raw[0] * VREF / ADC_MAXVAL;
    v[1] = adc_raw[1] * VREF / ADC_MAXVAL;
    v[2] = adc_raw[2] * VREF / ADC_MAXVAL;
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
    static float V[N_CAL][3];
    static float A[N_CAL][3];
    static float Ivec[N_CAL], Qvec[N_CAL];

    Serial.println("Calibrating...");

    for(int n=0;n<N_CAL;n++){
        while(!new_sample_ready);
        new_sample_ready=false;

        read_detectors(V[n]);

        // Zentrieren: 0→-0.5, 1→+0.5  (macht mean=0, lstsq korrekt)
        Ivec[n] = (float)I_sent - 0.5f;
        Qvec[n] = (float)Q_sent - 0.5f;
    }

    // dc = Mittelwert der Detektoren
    for(int k=0;k<3;k++){
        dc[k]=0;
        for(int n=0;n<N_CAL;n++) dc[k]+=V[n][k];
        dc[k]/=N_CAL;
    }

    // Residuen (DC-frei)
    for(int n=0;n<N_CAL;n++)
        for(int k=0;k<3;k++)
            A[n][k]=V[n][k]-dc[k];

    // Inverses Modell: finde rg,ig s.d. Σ rg[k]*ṽ[k] ≈ I-0.5
    lstsq3(A,Ivec,N_CAL,rg);
    lstsq3(A,Qvec,N_CAL,ig);

    calibrated=true;
    Serial.println("Calibration done");
    Serial.print("dc:  "); Serial.print(dc[0],4); Serial.print("  "); Serial.print(dc[1],4); Serial.print("  "); Serial.println(dc[2],4);
    Serial.print("rg:  "); Serial.print(rg[0],4); Serial.print("  "); Serial.print(rg[1],4); Serial.print("  "); Serial.println(rg[2],4);
    Serial.print("ig:  "); Serial.print(ig[0],4); Serial.print("  "); Serial.print(ig[1],4); Serial.print("  "); Serial.println(ig[2],4);
}

// ───────── Demod ─────────
static void demodulate(const float v[3], float* I, float* Q)
{
    *I = 0.5f;
    *Q = 0.5f;
    for(int k=0;k<3;k++){
        float vt=v[k]-dc[k];
        *I+=rg[k]*vt;
        *Q+=ig[k]*vt;
    }
}

// ───────── Display ─────────
static void update_display()
{
    const uint8_t dy = 12;
    uint8_t y = 0;

    myScreen.clearBuffer();
    myScreen.setFont(0);

    myScreen.text(0, y, "Five-Port Detector");
    y += dy;

    myScreen.text(0, y,
        "dc:" + String(dc[0],2) + " " + String(dc[1],2) + " " + String(dc[2],2));
    y += dy;

    myScreen.text(0, y,
        "rg:" + String(rg[0],1) + " " + String(rg[1],1) + " " + String(rg[2],1));
    y += dy;

    myScreen.text(0, y,
        "ig:" + String(ig[0],1) + " " + String(ig[1],1) + " " + String(ig[2],1));
    y += dy;

    bool ok = (disp_Iavg > 0.25f && disp_Iavg < 0.75f) && (disp_Qavg > 0.25f && disp_Qavg < 0.75f);
    myScreen.setReverse(!ok);
    myScreen.text(0, y, "AVG: I=" + String(disp_Iavg, 2) + " Q=" + String(disp_Qavg, 2));
    myScreen.setReverse(false);

    myScreen.flush();
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

    // GPIO-Ports für SPI und Screen-Steuerpins aktivieren
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);  // PD3=SCK, PD1=MOSI (SSI2, BoosterPack 1)
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);  // PC6 = LCD DISP
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);  // PE4=LCD VCC, PE5=LCD CS
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));

    SPI.setModule(2);  // SSI2 auf PD3/PD1 (BoosterPack 1, wo das EduBP MKII steckt)
    myScreen.begin();
    myScreen.clearBuffer();
    myScreen.setFont(1);
    myScreen.text(0, 0, "Initializing...");
    myScreen.flush();

    // GPIO
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // ADC Pins PE1/PE2/PE3 als Analogeingang
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

    // ADC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0));

    ADCSequenceConfigure(ADC0_BASE,0,ADC_TRIGGER_PROCESSOR,0);
    ADCSequenceStepConfigure(ADC0_BASE,0,0,ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE,0,1,ADC_CTL_CH1);
    ADCSequenceStepConfigure(ADC0_BASE,0,2,ADC_CTL_CH2 | ADC_CTL_IE | ADC_CTL_END);

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
    sym_idx = 0;
    TimerEnable(TIMER0_BASE, TIMER_A);
    calibrate();

    Serial.println("--- burst ---");
    float I_avg = 0, Q_avg = 0;
    for(int n = 0; n < N_CAL; n++){
        while(!new_sample_ready);
        new_sample_ready = false;

        float v[3];
        read_detectors(v);
        float I, Q;
        demodulate(v, &I, &Q);
        I_avg += I;
        Q_avg += Q;

        if(SERIAL_SHOW_ADC){
            Serial.print("n="); Serial.print(n);
            Serial.print(" I_sym="); Serial.print(I_sent);
            Serial.print(" Q_sym="); Serial.print(Q_sent);
            Serial.print(" | ADC: ");
            Serial.print(v[0],4); Serial.print("V  ");
            Serial.print(v[1],4); Serial.print("V  ");
            Serial.print(v[2],4); Serial.print("V");
            Serial.print(" | IQ: ");
            Serial.print(I,4); Serial.print("  ");
            Serial.println(Q,4);
        }
    }

    TimerDisable(TIMER0_BASE, TIMER_A);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0);

    disp_Iavg = I_avg / N_CAL;
    disp_Qavg = Q_avg / N_CAL;

    if(SERIAL_SHOW_IQ){
        Serial.print("AVG I="); Serial.print(disp_Iavg, 4);
        Serial.print("  Q="); Serial.println(disp_Qavg, 4);
    }

    update_display();

    delay(1000);
}