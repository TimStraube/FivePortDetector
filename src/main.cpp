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
#include <SPI.h>
#include "Screen_ST7735.h"

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


// Ausfuehrliches Pro-Sample-Print braucht bei 921600 Baud laenger als die
// 1ms Symbolperiode - waehrenddessen laeuft der 1kHz-Timer weiter und der
// Schleifenzaehler verliert die Synchronisation zu sym_idx (Symbole werden
// uebersprungen). Deshalb per Default aus.
#define SERIAL_SHOW_ADC 0
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

// ───────── Display-Historie ─────────
static float disp_errPct_I1 = 0, disp_errPct_Q1 = 0;
static float disp_errPct_I2 = 0, disp_errPct_Q2 = 0;
static float disp_doa = 0;

// Board steckt in BoosterPack 2 (nicht 1!) - Pins CS=PP3, RST=PA7, DC=PK7
// laut SPMU372A Table 2-2 (EK-TM4C129EXL-Handbuch).
Screen_ST7735 myScreen(PA_7, PK_7, PP_3, NULL);

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

// ───────── Display Layout (feste Positionen fuer Teil-Updates) ─────────
// Nur die Zahlenwerte werden neu gezeichnet, nicht der ganze Screen - das
// vermeidet das "Aufblitzen" durch ein volles clear() pro Zyklus. Statisch
// (einmal in draw_static_layout) sind nur Titel, Labels und die Achse fuer
// die Winkelanzeige; alles andere wird pro Burst in update_display() neu
// geschrieben (Monospace-Font ueberschreibt alte Ziffern vollstaendig, auch
// wenn die neue Zahl kuerzer ist, solange die Breite gepolstert ist).
#define ROW_TITLE      8
#define ROW_FP1        20
#define ROW_FP2        30
#define ROW_ERR        42
#define BAR_Y0         52
#define BAR_Y1         57
#define ROW_DOA_LABEL  66
#define ROW_DOA_BIG    76
#define DOA_SCALE      3   // Pixel-Skalierung (ix/iy) fuer die grosse DOA-Zahl

#define ANGLE_BAR_X0     4
#define ANGLE_BAR_X1     124
#define ANGLE_LINE_Y     112
#define ANGLE_MARKER_Y   106
#define ANGLE_MARKER_R   3
#define ANGLE_MIN       -90.0f
#define ANGLE_MAX        90.0f

#define STATUS_DOT_X (128 - 6)
#define STATUS_DOT_Y 6
#define STATUS_DOT_R 4

#define X_LABEL 0
#define X_FP    26

static bool layoutReady = false;
static float prevDoaMarkerX = -1;  // <0 = noch kein Marker gezeichnet

static String padNum(float v, uint8_t decimals, uint8_t width)
{
    String s = String(v, decimals);
    while (s.length() < width) s += ' ';
    return s;
}

// Rechtsbuendig (Leerzeichen vorne) - fuer Werte, an die direkt ein Suffix
// wie "%" angehaengt wird, damit das Suffix nicht mit umherspringt.
static String padNumLeft(float v, uint8_t decimals, uint8_t width)
{
    String s = String(v, decimals);
    while (s.length() < width) s = " " + s;
    return s;
}

static void draw_status_dot(uint16_t colour)
{
    myScreen.setPenSolid(true);
    myScreen.circle(STATUS_DOT_X, STATUS_DOT_Y, STATUS_DOT_R, colour);
    myScreen.setPenSolid(false);
}

static uint16_t angle_to_x(float deg)
{
    return ANGLE_BAR_X0 + (uint16_t)((ANGLE_BAR_X1 - ANGLE_BAR_X0) * ((deg - ANGLE_MIN) / (ANGLE_MAX - ANGLE_MIN)));
}

// Statische Elemente: einmalig zeichnen (Titel, Labels, Winkel-Achse)
static void draw_static_layout()
{
    myScreen.clear(blackColour);
    myScreen.setFontSize(0);

    myScreen.gText(X_LABEL, ROW_TITLE, "DOA Detector", whiteColour);
    myScreen.gText(X_LABEL, ROW_DOA_LABEL, "DOA (Grad):", whiteColour);

    // Winkel-Achse: Linie + Endmarken (-90/+90) + Mittelmarke (0 Grad)
    myScreen.line(ANGLE_BAR_X0, ANGLE_LINE_Y, ANGLE_BAR_X1, ANGLE_LINE_Y, whiteColour);
    uint16_t cx = angle_to_x(0);
    myScreen.line(ANGLE_BAR_X0, ANGLE_LINE_Y - 3, ANGLE_BAR_X0, ANGLE_LINE_Y + 3, whiteColour);
    myScreen.line(ANGLE_BAR_X1, ANGLE_LINE_Y - 3, ANGLE_BAR_X1, ANGLE_LINE_Y + 3, whiteColour);
    myScreen.line(cx, ANGLE_LINE_Y - 3, cx, ANGLE_LINE_Y + 3, whiteColour);

    draw_status_dot(redColour);
    layoutReady = true;
    prevDoaMarkerX = -1;
}

// Dynamische Werte: nur diese Felder werden pro Burst aktualisiert
static void update_display()
{
    if (!layoutReady) draw_static_layout();

    myScreen.setFontSize(0);

    // Vor der ersten Kalibrierung sind dc1/dc2 nur Platzhalter (Nullen) -
    // gelb zeigt "noch nicht belastbar", gruen nach calibrate() "gueltig".
    uint16_t dataCol = calibrated ? greenColour : yellowColour;

    myScreen.gText(X_LABEL, ROW_FP1, "FP1:", whiteColour);
    myScreen.gText(X_FP, ROW_FP1,
        padNum(dc1[0],2,4) + " " + padNum(dc1[1],2,4) + " " + padNum(dc1[2],2,4),
        dataCol);

    myScreen.gText(X_LABEL, ROW_FP2, "FP2:", whiteColour);
    myScreen.gText(X_FP, ROW_FP2,
        padNum(dc2[0],2,4) + " " + padNum(dc2[1],2,4) + " " + padNum(dc2[2],2,4),
        dataCol);

    // ── Fehlerrate pro Fiveport ──
    float errAvg1 = (disp_errPct_I1 + disp_errPct_Q1) / 2.0f;
    float errAvg2 = (disp_errPct_I2 + disp_errPct_Q2) / 2.0f;
    float errAvg  = (errAvg1 + errAvg2) / 2.0f;
    uint16_t errCol1 = (errAvg1 < 10.0f) ? greenColour : (errAvg1 < 50.0f) ? yellowColour : redColour;
    uint16_t errCol2 = (errAvg2 < 10.0f) ? greenColour : (errAvg2 < 50.0f) ? yellowColour : redColour;

    myScreen.gText(X_LABEL, ROW_ERR, "Err1:" + padNumLeft(errAvg1,0,3) + "% Err2:" + padNumLeft(errAvg2,0,3) + "%",
        (errAvg1 <= errAvg2) ? errCol2 : errCol1);

    myScreen.setPenSolid(true);
    myScreen.rectangle(0, BAR_Y0, 127, BAR_Y1, blackColour);
    uint16_t barFill = (uint16_t)(127 * (errAvg / 100.0f));
    uint16_t barCol = (errAvg < 10.0f) ? greenColour : (errAvg < 50.0f) ? yellowColour : redColour;
    if (barFill > 0) myScreen.rectangle(0, BAR_Y0, barFill, BAR_Y1, barCol);
    myScreen.setPenSolid(false);
    myScreen.rectangle(0, BAR_Y0, 127, BAR_Y1, whiteColour);

    // ── Status-Punkt oben rechts: gruen = laeuft & Fehlerrate <10% ──
    bool statusOk = calibrated && (errAvg < 10.0f);
    draw_status_dot(statusOk ? greenColour : redColour);

    // ── Grosse DOA-Zahl ──
    myScreen.gText(10, ROW_DOA_BIG, padNum(disp_doa, 1, 6) + " ", whiteColour, blackColour, DOA_SCALE, DOA_SCALE);

    // ── Winkel-Marker (erst alten loeschen, dann neuen zeichnen) ──
    uint16_t mx = angle_to_x(disp_doa);
    myScreen.setPenSolid(true);
    if (prevDoaMarkerX >= 0) {
        myScreen.rectangle((uint16_t)prevDoaMarkerX - ANGLE_MARKER_R - 1, ANGLE_MARKER_Y - ANGLE_MARKER_R - 1,
                            (uint16_t)prevDoaMarkerX + ANGLE_MARKER_R + 1, ANGLE_MARKER_Y + ANGLE_MARKER_R + 1, blackColour);
    }
    myScreen.circle(mx, ANGLE_MARKER_Y, ANGLE_MARKER_R, cyanColour);
    myScreen.setPenSolid(false);
    prevDoaMarkerX = mx;
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

    // GPIO-Ports fuer SPI und Screen-Steuerpins aktivieren (BoosterPack 2!)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);  // PQ0=SCK, PQ2=MOSI, PQ3=MISO (SSI3, BoosterPack 2)
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOQ));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);  // PP3=CS
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOP));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);  // PA7=RST
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);  // PK7=D/C
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK));

    SPI.setModule(4);  // SSI3 auf PQ0/PQ2/PQ3 (BoosterPack 2, wo das EduBP MKII wirklich steckt)
    myScreen.begin();
    // Zeigt dc1/dc2 direkt als Platzhalter (0.00, gelb) an, noch vor der
    // ersten calibrate() - sonst waere der gelbe Zustand nie sichtbar.
    update_display();

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
    float I1_avg = 0, Q1_avg = 0, I2_avg = 0, Q2_avg = 0;
    int err_count_I1 = 0, err_count_Q1 = 0, err_count_I2 = 0, err_count_Q2 = 0;

    for(int n = 0; n < N_CAL; n++){
        while(!new_sample_ready);
        new_sample_ready = false;

        float v1[3], v2[3];
        read_detectors(v1, v2);
        float I1, Q1, I2, Q2;
        demodulate(v1, rg1, ig1, dc1, &I1, &Q1);
        demodulate(v2, rg2, ig2, dc2, &I2, &Q2);
        I1_avg += I1; Q1_avg += Q1;
        I2_avg += I2; Q2_avg += Q2;

        uint8_t decided_I1 = (I1 > 0.5f) ? 1 : 0;
        uint8_t decided_Q1 = (Q1 > 0.5f) ? 1 : 0;
        uint8_t decided_I2 = (I2 > 0.5f) ? 1 : 0;
        uint8_t decided_Q2 = (Q2 > 0.5f) ? 1 : 0;
        err_count_I1 += (decided_I1 != I_sent);
        err_count_Q1 += (decided_Q1 != Q_sent);
        err_count_I2 += (decided_I2 != I_sent);
        err_count_Q2 += (decided_Q2 != Q_sent);

        if(SERIAL_SHOW_ADC){
            Serial.print("n="); Serial.print(n);
            Serial.print(" I_sym="); Serial.print(I_sent);
            Serial.print(" Q_sym="); Serial.print(Q_sent);
            Serial.print(" | FP1 IQ: "); Serial.print(I1,4); Serial.print("  "); Serial.print(Q1,4);
            Serial.print(" | FP2 IQ: "); Serial.print(I2,4); Serial.print("  "); Serial.println(Q2,4);
        }
    }

    I1_avg /= N_CAL; Q1_avg /= N_CAL;
    I2_avg /= N_CAL; Q2_avg /= N_CAL;

    disp_errPct_I1 = 100.0f * err_count_I1 / N_CAL;
    disp_errPct_Q1 = 100.0f * err_count_Q1 / N_CAL;
    disp_errPct_I2 = 100.0f * err_count_I2 / N_CAL;
    disp_errPct_Q2 = 100.0f * err_count_Q2 / N_CAL;

    float phi1 = compute_phase(I1_avg, Q1_avg);
    float phi2 = compute_phase(I2_avg, Q2_avg);

    float dphi = phi1 - phi2;

    // unwrap
    if(dphi > M_PI) dphi -= 2*M_PI;
    if(dphi < -M_PI) dphi += 2*M_PI;

    disp_doa = compute_doa(dphi);

    if(SERIAL_SHOW_IQ){
        Serial.print("Fehlerrate FP1: I="); Serial.print(disp_errPct_I1,1);
        Serial.print("%  Q="); Serial.print(disp_errPct_Q1,1); Serial.println("%");
        Serial.print("Fehlerrate FP2: I="); Serial.print(disp_errPct_I2,1);
        Serial.print("%  Q="); Serial.print(disp_errPct_Q2,1); Serial.println("%");
        Serial.print("AVG FP1 I="); Serial.print(I1_avg,4); Serial.print("  Q="); Serial.println(Q1_avg,4);
        Serial.print("AVG FP2 I="); Serial.print(I2_avg,4); Serial.print("  Q="); Serial.println(Q2_avg,4);
        Serial.print("dphi(deg)="); Serial.print(dphi*180.0f/M_PI,2);
        Serial.print("  DOA(deg)="); Serial.println(disp_doa,2);
    }

    TimerDisable(TIMER0_BASE, TIMER_A);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0);

    update_display();

    delay(1000);
}