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


#define D_ANT 0.05f //anpassen auf richtigen abstand in cm
#define LAMBDA 0.1237f

// dphi0 wird weiter berechnet/geloggt (siehe calibrate()), aber NICHT von der
// Messung abgezogen, solange die Kalibrier-ADC-Spanne so klein ist (siehe
// "Kalibrier-ADC-Spanne" im Serial-Log) - dphi0 ist dabei selbst nur Rauschen
// und verzerrt dP/DOA zusaetzlich an der +-180Grad-Wickelgrenze. Auf 1 setzen,
// sobald die Signalstaerke stimmt und dphi0 zwischen Kalibrierungen stabil ist.
#define USE_BORESIGHT_CORRECTION 0


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
static uint32_t lastCalibMillis = 0;

// ───────── Display-Historie ─────────
static float disp_errPct_I1 = 0, disp_errPct_Q1 = 0;
static float disp_errPct_I2 = 0, disp_errPct_Q2 = 0;
static float disp_phi1_deg = 0, disp_phi2_deg = 0, disp_dphi_deg = 0;
static float disp_p1p2_deg = 0;  // geglaetteter P1-P2 Zeigermittelwert, VOR Boresight-Korrektur
static float disp_doa = 0;

// Zeiger-Mittelwert (cos/sin) fuer den gleitenden Mittelwert von P1-P2 ueber
// mehrere Bursts - Start bei Winkel 0 (Zeiger (1,0)).
static float pdiffAvgCos = 1.0f, pdiffAvgSin = 0.0f;

// Referenz-Phasenversatz (Boresight): dphi, das calibrate() bei "identischem"
// Signal auf beiden Fiveports gemessen hat - wird von jeder Messung
// abgezogen, damit ein Pfad-/Kabellaengenunterschied zwischen FP1 und FP2
// nicht als scheinbarer Winkel erscheint.
static float dphi0 = 0;

static void demodulate(const float v[3], float rg[3], float ig[3], float dc[3], float* I, float* Q);
float compute_phase(float I, float Q);

// Board steckt in BoosterPack 2 (nicht 1!) - Pins CS=PP3, RST=PA7, DC=PK7
// laut SPMU372A Table 2-2 (EK-TM4C129EXL-Handbuch).
Screen_ST7735 myScreen(PA_7, PK_7, PP_3, NULL);

// ───────── Training ─────────
// Mehr Samples pro Kalibrierung/Burst = weniger Rauschen in der
// Least-Squares-Loesung (Fehler faellt in etwa mit 1/sqrt(N_CAL)).
// Bei FS=1000Hz dauert ein Burst/eine Kalibrierung N_CAL Millisekunden - bei
// 1000 also ca. 1s pro Messzyklus (kein delay() mehr danach, siehe loop()).
// RAM reicht dafuer locker (V1/A1/V2/A2/Ivec/Qvec zusammen ~56KB von 256KB).
#define N_CAL 1000

// QPSK-Trainingsfolge mit Periode 4 (I: 1,1,0,0 / Q: 1,0,1,0), beliebig lang
// wiederholt - als Formel statt Array, damit N_CAL frei waehlbar bleibt.
static inline uint8_t I_TRAIN_AT(int n) { return ((n & 3) < 2) ? 1 : 0; }
static inline uint8_t Q_TRAIN_AT(int n) { return ((n & 1) == 0) ? 1 : 0; }

// ───────── Timer ISR ─────────
void Timer0IntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    I_sent = I_TRAIN_AT(sym_idx);
    Q_sent = Q_TRAIN_AT(sym_idx);

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

    // Diagnose: Streuung der rohen ADC-Werte je Kanal waehrend der
    // Kalibrierung. Sehr kleine Spannen (Kanal kaum ausgesteuert/haengt fest)
    // oder Werte nahe 0V/VREF (Saettigung) sind typische Ursachen fuer eine
    // schlecht konditionierte Least-Squares-Loesung (riesige rg/ig danach).
    Serial.println("Kalibrier-ADC-Spanne (min..max V) je Kanal:");
    for(int k=0;k<3;k++){
        float min1=V1[0][k], max1=V1[0][k], min2=V2[0][k], max2=V2[0][k];
        for(int n=1;n<N_CAL;n++){
            if(V1[n][k]<min1) min1=V1[n][k];
            if(V1[n][k]>max1) max1=V1[n][k];
            if(V2[n][k]<min2) min2=V2[n][k];
            if(V2[n][k]>max2) max2=V2[n][k];
        }
        Serial.print("  FP1 K"); Serial.print(k); Serial.print(": ");
        Serial.print(min1,3); Serial.print(".."); Serial.print(max1,3);
        Serial.print("  FP2 K"); Serial.print(k); Serial.print(": ");
        Serial.print(min2,3); Serial.print(".."); Serial.println(max2,3);
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
    lastCalibMillis = millis();
    Serial.println("Calibration done");
    Serial.print("dc1:  "); Serial.print(dc1[0],4); Serial.print("  "); Serial.print(dc1[1],4); Serial.print("  "); Serial.println(dc1[2],4);
    Serial.print("rg1:  "); Serial.print(rg1[0],4); Serial.print("  "); Serial.print(rg1[1],4); Serial.print("  "); Serial.println(rg1[2],4);
    Serial.print("ig1:  "); Serial.print(ig1[0],4); Serial.print("  "); Serial.print(ig1[1],4); Serial.print("  "); Serial.println(ig1[2],4);
    Serial.print("dc2:  "); Serial.print(dc2[0],4); Serial.print("  "); Serial.print(dc2[1],4); Serial.print("  "); Serial.println(dc2[2],4);
    Serial.print("rg2:  "); Serial.print(rg2[0],4); Serial.print("  "); Serial.print(rg2[1],4); Serial.print("  "); Serial.println(rg2[2],4);
    Serial.print("ig2:  "); Serial.print(ig2[0],4); Serial.print("  "); Serial.print(ig2[1],4); Serial.print("  "); Serial.println(ig2[2],4);

    // Referenz-Phasenversatz: die gerade aufgenommenen Kalibrierdaten mit den
    // frischen Koeffizienten demodulieren und dphi als Boresight-Nullpunkt
    // merken. Setzt voraus, dass beim Kalibrieren beide Fiveports das
    // gleiche Referenzsignal sehen (z.B. gleiche Quelle, symmetrisch verkabelt).
    float I1sum=0, Q1sum=0, I2sum=0, Q2sum=0;
    for(int n=0;n<N_CAL;n++){
        float I1,Q1,I2,Q2;
        demodulate(V1[n], rg1, ig1, dc1, &I1, &Q1);
        demodulate(V2[n], rg2, ig2, dc2, &I2, &Q2);
        I1sum+=I1; Q1sum+=Q1;
        I2sum+=I2; Q2sum+=Q2;
    }
    float phi1_ref = compute_phase(I1sum/N_CAL, Q1sum/N_CAL);
    float phi2_ref = compute_phase(I2sum/N_CAL, Q2sum/N_CAL);
    dphi0 = phi1_ref - phi2_ref;
    if(dphi0 > M_PI) dphi0 -= 2*M_PI;
    if(dphi0 < -M_PI) dphi0 += 2*M_PI;
    Serial.print("dphi0 (Boresight, Grad): "); Serial.println(dphi0 * 180.0f / M_PI, 2);
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
#define ROW_PHASES     24
#define X_PHASE2       64
#define ROW_DPHI       36
#define X_DPHI         0
#define ROW_PDIFF_LABEL 46
#define ROW_PDIFF_BIG  56
#define DOA_SCALE      3   // Pixel-Skalierung (ix/iy) fuer die grosse P1-P2-Zahl
#define ROW_BTN        92  // statischer Hinweis, welcher Taster kalibriert
#define ROW_CAL_INFO   105 // dynamisch: wie lange die letzte Kalibrierung her ist

#define STATUS_DOT_X (128 - 6)
#define STATUS_DOT_Y 6
#define STATUS_DOT_R 4

#define X_LABEL 0

static bool layoutReady = false;

static String padNum(float v, uint8_t decimals, uint8_t width)
{
    String s = String(v, decimals);
    while (s.length() < width) s += ' ';
    return s;
}

static void draw_status_dot(uint16_t colour)
{
    myScreen.setPenSolid(true);
    myScreen.circle(STATUS_DOT_X, STATUS_DOT_Y, STATUS_DOT_R, colour);
    myScreen.setPenSolid(false);
}

// Statische Elemente: einmalig zeichnen (Titel, Labels, Taster-Hinweis)
static void draw_static_layout()
{
    myScreen.clear(blackColour);
    myScreen.setFontSize(0);

    myScreen.gText(X_LABEL, ROW_TITLE, "DOA Detector", whiteColour);
    myScreen.gText(X_LABEL, ROW_PDIFF_LABEL, "P1 - P2 (mean):", whiteColour);
    myScreen.gText(X_DPHI, ROW_DPHI, "dP:", whiteColour);

    // Hinweis, welcher Taster die Kalibrierung ausloest (PJ0, aendert sich nie)
    myScreen.gText(X_LABEL, ROW_BTN, "Kalibr.: Taste PJ0", whiteColour);

    draw_status_dot(redColour);
    layoutReady = true;
}

// Dynamische Werte: nur diese Felder werden pro Burst aktualisiert
static void update_display()
{
    if (!layoutReady) draw_static_layout();

    myScreen.setFontSize(0);

    // Fehlerrate wird nicht mehr angezeigt, fliesst aber weiter in den
    // Status-Punkt und die Farbe der Phasendifferenz ein.
    float errAvg1 = (disp_errPct_I1 + disp_errPct_Q1) / 2.0f;
    float errAvg2 = (disp_errPct_I2 + disp_errPct_Q2) / 2.0f;
    float errAvg  = (errAvg1 + errAvg2) / 2.0f;
    uint16_t barCol = (errAvg < 10.0f) ? greenColour : (errAvg < 50.0f) ? yellowColour : redColour;

    // ── Status-Punkt oben rechts: gruen = laeuft & Fehlerrate <10% ──
    bool statusOk = calibrated && (errAvg < 10.0f);
    draw_status_dot(statusOk ? greenColour : redColour);

    // ── Phasen (einzeln) + boresight-korrigierte Differenz ──
    myScreen.gText(X_LABEL, ROW_PHASES, "P1:" + padNum(disp_phi1_deg,0,4), whiteColour);
    myScreen.gText(X_PHASE2, ROW_PHASES, "P2:" + padNum(disp_phi2_deg,0,4), whiteColour);
    myScreen.gText(X_DPHI + 18, ROW_DPHI, padNum(disp_dphi_deg,0,4), barCol);
    myScreen.gText(10, ROW_PDIFF_BIG, padNum(disp_p1p2_deg, 0, 4) + " ", whiteColour, blackColour, DOA_SCALE, DOA_SCALE);

    // ── Wie lange die letzte Kalibrierung her ist ──
    if (calibrated) {
        uint32_t agoSec = (millis() - lastCalibMillis) / 1000;
        uint16_t agoCol = (agoSec < 60) ? greenColour : (agoSec < 300) ? yellowColour : redColour;
        myScreen.gText(X_LABEL, ROW_CAL_INFO, "Kal. vor: " + padNum((float)agoSec, 0, 5) + "s", agoCol);
    } else {
        myScreen.gText(X_LABEL, ROW_CAL_INFO, "Kal. vor: nie", redColour);
    }
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
        calibrate_request = false;
        // Timer kurz anhalten, nur um sym_idx race-frei auf 0 zu setzen -
        // VOR calibrate() wieder aktivieren, sonst kommen nie ADC-Samples
        // rein und calibrate()'s while(!new_sample_ready) haengt fuer immer.
        TimerDisable(TIMER0_BASE, TIMER_A);
        sym_idx = 0;
        TimerEnable(TIMER0_BASE, TIMER_A);
        Serial.print("Calibrating both FP...\n");
        calibrate();
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

    // Gleitender Mittelwert von P1-P2 ueber mehrere Bursts, als Mittelung des
    // Zeigers (cos/sin) statt des Winkels direkt - sonst wuerde Mitteln nahe
    // der +-180Grad-Wickelgrenze falsche Ergebnisse liefern (z.B. +179 und
    // -179 gemittelt wäre 0 statt ~180). alpha=Anteil des neuen Bursts.
    const float PDIFF_AVG_ALPHA = 0.5f;
    pdiffAvgCos = (1.0f - PDIFF_AVG_ALPHA) * pdiffAvgCos + PDIFF_AVG_ALPHA * cosf(dphi);
    pdiffAvgSin = (1.0f - PDIFF_AVG_ALPHA) * pdiffAvgSin + PDIFF_AVG_ALPHA * sinf(dphi);
    float dphi_avg = atan2f(pdiffAvgSin, pdiffAvgCos);

    // Boresight-Korrektur: Pfad-/Kabellaengenversatz zwischen FP1/FP2 abziehen,
    // der bei calibrate() mit (vermutlich) identischem Signal gemessen wurde.
    // Per USE_BORESIGHT_CORRECTION abschaltbar (siehe Kommentar oben bei D_ANT).
#if USE_BORESIGHT_CORRECTION
    float dphi_corr = dphi_avg - dphi0;
    if(dphi_corr > M_PI) dphi_corr -= 2*M_PI;
    if(dphi_corr < -M_PI) dphi_corr += 2*M_PI;
#else
    float dphi_corr = dphi_avg;
#endif

    disp_phi1_deg = phi1 * 180.0f / M_PI;
    disp_phi2_deg = phi2 * 180.0f / M_PI;
    disp_p1p2_deg = dphi_avg * 180.0f / M_PI;  // geglaettet, vor Boresight-Korrektur
    disp_dphi_deg = dphi_corr * 180.0f / M_PI;
    disp_doa = compute_doa(dphi_corr);

    if(SERIAL_SHOW_IQ){
        Serial.print("Fehlerrate FP1: I="); Serial.print(disp_errPct_I1,1);
        Serial.print("%  Q="); Serial.print(disp_errPct_Q1,1); Serial.println("%");
        Serial.print("Fehlerrate FP2: I="); Serial.print(disp_errPct_I2,1);
        Serial.print("%  Q="); Serial.print(disp_errPct_Q2,1); Serial.println("%");
        Serial.print("AVG FP1 I="); Serial.print(I1_avg,4); Serial.print("  Q="); Serial.println(Q1_avg,4);
        Serial.print("AVG FP2 I="); Serial.print(I2_avg,4); Serial.print("  Q="); Serial.println(Q2_avg,4);
        Serial.print("dphi_raw(deg)="); Serial.print(dphi*180.0f/M_PI,2);
        Serial.print("  dphi_avg(deg)="); Serial.print(dphi_avg*180.0f/M_PI,2);
        Serial.print("  dphi0(deg)="); Serial.print(dphi0*180.0f/M_PI,2);
        Serial.print("  dphi_korr(deg)="); Serial.print(disp_dphi_deg,2);
        Serial.print("  DOA(deg)="); Serial.println(disp_doa,2);
    }

    TimerDisable(TIMER0_BASE, TIMER_A);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5, 0);

    update_display();
}