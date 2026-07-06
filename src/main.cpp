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
 * Screen (Crystalfontz/EduMKII) steckt real in BoosterPack 2, nicht 1!
 * Laut SPMU372A (EK-TM4C129EXL User's Guide) Table 2-2:
 *   SPI SSI3 auf Port Q:  CLK=PQ0  MOSI=PQ2(XDAT0)  MISO=PQ3(XDAT1)
 *   CS=PP3 (D2,8)  RST=PA7 (D2,4)  DC=PK7 (C2,10)
 * (Positionsabgleich mit BoosterPack-1-Tabelle: CS/RST/DC liegen auf
 * BP1 und BP2 jeweils an der GLEICHEN Header/Pin-Position, nur mit
 * anderen GPIOs dahinter.)
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

// Diagnose-Modus: haengt in setup() fest und schaltet nur CS/RST/DC
// langsam (1Hz), zum Nachmessen mit Multimeter. 0 = normaler Betrieb.
#define PIN_TEST_MODE 0

// Ausfuehrliches Pro-Sample-Print braucht bei 921600 Baud ~1.3-1.6ms/Zeile -
// laenger als die 1ms Symbolperiode. Waehrend gedruckt wird, laeuft der
// 1kHz-Timer weiter und ueberschreibt adc_raw/I_sent/Q_sent im Hintergrund,
// wodurch der Schleifenzaehler n gegenueber sym_idx aus dem Tritt geraet
// (Symbole werden uebersprungen). Deswegen hier aus.
#define SERIAL_SHOW_ADC 0
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

// Board steckt real in BoosterPack 2 (nicht 1!) - Pins CS=PP3, RST=PA7,
// DC=PK7 laut SPMU372A Table 2-2 (EK-TM4C129EXL-Handbuch). Die
// Default-Pins der Klasse (13/17/31 = PN2/PH3/PL3) gelten nur fuer
// BoosterPack 1 und passen hier nicht.
Screen_ST7735 myScreen(PA_7, PK_7, PP_3, NULL);

// ───────── Training ─────────
#define N_CAL 16

const uint8_t I_TRAIN[N_CAL] = {
  1, 1, 0, 0,  1, 1, 0, 0,  1, 1, 0, 0,  1, 1, 0, 0
};

const uint8_t Q_TRAIN[N_CAL] = {
  1, 0, 1, 0,  1, 0, 1, 0,  1, 0, 1, 0,  1, 0, 1, 0
};

// ───────── Display-Historie (fuer Fehlerrate + IQ-Plot) ─────────
static float disp_errPct_I = 0, disp_errPct_Q = 0;
static float iq_hist_I[N_CAL], iq_hist_Q[N_CAL];
static uint8_t iq_hist_class[N_CAL];  // 0..3 = welches der 4 QPSK-Symbole erkannt wurde

// ───────── Klassifikations-Wahrscheinlichkeiten (die 4 QPSK-Zustaende) ─────────
// Klasse = decided_I*2 + decided_Q, passend zur (bi,bq)-Zaehlreihenfolge im Plot.
static const uint16_t CLASS_COLOUR[4] = { greenColour, yellowColour, cyanColour, magentaColour };
static float classProb[4] = { 0.25f, 0.25f, 0.25f, 0.25f };
static uint32_t classSampleCount = 0;

// Iterative Mittelwertbildung (laufender Schaetzer), kein Speichern der
// ganzen Historie noetig: p[c] konvergiert gegen die relative Haeufigkeit.
static void update_class_prob(uint8_t classIdx)
{
    classSampleCount++;
    for (uint8_t c = 0; c < 4; c++) {
        float indicator = (c == classIdx) ? 1.0f : 0.0f;
        classProb[c] += (indicator - classProb[c]) / (float)classSampleCount;
    }
}

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

// ───────── Display Layout (feste Positionen fuer Teil-Updates) ─────────
// Nur die Zahlenwerte werden neu gezeichnet, nicht der ganze Screen -
// das vermeidet das "Aufblitzen" durch ein volles clear() pro Zyklus.
// Monospace-Font vorausgesetzt: feste Breite ueberschreibt alte Ziffern
// vollstaendig, auch wenn die neue Zahl kuerzer ist (z.B. Vorzeichenwechsel).
#define ROW_TITLE 10
#define ROW_DC    20
#define ROW_RG    30
#define ROW_IG    40
#define ROW_AVG   54
#define ROW_ERR   64
#define BAR_Y0    74
#define BAR_Y1    79
#define PLOT_X0   2
#define PLOT_X1   125
#define PLOT_Y0   84
#define PLOT_Y1   126

#define STATUS_DOT_X (128 - 6)
#define STATUS_DOT_Y 6
#define STATUS_DOT_R 4

#define X_LABEL   0
#define X_VALUES  24
#define X_AVG_I   30
#define X_AVG_Q   78
#define X_ERR_I   42
#define X_ERR_Q   96

static bool layoutReady = false;
static float plotPrevI[N_CAL], plotPrevQ[N_CAL];
static bool  plotHasPrev = false;

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

#define PROB_R_MAX 6

static const float PLOT_LO = -0.3f, PLOT_HI = 1.3f;

// Pixelposition des idealen Referenzpunkts fuer Symbol (bi,bq) im Plot.
static void ref_point_xy(uint8_t bi, uint8_t bq, uint16_t &cx, uint16_t &cy)
{
    cx = PLOT_X0 + (uint16_t)((PLOT_X1 - PLOT_X0) * ((bi - PLOT_LO) / (PLOT_HI - PLOT_LO)));
    cy = PLOT_Y1 - (uint16_t)((PLOT_Y1 - PLOT_Y0) * ((bq - PLOT_LO) / (PLOT_HI - PLOT_LO)));
}

static void draw_status_dot(uint16_t colour)
{
    myScreen.setPenSolid(true);
    myScreen.circle(STATUS_DOT_X, STATUS_DOT_Y, STATUS_DOT_R, colour);
    myScreen.setPenSolid(false);
}

// Statische Elemente: einmalig zeichnen (Titel, Labels, Plot-Rahmen/Kreuze)
static void draw_static_layout()
{
    myScreen.clear(blackColour);
    myScreen.setFontSize(0);

    myScreen.gText(X_LABEL, ROW_TITLE, "Five-Port Detector", whiteColour);
    myScreen.gText(X_LABEL, ROW_DC, "dc:", whiteColour);
    myScreen.gText(X_LABEL, ROW_RG, "rg:", whiteColour);
    myScreen.gText(X_LABEL, ROW_IG, "ig:", whiteColour);
    myScreen.gText(X_LABEL, ROW_AVG, "AVG:", whiteColour);

    myScreen.rectangle(PLOT_X0, PLOT_Y0, PLOT_X1, PLOT_Y1, whiteColour);
    // Referenzkreuze werden NICHT mehr hier gezeichnet: ihre Groesse codiert
    // die laufend geschaetzte Wahrscheinlichkeit je Klasse und muss daher
    // jeden Zyklus in update_display() neu gezeichnet werden.

    draw_status_dot(redColour);
    layoutReady = true;
    plotHasPrev = false;
}

// Dynamische Werte: nur diese Felder werden pro Zyklus aktualisiert
static void update_display()
{
    if (!layoutReady) draw_static_layout();

    myScreen.setFontSize(0);

    // Vor der ersten Kalibrierung sind dc/rg/ig nur Platzhalter (Nullen) -
    // gelb zeigt "noch nicht belastbar", gruen nach calibrate() "gueltig".
    uint16_t dataCol = calibrated ? greenColour : yellowColour;

    myScreen.gText(X_VALUES, ROW_DC,
        padNum(dc[0],2,5) + " " + padNum(dc[1],2,5) + " " + padNum(dc[2],2,5),
        dataCol);

    myScreen.gText(X_VALUES, ROW_RG,
        padNum(rg[0],1,5) + " " + padNum(rg[1],1,5) + " " + padNum(rg[2],1,5),
        dataCol);

    myScreen.gText(X_VALUES, ROW_IG,
        padNum(ig[0],1,5) + " " + padNum(ig[1],1,5) + " " + padNum(ig[2],1,5),
        dataCol);

    uint16_t Icol = (disp_Iavg > 0.25f && disp_Iavg < 0.75f) ? whiteColour : redColour;
    uint16_t Qcol = (disp_Qavg > 0.25f && disp_Qavg < 0.75f) ? whiteColour : redColour;

    myScreen.gText(X_AVG_I, ROW_AVG, "I=" + padNum(disp_Iavg, 2, 5), Icol);
    myScreen.gText(X_AVG_Q, ROW_AVG, "Q=" + padNum(disp_Qavg, 2, 5), Qcol);

    // ── Fehlerrate: Text + farbcodierter Balken ──
    float errAvg = (disp_errPct_I + disp_errPct_Q) / 2.0f;
    uint16_t errCol = (errAvg < 10.0f) ? greenColour : (errAvg < 50.0f) ? yellowColour : redColour;

    myScreen.gText(X_LABEL, ROW_ERR, "Err I=", errCol);
    myScreen.gText(X_ERR_I, ROW_ERR, padNumLeft(disp_errPct_I, 0, 3) + "%", errCol);
    myScreen.gText(X_ERR_I + 30, ROW_ERR, "Q=", errCol);
    myScreen.gText(X_ERR_Q, ROW_ERR, padNumLeft(disp_errPct_Q, 0, 3) + "%", errCol);

    myScreen.setPenSolid(true);
    myScreen.rectangle(0, BAR_Y0, 127, BAR_Y1, blackColour);
    uint16_t barFill = (uint16_t)(127 * (errAvg / 100.0f));
    if (barFill > 0) myScreen.rectangle(0, BAR_Y0, barFill, BAR_Y1, errCol);
    myScreen.setPenSolid(false);
    myScreen.rectangle(0, BAR_Y0, 127, BAR_Y1, whiteColour);

    // ── Status-Punkt oben rechts: gruen = laeuft & Fehlerrate <10% ──
    bool statusOk = calibrated && (errAvg < 10.0f);
    draw_status_dot(statusOk ? greenColour : redColour);

    // ── IQ-Konstellationsplot: nur Referenzmarker + Punkte, nicht der Rahmen ──

    // Referenzpunkte der 4 QPSK-Symbole: Farbe = Klasse, Kreisgroesse =
    // laufend geschaetzte Wahrscheinlichkeit dieser Klasse (siehe
    // update_class_prob). Vorher feste Box um jeden Punkt loeschen, da der
    // Kreis von Zyklus zu Zyklus schrumpfen/wachsen kann.
    myScreen.setPenSolid(true);
    for (uint8_t bi = 0; bi <= 1; bi++) {
        for (uint8_t bq = 0; bq <= 1; bq++) {
            uint8_t idx = bi * 2 + bq;
            uint16_t cx, cy;
            ref_point_xy(bi, bq, cx, cy);
            myScreen.rectangle(cx - PROB_R_MAX, cy - PROB_R_MAX, cx + PROB_R_MAX, cy + PROB_R_MAX, blackColour);
            uint16_t r = 1 + (uint16_t)(classProb[idx] * (PROB_R_MAX - 1));
            myScreen.circle(cx, cy, r, CLASS_COLOUR[idx]);
        }
    }
    myScreen.setPenSolid(false);
    for (uint8_t bi = 0; bi <= 1; bi++) {
        for (uint8_t bq = 0; bq <= 1; bq++) {
            uint16_t cx, cy;
            ref_point_xy(bi, bq, cx, cy);
            myScreen.line(cx - 2, cy, cx + 2, cy, whiteColour);
            myScreen.line(cx, cy - 2, cx, cy + 2, whiteColour);
        }
    }

    myScreen.setPenSolid(true);
    // Alte Punkte loeschen (nur die Pixel der letzten Position, kein Flaechen-Clear)
    if (plotHasPrev) {
        for (int n = 0; n < N_CAL; n++) {
            float ci = plotPrevI[n] < PLOT_LO ? PLOT_LO : (plotPrevI[n] > PLOT_HI ? PLOT_HI : plotPrevI[n]);
            float cq = plotPrevQ[n] < PLOT_LO ? PLOT_LO : (plotPrevQ[n] > PLOT_HI ? PLOT_HI : plotPrevQ[n]);
            uint16_t px = PLOT_X0 + (uint16_t)((PLOT_X1 - PLOT_X0) * ((ci - PLOT_LO) / (PLOT_HI - PLOT_LO)));
            uint16_t py = PLOT_Y1 - (uint16_t)((PLOT_Y1 - PLOT_Y0) * ((cq - PLOT_LO) / (PLOT_HI - PLOT_LO)));
            myScreen.rectangle(px - 1, py - 1, px + 1, py + 1, blackColour);
        }
    }

    // Gemessene Punkte: Farbe = erkannte Klasse (wie Referenzpunkt-Farbe)
    for (int n = 0; n < N_CAL; n++) {
        float ci = iq_hist_I[n] < PLOT_LO ? PLOT_LO : (iq_hist_I[n] > PLOT_HI ? PLOT_HI : iq_hist_I[n]);
        float cq = iq_hist_Q[n] < PLOT_LO ? PLOT_LO : (iq_hist_Q[n] > PLOT_HI ? PLOT_HI : iq_hist_Q[n]);
        uint16_t px = PLOT_X0 + (uint16_t)((PLOT_X1 - PLOT_X0) * ((ci - PLOT_LO) / (PLOT_HI - PLOT_LO)));
        uint16_t py = PLOT_Y1 - (uint16_t)((PLOT_Y1 - PLOT_Y0) * ((cq - PLOT_LO) / (PLOT_HI - PLOT_LO)));
        uint16_t dotCol = CLASS_COLOUR[iq_hist_class[n]];
        myScreen.rectangle(px - 1, py - 1, px + 1, py + 1, dotCol);

        plotPrevI[n] = iq_hist_I[n];
        plotPrevQ[n] = iq_hist_Q[n];
    }
    plotHasPrev = true;
    myScreen.setPenSolid(false);
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

    // GPIO-Ports für SPI und Screen-Steuerpins aktivieren (BoosterPack 2!)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);  // PQ0=SCK, PQ2=MOSI, PQ3=MISO (SSI3, BoosterPack 2)
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOQ));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);  // PP3=CS
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOP));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);  // PA7=RST
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);  // PK7=D/C
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOK));

#if PIN_TEST_MODE
    // Diagnose: CS/RST/DC einzeln langsam schalten, mit Multimeter an
    // BoosterPack-2-Pin D2.8 (CS), D2.4 (RST), C2.10 (DC) gegen GND nachmessbar.
    // Jeder Pin 1s HIGH (3.3V), dann 1s LOW (0V), nacheinander, endlos.
    pinMode(PP_3, OUTPUT);
    pinMode(PA_7, OUTPUT);
    pinMode(PK_7, OUTPUT);
    while (true) {
        Serial.println("CS (PP3) HIGH");
        digitalWrite(PP_3, HIGH); digitalWrite(PA_7, LOW); digitalWrite(PK_7, LOW);
        delay(1000);
        Serial.println("CS (PP3) LOW");
        digitalWrite(PP_3, LOW);
        delay(1000);

        Serial.println("RST (PA7) HIGH");
        digitalWrite(PA_7, HIGH);
        delay(1000);
        Serial.println("RST (PA7) LOW");
        digitalWrite(PA_7, LOW);
        delay(1000);

        Serial.println("DC (PK7) HIGH");
        digitalWrite(PK_7, HIGH);
        delay(1000);
        Serial.println("DC (PK7) LOW");
        digitalWrite(PK_7, LOW);
        delay(1000);
    }
#endif

    SPI.setModule(4);  // SSI3 auf PQ0/PQ2/PQ3 (BoosterPack 2, wo das EduBP MKII wirklich steckt)
    myScreen.begin();
    // Zeigt dc/rg/ig direkt als Platzhalter (0.00, gelb) an, noch vor der
    // ersten calibrate() - sonst waere der gelbe Zustand nie sichtbar, da
    // update_display() sonst erst nach einer bereits abgeschlossenen
    // Kalibrierung aufgerufen wird.
    update_display();

    // GPIO
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC));
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // ADC Pins PE1/PE2/PE3 als Analogeingang
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));
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
    Serial.println("err_bit: 0=richtig, 1=falsch erkannt");
    float I_avg = 0, Q_avg = 0;
    int err_count_I = 0, err_count_Q = 0;
    for(int n = 0; n < N_CAL; n++){
        while(!new_sample_ready);
        new_sample_ready = false;

        float v[3];
        read_detectors(v);
        float I, Q;
        demodulate(v, &I, &Q);
        I_avg += I;
        Q_avg += Q;

        // Fehler ggü. Entscheidungsgrenze 0.5: Abweichung vom gesendeten
        // Sollwert (0 oder 1), d.h. wie weit I/Q vom idealen Symbol abweichen
        float err_I = I - (float)I_sent;
        float err_Q = Q - (float)Q_sent;

        // Harte Entscheidung an der 0.5-Grenze: 0 = richtig, 1 = falsch erkannt
        uint8_t decided_I = (I > 0.5f) ? 1 : 0;
        uint8_t decided_Q = (Q > 0.5f) ? 1 : 0;
        uint8_t err_bit_I = (decided_I != I_sent) ? 1 : 0;
        uint8_t err_bit_Q = (decided_Q != Q_sent) ? 1 : 0;
        err_count_I += err_bit_I;
        err_count_Q += err_bit_Q;

        uint8_t classIdx = decided_I * 2 + decided_Q;
        update_class_prob(classIdx);

        iq_hist_I[n] = I;
        iq_hist_Q[n] = Q;
        iq_hist_class[n] = classIdx;

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
            Serial.print(Q,4);
            Serial.print(" | err: ");
            Serial.print(err_I,4); Serial.print("  ");
            Serial.print(err_Q,4);
            Serial.print(" | err_bit: ");
            Serial.print(err_bit_I); Serial.print("  ");
            Serial.println(err_bit_Q);
        }
    }

    float errPct_I = 100.0f * err_count_I / N_CAL;
    float errPct_Q = 100.0f * err_count_Q / N_CAL;
    disp_errPct_I = errPct_I;
    disp_errPct_Q = errPct_Q;
    Serial.print("Fehlerrate Burst: I="); Serial.print(errPct_I, 1);
    Serial.print("%  Q="); Serial.print(errPct_Q, 1);
    Serial.println("%");

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