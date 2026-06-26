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
#include "constants.h"

// ── Portables Serial.printf-Ersatz (Energia hat kein Serial.printf) ───────────
#ifdef BOARD_TM4C129EXL
static char _sp_buf[200];
#  define serial_printf(fmt, ...) \
       do { snprintf(_sp_buf, sizeof(_sp_buf), fmt, ##__VA_ARGS__); \
            Serial.print(_sp_buf); } while(0)
#else
#  define serial_printf(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#endif

// ── Pin-Definitionen ──────────────────────────────────────────────────────────
#ifdef BOARD_TM4C129EXL
// EK-TM4C129EXL – BoosterPack 1 (Energia-Pinnummern)
#  define IQ_I_PIN     PC_4  // I-Bit → X8_05
#  define IQ_Q_PIN     PC_5  // Q-Bit → X8_07
#  define ADC_V1_PIN   A0    // PE3  – J1 Pin 2
#  define ADC_V2_PIN   A1    // PE2  – J1 Pin 6
#  define ADC_V3_PIN   A2    // PE1  – J1 Pin 5
#  define VREF_BOARD   3.3f
#  define TRIGGER_PIN  PUSH1  // SW1 auf dem LaunchPad (PJ0, active-low)
#else
// STM32F411RE Nucleo-64
#  define IQ_I_PIN     PA6
#  define IQ_Q_PIN     PA7
#  define ADC_V1_PIN   PA0
#  define ADC_V2_PIN   PA1
#  define ADC_V3_PIN   PB0
#  define VREF_BOARD   3.3f
#  define TRIGGER_PIN  PC13  // Nucleo User-Button (active-low)
#endif

static constexpr int N_RESULT = 20;  // Messungen nach Kalibrierung

// ── Kalibrierungsparameter ────────────────────────────────────────────────────
static constexpr int   N_CAL      = 10;    // Anzahl Trainingssymbole
static constexpr int   N_AVG      = 1;     // ADC-Mittelwertbildung pro Messung
static constexpr int   T_SETTLE   = 1;     // Einschwingzeit [ms] nach Symbolwechsel (1kHz)
static constexpr float VREF       = VREF_BOARD;
static constexpr float ADC_MAXVAL = 4095.0f;

// ── QPSK-Trainingssequenz (bekannt, ±1) ──────────────────────────────────────
// Je 16 Wiederholungen aller 4 Quadranten → gute Überbestimmung
static const int8_t I_TRAIN[N_CAL] = {
     1, -1,-1, 1, 1, 1,-1,-1, 1, 1,
};
static const int8_t Q_TRAIN[N_CAL] = {
     1, -1, 1, 1,-1,-1,-1,-1, 1, 1,
};

// ── Kalibrierungsergebnis ─────────────────────────────────────────────────────
// I(t) = rg[0]·ṽ_o1 + rg[1]·ṽ_o2 + rg[2]·ṽ_o3  (Gl. 19)
// Q(t) = ig[0]·ṽ_o1 + ig[1]·ṽ_o2 + ig[2]·ṽ_o3  (Gl. 20)
static float rg[3], ig[3], dc[3];
static bool  calibrated = false;


// ─────────────────────────────────────────────────────────────────────────────
// IQ-Generator Interface
// I-Bit → PC4 (IQ_I_PIN), Q-Bit → PC5 (IQ_Q_PIN)
// HIGH = +1, LOW = −1
// ─────────────────────────────────────────────────────────────────────────────
static void iqgen_send(int8_t I, int8_t Q)
{
    if (I < 0 && Q < 0) Q = 1;  // mindestens eines immer HIGH
    digitalWrite(IQ_I_PIN, I > 0 ? HIGH : LOW);
    digitalWrite(IQ_Q_PIN, Q > 0 ? HIGH : LOW);
    delay(T_SETTLE);
}

static void iqgen_send_actual(int8_t &I, int8_t &Q)
{
    if (I < 0 && Q < 0) Q = 1;
    iqgen_send(I, Q);
}


// ─────────────────────────────────────────────────────────────────────────────
// ADC-Hilfsfunktionen
// ─────────────────────────────────────────────────────────────────────────────
static float adc_voltage(uint32_t pin)
{
    uint32_t sum = 0;
    for (int i = 0; i < N_AVG; i++) sum += analogRead(pin);
    return (sum * VREF) / (ADC_MAXVAL * N_AVG);
}

static void read_detectors(float v[3])
{
    v[0] = adc_voltage(ADC_V1_PIN);
    v[1] = adc_voltage(ADC_V2_PIN);
    v[2] = adc_voltage(ADC_V3_PIN);
}


// ─────────────────────────────────────────────────────────────────────────────
// 3×3 Matrixinversion (Cramer-Regel)
// ─────────────────────────────────────────────────────────────────────────────
static bool mat3_inv(const float M[3][3], float R[3][3])
{
    float det =
        M[0][0] * (M[1][1]*M[2][2] - M[1][2]*M[2][1]) -
        M[0][1] * (M[1][0]*M[2][2] - M[1][2]*M[2][0]) +
        M[0][2] * (M[1][0]*M[2][1] - M[1][1]*M[2][0]);
    if (fabsf(det) < 1e-10f) return false;

    const float d = 1.0f / det;
    R[0][0] =  (M[1][1]*M[2][2] - M[1][2]*M[2][1]) * d;
    R[0][1] = -(M[0][1]*M[2][2] - M[0][2]*M[2][1]) * d;
    R[0][2] =  (M[0][1]*M[1][2] - M[0][2]*M[1][1]) * d;
    R[1][0] = -(M[1][0]*M[2][2] - M[1][2]*M[2][0]) * d;
    R[1][1] =  (M[0][0]*M[2][2] - M[0][2]*M[2][0]) * d;
    R[1][2] = -(M[0][0]*M[1][2] - M[0][2]*M[1][0]) * d;
    R[2][0] =  (M[1][0]*M[2][1] - M[1][1]*M[2][0]) * d;
    R[2][1] = -(M[0][0]*M[2][1] - M[0][1]*M[2][0]) * d;
    R[2][2] =  (M[0][0]*M[1][1] - M[0][1]*M[1][0]) * d;
    return true;
}


// ─────────────────────────────────────────────────────────────────────────────
// Kleinste-Quadrate-Lösung:  coeffs = (Aᵀ·A)⁻¹ · Aᵀ · b
// A: [N×3],  b: [N],  coeffs: [3]  (Gl. 23/24)
// ─────────────────────────────────────────────────────────────────────────────
static bool lstsq3(const float A[][3], const float b[], int N, float coeffs[3])
{
    float ATA[3][3] = {};
    float ATb[3]    = {};
    for (int n = 0; n < N; n++) {
        for (int i = 0; i < 3; i++) {
            ATb[i] += A[n][i] * b[n];
            for (int j = 0; j < 3; j++)
                ATA[i][j] += A[n][i] * A[n][j];
        }
    }
    float ATA_inv[3][3];
    if (!mat3_inv(ATA, ATA_inv)) return false;
    for (int i = 0; i < 3; i++) {
        coeffs[i] = 0;
        for (int j = 0; j < 3; j++)
            coeffs[i] += ATA_inv[i][j] * ATb[j];
    }
    return true;
}


// ─────────────────────────────────────────────────────────────────────────────
// Kalibrierungsroutine
// ─────────────────────────────────────────────────────────────────────────────
static bool calibrate()
{
    Serial.println("[CAL] Starte Kalibrierung...");

    // Statisch um Stack-Overflow zu vermeiden (~2 KB)
    static float V[N_CAL][3];
    static float A[N_CAL][3];
    static float Ivec[N_CAL], Qvec[N_CAL];
    static int8_t I_sent[N_CAL], Q_sent[N_CAL];

    // Schritt 1: Trainingssymbole senden (mit Constraint) und ADCs lesen
    for (int n = 0; n < N_CAL; n++) {
        I_sent[n] = I_TRAIN[n];
        Q_sent[n] = Q_TRAIN[n];
        iqgen_send_actual(I_sent[n], Q_sent[n]);
        read_detectors(V[n]);
    }

    // Schritt 2: DC-Offset bestimmen und entfernen (Gl. 15)
    dc[0] = dc[1] = dc[2] = 0.0f;
    for (int n = 0; n < N_CAL; n++)
        for (int k = 0; k < 3; k++)
            dc[k] += V[n][k];
    for (int k = 0; k < 3; k++) dc[k] /= N_CAL;
    if (SERIAL_SHOW_CALIBRATION_RESULTS)
        serial_printf("[CAL] DC-Offset: [%.3f %.3f %.3f] V\n", dc[0], dc[1], dc[2]);

    // Matrix A aus DC-bereinigten Spannungen aufbauen
    for (int n = 0; n < N_CAL; n++) {
        for (int k = 0; k < 3; k++)
            A[n][k] = V[n][k] - dc[k];
        Ivec[n] = (float)I_sent[n];
        Qvec[n] = (float)Q_sent[n];
    }

    // Schritt 3: Kleinste Quadrate → rg[], ig[]  (Gl. 23/24)
    if (!lstsq3(A, Ivec, N_CAL, rg)) {
        Serial.println("[CAL] FEHLER: Singulaere Matrix (I-Kanal)");
        return false;
    }
    if (!lstsq3(A, Qvec, N_CAL, ig)) {
        Serial.println("[CAL] FEHLER: Singulaere Matrix (Q-Kanal)");
        return false;
    }

    if (SERIAL_SHOW_CALIBRATION_RESULTS) {
        serial_printf("[CAL] rg = [%+.4f, %+.4f, %+.4f]\n", rg[0], rg[1], rg[2]);
        serial_printf("[CAL] ig = [%+.4f, %+.4f, %+.4f]\n", ig[0], ig[1], ig[2]);
        Serial.println("[CAL] Kalibrierung erfolgreich.");
    }
    return true;
}


// ─────────────────────────────────────────────────────────────────────────────
// I/Q-Demodulation (Gl. 19 / 20)
// ─────────────────────────────────────────────────────────────────────────────
static void demodulate(const float v[3], float *I_out, float *Q_out)
{
    *I_out = *Q_out = 0.0f;
    for (int k = 0; k < 3; k++) {
        const float vtilde = v[k] - dc[k];
        *I_out += rg[k] * vtilde;
        *Q_out += ig[k] * vtilde;
    }
}


// ─────────────────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(921600);
    delay(500);
#ifdef BOARD_TM4C129EXL
    Serial.println("=== Five-Port Detektor EK-TM4C129EXL ===");
#else
    Serial.println("=== Five-Port Detektor STM32F411RE ===");
#endif
    analogReadResolution(12);

    pinMode(TRIGGER_PIN, INPUT_PULLUP);
    pinMode(IQ_I_PIN, OUTPUT);
    pinMode(IQ_Q_PIN, OUTPUT);
    digitalWrite(IQ_I_PIN, LOW);
    digitalWrite(IQ_Q_PIN, LOW);


#if CALIBRATION_MODE
    Serial.println("Druecke SW1 oder sende 's' per Serial.");
#endif
}

void loop()
{
#if CALIBRATION_MODE
    {
        bool btn = (digitalRead(TRIGGER_PIN) == LOW);
        bool ser = (Serial.available() && Serial.read() == 's');
        if (!btn && !ser) return;
        if (btn) { delay(50); if (digitalRead(TRIGGER_PIN) != LOW) return; }
    }
    while (digitalRead(TRIGGER_PIN) == LOW) {}  // warte auf Loslassen
    delay(50);
    Serial.println("[START] Starte...");

    if (ENABLE_ADCS) {
        calibrated = calibrate();
        if (!calibrated) {
            Serial.println("[ERR] Kalibrierung fehlgeschlagen.");
            while (digitalRead(TRIGGER_PIN) == LOW) {}
            return;
        }
    }

    Serial.println("--- Start (SW1=Stop) ---");
    bool running = true;
    uint32_t t_print = 0;
    while (running) {
        // Komplette Sequenz mit 1kHz senden
        uint32_t t_sym = millis();
        for (int sym = 0; sym < N_CAL && running; sym++) {
            while (millis() < t_sym) {}   // 1kHz-Takt
            t_sym++;

            int8_t I_s = I_TRAIN[sym], Q_s = Q_TRAIN[sym];
            iqgen_send_actual(I_s, Q_s);

            if (ENABLE_ADCS) {
                float v[3];
                read_detectors(v);
                float I_dem, Q_dem;
                demodulate(v, &I_dem, &Q_dem);
                uint32_t now = millis();
                if (SERIAL_SHOW_IQ && now >= t_print) {
                    serial_printf("%+.4f,%+.4f\n", I_dem, Q_dem);
                    t_print = now + 500;
                }
                if (SERIAL_SHOW_ADC && now >= t_print) {
                    serial_printf("%.4f,%.4f,%.4f\n", v[0], v[1], v[2]);
                    t_print = now + 500;
                }
            }

            if (digitalRead(TRIGGER_PIN) == LOW) {
                delay(50);
                if (digitalRead(TRIGGER_PIN) == LOW) {
                    Serial.println("--- Stop ---");
                    while (digitalRead(TRIGGER_PIN) == LOW) {}
                    running = false;
                }
            }
        }

        if (!running) break;

        // Pause: beide LOW für 100ms (non-blocking)
        digitalWrite(IQ_I_PIN, LOW);
        digitalWrite(IQ_Q_PIN, LOW);
        uint32_t t_pause = millis();
        while (millis() - t_pause < 100) {
            if (digitalRead(TRIGGER_PIN) == LOW) {
                delay(50);
                if (digitalRead(TRIGGER_PIN) == LOW) {
                    Serial.println("--- Stop ---");
                    while (digitalRead(TRIGGER_PIN) == LOW) {}
                    running = false;
                    break;
                }
            }
        }
    }

#else
    float v[3];
    read_detectors(v);

    static uint32_t t_send = 0;
    uint32_t now = millis();
    if (now >= t_send) {
        if (SERIAL_SHOW_ADC)
            serial_printf("%.4f,%.4f,%.4f\n", v[0], v[1], v[2]);
        t_send = now + 500;
    }
#endif
}
