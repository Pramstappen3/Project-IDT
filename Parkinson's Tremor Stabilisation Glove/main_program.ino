/*
 * ============================================================
 *  Parkinson's Tremor Stabilization Glove
 *  File     : main.cpp  (rename to TremorGlove.ino for Arduino IDE)
 *  Board    : ESP32C3 Dev Module
 *  IDE      : Arduino IDE  (install esp32 by Espressif, v2.x)
 *  Language : C++
 *
 *  Library required — install from Library Manager:
 *    (none) — LEDC is built into the ESP32 Arduino core.
 *             ESP32Servo is NOT used or needed any more.
 *
 *  Wiring:
 *    MPU6050 SDA  → GPIO8   (native hardware I2C on ESP32-C3)
 *    MPU6050 SCL  → GPIO9   (native hardware I2C on ESP32-C3)
 *    MPU6050 VCC  → 3.3V    (through AMS1117-3.3)
 *    MPU6050 GND  → GND
 *    MPU6050 AD0  → GND     (sets I2C address to 0x68)
 *
 *    GPIO1 → 220 Ω → NPN base (S8050 or 2N2222)
 *    NPN collector  → Vibration motor –
 *    Motor +        → 3.7V battery rail
 *    1N4007 diode   → across motor (cathode to motor +)
 *    NPN emitter    → GND
 *
 *  IMPORTANT CHANGES vs previous BLDC/ESC version:
 *    • ESCDriver.h   → replaced by VibrationDriver.h (LEDC PWM)
 *    • No ESP32Servo library needed
 *    • No ESC arming sequence — vibration motors need none
 *    • PID output is now PWM duty (0–255) not microseconds
 *    • PID gains re-tuned for PWM range (Kp=4, Ki=0.02, Kd=0.5)
 *    • Motor is fully OFF when hand is still OR no tremor
 *    • ESC_IDLE_US removed — replaced with VIB_OFF_DUTY=0
 *
 *  Motor behaviour:
 *    Hand still              → motor FULLY OFF  (duty=0)
 *    Hand moving, no tremor  → motor FULLY OFF  (save power)
 *    Hand moving + tremor    → motor ON (duty from PID, 50–220)
 *
 *  Tremor frequency measurement:
 *    Method  : zero-crossing detection on gyroMag
 *    Counts  : upward crossings of FREQ_CROSS_THRESH per second
 *    Output  : Serial Monitor, every 1 second
 *    Range   : Parkinson's tremor = 4–8 Hz
 *
 *  NOTE: GPIO6–11 are connected to flash on ESP32-C3.
 *        Never use GPIO6–11 for any peripheral.
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include "MPU6050Driver.h"    // → SensorData, thresholds
#include "VibrationDriver.h"  // → VIB_MIN_DUTY, VIB_MAX_DUTY, VIB_OFF_DUTY
#include "PIDController.h"

// ─────────────────────────────────────────────────────────
//  PINS
//  GPIO8 + GPIO9: native I2C on ESP32-C3 — no pull-ups needed
//  GPIO1: PWM output to NPN transistor base via 220 Ω
//  NEVER use GPIO6–11 (connected to ESP32-C3 internal flash)
// ─────────────────────────────────────────────────────────
static constexpr uint8_t PIN_SDA = 8;
static constexpr uint8_t PIN_SCL = 9;
static constexpr uint8_t PIN_VIB = 1;   // vibration motor PWM

// ─────────────────────────────────────────────────────────
//  TIMING
// ─────────────────────────────────────────────────────────
static constexpr uint32_t LOOP_MS           = 10;   // 100 Hz control loop
static constexpr uint32_t REST_TIMEOUT_MS   = 1500; // still for 1.5s → motor off
static constexpr uint32_t TREMOR_CONFIRM_MS = 150;  // tremor must persist 150ms
static constexpr uint32_t FREQ_WINDOW_MS    = 1000; // compute frequency every 1s

// ─────────────────────────────────────────────────────────
//  FREQUENCY MEASUREMENT
//  FREQ_CROSS_THRESH: gyroMag must cross this value upward
//  to count as one tremor cycle.
//  Set to ~half of TREMOR_GYRO_THRESH (15.0 dps) = 7.5 dps.
//  Adjust if readings are wrong:
//    Hz reads too high → raise this value slightly
//    Hz reads too low  → lower this value slightly
// ─────────────────────────────────────────────────────────
static constexpr float FREQ_CROSS_THRESH = 7.5f;  // dps

// ─────────────────────────────────────────────────────────
//  OBJECTS
// ─────────────────────────────────────────────────────────
static MPU6050Driver    mpu;
static VibrationDriver  vib;   // replaces ESCDriver
static PIDController    pid;
static SensorData       sensor;

// ─────────────────────────────────────────────────────────
//  MOTOR CONTROL STATE
// ─────────────────────────────────────────────────────────
static unsigned long lastLoopTime    = 0;
static unsigned long lastMotionTime  = 0;
static unsigned long tremorSince     = 0;
static bool          tremorConfirmed = false;

// ─────────────────────────────────────────────────────────
//  FREQUENCY MEASUREMENT STATE
// ─────────────────────────────────────────────────────────
static float         prevGyroMag     = 0.0f;
static uint32_t      crossingCount   = 0;
static unsigned long freqWindowStart = 0;
static float         tremorFreqHz    = 0.0f;

// ─────────────────────────────────────────────────────────
//  measureFrequency()
//  Call every loop with current gyroMag and timestamp.
//  Detects upward zero-crossings → computes Hz each second.
//
//  How it works:
//    gyroMag oscillates with each tremor cycle.
//    Every time it rises THROUGH FREQ_CROSS_THRESH we count
//    one full oscillation peak.
//    After 1000ms: Hz = crossings / elapsed seconds
//
//  Example: 5 crossings in 1.02s → tremorFreqHz = 4.9 Hz
// ─────────────────────────────────────────────────────────
static void measureFrequency(float gyroMag, unsigned long now) {
    // Upward crossing: was below threshold, now at or above
    if (prevGyroMag < FREQ_CROSS_THRESH && gyroMag >= FREQ_CROSS_THRESH) {
        crossingCount++;
    }
    prevGyroMag = gyroMag;

    const unsigned long elapsed = now - freqWindowStart;
    if (elapsed >= FREQ_WINDOW_MS) {
        tremorFreqHz    = (float)crossingCount / (elapsed / 1000.0f);
        crossingCount   = 0;
        freqWindowStart = now;
    }
}

// ─────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("=========================================");
    Serial.println("  Parkinson's Tremor Stabilization Glove");
    Serial.println("  ESP32-C3 | MPU6050 | Vibration Motor");
    Serial.println("  Motor ON  = tremor confirmed");
    Serial.println("  Motor OFF = still / no tremor");
    Serial.println("  Tremor range: 4–8 Hz (Parkinson's)");
    Serial.println("=========================================");

    // I2C on native hardware pins — no external pull-ups on GPIO8/9
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);  // 400 kHz fast mode
    Serial.println("[I2C] 400kHz on GPIO8(SDA) GPIO9(SCL)");

    // ── I2C scanner ──────────────────────────────────────
    // MPU6050 must appear at 0x68 (AD0 → GND).
    // Nothing found = wiring problem, fix before continuing.
    Serial.println("[I2C] Scanning...");
    uint8_t devFound = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("[I2C] Device @ 0x");
            if (addr < 16) Serial.print('0');
            Serial.print(addr, HEX);
            if      (addr == 0x68) Serial.print("  ← MPU6050 (AD0=GND)  ✓");
            else if (addr == 0x69) Serial.print("  ← MPU6050 (AD0=3.3V) — move AD0 to GND");
            Serial.println();
            devFound++;
        }
    }
    if (devFound == 0) {
        Serial.println("[I2C] Nothing found. Check VCC→3.3V GND SDA→8 SCL→9 AD0→GND");
    }
    Serial.println("--------------------------------------------------");

    // ── MPU6050 init ─────────────────────────────────────
    if (!mpu.begin()) {
        Serial.println("[FATAL] MPU6050 not found. Fix wiring. Halting.");
        while (true) { delay(500); }
    }

    // ── Vibration motor init ─────────────────────────────
    // No arming needed. LEDC attach and done.
    vib.begin(PIN_VIB);

    // ── PID init ─────────────────────────────────────────
    // Kp=4.0  main strength     — tuned for 0–255 duty range
    // Ki=0.02 integral (small)  — prevents drift
    // Kd=0.5  derivative        — smooths onset
    pid.init(4.0f, 0.02f, 0.5f);

    Serial.println("[SYSTEM] Running. Watching for hand movement...");
    Serial.println("--------------------------------------------------");
    Serial.println("Hand\t\tTremor\t\tFreq(Hz)\tDuty(0-255)");

    const unsigned long now = millis();
    lastLoopTime    = now;
    lastMotionTime  = now;
    freqWindowStart = now;
}

// ─────────────────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────────────────
void loop() {
    const unsigned long now = millis();
    if (now - lastLoopTime < LOOP_MS) return;

    const float dt = (now - lastLoopTime) / 1000.0f;
    lastLoopTime = now;

    // ── 1. Read sensor ───────────────────────────────────
    mpu.read(sensor);

    // ── 2. Measure tremor frequency (100 Hz rate) ────────
    measureFrequency(sensor.gyroMag, now);

    // ── 3. Hand motion debounce ──────────────────────────
    // Motor only shuts off after REST_TIMEOUT_MS of stillness.
    if (sensor.inMotion) {
        lastMotionTime = now;
    }
    const bool handActive = (now - lastMotionTime <= REST_TIMEOUT_MS);

    // ── 4. Tremor confirmation debounce ──────────────────
    // Require continuous tremor for TREMOR_CONFIRM_MS to avoid
    // triggering on single spikes / bumps.
    if (sensor.tremor) {
        if (tremorSince == 0) tremorSince = now;
        tremorConfirmed = (now - tremorSince >= TREMOR_CONFIRM_MS);
    } else {
        tremorSince     = 0;
        tremorConfirmed = false;
    }

    // ── 5. PID → vibration duty ──────────────────────────
    int motorDuty = VIB_OFF_DUTY;

    if (handActive && tremorConfirmed) {
        // PID error = 0 - gyroMag → always negative
        // compute() returns a clamped value between outputMin(0)
        // and outputMax(220) — see PIDController.h
        // We invert the sign: more tremor → larger positive duty
        const float pidOut   = pid.compute(sensor.gyroMag, dt);
        // pidOut is negative (setpoint 0 - positive gyroMag)
        // multiply by -1 to get a positive duty value
        const float rawDuty  = -pidOut;
        motorDuty = constrain(static_cast<int>(rawDuty),
                              VIB_MIN_DUTY, VIB_MAX_DUTY);
    } else {
        pid.reset();  // clear integral — prevents windup while idle
    }

    // ── 6. Drive vibration motor ─────────────────────────
    vib.update(handActive, tremorConfirmed, motorDuty);

    // ── 7. Serial output (once per second) ───────────────
    static unsigned long lastPrintTime = 0;
    if (now - lastPrintTime >= FREQ_WINDOW_MS) {
        lastPrintTime = now;

        Serial.print(handActive      ? "MOVING  " : "still   ");
        Serial.print('\t');
        Serial.print(tremorConfirmed ? "TREMOR  " : "none    ");
        Serial.print('\t');

        if (tremorConfirmed) {
            Serial.print(tremorFreqHz, 1);
            Serial.print(" Hz");
            if      (tremorFreqHz >= 4.0f && tremorFreqHz <= 8.0f)
                Serial.print(" [Parkinson range]");
            else if (tremorFreqHz > 8.0f)
                Serial.print(" [above range — essential tremor?]");
            else if (tremorFreqHz >= 2.0f)
                Serial.print(" [low — shake more rhythmically]");
            else
                Serial.print(" [measuring...]");
        } else {
            Serial.print("--      ");
        }
        Serial.print('\t');

        // Print duty cycle and approximate % for readability
        Serial.print(vib.currentDuty());
        Serial.print(" (");
        Serial.print(vib.currentDuty() * 100 / 255);
        Serial.println("%)");
    }
}
