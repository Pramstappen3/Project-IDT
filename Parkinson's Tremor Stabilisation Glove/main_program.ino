/*
 * ============================================================
 *  Parkinson's Tremor Stabilization Glove — 5-Motor Version
 *  File     : main.cpp  (rename to TremorGlove.ino for Arduino IDE)
 *  Board    : ESP32C3 Dev Module
 *  IDE      : Arduino IDE (esp32 by Espressif v2.x)
 *
 *  NO external libraries needed — LEDC is built into the core.
 *
 *  WIRING SUMMARY:
 *    MPU6050 SDA  → GPIO8   (hardware I2C)
 *    MPU6050 SCL  → GPIO9   (hardware I2C)
 *    MPU6050 VCC  → 3.3V
 *    MPU6050 GND  → GND
 *    MPU6050 AD0  → GND     (I2C address 0x68)
 *
 *    5× vibration motors (each identical):
 *      Motor 0 (index)  : GPIO1 → 200Ω → NPN base
 *      Motor 1 (middle) : GPIO2 → 200Ω → NPN base
 *      Motor 2 (ring)   : GPIO3 → 200Ω → NPN base
 *      Motor 3 (pinky)  : GPIO4 → 200Ω → NPN base
 *      Motor 4 (thumb)  : GPIO5 → 200Ω → NPN base
 *      NPN collector    → Motor –
 *      Motor +          → 3.7V rail
 *      1N4007           → across motor (cathode to +)
 *      NPN emitter      → GND
 *
 *  AUTO ON/OFF BEHAVIOUR:
 *    Hand still              → ALL motors OFF
 *    Hand moving, no tremor  → ALL motors OFF
 *    Tremor confirmed 150ms  → motors ON (PID duty, zone-based)
 *    Tremor stops            → motors ramp off immediately
 *
 *  MOTOR ZONES (based on tremor intensity / PID duty):
 *    Mild   (50–100)   → 1 motor
 *    Medium (101–160)  → 2 motors
 *    Strong (161–200)  → 3 motors
 *    Severe (201–220)  → 4 motors
 *    Max    (>220)     → all 5 motors
 *
 *  NEVER use GPIO6–11 (connected to flash on ESP32-C3).
 * ============================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include "MPU6050Driver.h"    // SensorData, thresholds
#include "VibrationDriver.h"  // 5-motor LEDC driver
#include "PIDController.h"

// ─────────────────────────────────────────────────────────
//  I2C PINS (hardware I2C on ESP32-C3 — no pull-ups needed)
// ─────────────────────────────────────────────────────────
static constexpr uint8_t PIN_SDA = 8;
static constexpr uint8_t PIN_SCL = 9;

// ─────────────────────────────────────────────────────────
//  TIMING
// ─────────────────────────────────────────────────────────
static constexpr uint32_t LOOP_MS           = 10;    // 100 Hz control loop
static constexpr uint32_t REST_TIMEOUT_MS   = 1500;  // still for 1.5s → off
static constexpr uint32_t TREMOR_CONFIRM_MS = 150;   // tremor must last 150ms
static constexpr uint32_t FREQ_WINDOW_MS    = 1000;  // frequency computed every 1s
static constexpr uint32_t PRINT_INTERVAL_MS = 1000;  // serial print every 1s

// ─────────────────────────────────────────────────────────
//  FREQUENCY MEASUREMENT
//  gyroMag upward crossings of this threshold = one cycle
// ─────────────────────────────────────────────────────────
static constexpr float FREQ_CROSS_THRESH = 7.5f;  // dps (~half of TREMOR_GYRO_THRESH)

// ─────────────────────────────────────────────────────────
//  OBJECTS
// ─────────────────────────────────────────────────────────
static MPU6050Driver   mpu;
static VibrationDriver vib;   // 5 motors, GPIO1–5
static PIDController   pid;
static SensorData      sensor;

// ─────────────────────────────────────────────────────────
//  STATE
// ─────────────────────────────────────────────────────────
static unsigned long lastLoopTime    = 0;
static unsigned long lastMotionTime  = 0;
static unsigned long tremorSince     = 0;
static bool          tremorConfirmed = false;

// Frequency measurement state
static float         prevGyroMag     = 0.0f;
static uint32_t      crossingCount   = 0;
static unsigned long freqWindowStart = 0;
static float         tremorFreqHz    = 0.0f;

// ─────────────────────────────────────────────────────────
//  measureFrequency()
//  Counts upward crossings of FREQ_CROSS_THRESH per second.
//  Each crossing = one tremor oscillation peak.
//  Parkinson's range: 4–8 Hz.
// ─────────────────────────────────────────────────────────
static void measureFrequency(float gyroMag, unsigned long now) {
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

    Serial.println("==============================================");
    Serial.println("  Tremor Glove — 5 Motor Version");
    Serial.println("  ESP32-C3 | MPU6050 | 5× ERM Motors");
    Serial.println("  Motors: GPIO1, GPIO2, GPIO3, GPIO4, GPIO5");
    Serial.println("  AUTO ON  when tremor confirmed (150 ms)");
    Serial.println("  AUTO OFF when hand still (1.5 s timeout)");
    Serial.println("==============================================");

    // I2C on hardware pins — no external pull-ups needed on GPIO8/9
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);
    Serial.println("[I2C] 400kHz — GPIO8 SDA / GPIO9 SCL");

    // I2C scanner
    Serial.println("[I2C] Scanning...");
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print("[I2C] Device @ 0x");
            if (addr < 16) Serial.print('0');
            Serial.print(addr, HEX);
            if      (addr == 0x68) Serial.print("  ← MPU6050 (AD0=GND)  ✓");
            else if (addr == 0x69) Serial.print("  ← MPU6050 (AD0=3.3V) — move AD0 to GND");
            Serial.println();
            found++;
        }
    }
    if (found == 0) {
        Serial.println("[I2C] Nothing found. Check: SDA→8  SCL→9  VCC→3.3V  AD0→GND");
    }
    Serial.println("--------------------------------------------");

    // MPU6050
    if (!mpu.begin()) {
        Serial.println("[FATAL] MPU6050 not found. Halting.");
        vib.allOff();
        while (true) { delay(500); }
    }

    // 5 vibration motors — no arming needed
    vib.begin();

    // PID — tuned for 0–255 PWM duty range
    // Kp=4.0  main response
    // Ki=0.02 slow drift correction
    // Kd=0.5  smooths sudden onset
    pid.init(4.0f, 0.02f, 0.5f);

    Serial.println("[SYSTEM] Running. Watching for tremor...");
    Serial.println("--------------------------------------------");
    Serial.println("Hand\t\tTremor\t\tFreq\t\tDuty\tMotors ON");

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

    // ── 1. Read MPU6050 ───────────────────────────────────
    mpu.read(sensor);

    // ── 2. Measure tremor frequency ───────────────────────
    measureFrequency(sensor.gyroMag, now);

    // ── 3. Hand motion debounce ───────────────────────────
    //  Reset timer any time the hand is actively moving.
    //  handActive goes false only after 1.5 s of stillness.
    if (sensor.inMotion) {
        lastMotionTime = now;
    }
    const bool handActive = (now - lastMotionTime <= REST_TIMEOUT_MS);

    // ── 4. Tremor confirmation debounce ───────────────────
    //  Tremor must persist for TREMOR_CONFIRM_MS (150ms) before
    //  motors are allowed to turn on. Prevents false triggers
    //  from bumps or single spikes.
    if (sensor.tremor) {
        if (tremorSince == 0) tremorSince = now;
        tremorConfirmed = (now - tremorSince >= TREMOR_CONFIRM_MS);
    } else {
        tremorSince     = 0;
        tremorConfirmed = false;
        // tremorConfirmed = false immediately stops motors via vib.update()
    }

    // ── 5. PID → compute motor duty ──────────────────────
    //  Only computed when tremor is confirmed.
    //  PID setpoint = 0 (want gyroMag = 0).
    //  PID output is negative (error = 0 - gyroMag).
    //  We negate it to get a positive duty value.
    int motorDuty = VIB_OFF_DUTY;

    if (handActive && tremorConfirmed) {
        const float pidOut  = pid.compute(sensor.gyroMag, dt);
        motorDuty = constrain(static_cast<int>(-pidOut),
                              VIB_MIN_DUTY, VIB_MAX_DUTY);
    } else {
        pid.reset();  // clear integral — prevents windup while idle
    }

    // ── 6. Drive motors (AUTO ON/OFF inside vib.update) ──
    //  vib.update() handles everything:
    //    !handActive OR !tremorConfirmed → ramps all 5 motors to 0
    //    handActive AND tremorConfirmed  → activates motors by zone
    vib.update(handActive, tremorConfirmed, motorDuty);

    // ── 7. Serial output (once per second) ───────────────
    static unsigned long lastPrintTime = 0;
    if (now - lastPrintTime >= PRINT_INTERVAL_MS) {
        lastPrintTime = now;

        Serial.print(handActive      ? "MOVING  " : "still   ");
        Serial.print('\t');
        Serial.print(tremorConfirmed ? "TREMOR  " : "none    ");
        Serial.print('\t');

        if (tremorConfirmed) {
            Serial.print(tremorFreqHz, 1);
            Serial.print(" Hz");
            if      (tremorFreqHz >= 4.0f && tremorFreqHz <= 8.0f)
                Serial.print(" [Parkinson]");
            else if (tremorFreqHz > 8.0f)
                Serial.print(" [essential?]");
            else
                Serial.print(" [measuring]");
        } else {
            Serial.print("--      \t");
        }
        Serial.print('\t');
        Serial.print(motorDuty);
        Serial.print('\t');
        Serial.print(vib.activeCount());
        Serial.print("/5 motors");
        Serial.println();
    }
}
