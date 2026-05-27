/*test
 * ============================================================
 *  Parkinson's Tremor Stabilization Glove
 *  File     : main.cpp
 *  Board    : ESP32 C3 Dev Module (Arduino IDE)
 *  Language : C++
 *
 *  Wiring:
 *    MPU6050 SDA  → GPIO8   (default hardware I2C — no external pull-up needed)
 *    MPU6050 SCL  → GPIO9   (default hardware I2C — no external pull-up needed)
 *    MPU6050 VCC  → 3.3V    (through AMS1117-3.3)
 *    MPU6050 GND  → GND
 *    MPU6050 AD0  → GND     (sets I2C address to 0x68)
 *    ESC signal   → GPIO1   (through 200Ω resistor)
 *    ESC BAT+     → battery+ (through diode + capacitor)
 *    ESC BAT-     → GND
 *    BLDC phase A → ESC wire A
 *    BLDC phase B → ESC wire B
 *    BLDC phase C → ESC wire C
 *
 *  NOTE: GPIO8 and GPIO9 are the native hardware I2C pins on
 *  ESP32-C3. Do NOT add external pull-up resistors — it will
 *  conflict and cause I2C to fail.
 *
 *  Motor behavior (fully automatic, no display, no LED):
 *    Hand still              → motor STOPPED
 *    Hand moving, no tremor  → motor IDLE spin (ready)
 *    Hand moving + tremor    → motor ACTIVE stabilization
 *
 *  Tremor frequency measurement:
 *    Method : zero-crossing detection on gyroMag signal
 *    How    : counts how many times gyroMag rises above a
 *             mid-threshold per second. Each rise = one cycle.
 *             frequency (Hz) = crossings / elapsed time (s)
 *    Output : printed on Serial Monitor every 1 second
 *    Range  : Parkinson's tremor = 4–8 Hz
 *
 *  ESC constants (ESC_IDLE_US, MOTOR_MIN_US, MOTOR_MAX_US)
 *  are defined in ESCDriver.h and available here via #include.
 *
 *  Library required (install from Library Manager):
 *    ESP32Servo by Kevin Harrington
 * ============================================================
 */

#include "Arduino.h"
#include <Wire.h>
#include "MPU6050Driver.h"   // also brings: SensorData, thresholds
#include "ESCDriver.h"       // also brings: ESC_IDLE_US, MOTOR_MIN_US, MOTOR_MAX_US
#include "PIDController.h"

// ─────────────────────────────────────────────────────────
//  PINS
//  GPIO8 + GPIO9 are native hardware I2C on ESP32-C3.
//  GPIO6–11 are connected to flash — never use those.
// ─────────────────────────────────────────────────────────
static constexpr uint8_t PIN_SDA = 8;
static constexpr uint8_t PIN_SCL = 9;
static constexpr uint8_t PIN_ESC = 1;

// ─────────────────────────────────────────────────────────
//  TIMING
// ─────────────────────────────────────────────────────────
static constexpr uint32_t LOOP_MS           = 10;   // 100 Hz control loop
static constexpr uint32_t REST_TIMEOUT_MS   = 1500; // still 1.5s → motor off
static constexpr uint32_t TREMOR_CONFIRM_MS = 150;  // tremor 150ms → motor on
static constexpr uint32_t FREQ_WINDOW_MS    = 1000; // measure frequency every 1s

// ─────────────────────────────────────────────────────────
//  FREQUENCY MEASUREMENT CONSTANTS
//  Zero-crossing threshold: gyroMag must cross THIS value
//  to count as one tremor cycle.
//  Set to roughly half of TREMOR_GYRO_THRESH (15.0 dps).
//  Adjust if frequency reads incorrectly:
//    Too high (>10Hz) → raise FREQ_CROSS_THRESH slightly
//    Too low  (<2Hz)  → lower FREQ_CROSS_THRESH slightly
// ─────────────────────────────────────────────────────────
static constexpr float FREQ_CROSS_THRESH = 8.0f;  // dps — zero-crossing level

// ─────────────────────────────────────────────────────────
//  OBJECTS
// ─────────────────────────────────────────────────────────
static MPU6050Driver mpu;
static ESCDriver     esc;
static PIDController pid;
static SensorData    sensor;

// ─────────────────────────────────────────────────────────
//  STATE VARIABLES — motor control
// ─────────────────────────────────────────────────────────
static unsigned long lastLoopTime    = 0;
static unsigned long lastMotionTime  = 0;
static unsigned long tremorSince     = 0;
static bool          tremorConfirmed = false;

// ─────────────────────────────────────────────────────────
//  STATE VARIABLES — frequency measurement
//
//  How zero-crossing works:
//    gyroMag oscillates up and down with each tremor cycle.
//    Every time it crosses FREQ_CROSS_THRESH going UPWARD,
//    we count one full cycle.
//    After 1 second we divide crossings by elapsed time → Hz.
//
//    Example:
//      5 upward crossings in 1.0s = 5.0 Hz tremor ← Parkinson's range
// ─────────────────────────────────────────────────────────
static float         prevGyroMag      = 0.0f;  // gyroMag from last loop
static uint32_t      crossingCount    = 0;     // upward crossings this window
static unsigned long freqWindowStart  = 0;     // when current 1s window started
static float         tremorFreqHz     = 0.0f;  // last measured frequency

// ─────────────────────────────────────────────────────────
//  FREQUENCY MEASUREMENT FUNCTION
//  Call every loop with current gyroMag and timestamp.
//  Detects upward zero-crossings and computes Hz every 1s.
// ─────────────────────────────────────────────────────────
static void measureFrequency(float gyroMag, unsigned long now) {
    // Detect upward crossing:
    //   previous sample was BELOW threshold
    //   current  sample is  ABOVE threshold
    //   → one complete oscillation peak detected
    if (prevGyroMag < FREQ_CROSS_THRESH && gyroMag >= FREQ_CROSS_THRESH) {
        crossingCount++;
    }
    prevGyroMag = gyroMag;

    // Every FREQ_WINDOW_MS (1 second) compute frequency
    const unsigned long elapsed = now - freqWindowStart;
    if (elapsed >= FREQ_WINDOW_MS) {
        // frequency = number of upward crossings per second
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
    Serial.println("  Parkinson's Tremor Glove");
    Serial.println("  ESP32 C3 | MPU6050 | BLDC | C++");
    Serial.println("  Motor ON  = hand moving");
    Serial.println("  Motor OFF = hand still");
    Serial.println("  Parkinson's tremor range: 4–8 Hz");
    Serial.println("=========================================");

    // I2C — GPIO8/9 are native hardware I2C pins on ESP32-C3
    // Do NOT add external pull-up resistors on GPIO8/9
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);
    Serial.println("[I2C] 400kHz on GPIO8(SDA) GPIO9(SCL)");

    // ── I2C Scanner ──────────────────────────────────────
    // Scans all I2C addresses and prints what is found.
    // MPU6050 must appear at 0x68 (AD0→GND) or 0x69 (AD0→3.3V).
    // If nothing is found → check wiring before proceeding.
    Serial.println("[I2C] Scanning bus...");
    uint8_t devicesFound = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        const uint8_t err = Wire.endTransmission();
        if (err == 0) {
            Serial.print("[I2C] Device found at 0x");
            if (addr < 16) Serial.print("0");
            Serial.print(addr, HEX);
            if      (addr == 0x68) Serial.print("  ← MPU6050 (AD0=GND)  ✓");
            else if (addr == 0x69) Serial.print("  ← MPU6050 (AD0=3.3V) — move AD0 to GND");
            Serial.println();
            devicesFound++;
        }
    }
    if (devicesFound == 0) {
        Serial.println("[I2C] No devices found.");
        Serial.println("[I2C] Check: VCC→3.3V  GND→GND  SDA→GPIO8  SCL→GPIO9  AD0→GND");
    } else {
        Serial.print("[I2C] Scan done. ");
        Serial.print(devicesFound);
        Serial.println(" device(s) found.");
    }
    Serial.println("--------------------------------------------------");

    // MPU6050 — halt with Serial error if not found
    if (!mpu.begin()) {
        Serial.println("[ERROR] MPU6050 not found. Check wiring. Halting.");
        while (true) { delay(1000); }
    }

    // ESC
    esc.begin(PIN_ESC);
    delay(1000);
    esc.arm();

    // PID
    // Kp=8.0  main response strength
    // Ki=0.05 slow drift correction (keep small)
    // Kd=1.2  smooths sudden motor changes
    pid.init(8.0f, 0.05f, 1.2f);

    Serial.println("[SYSTEM] Running. Watching for hand movement...");
    Serial.println("--------------------------------------------------");
    Serial.println("Hand\t\tTremor\t\tFreq(Hz)\tMotor(us)");

    lastLoopTime   = millis();
    lastMotionTime = millis();
    freqWindowStart = millis();
}

// ─────────────────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────────────────
void loop() {
    const unsigned long now = millis();
    if (now - lastLoopTime < LOOP_MS) return;

    const float dt = (now - lastLoopTime) / 1000.0f;
    lastLoopTime   = now;

    // ── 1. Read sensor ───────────────────────────────────
    mpu.read(sensor);

    // ── 2. Measure tremor frequency ──────────────────────
    // Runs every loop (every 10ms = 100Hz sampling rate)
    // Result stored in tremorFreqHz, updated every 1 second
    // Only meaningful when tremorConfirmed is true
    measureFrequency(sensor.gyroMag, now);

    // ── 3. Debounce: hand motion ─────────────────────────
    // Refresh timestamp whenever hand is moving.
    // Motor only turns off after REST_TIMEOUT_MS of no movement.
    if (sensor.inMotion) {
        lastMotionTime = now;
    }
    const bool handActive = (now - lastMotionTime <= REST_TIMEOUT_MS);

    // ── 4. Debounce: tremor confirmation ─────────────────
    // A single spike does NOT trigger the motor.
    // Tremor must persist continuously for TREMOR_CONFIRM_MS.
    if (sensor.tremor) {
        if (tremorSince == 0) tremorSince = now;
        tremorConfirmed = (now - tremorSince >= TREMOR_CONFIRM_MS);
    } else {
        tremorSince     = 0;
        tremorConfirmed = false;
    }

    // ── 5. PID motor speed ───────────────────────────────
    int motorUS = ESC_IDLE_US;

    if (handActive && tremorConfirmed) {
        // correction is NEGATIVE (error = 0 - gyroMag)
        // multiply by -1 → positive speedAdd
        // motor spins UP during tremor
        // VERIFY: Motor(us) should INCREASE when shaking
        // e.g. 1100 → 1400. If it goes DOWN remove the minus sign.
        const float correction = pid.compute(sensor.gyroMag, dt);
        const float speedAdd   = -correction * 10.0f;

        motorUS = ESC_IDLE_US + static_cast<int>(speedAdd);
        motorUS = constrain(motorUS, MOTOR_MIN_US, MOTOR_MAX_US);
    } else {
        pid.reset();  // clear integral — prevents windup while idle/off
    }

    // ── 6. Update motor ──────────────────────────────────
    esc.update(handActive, tremorConfirmed, motorUS);

    // ── 7. Serial output every 1 second ──────────────────
    // Frequency is printed once per second (same as FREQ_WINDOW_MS)
    // so the Hz value is always fresh when printed
    static unsigned long lastPrintTime = 0;
    if (now - lastPrintTime >= FREQ_WINDOW_MS) {
        lastPrintTime = now;

        // Hand status
        Serial.print(handActive      ? "MOVING  " : "still   ");
        Serial.print('\t');

        // Tremor status
        Serial.print(tremorConfirmed ? "TREMOR  " : "none    ");
        Serial.print('\t');

        // Tremor frequency
        // Only show Hz when tremor is confirmed — otherwise shows 0.0
        if (tremorConfirmed) {
            Serial.print(tremorFreqHz, 1);
            Serial.print(" Hz");
            // Tag if frequency is in Parkinson's range (4–8 Hz)
            if (tremorFreqHz >= 4.0f && tremorFreqHz <= 8.0f) {
                Serial.print(" [Parkinson range]");
            } else if (tremorFreqHz > 8.0f) {
                Serial.print(" [above range]");
            } else {
                Serial.print(" [below range]");
            }
        } else {
            Serial.print("--      ");
        }
        Serial.print('\t');

        // Motor speed
        Serial.println(esc.currentUS());
    }
}
