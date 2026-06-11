#pragma once
/*
 * ============================================================
 *  VibrationDriver.h  — 5-motor version
 *  Drives 5 ERM coin vibration motors via ESP32-C3 LEDC PWM.
 *
 *  WIRING (each motor identical):
 *    GPIOx → 200Ω → NPN base (2N2222 / BC547 / S8050)
 *    NPN collector → Motor –
 *    Motor +       → 3.7V battery rail
 *    1N4007        → across motor (cathode to +)
 *    NPN emitter   → GND
 *
 *  PIN ASSIGNMENT (5 motors):
 *    Motor 0 (index)   → GPIO1
 *    Motor 1 (middle)  → GPIO2
 *    Motor 2 (ring)    → GPIO3
 *    Motor 3 (pinky)   → GPIO4
 *    Motor 4 (thumb)   → GPIO5
 *
 *  LEDC config per channel:
 *    Freq : 2000 Hz (2 kHz — good for ERM motors)
 *    Res  : 8-bit  (0–255 duty)
 *    Channels: 0–4 (one per motor)
 *
 *  AUTO ON/OFF LOGIC:
 *    Hand still              → ALL motors OFF (duty = 0)
 *    Hand moving, no tremor  → ALL motors OFF (save power)
 *    Hand moving + tremor    → motors ON, duty from PID
 *
 *  ZONE ACTIVATION (based on tremor severity):
 *    Mild   (duty 50–100)  → Motor 0 only
 *    Medium (duty 101–160) → Motors 0 + 1
 *    Strong (duty 161–220) → Motors 0 + 1 + 2
 *    Severe (duty > 220)   → All 5 motors ON
 *
 *  NOTE: GPIO6–11 are flash on ESP32-C3 — NEVER use them.
 * ============================================================
 */
#include <Arduino.h>

// ── Pin assignments ───────────────────────────────────────
static constexpr uint8_t MOTOR_COUNT = 5;
static constexpr uint8_t MOTOR_PINS[MOTOR_COUNT] = {2, 3, 4, 5, 10};

// ── LEDC config ───────────────────────────────────────────
static constexpr uint32_t VIB_LEDC_FREQ_HZ    = 2000;
static constexpr uint8_t  VIB_LEDC_RESOLUTION = 8;     // 8-bit → 0–255

// ── Duty limits ───────────────────────────────────────────
static constexpr int VIB_MIN_DUTY = 50;   // ~20% — motor start threshold
static constexpr int VIB_MAX_DUTY = 220;  // ~86% — safe at 3.7V
static constexpr int VIB_OFF_DUTY = 0;

// ── Zone thresholds (how many motors activate) ────────────
static constexpr int ZONE_1_MAX = 100;   // 1 motor
static constexpr int ZONE_2_MAX = 160;   // 2 motors
static constexpr int ZONE_3_MAX = 200;   // 3 motors
static constexpr int ZONE_4_MAX = 220;   // 4 motors
// above 220 → all 5 motors

class VibrationDriver {
public:

    // Call once in setup() — no arming needed
   // Call once in setup() — updated for ESP32 Arduino Core v3.0+
    void begin() {
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            // New v3.0+ API: ledcAttach replaces both ledcSetup and ledcAttachPin
            ledcAttach(MOTOR_PINS[i], VIB_LEDC_FREQ_HZ, VIB_LEDC_RESOLUTION);
            
            ledcWrite(MOTOR_PINS[i], VIB_OFF_DUTY);
            _duty[i] = VIB_OFF_DUTY;

            Serial.print("[VIB] Motor ");
            Serial.print(i);
            Serial.print(" → GPIO");
            Serial.print(MOTOR_PINS[i]);
            Serial.println(" initialized with LEDC.");
        }
        Serial.println("[VIB] All 5 motors ready.");
    }

    /*
     * Call every loop iteration.
     *   inMotion   : hand is actively moving
     *   tremor     : tremor confirmed by debounce
     *   targetDuty : PID output (0–255)
     *
     * Auto ON/OFF:
     *   !inMotion OR !tremor → ramp ALL motors to 0 (OFF)
     *   inMotion AND tremor  → activate motors by zone
     */
    void update(bool inMotion, bool tremor, int targetDuty) {
        if (!inMotion || !tremor) {
            // ── AUTO OFF: ramp all motors down ────────────
            for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
                rampMotor(i, VIB_OFF_DUTY, 8);
            }
            return;
        }

        // ── AUTO ON: zone-based activation ────────────────
        const int clamped = constrain(targetDuty, VIB_MIN_DUTY, VIB_MAX_DUTY);
        const uint8_t activeMotors = zonesActive(clamped);

        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            if (i < activeMotors) {
                rampMotor(i, clamped, 6);   // ramp UP
            } else {
                rampMotor(i, VIB_OFF_DUTY, 8); // ramp off unused motors
            }
        }
    }

    // Returns how many motors should be ON for a given duty level
    uint8_t zonesActive(int duty) const {
        if      (duty > ZONE_4_MAX) return 5;
        else if (duty > ZONE_3_MAX) return 4;
        else if (duty > ZONE_2_MAX) return 3;
        else if (duty > ZONE_1_MAX) return 2;
        else                        return 1;
    }

    // Turn off all motors immediately (emergency stop)
    void allOff() {
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            ledcWrite(i, VIB_OFF_DUTY);
            _duty[i] = VIB_OFF_DUTY;
        }
    }

    int  currentDuty(uint8_t motor = 0) const {
        if (motor >= MOTOR_COUNT) return 0;
        return _duty[motor];
    }

    uint8_t activeCount() const {
        uint8_t n = 0;
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            if (_duty[i] > 0) n++;
        }
        return n;
    }

private:
    int _duty[MOTOR_COUNT] = {};

    void rampMotor(uint8_t ch, int target, int step) {
        if      (_duty[ch] < target) _duty[ch] = min(_duty[ch] + step, target);
        else if (_duty[ch] > target) _duty[ch] = max(_duty[ch] - step, target);
        ledcWrite(ch, _duty[ch]);
    }
};
