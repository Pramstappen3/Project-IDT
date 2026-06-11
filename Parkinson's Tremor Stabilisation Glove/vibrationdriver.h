#pragma once
/*
 * ============================================================
 *  VibrationDriver.h
 *  Replaces ESCDriver.h — drives an ERM/LRA/BLDC-coin
 *  vibration motor via ESP32-C3 LEDC hardware PWM.
 *
 *  WHY THIS REPLACES ESCDriver:
 *    ESCDriver used ESP32Servo to generate 1000–2000 µs
 *    servo-style PWM for a brushless ESC.  A vibration motor
 *    (ERM coin, LRA, or BLDC coin) is driven by a simple
 *    NPN transistor and needs a 0–255 duty-cycle PWM at
 *    ~1–5 kHz — completely different signal.
 *    ESP32Servo / servo arming sequences must NOT be used.
 *
 *  Wiring:
 *    GPIO1  → 220 Ω → NPN base (S8050 or 2N2222)
 *    NPN collector → Motor –
 *    Motor + → 3.7 V battery rail
 *    1N4007 flyback diode across motor (cathode to +)
 *    NPN emitter → GND
 *
 *  LEDC channel config (ESP32-C3):
 *    Channel : 0  (channels 0–5 available on C3)
 *    Freq    : 2000 Hz  (2 kHz — good for ERM motors)
 *    Res     : 8-bit    (0–255 duty values)
 *
 *  Motor behaviour:
 *    Hand still              → PWM duty = 0   (fully off)
 *    Hand moving, no tremor  → PWM duty = 0   (off — save power)
 *    Hand moving + tremor    → PWM duty from PID (40–255)
 *
 *  NOTE: Unlike ESCDriver there is NO arming sequence.
 *        Call begin() once in setup(). That's it.
 * ============================================================
 */
#include <Arduino.h>

// ── LEDC hardware config ──────────────────────────────────
static constexpr uint8_t  VIB_LEDC_CHANNEL   = 0;
static constexpr uint32_t VIB_LEDC_FREQ_HZ   = 2000;  // 2 kHz
static constexpr uint8_t  VIB_LEDC_RESOLUTION = 8;    // 8-bit → 0–255

// ── Vibration duty limits ─────────────────────────────────
// MIN_DUTY: minimum duty at which the motor actually spins
//   (below this the motor stalls but still draws current)
//   Typical ERM coin motor: 40–60.  Tune on your motor.
// MAX_DUTY: cap to protect transistor and motor at 3.7V
static constexpr int VIB_MIN_DUTY = 50;   // ~20% — motor start threshold
static constexpr int VIB_MAX_DUTY = 220;  // ~86% — protect motor at 3.7V
static constexpr int VIB_OFF_DUTY = 0;    // fully off

class VibrationDriver {
public:

    // Call once in setup() — no arming needed
    void begin(uint8_t pin) {
        _pin = pin;
        ledcSetup(VIB_LEDC_CHANNEL, VIB_LEDC_FREQ_HZ, VIB_LEDC_RESOLUTION);
        ledcAttachPin(pin, VIB_LEDC_CHANNEL);
        ledcWrite(VIB_LEDC_CHANNEL, VIB_OFF_DUTY);
        _currentDuty = VIB_OFF_DUTY;
        Serial.print("[VIB] LEDC PWM attached GPIO");
        Serial.print(pin);
        Serial.print("  freq=");
        Serial.print(VIB_LEDC_FREQ_HZ);
        Serial.println("Hz  res=8bit");
    }

    /*
     * Call every loop.
     *   inMotion : hand is moving
     *   tremor   : tremor confirmed
     *   targetDuty : PID-computed duty (0–255)
     *
     *   !inMotion            → OFF (duty=0)
     *   inMotion && !tremor  → OFF (no point spinning while steady)
     *   inMotion && tremor   → ramp to targetDuty
     */
    void update(bool inMotion, bool tremor, int targetDuty) {
        if (!inMotion || !tremor) {
            rampTo(VIB_OFF_DUTY, 8);  // ramp down smoothly — avoids click
        } else {
            // Clamp to motor-safe range
            const int clamped = constrain(targetDuty, VIB_MIN_DUTY, VIB_MAX_DUTY);
            rampTo(clamped, 6);       // ramp up — avoids current spike
        }
    }

    // Write duty immediately (0–255)
    void writeDuty(int duty) {
        _currentDuty = constrain(duty, VIB_OFF_DUTY, VIB_MAX_DUTY);
        ledcWrite(VIB_LEDC_CHANNEL, _currentDuty);
    }

    // Increment/decrement by stepSize per call until target reached
    void rampTo(int targetDuty, int stepSize = 4) {
        if      (_currentDuty < targetDuty) _currentDuty = min(_currentDuty + stepSize, targetDuty);
        else if (_currentDuty > targetDuty) _currentDuty = max(_currentDuty - stepSize, targetDuty);
        ledcWrite(VIB_LEDC_CHANNEL, _currentDuty);
    }

    int  currentDuty() const { return _currentDuty; }

private:
    uint8_t _pin        = 0;
    int     _currentDuty = VIB_OFF_DUTY;
};
