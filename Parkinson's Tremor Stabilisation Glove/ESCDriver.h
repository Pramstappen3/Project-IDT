#pragma once
/*
 * ESCDriver.h
 * Controls BLDC motor via 30A ESC using PWM.
 *
 * Wiring:
 *   GPIO1 → 200Ω resistor → ESC signal pin
 *   ESC BAT+ → battery+ (through diode + capacitor)
 *   ESC BAT- → GND
 *
 * Motor behavior:
 *   Hand still              → motor FULLY STOPPED
 *   Hand moving, no tremor  → motor IDLE spin (ready)
 *   Hand moving + tremor    → motor ACTIVE stabilization
 */
#include "Arduino.h"
#include <ESP32Servo.h>

static constexpr int ESC_MIN_US   = 1000;  // fully stopped
static constexpr int ESC_MAX_US   = 2000;  // full speed
static constexpr int ESC_IDLE_US  = 1200;  // raised for A2212 1400KV at 3.7V
static constexpr int MOTOR_MIN_US = 1200;  // raised for A2212 1400KV at 3.7V
static constexpr int MOTOR_MAX_US = 1900;  // raised for A2212 1400KV at 3.7V

class ESCDriver {
public:

    void begin(uint8_t pin) {
        _pin = pin;
        _esc.setPeriodHertz(50);
        _esc.attach(pin, ESC_MIN_US, ESC_MAX_US);
        writeUS(ESC_MIN_US);
        Serial.print("[ESC] Attached GPIO");
        Serial.println(pin);
    }

    // Must run once on every power-up
    // ESC beeps 2-3 times when armed successfully
    void arm() {
        Serial.println("[ESC] Arming: MAX throttle 2s...");
        writeUS(ESC_MAX_US);
        delay(2000);
        Serial.println("[ESC] Arming: MIN throttle 2s...");
        writeUS(ESC_MIN_US);
        delay(2000);
        _armed = true;
        Serial.println("[ESC] Armed.");
        delay(500);
    }

    /*
     * Call every loop.
     *   inMotion : hand is moving
     *   tremor   : tremor confirmed
     *   targetUS : PID-computed speed (used only when tremor=true)
     *
     *   !inMotion            → ramp to STOPPED
     *   inMotion && !tremor  → ramp to IDLE
     *   inMotion && tremor   → write targetUS
     */
    void update(bool inMotion, bool tremor, int targetUS) {
        if (!inMotion) {
            rampTo(ESC_MIN_US, 5);   // hand still — shut off completely
        } else if (!tremor) {
            rampTo(ESC_IDLE_US, 3);  // moving, no tremor — stay ready
        } else {
            writeUS(constrain(targetUS, MOTOR_MIN_US, MOTOR_MAX_US));
        }
    }

    void writeUS(int us) {
        _currentUS = constrain(us, ESC_MIN_US, ESC_MAX_US);
        _esc.writeMicroseconds(_currentUS);
    }

    void rampTo(int targetUS, int stepUS = 2) {
        if      (_currentUS < targetUS) _currentUS = min(_currentUS + stepUS, targetUS);
        else if (_currentUS > targetUS) _currentUS = max(_currentUS - stepUS, targetUS);
        _esc.writeMicroseconds(_currentUS);
    }

    int  currentUS() const { return _currentUS; }
    bool armed()     const { return _armed;     }

private:
    Servo   _esc;
    uint8_t _pin       = 0;
    int     _currentUS = ESC_MIN_US;
    bool    _armed     = false;
};
