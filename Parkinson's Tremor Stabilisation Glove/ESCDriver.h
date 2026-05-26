#pragma once

/*
 * ESCDriver.h
 * Controls BLDC motor via 30A ESC using PWM.
 *
 * Wiring:
 *   GPIO1 → 200Ω resistor → ESC signal pin
 *   Battery+ → diode → capacitor → ESC power+
 *   GND      → ESC power−
 *
 * Motor behavior:
 *   Hand moving + tremor  → motor ON  at stabilization speed
 *   Hand moving, no tremor→ motor ON  at idle speed (ready)
 *   Hand completely still → motor OFF (stopped, saves battery)
 */

#include "Arduino.h"
#include <ESP32Servo.h>

// ── ESC PWM range ─────────────────────────────────────────
static constexpr int ESC_MIN_US   = 1000;  // fully stopped
static constexpr int ESC_MAX_US   = 2000;  // full speed
static constexpr int ESC_IDLE_US  = 1100;  // idle when hand is moving
static constexpr int MOTOR_MIN_US = 1100;  // min stabilization speed
static constexpr int MOTOR_MAX_US = 1700;  // max safe speed on glove

class ESCDriver {
public:

    void begin(uint8_t pin) {
        _pin = pin;
        _esc.setPeriodHertz(50);                   // 50 Hz standard ESC
        _esc.attach(pin, ESC_MIN_US, ESC_MAX_US);
        writeUS(ESC_MIN_US);
        Serial.print("[ESC] Attached GPIO");
        Serial.println(pin);
    }

    /*
     * Arming sequence — run once on every power-up.
     * ESC must see MAX throttle then MIN throttle to calibrate.
     * Listen for 2–3 beeps = armed successfully.
     */
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
     *   inMotion  : true = hand is moving
     *   tremor    : true = Parkinson's tremor confirmed
     *   targetUS  : PID-computed motor speed (used only when tremor=true)
     *
     * Logic:
     *   !inMotion            → ramp to STOPPED
     *   inMotion && !tremor  → ramp to IDLE (stays ready)
     *   inMotion && tremor   → write targetUS (stabilizing)
     */
    void update(bool inMotion, bool tremor, int targetUS) {
        if (!inMotion) {
            // Hand at rest — shut motor off completely
            rampTo(ESC_MIN_US, 5);

        } else if (!tremor) {
            // Hand moving but no tremor — idle spin, ready to react
            rampTo(ESC_IDLE_US, 3);

        } else {
            // Tremor detected — run at PID-computed speed
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