#pragma once
/*
 * ============================================================
 *  PIDController.h  — no changes needed for 5-motor version
 *  Kp=4.0  Ki=0.02  Kd=0.5  (tuned for 0–255 PWM range)
 * ============================================================
 */
#include <Arduino.h>

struct PIDController {
    float kp = 0.0f, ki = 0.0f, kd = 0.0f;
    float setpoint    = 0.0f;
    float integral    = 0.0f;
    float prevError   = 0.0f;
    float integralMax =  20.0f;
    float integralMin = -20.0f;
    float outputMin   =   0.0f;
    float outputMax   = 220.0f;

    void init(float p, float i, float d) {
        kp = p; ki = i; kd = d;
        reset();
    }
    float compute(float input, float dt) {
        const float error = setpoint - input;
        const float pTerm = kp * error;
        integral = constrain(integral + error * dt, integralMin, integralMax);
        const float iTerm = ki * integral;
        const float dTerm = (dt > 0.0f) ? kd * (error - prevError) / dt : 0.0f;
        prevError = error;
        return constrain(pTerm + iTerm + dTerm, outputMin, outputMax);
    }
    void reset() {
        integral  = 0.0f;
        prevError = 0.0f;
    }
};
