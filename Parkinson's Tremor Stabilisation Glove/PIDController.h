#pragma once
/*
 * PIDController.h
 * Standard PID with anti-windup clamping
 * Kp=8.0  main response strength
 * Ki=0.05 slow drift correction
 * Kd=1.2  smooths sudden changes
 */
#include "Arduino.h"

struct PIDController {
    float kp = 0, ki = 0, kd = 0;
    float setpoint    =  0.0f;
    float integral    =  0.0f;
    float prevError   =  0.0f;
    float integralMax =  50.0f;
    float integralMin = -50.0f;

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
        return pTerm + iTerm + dTerm;
    }

    void reset() {
        integral  = 0.0f;
        prevError = 0.0f;
    }
};
