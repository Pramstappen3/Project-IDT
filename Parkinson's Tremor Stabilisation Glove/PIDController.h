#pragma once
/*
 * ============================================================
 *  PIDController.h
 *  Standard PID with anti-windup clamping.
 *
 *  Tuned for vibration motor PWM output (0–255 duty):
 *    Kp = 4.0   main response  (was 8.0/15.0 — far too strong
 *               for PWM duty range; caused immediate full-blast)
 *    Ki = 0.02  slow drift correction (reduced from 0.05)
 *    Kd = 0.5   smooths sudden changes (reduced from 1.2)
 *
 *  FIXES vs original:
 *    1. integralMax/Min reduced to ±20 (was ±50) — prevents
 *       integral windup that would cause motor to keep running
 *       after tremor stops.
 *    2. compute() now clamps its own output to outputMin/Max
 *       so callers never receive out-of-range values regardless
 *       of gain settings.
 *    3. outputMin / outputMax exposed so main.cpp can adjust
 *       them once at startup if needed.
 * ============================================================
 */
#include <Arduino.h>

struct PIDController {

    // Gains — set via init()
    float kp = 0.0f, ki = 0.0f, kd = 0.0f;

    // Setpoint: we want gyroMag → 0 (no tremor)
    float setpoint = 0.0f;

    // Internal state
    float integral  = 0.0f;
    float prevError = 0.0f;

    // Anti-windup: clamp integral accumulator
    // ±20 gives max integral contribution of ki*20 = 0.4 at Ki=0.02
    float integralMax =  20.0f;
    float integralMin = -20.0f;

    // Output clamp: maps directly to VIB_MIN_DUTY / VIB_MAX_DUTY
    // Set these from main.cpp if you change motor limits.
    float outputMin =   0.0f;
    float outputMax = 220.0f;

    void init(float p, float i, float d) {
        kp = p; ki = i; kd = d;
        reset();
    }

    // Returns clamped PID output.
    // input = current gyroMag (dps)
    // dt    = elapsed time in seconds since last call
    float compute(float input, float dt) {
        const float error = setpoint - input;  // negative (gyroMag > 0)

        const float pTerm = kp * error;

        // Accumulate and clamp integral
        integral = constrain(integral + error * dt,
                             integralMin, integralMax);
        const float iTerm = ki * integral;

        // Derivative — guard against dt=0
        const float dTerm = (dt > 0.0f)
                            ? kd * (error - prevError) / dt
                            : 0.0f;
        prevError = error;

        // Clamp total output to safe range
        return constrain(pTerm + iTerm + dTerm, outputMin, outputMax);
    }

    void reset() {
        integral  = 0.0f;
        prevError = 0.0f;
    }
};
