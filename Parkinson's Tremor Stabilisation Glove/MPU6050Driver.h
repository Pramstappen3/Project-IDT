#pragma once

/*
 * MPU6050Driver.h
 * Reads MPU6050 over I2C.
 * Detects hand movement and Parkinson's tremor.
 *
 * Wiring:
 *   VCC  → 3.3V
 *   GND  → GND
 *   SDA  → GPIO8
 *   SCL  → GPIO9
 */

#include "Arduino.h"
#include <Wire.h>
#include <math.h>

// ── Registers ────────────────────────────────────────────
static constexpr uint8_t MPU_ADDR          = 0x68;
static constexpr uint8_t REG_PWR_MGMT_1    = 0x6B;
static constexpr uint8_t REG_GYRO_CONFIG   = 0x1B;
static constexpr uint8_t REG_ACCEL_CONFIG  = 0x1C;
static constexpr uint8_t REG_ACCEL_XOUT_H  = 0x3B;
static constexpr uint8_t REG_WHO_AM_I      = 0x75;

// ── Scale factors ─────────────────────────────────────────
static constexpr float GYRO_SCALE  = 131.0f;    // LSB/dps for ±250 dps
static constexpr float ACCEL_SCALE = 16384.0f;  // LSB/g   for ±2g

// ── Hand movement threshold ───────────────────────────────
// Any movement above this = hand is in use → motor ON
// Below this = hand at rest → motor OFF
// Resting sensor noise is typically ~1–2 dps and ~0.02g
static constexpr float MOTION_GYRO_FLOOR  = 3.0f;   // dps
static constexpr float MOTION_ACCEL_FLOOR = 0.05f;  // g

// ── Tremor detection thresholds ───────────────────────────
// Parkinson's tremor: 4–8 Hz, high-frequency involuntary bursts
static constexpr float TREMOR_GYRO_THRESH  = 15.0f; // dps
static constexpr float TREMOR_ACCEL_THRESH = 0.12f; // g

// ── SensorData struct ─────────────────────────────────────
struct SensorData {
    float ax = 0, ay = 0, az = 0;   // accelerometer (g)
    float gx = 0, gy = 0, gz = 0;   // gyroscope (dps)
    float gyroMag  = 0;              // gyro vector magnitude
    float accelDev = 0;              // deviation from 1g
    bool  inMotion = false;          // hand is moving
    bool  tremor   = false;          // tremor detected
};

// ── MPU6050Driver class ───────────────────────────────────
class MPU6050Driver {
public:

    void writeReg(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(reg);
        Wire.write(val);
        Wire.endTransmission();
    }

    uint8_t readReg(uint8_t reg) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom(static_cast<uint8_t>(MPU_ADDR),
                         static_cast<uint8_t>(1), true);
        return Wire.read();
    }

    bool begin() {
        writeReg(REG_PWR_MGMT_1,   0x00);  // wake up
        delay(100);
        writeReg(REG_GYRO_CONFIG,  0x00);  // ±250 dps
        writeReg(REG_ACCEL_CONFIG, 0x00);  // ±2g
        delay(100);

        const uint8_t id = readReg(REG_WHO_AM_I);
        if (id == 0x68) {
            Serial.println("[MPU6050] Connected OK");
            return true;
        }
        Serial.print("[MPU6050] ERROR WHO_AM_I=0x");
        Serial.println(id, HEX);
        return false;
    }

    void read(SensorData &s) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(REG_ACCEL_XOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom(static_cast<uint8_t>(MPU_ADDR),
                         static_cast<uint8_t>(14), true);

        auto rw = []() -> int16_t {
            return static_cast<int16_t>((Wire.read() << 8) | Wire.read());
        };

        const int16_t rAX = rw(), rAY = rw(), rAZ = rw();
        rw();   // temperature — discard
        const int16_t rGX = rw(), rGY = rw(), rGZ = rw();

        s.ax = rAX / ACCEL_SCALE;
        s.ay = rAY / ACCEL_SCALE;
        s.az = rAZ / ACCEL_SCALE;
        s.gx = rGX / GYRO_SCALE;
        s.gy = rGY / GYRO_SCALE;
        s.gz = rGZ / GYRO_SCALE;

        // Total rotational speed across all axes
        s.gyroMag = sqrtf(s.gx*s.gx + s.gy*s.gy + s.gz*s.gz);

        // How far total acceleration deviates from 1g (still hand = 1g)
        const float totalG = sqrtf(s.ax*s.ax + s.ay*s.ay + s.az*s.az);
        s.accelDev = fabsf(totalG - 1.0f);

        // Hand movement: any movement above resting noise floor
        s.inMotion = (s.gyroMag  > MOTION_GYRO_FLOOR ||
                      s.accelDev > MOTION_ACCEL_FLOOR);

        // Tremor: high-frequency burst — only when hand is already moving
        s.tremor = s.inMotion &&
                   (s.gyroMag  > TREMOR_GYRO_THRESH ||
                    s.accelDev > TREMOR_ACCEL_THRESH);
    }
};