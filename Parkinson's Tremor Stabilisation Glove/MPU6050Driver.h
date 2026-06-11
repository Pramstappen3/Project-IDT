#pragma once
/*
 * ============================================================
 *  MPU6050Driver.h  — no changes needed for 5-motor version
 *  Wiring:
 *    VCC → 3.3V  GND → GND
 *    SDA → GPIO8  SCL → GPIO9  AD0 → GND (address 0x68)
 * ============================================================
 */
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

static constexpr uint8_t MPU_ADDR         = 0x68;
static constexpr uint8_t REG_SMPLRT_DIV   = 0x19;
static constexpr uint8_t REG_CONFIG       = 0x1A;
static constexpr uint8_t REG_GYRO_CONFIG  = 0x1B;
static constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;
static constexpr uint8_t REG_ACCEL_XOUT_H = 0x3B;
static constexpr uint8_t REG_PWR_MGMT_1   = 0x6B;
static constexpr uint8_t REG_WHO_AM_I     = 0x75;

static constexpr float GYRO_SCALE  = 131.0f;    // LSB/dps  ±250 dps
static constexpr float ACCEL_SCALE = 16384.0f;  // LSB/g    ±2g

static constexpr float MOTION_GYRO_FLOOR  = 1.5f;   // dps
static constexpr float MOTION_ACCEL_FLOOR = 0.03f;  // g
static constexpr float TREMOR_GYRO_THRESH  = 15.0f;  // dps
static constexpr float TREMOR_ACCEL_THRESH = 0.12f;  // g

struct SensorData {
    float ax = 0, ay = 0, az = 0;
    float gx = 0, gy = 0, gz = 0;
    float gyroMag  = 0;
    float accelDev = 0;
    bool  inMotion = false;
    bool  tremor   = false;
};

class MPU6050Driver {
public:
    void writeReg(uint8_t reg, uint8_t val) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(reg); Wire.write(val);
        Wire.endTransmission();
    }
    uint8_t readReg(uint8_t reg) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(reg);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1, (uint8_t)true);
        return Wire.read();
    }
    bool begin() {
        writeReg(REG_PWR_MGMT_1, 0x00); delay(100);
        writeReg(REG_CONFIG,       0x03); // DLPF 44 Hz — cuts motor noise, passes 4–8 Hz tremor
        writeReg(REG_SMPLRT_DIV,   0x09); // 100 Hz sample rate
        writeReg(REG_GYRO_CONFIG,  0x00); // ±250 dps
        writeReg(REG_ACCEL_CONFIG, 0x00); // ±2g
        delay(100);
        const uint8_t id = readReg(REG_WHO_AM_I);
        if (id == 0x68 || id == 0x98) {
            Serial.print("[MPU6050] OK  WHO_AM_I=0x"); Serial.println(id, HEX);
            return true;
        }
        Serial.print("[MPU6050] ERROR  WHO_AM_I=0x"); Serial.println(id, HEX);
        return false;
    }
    void read(SensorData &s) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(REG_ACCEL_XOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);
        auto rw = []() -> int16_t {
            return (int16_t)((Wire.read() << 8) | Wire.read());
        };
        const int16_t rAX=rw(), rAY=rw(), rAZ=rw();
        rw(); // temp — discard
        const int16_t rGX=rw(), rGY=rw(), rGZ=rw();
        s.ax = rAX / ACCEL_SCALE; s.ay = rAY / ACCEL_SCALE; s.az = rAZ / ACCEL_SCALE;
        s.gx = rGX / GYRO_SCALE;  s.gy = rGY / GYRO_SCALE;  s.gz = rGZ / GYRO_SCALE;
        s.gyroMag  = sqrtf(s.gx*s.gx + s.gy*s.gy + s.gz*s.gz);
        const float totalG = sqrtf(s.ax*s.ax + s.ay*s.ay + s.az*s.az);
        s.accelDev = fabsf(totalG - 1.0f);
        s.inMotion = (s.gyroMag > MOTION_GYRO_FLOOR || s.accelDev > MOTION_ACCEL_FLOOR);
        s.tremor   = s.inMotion &&
                     (s.gyroMag > TREMOR_GYRO_THRESH || s.accelDev > TREMOR_ACCEL_THRESH);
    }
};
