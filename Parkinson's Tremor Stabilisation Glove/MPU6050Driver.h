#pragma once
/*
 * ============================================================
 *  MPU6050Driver.h
 *  Wiring (ESP32-C3):
 *    VCC → 3.3V  (through AMS1117-3.3)
 *    GND → GND
 *    SDA → GPIO8   (native hardware I2C on ESP32-C3)
 *    SCL → GPIO9   (native hardware I2C on ESP32-C3)
 *    AD0 → GND     (I2C address = 0x68)
 *
 *  FIXES vs original:
 *    1. Added DLPF (Digital Low Pass Filter) at 44 Hz — cuts
 *       high-frequency motor/mechanical noise above 44 Hz so
 *       the 4–8 Hz tremor signal is much cleaner.
 *    2. WHO_AM_I accepts 0x68 OR 0x98 — some common MPU-6050
 *       clones (GY-521 blue board) return 0x98 but work fine.
 *    3. #include <Arduino.h> with angle brackets (not quotes)
 *       — required for Arduino IDE on ESP32 platform.
 *    4. Added SMPLRT_DIV register to set sample rate = 100 Hz
 *       (matches LOOP_MS = 10ms in main).
 * ============================================================
 */
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// ── Registers ────────────────────────────────────────────
static constexpr uint8_t MPU_ADDR          = 0x68;
static constexpr uint8_t REG_SMPLRT_DIV    = 0x19;  // sample rate divider
static constexpr uint8_t REG_CONFIG        = 0x1A;  // DLPF config  ← NEW
static constexpr uint8_t REG_GYRO_CONFIG   = 0x1B;
static constexpr uint8_t REG_ACCEL_CONFIG  = 0x1C;
static constexpr uint8_t REG_ACCEL_XOUT_H  = 0x3B;
static constexpr uint8_t REG_PWR_MGMT_1    = 0x6B;
static constexpr uint8_t REG_WHO_AM_I      = 0x75;

// ── Scale factors ─────────────────────────────────────────
static constexpr float GYRO_SCALE  = 131.0f;    // LSB/dps — ±250 dps range
static constexpr float ACCEL_SCALE = 16384.0f;  // LSB/g   — ±2g range

// ── Motion detection thresholds ───────────────────────────
// Resting sensor noise: ~1–2 dps and ~0.02g
static constexpr float MOTION_GYRO_FLOOR  = 1.5f;   // dps
static constexpr float MOTION_ACCEL_FLOOR = 0.03f;  // g

// ── Tremor detection thresholds ───────────────────────────
// Parkinson's tremor: 4–8 Hz, involuntary burst pattern
static constexpr float TREMOR_GYRO_THRESH  = 15.0f;  // dps
static constexpr float TREMOR_ACCEL_THRESH = 0.12f;  // g

// ── SensorData ────────────────────────────────────────────
struct SensorData {
    float ax = 0, ay = 0, az = 0;  // accelerometer (g)
    float gx = 0, gy = 0, gz = 0;  // gyroscope (dps)
    float gyroMag  = 0;             // gyro vector magnitude (dps)
    float accelDev = 0;             // deviation from 1g
    bool  inMotion = false;         // hand is moving
    bool  tremor   = false;         // tremor burst detected
};

// ── MPU6050Driver ─────────────────────────────────────────
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
                         static_cast<uint8_t>(1),
                         static_cast<uint8_t>(true));
        return Wire.read();
    }

    bool begin() {
        // Wake up — clear sleep bit
        writeReg(REG_PWR_MGMT_1, 0x00);
        delay(100);

        // DLPF: 0x03 = 44 Hz bandwidth, 4.9 ms delay
        // This removes noise above 44 Hz (motor vibration, etc.)
        // while passing the 4–8 Hz Parkinson's tremor band cleanly.
        // REG_CONFIG bits [2:0] set DLPF_CFG:
        //   0x00 = no filter (260 Hz) — too noisy
        //   0x03 = 44 Hz  ← correct for tremor detection
        //   0x04 = 21 Hz  — too aggressive, may round tremor peaks
        writeReg(REG_CONFIG, 0x03);

        // Sample rate: SMPLRT_DIV = 9 → 1000 / (1+9) = 100 Hz
        // Matches LOOP_MS = 10ms in main.cpp
        writeReg(REG_SMPLRT_DIV, 0x09);

        // Gyro: ±250 dps (0x00)
        writeReg(REG_GYRO_CONFIG,  0x00);

        // Accel: ±2g (0x00)
        writeReg(REG_ACCEL_CONFIG, 0x00);
        delay(100);

        // WHO_AM_I check — genuine MPU-6050 returns 0x68
        // Many GY-521 blue-board clones return 0x98 but are otherwise identical
        const uint8_t id = readReg(REG_WHO_AM_I);
        if (id == 0x68 || id == 0x98) {
            Serial.print("[MPU6050] Connected OK  WHO_AM_I=0x");
            Serial.println(id, HEX);
            return true;
        }
        Serial.print("[MPU6050] ERROR — unexpected WHO_AM_I=0x");
        Serial.println(id, HEX);
        Serial.println("[MPU6050] Expected 0x68 (genuine) or 0x98 (clone)");
        Serial.println("[MPU6050] Check: SDA→GPIO8  SCL→GPIO9  VCC→3.3V  AD0→GND");
        return false;
    }

    void read(SensorData &s) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(REG_ACCEL_XOUT_H);
        Wire.endTransmission(false);
        Wire.requestFrom(static_cast<uint8_t>(MPU_ADDR),
                         static_cast<uint8_t>(14),
                         static_cast<uint8_t>(true));

        // Helper: read two bytes as big-endian int16
        auto rw = []() -> int16_t {
            return static_cast<int16_t>((Wire.read() << 8) | Wire.read());
        };

        const int16_t rAX = rw(), rAY = rw(), rAZ = rw();
        rw();  // temperature — discard
        const int16_t rGX = rw(), rGY = rw(), rGZ = rw();

        s.ax = rAX / ACCEL_SCALE;
        s.ay = rAY / ACCEL_SCALE;
        s.az = rAZ / ACCEL_SCALE;
        s.gx = rGX / GYRO_SCALE;
        s.gy = rGY / GYRO_SCALE;
        s.gz = rGZ / GYRO_SCALE;

        // Gyro magnitude: total rotational speed across all axes
        s.gyroMag = sqrtf(s.gx*s.gx + s.gy*s.gy + s.gz*s.gz);

        // Accel deviation from 1g — still hand = exactly 1g (gravity only)
        const float totalG = sqrtf(s.ax*s.ax + s.ay*s.ay + s.az*s.az);
        s.accelDev = fabsf(totalG - 1.0f);

        // inMotion: any reading above resting noise floor
        s.inMotion = (s.gyroMag  > MOTION_GYRO_FLOOR ||
                      s.accelDev > MOTION_ACCEL_FLOOR);

        // tremor: high-frequency burst, only when hand is already moving
        s.tremor = s.inMotion &&
                   (s.gyroMag  > TREMOR_GYRO_THRESH ||
                    s.accelDev > TREMOR_ACCEL_THRESH);
    }
};
