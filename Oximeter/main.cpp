#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

// Buffers for SpO2 algorithm
uint32_t irBuffer[100];
uint32_t redBuffer[100];

int32_t bufferLength = 100;
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("Initializing MAX30102...");

  // Initialize I2C for ESP32-C3 (SDA=GPIO8, SCL=GPIO9 by default)
  Wire.begin(6, 7);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 not found. Check wiring!");
    while (1);
  }

  Serial.println("Place your finger on the sensor...");

  // Configure sensor
  byte ledBrightness = 60;   // 0=Off to 255=50mA
  byte sampleAverage = 4;    // 1, 2, 4, 8, 16, 32
  byte ledMode = 2;          // 1=Red only, 2=Red+IR
  byte sampleRate = 100;     // 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411;      // 69, 118, 215, 411
  int adcRange = 4096;       // 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode,
                       sampleRate, pulseWidth, adcRange);
}

void loop() {
  // Fill buffer with 100 samples first
  for (byte i = 0; i < bufferLength; i++) {
    while (!particleSensor.available())
      particleSensor.check();

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();
  }

  // Calculate SpO2 and heart rate
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, bufferLength, redBuffer,
    &spo2, &validSPO2, &heartRate, &validHeartRate
  );

  // Continuous reading loop
  while (1) {
    // Shift buffer left by 25 samples
    for (byte i = 25; i < 100; i++) {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25]  = irBuffer[i];
    }

    // Read 25 new samples
    for (byte i = 75; i < 100; i++) {
      while (!particleSensor.available())
        particleSensor.check();

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i]  = particleSensor.getIR();
      particleSensor.nextSample();
    }

    // Recalculate
    maxim_heart_rate_and_oxygen_saturation(
      irBuffer, bufferLength, redBuffer,
      &spo2, &validSPO2, &heartRate, &validHeartRate
    );

    // Print results
    Serial.print("HR=");
    Serial.print(heartRate, DEC);
    Serial.print(" bpm | Valid=");
    Serial.print(validHeartRate, DEC);
    Serial.print(" | SpO2=");
    Serial.print(spo2, DEC);
    Serial.print("% | Valid=");
    Serial.println(validSPO2, DEC);

    // Finger detection (IR > 50000 = finger present)
    if (irBuffer[99] < 50000) {
      Serial.println("No finger detected! Please place finger on sensor.");
    }
  }
}
