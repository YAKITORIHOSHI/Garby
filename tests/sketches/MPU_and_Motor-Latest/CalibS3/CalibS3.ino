// ESP32-S3 MPU sender for the Calib motor sketch.
// Mount the MPU6050 level with +Z upward; keep still during boot calibration.
#include <Arduino.h>
#include <HardwareSerial.h>
#include <math.h>
#include <stdio.h>
#include "MpuHeading.h"
#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Build CalibS3.ino for the ESP32-S3 board."
#endif

static constexpr int MPU_SDA_PIN = 8;
static constexpr int MPU_SCL_PIN = 9;
static constexpr int UART_RX_PIN = 18;  // from ESP32 TX19, reserved
static constexpr int UART_TX_PIN = 17;  // to ESP32 RX18
static constexpr uint32_t SEND_INTERVAL_MS = 20;

HardwareSerial motorLink(1);
MpuHeading mpu;
uint32_t sequence = 0;
// Numbers each heading reference (never 0). Calib.ino needs this fifth
// field: it rejects 3-field packets and then drives fully open loop.
uint32_t referenceEpoch = 0;
uint32_t lastSendMs = 0;
uint32_t lastFaultMs = 0;
bool faultSent = false;

static void sendFault() {
  const uint32_t now = millis();
  if (!faultSent || now - lastFaultMs >= 200) {
    motorLink.println("F");
    lastFaultMs = now;
    faultSent = true;
  }
}

// Every new heading reference reads 0 at an unknown chassis angle; a new
// epoch tells the ESP32 not to treat that jump as real rotation.
static void startReference() {
  mpu.requestReference();
  if (++referenceEpoch == 0) ++referenceEpoch;
}

static void sendSample() {
  char payload[48];
  const long headingCdeg = lroundf(mpu.headingDeg() * 100.0f);
  const long rateCdps = lroundf(mpu.rateDps() * 100.0f);
  if (++sequence == 0) ++sequence;
  snprintf(payload, sizeof(payload), "M,%lu,%ld,%ld,%lu",
           static_cast<unsigned long>(sequence), headingCdeg, rateCdps,
           static_cast<unsigned long>(referenceEpoch));
  uint8_t check = 0;
  for (const char* p = payload; *p; ++p) check ^= static_cast<uint8_t>(*p);
  motorLink.printf("%s*%02X\n", payload, check);
  faultSent = false;
}

void setup() {
  motorLink.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  sendFault();  // tell ESP32 to clear steering if only the S3 reboots
  if (mpu.begin(MPU_SDA_PIN, MPU_SCL_PIN, false)) {
    startReference();
  } else {
    mpu.beginRecovery();
    sendFault();
  }
}

void loop() {
  const MpuHeading::Sample sample = mpu.poll();
  if (sample == MpuHeading::Sample::FAULT) {
    sendFault();
    mpu.beginRecovery();
  }
  if (!mpu.ready()) {
    sendFault();
    if (mpu.serviceRecovery() && !mpu.valid()) {
      startReference();
    }
  } else if (mpu.valid() &&
             (sample == MpuHeading::Sample::REFERENCE ||
              sample == MpuHeading::Sample::UPDATED) &&
             millis() - lastSendMs >= SEND_INTERVAL_MS) {
    sendSample();
    lastSendMs = millis();
  } else if (!mpu.valid() && !mpu.referenceSettling()) {
    sendFault();
    mpu.beginRecovery();
  }
  delay(1);
}
