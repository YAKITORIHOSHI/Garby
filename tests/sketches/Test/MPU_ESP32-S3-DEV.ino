/*
 * MPU-6050 / GY-521 relative FRONT / LEFT / RIGHT
 * Target: ESP32-S3, Arduino framework
 *
 * Mount the sensor level on the moving system, with +Z upward.
 * It must rotate with the system.
 *
 * Wiring with the generic ESP32S3 Dev Module board setting:
 *   VCC -> 3V3 (use a breakout that supports a 3.3 V supply)
 *   GND -> GND
 *   SDA -> GPIO8 (IO8)
 *   SCL -> GPIO9 (IO9)
 *   AD0 -> GND (I2C address 0x68)
 *   XDA, XCL, INT -> Leave disconnected
 * The code uses SDA/SCL from the selected Arduino board variant.
 * Other board variants can use different pins; match your wiring to
 * I2C_SDA_PIN and I2C_SCL_PIN below, or set those constants explicitly.
 * IO8/IO9 are GPIO labels printed on the board, not header positions.
 * Keep SDA/SCL pull-ups at 3.3 V for the ESP32-S3.
 * Select the matching ESP32-S3 board in your build/upload GUI.
 * For Serial over native USB, enable USB CDC On Boot in board settings.
 * For Serial over a USB-to-UART bridge, disable USB CDC On Boot.
 *
 * Serial at 115200 baud:
 *   Startup sets the first valid heading as FRONT.
 *   Send R or r to make the CURRENT heading the new FRONT.
 *   LEFT / RIGHT describe deviation from that reference heading.
 *   The label remains LEFT / RIGHT when rotation stops.
 *   Return toward the reference heading to get FRONT again.
 *
 * Raw gyro Z is integrated over time. There is no calibration or
 * automatic offset subtraction. Gyro bias can therefore cause drift;
 * R/r resets the reference, not the gyro bias.
 * FRONT indicates heading alignment, not proof of forward motion or
 * position on a physical line.
 *
 * Register reference:
 * https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf
 */

#include <Arduino.h>
#include <Wire.h>
#include <FastAccelStepper.h>
#include <HardwareSerial.h>

HardwareSerial ESP32_Serial(1);

// Board defaults, or replace SDA/SCL with pin numbers matching your wiring.
const int I2C_SDA_PIN = SDA;  // GPIO8 (IO8) with ESP32S3 Dev Module.
const int I2C_SCL_PIN = SCL;  // GPIO9 (IO9) with ESP32S3 Dev Module.
const uint32_t I2C_CLOCK_HZ = 100000UL;
const float GYRO_COUNTS_PER_DPS = 131.0f;  // +/-250 degrees/second.
const bool REVERSE_LEFT_RIGHT = false;
const bool PRINT_RAW_VALUES = false;
const float TURN_ANGLE_DEG = 2.0f;    // Enter LEFT/RIGHT at 2 degrees.
const float FRONT_ANGLE_DEG = 1.0f;   // Return to FRONT within 1 degree.
const uint32_t SAMPLE_INTERVAL_MS = 10;
const uint32_t PRINT_INTERVAL_MS = 200;
const uint32_t SENSOR_SETTLE_MS = 100;  // Hardware startup, no calibration.
const uint32_t MAX_SAMPLE_GAP_US = 100000;

const byte FRONT = 0;
const byte LEFT = 1;
const byte RIGHT = 2;
const byte REG_SMPLRT_DIV = 0x19;
const byte REG_CONFIG = 0x1A;
const byte REG_GYRO_CONFIG = 0x1B;
const byte REG_FIFO_EN = 0x23;
const byte REG_INT_ENABLE = 0x38;
const byte REG_GYRO_ZOUT_H = 0x47;
const byte REG_USER_CTRL = 0x6A;
const byte REG_PWR_MGMT_1 = 0x6B;
const byte REG_PWR_MGMT_2 = 0x6C;
const byte REG_WHO_AM_I = 0x75;

byte mpuAddress = 0x68;
bool sensorReady = false;
bool headingValid = false;
bool resetRequested = true;
bool faultReported = false;
byte currentDirection = FRONT;
float headingDeg = 0.0f;
float previousRateDps = 0.0f;
uint32_t startedAt = 0;
uint32_t lastSampleMs = 0;
uint32_t lastPrintMs = 0;
uint32_t lastErrorMs = 0;
uint32_t lastGyroUs = 0;

bool mpuWrite(byte reg, byte value) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool mpuReadMulti(byte startReg, byte *buffer, byte count) {
  Wire.beginTransmission(mpuAddress);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom((uint8_t)mpuAddress, (size_t)count) != count) {
    while (Wire.available()) {
      Wire.read();
    }
    return false;
  }
  for (byte i = 0; i < count; i++) {
    if (!Wire.available()) {
      return false;
    }
    buffer[i] = Wire.read();
  }
  return true;
}

bool findMPU6050() {
  for (byte address = 0x68; address <= 0x69; address++) {
    mpuAddress = address;
    byte deviceId = 0;
    // WHO_AM_I stays 0x68 at either bus address.
    if (mpuReadMulti(REG_WHO_AM_I, &deviceId, 1) && deviceId == 0x68) {
      return true;
    }
  }
  return false;
}

byte directionFromHeading() {
  // Separate enter/release angles prevent chatter near FRONT.
  if (currentDirection == LEFT && headingDeg > FRONT_ANGLE_DEG) {
    return LEFT;
  }
  if (currentDirection == RIGHT && headingDeg < -FRONT_ANGLE_DEG) {
    return RIGHT;
  }
  if (headingDeg >= TURN_ANGLE_DEG) {
    return LEFT;
  }
  if (headingDeg <= -TURN_ANGLE_DEG) {
    return RIGHT;
  }
  return FRONT;
}

void printDirection(int16_t rawGyroZ) {
  if (PRINT_RAW_VALUES) {
    Serial.print(F("GZ="));
    Serial.print(rawGyroZ);
    Serial.print(F(" | heading="));
    Serial.print(headingDeg);
    Serial.print(F(" | "));
  }
  switch (currentDirection) {
    case LEFT: Serial.println(F("LEFT")); break;
    case RIGHT: Serial.println(F("RIGHT")); break;
    default: Serial.println(F("FRONT")); break;
  }
}

void trackingError(byte reason, uint32_t nowMs) {
  headingValid = false;
  if (!faultReported || nowMs - lastErrorMs >= 1000UL) {
    if (reason == 1) {
      Serial.println(F("ERROR: MPU-6050 I2C read failed; heading unavailable."));
    } else if (reason == 2) {
      Serial.println(F("ERROR: Sampling gap; heading unavailable."));
    } else {
      Serial.println(F("ERROR: Gyro range exceeded; heading unavailable."));
    }
    Serial.println(F("Check the sensor, then send R or r to set a new FRONT."));
    lastErrorMs = nowMs;
    faultReported = true;
  }
}

void setup() {

  Serial.begin(115200);
  ESP32_Serial.begin(115200);
  
  if (!Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK_HZ)) {
    Serial.printf("ERROR: ESP32-S3 I2C initialization failed. Check SDA pin=%d and SCL pin=%d.\n",
                  I2C_SDA_PIN, I2C_SCL_PIN);
    ESP32_Serial.printf("ERROR: ESP32-S3 I2C initialization failed. Check SDA pin=%d and SCL pin=%d.\n",
                  I2C_SDA_PIN, I2C_SCL_PIN);
    return;
  }
  Wire.setTimeOut(25);  // Arduino-ESP32 I2C timeout in milliseconds.

  if (!findMPU6050()) {
    Serial.printf("ERROR: MPU-6050 not found. Check power, SDA pin=%d, SCL pin=%d and AD0, then reset.\n",
                  I2C_SDA_PIN, I2C_SCL_PIN);
    return;
  }

  // +/-250 degrees/s gyro, 20 Hz low-pass filter, 100 Hz samples.
  // FIFO, DMP and interrupts are disabled.
  sensorReady = mpuWrite(REG_PWR_MGMT_1, 0x01)
             && mpuWrite(REG_PWR_MGMT_2, 0x00)
             && mpuWrite(REG_USER_CTRL, 0x00)
             && mpuWrite(REG_FIFO_EN, 0x00)
             && mpuWrite(REG_CONFIG, 0x04)
             && mpuWrite(REG_SMPLRT_DIV, 0x09)
             && mpuWrite(REG_GYRO_CONFIG, 0x00)
             && mpuWrite(REG_INT_ENABLE, 0x00);
  if (!sensorReady) {
    Serial.println(F("ERROR: MPU-6050 configuration failed. Check wiring, then reset."));
    return;
  }
  startedAt = millis();
  lastSampleMs = startedAt;
}

void loop() {
  if (!sensorReady) {
    return;
  }

  while (Serial.available()) {
    const char command = Serial.read();
    if (command == 'R' || command == 'r') {
      // Apply the new reference at the next valid sensor sample.
      resetRequested = true;
    }
  }

  const uint32_t nowMs = millis();
  if ((!headingValid && !resetRequested)
      || nowMs - startedAt < SENSOR_SETTLE_MS
      || nowMs - lastSampleMs < SAMPLE_INTERVAL_MS) {
    return;
  }
  lastSampleMs = nowMs;

  byte raw[2];
  if (!mpuReadMulti(REG_GYRO_ZOUT_H, raw, sizeof(raw))) {
    trackingError(1, nowMs);
    return;
  }
  const int16_t rawGyroZ = (int16_t)(((uint16_t)raw[0] << 8) | raw[1]);
  if (rawGyroZ >= 32760 || rawGyroZ <= -32760) {
    trackingError(3, nowMs);
    return;
  }

  float rateDps = rawGyroZ / GYRO_COUNTS_PER_DPS;
  if (REVERSE_LEFT_RIGHT) {
    rateDps = -rateDps;
  }
  const uint32_t nowUs = micros();

  if (resetRequested) {
    headingDeg = 0.0f;
    previousRateDps = rateDps;
    lastGyroUs = nowUs;
    lastPrintMs = nowMs;
    currentDirection = FRONT;
    headingValid = true;
    resetRequested = false;
    faultReported = false;
    printDirection(rawGyroZ);
    return;
  }

  const uint32_t elapsedUs = nowUs - lastGyroUs;
  if (elapsedUs > MAX_SAMPLE_GAP_US) {
    // Rotation during a long gap is unknown; require an explicit new reference.
    trackingError(2, nowMs);
    return;
  }
  lastGyroUs = nowUs;

  // Angle = integral of angular rate. Do not discard small rates:
  // a slow turn must still change heading.
  headingDeg += 0.5f * (previousRateDps + rateDps) * (elapsedUs * 0.000001f);
  previousRateDps = rateDps;
  if (headingDeg >= 180.0f) {
    headingDeg -= 360.0f;
  } else if (headingDeg < -180.0f) {
    headingDeg += 360.0f;
  }

  const byte direction = directionFromHeading();
  if (direction != currentDirection || nowMs - lastPrintMs >= PRINT_INTERVAL_MS) {
    currentDirection = direction;
    printDirection(rawGyroZ);
    lastPrintMs = nowMs;
  }
}
