#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

// Adapted from Test/MPU_ESP32-S3-DEV.ino, with the warm runtime recovery of
// MPU_and_Motor-Latest/CalibS3 (2026-11-11). Positive yaw is LEFT (counter-
// clockwise seen from above). This is relative heading only: the bridge keeps
// one continuous reference from boot, and sideways position is not measured.
//
// 2026-11-11 navigation review:
//   * Yaw rate is the calibrated gyro VECTOR projected on the gravity direction
//     measured at boot. A level sensor gives exactly gyro Z, as before; a tilted
//     mount no longer under-reads turns by cos(tilt).
//   * GYRO_SCALE corrects the MPU6050's +/-3% sensitivity tolerance (about
//     +/-2.7 degrees on a 90 degree turn). Measure it with the calibration
//     procedure in docs/navigation-2026-11-11.md. The current correction is based
//     on the owner's approximate 100-degree floor turn for 90 reported degrees.
//   * While the bridge holds motion stopped (setStill), the bias is measured
//     again every 1.5 s of genuinely still data. The heading reference is kept.
class MpuHeading {
 public:
  enum class Sample { NONE, REFERENCE, UPDATED, FAULT };
  static constexpr uint32_t SAMPLE_INTERVAL_MS = 10;
  static constexpr uint32_t MAX_SAMPLE_GAP_US = 100000;
  static constexpr float ENTER_DEG = 2.0f;
  static constexpr float RELEASE_DEG = 1.0f;
  // 2 s of fresh stationary samples. CalibS3 used 24 so its bench route could
  // start 500 ms after power-up; production holds STOP until BLE, the Pi and
  // fresh LiDAR are ready, so the longer, quieter bias estimate costs nothing.
  static constexpr uint16_t CALIBRATION_SAMPLES = 200;
  // A chassis cannot do this between two samples: treat it as a sensor fault.
  static constexpr float MAX_YAW_JUMP_DEG = 20.0f;
  // True angle / reported angle, multiplied into BOTH rate and integration.
  // Owner's estimate (2026-11-16): reported 90, floor about 100. Provisional:
  // 100/90 makes the old raw 81-degree endpoint report the desired 90 degrees.
  // Keep targets in real degrees; do not also reduce the executor's turn angle.
  static constexpr float GYRO_SCALE = 100.0f / 90.0f;

  // Stationary bias refresh. Samples must stay within the boot calibration's
  // 1.5 deg/s spread limit and under STILL_MAX_RATE_DPS of yaw, or the window restarts.
  static constexpr uint16_t STILL_SAMPLES = 150;
  static constexpr float STILL_MAX_RATE_DPS = 1.0f;
  // Real drift moves slowly. A larger change in one window is a slow push by hand.
  static constexpr float MAX_BIAS_SHIFT_DPS = 0.2f;
  // Total distance from the boot calibration, so repeated windows cannot ratchet.
  static constexpr float MAX_TOTAL_BIAS_DRIFT_DPS = 1.0f;

  // Retry timing for boot calibration if no bias has ever been measured.
  static constexpr uint32_t RECOVERY_SETTLE_MS = 1500;
  static constexpr uint32_t RECOVERY_RETRY_MS = 3000;
  static constexpr uint32_t MAX_WARM_REFERENCE_GAP_MS = 500;

  bool begin(int sdaPin, int sclPin, bool reverseZ = false) {
    ready_ = calibrating_ = false;
    biasReady_ = referenceEstablished_ = warmRecovery_ = warmConfigured_ = false;
    invalidate();
    reverseZ_ = reverseZ;
    sdaPin_ = sdaPin;
    sclPin_ = sclPin;
    // Keep the chassis still before the motor controller receives heading data.
    // A new reference must never recalibrate a moving robot.
    if (configureSensor()) {
      delay(100);
      startCalibration();
      CalibrationStep step;
      do {
        delay(SAMPLE_INTERVAL_MS);
        step = calibrationStep();
      } while (step == CalibrationStep::RUNNING);
      ready_ = step == CalibrationStep::DONE;
      biasReady_ = ready_;
    }
    lastSampleMs_ = (uint32_t)millis();
    // Calibration already collected fresh samples. Permit the next
    // sample after 10 ms, so an immediate reference request cannot expire
    // behind poll()'s 100 ms startup guard.
    startedAtMs_ = lastSampleMs_ - 100UL;
    nextRecoveryMs_ = lastSampleMs_ + RECOVERY_RETRY_MS;
    return ready_;
  }

  // A runtime fault happens while the motors are moving. Keep the bias measured
  // before motion; a new "stationary" bias measurement would include real yaw.
  void beginRecovery() {
    ready_ = calibrating_ = false;
    invalidate();
    problem_ = lastFault_;
    warmRecovery_ = biasReady_;
    warmConfigured_ = false;
    recoveryStartedMs_ = (uint32_t)millis();
    nextRecoveryMs_ = recoveryStartedMs_ +
        (warmRecovery_ ? SAMPLE_INTERVAL_MS : RECOVERY_SETTLE_MS);
  }

  // At runtime, reconfigure the MPU and reuse the stored bias and heading.
  // Boot recovery still needs calibration if no bias has ever been measured.
  bool serviceRecovery() {
    if (ready_) return false;
    const uint32_t nowMs = (uint32_t)millis();
    if (warmRecovery_) {
      if ((int32_t)(nowMs - nextRecoveryMs_) < 0) return false;
      if (!warmConfigured_) {
        Wire.end();
        if (!configureSensor()) {
          nextRecoveryMs_ = nowMs + RECOVERY_RETRY_MS;
          return false;
        }
        warmConfigured_ = true;
        nextRecoveryMs_ = nowMs + 100UL;
        return false;
      }
      uint8_t raw[15];
      if (!readRegisters(0x3a, raw, sizeof(raw))) {
        warmConfigured_ = false;
        nextRecoveryMs_ = nowMs + RECOVERY_RETRY_MS;
        return false;
      }
      if (!(raw[0] & 0x01)) {
        nextRecoveryMs_ = nowMs + SAMPLE_INTERVAL_MS;
        return false;
      }
      int16_t gyro[3];
      if (!decodeGyro(raw, gyro)) {
        warmConfigured_ = false;
        nextRecoveryMs_ = nowMs + RECOVERY_RETRY_MS;
        return false;
      }
      previousRateDps_ = yawRateDps(gyro);
      lastGyroUs_ = (uint32_t)micros();
      lastSampleMs_ = nowMs;
      // If the sensor was blind long enough for an unknown turn, reuse the
      // bias but establish a new heading reference at the current posture.
      if (nowMs - recoveryStartedMs_ > MAX_WARM_REFERENCE_GAP_MS)
        referenceEstablished_ = false;
      valid_ = referenceEstablished_;
      referencePending_ = false;
      ready_ = true;
      warmRecovery_ = warmConfigured_ = false;
      problem_ = "OK";
      return true;
    }
    if (!calibrating_) {
      if ((int32_t)(nowMs - nextRecoveryMs_) < 0) return false;
      Wire.end();
      if (!configureSensor()) {
        nextRecoveryMs_ = nowMs + RECOVERY_RETRY_MS;
        return false;
      }
      startCalibration();
      calibrating_ = true;
      lastSampleMs_ = nowMs + 100UL; // the sensor's first samples follow its new configuration
      return false;
    }
    if ((int32_t)(nowMs - lastSampleMs_) < (int32_t)SAMPLE_INTERVAL_MS) return false;
    lastSampleMs_ = nowMs;
    const CalibrationStep step = calibrationStep();
    if (step == CalibrationStep::RUNNING) return false;
    calibrating_ = false;
    if (step == CalibrationStep::FAILED) {
      nextRecoveryMs_ = nowMs + RECOVERY_RETRY_MS;
      return false;
    }
    ready_ = true;
    biasReady_ = true;
    startedAtMs_ = nowMs - 100UL;
    return true;
  }

  void requestReference() {
    valid_ = false;
    referenceEstablished_ = false;
    referencePending_ = true;
    referenceRequestedUs_ = (uint32_t)micros();
    direction_ = 0;
    previousRateDps_ = 0;
  }

  void invalidate() {
    valid_ = false;
    referencePending_ = false;
    direction_ = 0;
    previousRateDps_ = 0;
    resetStillWindow();
    // referenceEstablished_ survives a brief sensor fault; recovery decides
    // whether to keep that heading or request a new reference.
  }

  // True while the caller knows the wheels are held stopped. Data from that time
  // re-measures the bias; anything that looks like rotation restarts the window.
  void setStill(bool still) {
    if (still == still_) return;
    still_ = still;
    resetStillWindow();
  }

  Sample poll() {
    const uint32_t nowMs = (uint32_t)millis();
    if (!ready_ || (!valid_ && !referencePending_)
        || nowMs - startedAtMs_ < 100UL
        || nowMs - lastSampleMs_ < SAMPLE_INTERVAL_MS) return Sample::NONE;
    lastSampleMs_ = nowMs;
    // Read DATA_RDY and the sensor block together. Polling successfully over
    // I2C does not prove that a sleeping/frozen gyro produced a fresh sample.
    uint8_t raw[15];
    if (!readRegisters(0x3a, raw, sizeof(raw))) return missedSample("READ");
    if (!(raw[0] & 0x01)) return missedSample("NOT_FRESH");
    int16_t gyro[3];
    if (!decodeGyro(raw, gyro)) return missedSample("SATURATED");
    const float rateDps = yawRateDps(gyro);
    const uint32_t nowUs = (uint32_t)micros();
    if (referencePending_) {
      headingDeg_ = 0;
      previousRateDps_ = rateDps;
      lastGyroUs_ = nowUs;
      direction_ = 0;
      valid_ = true;
      referenceEstablished_ = true;
      referencePending_ = false;
      return Sample::REFERENCE;
    }
    const uint32_t elapsedUs = nowUs - lastGyroUs_;
    if (elapsedUs > MAX_SAMPLE_GAP_US) return fault("GAP");
    // Keep small rates: slow rotation must still change the heading.
    const float deltaDeg = 0.5f * (previousRateDps_ + rateDps) * (elapsedUs * 0.000001f);
    if (!isfinite(deltaDeg) || fabsf(deltaDeg) > MAX_YAW_JUMP_DEG) return fault("JUMP");
    lastGyroUs_ = nowUs;
    headingDeg_ += deltaDeg;
    previousRateDps_ = rateDps;
    if (headingDeg_ >= 180.0f) headingDeg_ -= 360.0f;
    else if (headingDeg_ < -180.0f) headingDeg_ += 360.0f;
    if (still_) stillSample(gyro, rateDps);
    if (direction_ == 1 && headingDeg_ > RELEASE_DEG) return Sample::UPDATED;
    if (direction_ == -1 && headingDeg_ < -RELEASE_DEG) return Sample::UPDATED;
    direction_ = headingDeg_ >= ENTER_DEG ? 1 : (headingDeg_ <= -ENTER_DEG ? -1 : 0);
    return Sample::UPDATED;
  }

  bool valid() const {
    return valid_ && (uint32_t)((uint32_t)micros() - lastGyroUs_) <= MAX_SAMPLE_GAP_US;
  }
  bool referencePending() const { return referencePending_; }
  // True only in the bounded window while a requested reference waits for its
  // first fresh sample. A sensor that never delivers one becomes a fault.
  bool referenceSettling() const {
    return referencePending_ && (uint32_t)((uint32_t)micros() - referenceRequestedUs_) <= MAX_SAMPLE_GAP_US;
  }
  bool ready() const { return ready_; }
  // Why the MPU is not ready, or "OK". Reported to the Pi: nobody sees the console on battery.
  const char* problem() const { return problem_; }
  // Which check ended the last tracking, or "NONE".
  const char* lastFault() const { return lastFault_; }
  // Bias along the measured up axis, deg/s (the part that becomes yaw drift).
  float biasDps() const {
    return (bias_[0] * up_[0] + bias_[1] * up_[1] + bias_[2] * up_[2]) * GYRO_SCALE / LSB_PER_DPS;
  }
  // Angle between the sensor's +Z axis and measured up, degrees.
  float tiltDeg() const { return acosf(constrain(up_[2], -1.0f, 1.0f)) * 57.2957795f; }
  uint32_t biasRefreshes() const { return biasRefreshes_; }
  float headingDeg() const { return headingDeg_; }
  // Same calibrated, mounting-corrected sample used by yaw integration. A
  // caller must not receive an old rate after the sampling deadline expires.
  float rateDps() const { return valid() ? previousRateDps_ : 0.0f; }
  int direction() const { return direction_; }

 private:
  enum class CalibrationStep { RUNNING, DONE, FAILED };
  static constexpr float LSB_PER_DPS = 131.0f;

  uint8_t address_ = 0x68;
  bool ready_ = false, valid_ = false, referencePending_ = false, reverseZ_ = false;
  bool calibrating_ = false;
  bool biasReady_ = false, referenceEstablished_ = false;
  bool warmRecovery_ = false, warmConfigured_ = false;
  bool still_ = false;
  int direction_ = 0;
  int sdaPin_ = -1, sclPin_ = -1;
  float headingDeg_ = 0, previousRateDps_ = 0;
  float bias_[3] = {0.0f, 0.0f, 0.0f};   // raw gyro counts
  float bootBias_[3] = {0.0f, 0.0f, 0.0f};  // the last stationary calibration
  float up_[3] = {0.0f, 0.0f, 1.0f};     // unit gravity direction in sensor axes
  uint32_t startedAtMs_ = 0, lastSampleMs_ = 0, lastGyroUs_ = 0, referenceRequestedUs_ = 0;
  uint32_t nextRecoveryMs_ = 0, recoveryStartedMs_ = 0;
  const char* problem_ = "NOT_STARTED";
  const char* lastFault_ = "NONE";
  int32_t calibrationSum_[6] = {};
  int16_t calibrationLow_[6] = {}, calibrationHigh_[6] = {};
  uint16_t calibrationCount_ = 0;
  uint8_t calibrationStale_ = 0;
  int32_t stillSum_[3] = {};
  int16_t stillLow_[3] = {}, stillHigh_[3] = {};
  uint16_t stillCount_ = 0;
  uint32_t biasRefreshes_ = 0;

  Sample fault(const char* reason) { lastFault_ = reason; invalidate(); return Sample::FAULT; }
  // Stepper EMI can corrupt a single I2C transaction during a long straight run.
  // Skipping that sample is enough: the next good one integrates across the gap,
  // and the 100 ms blind budget still ends in a fault if the data stays bad.
  Sample missedSample(const char* reason) {
    const uint32_t sinceUs = (uint32_t)micros() - (valid_ ? lastGyroUs_ : referenceRequestedUs_);
    return (valid_ || referencePending_) && sinceUs > MAX_SAMPLE_GAP_US ? fault(reason) : Sample::NONE;
  }

  static int16_t word(const uint8_t* raw, uint8_t offset) {
    return (int16_t)(((uint16_t)raw[offset] << 8) | raw[offset + 1]);
  }
  // Gyro X/Y/Z from the 0x3a block. Any saturated axis makes the sample unusable:
  // its projection on up would be wrong as well.
  static bool decodeGyro(const uint8_t* raw, int16_t* gyro) {
    for (uint8_t axis = 0; axis < 3; ++axis) {
      gyro[axis] = word(raw, 9 + axis * 2);
      if (gyro[axis] >= 32760 || gyro[axis] <= -32760) return false;
    }
    return true;
  }
  float yawRateDps(const int16_t* gyro) const {
    float rate = 0.0f;
    for (uint8_t axis = 0; axis < 3; ++axis) rate += ((float)gyro[axis] - bias_[axis]) * up_[axis];
    rate *= GYRO_SCALE / LSB_PER_DPS;
    return reverseZ_ ? -rate : rate;
  }

  void resetStillWindow() {
    stillCount_ = 0;
    for (uint8_t axis = 0; axis < 3; ++axis) {
      stillSum_[axis] = 0;
      stillLow_[axis] = 32767;
      stillHigh_[axis] = -32768;
    }
  }
  void stillSample(const int16_t* gyro, float rateDps) {
    if (fabsf(rateDps) > STILL_MAX_RATE_DPS) {
      resetStillWindow();
      return;
    }
    for (uint8_t axis = 0; axis < 3; ++axis) {
      stillSum_[axis] += gyro[axis];
      if (gyro[axis] < stillLow_[axis]) stillLow_[axis] = gyro[axis];
      if (gyro[axis] > stillHigh_[axis]) stillHigh_[axis] = gyro[axis];
      if ((int)stillHigh_[axis] - (int)stillLow_[axis] > 197) {  // 1.5 deg/s, as at boot
        resetStillWindow();
        return;
      }
    }
    if (++stillCount_ < STILL_SAMPLES) return;
    float candidate[3];
    for (uint8_t axis = 0; axis < 3; ++axis) {
      candidate[axis] = (float)stillSum_[axis] / (float)STILL_SAMPLES;
      // Keep the physical drift budgets at 0.2/window and 1.0 from boot after
      // scaling; increasing GYRO_SCALE must not increase the allowed yaw drift.
      if (fabsf(candidate[axis] - bias_[axis]) > MAX_BIAS_SHIFT_DPS * LSB_PER_DPS / GYRO_SCALE ||
          fabsf(candidate[axis] - bootBias_[axis]) > MAX_TOTAL_BIAS_DRIFT_DPS * LSB_PER_DPS / GYRO_SCALE) {
        resetStillWindow();  // a jump this size is not drift; keep the old bias
        return;
      }
    }
    for (uint8_t axis = 0; axis < 3; ++axis) bias_[axis] = candidate[axis];
    ++biasRefreshes_;
    resetStillWindow();
  }

  bool configureSensor() {
    if (!Wire.begin(sdaPin_, sclPin_, 100000UL)) { problem_ = "BUS"; return false; }
    Wire.setTimeOut(25);
    bool found = false;
    for (uint8_t address = 0x68; address <= 0x69; ++address) {
      address_ = address;
      uint8_t id = 0;
      if (readRegisters(0x75, &id, 1) && id == 0x68) { found = true; break; }
    }
    if (!found) { problem_ = "NOT_FOUND"; return false; }
    // +/-250 degrees/s, 20 Hz low-pass, 100 Hz. No FIFO/DMP/interrupts.
    const bool configured =
        writeRegister(0x6b, 0x01) && writeRegister(0x6c, 0x00)
        && writeRegister(0x6a, 0x00) && writeRegister(0x23, 0x00)
        && writeRegister(0x1a, 0x04) && writeRegister(0x19, 0x09)
        && writeRegister(0x1b, 0x00) && writeRegister(0x1c, 0x00)
        && writeRegister(0x38, 0x01); // enable DATA_RDY status; INT wire unused
    if (!configured) problem_ = "SETUP";
    return configured;
  }
  void startCalibration() {
    calibrationCount_ = 0;
    calibrationStale_ = 0;
    for (uint8_t axis = 0; axis < 6; ++axis) {
      calibrationSum_[axis] = 0;
      calibrationLow_[axis] = 32767;
      calibrationHigh_[axis] = -32768;
    }
  }
  CalibrationStep calibrationFailed(const char* reason) {
    problem_ = reason;
    return CalibrationStep::FAILED;
  }
  // One sensor read. Boot and recovery share it, so recovery cannot be more lenient.
  CalibrationStep calibrationStep() {
    uint8_t raw[15];
    if (!readRegisters(0x3a, raw, sizeof(raw))) return calibrationFailed("READ");
    if (!(raw[0] & 0x01)) {
      return ++calibrationStale_ >= 5 ? calibrationFailed("NOT_FRESH") : CalibrationStep::RUNNING;
    }
    calibrationStale_ = 0;
    int16_t value[6];
    for (uint8_t axis = 0; axis < 6; ++axis) {
      const uint8_t offset = axis < 3 ? axis * 2 + 1 : axis * 2 + 3;
      value[axis] = word(raw, offset);
      if (value[axis] >= 32760 || value[axis] <= -32760) return calibrationFailed("SATURATED");
      if (value[axis] < calibrationLow_[axis]) calibrationLow_[axis] = value[axis];
      if (value[axis] > calibrationHigh_[axis]) calibrationHigh_[axis] = value[axis];
      // Initial zero-rate offset can be +/-20 deg/s per MPU6050 datasheet.
      if (axis >= 3 && (value[axis] > 3275 || value[axis] < -3275)) return calibrationFailed("GYRO_OFFSET");
    }
    const float ax = value[0] / 16384.0f, ay = value[1] / 16384.0f;
    const float az = value[2] / 16384.0f;
    const float gravitySq = ax*ax + ay*ay + az*az;
    if (gravitySq < 0.64f || gravitySq > 1.44f || az < 0.8f) return calibrationFailed("GRAVITY");
    for (uint8_t axis = 0; axis < 6; ++axis) calibrationSum_[axis] += value[axis];
    if (++calibrationCount_ < CALIBRATION_SAMPLES) return CalibrationStep::RUNNING;
    for (uint8_t axis = 0; axis < 6; ++axis) {
      const int spread = (int)calibrationHigh_[axis] - (int)calibrationLow_[axis];
      if (spread > (axis < 3 ? 2458 : 197)) return calibrationFailed("MOVING"); // .15g / 1.5 deg/s
    }
    float up[3], norm = 0.0f;
    for (uint8_t axis = 0; axis < 3; ++axis) {
      up[axis] = (float)calibrationSum_[axis] / (float)CALIBRATION_SAMPLES;
      norm += up[axis] * up[axis];
    }
    norm = sqrtf(norm);
    for (uint8_t axis = 0; axis < 3; ++axis) {
      up_[axis] = up[axis] / norm;
      bias_[axis] = bootBias_[axis] = (float)calibrationSum_[axis + 3] / (float)CALIBRATION_SAMPLES;
    }
    problem_ = "OK";
    return CalibrationStep::DONE;
  }
  bool writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(address_);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
  }
  bool readRegisters(uint8_t reg, uint8_t* data, uint8_t count) {
    Wire.beginTransmission(address_);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((uint8_t)address_, (size_t)count) != count) {
      while (Wire.available()) Wire.read();
      return false;
    }
    for (uint8_t i = 0; i < count; ++i) {
      if (!Wire.available()) return false;
      data[i] = (uint8_t)Wire.read();
    }
    return true;
  }
};
