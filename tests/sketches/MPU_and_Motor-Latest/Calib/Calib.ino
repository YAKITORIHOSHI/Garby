#include <Arduino.h>
#include <FastAccelStepper.h>
#include <HardwareSerial.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#if !defined(ARDUINO_ARCH_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Build Calib.ino for the main ESP32 motor controller, not ESP32-S3."
#endif

// ESP32 motor controller. The separate S3 sketch sends calibrated yaw/rate.
// Pins follow GARBY's documented wiring; verify the actual connections.
static constexpr int STEP_PIN1 = 21;
static constexpr int DIR_PIN1 = 22;
static constexpr int STEP_PIN2 = 13;
static constexpr int DIR_PIN2 = 12;
static constexpr int UART_RX_PIN = 18;  // from S3 TX17
static constexpr int UART_TX_PIN = 19;  // to S3 RX18
static constexpr uint32_t BASE_SPEED_HZ = 8000;
static constexpr int32_t ACCELERATION = 8000;
static constexpr uint32_t LINK_TIMEOUT_MS = 250;
static constexpr uint32_t CONTROL_INTERVAL_MS = 20;
static constexpr uint8_t STARTUP_SAMPLES = 10;
static constexpr uint32_t STARTUP_CONFIRM_MS = 180;
static constexpr uint32_t DRIVE_SETTLE_MS = 1200;
static constexpr uint32_t BASELINE_MS = 350;
static constexpr uint32_t PROBE_MS = 400;
static constexpr float FIRST_PROBE_CUT = 0.03f;
static constexpr float SECOND_PROBE_CUT = 0.08f;
static constexpr float MIN_PROBE_RESPONSE_DPS = 0.8f;
static constexpr float MAX_SPEED_CUT = 0.30f;
static constexpr float MAX_CUT_CHANGE = 0.015f;  // per 20 ms; within 8000 steps/s^2
static constexpr float MAX_TRIM_CUT = 0.10f;
static constexpr float HEADING_GAIN_PER_S = 2.5f;
static constexpr float LOOKAHEAD_S = 0.28f;
static constexpr float TRIM_GAIN_PER_S2 = 0.15f;
static constexpr float RATE_FILTER_MS = 80.0f;
// Cross-track is stored in equivalent wheel STEP pulses, so no wheel
// diameter is needed. At 8000 steps/s this is a 2.5-second lookahead.
static constexpr float PATH_LOOKAHEAD_STEPS = 20000.0f;
static constexpr float MAX_CROSS_TRACK_STEPS = 40000.0f;
static constexpr float MAX_RETURN_HEADING_DEG = 12.0f;

// ---- Route: straight segments and MPU-measured turns ----
// Straights longer than this cruise with heading hold, then hand the last
// FINISH_STEPS to the stepper ramp so the step count ends exactly.
// 4000 steps stop 8000 steps/s at 8000 steps/s^2; 400 steps are margin.
static constexpr int32_t FINISH_STEPS =
    static_cast<int32_t>(static_cast<uint64_t>(BASE_SPEED_HZ) * BASE_SPEED_HZ /
                         (2ULL * ACCELERATION)) + BASE_SPEED_HZ / 20;
// The steering probe needs this much straight cruise left before it starts.
static constexpr int32_t PROBE_RESERVE_STEPS =
    FINISH_STEPS + static_cast<int32_t>(
        BASE_SPEED_HZ * (BASELINE_MS + PROBE_MS + 300) / 1000);

// Guess only. The first MPU turn checks it and corrects it if the chassis
// rotates the wrong way. An MPU-less fallback turn relies on it.
static constexpr bool MOTOR1_IS_LEFT_GUESS = true;
// The old open-loop turnLeft(4900) was a 90-degree turn. This is only the
// starting estimate; each MPU turn measures and refines it.
static constexpr float DEFAULT_STEPS_PER_DEG = 4900.0f / 90.0f;
// Spin speeds stay well under the MPU's +/-250 deg/s gyro range:
// 3000 steps/s is about 55 deg/s at 54 steps/degree.
static constexpr uint32_t TURN_MAX_SPEED_HZ = 3000;
static constexpr uint32_t TURN_CORRECTION_SPEED_HZ = 800;
static constexpr uint32_t TURN_MIN_SPEED_HZ = 300;
// Plan the slow-down with 60% of the real acceleration, leaving margin
// for the 20 ms sample interval.
static constexpr float TURN_PROFILE_DECEL_FRACTION = 0.6f;
static constexpr float TURN_STOP_DEG = 0.3f;       // command stop this close
static constexpr float TURN_TOLERANCE_DEG = 1.0f;  // accept final heading
static constexpr float TURN_WRONG_WAY_DEG = 3.0f;
// Stop a pass that needs 1.5x the expected steps: the wheels are slipping
// or lifted and the chassis is not rotating.
static constexpr float TURN_STEP_LIMIT_FACTOR = 1.5f;
static constexpr uint8_t TURN_MAX_PASSES = 5;
static constexpr uint32_t TURN_SETTLE_MS = 250;
static constexpr uint32_t TURN_DATA_TIMEOUT_MS = 100;  // 5 missed samples
static constexpr uint32_t MPU_WAIT_MS = 1500;
static constexpr uint32_t START_MPU_WAIT_MS = 3000;
static constexpr float S3_SAMPLE_LATENCY_S = 0.01f;

enum class ProbeState : uint8_t { WAITING, BASELINE, PULSE, READY, FAILED };
// CRUISE: straight with steering. FINISH: stepper ramp ends the straight.
enum class DriveMode : uint8_t { IDLE, CRUISE, FINISH, TURN };
enum class SpinResult : uint8_t { REACHED, WRONG_WAY, DATA_LOST, STEP_LIMIT };

FastAccelStepperEngine engine;
FastAccelStepper* motor1 = nullptr;
FastAccelStepper* motor2 = nullptr;
HardwareSerial mpuLink(1);

static bool haveSample = false;
static uint32_t lastSequence = 0;
static uint32_t lastEpoch = 0;
static uint32_t lastSampleMs = 0;
static uint32_t firstSampleMs = 0;
static uint8_t startupSamples = 0;
static uint32_t lastControlMs = 0;
static uint32_t lastPathMs = 0;
static uint32_t driveStartedMs = 0;
static uint32_t probeStartedMs = 0;
static uint32_t probeLastSequence = 0;
// Unwrapped heading in the route frame: 0 = facing the way the robot faced
// at power-up, + = left. It keeps counting past +/-180 degrees.
static float continuousHeadingDeg = 0.0f;
static float lastRawHeadingDeg = 0.0f;
static bool headingAnchored = false;
static int32_t samplePos1 = 0;  // motor positions at the last MPU sample
static int32_t samplePos2 = 0;
static float routeTargetHeadingDeg = 0.0f;
static float stepsPerDeg = DEFAULT_STEPS_PER_DEG;
static bool motor1IsLeft = MOTOR1_IS_LEFT_GUESS;
static bool turnSideConfirmed = false;
static DriveMode driveMode = DriveMode::IDLE;
static int32_t cruiseRemainingSteps = 0;
static bool routeFinished = false;
static float latestRateDps = 0.0f;
static float filteredRateDps = 0.0f;
static float crossTrackSteps = 0.0f;
static float appliedCut = 0.0f;  // signed: + slows sensor-negative wheel
static float yawGainDpsPerCut = 0.0f;
static float trimDps = 0.0f;
static float baselineRateDps = 0.0f;
static float baselineStdErrorSq = 0.0f;
static float activeProbeCut = FIRST_PROBE_CUT;
static float rateSum = 0.0f;
static float rateSumSquares = 0.0f;
static uint16_t rateCount = 0;
static uint8_t probeAttempts = 0;
static bool pathReady = false;
static bool motor1PositiveYaw = false;
static ProbeState probeState = ProbeState::WAITING;
static uint32_t appliedSpeed1 = BASE_SPEED_HZ;
static uint32_t appliedSpeed2 = BASE_SPEED_HZ;
static char rxLine[80];
static uint8_t rxLength = 0;
static bool discardLine = false;
static uint32_t acceptedPackets = 0;  // for the USB serial report only
static uint32_t rejectedPackets = 0;

void runStart();

static bool setMotorSpeed(FastAccelStepper* motor, uint32_t speed) {
  if (motor->setSpeedInHz(speed) != 0) return false;
  motor->applySpeedAcceleration();
  return true;
}

static float wrap180(float deg) {
  while (deg > 180.0f) deg -= 360.0f;
  while (deg < -180.0f) deg += 360.0f;
  return deg;
}

static FastAccelStepper* leftMotor() { return motor1IsLeft ? motor1 : motor2; }
static FastAccelStepper* rightMotor() { return motor1IsLeft ? motor2 : motor1; }

// Chassis rotation implied by wheel STEP travel (+ = left). Slip makes it
// approximate; it only bridges the gaps between MPU samples.
static float wheelYawDeg(int32_t delta1, int32_t delta2) {
  const float left = static_cast<float>(motor1IsLeft ? delta1 : delta2);
  const float right = static_cast<float>(motor1IsLeft ? delta2 : delta1);
  return 0.5f * (right - left) / stepsPerDeg;
}

// Last MPU heading plus wheel rotation since that sample. Without MPU data
// this is wheel dead reckoning from power-up.
static float estimatedHeadingDeg() {
  if (!motor1 || !motor2) return continuousHeadingDeg;
  return continuousHeadingDeg +
         wheelYawDeg(motor1->getCurrentPosition() - samplePos1,
                     motor2->getCurrentPosition() - samplePos2);
}

static float segmentHeadingErrorDeg() {
  return continuousHeadingDeg - routeTargetHeadingDeg;
}

static void resetSteering() {
  haveSample = false;
  startupSamples = 0;
  filteredRateDps = 0.0f;
  latestRateDps = 0.0f;
  appliedCut = 0.0f;
  crossTrackSteps = 0.0f;
  pathReady = false;
  if (probeState != ProbeState::READY && probeState != ProbeState::FAILED) {
    probeState = ProbeState::WAITING;
    rateCount = 0;
  }
  // MPU/UART faults remove straight-line correction, not the drive itself.
  // Turns and the final ramp own their motor speeds; leave those alone.
  if (driveMode != DriveMode::CRUISE) return;
  if (motor1 && appliedSpeed1 != BASE_SPEED_HZ &&
      setMotorSpeed(motor1, BASE_SPEED_HZ)) appliedSpeed1 = BASE_SPEED_HZ;
  if (motor2 && appliedSpeed2 != BASE_SPEED_HZ &&
      setMotorSpeed(motor2, BASE_SPEED_HZ)) appliedSpeed2 = BASE_SPEED_HZ;
}

static bool parseUnsigned(char** field, char terminator, unsigned long* value) {
  if (**field < '0' || **field > '9') return false;
  char* end = nullptr;
  errno = 0;
  *value = strtoul(*field, &end, 10);
  if (errno == ERANGE || *end != terminator) return false;
  *field = end + 1;
  return true;
}

static bool parseSigned(char** field, char terminator, long* value) {
  const char* digit = **field == '-' ? *field + 1 : *field;
  if (*digit < '0' || *digit > '9') return false;
  char* end = nullptr;
  errno = 0;
  *value = strtol(*field, &end, 10);
  if (errno == ERANGE || *end != terminator) return false;
  *field = end + 1;
  return true;
}

// "M,<sequence>,<heading cdeg>,<rate cdeg/s>,<reference epoch>*<xor hex>"
static bool decodeSample(char* line, uint32_t* sequence, float* heading,
                         float* rate, uint32_t* epoch) {
  char* star = strchr(line, '*');
  if (!star || strlen(star) != 3) return false;
  auto hexDigit = [](char c) -> int {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  };
  const int high = hexDigit(star[1]);
  const int low = hexDigit(star[2]);
  if (high < 0 || low < 0) return false;
  const uint8_t sentCheck = static_cast<uint8_t>((high << 4) | low);
  uint8_t check = 0;
  for (char* p = line; p < star; ++p) check ^= static_cast<uint8_t>(*p);
  if (check != sentCheck) return false;
  *star = '\0';
  if (line[0] != 'M' || line[1] != ',') return false;
  char* field = line + 2;
  unsigned long number = 0, referenceEpoch = 0;
  long headingCdeg = 0, rateCdps = 0;
  if (!parseUnsigned(&field, ',', &number) ||
      !parseSigned(&field, ',', &headingCdeg) ||
      !parseSigned(&field, ',', &rateCdps) ||
      !parseUnsigned(&field, '\0', &referenceEpoch)) return false;
  if (number == 0 || referenceEpoch == 0 ||
      headingCdeg < -18000 || headingCdeg > 18000 ||
      rateCdps < -25000 || rateCdps > 25000) return false;
  *sequence = static_cast<uint32_t>(number);
  *heading = headingCdeg * 0.01f;
  *rate = rateCdps * 0.01f;
  *epoch = static_cast<uint32_t>(referenceEpoch);
  return true;
}

static void handleLine() {
  rxLine[rxLength] = '\0';
  if (strcmp(rxLine, "F") == 0) {
    resetSteering();
    return;
  }
  uint32_t sequence = 0, epoch = 0;
  float heading = 0.0f, rate = 0.0f;
  // Ignore one damaged/duplicate packet; the freshness timeout handles a
  // broken link without repeatedly erasing a valid steering calibration.
  if (!decodeSample(rxLine, &sequence, &heading, &rate, &epoch)) {
    ++rejectedPackets;
    return;
  }
  if (haveSample && static_cast<int32_t>(sequence - lastSequence) <= 0) return;
  ++acceptedPackets;
  const uint32_t now = millis();
  if (haveSample && now - lastSampleMs > LINK_TIMEOUT_MS) resetSteering();
  // Same S3 heading reference: add its change, even across a short outage.
  // A new reference (S3 recovery after >500 ms, or S3 reboot, which also
  // restarts the sequence) reads 0 at an unknown chassis angle, so continue
  // from the last heading plus the wheel rotation since then.
  const bool sameReference = headingAnchored && epoch == lastEpoch &&
      static_cast<int32_t>(sequence - lastSequence) > 0;
  if (sameReference) {
    continuousHeadingDeg += wrap180(heading - lastRawHeadingDeg);
  } else {
    continuousHeadingDeg = estimatedHeadingDeg();
  }
  headingAnchored = true;
  lastRawHeadingDeg = heading;
  lastEpoch = epoch;
  if (motor1 && motor2) {
    samplePos1 = motor1->getCurrentPosition();
    samplePos2 = motor2->getCurrentPosition();
  }
  if (!haveSample) {
    firstSampleMs = now;
    startupSamples = 0;
    filteredRateDps = rate;
  } else {
    const float dtMs = static_cast<float>(now - lastSampleMs);
    const float alpha = dtMs / (RATE_FILTER_MS + dtMs);
    filteredRateDps += alpha * (rate - filteredRateDps);
  }
  if (startupSamples < STARTUP_SAMPLES) ++startupSamples;
  lastSequence = sequence;
  latestRateDps = rate;
  lastSampleMs = now;
  haveSample = true;
}

static void pollMpuLink() {
  uint16_t budget = 256;
  while (budget-- && mpuLink.available()) {
    const char c = static_cast<char>(mpuLink.read());
    if (c == '\n') {
      if (!discardLine && rxLength > 0) handleLine();
      discardLine = false;
      rxLength = 0;
    } else if (discardLine) {
      continue;
    } else if (c == '\r') {
      continue;
    } else if (c < 32 || c > 126 || rxLength >= sizeof(rxLine) - 1) {
      discardLine = true;
      rxLength = 0;
    } else {
      rxLine[rxLength++] = c;
    }
  }
}

static bool sampleStreamReady(uint32_t now) {
  return haveSample && now - lastSampleMs <= LINK_TIMEOUT_MS &&
         startupSamples >= STARTUP_SAMPLES &&
         now - firstSampleMs >= STARTUP_CONFIRM_MS;
}

static void servicePathEstimate(uint32_t now) {
  if (now - lastPathMs < CONTROL_INTERVAL_MS) return;
  const uint32_t elapsedMs = now - lastPathMs;
  lastPathMs = now;
  if (!motor1 || !motor2 || !haveSample ||
      now - lastSampleMs > LINK_TIMEOUT_MS || elapsedMs > 60) {
    pathReady = false;
    crossTrackSteps = 0.0f;
    return;
  }
  if (!pathReady) {
    pathReady = true;
    crossTrackSteps = 0.0f;
    return;
  }
  // FastAccelStepper reports ramp-generator speed, not wheel encoder data.
  // Integrating forward travel * sin(yaw) estimates displacement from this
  // segment's line; gyro-only heading hold would stop nudging while off-line.
  const float speed1 = fabsf(static_cast<float>(
      motor1->getCurrentSpeedInMilliHz(false))) * 0.001f;
  const float speed2 = fabsf(static_cast<float>(
      motor2->getCurrentSpeedInMilliHz(false))) * 0.001f;
  const float forwardSteps = 0.5f * (speed1 + speed2) *
                             (elapsedMs * 0.001f);
  crossTrackSteps += forwardSteps * sinf(segmentHeadingErrorDeg() * DEG_TO_RAD);
  crossTrackSteps = constrain(crossTrackSteps, -MAX_CROSS_TRACK_STEPS,
                             MAX_CROSS_TRACK_STEPS);
}

static void clearRateAverage(uint32_t now) {
  probeStartedMs = now;
  probeLastSequence = lastSequence;
  rateSum = 0.0f;
  rateSumSquares = 0.0f;
  rateCount = 0;
}

// A straight is ending before the probe finished: discard this attempt.
static void abortProbe() {
  if (probeState != ProbeState::BASELINE && probeState != ProbeState::PULSE)
    return;
  if (probeState == ProbeState::PULSE && probeAttempts > 0) --probeAttempts;
  probeState = ProbeState::WAITING;
  clearRateAverage(millis());
}

static void serviceProbe(uint32_t now) {
  if (probeState == ProbeState::READY || probeState == ProbeState::FAILED ||
      !motor1 || !motor2 || !motor1->isRunning() ||
      !motor2->isRunning() || !sampleStreamReady(now)) return;
  if (probeState == ProbeState::WAITING) {
    if (now - driveStartedMs < DRIVE_SETTLE_MS ||
        now - probeStartedMs < 800 ||
        cruiseRemainingSteps < PROBE_RESERVE_STEPS) return;
    probeState = ProbeState::BASELINE;
    clearRateAverage(now);
    return;
  }
  if (lastSequence != probeLastSequence) {
    probeLastSequence = lastSequence;
    // Ignore the first part of each window while motor speed and MPU filter settle.
    const uint32_t settleMs = probeState == ProbeState::PULSE ? 150 : 100;
    if (now - probeStartedMs >= settleMs) {
      rateSum += filteredRateDps;
      rateSumSquares += filteredRateDps * filteredRateDps;
      ++rateCount;
    }
  }
  if (probeState == ProbeState::BASELINE &&
      now - probeStartedMs >= BASELINE_MS) {
    if (rateCount < 8) {
      probeState = ProbeState::WAITING;
      return;
    }
    baselineRateDps = rateSum / rateCount;
    const float variance = fmaxf(0.0f,
        rateSumSquares / rateCount - baselineRateDps * baselineRateDps);
    baselineStdErrorSq = variance / rateCount;
    activeProbeCut = probeAttempts == 0 ? FIRST_PROBE_CUT : SECOND_PROBE_CUT;
    const uint32_t probeSpeed = static_cast<uint32_t>(
        lroundf(BASE_SPEED_HZ * (1.0f - activeProbeCut)));
    if (!setMotorSpeed(motor1, probeSpeed)) {
      probeState = ProbeState::FAILED;
      return;
    }
    appliedSpeed1 = probeSpeed;
    ++probeAttempts;
    probeState = ProbeState::PULSE;
    clearRateAverage(now);
  } else if (probeState == ProbeState::PULSE &&
             now - probeStartedMs >= PROBE_MS) {
    if (!setMotorSpeed(motor1, BASE_SPEED_HZ)) {
      probeState = ProbeState::FAILED;
      return;
    }
    appliedSpeed1 = BASE_SPEED_HZ;
    const float probeRate = rateCount ? rateSum / rateCount : 0.0f;
    const float variance = rateCount ? fmaxf(0.0f,
        rateSumSquares / rateCount - probeRate * probeRate) : 0.0f;
    const float standardError = rateCount ?
        sqrtf(baselineStdErrorSq + variance / rateCount) : 1000.0f;
    const float response = probeRate - baselineRateDps;
    const float measuredGain = fabsf(response) / activeProbeCut;
    if (rateCount >= 8 &&
        fabsf(response) >= fmaxf(MIN_PROBE_RESPONSE_DPS, 4.0f * standardError) &&
        measuredGain >= 10.0f && measuredGain <= 500.0f) {
      // Slowing motor 1 made positive sensor yaw iff it is the sensor-left wheel.
      motor1PositiveYaw = response > 0.0f;
      if (!turnSideConfirmed) motor1IsLeft = motor1PositiveYaw;
      yawGainDpsPerCut = measuredGain;
      trimDps = constrain(baselineRateDps,
                          -MAX_TRIM_CUT * measuredGain,
                          MAX_TRIM_CUT * measuredGain);
      probeState = ProbeState::READY;
      lastControlMs = now;
      // One short line fits the UART FIFO; it does not delay the motors.
      Serial.printf("PROBE ok: nudging ON, gain %.0f deg/s per cut, "
                    "motor1 = %s wheel\n", measuredGain,
                    motor1PositiveYaw ? "left" : "right");
    } else {
      probeState = probeAttempts >= 2 ? ProbeState::FAILED : ProbeState::WAITING;
      probeStartedMs = now;
      if (probeState == ProbeState::FAILED)
        Serial.println("PROBE failed: straights run WITHOUT nudging");
    }
  }
}

static void serviceSteering(uint32_t now) {
  if (probeState != ProbeState::READY || !sampleStreamReady(now) ||
      !motor1 || !motor2 || now - lastControlMs < CONTROL_INTERVAL_MS) return;
  const float dt = fminf((now - lastControlMs) * 0.001f, 0.04f);
  lastControlMs = now;

  // Measured yaw response to the probe supplies both motor polarity and
  // deg/s per fractional wheel-speed cut. Lead predicts where the heading is
  // going; the slow trim cancels steady wheel mismatch without a 1-degree
  // on/off threshold that causes a long-run weaving pattern.
  const float returnHeadingDeg = constrain(
      -atanf(crossTrackSteps / PATH_LOOKAHEAD_STEPS) * RAD_TO_DEG,
      -MAX_RETURN_HEADING_DEG, MAX_RETURN_HEADING_DEG);
  const float errorDeg = constrain(segmentHeadingErrorDeg() - returnHeadingDeg,
                                   -30.0f, 30.0f);
  const float predictedDeg = errorDeg + LOOKAHEAD_S * filteredRateDps;
  const float rawCut = (HEADING_GAIN_PER_S * predictedDeg + trimDps) /
                       yawGainDpsPerCut;
  if (fabsf(rawCut) < MAX_SPEED_CUT || errorDeg * rawCut < 0.0f) {
    trimDps += TRIM_GAIN_PER_S2 * errorDeg * dt;
    trimDps = constrain(trimDps, -MAX_TRIM_CUT * yawGainDpsPerCut,
                       MAX_TRIM_CUT * yawGainDpsPerCut);
  }
  const float targetCut = constrain(
      (HEADING_GAIN_PER_S * predictedDeg + trimDps) / yawGainDpsPerCut,
      -MAX_SPEED_CUT, MAX_SPEED_CUT);
  appliedCut += constrain(targetCut - appliedCut,
                          -MAX_CUT_CHANGE, MAX_CUT_CHANGE);
  const uint32_t slowed = static_cast<uint32_t>(
      lroundf(BASE_SPEED_HZ * (1.0f - fabsf(appliedCut))));
  const bool slowMotor1 = appliedCut > 0.0f ? !motor1PositiveYaw :
                          motor1PositiveYaw;
  const uint32_t speed1 = fabsf(appliedCut) < 0.0001f ? BASE_SPEED_HZ :
                          (slowMotor1 ? slowed : BASE_SPEED_HZ);
  const uint32_t speed2 = fabsf(appliedCut) < 0.0001f ? BASE_SPEED_HZ :
                          (slowMotor1 ? BASE_SPEED_HZ : slowed);
  if (speed1 != appliedSpeed1) {
    if (!setMotorSpeed(motor1, speed1)) { resetSteering(); return; }
    appliedSpeed1 = speed1;
  }
  if (speed2 != appliedSpeed2) {
    if (!setMotorSpeed(motor2, speed2)) { resetSteering(); return; }
    appliedSpeed2 = speed2;
  }
}

// Everything that must keep running while a route step waits.
static void serviceBackground() {
  const uint32_t beforePollMs = millis();
  if (haveSample && beforePollMs - lastSampleMs > LINK_TIMEOUT_MS) resetSteering();
  if (mpuLink.available() > 128) resetSteering();
  pollMpuLink();
  if (driveMode == DriveMode::CRUISE) {
    const uint32_t controlNow = millis();
    servicePathEstimate(controlNow);
    serviceProbe(controlNow);
    serviceSteering(controlNow);
  }
  delay(1);
}

static void serviceFor(uint32_t ms) {
  const uint32_t start = millis();
  while (millis() - start < ms) serviceBackground();
}

static void waitUntilStopped() {
  while ((motor1 && motor1->isRunning()) || (motor2 && motor2->isRunning()))
    serviceBackground();
}

static bool headingReady(uint32_t now) {
  return headingAnchored && sampleStreamReady(now);
}

static bool waitForHeading(uint32_t timeoutMs) {
  const uint32_t start = millis();
  while (!headingReady(millis())) {
    if (millis() - start >= timeoutMs) return false;
    serviceBackground();
  }
  return true;
}

// ---- Straight segments ----

// Drives forward exactly `steps` STEP pulses (average of both wheels),
// holding the route heading with the MPU, then stops.
void moveForward(int32_t steps) {
  if (!motor1 || !motor2 || steps <= 0) return;
  waitUntilStopped();
  const int32_t start1 = motor1->getCurrentPosition();
  const int32_t start2 = motor2->getCurrentPosition();
  setMotorSpeed(motor1, BASE_SPEED_HZ);
  setMotorSpeed(motor2, BASE_SPEED_HZ);
  appliedSpeed1 = appliedSpeed2 = BASE_SPEED_HZ;
  appliedCut = 0.0f;
  crossTrackSteps = 0.0f;
  pathReady = false;
  if (steps <= FINISH_STEPS) {
    // Too short to steer: one ramp, both wheels the same step count.
    driveMode = DriveMode::FINISH;
    motor1->move(steps);
    motor2->move(steps);
  } else {
    driveStartedMs = lastControlMs = lastPathMs = millis();
    cruiseRemainingSteps = steps;
    driveMode = DriveMode::CRUISE;
    motor1->runForward();
    motor2->runForward();
  }
  while (driveMode == DriveMode::CRUISE) {
    serviceBackground();
    const int32_t travelled = ((motor1->getCurrentPosition() - start1) +
                               (motor2->getCurrentPosition() - start2)) / 2;
    cruiseRemainingSteps = steps - travelled;
    if (cruiseRemainingSteps <= FINISH_STEPS) {
      // Both wheels travel the same remaining distance, so the steering
      // difference already applied is kept, and the ramp stops on target.
      driveMode = DriveMode::FINISH;
      abortProbe();
      const int32_t remaining = cruiseRemainingSteps > 0 ? cruiseRemainingSteps : 0;
      setMotorSpeed(motor1, BASE_SPEED_HZ);
      setMotorSpeed(motor2, BASE_SPEED_HZ);
      appliedSpeed1 = appliedSpeed2 = BASE_SPEED_HZ;
      motor1->moveTo(motor1->getCurrentPosition() + remaining);
      motor2->moveTo(motor2->getCurrentPosition() + remaining);
    }
  }
  waitUntilStopped();
  appliedCut = 0.0f;
  driveMode = DriveMode::IDLE;
  if (steps > FINISH_STEPS) {
    if (headingReady(millis())) {
      Serial.printf("STRAIGHT %ld steps: heading error %+.1f deg, nudging %s\n",
                    static_cast<long>(steps), segmentHeadingErrorDeg(),
                    probeState == ProbeState::READY ? "ON" : "OFF");
    } else {
      Serial.printf("STRAIGHT %ld steps: no MPU data, nudging OFF\n",
                    static_cast<long>(steps));
    }
  }
}

// ---- Turns ----

static uint32_t turnProfileSpeed(float remainingDeg, uint32_t maxSpeedHz) {
  const float remainingSteps = fmaxf(remainingDeg, 0.0f) * stepsPerDeg;
  const float speed = sqrtf(2.0f * ACCELERATION *
                            TURN_PROFILE_DECEL_FRACTION * remainingSteps);
  const uint32_t limited = static_cast<uint32_t>(constrain(
      speed, static_cast<float>(TURN_MIN_SPEED_HZ),
      static_cast<float>(maxSpeedHz)));
  return (limited / 10) * 10;  // avoid re-sending tiny speed changes
}

static void setSpinSpeed(uint32_t speedHz) {
  setMotorSpeed(motor1, speedHz);
  setMotorSpeed(motor2, speedHz);
}

// One spin in place toward the target, stopping on the MPU heading.
static SpinResult spinToward(float targetDeg, uint32_t maxSpeedHz) {
  const float startHeading = continuousHeadingDeg;
  const float startError = targetDeg - startHeading;
  const int dir = startError > 0.0f ? 1 : -1;  // + = left
  const int32_t start1 = motor1->getCurrentPosition();
  const int32_t start2 = motor2->getCurrentPosition();
  const float stepLimit =
      fabsf(startError) * stepsPerDeg * TURN_STEP_LIMIT_FACTOR + 300.0f;
  FastAccelStepper* left = leftMotor();
  FastAccelStepper* right = rightMotor();
  uint32_t speed = turnProfileSpeed(fabsf(startError),
      turnSideConfirmed ? maxSpeedHz : TURN_CORRECTION_SPEED_HZ);
  setSpinSpeed(speed);
  if (dir > 0) { left->runBackward(); right->runForward(); }
  else         { left->runForward();  right->runBackward(); }

  SpinResult result = SpinResult::REACHED;
  while (true) {
    serviceBackground();
    const uint32_t now = millis();
    if (!haveSample || !headingAnchored ||
        now - lastSampleMs > TURN_DATA_TIMEOUT_MS) {
      result = SpinResult::DATA_LOST;
      break;
    }
    if ((continuousHeadingDeg - startHeading) * dir < -TURN_WRONG_WAY_DEG) {
      result = SpinResult::WRONG_WAY;
      break;
    }
    const float spunSteps = 0.5f * (
        fabsf(static_cast<float>(motor1->getCurrentPosition() - start1)) +
        fabsf(static_cast<float>(motor2->getCurrentPosition() - start2)));
    if (spunSteps > stepLimit) {
      result = SpinResult::STEP_LIMIT;
      break;
    }
    // MPU heading, wheel travel since that sample, and the S3 sample age.
    const float headingNow =
        estimatedHeadingDeg() + latestRateDps * S3_SAMPLE_LATENCY_S;
    const float remainingDeg = (targetDeg - headingNow) * dir;
    if (remainingDeg <= TURN_STOP_DEG) break;
    // Until a turn has proven which motor is left, creep at correction speed
    // for the first few degrees. A wrong guess then backs off ~1 degree
    // instead of the ~8 degree wrong-way jerk a full-speed start makes.
    const bool sideUnproven = !turnSideConfirmed &&
        (continuousHeadingDeg - startHeading) * dir < TURN_WRONG_WAY_DEG;
    const uint32_t wanted = turnProfileSpeed(
        remainingDeg, sideUnproven ? TURN_CORRECTION_SPEED_HZ : maxSpeedHz);
    if (wanted != speed) {
      setSpinSpeed(wanted);
      speed = wanted;
    }
  }
  motor1->stopMove();
  motor2->stopMove();
  waitUntilStopped();
  return result;
}

// Without MPU data: rotate by wheel steps only, like the old fixed turns.
static void openLoopRotate(float deg) {
  const int32_t steps = static_cast<int32_t>(lroundf(fabsf(deg) * stepsPerDeg));
  if (steps < 1) return;
  setSpinSpeed(TURN_MAX_SPEED_HZ);
  const int32_t leftMove = deg > 0.0f ? -steps : steps;
  leftMotor()->move(leftMove);
  rightMotor()->move(-leftMove);
  waitUntilStopped();
}

static void rotateToHeading(float targetDeg) {
  if (!motor1 || !motor2) return;
  waitUntilStopped();
  driveMode = DriveMode::TURN;
  const float startHeading = continuousHeadingDeg;
  const int32_t start1 = motor1->getCurrentPosition();
  const int32_t start2 = motor2->getCurrentPosition();
  bool flipped = false;
  bool closedLoop = true;
  bool openLoopUsed = false;
  uint8_t spins = 0;
  for (uint8_t pass = 0; pass < TURN_MAX_PASSES; ++pass) {
    if (!waitForHeading(MPU_WAIT_MS)) {
      openLoopRotate(targetDeg - estimatedHeadingDeg());
      closedLoop = false;
      openLoopUsed = true;
      break;
    }
    const float error = targetDeg - continuousHeadingDeg;
    if (fabsf(error) <= TURN_TOLERANCE_DEG) break;
    const uint32_t maxSpeed = (pass == 0 || fabsf(error) > 10.0f)
        ? TURN_MAX_SPEED_HZ : TURN_CORRECTION_SPEED_HZ;
    ++spins;
    const SpinResult result = spinToward(targetDeg, maxSpeed);
    if (result == SpinResult::WRONG_WAY) {
      // The motor-side guess was wrong; swap it once and retry.
      if (flipped || turnSideConfirmed) { closedLoop = false; break; }
      motor1IsLeft = !motor1IsLeft;
      flipped = true;
      serviceFor(TURN_SETTLE_MS);
      continue;
    }
    if (result == SpinResult::STEP_LIMIT) { closedLoop = false; break; }
    if (result == SpinResult::REACHED) turnSideConfirmed = true;
    serviceFor(TURN_SETTLE_MS);  // let the final heading settle, then check
  }
  // Learn wheel steps per chassis degree from a clean MPU-measured turn.
  const float turnedDeg = continuousHeadingDeg - startHeading;
  if (closedLoop && !flipped && fabsf(turnedDeg) >= 30.0f) {
    const float spunSteps = wheelYawDeg(motor1->getCurrentPosition() - start1,
                                        motor2->getCurrentPosition() - start2) *
                            stepsPerDeg;
    const float measured = spunSteps / turnedDeg;
    if (measured > 0.5f * DEFAULT_STEPS_PER_DEG &&
        measured < 2.0f * DEFAULT_STEPS_PER_DEG) {
      stepsPerDeg = 0.5f * (stepsPerDeg + measured);
    }
  }
  driveMode = DriveMode::IDLE;
  if (openLoopUsed) {
    Serial.printf("TURN to %+.0f: NO MPU, wheel steps only (%.1f steps/deg), "
                  "angle not measured\n", targetDeg, stepsPerDeg);
  } else {
    Serial.printf("TURN to %+.0f: MPU reads %+.1f (error %+.1f deg), "
                  "%u spin(s), %.1f steps/deg%s\n", targetDeg,
                  continuousHeadingDeg, continuousHeadingDeg - targetDeg,
                  static_cast<unsigned>(spins), stepsPerDeg,
                  closedLoop ? "" : " [stopped early]");
  }
  if (flipped && turnSideConfirmed) {
    Serial.printf("  first spin went the wrong way; set "
                  "MOTOR1_IS_LEFT_GUESS = %s to remove that wiggle\n",
                  motor1IsLeft ? "true" : "false");
  }
}

// Turn angles are absolute along the route: each turn aims at the start
// heading +/- the sum of all turns so far, so heading drift picked up on a
// straight is removed by the next turn.
void turnLeft(float degrees) {
  if (degrees <= 0.0f) return;
  routeTargetHeadingDeg += degrees;
  rotateToHeading(routeTargetHeadingDeg);
}

void turnRight(float degrees) {
  if (degrees <= 0.0f) return;
  routeTargetHeadingDeg -= degrees;
  rotateToHeading(routeTargetHeadingDeg);
}

void setup() {
  // USB report: MPU link state, one line per turn/long straight, probe result.
  Serial.begin(115200);
  delay(500);  // match the known-working motor sketch's power-up timing
  engine.init();
  motor1 = engine.stepperConnectToPin(STEP_PIN1);
  motor2 = engine.stepperConnectToPin(STEP_PIN2);
  if (motor1) {
    motor1->setDirectionPin(DIR_PIN1, false);
    motor1->setSpeedInHz(BASE_SPEED_HZ);
    motor1->setAcceleration(ACCELERATION);
  }
  if (motor2) {
    motor2->setDirectionPin(DIR_PIN2, true);
    motor2->setSpeedInHz(BASE_SPEED_HZ);
    motor2->setAcceleration(ACCELERATION);
  }
  mpuLink.setRxBufferSize(256);
  mpuLink.begin(115200, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
}

void loop() {
  if (!routeFinished) {
    // Give the S3 time to calibrate while still; start anyway without it.
    if (waitForHeading(START_MPU_WAIT_MS)) {
      Serial.println("MPU link OK: turns are MPU-measured");
    } else {
      Serial.printf("MPU link DOWN (%lu packets OK, %lu rejected): route runs "
                    "open loop%s\n", static_cast<unsigned long>(acceptedPackets),
                    static_cast<unsigned long>(rejectedPackets),
                    rejectedPackets ? " - reflash CalibS3 (packet format)" : "");
    }
    runStart();
    Serial.println("ROUTE done");
    routeFinished = true;
  }
  serviceBackground();  // route done: stay stopped, keep reading the link
}

void runStart() {

  // Adj
  for(int i = 0; i < 5; i++){
    moveForward(100);
    delay(1);
  }

  moveForward(10000);

  turnLeft(90);

  // Adj
  for(int i = 0; i < 5; i++){
    moveForward(100);
    delay(1);
  }

  moveForward(605000); 

  turnLeft(90);

  // Adj
  for(int i = 0; i < 5; i++){
    moveForward(100);
    delay(1);
  }

  moveForward(6000);

  turnRight(90);

  // Adj
  for(int i = 0; i < 5; i++){
    moveForward(100);
    delay(1);
  }

  moveForward(25000);

  turnRight(90);

  // Adj
  for(int i = 0; i < 5; i++){
    moveForward(100);
    delay(1);
  }

  moveForward(14000);

  turnLeft(180);

}
