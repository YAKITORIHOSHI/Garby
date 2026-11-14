#pragma once
// MPU heading control and gyro/LiDAR wall fusion for the executor. Merged from
// the calibrated MPU_and_Motor-Latest/Calib test (2026-11-11), with the hallway
// fusion of the 2026-11-11 navigation review. Include after the motor constants
// in MAIN_EXECUTOR_CODE.h.
//
// The S3 bridge sends on the existing UART, never ACKed:
//   M,<seq>,<heading cdeg>,<rate cdeg/s>,<epoch>,<allow>*<xor hex>   50 Hz
//   MPU:FAULT
//   W,<acq>,<age ms>,<flags>,<left mm>,<right mm>,<H cdeg>*<xor hex>  per LiDAR scan
// <epoch> numbers the S3 heading reference; a new one (S3 reboot or a sensor
// outage over 500 ms) reads 0 at an unknown chassis angle, so the executor then
// continues from its last heading plus the wheel rotation since that sample.
// <allow> bit 0 permits steering LEFT, bit 1 RIGHT; the bridge clears a bit when
// nearest-object clearance on that side is unsafe, and both while stopped.
// W carries the Pi's wall line fit: perpendicular distances of the left/right
// walls from the chassis centerline and the robot's heading relative to them
// (+ = rotated left); flags bit0/1/2 mark left/right/heading present. <age> is
// the time since that scan was acquired, so it is matched to the gyro heading
// of that moment, not of its arrival.
//
// These lines are heading evidence only. They never refresh path permission or
// link freshness and cannot release a STOP.
//
// Heading frame: + is LEFT. The executor keeps an unwrapped heading from boot.
// A route anchors its frame at start; each turn adds or subtracts its angle.
// Root cause of the off-centre runs (docs/navigation-2026-11-11.md): that frame
// is gyro-only, so its error at segment start and any residual gyro bias grow
// into a lateral error of distance x sin(error). The fusion below measures the
// hallway direction with the LiDAR walls, so the target is the hallway itself:
//   axis    = gyro heading at the scan - H     (hallway direction, gyro frame)
//   lateral = (right - left) / 2              (+ = left of centre)
//   target  = axis + centring term(lateral)   (bounded, deadband + hysteresis)
// Between scans and through wall gaps the gyro carries the axis, and step
// travel carries the lateral estimate for a bounded time. Steps are commanded
// pulses, not encoders: slip makes that dead reckoning approximate.
#include <Arduino.h>
#include <FastAccelStepper.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

class HeadingControl {
 public:
  // ---- stream ----
  static constexpr uint32_t LINK_TIMEOUT_MS = 250;
  static constexpr uint32_t CONTROL_INTERVAL_MS = 20;
  static constexpr uint8_t STARTUP_SAMPLES = 10;
  static constexpr uint32_t STARTUP_CONFIRM_MS = 180;
  static constexpr float RATE_FILTER_MS = 80.0f;
  static constexpr uint8_t ALLOW_LEFT = 0x01;
  static constexpr uint8_t ALLOW_RIGHT = 0x02;

  // ---- straight heading hold (Calib values; probe measures the gain) ----
  static constexpr uint32_t DRIVE_SETTLE_MS = 1200;
  static constexpr uint32_t BASELINE_MS = 350;
  static constexpr uint32_t PROBE_MS = 400;
  static constexpr float FIRST_PROBE_CUT = 0.03f;
  static constexpr float SECOND_PROBE_CUT = 0.08f;
  static constexpr float MIN_PROBE_RESPONSE_DPS = 0.8f;
  static constexpr float MAX_SPEED_CUT = 0.30f;
  // Per 20 ms: 0.015 * 5600 steps/s = 84 steps/s, i.e. 4200 steps/s^2, inside
  // the 4800 steps/s^2 straight acceleration.
  static constexpr float MAX_CUT_CHANGE = 0.015f;
  static constexpr float MAX_TRIM_CUT = 0.10f;
  static constexpr float HEADING_GAIN_PER_S = 2.5f;
  static constexpr float LOOKAHEAD_S = 0.28f;
  static constexpr float TRIM_GAIN_PER_S2 = 0.15f;
  // Calib used 20000/40000 steps at 8000 steps/s: 2.5 s and 5 s of travel.
  // Kept in time, so the slower production cruise returns to its line alike.
  static constexpr float PATH_LOOKAHEAD_S = 2.5f;
  static constexpr float MAX_CROSS_TRACK_S = 5.0f;
  static constexpr float MAX_RETURN_HEADING_DEG = 12.0f;
  static constexpr uint32_t PROBE_RESERVE_EXTRA_MS = 300;

  // ---- gyro/LiDAR wall fusion (straights with centering enabled) ----
  static constexpr bool WALL_FUSION_ENABLED = true;
  static constexpr uint32_t WALL_MAX_AGE_MS = 900;       // older scans are not fused
  // Hallway direction: an alpha-beta filter per accepted scan (~5.75 Hz). The
  // gyro carries it between scans; the beta term learns residual gyro drift.
  static constexpr float AXIS_GAIN = 0.15f;              // ~1.2 s time constant
  static constexpr float DRIFT_GAIN = 0.01f;
  static constexpr float MAX_DRIFT_DPS = 0.2f;
  static constexpr float AXIS_GATE_DEG = 4.0f;           // larger jumps are door leaves/clutter
  static constexpr float AXIS_MAX_FROM_ROUTE_DEG = 15.0f; // walls further off the route are not this hallway
  static constexpr uint8_t AXIS_RESEED_REJECTS = 6;      // consecutive gated scans that replace the estimate
  // Lateral position: measured walls, carried by step travel between scans.
  static constexpr float LATERAL_GAIN = 0.5f;
  static constexpr float LATERAL_GATE_CM = 15.0f;
  static constexpr uint8_t LATERAL_RESEED_REJECTS = 4;
  static constexpr uint32_t LATERAL_HOLD_MS = 8000;      // dead-reckoning limit through a wall gap
  static constexpr float WIDTH_MIN_CM = 120.0f;
  static constexpr float WIDTH_MAX_CM = 450.0f;
  static constexpr float WIDTH_GAIN = 0.2f;
  static constexpr float WIDTH_GATE_CM = 25.0f;          // posts (164 cm) never retune a 190 cm width
  // Centring: target = axis - atan(excess / lookahead), excess beyond a deadband.
  // Engages beyond ENGAGE, releases inside RELEASE (hysteresis), never over MAX.
  static constexpr float CENTER_ENGAGE_CM = 5.0f;
  static constexpr float CENTER_RELEASE_CM = 2.0f;
  static constexpr float CENTER_DEADBAND_CM = 2.0f;
  static constexpr float CENTER_LOOKAHEAD_CM = 120.0f;   // lateral time constant = lookahead / speed
  static constexpr float MAX_CENTER_DEG = 5.0f;
  static constexpr float CENTER_SLEW_DEG_PER_S = 4.0f;
  // 1600 steps/rev and a 65 mm wheel (MAIN_EXECUTOR_CODE.h). Calculated, NOT measured.
  static constexpr float STEPS_PER_CM = STEPS_PER_REV / (3.14159265f * WHEEL_DIAMETER_MM * 0.1f);

  // ---- turns (Calib values) ----
  // Guess only: the first MPU turn checks it and swaps it if the chassis
  // rotates the wrong way. The steering probe also measures it.
  static constexpr bool MOTOR1_IS_LEFT_GUESS = true;
  // The old open-loop turnLeft(4900) was 90 degrees; refined by every clean turn.
  static constexpr float DEFAULT_STEPS_PER_DEG = 4900.0f / 90.0f;
  static constexpr uint32_t TURN_MAX_SPEED_HZ = 3000;  // ~55 deg/s, well inside +/-250 deg/s
  static constexpr uint32_t TURN_CORRECTION_SPEED_HZ = 800;
  static constexpr uint32_t TURN_MIN_SPEED_HZ = 300;
  static constexpr float TURN_PROFILE_DECEL_FRACTION = 0.6f;
  static constexpr float TURN_STOP_DEG = 0.3f;
  static constexpr float TURN_TOLERANCE_DEG = 1.0f;
  static constexpr float TURN_WRONG_WAY_DEG = 3.0f;
  static constexpr float TURN_STEP_LIMIT_FACTOR = 1.5f;
  static constexpr uint8_t TURN_MAX_PASSES = 5;
  static constexpr uint32_t TURN_SETTLE_MS = 250;
  static constexpr uint32_t TURN_DATA_TIMEOUT_MS = 100;  // 5 missed samples
  static constexpr uint32_t MPU_WAIT_MS = 1500;
  static constexpr uint32_t START_MPU_WAIT_MS = 5000;
  // Motion without heading feedback is not allowed. A stream missing this long
  // while the robot is held ends the route as a fault. Was 10 s until 2026-11-12:
  // an S3 brownout restart (logged on 2026-11-04 with up to two minutes without
  // advertising) then latched a route fault and the robot stayed stopped for good.
  // The wheels are stopped throughout; the hold drops to low power after 10 s.
  static constexpr uint32_t HEADING_RECOVERY_MS = 300000;
  // Recovery includes any required wall confirmation within the same deadline.
  // After a hold this long with a new S3 reference, two agreeing wall scans must
  // confirm the route direction before motion; missing walls keep it stopped.
  static constexpr uint32_t HEADING_VERIFY_AFTER_MS = 10000;
  static constexpr uint32_t HEADING_LOSS_HALT_MS = 300;
  static constexpr float S3_SAMPLE_LATENCY_S = 0.01f;
  // Wall alignment after each turn: two fresh scans taken after the spin stopped.
  static constexpr uint32_t ALIGN_WAIT_MS = 1500;
  static constexpr float ALIGN_TOLERANCE_DEG = 0.75f;
  static constexpr float ALIGN_MAX_CORRECTION_DEG = 8.0f;  // more is not the expected hallway
  static constexpr float ALIGN_AGREE_DEG = 1.5f;
  static constexpr uint8_t ALIGN_MAX_PASSES = 2;

  // ---- pre-run centring (2026-11-12, owner's idea) ----
  // Before a long centred straight the robot stands still, measures where it is
  // between the walls and fixes it there: face the hallway axis, then sidestep
  // (turn toward the centre, drive the diagonal, turn back) until it is within
  // PRECENTER_TOLERANCE_CM. The straight then starts centred and aligned, so the
  // in-run centring (engages at CENTER_ENGAGE_CM) has nothing to correct and the
  // robot does not weave down the hallway correcting a start error.
  static constexpr int32_t PRECENTER_MIN_STEPS = 50000;   // only the long hallway runs
  static constexpr float PRECENTER_TOLERANCE_CM = 3.0f;   // inside the 5 cm engage threshold
  static constexpr float PRECENTER_MAX_CM = 60.0f;        // more is not this hallway's centre
  static constexpr float PRECENTER_AGREE_CM = 3.0f;       // two scans must agree on the offset
  static constexpr float PRECENTER_ANGLE_DEG = 35.0f;     // sidestep heading off the axis
  static constexpr float PRECENTER_MIN_MOVE_CM = 2.0f;    // shorter diagonals are not worth a turn pair
  static constexpr uint8_t PRECENTER_MAX_PASSES = 3;
  static constexpr uint32_t PRECENTER_WAIT_MS = 2500;     // ~14 scans at 5.75 Hz
  static constexpr float PRECENTER_LEARN_MIN_CM = 8.0f;   // learn the distance scale from bigger moves

  enum class ProbeState : uint8_t { WAITING, BASELINE, PULSE, READY, FAILED };

  void attach(FastAccelStepper* motor1, FastAccelStepper* motor2) {
    m1_ = motor1;
    m2_ = motor2;
  }

  // ======================= stream =======================
  // Returns true when the line belongs to the heading stream.
  bool handleLine(const char* line, uint32_t now) {
    if (line == nullptr) return false;
    if (strcmp(line, "MPU:FAULT") == 0) {
      ++faultLines_;
      resetStream();
      return true;
    }
    if (line[0] == 'W' && line[1] == ',') {
      handleWallLine(line, now);
      return true;
    }
    if (line[0] != 'M' || line[1] != ',') return false;
    char copy[96];
    const size_t length = strnlen(line, sizeof(copy));
    if (length >= sizeof(copy)) {
      ++rejected_;
      return true;
    }
    memcpy(copy, line, length + 1U);
    uint32_t sequence = 0, epoch = 0;
    float headingDeg = 0.0f, rateDps = 0.0f;
    uint8_t allow = 0;
    // One damaged/duplicate packet is ignored; the freshness timeout handles a
    // broken link without repeatedly erasing a valid steering calibration.
    if (!decodeSample(copy, &sequence, &headingDeg, &rateDps, &epoch, &allow)) {
      ++rejected_;
      return true;
    }
    if (haveSample_ && (int32_t)(sequence - lastSequence_) <= 0) return true;
    ++accepted_;
    if (haveSample_ && now - lastSampleMs_ > LINK_TIMEOUT_MS) resetStream();
    // Same S3 reference: add its change, even across a short UART outage (the
    // S3 kept integrating). A new reference continues from the last heading
    // plus the wheel rotation since that sample.
    const bool sameReference = headingAnchored_ && epoch == lastEpoch_ &&
        (int32_t)(sequence - lastSequence_) > 0;
    if (sameReference) {
      continuous_ += wrap180(headingDeg - lastRawDeg_);
    } else {
      continuous_ = estimatedHeading();
      if (headingAnchored_) ++reanchors_;
    }
    headingAnchored_ = true;
    lastRawDeg_ = headingDeg;
    lastEpoch_ = epoch;
    if (m1_ && m2_) {
      samplePos1_ = m1_->getCurrentPosition();
      samplePos2_ = m2_->getCurrentPosition();
    }
    if (!haveSample_) {
      firstSampleMs_ = now;
      startupSamples_ = 0;
      filteredRate_ = rateDps;
    } else {
      const float dtMs = (float)(now - lastSampleMs_);
      const float alpha = dtMs / (RATE_FILTER_MS + dtMs);
      filteredRate_ += alpha * (rateDps - filteredRate_);
    }
    if (startupSamples_ < STARTUP_SAMPLES) ++startupSamples_;
    lastSequence_ = sequence;
    latestRate_ = rateDps;
    lastSampleMs_ = now;
    haveSample_ = true;
    allow_ = allow;
    pushHistory(now, continuous_);
    return true;
  }

  void serviceLink(uint32_t now) {
    if (haveSample_ && now - lastSampleMs_ > LINK_TIMEOUT_MS) resetStream();
  }

  bool streamReady(uint32_t now) const {
    return haveSample_ && now - lastSampleMs_ <= LINK_TIMEOUT_MS &&
           startupSamples_ >= STARTUP_SAMPLES &&
           now - firstSampleMs_ >= STARTUP_CONFIRM_MS;
  }
  bool headingReady(uint32_t now) const { return headingAnchored_ && streamReady(now); }
  bool sampleFresh(uint32_t now, uint32_t maxAgeMs) const {
    return haveSample_ && headingAnchored_ && now - lastSampleMs_ <= maxAgeMs;
  }

  float heading() const { return continuous_; }
  float latestRate() const { return latestRate_; }

  // Last MPU heading plus wheel rotation since that sample. Without MPU data this
  // is wheel dead reckoning; slip makes it approximate.
  float estimatedHeading() const {
    if (!m1_ || !m2_) return continuous_;
    return continuous_ + wheelYawDeg(m1_->getCurrentPosition() - samplePos1_,
                                     m2_->getCurrentPosition() - samplePos2_);
  }

  // Gyro heading (continuous frame) at a past moment, from the 50 Hz history.
  bool headingAt(uint32_t ms, float* out) const {
    if (historyCount_ == 0 || out == nullptr) return false;
    const HeadingPoint& newest = history_[(uint8_t)(historyNext_ + HISTORY - 1U) % HISTORY];
    const int32_t ahead = (int32_t)(ms - newest.ms);
    if (ahead >= 0) {
      if (ahead > 150) return false;
      *out = newest.deg + latestRate_ * (float)ahead * 0.001f;
      return true;
    }
    for (uint8_t i = 1; i < historyCount_; ++i) {
      const HeadingPoint& later = history_[(uint8_t)(historyNext_ + HISTORY - i) % HISTORY];
      const HeadingPoint& earlier = history_[(uint8_t)(historyNext_ + HISTORY - i - 1U) % HISTORY];
      if ((int32_t)(ms - earlier.ms) >= 0) {
        const float span = (float)(later.ms - earlier.ms);
        const float t = span > 0.0f ? (float)(ms - earlier.ms) / span : 0.0f;
        *out = earlier.deg + t * (later.deg - earlier.deg);
        return true;
      }
    }
    return false;  // older than the history
  }

  // ======================= route frame =======================
  void beginRoute(uint32_t now) {
    routeTarget_ = headingReady(now) ? continuous_ : estimatedHeading();
  }
  void addTurn(float degrees) { routeTarget_ += degrees; }
  float routeTarget() const { return routeTarget_; }
  float routeError() const { return continuous_ - routeTarget_; }
  // A turn's wall alignment found the real hallway direction.
  void setRouteTarget(float degrees) { routeTarget_ = degrees; }

  // ======================= straights =======================
  void beginStraight(uint32_t now, bool centering) {
    centering_ = centering;
    straightActive_ = true;
    appliedCut_ = 0.0f;
    crossTrackSteps_ = 0.0f;
    pathReady_ = false;
    driveStartedMs_ = lastControlMs_ = lastPathMs_ = now;
    applied1_ = applied2_ = 0;
    // Each straight is a new hallway (a turn came before it). The learned gyro
    // drift is a property of the sensor and is kept.
    axisKnown_ = lateralKnown_ = widthKnown_ = centerActive_ = false;
    axisRejects_ = lateralRejects_ = 0;
    centerTargetDeg_ = 0.0f;
    lastRealignDeg_ = 0.0f;
    segmentAxisUpdates_ = segmentWallScans_ = 0;
  }

  // Cruise restarted from rest (segment start or after an obstacle hold). The
  // sideways estimate is kept: the robot did not move sideways while stopped.
  void resumeCruise(uint32_t now) {
    abortProbe(now);
    appliedCut_ = 0.0f;
    driveStartedMs_ = lastControlMs_ = lastPathMs_ = now;
    applied1_ = applied2_ = 0;
  }

  // The caller brakes or hands the straight to the final ramp.
  void pauseCruise(uint32_t now) {
    abortProbe(now);
    appliedCut_ = 0.0f;
    applied1_ = applied2_ = 0;
  }

  // Returns true when the walls realigned the route frame to the hallway.
  bool endStraight(uint32_t now) {
    pauseCruise(now);
    bool realigned = false;
    if (straightActive_ && centering_ && axisKnown_ && now - axisAtMs_ <= 3000U) {
      const float axis = axisAt(now);
      lastRealignDeg_ = axis - routeTarget_;
      routeTarget_ = axis;
      realigned = true;
    }
    straightActive_ = false;
    centering_ = false;
    return realigned;
  }

  // Every loop while cruising. Applies both wheel speeds; base1/base2 already
  // include the motor trim. Returns false if a motor rejected a speed.
  bool serviceCruise(uint32_t now, uint32_t base1, uint32_t base2,
                     int32_t remainingSteps, uint32_t acceleration) {
    if (!m1_ || !m2_) return false;
    remaining_ = remainingSteps;
    base1_ = base1;
    base2_ = base2;
    servicePathEstimate(now);
    serviceProbe(now, acceleration);
    if (!serviceSteering(now)) {
      // Without a steering command, both wheels run their plain base speed.
      if (probeState_ != ProbeState::PULSE) {
        appliedCut_ = 0.0f;
        return applySpeeds(base1_, base2_, acceleration);
      }
      return true;
    }
    return applySpeeds(target1_, target2_, acceleration);
  }

  int32_t probeReserveSteps(uint32_t baseSpeed, int32_t finishSteps) const {
    return finishSteps + (int32_t)((uint64_t)baseSpeed *
        (BASELINE_MS + PROBE_MS + PROBE_RESERVE_EXTRA_MS) / 1000ULL);
  }
  void setProbeReserve(int32_t steps) { probeReserve_ = steps; }

  ProbeState probeState() const { return probeState_; }
  // Heading hold runs once the probe measured the steering, or once a turn has
  // proved which motor is the left wheel (gain from the measured spin geometry).
  bool steeringOn() const { return probeState_ == ProbeState::READY || turnSideConfirmed_; }
  float appliedCut() const { return appliedCut_; }
  float yawGain() const { return yawGain_; }
  float crossTrackSteps() const { return crossTrackSteps_; }
  uint8_t allowMask() const { return allow_; }
  bool motor1PositiveYaw() const { return motor1PositiveYaw_; }

  // ---- fusion state, for logs and tests ----
  bool axisKnown() const { return axisKnown_; }
  float axisAt(uint32_t ms) const {
    return axisDeg_ + driftDps_ * (float)(int32_t)(ms - axisAtMs_) * 0.001f;
  }
  float driftDps() const { return driftDps_; }
  bool lateralKnown(uint32_t now) const { return lateralKnown_ && now - lateralAtMs_ <= LATERAL_HOLD_MS; }
  float lateralCm() const { return lateralCm_; }
  bool widthKnown() const { return widthKnown_; }
  float widthCm() const { return widthCm_; }
  float centerTargetDeg() const { return centerTargetDeg_; }
  float lastRealignDeg() const { return lastRealignDeg_; }
  uint32_t wallLines() const { return wallLines_; }
  uint32_t wallRejected() const { return wallRejected_; }
  uint32_t axisReseeds() const { return axisReseeds_; }
  uint16_t segmentAxisUpdates() const { return segmentAxisUpdates_; }
  uint16_t segmentWallScans() const { return segmentWallScans_; }
  // The heading this straight is steering to (hallway axis + centring, or the route).
  float steeringTarget(uint32_t now) const {
    return lidarSteering() ? axisAt(now) + centerTargetDeg_ : routeTarget_;
  }

  // One line of fusion telemetry. Returns false outside a straight.
  bool fusionReport(uint32_t now, char* out, size_t size) const {
    if (!straightActive_ || out == nullptr || size == 0) return false;
    const bool lateral = lateralKnown(now);
    // Kept short: every 20 characters is one BLE notification on the way to the Pi.
    // lidar = steering to the walls; y = cm left(+) of centre; hdg = degrees left(+)
    // of the hallway; ctr = centring command; w = hallway width; drift = learned
    // gyro drift; n = wall scans seen/fused this segment; rej = rejected in total.
    snprintf(out, size,
             "[HEADING FUSE lidar=%d y=%+.1f hdg=%+.2f ctr=%+.2f w=%.0f drift=%+.3f n=%u/%u rej=%lu]",
             lidarSteering() ? 1 : 0, lateral ? lateralCm_ : 0.0f,
             axisKnown_ ? continuous_ - axisAt(now) : 0.0f, centerTargetDeg_,
             widthKnown_ ? widthCm_ : 0.0f, driftDps_, (unsigned)segmentWallScans_,
             (unsigned)segmentAxisUpdates_, (unsigned long)wallRejected_);
    return true;
  }

  // ======================= turns =======================
  FastAccelStepper* leftMotor() const { return motor1IsLeft_ ? m1_ : m2_; }
  FastAccelStepper* rightMotor() const { return motor1IsLeft_ ? m2_ : m1_; }
  bool motor1IsLeft() const { return motor1IsLeft_; }
  bool turnSideConfirmed() const { return turnSideConfirmed_; }
  void swapTurnSides() { motor1IsLeft_ = !motor1IsLeft_; }
  void confirmTurnSides() { turnSideConfirmed_ = true; }
  float stepsPerDeg() const { return stepsPerDeg_; }

  // Chassis rotation implied by wheel STEP travel (+ = left).
  float wheelYawDeg(int32_t delta1, int32_t delta2) const {
    const float left = (float)(motor1IsLeft_ ? delta1 : delta2);
    const float right = (float)(motor1IsLeft_ ? delta2 : delta1);
    return 0.5f * (right - left) / stepsPerDeg_;
  }

  uint32_t turnProfileSpeed(float remainingDeg, uint32_t maxSpeedHz,
                            uint32_t acceleration) const {
    const float remainingSteps = fmaxf(remainingDeg, 0.0f) * stepsPerDeg_;
    const float speed = sqrtf(2.0f * (float)acceleration *
                              TURN_PROFILE_DECEL_FRACTION * remainingSteps);
    const uint32_t limited = (uint32_t)constrain(speed, (float)TURN_MIN_SPEED_HZ,
                                                 (float)maxSpeedHz);
    return (limited / 10U) * 10U;  // avoid re-sending tiny speed changes
  }

  // Heading at the MPU sample, wheel travel since then and the S3 sample age.
  float turnHeadingNow() const {
    return estimatedHeading() + latestRate_ * S3_SAMPLE_LATENCY_S;
  }

  // Learn wheel steps per chassis degree from a clean MPU-measured turn.
  void learnStepsPerDeg(float turnedDeg, int32_t delta1, int32_t delta2) {
    if (fabsf(turnedDeg) < 30.0f) return;
    const float spunSteps = wheelYawDeg(delta1, delta2) * stepsPerDeg_;
    const float measured = spunSteps / turnedDeg;
    if (measured > 0.5f * DEFAULT_STEPS_PER_DEG && measured < 2.0f * DEFAULT_STEPS_PER_DEG) {
      stepsPerDeg_ = 0.5f * (stepsPerDeg_ + measured);
    }
  }

  // The newest wall scan acquired at or after sinceMs, with the gyro heading at
  // its acquisition. For measurements while standing still (pre-run centring).
  struct WallView {
    uint32_t acquisition = 0, scanMs = 0;
    uint8_t flags = 0;              // bit0 left, bit1 right, bit2 heading
    float leftCm = 0.0f, rightCm = 0.0f, wallHeadingDeg = 0.0f, gyroThenDeg = 0.0f;
    bool both() const { return (flags & (WALL_LEFT | WALL_RIGHT)) == (WALL_LEFT | WALL_RIGHT); }
    bool hasHeading() const { return (flags & WALL_HEADING) != 0; }
    float lateralCm() const { return 0.5f * (rightCm - leftCm); }   // + = left of centre
    float widthCm() const { return leftCm + rightCm; }
    float axisDeg() const { return gyroThenDeg - wallHeadingDeg; }  // hallway, gyro frame
  };
  bool wallViewSince(uint32_t sinceMs, uint32_t now, WallView* out) const {
    if (!wall_.valid || out == nullptr) return false;
    if ((int32_t)(wall_.scanMs - sinceMs) < 0 || now - wall_.scanMs > WALL_MAX_AGE_MS) return false;
    float headingThen = 0.0f;
    if (!headingAt(wall_.scanMs, &headingThen)) return false;
    out->acquisition = wall_.acquisition;
    out->scanMs = wall_.scanMs;
    out->flags = wall_.flags;
    out->leftCm = wall_.leftCm;
    out->rightCm = wall_.rightCm;
    out->wallHeadingDeg = wall_.headingDeg;
    out->gyroThenDeg = headingThen;
    return true;
  }

  // Hallway direction (gyro frame) from the newest wall scan acquired at or
  // after sinceMs, for the post-turn alignment. False without a usable scan.
  bool wallAxisSince(uint32_t sinceMs, uint32_t now, float* axisDeg, uint32_t* acquisition) const {
    if (!wall_.valid || !(wall_.flags & WALL_HEADING) || axisDeg == nullptr) return false;
    if ((int32_t)(wall_.scanMs - sinceMs) < 0 || now - wall_.scanMs > WALL_MAX_AGE_MS) return false;
    float headingThen = 0.0f;
    if (!headingAt(wall_.scanMs, &headingThen)) return false;
    *axisDeg = headingThen - wall_.headingDeg;
    if (acquisition != nullptr) *acquisition = wall_.acquisition;
    return true;
  }

  // ======================= diagnostics =======================
  uint32_t accepted() const { return accepted_; }
  uint32_t rejected() const { return rejected_; }
  uint32_t faultLines() const { return faultLines_; }
  uint32_t reanchors() const { return reanchors_; }

  static float wrap180(float deg) {
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
  }

 private:
  static constexpr uint8_t HISTORY = 64;  // 1.28 s at 50 Hz: covers the oldest fused scan
  static constexpr uint8_t WALL_LEFT = 0x01, WALL_RIGHT = 0x02, WALL_HEADING = 0x04;
  struct HeadingPoint { uint32_t ms; float deg; };
  struct WallSample {
    bool valid = false;
    uint32_t acquisition = 0, scanMs = 0;
    uint8_t flags = 0;
    float leftCm = 0.0f, rightCm = 0.0f, headingDeg = 0.0f;
  };

  FastAccelStepper* m1_ = nullptr;
  FastAccelStepper* m2_ = nullptr;
  // stream
  bool haveSample_ = false;
  bool headingAnchored_ = false;
  uint32_t lastSequence_ = 0, lastEpoch_ = 0, lastSampleMs_ = 0, firstSampleMs_ = 0;
  uint8_t startupSamples_ = 0;
  float continuous_ = 0.0f, lastRawDeg_ = 0.0f;
  int32_t samplePos1_ = 0, samplePos2_ = 0;
  float latestRate_ = 0.0f, filteredRate_ = 0.0f;
  uint8_t allow_ = 0;
  uint32_t accepted_ = 0, rejected_ = 0, faultLines_ = 0, reanchors_ = 0;
  HeadingPoint history_[HISTORY] = {};
  uint8_t historyNext_ = 0, historyCount_ = 0;
  // walls
  WallSample wall_;
  bool haveWallAcquisition_ = false;
  uint32_t lastWallAcquisition_ = 0, wallLines_ = 0, wallRejected_ = 0;
  // fusion
  bool axisKnown_ = false, lateralKnown_ = false, widthKnown_ = false, centerActive_ = false;
  float axisDeg_ = 0.0f, driftDps_ = 0.0f, lateralCm_ = 0.0f, widthCm_ = 0.0f;
  float centerTargetDeg_ = 0.0f, lastRealignDeg_ = 0.0f;
  uint32_t axisAtMs_ = 0, lateralAtMs_ = 0, axisReseeds_ = 0;
  uint8_t axisRejects_ = 0, lateralRejects_ = 0;
  uint16_t segmentAxisUpdates_ = 0, segmentWallScans_ = 0;
  // route
  float routeTarget_ = 0.0f;
  // straight
  bool straightActive_ = false, centering_ = false, pathReady_ = false;
  uint32_t driveStartedMs_ = 0, lastControlMs_ = 0, lastPathMs_ = 0;
  int32_t remaining_ = 0, probeReserve_ = 0;
  uint32_t base1_ = 0, base2_ = 0, target1_ = 0, target2_ = 0, applied1_ = 0, applied2_ = 0;
  float crossTrackSteps_ = 0.0f, appliedCut_ = 0.0f;
  // probe
  ProbeState probeState_ = ProbeState::WAITING;
  uint32_t probeStartedMs_ = 0, probeLastSequence_ = 0, probeBaseSpeed_ = 0;
  float yawGain_ = 0.0f, trimDps_ = 0.0f;
  float baselineRate_ = 0.0f, baselineStdErrorSq_ = 0.0f, activeProbeCut_ = FIRST_PROBE_CUT;
  float rateSum_ = 0.0f, rateSumSquares_ = 0.0f;
  uint16_t rateCount_ = 0;
  uint8_t probeAttempts_ = 0;
  bool motor1PositiveYaw_ = false;
  // turns
  float stepsPerDeg_ = DEFAULT_STEPS_PER_DEG;
  bool motor1IsLeft_ = MOTOR1_IS_LEFT_GUESS;
  bool turnSideConfirmed_ = false;

  bool lidarSteering() const { return centering_ && WALL_FUSION_ENABLED && axisKnown_; }

  void pushHistory(uint32_t ms, float deg) {
    history_[historyNext_] = {ms, deg};
    historyNext_ = (uint8_t)((historyNext_ + 1U) % HISTORY);
    if (historyCount_ < HISTORY) ++historyCount_;
  }

  void resetStream() {
    haveSample_ = false;
    startupSamples_ = 0;
    filteredRate_ = latestRate_ = 0.0f;
    allow_ = 0;
    pathReady_ = false;
    crossTrackSteps_ = 0.0f;
    // A probe needs an uninterrupted rate record; a measured gain survives.
    if (probeState_ == ProbeState::BASELINE || probeState_ == ProbeState::PULSE) {
      if (probeState_ == ProbeState::PULSE && probeAttempts_ > 0) --probeAttempts_;
      probeState_ = ProbeState::WAITING;
      rateCount_ = 0;
    }
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

  static int hexDigit(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
  }

  // Verifies and strips "*XX". Returns the first field after "<letter>,".
  static char* checkedFields(char* line) {
    char* star = strchr(line, '*');
    if (!star || strlen(star) != 3) return nullptr;
    const int high = hexDigit(star[1]);
    const int low = hexDigit(star[2]);
    if (high < 0 || low < 0) return nullptr;
    uint8_t check = 0;
    for (char* p = line; p < star; ++p) check ^= (uint8_t)*p;
    if (check != (uint8_t)((high << 4) | low)) return nullptr;
    *star = '\0';
    return line + 2;
  }

  static bool decodeSample(char* line, uint32_t* sequence, float* heading, float* rate,
                           uint32_t* epoch, uint8_t* allow) {
    char* field = checkedFields(line);
    if (field == nullptr) return false;
    unsigned long number = 0, referenceEpoch = 0, allowBits = 0;
    long headingCdeg = 0, rateCdps = 0;
    if (!parseUnsigned(&field, ',', &number) ||
        !parseSigned(&field, ',', &headingCdeg) ||
        !parseSigned(&field, ',', &rateCdps) ||
        !parseUnsigned(&field, ',', &referenceEpoch) ||
        !parseUnsigned(&field, '\0', &allowBits)) return false;
    if (number == 0 || referenceEpoch == 0 || allowBits > 3UL ||
        headingCdeg < -18000 || headingCdeg > 18000 ||
        rateCdps < -25000 || rateCdps > 25000) return false;
    *sequence = (uint32_t)number;
    *heading = headingCdeg * 0.01f;
    *rate = rateCdps * 0.01f;
    *epoch = (uint32_t)referenceEpoch;
    *allow = (uint8_t)allowBits;
    return true;
  }

  void handleWallLine(const char* line, uint32_t now) {
    char copy[96];
    const size_t length = strnlen(line, sizeof(copy));
    if (length >= sizeof(copy)) {
      ++wallRejected_;
      return;
    }
    memcpy(copy, line, length + 1U);
    char* field = checkedFields(copy);
    unsigned long acquisition = 0, ageMs = 0, flags = 0;
    long leftMm = 0, rightMm = 0, headingCdeg = 0;
    if (field == nullptr ||
        !parseUnsigned(&field, ',', &acquisition) || !parseUnsigned(&field, ',', &ageMs) ||
        !parseUnsigned(&field, ',', &flags) || !parseSigned(&field, ',', &leftMm) ||
        !parseSigned(&field, ',', &rightMm) || !parseSigned(&field, '\0', &headingCdeg) ||
        flags == 0 || flags > 7UL || ageMs > WALL_MAX_AGE_MS ||
        leftMm < 0 || leftMm > 4000 || rightMm < 0 || rightMm > 4000 ||
        headingCdeg < -4500 || headingCdeg > 4500 ||
        ((flags & WALL_LEFT) && leftMm == 0) || ((flags & WALL_RIGHT) && rightMm == 0)) {
      ++wallRejected_;
      return;
    }
    // A repeated scan earns nothing; an older one arriving late is ignored.
    if (haveWallAcquisition_ && (int32_t)((uint32_t)acquisition - lastWallAcquisition_) <= 0) return;
    haveWallAcquisition_ = true;
    lastWallAcquisition_ = (uint32_t)acquisition;
    ++wallLines_;
    wall_.valid = true;
    wall_.acquisition = (uint32_t)acquisition;
    wall_.scanMs = now - (uint32_t)ageMs;
    wall_.flags = (uint8_t)flags;
    wall_.leftCm = leftMm * 0.1f;
    wall_.rightCm = rightMm * 0.1f;
    wall_.headingDeg = headingCdeg * 0.01f;
    if (straightActive_ && centering_ && WALL_FUSION_ENABLED) fuseWall(now);
  }

  void fuseWall(uint32_t now) {
    float headingThen = 0.0f;
    if (!headingAnchored_ || !headingAt(wall_.scanMs, &headingThen)) {
      ++wallRejected_;
      return;
    }
    ++segmentWallScans_;
    if (wall_.flags & WALL_HEADING) {
      const float measured = headingThen - wall_.headingDeg;
      if (fabsf(wrap180(measured - routeTarget_)) > AXIS_MAX_FROM_ROUTE_DEG) {
        ++wallRejected_;  // not the hallway this route is driving along
      } else if (!axisKnown_) {
        axisDeg_ = measured;
        axisAtMs_ = wall_.scanMs;
        axisKnown_ = true;
        axisRejects_ = 0;
        ++segmentAxisUpdates_;
      } else {
        const float predicted = axisAt(wall_.scanMs);
        const float innovation = wrap180(measured - predicted);
        if (fabsf(innovation) <= AXIS_GATE_DEG) {
          const float dt = fmaxf(0.05f, (float)(int32_t)(wall_.scanMs - axisAtMs_) * 0.001f);
          axisDeg_ = predicted + AXIS_GAIN * innovation;
          driftDps_ = constrain(driftDps_ + DRIFT_GAIN * innovation / dt, -MAX_DRIFT_DPS, MAX_DRIFT_DPS);
          axisAtMs_ = wall_.scanMs;
          axisRejects_ = 0;
          ++segmentAxisUpdates_;
        } else if (++axisRejects_ >= AXIS_RESEED_REJECTS) {
          axisDeg_ = measured;
          axisAtMs_ = wall_.scanMs;
          axisRejects_ = 0;
          ++axisReseeds_;
        } else {
          ++wallRejected_;
        }
      }
    }
    float measuredLateral = 0.0f;
    bool haveLateral = false;
    const bool left = (wall_.flags & WALL_LEFT) != 0, right = (wall_.flags & WALL_RIGHT) != 0;
    if (left && right) {
      const float width = wall_.leftCm + wall_.rightCm;
      if (width >= WIDTH_MIN_CM && width <= WIDTH_MAX_CM) {
        if (!widthKnown_) {
          widthCm_ = width;
          widthKnown_ = true;
        } else if (fabsf(width - widthCm_) <= WIDTH_GATE_CM) {
          widthCm_ += WIDTH_GAIN * (width - widthCm_);
        }
        measuredLateral = 0.5f * (wall_.rightCm - wall_.leftCm);
        haveLateral = true;
      }
    } else if (left && widthKnown_) {
      measuredLateral = 0.5f * widthCm_ - wall_.leftCm;
      haveLateral = true;
    } else if (right && widthKnown_) {
      measuredLateral = wall_.rightCm - 0.5f * widthCm_;
      haveLateral = true;
    }
    if (!haveLateral) return;
    if (!lateralKnown(now)) {
      lateralCm_ = measuredLateral;
      lateralKnown_ = true;
      lateralRejects_ = 0;
    } else if (fabsf(measuredLateral - lateralCm_) <= LATERAL_GATE_CM) {
      lateralCm_ += LATERAL_GAIN * (measuredLateral - lateralCm_);
      lateralRejects_ = 0;
    } else if (++lateralRejects_ >= LATERAL_RESEED_REJECTS) {
      lateralCm_ = measuredLateral;
      lateralRejects_ = 0;
    } else {
      return;  // a single odd scan neither moves nor refreshes the estimate
    }
    lateralAtMs_ = now;
  }

  float speedRatio() const {
    const uint32_t base = (base1_ + base2_) / 2U;
    return probeBaseSpeed_ > 0 ? (float)base / (float)probeBaseSpeed_ : 1.0f;
  }
  float lookaheadSteps() const {
    return fmaxf(1.0f, 0.5f * (float)(base1_ + base2_) * PATH_LOOKAHEAD_S);
  }

  // FastAccelStepper reports ramp-generator speed, not wheel encoder data.
  // Without wall fusion, integrating forward travel * sin(yaw error) estimates
  // displacement from the segment's line, so a disturbed heading is followed by
  // a return to it. With fusion, the same travel carries the measured lateral
  // position between scans.
  void servicePathEstimate(uint32_t now) {
    if (now - lastPathMs_ < CONTROL_INTERVAL_MS) return;
    const uint32_t elapsedMs = now - lastPathMs_;
    lastPathMs_ = now;
    if (!m1_ || !m2_ || !headingReady(now) || elapsedMs > 60) {
      pathReady_ = false;
      return;
    }
    if (!pathReady_) {
      pathReady_ = true;
      return;
    }
    const float speed1 = fabsf((float)m1_->getCurrentSpeedInMilliHz(false)) * 0.001f;
    const float speed2 = fabsf((float)m2_->getCurrentSpeedInMilliHz(false)) * 0.001f;
    const float forwardSteps = 0.5f * (speed1 + speed2) * (elapsedMs * 0.001f);
    crossTrackSteps_ += forwardSteps * sinf((continuous_ - routeTarget_) * DEG_TO_RAD);
    const float limit = 0.5f * (float)(base1_ + base2_) * MAX_CROSS_TRACK_S;
    crossTrackSteps_ = constrain(crossTrackSteps_, -limit, limit);
    if (lateralKnown(now)) {
      const float axis = axisKnown_ ? axisAt(now) : routeTarget_;
      lateralCm_ += forwardSteps / STEPS_PER_CM * sinf((continuous_ - axis) * DEG_TO_RAD);
    }
  }

  // Centring term with deadband and hysteresis, slewed so it is never a step.
  void serviceCentering(uint32_t now, float dt) {
    float desired = 0.0f;
    if (lateralKnown(now)) {
      const float magnitude = fabsf(lateralCm_);
      if (!centerActive_ && magnitude > CENTER_ENGAGE_CM) centerActive_ = true;
      else if (centerActive_ && magnitude < CENTER_RELEASE_CM) centerActive_ = false;
      if (centerActive_) {
        const float excess = copysignf(fmaxf(magnitude - CENTER_DEADBAND_CM, 0.0f), lateralCm_);
        desired = constrain(-atanf(excess / CENTER_LOOKAHEAD_CM) * RAD_TO_DEG,
                            -MAX_CENTER_DEG, MAX_CENTER_DEG);
      }
    } else {
      centerActive_ = false;
    }
    const float step = CENTER_SLEW_DEG_PER_S * dt;
    centerTargetDeg_ += constrain(desired - centerTargetDeg_, -step, step);
  }

  void clearRateAverage(uint32_t now) {
    probeStartedMs_ = now;
    probeLastSequence_ = lastSequence_;
    rateSum_ = rateSumSquares_ = 0.0f;
    rateCount_ = 0;
  }

  void abortProbe(uint32_t now) {
    if (probeState_ != ProbeState::BASELINE && probeState_ != ProbeState::PULSE) return;
    if (probeState_ == ProbeState::PULSE && probeAttempts_ > 0) --probeAttempts_;
    probeState_ = ProbeState::WAITING;
    clearRateAverage(now);
  }

  bool setSpeed(FastAccelStepper* motor, uint32_t speed, uint32_t acceleration) {
    if (speed == 0 || motor->setSpeedInHz(speed) != 0) return false;
    motor->setAcceleration(acceleration);
    motor->applySpeedAcceleration();
    return true;
  }

  bool applySpeeds(uint32_t speed1, uint32_t speed2, uint32_t acceleration) {
    if (speed1 != applied1_) {
      if (!setSpeed(m1_, speed1, acceleration)) return false;
      applied1_ = speed1;
    }
    if (speed2 != applied2_) {
      if (!setSpeed(m2_, speed2, acceleration)) return false;
      applied2_ = speed2;
    }
    return true;
  }

  // Slow motor 1 briefly while cruising. The change in gyro rate gives the
  // physical motor side and deg/s per fractional wheel-speed cut. Not needed
  // once a turn has proved the motor sides: the spin geometry gives the gain.
  void serviceProbe(uint32_t now, uint32_t acceleration) {
    if (probeState_ == ProbeState::READY || probeState_ == ProbeState::FAILED ||
        turnSideConfirmed_ ||
        !m1_->isRunning() || !m2_->isRunning() || !streamReady(now)) return;
    if (probeState_ == ProbeState::WAITING) {
      if (now - driveStartedMs_ < DRIVE_SETTLE_MS || now - probeStartedMs_ < 800 ||
          remaining_ < probeReserve_) return;
      probeState_ = ProbeState::BASELINE;
      clearRateAverage(now);
      return;
    }
    if (lastSequence_ != probeLastSequence_) {
      probeLastSequence_ = lastSequence_;
      // Ignore the first part of each window while motor speed and filter settle.
      const uint32_t settleMs = probeState_ == ProbeState::PULSE ? 150 : 100;
      if (now - probeStartedMs_ >= settleMs) {
        rateSum_ += filteredRate_;
        rateSumSquares_ += filteredRate_ * filteredRate_;
        ++rateCount_;
      }
    }
    if (probeState_ == ProbeState::BASELINE && now - probeStartedMs_ >= BASELINE_MS) {
      if (rateCount_ < 8) {
        probeState_ = ProbeState::WAITING;
        return;
      }
      baselineRate_ = rateSum_ / rateCount_;
      const float variance = fmaxf(0.0f, rateSumSquares_ / rateCount_ - baselineRate_ * baselineRate_);
      baselineStdErrorSq_ = variance / rateCount_;
      activeProbeCut_ = probeAttempts_ == 0 ? FIRST_PROBE_CUT : SECOND_PROBE_CUT;
      probeBaseSpeed_ = (base1_ + base2_) / 2U;
      const uint32_t probeSpeed = (uint32_t)lroundf(base1_ * (1.0f - activeProbeCut_));
      if (!setSpeed(m1_, probeSpeed, acceleration)) {
        probeState_ = ProbeState::FAILED;
        return;
      }
      applied1_ = probeSpeed;
      ++probeAttempts_;
      probeState_ = ProbeState::PULSE;
      clearRateAverage(now);
    } else if (probeState_ == ProbeState::PULSE && now - probeStartedMs_ >= PROBE_MS) {
      if (!setSpeed(m1_, base1_, acceleration)) {
        probeState_ = ProbeState::FAILED;
        return;
      }
      applied1_ = base1_;
      const float probeRate = rateCount_ ? rateSum_ / rateCount_ : 0.0f;
      const float variance = rateCount_ ? fmaxf(0.0f,
          rateSumSquares_ / rateCount_ - probeRate * probeRate) : 0.0f;
      const float standardError = rateCount_ ?
          sqrtf(baselineStdErrorSq_ + variance / rateCount_) : 1000.0f;
      const float response = probeRate - baselineRate_;
      const float measuredGain = fabsf(response) / activeProbeCut_;
      if (rateCount_ >= 8 &&
          fabsf(response) >= fmaxf(MIN_PROBE_RESPONSE_DPS, 4.0f * standardError) &&
          measuredGain >= 10.0f && measuredGain <= 500.0f) {
        // Slowing motor 1 made positive (left) yaw iff it is the left wheel.
        motor1PositiveYaw_ = response > 0.0f;
        if (!turnSideConfirmed_) motor1IsLeft_ = motor1PositiveYaw_;
        yawGain_ = measuredGain;
        trimDps_ = constrain(baselineRate_, -MAX_TRIM_CUT * measuredGain,
                             MAX_TRIM_CUT * measuredGain);
        probeState_ = ProbeState::READY;
        lastControlMs_ = now;
        Serial.printf("[HEADING] PROBE ok: hold ON, gain %.0f deg/s per cut, motor1 = %s wheel\n",
                      measuredGain, motor1PositiveYaw_ ? "left" : "right");
      } else {
        probeState_ = probeAttempts_ >= 2 ? ProbeState::FAILED : ProbeState::WAITING;
        probeStartedMs_ = now;
        if (probeState_ == ProbeState::FAILED)
          Serial.println("[HEADING] PROBE failed: straights run WITHOUT heading hold");
      }
    }
  }

  // Computes target1_/target2_. Returns false when no steering applies.
  bool serviceSteering(uint32_t now) {
    const bool probeReady = probeState_ == ProbeState::READY;
    if ((!probeReady && !turnSideConfirmed_) || !headingReady(now)) return false;
    if (now - lastControlMs_ < CONTROL_INTERVAL_MS) {
      target1_ = applied1_ ? applied1_ : base1_;
      target2_ = applied2_ ? applied2_ : base2_;
      return true;
    }
    const float dt = fminf((now - lastControlMs_) * 0.001f, 0.04f);
    lastControlMs_ = now;
    // Probe-measured gain, or the spin geometry: slowing one wheel by a fraction
    // c of base speed V turns the chassis at 0.5 * c * V / steps-per-degree.
    const float baseSpeed = 0.5f * (float)(base1_ + base2_);
    const float gain = probeReady ? fmaxf(yawGain_ * speedRatio(), 1.0f)
                                  : fmaxf(0.5f * baseSpeed / stepsPerDeg_, 1.0f);
    const bool motor1Positive = probeReady ? motor1PositiveYaw_ : motor1IsLeft_;

    float returnHeadingDeg = 0.0f;
    if (lidarSteering()) {
      serviceCentering(now, dt);  // the walls own the lateral position
    } else {
      // Lead predicts where the heading is going; the slow trim cancels steady
      // wheel mismatch without an on/off threshold that weaves on long runs.
      returnHeadingDeg = constrain(-atanf(crossTrackSteps_ / lookaheadSteps()) * RAD_TO_DEG,
                                   -MAX_RETURN_HEADING_DEG, MAX_RETURN_HEADING_DEG);
    }
    const float targetHeading = steeringTarget(now);
    const float errorDeg = constrain(continuous_ - targetHeading - returnHeadingDeg, -30.0f, 30.0f);
    const float predictedDeg = errorDeg + LOOKAHEAD_S * filteredRate_;
    const float rawCut = (HEADING_GAIN_PER_S * predictedDeg + trimDps_) / gain;
    // + cut steers RIGHT, - cut steers LEFT. Unsafe nearest-object clearance on
    // a side vetoes steering toward it; it never adds a turn the other way.
    const bool rightVetoed = (allow_ & ALLOW_RIGHT) == 0;
    const bool leftVetoed = (allow_ & ALLOW_LEFT) == 0;
    const bool vetoed = (rawCut > 0.0f && rightVetoed) || (rawCut < 0.0f && leftVetoed);
    if (!vetoed && (fabsf(rawCut) < MAX_SPEED_CUT || errorDeg * rawCut < 0.0f)) {
      trimDps_ += TRIM_GAIN_PER_S2 * errorDeg * dt;
      trimDps_ = constrain(trimDps_, -MAX_TRIM_CUT * gain, MAX_TRIM_CUT * gain);
    }
    float targetCut = constrain((HEADING_GAIN_PER_S * predictedDeg + trimDps_) / gain,
                                -MAX_SPEED_CUT, MAX_SPEED_CUT);
    if ((targetCut > 0.0f && rightVetoed) || (targetCut < 0.0f && leftVetoed)) targetCut = 0.0f;
    if ((appliedCut_ > 0.0f && rightVetoed) || (appliedCut_ < 0.0f && leftVetoed)) appliedCut_ = 0.0f;
    appliedCut_ += constrain(targetCut - appliedCut_, -MAX_CUT_CHANGE, MAX_CUT_CHANGE);
    // At most one wheel slows at a time.
    const bool slowMotor1 = appliedCut_ > 0.0f ? !motor1Positive : motor1Positive;
    const float keep = 1.0f - fabsf(appliedCut_);
    const bool none = fabsf(appliedCut_) < 0.0001f;
    target1_ = none || !slowMotor1 ? base1_ : (uint32_t)lroundf(base1_ * keep);
    target2_ = none || slowMotor1 ? base2_ : (uint32_t)lroundf(base2_ * keep);
    return true;
  }
};
