#include "stubs/Arduino.h"
#include "stubs/Wire.h"
#include "../../BLE_Receiver-Final/MpuHeading.h"
#include <cstring>

static void check(bool ok, const char* label) {
  if (!ok) { std::fprintf(stderr, "FAIL %s\n", label); std::exit(1); }
  std::printf("PASS %s\n", label);
}
static void sample(MpuHeading& mpu, int16_t raw, uint32_t elapsedMs = 10) {
  Wire.setGyroZ(raw); fakeNow += elapsedMs; mpu.poll();
}
int main() {
  // A stationary MPU may have substantial zero-rate bias. It must be measured,
  // not mistaken for real yaw throughout the next straight segment.
  fakeNow = 1000;
  MpuHeading biased;
  Wire = TwoWire{};
  Wire.registers[0x3a] = 0;
  check(!biased.begin(8, 9), "repeated stale register contents cannot calibrate as new samples");
  Wire = TwoWire{};
  Wire.setGyroZ(1310); // +10 degrees/s zero-rate offset
  check(biased.begin(8, 9), "stationary gyro offset within datasheet limits calibrates");
  biased.requestReference(); sample(biased, 1310, 100);
  for (int i = 0; i < 1000; ++i) sample(biased, 1310);
  check(biased.valid() && std::fabs(biased.headingDeg()) < 0.05f,
        "calibrated stationary bias does not create fictitious heading corrections");
  check(std::fabs(biased.rateDps()) < 0.001f,
        "stationary calibrated gyro offset is removed from the reported yaw rate");
  sample(biased, 1965);
  check(std::fabs(biased.rateDps() - 5.0f) < 0.001f,
        "reported positive yaw rate subtracts the measured ten-degree boot bias");
  sample(biased, 655);
  check(std::fabs(biased.rateDps() + 5.0f) < 0.001f,
        "reported closing yaw rate remains signed after boot-bias subtraction");
  sample(biased, 1310);
  check(std::fabs(biased.rateDps()) < 0.001f,
        "returning to the calibrated stationary reading immediately reports zero rate");
  Wire = TwoWire{};
  Wire.registers[0x3f] = 0; // no gravity: motion/disconnected data
  check(!biased.begin(8, 9), "boot calibration rejects missing gravity instead of arming bad yaw");
  Wire = TwoWire{};
  fakeNow = 1000;
  MpuHeading mpu;
  check(mpu.begin(8, 9), "MPU6050 is detected and configured through Wire");
  check(Wire.sda == 8 && Wire.scl == 9 && Wire.clockHz == 100000 && Wire.timeoutMs == 25,
        "MPU I2C uses explicit S3 pins and a bounded transaction timeout");
  check(Wire.registers[0x1b] == 0 && Wire.registers[0x1a] == 4 && Wire.registers[0x19] == 9
        && Wire.registers[0x38] == 1,
        "MPU gyro uses reference sketch range, filter and 100 Hz sample rate");
  mpu.requestReference();
  check(!mpu.valid() && mpu.rateDps() == 0,
        "requesting FRONT exposes no rate until a fresh reference sample arrives");
  sample(mpu, 1310, 100);
  check(mpu.valid() && mpu.headingDeg() == 0, "first valid sample establishes FRONT without integrating the startup gap");
  check(std::fabs(mpu.rateDps() - 10.0f) < 0.001f,
        "the first reference reports its current gyro rate without integrating the startup gap");
  for (int i = 0; i < 25; ++i) sample(mpu, 1310);
  check(mpu.direction() == 1 && std::fabs(mpu.headingDeg() - 2.5f) < 0.01f,
        "positive gyro Z integrates to a persistent left heading deviation");
  sample(mpu, 0);
  check(mpu.rateDps() == 0,
        "a fresh zero-rate sample is exposed while the heading deviation remains");
  for (int i = 0; i < 10; ++i) sample(mpu, 0);
  check(mpu.direction() == 1, "stopping rotation does not erase heading deviation");
  for (int i = 0; i < 10; ++i) sample(mpu, -1310);
  check(mpu.direction() == 1, "left heading is retained above the one-degree release threshold");
  check(std::fabs(mpu.rateDps() + 10.0f) < 0.001f && mpu.headingDeg() > 0,
        "closing rotation is reported separately from the still-positive heading error");
  for (int i = 0; i < 10; ++i) sample(mpu, -1310);
  check(mpu.direction() == 0, "returning within one degree releases heading correction");
  mpu.requestReference();
  check(mpu.rateDps() == 0, "a new segment discards the previous segment's closing rate");
  sample(mpu, -1310);
  for (int i = 0; i < 25; ++i) sample(mpu, -1310);
  check(mpu.direction() == -1, "negative signed gyro data produces right heading deviation");
  sample(mpu, -1310, 101);
  check(!mpu.valid() && !mpu.referencePending(), "a sampling gap over 100 ms invalidates heading until a new reference");
  check(mpu.rateDps() == 0, "a sampling-gap fault discards the previously nonzero yaw rate");
  sample(mpu, 0);
  check(!mpu.valid(), "later good reads cannot silently recover a lost heading");
  mpu.requestReference(); sample(mpu, 0);
  check(mpu.valid() && mpu.headingDeg() == 0, "new segment reference explicitly recovers tracking");
  Wire.registers[0x3a] = 0;
  for (int i = 0; i < 11; ++i) sample(mpu, 0);
  check(!mpu.valid(), "missing DATA_RDY cannot keep frozen gyro data fresh");
  Wire.registers[0x3a] = 1; mpu.requestReference(); sample(mpu, 1310);
  // Stepper EMI can corrupt one I2C transaction in a long run. It must not end
  // the segment, but the existing 100 ms blind budget stays the hard limit.
  Wire.readFailure = true; sample(mpu, 0);
  check(mpu.valid(), "one failed I2C transaction does not lose the heading reference");
  Wire.readFailure = false; sample(mpu, 1310);
  check(mpu.valid() && mpu.headingDeg() > 0.0f, "the next good sample resumes integration after a transient read failure");
  mpu.requestReference(); sample(mpu, 1310);
  Wire.readFailure = true;
  for (int i = 0; i < 11; ++i) sample(mpu, 0);
  check(!mpu.valid() && !mpu.referencePending(), "sustained I2C failure invalidates heading within the 100 ms gap budget");
  check(mpu.rateDps() == 0, "an I2C fault exposes no rate from the preceding valid sample");
  Wire.readFailure = false; sample(mpu, 0);
  check(!mpu.valid(), "good reads after a sustained failure cannot silently recover heading");
  mpu.requestReference(); sample(mpu, 32767);
  check(!mpu.valid(), "saturated gyro data cannot establish a heading reference");
  mpu.requestReference(); sample(mpu, 0); sample(mpu, 32767);
  check(mpu.valid() && std::fabs(mpu.headingDeg()) < 0.01f, "one glitched saturated sample is rejected without integrating it");
  for (int i = 0; i < 11; ++i) sample(mpu, 32767);
  check(!mpu.valid(), "sustained gyro saturation invalidates heading");
  mpu.requestReference(); sample(mpu, 0); Wire.shortRead = 1; sample(mpu, 13100); Wire.shortRead = 0;
  check(mpu.valid() && std::fabs(mpu.headingDeg()) < 0.01f, "partial gyro reads cannot be integrated");
  for (int i = 0; i < 11; ++i) { Wire.shortRead = 1; sample(mpu, 0); }
  Wire.shortRead = 0;
  check(!mpu.valid(), "sustained partial reads invalidate heading");
  // A pending reference on a frozen sensor must not wait forever: the bridge
  // lets path packets through while a new reference is settling.
  Wire.registers[0x3a] = 0; mpu.requestReference();
  for (int i = 0; i < 12; ++i) sample(mpu, 0);
  check(!mpu.valid() && !mpu.referencePending(), "a reference that never gets fresh data becomes a fault");
  Wire.registers[0x3a] = 1;
  Wire = TwoWire{}; Wire.deviceAddress = 0x69;
  check(mpu.begin(8, 9), "MPU6050 is detected with AD0 high at address 0x69");
  Wire.registers[0x75] = 0x70;
  check(!mpu.begin(8, 9), "an unexpected WHO_AM_I is rejected");
  Wire = TwoWire{};
  Wire = TwoWire{};
  check(mpu.begin(8, 9, true), "reversed mounting can explicitly reverse gyro Z");
  mpu.requestReference(); sample(mpu, 1310, 100);
  for (int i = 0; i < 25; ++i) sample(mpu, 1310);
  check(mpu.direction() == -1, "configured gyro sign reversal reverses heading direction");
  check(std::fabs(mpu.rateDps() + 10.0f) < 0.001f,
        "configured mounting reversal also reverses the reported yaw-rate sign");
  mpu.invalidate();
  check(!mpu.valid() && !mpu.referencePending(), "segment end disables the heading reference");
  check(mpu.rateDps() == 0, "explicit segment invalidation clears its nonzero yaw rate");

  Wire = TwoWire{};
  check(mpu.begin(8, 9), "normal mounting is restored for yaw-rate freshness checks");
  mpu.requestReference(); sample(mpu, 1310, 100);
  fakeNow += 100;
  check(mpu.valid() && std::fabs(mpu.rateDps() - 10.0f) < 0.001f,
        "yaw rate follows the existing inclusive hundred-millisecond freshness boundary");
  fakeNow += 1;
  check(!mpu.valid() && mpu.rateDps() == 0,
        "an expired yaw rate is hidden even before the next sensor poll detects the gap");
  mpu.poll();
  mpu.requestReference(); Wire.registers[0x3a] = 0;
  sample(mpu, -655);
  check(mpu.referencePending() && !mpu.valid() && mpu.rateDps() == 0,
        "stale DATA_RDY cannot resurrect an old yaw rate while FRONT is pending");
  Wire.registers[0x3a] = 1;
  sample(mpu, -655);
  check(mpu.valid() && mpu.headingDeg() == 0 && std::fabs(mpu.rateDps() + 5.0f) < 0.001f,
        "a fresh reference exposes only its new signed rate after the stale-data gap");
  // Test micros() wrapping across 2^32 without manufacturing a sampling gap.
  Wire = TwoWire{};
  check(mpu.begin(8, 9), "normal gyro orientation is restored for the rollover check");
  fakeNow = 4294960; mpu.requestReference(); sample(mpu, 1310);
  sample(mpu, 1310);
  check(mpu.valid() && std::fabs(mpu.headingDeg() - 0.1f) < 0.01f,
        "gyro integration survives the micros clock rollover");

  // On battery nobody can read the USB console or type R, so the reason has to be
  // reportable and a held robot has to be able to recover by itself.
  const auto pump = [](MpuHeading& target, uint32_t ms) {
    bool completed = false;
    for (uint32_t elapsed = 0; elapsed < ms; elapsed += 10) {
      fakeNow += 10;
      completed = target.serviceRecovery() || completed;
    }
    return completed;
  };
  const uint32_t attemptMs = 100 + 20 +
      MpuHeading::CALIBRATION_SAMPLES * MpuHeading::SAMPLE_INTERVAL_MS;
  const uint32_t fullCycleMs = MpuHeading::RECOVERY_RETRY_MS + attemptMs + 200;
  const auto problemIs = [](const MpuHeading& target, const char* text) {
    return std::strcmp(target.problem(), text) == 0;
  };

  fakeNow = 20000; Wire = TwoWire{}; Wire.registers[0x3a] = 0;
  MpuHeading held;
  check(!held.begin(8, 9) && problemIs(held, "NOT_FRESH"),
        "a failed boot calibration records that the sensor gave no fresh data");
  const uint32_t beforeService = fakeNow;
  for (int i = 0; i < 500; ++i) held.serviceRecovery();
  check(fakeNow == beforeService && !held.ready(), "recovery never blocks the main loop");
  check(!pump(held, fullCycleMs) && !held.ready(), "a sensor that still has no fresh data stays not ready");
  Wire.registers[0x3a] = 1; Wire.setGyroZ(655);
  check(pump(held, fullCycleMs) && held.ready() && problemIs(held, "OK"),
        "a failed boot calibration recovers without a reset once the sensor is good");
  check(std::fabs(held.biasDps() - 5.0f) < 0.001f, "recovery measures a new gyro bias");

  Wire = TwoWire{}; Wire.busReady = false;
  check(!held.begin(8, 9) && problemIs(held, "BUS"), "an unavailable I2C bus is named");
  Wire = TwoWire{}; Wire.deviceAddress = 0x70;
  check(!held.begin(8, 9) && problemIs(held, "NOT_FOUND"), "a missing MPU is named");
  Wire = TwoWire{}; Wire.registers[0x3f] = 0;
  check(!held.begin(8, 9) && problemIs(held, "GRAVITY"), "missing gravity is named");
  Wire = TwoWire{}; Wire.setGyroZ(32767);
  check(!held.begin(8, 9) && problemIs(held, "SATURATED"), "a saturated sensor is named");
  Wire = TwoWire{}; Wire.setGyroZ(3276);
  check(!held.begin(8, 9) && problemIs(held, "GYRO_OFFSET"), "an out-of-range gyro offset is named");

  // A runtime fault happens with the motors turning, so it must not measure a new
  // "stationary" bias. It reconfigures the sensor and reuses the boot bias.
  Wire = TwoWire{}; fakeNow = 40000;
  MpuHeading resetSensor;
  Wire.setGyroZ(655);
  check(resetSensor.begin(8, 9), "recovery fixture starts healthy with a 5 deg/s bias");
  resetSensor.requestReference(); sample(resetSensor, 655, 100);
  for (int i = 0; i < 50; ++i) sample(resetSensor, 1965);  // +10 deg/s for 0.5 s
  const float beforeFault = resetSensor.headingDeg();
  resetSensor.beginRecovery();
  check(!resetSensor.ready() && !resetSensor.valid(), "starting recovery withdraws MPU readiness at once");
  Wire.registers[0x6b] = 0x40; // what a supply dip leaves: the MPU reset and asleep
  Wire.setGyroZ(655);
  check(pump(resetSensor, 200) && resetSensor.ready() && Wire.registers[0x6b] == 0x01,
        "warm recovery reconfigures a sensor that reset itself");
  check(resetSensor.valid() && std::fabs(resetSensor.headingDeg() - beforeFault) < 0.01f &&
        std::fabs(resetSensor.biasDps() - 5.0f) < 0.001f,
        "a recovery inside 500 ms keeps the heading reference and the boot bias");
  resetSensor.beginRecovery(); fakeNow += 600;
  check(pump(resetSensor, 200) && resetSensor.ready() && !resetSensor.valid() && !resetSensor.referencePending(),
        "a longer blind time recovers the sensor but does not invent a heading reference");

  // The existing limit stays exactly 197 counts (1.5 degrees/s): retrying must not
  // become a way for a moving robot to calibrate.
  const auto alternate = [&](MpuHeading& target, int16_t high, uint32_t ms) {
    for (uint32_t elapsed = 0; elapsed < ms; elapsed += 10) {
      Wire.setGyroZ((elapsed / 10) % 2 ? high : 0); fakeNow += 10; target.serviceRecovery();
    }
  };
  Wire = TwoWire{}; Wire.registers[0x3a] = 0; fakeNow = 60000;
  MpuHeading moving;
  check(!moving.begin(8, 9), "motion fixture has never measured a bias");
  Wire.registers[0x3a] = 1;
  alternate(moving, 198, fullCycleMs);
  check(!moving.ready() && problemIs(moving, "MOVING"), "a rate spread over the limit fails calibration and is named");
  alternate(moving, 198, 3 * fullCycleMs);
  check(!moving.ready(), "repeated retries on a moving robot never calibrate");
  alternate(moving, 197, 2 * fullCycleMs);
  check(moving.ready(), "a rate spread at the unchanged limit still calibrates");

  // A yaw step no chassis can make is a sensor fault. Never steer on it.
  Wire = TwoWire{}; fakeNow = 80000;
  MpuHeading jump;
  check(jump.begin(8, 9), "yaw-jump fixture starts healthy");
  jump.requestReference(); sample(jump, 31000, 100);
  sample(jump, 31000, 95);
  check(!jump.valid() && std::strcmp(jump.lastFault(), "JUMP") == 0,
        "a yaw step over MAX_YAW_JUMP_DEG between two samples faults instead of integrating");
  jump.requestReference(); sample(jump, 0);
  const float beforeSpin = jump.headingDeg();
  for (int i = 0; i < 40; ++i) sample(jump, 26200);  // a real 200 degrees/s spin
  check(jump.valid() && jump.headingDeg() - beforeSpin > 70.0f,
        "the fastest rate the sensor can report, sampled normally, is not a jump");

  // Each runtime fault says which check ended it.
  Wire = TwoWire{}; fakeNow = 90000;
  MpuHeading named;
  check(named.begin(8, 9), "fault-reason fixture starts healthy");
  named.requestReference(); sample(named, 0, 100);
  Wire.registers[0x3a] = 0;
  for (int i = 0; i < 12; ++i) sample(named, 0);
  check(!named.valid() && std::strcmp(named.lastFault(), "NOT_FRESH") == 0, "a fault from missing DATA_RDY is named");
  Wire.registers[0x3a] = 1; named.requestReference(); sample(named, 0);
  Wire.readFailure = true;
  for (int i = 0; i < 12; ++i) sample(named, 0);
  check(!named.valid() && std::strcmp(named.lastFault(), "READ") == 0, "a fault from failed I2C reads is named");
  Wire.readFailure = false; named.requestReference(); sample(named, 0);
  for (int i = 0; i < 12; ++i) sample(named, 32767);
  check(!named.valid() && std::strcmp(named.lastFault(), "SATURATED") == 0, "a fault from saturated data is named");
  named.requestReference(); sample(named, 0); sample(named, 0, 101);
  check(!named.valid() && std::strcmp(named.lastFault(), "GAP") == 0, "a fault from a late sample is named");

  // ---- 2026-11-11 navigation review ----
  // A tilted mount: yaw is the gyro vector projected on measured gravity. Reading
  // gyro Z alone would report cos(20 deg) = 94% of a turn about the vertical.
  const auto setWord = [](uint8_t reg, int16_t value) {
    Wire.registers[reg] = (uint16_t)value >> 8; Wire.registers[reg + 1] = (uint16_t)value & 0xff;
  };
  Wire = TwoWire{}; fakeNow = 100000;
  setWord(0x3d, 5604); setWord(0x3f, 15396);  // gravity 20 degrees off sensor Z, toward +Y
  MpuHeading tilted;
  check(tilted.begin(8, 9) && std::fabs(tilted.tiltDeg() - 20.0f) < 0.2f,
        "boot calibration measures the mount tilt from gravity");
  tilted.requestReference(); sample(tilted, 0, 100);
  setWord(0x45, 448); setWord(0x47, 1231);    // 10 deg/s about the vertical, in sensor axes
  fakeNow += 10; tilted.poll();
  check(std::fabs(tilted.rateDps() - 10.0f) < 0.05f,
        "a tilted sensor reports the full yaw rate about the vertical");
  for (int i = 0; i < 899; ++i) { fakeNow += 10; tilted.poll(); }
  check(std::fabs(tilted.headingDeg() - 90.0f) < 0.5f,
        "a 90 degree turn integrates to 90 degrees on a tilted mount");

  // Bias drifts after boot. While the caller holds the robot still, it is measured
  // again without touching the heading reference.
  Wire = TwoWire{}; fakeNow = 120000;
  MpuHeading drifting;
  check(drifting.begin(8, 9), "bias-refresh fixture calibrates at zero bias");
  drifting.requestReference(); sample(drifting, 0, 100);
  for (int i = 0; i < 300; ++i) sample(drifting, 13);  // 0.1 deg/s of new drift, robot moving
  check(drifting.biasRefreshes() == 0 && std::fabs(drifting.rateDps() - 0.0992f) < 0.01f,
        "drift is never re-measured unless the caller says the robot is held still");
  drifting.setStill(true);
  const float headingBeforeRefresh = drifting.headingDeg();
  for (int i = 0; i < MpuHeading::STILL_SAMPLES; ++i) sample(drifting, 13);
  check(drifting.biasRefreshes() == 1 && std::fabs(drifting.biasDps() - 0.0992f) < 0.01f,
        "1.5 s of still data re-measures the bias");
  sample(drifting, 13);
  check(drifting.valid() && std::fabs(drifting.rateDps()) < 0.01f &&
        std::fabs(drifting.headingDeg() - headingBeforeRefresh) < 0.2f,
        "the refreshed bias stops the drift and keeps the heading reference");
  for (int i = 0; i < MpuHeading::STILL_SAMPLES - 10; ++i) sample(drifting, 13);
  sample(drifting, 300);  // a real rotation begins
  for (int i = 0; i < 5; ++i) sample(drifting, 13);
  check(drifting.biasRefreshes() == 1, "rotation inside a still window restarts it");
  for (int i = 0; i < 2 * MpuHeading::STILL_SAMPLES; ++i) sample(drifting, 13 + 80);  // 0.6 deg/s: a slow push
  check(drifting.biasRefreshes() == 1 && std::fabs(drifting.biasDps() - 0.0992f) < 0.01f,
        "a steady rate beyond the drift limit is treated as motion, not bias");
  drifting.setStill(false);
  for (int i = 0; i < 2 * MpuHeading::STILL_SAMPLES; ++i) sample(drifting, 13);
  check(drifting.biasRefreshes() == 1, "leaving the still state stops bias refresh");

  // Each window may move the bias by 0.2 deg/s at most; the total from the boot
  // calibration is capped at 1 deg/s, so repeated windows cannot ratchet it away.
  Wire = TwoWire{}; fakeNow = 140000;
  MpuHeading ratchet;
  check(ratchet.begin(8, 9), "ratchet fixture calibrates at zero bias");
  ratchet.requestReference(); sample(ratchet, 0, 100);
  ratchet.setStill(true);
  for (int step = 1; step <= 9; ++step)
    for (int i = 0; i < MpuHeading::STILL_SAMPLES + 2; ++i) sample(ratchet, (int16_t)(20 * step));
  check(ratchet.biasRefreshes() == 6 && ratchet.biasDps() > 0.85f &&
        ratchet.biasDps() <= MpuHeading::MAX_TOTAL_BIAS_DRIFT_DPS,
        "bias follows slow drift in 0.15 deg/s steps but never passes the 1 deg/s total limit");
}
