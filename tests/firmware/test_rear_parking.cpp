// Parking uses the wall BEHIND the robot. Exercise the actual executor and UART
// parser with the kinematic plant; this cannot prove physical wall alignment.
#pragma GCC optimize("O1")
#include "stubs/Arduino.h"
#include "../../MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.cpp"
#include "../../MAIN_EXECUTOR_CODE/pointsRun.ino"
#include "../../MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.ino"
#include "hallway_sim.h"
#include <climits>

static FastAccelStepper a, b;
static hallway_sim::Plant plant;
static int failures = 0;
static void check(bool ok, const char* label) {
  std::printf("%s %s\n", ok ? "PASS" : "FAIL", label);
  if (!ok) ++failures;
}
static void fresh() {
  tickHook = backgroundHook = nullptr;
  fakeNow = 5000;
  operatorStopMarker = 0;
  operatorEmergencyLatched = operatorManualStartRequired = false;
  operatorSonarEnabled = false;
  a = FastAccelStepper(); b = FastAccelStepper(); stepper1 = &a; stepper2 = &b;
  fullReset(); garbyState = GarbyState::RUNNING;
  heading = HeadingControl();
  plant = hallway_sim::Plant(); plant.walls = false; plant.rearWall = false;
  plant.attach(&a, &b, &ESP_Serial.rx); plant.backing = &backingActive;
  ESP_Serial.rx.clear(); ESP_Serial.tx.clear();
  backgroundHook = [] { plant.tick(); };
  for (int i = 0; i < 400; ++i) { delay(1); pollESP(); }
  beginRouteHeading(); heading.confirmTurnSides();
}
static void rear(uint32_t acquisition, double angle, unsigned age = 0, bool valid = true) {
  char payload[80], line[96];
  std::snprintf(payload, sizeof(payload), "D,%lu,%u,%u,1600,%ld",
                (unsigned long)acquisition, age, valid ? 1U : 0U, std::lround(angle * 100));
  unsigned char sum = 0;
  for (const char* p = payload; *p; ++p) sum ^= (unsigned char)*p;
  std::snprintf(line, sizeof(line), "%s*%02X\n", payload, sum);
  ESP_Serial.rx += line;
}
int main() {
  fresh();
  const long before = a.getCurrentPosition() + b.getCurrentPosition();
  const uint32_t t0 = fakeNow;
  tickHook = [&] { if (fakeNow - t0 > 4000) resetQueued = true; };
  bool ok = backIntoPlace(2000);
  check(!ok && a.getCurrentPosition() + b.getCurrentPosition() == before,
        "no rear-wall measurement: no backward travel, even with GO and MPU");

  fresh();
  const uint32_t started = fakeNow;
  uint32_t scanMs = fakeNow, acq = 0;
  bool backedBeforeAligned = false;
  plant.thetaDeg = 4.0; // gyro's route target is zero, but the rear wall says +4.
  tickHook = [&] {
    if (fakeNow - scanMs >= 174) { scanMs = fakeNow; rear(++acq, plant.thetaDeg); }
    if (a.getCurrentPosition() + b.getCurrentPosition() < -10 && std::fabs(plant.thetaDeg) > 1.1)
      backedBeforeAligned = true;
    if (fakeNow - started > 30000) resetQueued = true;
  };
  ok = backIntoPlace(2000);
  check(ok && !backedBeforeAligned && std::fabs(plant.thetaDeg) <= 1.1,
        "the rear wall aligns the chassis before reversing despite the gyro route reference");
  check(a.getCurrentPosition() + b.getCurrentPosition() == -4000,
        "alignment spins do not change calibrated backward travel");

  fresh();
  const uint32_t duplicateStart = fakeNow;
  scanMs = fakeNow;
  tickHook = [&] {
    if (fakeNow - scanMs >= 174) { scanMs = fakeNow; rear(1, 0); }
    if (fakeNow - duplicateStart > 3500) resetQueued = true;
  };
  ok = backIntoPlace(2000);
  check(!ok && a.getCurrentPosition() == 0 && b.getCurrentPosition() == 0,
        "repeated acquisition IDs cannot supply the two rear alignment scans");

  fresh();
  scanMs = fakeNow; acq = 100;
  const uint32_t staleStart = fakeNow;
  tickHook = [&] {
    if (fakeNow - scanMs >= 174) { scanMs = fakeNow; rear(++acq, 0, 901); }
    if (fakeNow - staleStart > 3500) resetQueued = true;
  };
  ok = backIntoPlace(2000);
  check(!ok && a.getCurrentPosition() == 0 && b.getCurrentPosition() == 0,
        "new IDs with stale source acquisition ages still cannot authorize parking");

  fresh();
  scanMs = fakeNow; acq = 100;
  const uint32_t unstableStart = fakeNow;
  tickHook = [&] {
    if (fakeNow - scanMs >= 174) { scanMs = fakeNow; ++acq; rear(acq, acq % 2 ? 3 : -3); }
    if (fakeNow - unstableStart > 3500) resetQueued = true;
  };
  ok = backIntoPlace(2000);
  check(!ok && a.getCurrentPosition() == 0 && b.getCurrentPosition() == 0,
        "disagreeing rear scans never start the wheels");

  fresh();
  shouldStop = true;
  const uint32_t pathTime = lastPathCommandMs;
  rear(1000, 0); pollESP();
  HeadingControl::RearView view;
  check(shouldStop && lastPathCommandMs == pathTime,
        "rear wall evidence never grants GO or refreshes path permission");
  const float continuousHeading = heading.heading();
  processESPMessage("[BLE CONNECTION ESTABLISHED]");
  rear(1, 0); pollESP();
  check(heading.rearViewSince(fakeNow, fakeNow, &view) && view.acquisition == 1 &&
        heading.heading() == continuousHeading,
        "a restarted Pi scan sequence is accepted without resetting continuous heading");
  ESP_Serial.rx += "D,2,0,1,1600,0*00\n"; pollESP();
  check(!heading.rearViewSince(fakeNow, fakeNow, &view),
        "a damaged rear UART checksum withdraws rear alignment evidence");

  fresh();
  scanMs = fakeNow; acq = 0;
  const uint32_t heldStart = fakeNow;
  bool heldStill = true, moved = false;
  long heldAt = LONG_MIN;
  tickHook = [&] {
    const uint32_t age = fakeNow - heldStart;
    if (fakeNow - scanMs >= 174) {
      scanMs = fakeNow; rear(++acq, 0, 0, !(age >= 1800 && age < 4500));
    }
    moved = moved || a.getCurrentPosition() + b.getCurrentPosition() < -10;
    if (age >= 2200 && age < 4400) {
      const long position = a.getCurrentPosition() + b.getCurrentPosition();
      if (heldAt == LONG_MIN) heldAt = position;
      heldStill = heldStill && heldAt == position;
    }
    if (age > 30000) resetQueued = true;
  };
  ok = backIntoPlace(8000);
  check(ok && moved && heldStill && heldAt < 0 &&
        a.getCurrentPosition() + b.getCurrentPosition() == -16000,
        "rear wall loss mid-parking holds, then fresh pairs resume only remaining travel");

  fresh();
  scanMs = fakeNow; acq = 0;
  const uint32_t driftStart = fakeNow;
  plant.gyroBiasDps = 0.4;
  tickHook = [&] {
    if (fakeNow - scanMs >= 174) { scanMs = fakeNow; rear(++acq, plant.thetaDeg); }
    if (fakeNow - driftStart > 30000) resetQueued = true;
  };
  ok = backIntoPlace(8000);
  check(ok && std::fabs(plant.thetaDeg) < 0.1 &&
        ESP_Serial.tx.find("[TURN:START]") == std::string::npos,
        "gyro bias alone does not trigger a post-parking spin when the rear wall is square");

  fresh();
  scanMs = fakeNow; acq = 0;
  const uint32_t offAngleStart = fakeNow;
  bool lowPowerSeen = false;
  tickHook = [&] {
    if (fakeNow - scanMs >= 174) { scanMs = fakeNow; rear(++acq, 12); }
    lowPowerSeen = lowPowerSeen || lowPowerActive;
    if (fakeNow - offAngleStart > 12000) resetQueued = true;
  };
  ok = backIntoPlace(2000);
  check(!ok && lowPowerSeen && a.getCurrentPosition() == 0 && b.getCurrentPosition() == 0,
        "an unconfirmed rear angle holds in low power instead of repeatedly spinning");
  tickHook = backgroundHook = nullptr;
  return failures ? 1 : 0;
}
