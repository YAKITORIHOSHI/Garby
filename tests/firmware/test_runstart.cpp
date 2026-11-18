// RunStart (owner, 2026-11-16): the Developer window's step list. It runs with no LiDAR,
// sonar or gyro check and no time limit; the 800 ms KEEP lease and the emergency stop
// end it, and Continue run / Reverse to start pick it up from its exact saved place.
#include "stubs/Arduino.h"
#include "../../MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.cpp"
#include "../../MAIN_EXECUTOR_CODE/pointsRun.ino"
#include "../../MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.ino"

static FastAccelStepper wheel1, wheel2;
static unsigned failures = 0;
static void check(bool ok, const char* label) {
  if (!ok) { std::fprintf(stderr, "FAIL %s\n", label); ++failures; }
  else std::printf("PASS %s\n", label);
}
static bool sent(const std::string& text) { return ESP_Serial.tx.find(text) != std::string::npos; }
static void fixture() {
  tickHook = backgroundHook = nullptr;
  fakeNow = 1000;
  wheel1 = FastAccelStepper(); wheel2 = FastAccelStepper();
  stepper1 = &wheel1; stepper2 = &wheel2;
  heading = HeadingControl();
  operatorEmergencyLatched = false;
  operatorStopMarker = 0;
  operatorRebootScheduled = false;
  operatorSonarEnabled = true;
  operatorLastId = operatorLastAction = operatorLastFailure = "";
  uartRxBuffer = ""; ESP_Serial.rx.clear();
  uartDiscardUntilNewline = uartCarriageReturnSeen = false;
  cancelUltrasonicPing();
  latestFrontSampleValid = false;
  lastFrontSampleMs = 0;
  frontObstacleLatched = frontEmergencyActive = false;
  frontFilterCount = frontFilterIndex = ultrasonicBlockedCount = ultrasonicClearCount = 0;
  sonarSweep.reset();
  spinSonarCheck = spinInProgress = false;
  discardRunCheckpoint();
  developerLastNumber = 0;
  developerId = "";
  fullReset();
  processESPMessage("[BLE CONNECTION ESTABLISHED]");
  ESP_Serial.tx.clear();
}
// Runs the loop until the test ends, renewing the lease every 100 ms while `keep` says
// so and sending STOP all the time (no sensor gate may hold a list). `during` runs too.
static void runLoop(const std::string& id, std::function<bool()> keep, std::function<void()> during = nullptr,
                    uint32_t limitMs = 60000) {
  const uint32_t started = fakeNow;
  uint32_t lastKeep = 0;
  tickHook = [&]() {
    if (keep() && fakeNow - lastKeep >= 100) { processESPMessage(("[TEST KEEP:" + id + "]").c_str()); lastKeep = fakeNow; }
    processESPMessage("STOP");
    if (during) during();
    if (fakeNow - started > limitMs) std::exit(30);
  };
  loop();
  tickHook = nullptr;
}
static bool at(long p1, long p2) { return wheel1.getCurrentPosition() == p1 && wheel2.getCurrentPosition() == p2; }

int main() {
  // Wire format bounds.
  for (const char* bad : {"[TEST:10:P:8001:4800:F100]", "[TEST:11:P:9:4800:F100]", "[TEST:12:P:2351:99:F100]",
                          "[TEST:13:P:2351:20001:F100]", "[TEST:14:P:2351:4800:F0]", "[TEST:15:P:2351:4800:F100001]",
                          "[TEST:16:P:2351:4800:X100]", "[TEST:17:P:2351:4800:F0100]", "[TEST:18:P:2351:4800:]",
                          "[TEST:19:P:2351:4800:F100,]", "[TEST:20:P:2351:4800:F100:1]", "[TEST:21:P:2351:4800]",
                          "[TEST:22:C:]", "[TEST:23:U:7:1]"}) {
    fixture(); processESPMessage(bad);
    check(garbyState == GarbyState::IDLE && !developerQueued && sent(":INVALID_RUN]"), bad);
  }
  fixture();
  std::string many = "[TEST:30:P:2351:4800:F1";
  for (int i = 0; i < 20; ++i) many += ",F1";
  processESPMessage((many + "]").c_str());
  check(sent("[TEST FAILED:30:INVALID_RUN]"), "a list of more than 20 steps is refused");
  fixture(); processESPMessage("[TEST:31:C:5]");
  check(sent("[TEST FAILED:31:NO_RUN]"), "Continue without a saved list is refused");
  fixture(); processESPMessage("[TEST:32:P:8000:20000:F16000]");
  uint32_t fastestAllowed = 0;
  runLoop("32", [] { return true; }, [&]() { fastestAllowed = max(fastestAllowed, max(wheel1.speed, wheel2.speed)); });
  check(at(16000, 16000) && sent("[TEST DONE:32]") && fastestAllowed == 8000,
        "a list may run at up to 8000 steps/s (owner, 2026-11-16), above the routes' 5600");

  // A whole list: forward, spin left, backward, spin right, bumps. Motor 1 is the left
  // wheel by the current guess, so a left spin is motor 1 back and motor 2 forward.
  fixture();
  processESPMessage("[TEST:40:P:4000:9000:F40000,L300,B200,R300,C250]");
  check(sent("[TEST ACK:40]"), "a RunStart list is accepted from idle with no sensor gates");
  uint32_t fastest = 0, accel = 0;
  const uint32_t listAt = fakeNow;
  runLoop("40", [] { return true; }, [&]() {
    fastest = max(fastest, max(wheel1.speed, wheel2.speed));
    accel = max(accel, max(wheel1.acceleration, wheel2.acceleration));
  });
  std::printf("INFO list took %u ms, fastest %u steps/s\n", (unsigned)(fakeNow - listAt), (unsigned)fastest);
  check(at(40000 - 300 - 200 + 300 + 250, 40000 + 300 - 200 - 300 + 250) && sent("[TEST DONE:40]"),
        "every step ends exactly at its wheel targets while STOP is sent throughout");
  check(fakeNow - listAt > TEST_LIMIT_MS && fastest == 4000 && accel == 9000,
        "a list has no 8 s limit and uses its own max speed and acceleration");
  check(garbyState == GarbyState::IDLE && shouldStop && !pathCommandFresh() && operatorManualStartRequired,
        "after a list every normal gate is back and no route can start by itself");
  check(sent("[RUN SAVED:40:4:250:4000:9000:F40000,L300,B200,R300,C250]"),
        "the finished list stays saved at its end for Reverse to start");

  // Reverse to start after it finished: every step undone, back to the exact start.
  processESPMessage("[TEST:41:U:40]");
  runLoop("41", [] { return true; });
  check(at(0, 0) && sent("[TEST DONE:41]") && sent("[RUN SAVED:41:0:0:4000:9000:"),
        "Reverse to start drives the list back to exactly where it began");

  // Emergency stop part-way, release, Continue run.
  fixture();
  processESPMessage("[TEST:50:P:2351:4800:F4000,L1000,F3000]");
  bool stopped = false;
  long stop1 = 0, stop2 = 0;
  runLoop("50", [] { return true; }, [&]() {
    if (!stopped && wheel2.getCurrentPosition() > 4400) {  // inside the left spin
      stopped = true; processESPMessage("[OP:900:ESTOP]");
      stop1 = wheel1.getCurrentPosition(); stop2 = wheel2.getCurrentPosition();
    }
  });
  check(stopped && operatorEmergencyLatched && !motorsRunning() && at(stop1, stop2) &&
        sent("[TEST FAILED:50:EMERGENCY_STOP]"), "the emergency stop stops a list at once");
  check(sent("[RUN SAVED:50:1:"), "the stopped list is saved at its step for Continue run / Reverse to start");
  processESPMessage("[TEST:51:C:50]");
  check(sent("[TEST FAILED:51:EMERGENCY_STOP]") && at(stop1, stop2), "Continue cannot release the emergency stop itself");
  processESPMessage("[OP:901:RELEASE]");
  check(!operatorEmergencyLatched && runSaved, "releasing the stop keeps the saved list");
  processESPMessage("[TEST:52:C:49]");
  check(sent("[TEST FAILED:52:NO_RUN]"), "Continue must name the saved list");
  processESPMessage("[TEST:53:C:50]");
  runLoop("53", [] { return true; });
  check(at(4000 - 1000 + 3000, 4000 + 1000 + 3000) && sent("[TEST DONE:53]"),
        "Continue run finishes the list exactly from where it stopped");

  // Emergency stop part-way, release, Reverse to start.
  fixture();
  processESPMessage("[TEST:60:P:2351:4800:F4000,R1000,B500]");
  stopped = false;
  runLoop("60", [] { return true; }, [&]() {
    if (!stopped && wheel1.getCurrentPosition() > 4300) { stopped = true; processESPMessage("[OP:910:ESTOP]"); }
  });
  processESPMessage("[OP:911:RELEASE]");
  processESPMessage("[TEST:61:U:60]");
  runLoop("61", [] { return true; });
  check(stopped && at(0, 0) && sent("[TEST DONE:61]"), "Reverse to start undoes a stopped list back to its start");

  // A lost lease (window closed, ssh dropped, link lost) stops it and keeps the place.
  fixture();
  processESPMessage("[TEST:70:P:2351:4800:F20000]");
  const uint32_t keepUntil = fakeNow + 3000;
  uint32_t lastRunning = 0;
  runLoop("70", [&] { return fakeNow < keepUntil; }, [&]() { if (motorsRunning()) lastRunning = fakeNow; });
  std::printf("INFO list, Pi gone: last ran %ld ms after the KEEPs stopped\n", (long)(lastRunning - keepUntil));
  check(lastRunning > keepUntil && lastRunning - keepUntil <= TEST_LEASE_MS + 150 && !motorsRunning() &&
        sent("[TEST FAILED:70:LEASE_EXPIRED]") && !operatorEmergencyLatched,
        "if the Pi goes quiet the list stops within its 800 ms lease");
  const long lost1 = wheel1.getCurrentPosition();
  check(lost1 > 0 && lost1 < 20000 && sent("[RUN SAVED:70:0:"), "a lease stop keeps the list saved");
  processESPMessage("[TEST:71:C:70]");
  runLoop("71", [] { return true; });
  check(at(20000, 20000) && sent("[TEST DONE:71]"), "Continue after a lease stop reaches the exact end");

  // Anything else that moves the wheels forgets the saved list.
  fixture();
  processESPMessage("[TEST:80:P:2351:4800:F500]");
  runLoop("80", [] { return true; });
  processESPMessage("[TEST:81:F:400:0]");
  runLoop("81", [] { return true; });
  ESP_Serial.tx.clear();
  processESPMessage("[TEST:82:U:80]");
  check(sent("[TEST FAILED:82:NO_RUN]") && !runSaved, "a bench test in between forgets the saved list");
  fixture();
  processESPMessage("[TEST:90:P:2351:4800:F500]");
  runLoop("90", [] { return true; });
  check(runSaved, "precondition: a saved list");
  processESPMessage("[RUN CLEAR:89]");
  check(runSaved && sent("[RUN SAVED:90:"), "Discard names the list it forgets");
  processESPMessage("[RUN CLEAR:90]");
  check(!runSaved && sent("[RUN SAVED:0]"), "Discard forgets the saved list");
  fixture();
  processESPMessage("[TEST:95:P:2351:4800:F500]");
  runLoop("95", [] { return true; });
  routeResume.endRoute(); routeResume.discard();
  runRoute(RouteResume::OUTBOUND, OUTBOUND_ROUTE, 0, "TEST");
  check(!runSaved, "a route forgets the saved list");

  // A list cannot start during a route, a latched stop or without a link.
  fixture(); processESPMessage("[OP:920:ESTOP]");
  processESPMessage("[TEST:96:P:2351:4800:F500]");
  check(sent("[TEST FAILED:96:EMERGENCY_STOP]") && at(0, 0), "a list cannot start while the emergency stop is latched");

  return failures ? 1 : 0;
}
