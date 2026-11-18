// "Continue run" after the operator's emergency stop (owner, 2026-11-14).
//
// The real executor main loop drives the production outbound route through the
// kinematic hallway (hallway_sim.h states what that model cannot show). An
// emergency stop arrives mid-straight, mid-turn and while parked at the
// destination; the operator then continues. The route must pick up where it
// stopped: the rest of the straight (not all of it again, not none of it), the
// rest of the turn (not a second 90 degrees), and parked stays parked. Release
// still ends the route at IDLE and leaves nothing to continue.
//
// This file uses no new executor names, so it also builds against the code from
// before the change, where CONTINUE is an unknown action and these checks fail.
#pragma GCC optimize("O1")
#include "stubs/Arduino.h"
#include "../../MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.cpp"
#include "../../MAIN_EXECUTOR_CODE/pointsRun.ino"
#include "../../MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.ino"
#include "hallway_sim.h"
#include <climits>
#include <functional>

static FastAccelStepper wheelA, wheelB;
static hallway_sim::Plant plant;
static int failures = 0;
static unsigned long nextOperatorId = 100;

static void check(bool ok, const std::string& label) {
  if (ok) { std::printf("PASS %s\n", label.c_str()); return; }
  std::fprintf(stderr, "FAIL %s\n", label.c_str());
  failures++;
}
static bool sent(const std::string& text) { return ESP_Serial.tx.find(text) != std::string::npos; }
static long meanPosition() { return (wheelA.getCurrentPosition() + wheelB.getCurrentPosition()) / 2; }
// The bridge link stays up throughout (the operator actions need it).
static void link() { manualStartLinkReady = lastConnected = true; lastBridgeRxMs = fakeNow; }

static void freshRobot() {
  tickHook = nullptr; backgroundHook = nullptr;
  fakeNow = 5000;
  operatorStopMarker = 0;
  operatorEmergencyLatched = operatorManualStartRequired = false;
  operatorSonarEnabled = false;  // obstacle sonar is covered by test_controller/test_hold_resume
  operatorRebootScheduled = false;
  wheelA = FastAccelStepper(); wheelB = FastAccelStepper();
  stepper1 = &wheelA; stepper2 = &wheelB;
  fullReset();
  heading = HeadingControl();
  lidarSlowActive = false; lidarNormalGoCount = 0;
  uartRxBuffer = ""; ESP_Serial.rx.clear(); ESP_Serial.tx.clear();
  uartDiscardUntilNewline = uartCarriageReturnSeen = false;
  frontObstacleLatched = frontEmergencyActive = false;
  sonarSweep.reset(); ultrasonicPingInFlight = false;
  plant = hallway_sim::Plant();
  plant.attach(&wheelA, &wheelB, &ESP_Serial.rx);
  plant.backing = &backingActive;
  backgroundHook = []() { plant.tick(); link(); };
  for (int i = 0; i < 400; ++i) { delay(1); pollESP(); }
}

// An operator action as the bridge relays it over the UART.
static std::string operatorAction(const char* action) {
  const std::string id = std::to_string(++nextOperatorId);
  processESPMessage(String(("[OP:" + id + ":" + action + "]").c_str()));
  return id;
}

// Run the real main loop until done() or the time limit (fake milliseconds).
static bool runLoopUntil(const std::function<bool()>& done, uint32_t limitMs) {
  const uint32_t until = fakeNow + limitMs;
  while ((int32_t)(fakeNow - until) < 0) {
    loop();
    if (done()) return true;
    delay(1);
  }
  return false;
}

static bool stoppedAtIdle() {
  return operatorEmergencyLatched && garbyState == GarbyState::IDLE &&
         !wheelA.isRunning() && !wheelB.isRunning();
}

static std::string lastOpStatus() {
  const size_t at = ESP_Serial.tx.rfind("[OP STATUS:");
  if (at == std::string::npos) return "";
  return ESP_Serial.tx.substr(at, ESP_Serial.tx.find(']', at) - at + 1);
}

// The outbound route opens with 500 steps of calibration bumps and an
// 8000-step straight, then turns left 90 degrees.
static const long FIRST_LEG_END = 500 + 8000;

static void checkContinueMidStraightAndParked() {
  freshRobot();
  garbyState = GarbyState::RUNNING; outboundComplete = false;
  bool stopped = false;
  long atFirstSpin = LONG_MIN;
  tickHook = [&]() {
    if (!stopped && meanPosition() > 500 + 3000) {
      stopped = true;
      ESP_Serial.rx += "[OP:" + std::to_string(++nextOperatorId) + ":ESTOP]\n";
    }
    if (atFirstSpin == LONG_MIN && spinInProgress) atFirstSpin = meanPosition();
  };
  check(runLoopUntil(stoppedAtIdle, 60000) && stopped,
        "an emergency stop mid-straight force-stops the route and unwinds it to IDLE");
  const long stoppedAt = meanPosition();
  check(lastOpStatus().find(":RESUME=1:") != std::string::npos,
        "while stopped, the controller reports that the interrupted route can continue");

  const std::string id = operatorAction("CONTINUE");
  check(sent("[OP ACK:" + id + ":CONTINUE]") && !operatorEmergencyLatched &&
        garbyState == GarbyState::RUNNING && sent("[RUNNING]") && shouldStop,
        "Continue run releases the stop into the same route, still holding for fresh GO");
  check(runLoopUntil([]() { return outboundComplete; }, 400000),
        "the continued route completes to the destination");
  tickHook = nullptr;
  check(sent("[ROUTE CONTINUE:1/13]"),
        "the route continues at the interrupted straight (step 1 of 13), not from the start");
  std::printf("INFO stopped at mean step %ld; first turn began at %ld (expected %ld)\n",
              stoppedAt, atFirstSpin, FIRST_LEG_END);
  check(stoppedAt > 3500 && stoppedAt < FIRST_LEG_END && atFirstSpin == FIRST_LEG_END,
        "the straight drives only what was left: the first turn begins exactly at its calibrated end");
  check(std::fabs(hallway_sim::Plant::wrap(plant.thetaDeg - 180.0)) < 2.0,
        "the continued route ends facing its final direction (every turn made once)");

  // Parked at the destination: an emergency stop there, then Continue, parks again.
  const long parkedAt = meanPosition();
  const double parkedHeading = plant.thetaDeg;
  operatorAction("ESTOP");
  check(runLoopUntil(stoppedAtIdle, 2000) && lastOpStatus().find(":RESUME=1:") != std::string::npos,
        "an emergency stop while parked at the destination can be continued");
  ESP_Serial.tx.clear();
  const std::string parkedId = operatorAction("CONTINUE");
  check(runLoopUntil([]() { return outboundComplete; }, 5000) &&
        sent("[OP ACK:" + parkedId + ":CONTINUE]") && sent("[OUTBOUND COMPLETE]") &&
        meanPosition() == parkedAt && std::fabs(plant.thetaDeg - parkedHeading) < 0.01,
        "continuing at the destination parks there again without moving (no second outbound run)");

  // The return route: an emergency stop in its first straight, then Continue,
  // carries on returning (not a new outbound run). The operator stops it again.
  processESPMessage("[RESET:1234567890100]");
  bool returnStopped = false;
  const long returnStart = meanPosition();
  tickHook = [&]() {
    if (!returnStopped && garbyState == GarbyState::RETURNING && meanPosition() > returnStart + 500 + 3000) {
      returnStopped = true;
      ESP_Serial.rx += "[OP:" + std::to_string(++nextOperatorId) + ":ESTOP]\n";
    }
  };
  check(runLoopUntil(stoppedAtIdle, 60000) && returnStopped &&
        lastOpStatus().find(":RESUME=1:") != std::string::npos,
        "an emergency stop on the return route can be continued");
  ESP_Serial.tx.clear();
  const std::string returnId = operatorAction("CONTINUE");
  bool drove = false;
  tickHook = [&]() {
    if (!drove && wheelA.isRunning()) {
      drove = true;
      ESP_Serial.rx += "[OP:" + std::to_string(++nextOperatorId) + ":ESTOP]\n";
    }
  };
  check(sent("[OP ACK:" + returnId + ":CONTINUE]") && garbyState == GarbyState::RETURNING &&
        sent("[RETURNING]"),
        "Continue on the return route resumes returning");
  check(runLoopUntil([&]() { return drove && stoppedAtIdle(); }, 60000) && sent("[ROUTE CONTINUE:1/13]"),
        "the return route continues at its interrupted straight");
  check(!returnComplete && !sent("[RETURN COMPLETE]"),
        "an interrupted return is not station arrival even when it unwinds to IDLE");
  tickHook = nullptr;
  operatorAction("CONTINUE");
  check(runLoopUntil([]() { return returnComplete; }, 400000) &&
        garbyState == GarbyState::IDLE && sent("[RETURN COMPLETE]"),
        "a continued return reports station arrival only after the remaining route completes");
}

// Run the outbound route until an emergency stop at stopAtDeg into the first turn.
static double stopInFirstTurn(double stopAtDeg) {
  freshRobot();
  garbyState = GarbyState::RUNNING; outboundComplete = false;
  bool stopped = false;
  double stoppedHeading = 0.0;
  tickHook = [&]() {
    if (!stopped && spinInProgress && plant.thetaDeg > stopAtDeg) {
      stopped = true;
      stoppedHeading = plant.thetaDeg;
      ESP_Serial.rx += "[OP:" + std::to_string(++nextOperatorId) + ":ESTOP]\n";
    }
  };
  const bool idle = runLoopUntil(stoppedAtIdle, 60000);
  tickHook = nullptr;
  return idle && stopped ? stoppedHeading : -1.0;
}

// Continue, and return the heading when the robot next drives forward (1e9: never).
// The operator stops it again right there, so the route unwinds to IDLE.
static double continueUntilNextLeg(uint32_t limitMs) {
  ESP_Serial.tx.clear();
  operatorAction("CONTINUE");
  double headingAfterTurn = 1e9;
  tickHook = [&]() {
    if (headingAfterTurn > 1e8 && !spinInProgress && meanPosition() > FIRST_LEG_END + 50) {
      headingAfterTurn = plant.thetaDeg;
      ESP_Serial.rx += "[OP:" + std::to_string(++nextOperatorId) + ":ESTOP]\n";
    }
  };
  // An emergency stop also passes through a momentary route fault on its way to
  // IDLE; only a fault without the operator stop ends the wait early.
  runLoopUntil([&]() {
    return (headingAfterTurn < 1e8 && stoppedAtIdle()) || (routeFaultLatched && !operatorEmergencyLatched);
  }, limitMs);
  tickHook = nullptr;
  return headingAfterTurn;
}

static void checkContinueMidTurnAndRelease() {
  // 40 degrees into the first 90 degree turn, same S3 heading reference.
  const double stoppedAt = stopInFirstTurn(40.0);
  check(stoppedAt > 40.0 && stoppedAt < 80.0,
        "an emergency stop in the middle of the first 90 degree turn stops it part way");
  const double ended = continueUntilNextLeg(120000);
  std::printf("INFO turn stopped at %.1f deg; continued turn ended at %.2f deg\n", stoppedAt, ended);
  check(ended < 1e8 && std::fabs(ended - 90.0) < 1.5 && sent("[HEADING ROUTE:CONTINUE]"),
        "the continued turn finishes the same 90 degrees (it does not add another 90) and drives on");

  // Release instead of continue: the route ends at IDLE with nothing left to continue.
  check(stoppedAtIdle() && lastOpStatus().find(":RESUME=1:") != std::string::npos,
        "a second emergency stop during the continued route can be continued again");
  const std::string released = operatorAction("RELEASE");
  check(sent("[OP ACK:" + released + ":RELEASE]") && garbyState == GarbyState::IDLE &&
        operatorManualStartRequired && lastOpStatus().find(":RESUME=0:") != std::string::npos,
        "Release still ends the route at IDLE and needs a fresh deliberate start");
  operatorAction("ESTOP");
  runLoopUntil(stoppedAtIdle, 2000);
  const std::string nothing = operatorAction("CONTINUE");
  check(operatorEmergencyLatched && garbyState == GarbyState::IDLE &&
        sent("[OP FAILED:" + nothing + ":NO_ROUTE]") &&
        lastOpStatus().find(":RESUME=0:") != std::string::npos,
        "after a release, an emergency stop at IDLE has no route to continue");
  const std::string stillLatched = operatorAction("RELEASE");
  check(sent("[OP ACK:" + stillLatched + ":RELEASE]"), "release still works after a refused continue");

  // The S3 bridge restarts while stopped 12 degrees into the turn: a new heading
  // reference reading 0. The old hallway's walls are still in view and confirm
  // the direction (the same rule as a long heading outage), then the turn ends.
  const double early = stopInFirstTurn(12.0);
  plant.epoch = 2; plant.gyroDeg = 0.0;
  for (int i = 0; i < 500; ++i) { delay(1); loop(); }
  const double verified = continueUntilNextLeg(120000);
  std::printf("INFO new S3 reference at %.1f deg: continued turn ended at %.2f deg\n", early, verified);
  check(early > 12.0 && verified < 1e8 && std::fabs(verified - 90.0) < 1.5 &&
        sent("[HEADING CONTINUE verify=OK"),
        "after a new S3 heading reference the walls confirm the direction, then the turn finishes");

  // The same restart 40 degrees into the turn: no wall line is in view at that
  // angle, so the direction cannot be confirmed. The robot must not move, and the
  // route faults at the 5-minute heading limit, as for a long outage.
  stopInFirstTurn(40.0);
  plant.epoch = 3; plant.gyroDeg = 0.0;
  for (int i = 0; i < 500; ++i) { delay(1); loop(); }
  const long before = meanPosition();
  const double headingBefore = plant.thetaDeg;
  const double blind = continueUntilNextLeg(HeadingControl::HEADING_RECOVERY_MS + 60000);
  check(blind > 1e8 && routeFaultLatched && sent("[HEADING CONTINUE verify=TIMEOUT]") &&
        meanPosition() == before && std::fabs(plant.thetaDeg - headingBefore) < 0.01,
        "a new S3 reference with no confirmable walls keeps the robot still and faults at the heading limit");
}

// Along-hallway travel (cm) when the second route turn begins, at the end of the
// long hallway. stopAfter > 0: an emergency stop that many steps into it, the robot
// pushed shiftCm sideways while stopped, then Continue.
static double alongHallwayAtSecondTurn(long stopAfter, double shiftCm = 0.0) {
  const bool interrupt = stopAfter > 0;
  freshRobot();
  garbyState = GarbyState::RUNNING; outboundComplete = false;
  bool wasSpinning = false, firstTurnDone = false, stopped = false;
  double x = -1.0;
  tickHook = [&]() {
    if (spinInProgress && !wasSpinning && firstTurnDone && x < 0.0 && meanPosition() > 590000)
      x = plant.xCm;
    if (!spinInProgress && wasSpinning && !firstTurnDone) {
      firstTurnDone = true;
      plant.enterHallway(0.0);  // the first turn put the robot in the long hallway
    }
    wasSpinning = spinInProgress;
    if (interrupt && !stopped && firstTurnDone && meanPosition() > FIRST_LEG_END + 500 + stopAfter) {
      stopped = true;
      ESP_Serial.rx += "[OP:" + std::to_string(++nextOperatorId) + ":ESTOP]\n";
    }
    if (x >= 0.0 && !operatorEmergencyLatched)
      ESP_Serial.rx += "[OP:" + std::to_string(++nextOperatorId) + ":ESTOP]\n";
  };
  if (interrupt) {
    if (!runLoopUntil(stoppedAtIdle, 400000) || !stopped) return -1.0;
    plant.yCm += shiftCm;
    ESP_Serial.tx.clear();
    operatorAction("CONTINUE");
  }
  runLoopUntil([&]() { return x >= 0.0 && stoppedAtIdle(); }, 400000);
  tickHook = nullptr;
  operatorAction("RELEASE");
  return x;
}

static void checkContinueInLongHallway() {
  const double straight = alongHallwayAtSecondTurn(0);
  const double continued = alongHallwayAtSecondTurn(200000);
  const bool recentred = sent("[HEADING CENTER");
  std::printf("INFO long hallway: %.1f cm uninterrupted, %.1f cm stopped and continued\n",
              straight, continued);
  check(straight > 7000.0 && continued > 7000.0 && std::fabs(continued - straight) < 3.0 && recentred,
        "a long hallway stopped part way and continued (centring again) ends where it would have");
  // Stopped 1500 steps before the end and pushed 20 cm off centre: a 20 cm sidestep
  // pass would itself travel ~2240 steps forward, more than the tail, so the tail is
  // driven with the straight's own centring instead of pre-run centring.
  const double tail = alongHallwayAtSecondTurn(594000, 20.0);
  std::printf("INFO long hallway tail: %.1f cm after a stop 1500 steps from its end\n", tail);
  check(tail > 7000.0 && std::fabs(tail - straight) < 3.0 && !sent("[HEADING CENTER"),
        "a short continued tail of the long hallway skips pre-run centring and ends where it would have");
}

// Reverse parking (2026-11-16): an emergency stop while backing into the destination,
// then Continue, backs exactly the rest of the way (the leg counts falling steps).
static void checkContinueWhileBackingIn() {
  freshRobot();
  garbyState = GarbyState::RUNNING; outboundComplete = false;
  long backStart = LONG_MIN;
  bool stopped = false;
  tickHook = [&]() {
    if (backStart == LONG_MIN && backingActive) backStart = meanPosition();
    if (!stopped && backStart != LONG_MIN && meanPosition() < backStart - 4000) {
      stopped = true;
      ESP_Serial.rx += "[OP:" + std::to_string(++nextOperatorId) + ":ESTOP]\n";
    }
  };
  check(runLoopUntil(stoppedAtIdle, 400000) && stopped,
        "an emergency stop while backing into the destination unwinds the route to IDLE");
  const long stoppedAt = meanPosition();
  check(!backingActive && sent("[BACK:END]") && lastOpStatus().find(":RESUME=1:") != std::string::npos,
        "the backing scope ends with the stop, and the route can continue");
  ESP_Serial.tx.clear();
  operatorAction("CONTINUE");
  check(runLoopUntil([]() { return outboundComplete; }, 60000) && sent("[ROUTE CONTINUE:12/13]") &&
        sent("[BACK:START]") && sent("[LEG:OF:12/13:K:0]"),
        "Continue resumes the backing step itself");
  tickHook = nullptr;
  std::printf("INFO backing began at %ld, stopped at %ld, parked at %ld\n", backStart, stoppedAt, meanPosition());
  check(stoppedAt < backStart - 4000 && stoppedAt > backStart - 11100 &&
        meanPosition() == backStart - 11100,
        "it backs only what was left: 11100 steps from where backing began, in total");
  check(std::fabs(hallway_sim::Plant::wrap(plant.thetaDeg - 180.0)) < 2.0,
        "and is parked facing the route's final direction");
  operatorAction("ESTOP");
  runLoopUntil(stoppedAtIdle, 2000);
  operatorAction("RELEASE");
}

// An older bridge has no backing rule and keeps sending plain GO: the robot must
// never back up, and only after the grace period is the bridge named as the cause
// (a GO already in flight as the scope starts or ends is ordinary).
static void checkOldBridgeNeverBacks() {
  freshRobot();
  plant.backing = nullptr;   // the simulated bridge now sends plain GO throughout
  const long start = meanPosition();
  const uint32_t t0 = fakeNow;
  bool namedEarly = false;
  tickHook = [&]() {
    if (fakeNow - t0 < 800 && lastStopReason == "STOP:BACK_UNCONFIRMED") namedEarly = true;
    if (fakeNow - t0 > 4000) resetQueued = true;   // end the wait for permission
  };
  const bool ok = backIntoPlace(2000);
  tickHook = nullptr; resetQueued = false;
  check(!ok && meanPosition() == start && !namedEarly && sent("[MOTION:STOP:BACK_UNCONFIRMED]"),
        "with a bridge that only sends plain GO the robot never backs, and says why after 1 s");
  plant.backing = &backingActive;
  processESPMessage(String("GO:BACK"));
  check(shouldStop && lastStopReason == "STOP:CLEARING",
        "a GO:BACK still in flight after the backing scope ends is neither permission nor an alarm");
}

int main() {
  checkContinueMidStraightAndParked();
  checkContinueWhileBackingIn();
  checkOldBridgeNeverBacks();
  checkContinueInLongHallway();
  checkContinueMidTurnAndRelease();
  if (failures) { std::fprintf(stderr, "%d continue-run check(s) failed\n", failures); return 1; }
  std::printf("Continue-run checks PASS\n");
  return 0;
}
