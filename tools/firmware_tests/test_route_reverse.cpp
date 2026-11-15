// "Reverse to start" and the route leg notices (owner, 2026-11-14).
//
// The real executor main loop drives the production routes through the
// kinematic plant of hallway_sim.h. An emergency stop arrives in a straight, in
// the long hallway, in the middle of a turn, parked at the destination and on
// the return route; the operator then chooses Reverse. The robot must turn
// around, drive forward back along what it covered and stand where the route
// started, facing the way it started.
//
// The plant's wheels never slip, so "back at the start" here shows the step and
// heading bookkeeping only. How close a real robot comes is a floor measurement.
//
// Walls are off for the pose checks (the plant's single straight hallway cannot
// hold a route with turns in it); one check runs with walls to exercise the wall
// alignment after the turn-around.
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
static size_t countSent(const std::string& text) {
  size_t count = 0;
  for (size_t at = ESP_Serial.tx.find(text); at != std::string::npos;
       at = ESP_Serial.tx.find(text, at + 1)) ++count;
  return count;
}
static long meanPosition() { return (wheelA.getCurrentPosition() + wheelB.getCurrentPosition()) / 2; }
static void link() { manualStartLinkReady = lastConnected = true; lastBridgeRxMs = fakeNow; }

static void freshRobot(bool walls = false) {
  tickHook = nullptr; backgroundHook = nullptr;
  fakeNow = 5000;
  operatorStopMarker = 0;
  operatorEmergencyLatched = operatorManualStartRequired = false;
  operatorSonarEnabled = false;
  operatorRebootScheduled = false;
  wheelA = FastAccelStepper(); wheelB = FastAccelStepper();
  stepper1 = &wheelA; stepper2 = &wheelB;
  fullReset();
  heading = HeadingControl();
  routeLegEnd();
  lidarSlowActive = false; lidarNormalGoCount = 0;
  uartRxBuffer = ""; ESP_Serial.rx.clear(); ESP_Serial.tx.clear();
  uartDiscardUntilNewline = uartCarriageReturnSeen = false;
  frontObstacleLatched = frontEmergencyActive = false;
  sonarSweep.reset(); ultrasonicPingInFlight = false;
  plant = hallway_sim::Plant();
  plant.walls = walls;
  plant.attach(&wheelA, &wheelB, &ESP_Serial.rx);
  backgroundHook = []() { plant.tick(); link(); };
  for (int i = 0; i < 400; ++i) { delay(1); pollESP(); }
}

static std::string operatorAction(const char* action) {
  const std::string id = std::to_string(++nextOperatorId);
  processESPMessage(String(("[OP:" + id + ":" + action + "]").c_str()));
  return id;
}

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

static void queueStop() { ESP_Serial.rx += "[OP:" + std::to_string(++nextOperatorId) + ":ESTOP]\n"; }

// Start the outbound route and stop it when `when()` first holds.
static bool outboundUntilStop(const std::function<bool()>& when, uint32_t limitMs = 400000,
                              bool walls = false) {
  freshRobot(walls);
  garbyState = GarbyState::RUNNING; outboundComplete = false;
  bool stopped = false;
  tickHook = [&]() {
    if (!stopped && when()) { stopped = true; queueStop(); }
  };
  const bool idle = runLoopUntil(stoppedAtIdle, limitMs);
  tickHook = nullptr;
  return idle && stopped;
}

static bool atStart(double toleranceCm = 1.5, double toleranceDeg = 1.5) {
  return std::fabs(plant.xCm) < toleranceCm && std::fabs(plant.yCm) < toleranceCm &&
         std::fabs(hallway_sim::Plant::wrap(plant.thetaDeg)) < toleranceDeg;
}

static bool reverseHome(uint32_t limitMs) {
  return runLoopUntil([]() { return garbyState == GarbyState::IDLE && !operatorEmergencyLatched; },
                      limitMs);
}

static void checkReverseFromTheFirstStraight() {
  check(outboundUntilStop([]() { return meanPosition() > 500 + 3000; }),
        "an emergency stop in the first straight stops the route");
  const double stoppedX = plant.xCm;
  check(lastOpStatus().find(":RESUME=1:REV=1]") != std::string::npos,
        "while stopped, the controller reports that the route can be continued or reversed");
  ESP_Serial.tx.clear();
  const std::string id = operatorAction("REVERSE");
  check(sent("[OP ACK:" + id + ":REVERSE]") && !operatorEmergencyLatched &&
        garbyState == GarbyState::REVERSING && sent("[REVERSING]") && shouldStop &&
        operatorManualStartRequired,
        "Reverse releases the stop into the way back, still holding for fresh GO");
  long furthest = meanPosition();
  tickHook = [&]() { if (plant.xCm > stoppedX + 2.0) furthest = LONG_MAX; };
  check(reverseHome(120000) && sent("[REVERSE COMPLETE]") && sent("[IDLE]"),
        "the way back ends at IDLE");
  tickHook = nullptr;
  std::printf("INFO stopped %.1f cm out; after the reverse x %+.2f cm, y %+.2f cm, heading %+.2f deg\n",
              stoppedX, plant.xCm, plant.yCm, hallway_sim::Plant::wrap(plant.thetaDeg));
  check(stoppedX > 40.0 && atStart(), "the robot stands where the route started, facing the way it started");
  check(furthest != LONG_MAX, "it never drives further out before coming back");
  check(sent("[ROUTE REVERSE:1/15]") && sent("[LEG:OB:1/15:S:5]") && sent("[LEG:OB:0/15:B:5]") &&
        sent("[LEG:0]"),
        "the way back is announced step by step, with the station flags of the steps it retraces");
  check(operatorManualStartRequired && lastOpStatus().find(":RESUME=0:REV=0]") != std::string::npos &&
        !routeResume.available(),
        "afterwards nothing is left to continue or reverse, and a new run needs a deliberate start");
  check(runLoopUntil([]() { return false; }, 3000) == false && garbyState == GarbyState::IDLE &&
        !wheelA.isRunning(), "the robot stays at IDLE after the reverse");
}

static void checkReverseFromTheLongHallway() {
  // Step 4: 500 + 8000 + turn + 500, then 30000 steps into the long hallway.
  check(outboundUntilStop([]() { return meanPosition() > 500 + 8000 + 500 + 30000 &&
                                        !spinInProgress; }),
        "an emergency stop in the long hallway stops the route");
  const double outY = plant.yCm;
  ESP_Serial.tx.clear();
  operatorAction("REVERSE");
  check(reverseHome(200000) && sent("[REVERSE COMPLETE]"), "the way back from the long hallway ends at IDLE");
  std::printf("INFO stopped %.0f cm down the hallway; after the reverse x %+.2f cm, y %+.2f cm, heading %+.2f deg\n",
              outY, plant.xCm, plant.yCm, hallway_sim::Plant::wrap(plant.thetaDeg));
  check(outY > 300.0 && atStart(2.5, 1.5),
        "it retraces the hallway, the mirrored turn and the first leg back to the start");
  check(sent("[ROUTE REVERSE:4/15]") && sent("[LEG:OB:4/15:H:0]") && sent("[LEG:OB:3/15:B:0]") &&
        sent("[LEG:OB:1/15:S:5]"),
        "each straight step of the way back is announced in reverse order");
}

static void checkReverseFromTheMiddleOfATurn() {
  check(outboundUntilStop([]() { return spinInProgress && plant.thetaDeg > 40.0; }),
        "an emergency stop in the middle of the first turn stops it part way");
  const double stoppedHeading = plant.thetaDeg;
  ESP_Serial.tx.clear();
  operatorAction("REVERSE");
  check(reverseHome(120000) && sent("[REVERSE COMPLETE]"), "the way back from a half-made turn ends at IDLE");
  std::printf("INFO turn stopped at %.1f deg; after the reverse x %+.2f cm, y %+.2f cm, heading %+.2f deg\n",
              stoppedHeading, plant.xCm, plant.yCm, hallway_sim::Plant::wrap(plant.thetaDeg));
  check(stoppedHeading > 40.0 && stoppedHeading < 85.0 && atStart(),
        "a half-made turn needs no special case: the first leg is driven back at its own heading");
}

static void checkStopDuringTheWayBack() {
  check(outboundUntilStop([]() { return meanPosition() > 500 + 6000; }),
        "an emergency stop in the first straight (second scenario)");
  operatorAction("REVERSE");
  // Stop again once it is driving back.
  bool again = false;
  tickHook = [&]() {
    if (!again && garbyState == GarbyState::REVERSING && !spinInProgress && plant.xCm < 50.0 &&
        wheelA.isRunning()) { again = true; queueStop(); }
  };
  check(runLoopUntil([&]() { return again && stoppedAtIdle(); }, 120000),
        "a second emergency stop on the way back stops it");
  tickHook = nullptr;
  const double midX = plant.xCm;
  check(midX > 5.0 && midX < 50.0 && lastOpStatus().find(":RESUME=1:REV=1]") != std::string::npos &&
        routeResume.saved().reversed,
        "the record says the robot is on its way back, part of the way");
  ESP_Serial.tx.clear();
  const std::string id = operatorAction("CONTINUE");
  check(sent("[OP ACK:" + id + ":CONTINUE]") && garbyState == GarbyState::REVERSING && sent("[REVERSING]"),
        "Continue after a stop on the way back continues the way back, not the outbound route");
  check(reverseHome(120000) && atStart(),
        "and it still ends at the start, having driven only what was left");

  // The same, answered with Reverse again, and then with Release.
  check(outboundUntilStop([]() { return meanPosition() > 500 + 6000; }), "third scenario stop");
  operatorAction("REVERSE");
  again = false;
  tickHook = [&]() {
    if (!again && garbyState == GarbyState::REVERSING && !spinInProgress && plant.xCm < 50.0 &&
        wheelA.isRunning()) { again = true; queueStop(); }
  };
  runLoopUntil([&]() { return again && stoppedAtIdle(); }, 120000);
  tickHook = nullptr;
  operatorAction("REVERSE");
  check(reverseHome(120000) && atStart(), "Reverse after a stop on the way back also ends at the start");

  check(outboundUntilStop([]() { return meanPosition() > 500 + 6000; }), "fourth scenario stop");
  operatorAction("REVERSE");
  again = false;
  tickHook = [&]() {
    if (!again && garbyState == GarbyState::REVERSING && !spinInProgress && plant.xCm < 50.0 &&
        wheelA.isRunning()) { again = true; queueStop(); }
  };
  runLoopUntil([&]() { return again && stoppedAtIdle(); }, 120000);
  tickHook = nullptr;
  ESP_Serial.tx.clear();
  const std::string released = operatorAction("RELEASE");
  const long releasedAt = meanPosition();
  check(sent("[OP ACK:" + released + ":RELEASE]") && garbyState == GarbyState::IDLE &&
        lastOpStatus().find(":RESUME=0:REV=0]") != std::string::npos,
        "Release on the way back ends it at IDLE with nothing left to continue or reverse");
  runLoopUntil([]() { return false; }, 3000);
  check(meanPosition() == releasedAt && garbyState == GarbyState::IDLE, "and the robot does not move");
}

static void checkRefusals() {
  freshRobot();
  ESP_Serial.tx.clear();
  const std::string notStopped = operatorAction("REVERSE");
  check(sent("[OP FAILED:" + notStopped + ":NOT_STOPPED]") && garbyState == GarbyState::IDLE,
        "Reverse without an emergency stop is refused");
  operatorAction("ESTOP");
  runLoopUntil(stoppedAtIdle, 2000);
  ESP_Serial.tx.clear();
  const std::string nothing = operatorAction("REVERSE");
  check(sent("[OP FAILED:" + nothing + ":NO_ROUTE]") && operatorEmergencyLatched &&
        garbyState == GarbyState::IDLE && lastOpStatus().find(":REV=0]") != std::string::npos,
        "Reverse with no interrupted route is refused and the stop stays latched");
  operatorAction("RELEASE");

  // A return request while reversing is refused, like one during the return route.
  check(outboundUntilStop([]() { return meanPosition() > 500 + 3000; }), "refusal scenario stop");
  operatorAction("REVERSE");
  ESP_Serial.tx.clear();
  processESPMessage("[RESET:1234567890555]");
  check(garbyState == GarbyState::REVERSING && sent("[RESET FAILED:1234567890555]"),
        "a return request during the way back is refused and does not disturb it");
  check(reverseHome(120000) && atStart(), "the way back still completes");
}

static void checkReverseOfTheWholeOutboundAndOfTheReturn() {
  // Parked at the destination: stop, Reverse = the whole outbound route backwards.
  freshRobot();
  garbyState = GarbyState::RUNNING; outboundComplete = false;
  check(runLoopUntil([]() { return outboundComplete; }, 600000), "the outbound route completes");
  const double destX = plant.xCm, destY = plant.yCm, destHeading = plant.thetaDeg;
  check(sent("[LEG:OF:0/15:B:5]") && sent("[LEG:OF:1/15:S:5]") && sent("[LEG:OF:2/15:L:0]") &&
        sent("[LEG:OF:4/15:H:0]") && sent("[LEG:OF:14/15:L:0]") && countSent("[LEG:0]") == 1,
        "the outbound route announces each step, station steps flagged, and ends the notice once");
  check(countSent("[LEG:OF:4/15:H:0]") > 20,
        "a long step repeats its notice every 2 s so a lost one cannot leave the Pi guessing");

  // The return route's first straight, then Reverse: back at the destination, parked.
  processESPMessage("[RESET:1234567890700]");
  bool stopped = false;
  const long returnStart = meanPosition();
  tickHook = [&]() {
    if (!stopped && garbyState == GarbyState::RETURNING && meanPosition() > returnStart + 500 + 3000) {
      stopped = true; queueStop();
    }
  };
  check(runLoopUntil(stoppedAtIdle, 60000) && stopped, "an emergency stop in the return route's first straight");
  tickHook = nullptr;
  ESP_Serial.tx.clear();
  operatorAction("REVERSE");
  check(runLoopUntil([]() { return outboundComplete && garbyState == GarbyState::RUNNING; }, 120000) &&
        sent("[REVERSE COMPLETE]") && sent("[OUTBOUND COMPLETE]") && sent("[LEG:RB:1/15:S:0]"),
        "reversing the return route ends parked at the destination, not at IDLE");
  std::printf("INFO destination (%.1f, %.1f, %.1f deg); after reversing the return (%.1f, %.1f, %.1f deg)\n",
              destX, destY, destHeading, plant.xCm, plant.yCm, plant.thetaDeg);
  check(std::fabs(plant.xCm - destX) < 1.5 && std::fabs(plant.yCm - destY) < 1.5 &&
        std::fabs(hallway_sim::Plant::wrap(plant.thetaDeg - destHeading)) < 1.5,
        "it stands where the return route started, facing the way it started");
  const long parkedAt = meanPosition();
  runLoopUntil([]() { return false; }, 3000);
  check(meanPosition() == parkedAt && outboundComplete, "and stays parked (a return can be requested again)");

  // From the destination, the whole outbound route backwards to the station.
  operatorAction("ESTOP");
  check(runLoopUntil(stoppedAtIdle, 2000) && lastOpStatus().find(":REV=0]") != std::string::npos,
        "after a reverse the parked robot has no route record: an emergency stop there offers no reverse");
}

static void checkReverseFromTheDestination() {
  freshRobot();
  garbyState = GarbyState::RUNNING; outboundComplete = false;
  check(runLoopUntil([]() { return outboundComplete; }, 600000), "the outbound route completes (second run)");
  operatorAction("ESTOP");
  check(runLoopUntil(stoppedAtIdle, 2000) && lastOpStatus().find(":RESUME=1:REV=1]") != std::string::npos,
        "an emergency stop while parked at the destination can be reversed");
  ESP_Serial.tx.clear();
  operatorAction("REVERSE");
  check(reverseHome(900000) && sent("[REVERSE COMPLETE]") && sent("[ROUTE REVERSE:15/15]"),
        "the whole outbound route is driven back");
  std::printf("INFO after reversing the whole outbound route: x %+.2f cm, y %+.2f cm, heading %+.2f deg\n",
              plant.xCm, plant.yCm, hallway_sim::Plant::wrap(plant.thetaDeg));
  check(atStart(6.0, 2.0), "and ends at the station, facing the way the route started");
}

static void checkReverseWithWallsAndStationTurns() {
  // With walls: the turn-around in the first straight is aligned to them, unless
  // the step is a station step (the outbound's first straight is one).
  check(outboundUntilStop([]() { return meanPosition() > 500 + 5000; }, 400000, true),
        "an emergency stop in the first straight, walls in view");
  ESP_Serial.tx.clear();
  operatorAction("REVERSE");
  check(reverseHome(120000) && atStart(3.0, 2.0), "the way back with walls in view ends at the start");
  check(sent("align=SKIP") && !sent("align=OK") && !sent("align=CORRECTED"),
        "turn-arounds inside station steps skip the wall alignment");

  // The return route's last turn is a station step: no wall alignment beside the station.
  freshRobot(true);
  garbyState = GarbyState::RETURNING; outboundComplete = false;
  check(runLoopUntil([]() { return garbyState == GarbyState::IDLE; }, 900000),
        "the return route completes");
  const size_t parkTurn = ESP_Serial.tx.rfind("[HEADING TURN cmd=+180.0");
  check(parkTurn != std::string::npos &&
        ESP_Serial.tx.substr(parkTurn, 120).find("align=SKIP") != std::string::npos &&
        sent("[LEG:RF:13/15:S:3]") && sent("[LEG:RF:14/15:L:3]"),
        "the parking turn is a station step: flagged in its notice and not aligned to walls");
  const size_t firstTurn = ESP_Serial.tx.find("[HEADING TURN cmd=+90.0");
  check(firstTurn != std::string::npos &&
        ESP_Serial.tx.substr(firstTurn, 120).find("align=SKIP") == std::string::npos,
        "an ordinary turn of the same route still checks the walls");
}

static void checkStationSonarRule() {
  freshRobot();
  operatorSonarEnabled = true;
  frontObstacleLatched = true;   // the station wall's own echo latched it
  routeLegBegin(RouteResume::RETURN, false, 13, 15, RouteOp::STRAIGHT, STEP_STATION | STEP_WALL_AHEAD_FWD, 0);
  check(!frontObstacleDetected() && !frontEmergencyObstacleDetected() && latestFrontDistance() > 900.0f,
        "facing the station wall with its distance unmeasured, the forward sonar holds nothing");
  routeLegBegin(RouteResume::OUTBOUND, false, 1, 15, RouteOp::STRAIGHT, STEP_STATION | STEP_WALL_AHEAD_REV, 0);
  check(frontObstacleDetected(), "leaving the station forward, the sonar holds as on any step");
  routeLegBegin(RouteResume::OUTBOUND, true, 1, 15, RouteOp::STRAIGHT, STEP_STATION | STEP_WALL_AHEAD_REV, 0);
  check(!frontObstacleDetected(), "the same step on the way back faces the wall");
  routeLegEnd();
  check(frontObstacleDetected(), "when the step ends the ordinary rule is back at once");
  frontObstacleLatched = false;
  operatorSonarEnabled = false;
}

int main() {
  checkReverseFromTheFirstStraight();
  checkReverseFromTheLongHallway();
  checkReverseFromTheMiddleOfATurn();
  checkStopDuringTheWayBack();
  checkRefusals();
  checkReverseOfTheWholeOutboundAndOfTheReturn();
  checkReverseFromTheDestination();
  checkReverseWithWallsAndStationTurns();
  checkStationSonarRule();
  if (failures) { std::fprintf(stderr, "%d reverse-run check(s) failed\n", failures); return 1; }
  std::printf("Reverse-run checks PASS\n");
  return 0;
}
