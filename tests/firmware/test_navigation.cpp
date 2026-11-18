// Closed-loop navigation: the real executor firmware against a kinematic hallway.
//
// Covers the 2026-11-11 navigation review: MPU turns checked against the walls,
// gyro/LiDAR fusion for hallway centring, wall gaps, steering vetoes, the LiDAR
// caution zone, and the rule that the robot never moves without heading feedback.
// hallway_sim.h states what this model cannot show.
#pragma GCC optimize("O1")
#include "stubs/Arduino.h"
#include "../../MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.cpp"
#include "../../MAIN_EXECUTOR_CODE/pointsRun.ino"
#include "../../MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.ino"
#include "hallway_sim.h"

static FastAccelStepper wheelA, wheelB;
static hallway_sim::Plant plant;
static int failures = 0;
static void check(bool ok, const std::string& label) {
  if (ok) { std::printf("PASS %s\n", label.c_str()); return; }
  std::fprintf(stderr, "FAIL %s\n", label.c_str());
  failures++;
}

static void freshRobot() {
  tickHook = nullptr; backgroundHook = nullptr;
  fakeNow = 5000;
  operatorStopMarker = 0;
  operatorEmergencyLatched = operatorManualStartRequired = false;
  operatorSonarEnabled = false;  // obstacle sonar is covered by test_controller
  operatorRebootScheduled = false;
  wheelA = FastAccelStepper(); wheelB = FastAccelStepper();
  stepper1 = &wheelA; stepper2 = &wheelB;
  fullReset();
  garbyState = GarbyState::RUNNING;
  heading = HeadingControl();
  lidarSlowActive = false; lidarNormalGoCount = 0;
  uartRxBuffer = ""; ESP_Serial.rx.clear(); ESP_Serial.tx.clear();
  uartDiscardUntilNewline = uartCarriageReturnSeen = false;
  frontObstacleLatched = frontEmergencyActive = false;
  sonarSweep.reset(); ultrasonicPingInFlight = false;
  plant = hallway_sim::Plant();
  plant.attach(&wheelA, &wheelB, &ESP_Serial.rx);
  plant.backing = &backingActive;
  backgroundHook = []() { plant.tick(); };
}
static void settle(uint32_t ms) {
  for (uint32_t i = 0; i < ms; ++i) { delay(1); pollESP(); }
}
static bool sent(const char* text) { return ESP_Serial.tx.find(text) != std::string::npos; }
// Hallway direction in the executor's (gyro) frame, from the plant's truth.
static double axisInGyroFrame() { return plant.gyroDeg - plant.headingVsHallway(); }

static void checkTurnEndpointStop() {
  for (int dir : {1, -1}) {
    const std::string side = dir > 0 ? "left" : "right";
    freshRobot(); plant.walls = false;
    settle(400); beginRouteHeading(); heading.confirmTurnSides();
    // A normal stop leaves these pulses to run; an immediate stop cancels them.
    // The old default stub stopped instantly even on stopMove(), hiding run-out.
    wheelA.stopRunoutSteps = wheelB.stopRunoutSteps = 80;
    wheelA.normalStops = wheelB.normalStops = 0;
    wheelA.forcedStops = wheelB.forcedStops = 0;
    const float target = heading.heading() + dir * 90.0f;
    const SpinResult result = spinToward(target, HeadingControl::TURN_MAX_SPEED_HZ);
    std::printf("INFO %s target stop: target=%+.3f estimate=%+.3f chassis=%+.3f\n",
                side.c_str(), target, heading.turnHeadingNow(), plant.thetaDeg);
    check(result == SpinResult::REACHED, side + " 90 degree spin reaches its heading target");
    // This plant delivers gyro samples without S3 sample latency. Account for
    // the existing compensation when comparing its truth with the stop target.
    const double expectedChassis = target - heading.latestRate() * HeadingControl::S3_SAMPLE_LATENCY_S;
    check(std::fabs(heading.turnHeadingNow() - target) < 0.03f &&
          std::fabs(plant.thetaDeg - expectedChassis) < 0.03,
          side + " spin stops at the estimated target without early cutoff or normal-stop run-out");
    check(wheelA.forcedStops == 1 && wheelB.forcedStops == 1 &&
          wheelA.normalStops == 0 && wheelB.normalStops == 0,
          side + " endpoint aborts both wheel queues without requesting a decelerating stop");
    const long stopped1 = wheelA.getCurrentPosition(), stopped2 = wheelB.getCurrentPosition();
    settle(300);
    check(!wheelA.isRunning() && !wheelB.isRunning() &&
          wheelA.getCurrentPosition() == stopped1 && wheelB.getCurrentPosition() == stopped2 &&
          stopped1 != 0 && stopped2 != 0,
          side + " immediate stop preserves wheel coordinates and leaves no queued pulses");
    check(!operatorEmergencyLatched && !routeFaultLatched && !shouldStop,
          side + " successful turn stop does not latch an emergency or prevent the next route step");
  }

  freshRobot(); settle(400); beginRouteHeading(); heading.confirmTurnSides();
  wheelA.normalStops = wheelB.normalStops = 0;
  wheelA.forcedStops = wheelB.forcedStops = 0;
  bool obstacle = false;
  tickHook = [&]() {
    if (!obstacle && heading.turnHeadingNow() > 89.8f) {
      obstacle = true;
      processESPMessage("STOP:TURN");
    }
  };
  check(spinToward(90.0f, HeadingControl::TURN_MAX_SPEED_HZ) == SpinResult::HALTED && obstacle &&
        wheelA.normalStops > 0 && wheelB.normalStops > 0 &&
        wheelA.forcedStops == 0 && wheelB.forcedStops == 0 &&
        !wheelA.isRunning() && !wheelB.isRunning(),
        "an obstacle at the turn endpoint still takes the safety hold path, not successful completion");
  tickHook = nullptr;
}

static void checkTurns() {
  // Gyro reads 3% low (inside the MPU6050 tolerance): a gyro-only 90 becomes 92.8.
  freshRobot(); plant.gyroScale = 0.97; plant.walls = false;
  settle(400);
  check(beginRouteHeading(), "route frame anchors on a confirmed MPU stream");
  check(turnLeft(90) && std::fabs(plant.thetaDeg - 90.0 / 0.97) < 1.0 && sent("align=NO_WALLS"),
        "without walls a turn ends where the gyro says, scale error included, and says so");
  check(sent("[HEADING TURN cmd=+90.0 gyro=+") && sent("wall_before=") && sent("wall_after="),
        "each turn reports commanded angle, gyro-measured angle and the wall check");

  freshRobot(); plant.gyroScale = 0.97;
  settle(400); beginRouteHeading();
  check(turnLeft(90) && std::fabs(plant.thetaDeg - 90.0) < 1.2 && sent("align=CORRECTED"),
        "the wall alignment removes a 3% gyro scale error from a 90 degree turn");
  check(std::fabs(heading.routeTarget() - axisInGyroFrame()) < 1.0,
        "after alignment the route frame is the measured hallway direction");
  ESP_Serial.tx.clear();
  check(turnRight(90) && std::fabs(plant.thetaDeg) < 1.2,
        "the next turn starts from the aligned frame and lands on the hallway again");

  freshRobot(); settle(400); beginRouteHeading();
  check(turnRight(90) && std::fabs(plant.thetaDeg + 90.0) < 1.2 && sent("align=OK"),
        "an accurate gyro turn is confirmed by the walls without a correction spin");
  check(turnLeft(180) && std::fabs(plant.thetaDeg - 90.0) < 1.5,
        "a 180 degree turn is measured and wall-checked the same way");

  // The motor-side guess is wrong: the first spin backs off and swaps.
  freshRobot(); plant.motor1IsLeft = false; settle(400); beginRouteHeading();
  check(turnLeft(90) && std::fabs(plant.thetaDeg - 90.0) < 1.2 && !heading.motor1IsLeft() &&
        heading.turnSideConfirmed(),
        "a wrong motor-side guess is detected by the MPU, swapped, and the turn still lands");

  // Walls far off the commanded direction are not this hallway: no correction.
  freshRobot(); settle(400); beginRouteHeading();
  plant.wallHeadingErrorDeg = 12.0;  // the only line in view is 12 degrees off the hallway
  check(turnLeft(90) && std::fabs(plant.thetaDeg - 90.0) < 1.0 && sent("align=INCONSISTENT"),
        "walls more than the correction limit from the commanded heading are not followed");
}

static void checkNoHeadingNoMotion() {
  freshRobot(); plant.streamMpu = false; settle(400);
  const long before1 = wheelA.getCurrentPosition(), before2 = wheelB.getCurrentPosition();
  check(!beginRouteHeading() && sent("[HEADING ROUTE:NO_MPU"),
        "a route does not start without the MPU heading stream");
  check(wheelA.getCurrentPosition() == before1 && wheelB.getCurrentPosition() == before2,
        "refusing the route moves no wheel");

  freshRobot(); settle(400); beginRouteHeading();
  plant.streamMpu = false; settle(300);
  const uint32_t started = fakeNow;
  const long turn1 = wheelA.getCurrentPosition(), turn2 = wheelB.getCurrentPosition();
  check(!turnLeft(90) && fakeNow - started >= HeadingControl::HEADING_RECOVERY_MS &&
        sent("[HEADING LOST fault]"),
        "a turn without heading data holds, then fails the route instead of turning by step count");
  check(wheelA.getCurrentPosition() == turn1 && wheelB.getCurrentPosition() == turn2,
        "no wheel step is commanded while the heading is missing");

  freshRobot(); settle(400); beginRouteHeading();
  plant.streamMpu = false; settle(300);
  const uint32_t outage = fakeNow;
  tickHook = [&]() { if (fakeNow - outage >= 2500) plant.streamMpu = true; };
  check(turnLeft(90) && std::fabs(plant.thetaDeg - 90.0) < 1.2 && sent("[HEADING LOST recovered]"),
        "a turn waits out a short heading outage and then completes by MPU");
  tickHook = nullptr;

  // Mid-straight outage: brake, wait, re-confirm the path, finish the same distance.
  freshRobot(); heading.confirmTurnSides(); settle(400); beginRouteHeading();
  bool cut = false, stoppedDuringOutage = false; uint32_t cutAt = 0;
  tickHook = [&]() {
    if (!cut && plant.xCm > 150) { cut = true; cutAt = fakeNow; plant.streamMpu = false; }
    if (cut && fakeNow - cutAt > 600 && fakeNow - cutAt < 1900 &&
        !wheelA.isRunning() && !wheelB.isRunning()) stoppedDuringOutage = true;
    if (cut && fakeNow - cutAt >= 2000) plant.streamMpu = true;
  };
  const long start1 = wheelA.getCurrentPosition(), start2 = wheelB.getCurrentPosition();
  check(safeMoveDistance(40000, false, true) && stoppedDuringOutage,
        "losing the heading stream mid-straight brakes the robot until it returns");
  check(std::labs((wheelA.getCurrentPosition() - start1) + (wheelB.getCurrentPosition() - start2) - 80000) <= 1,
        "after the outage the straight completes its commanded mean distance (to the half step)");
  tickHook = nullptr;

  freshRobot(); heading.confirmTurnSides(); settle(400); beginRouteHeading();
  cut = false;
  tickHook = [&]() { if (!cut && plant.xCm > 150) { cut = true; plant.streamMpu = false; } };
  check(!safeMoveDistance(40000, false, true) && !wheelA.isRunning() && !wheelB.isRunning() &&
        plant.xCm < 400.0,
        "a heading stream that stays lost ends the straight stopped, as a route failure");
  tickHook = nullptr;
}

struct StraightResult { bool ok; double endY, endHeading, maxY; };
static StraightResult drive(int32_t steps, bool centering) {
  heading.confirmTurnSides();  // a turn has already proved the motor sides
  settle(400);
  beginRouteHeading();
  const bool ok = safeMoveDistance(steps, false, centering);
  return {ok, plant.yCm, plant.headingVsHallway(), plant.maxAbsY};
}

static void checkCentering() {
  // The reported failure: starts 3 degrees off the hallway, gyro drifting, wheels unequal.
  const auto disturbed = []() {
    freshRobot();
    plant.thetaDeg = 3.0; plant.gyroDeg = 0.0; plant.yCm = 0.0;
    plant.gyroBiasDps = 0.05; plant.leftWheelScale = 1.004;
  };
  disturbed();
  StraightResult gyroOnly = drive(120000, false);
  check(gyroOnly.ok && std::fabs(gyroOnly.endY) > 60.0,
        "ROOT CAUSE reproduced: gyro-only heading hold from a 3 degree start error drifts over 60 cm in 15 m");
  std::printf("INFO gyro-only: end y %+.1f cm, heading vs hallway %+.2f deg\n", gyroOnly.endY, gyroOnly.endHeading);

  disturbed();
  StraightResult fused = drive(120000, true);
  check(fused.ok && std::fabs(fused.endY) < 5.0 && std::fabs(fused.endHeading) < 1.5,
        "with wall fusion the same run ends within 5 cm of centre and 1.5 degrees of the hallway");
  check(fused.maxY < 20.0, "the fused run never strays 20 cm from centre on the way");
  check(std::labs(wheelA.getCurrentPosition() + wheelB.getCurrentPosition() - 240000) <= 1 &&
        std::labs(wheelA.getCurrentPosition() - wheelB.getCurrentPosition()) > 50,
        "the steered straight keeps its wheel differential and ends on its mean distance (to the half step)");
  check(std::fabs(heading.routeTarget() - axisInGyroFrame()) < 1.0 && sent("realign="),
        "the straight hands the measured hallway direction to the route frame for the next turn");
  std::printf("INFO fused: end y %+.1f cm, heading %+.2f deg, max |y| %.1f cm, drift estimate %+.3f dps\n",
              fused.endY, fused.endHeading, fused.maxY, heading.driftDps());

  // Starts well off centre but parallel: bounded, hysteretic centring.
  freshRobot(); plant.yCm = 30.0; plant.maxAbsY = 30.0;
  double minHeading = 0.0;
  tickHook = [&]() { minHeading = std::min(minHeading, plant.headingVsHallway()); };
  StraightResult offset = drive(120000, true);
  tickHook = nullptr;
  check(offset.ok && std::fabs(offset.endY) < 5.0 && offset.maxY <= 31.0,
        "a 30 cm lateral offset is steered back to centre without overshooting the far side");
  check(minHeading > -(HeadingControl::MAX_CENTER_DEG + 1.5),
        "the centring heading never exceeds its bound");

  // Inside the deadband nothing is asked of the wheels.
  freshRobot(); plant.yCm = 1.5; plant.distanceNoiseCm = 0.3; plant.headingNoiseDeg = 0.1;
  double maxCenter = 0.0;
  tickHook = [&]() { maxCenter = std::max(maxCenter, (double)std::fabs(heading.centerTargetDeg())); };
  StraightResult centred = drive(60000, true);
  tickHook = nullptr;
  check(centred.ok && maxCenter == 0.0 && std::fabs(centred.endY) < 4.0,
        "a robot already centred gets no centring command at all");

  // Wall gaps: open doors on both sides for 4 m. The gyro carries the hallway axis.
  freshRobot(); plant.gyroBiasDps = 0.05; plant.leftWheelScale = 1.004;
  plant.leftGaps = {{300.0, 700.0}}; plant.rightGaps = {{300.0, 700.0}};
  StraightResult gap = drive(120000, true);
  check(gap.ok && gap.maxY < 15.0 && std::fabs(gap.endY) < 6.0,
        "a 4 m stretch with no walls is crossed on the gyro and re-centred afterwards");

  // One side open (the measured open door): the other wall and the known width centre it.
  freshRobot(); plant.yCm = 15.0; plant.rightGaps = {{150.0, 1200.0}};
  StraightResult oneWall = drive(120000, true);
  check(oneWall.ok && std::fabs(oneWall.endY) < 6.0,
        "with one wall missing the remaining wall and the learned width keep centring");

  // No walls at all: plain gyro heading hold, never a fault.
  freshRobot(); plant.walls = false;
  StraightResult open = drive(60000, true);
  check(open.ok && !heading.axisKnown() && std::fabs(open.endY) < 3.0,
        "missing walls remove centring evidence, not the MPU heading hold");

  // A door leaf or clutter: wall headings that jump are gated, not followed.
  freshRobot();
  uint32_t jumps = 0;
  tickHook = [&]() {
    // 0.7 s of scans 9 degrees off the real walls, then normal again
    plant.wallHeadingErrorDeg = (plant.xCm > 300.0 && plant.xCm < 350.0) ? 9.0 : 0.0;
    if (plant.wallHeadingErrorDeg != 0.0) ++jumps;
  };
  StraightResult clutter = drive(80000, true);
  tickHook = nullptr;
  check(clutter.ok && jumps > 0 && clutter.maxY < 8.0 && heading.axisReseeds() == 0,
        "a short run of wall headings 9 degrees off is rejected by the gate");
}

static void checkStraightLifecycle() {
  // Long straights are bracketed for the bridge; short and adjustment moves are not steered.
  freshRobot(); heading.confirmTurnSides(); settle(400); beginRouteHeading(); ESP_Serial.tx.clear();
  check(safeMoveDistance(20000, false, true), "a long straight completes for the notice check");
  const size_t start = ESP_Serial.tx.find("[HEADING:START:");
  check(start != std::string::npos && ESP_Serial.tx.rfind("[HEADING:END]") != std::string::npos &&
        ESP_Serial.tx.rfind("[HEADING:END]") > start && sent("[HEADING STRAIGHT steps=20000"),
        "a long straight sends START with its route error, END, and its straight report");
  ESP_Serial.tx.clear();
  const long a0 = wheelA.getCurrentPosition(), b0 = wheelB.getCurrentPosition();
  check(safeMoveDistance(100, true, false) && !sent("[HEADING:START") &&
        wheelA.getCurrentPosition() - a0 == 100 && wheelB.getCurrentPosition() - b0 == 100,
        "an adjustment move is one plain ramp: no heading segment, both wheels the same distance");
  freshRobot(); heading.confirmTurnSides(); settle(400); beginRouteHeading(); ESP_Serial.tx.clear();
  wheelB.rejectNext = true;
  check(!safeMoveDistance(20000, false, true) && sent("[HEADING:START:") &&
        ESP_Serial.tx.rfind("[HEADING:END]") > ESP_Serial.tx.find("[HEADING:START:") &&
        !wheelA.isRunning() && !wheelB.isRunning(),
        "a rejected wheel command stops both wheels and still ends the heading segment");

  // A path STOP in the middle of a straight halts it; fresh GO finishes the same distance.
  freshRobot(); heading.confirmTurnSides(); settle(400); beginRouteHeading();
  bool stopped = false, halted = false; uint32_t stopAt = 0;
  tickHook = [&]() {
    if (!stopped && plant.xCm > 100.0) {
      stopped = true; stopAt = fakeNow; plant.sendGo = false; ESP_Serial.rx += "STOP\n";
    }
    if (stopped && fakeNow - stopAt > 900 && fakeNow - stopAt < 1500 &&
        !wheelA.isRunning() && !wheelB.isRunning()) halted = true;
    if (stopped && fakeNow - stopAt >= 1500) plant.sendGo = true;
  };
  check(safeMoveDistance(40000, false, true) && stopped && halted &&
        std::labs(wheelA.getCurrentPosition() + wheelB.getCurrentPosition() - 80000) <= 1,
        "a path STOP mid-straight halts the robot; after fresh GO it finishes the remaining distance (to the half step)");
  tickHook = nullptr;

  // Routine steering does not write the USB console every control tick.
  freshRobot(); heading.confirmTurnSides(); settle(400); beginRouteHeading(); plant.yCm = 20.0;
  Serial.tx.clear();
  check(safeMoveDistance(120000, false, true) && Serial.tx.size() < 1500,
        "a 15 m steered straight writes only its summary to the USB console");

  // A stalled wheel inside an MPU turn.
  freshRobot(); settle(400); beginRouteHeading(); wheelB.frozen = true;
  check(!turnLeft(90) && routeFaultLatched && !wheelA.isRunning() && !wheelB.isRunning(),
        "a stalled wheel during an MPU turn stops both wheels and faults the route");

  // A reset that lands during the wall check ends the turn as not completed.
  freshRobot(); settle(400); beginRouteHeading();
  bool resetSent = false;
  tickHook = [&]() {
    if (!resetSent && std::fabs(plant.thetaDeg - 90.0) < 1.5 && !wheelA.isRunning() && !wheelB.isRunning()) {
      resetSent = true; resetQueued = true;
    }
  };
  check(!turnLeft(90) && resetSent, "a reset during the post-turn wall check reports the turn as not completed");
  tickHook = nullptr;
}

static void checkVetoAndSlow() {
  // Robot right of centre must steer LEFT; the bridge permits only RIGHT.
  freshRobot(); plant.yCm = -20.0; plant.allow = HeadingControl::ALLOW_RIGHT;
  float minCut = 0.0f;
  tickHook = [&]() { minCut = std::min(minCut, heading.appliedCut()); };
  StraightResult vetoed = drive(40000, true);
  tickHook = nullptr;
  check(vetoed.ok && minCut >= 0.0f && vetoed.endY < -15.0,
        "nearest-object clearance on the left vetoes every steering command toward it");
  freshRobot(); plant.yCm = -20.0; plant.allow = HeadingControl::ALLOW_LEFT;
  float maxCut = 0.0f;
  tickHook = [&]() { maxCut = std::max(maxCut, heading.appliedCut()); };
  StraightResult allowed = drive(40000, true);
  tickHook = nullptr;
  check(allowed.ok && maxCut <= 0.0f && allowed.endY > -10.0,
        "the same correction runs when that side is permitted, and never steers to the vetoed side");
  freshRobot(); plant.yCm = -20.0; plant.allow = 0;
  float anyCut = 0.0f;
  tickHook = [&]() { anyCut = std::max(anyCut, std::fabs(heading.appliedCut())); };
  StraightResult none = drive(40000, true);
  tickHook = nullptr;
  check(none.ok && anyCut == 0.0f, "with no permission bits the wheels are never steered");

  // LiDAR caution zone.
  freshRobot();
  bool sawCaution = false, sawFullAgain = false, sawFullFirst = false;
  tickHook = [&]() {
    const uint32_t top = std::max(wheelA.speed, wheelB.speed);
    plant.slowZone = plant.xCm > 150.0 && plant.xCm < 350.0;
    if (plant.xCm > 50.0 && plant.xCm < 140.0 && top == MAX_SPEED) sawFullFirst = true;
    if (plant.xCm > 220.0 && plant.xCm < 340.0 && top == CAUTION_SPEED) sawCaution = true;
    if (plant.xCm > 500.0 && plant.xCm < 600.0 && top == MAX_SPEED) sawFullAgain = true;
  };
  StraightResult slow = drive(80000, true);
  tickHook = nullptr;
  check(slow.ok && sawFullFirst && sawCaution && sawFullAgain,
        "GO:SLOW drops the straight to caution speed and plain GO restores full speed");
}

// Pre-run centring (2026-11-12): before the long run the robot stands still and the
// walls put it on the centre line and axis, so the straight starts with nothing to correct.
static long wheelSum() { return wheelA.getCurrentPosition() + wheelB.getCurrentPosition(); }
static void checkPreRunCentring() {
  const auto afterTurn = [](double lateralCm, double headingDeg) {
    freshRobot(); heading.confirmTurnSides(); settle(400); beginRouteHeading();
    plant.yCm = lateralCm; plant.maxAbsY = std::fabs(lateralCm); plant.thetaDeg = headingDeg;
  };
  afterTurn(22.0, 3.0);
  int32_t progress = 0;
  bool ok = centerBeforeLongRun(&progress);
  std::printf("INFO pre-run centring from y +22 cm, 3 deg: y %+.1f cm, heading %+.2f deg, %ld steps forward\n",
              plant.yCm, plant.headingVsHallway(), (long)progress);
  check(ok && std::fabs(plant.yCm) <= HeadingControl::PRECENTER_TOLERANCE_CM + 0.5 &&
        std::fabs(plant.headingVsHallway()) < 1.0 && progress > 0 && sent("[HEADING CENTER done"),
        "pre-run centring puts a robot 22 cm off centre and 3 degrees off the axis on the centre line, facing the hallway");
  check(std::fabs(heading.routeTarget() - axisInGyroFrame()) < 1.0,
        "the route frame is the measured hallway axis when the long straight starts");
  check(std::fabs(progress / plant.stepsPerCm - 22.0 / std::tan(HeadingControl::PRECENTER_ANGLE_DEG * 0.0174533)) < 3.0,
        "the reported forward progress is the sidestep's travel along the hallway");

  // Wheel travel half of what the firmware assumes (the unresolved 1600 vs 3200
  // steps/rev question): the first sidestep falls short and teaches the scale.
  afterTurn(-25.0, -2.0);
  plant.stepsPerCm = 2.0 * HeadingControl::STEPS_PER_CM;
  ok = centerBeforeLongRun(&progress);
  check(ok && std::fabs(plant.yCm) <= HeadingControl::PRECENTER_TOLERANCE_CM + 0.5 &&
        sent("[HEADING CENTER learn k=0.5"),
        "with the wheel distance off by 2x the sidestep learns the scale and still centres (k reported for calibration)");

  // Already centred and aligned: nothing moves.
  afterTurn(1.5, 0.2);
  plant.distanceNoiseCm = 0.3; plant.headingNoiseDeg = 0.1;
  long before = wheelSum();
  ok = centerBeforeLongRun(&progress);
  check(ok && progress == 0 && wheelSum() == before && sent("[HEADING CENTER done"),
        "a robot already on the centre line is not moved");

  // No walls: skipped, nothing moves; the straight's own centring remains.
  afterTurn(15.0, 0.0);
  plant.walls = false;
  before = wheelSum();
  ok = centerBeforeLongRun(&progress);
  check(ok && progress == 0 && wheelSum() == before && sent("skip=NO_WALLS"),
        "without walls the pre-run centring is skipped, not a fault");

  // The walls say the robot went the wrong way (a left/right sign error between
  // LiDAR and MPU): it stops correcting after one sidestep instead of running away.
  afterTurn(12.0, 0.0);
  plant.swapWalls = true;
  ok = centerBeforeLongRun(&progress);
  check(ok && sent("stop=WRONG_WAY") && std::fabs(plant.yCm) < 30.0,
        "with LiDAR left/right reversed against the MPU, one sidestep shows it and the centring stops");
}

// A bridge restart longer than 10 s in the middle of a turn (2026-11-12): the heading
// returns as a new S3 reference and the walls in view may still be the old hallway's,
// 90 degrees from the turn's target. The check compares with the nearest right angle.
static void checkLongOutageMidTurn() {
  freshRobot(); heading.confirmTurnSides(); settle(400); beginRouteHeading();
  bool cut = false, restored = false;
  uint32_t cutAt = 0;
  tickHook = [&]() {
    if (!cut && plant.thetaDeg > 12.0) {
      cut = true; cutAt = fakeNow; plant.streamMpu = false; plant.sendGo = false;
      ESP_Serial.rx += "STOP:MPU\n";
    }
    if (cut && !restored && fakeNow - cutAt >= 12000) {
      restored = true; plant.epoch += 1; plant.streamMpu = true; plant.sendGo = true;
    }
  };
  const bool ok = turnLeft(90);
  tickHook = nullptr;
  std::printf("INFO long outage mid-turn: ok=%d heading %.2f deg, verify %s\n", (int)ok, plant.thetaDeg,
              sent("[HEADING LOST verify=OK") ? "OK" : sent("verify=NO_WALLS") ? "NO_WALLS" : "other");
  check(cut && restored && ok && !routeFaultLatched && std::fabs(plant.thetaDeg - 90.0) < 1.5 &&
        sent("[HEADING LOST verify=OK"),
        "a 12 s outage 12 degrees into a turn is verified against the old hallway's walls and the turn completes");
}

static void checkHeadingRecoveryOnEveryResume() {
  // A real S3 outage carries STOP:MPU. GO may return later than heading/walls.
  freshRobot(); heading.confirmTurnSides(); settle(400); beginRouteHeading();
  bool cut = false, restored = false, movedBeforeClear = false;
  bool lowPowerSeen = false; uint32_t cutAt = 0;
  long held1 = 0, held2 = 0;
  const long start1 = wheelA.getCurrentPosition(), start2 = wheelB.getCurrentPosition();
  tickHook = [&]() {
    if (!cut && plant.xCm > 150) {
      cut = true; cutAt = fakeNow; plant.streamMpu = plant.sendGo = false;
      ESP_Serial.rx += "STOP:MPU\n";
    }
    if (!cut) return;
    lowPowerSeen = lowPowerSeen || lowPowerActive;
    const uint32_t age = fakeNow - cutAt;
    if (age > 1000 && held1 == 0) {
      held1 = wheelA.getCurrentPosition(); held2 = wheelB.getCurrentPosition();
    }
    if (held1 != 0 && age < 20000 &&
        (wheelA.getCurrentPosition() != held1 || wheelB.getCurrentPosition() != held2)) movedBeforeClear = true;
    if (!restored && age >= 12000) { restored = true; plant.epoch++; plant.streamMpu = true; }
    if (age >= 20000) plant.sendGo = true;
  };
  const bool ok = safeMoveDistance(40000, false, true);
  tickHook = nullptr;
  check(ok && cut && restored && lowPowerSeen && !lowPowerActive && !movedBeforeClear &&
        sent("[HEADING LOST holding]") && sent("[HEADING LOST verify=OK"),
        "STOP:MPU recovery verifies the new reference and still waits for later GO before resuming");
  check(std::labs((wheelA.getCurrentPosition() - start1) + (wheelB.getCurrentPosition() - start2) - 80000) <= 1,
        "STOP:MPU recovery retains the original straight's remaining mean distance");

  // Heading can fail after an ordinary obstacle hold has already begun.
  freshRobot(); heading.confirmTurnSides(); settle(400); beginRouteHeading();
  cut = false; uint32_t blockedAt = 0;
  tickHook = [&]() {
    if (!cut && plant.xCm > 150) {
      cut = true; blockedAt = fakeNow; plant.sendGo = false; ESP_Serial.rx += "STOP:OBSTACLE\n";
    }
    if (cut && fakeNow - blockedAt >= 1500) plant.streamMpu = false;
  };
  const bool completed = safeMoveDistance(40000, false, true);
  tickHook = nullptr;
  check(!completed && cut && fakeNow - blockedAt >= HeadingControl::HEADING_RECOVERY_MS &&
        fakeNow - blockedAt < HeadingControl::HEADING_RECOVERY_MS + 4000 &&
        sent("[HEADING LOST fault]") && !wheelA.isRunning() && !wheelB.isRunning() &&
        !headingHoldActive && !lowPowerActive,
        "an MPU outage inside an existing obstacle STOP cannot bypass the five-minute fault deadline");

  // New reference after 12 s: missing walls cannot authorize recovery.
  freshRobot(); heading.confirmTurnSides(); settle(400); beginRouteHeading();
  plant.walls = plant.streamMpu = false; settle(300);
  uint32_t started = fakeNow; restored = false; lowPowerSeen = false;
  const long before1 = wheelA.getCurrentPosition(), before2 = wheelB.getCurrentPosition();
  tickHook = [&]() {
    lowPowerSeen = lowPowerSeen || lowPowerActive;
    if (!restored && fakeNow - started >= 12000) { restored = true; plant.epoch++; plant.streamMpu = true; }
  };
  const bool accepted = holdForHeading();
  tickHook = nullptr;
  check(!accepted && restored && lowPowerSeen && fakeNow - started >= HeadingControl::HEADING_RECOVERY_MS &&
        fakeNow - started < HeadingControl::HEADING_RECOVERY_MS + 50 &&
        sent("verify=WAIT_WALLS") && sent("[HEADING LOST fault]") &&
        !sent("verify=OK") && !headingHoldActive && !lowPowerActive &&
        wheelA.getCurrentPosition() == before1 && wheelB.getCurrentPosition() == before2,
        "missing recovery walls keep both wheels still and fault within the original five-minute budget");

  freshRobot(); heading.confirmTurnSides(); settle(400); beginRouteHeading();
  plant.walls = plant.streamMpu = false; settle(300);
  started = fakeNow; restored = false;
  tickHook = [&]() {
    if (!restored && fakeNow - started >= 12000) { restored = true; plant.epoch++; plant.streamMpu = true; }
    if (fakeNow - started >= 25000) plant.walls = true;
  };
  const bool wallsRecovered = holdForHeading();
  tickHook = nullptr;
  check(wallsRecovered && fakeNow - started >= 25000 && sent("verify=WAIT_WALLS") &&
        sent("verify=OK") && !headingHoldActive && !lowPowerActive,
        "walls arriving after the MPU release recovery only after two fresh agreeing scans");

  // Ordinary wall gaps/obstacles never start the heading deadline if the stream
  // remains valid, even when the obstacle hold exceeds five minutes.
  freshRobot(); heading.confirmTurnSides(); settle(400); beginRouteHeading();
  plant.walls = plant.sendGo = false; processESPMessage("STOP:OBSTACLE");
  started = fakeNow;
  tickHook = [&]() { if (fakeNow - started >= HeadingControl::HEADING_RECOVERY_MS + 1000) plant.sendGo = true; };
  const bool obstacleCleared = waitUntilMotionPermitted("regression obstacle");
  tickHook = nullptr;
  check(obstacleCleared && fakeNow - started > HeadingControl::HEADING_RECOVERY_MS &&
        !sent("[HEADING LOST holding]") && !routeFaultLatched,
        "a long ordinary obstacle hold with continuous heading and no walls still resumes normally");

  // The short/reverse path uses the same recovery gate as a cruise.
  freshRobot(); heading.confirmTurnSides(); settle(400); beginRouteHeading();
  cut = false; restored = false;
  tickHook = [&]() {
    if (!cut && plant.xCm < -30) { cut = true; cutAt = fakeNow; plant.streamMpu = false; }
    if (cut && !restored && fakeNow - cutAt >= 1000) { restored = true; plant.streamMpu = true; }
  };
  const bool reverseCompleted = safeMoveDistance(-10000, false, false);
  tickHook = nullptr;
  check(reverseCompleted && cut && restored && sent("[HEADING LOST holding]"),
        "reverse/plain moves brake and wait for heading feedback before resuming their targets");

  freshRobot(); heading.confirmTurnSides(); settle(400); beginRouteHeading();
  plant.walls = plant.streamMpu = false; settle(300);
  started = fakeNow; restored = false;
  tickHook = [&]() {
    if (!restored && fakeNow - started >= 12000) { restored = true; plant.epoch++; plant.streamMpu = true; }
    if (fakeNow - started >= 18000) processESPMessage("[OP:900:ESTOP]");
  };
  const bool stopped = holdForHeading();
  tickHook = nullptr;
  check(!stopped && operatorEmergencyLatched && !wheelA.isRunning() && !wheelB.isRunning() &&
        !headingHoldActive && !lowPowerActive,
        "operator emergency stop aborts recovery while required walls are still absent");
}

// Floor-level spin clearance (2026-11-12): the LiDAR's turn sweep is above the bin,
// so the sonar surveys its span before a spin; something low inside the circle the
// base sweeps holds the turn until it is removed.
static void checkSpinFloorCheck() {
  freshRobot(); operatorSonarEnabled = true; heading.confirmTurnSides(); settle(400); beginRouteHeading();
  bool objectThere = true, spunWhileThere = false;
  uint32_t heldAt = 0;
  const double startTheta = plant.thetaDeg;
  tickHook = [&]() {
    if (ultrasonicPingInFlight) {
      ultrasonicEchoReady = true;
      // A box beside the right front corner: 25 cm from the sonar at -70 degrees.
      const bool corner = ultrasonicPingCommand == DEFAULT_VIEW - 70;
      ultrasonicEchoWidthUs = objectThere && corner ? 1458 : 7000;
    }
    if (objectThere && std::fabs(plant.thetaDeg - startTheta) > 0.5) spunWhileThere = true;
    if (heldAt == 0 && haltWaitingForClearance) heldAt = fakeNow;
    if (heldAt != 0 && fakeNow - heldAt > 3000) objectThere = false;
  };
  const bool ok = turnLeft(90);
  tickHook = nullptr;
  check(ok && heldAt != 0 && !spunWhileThere && !objectThere && std::fabs(plant.thetaDeg - 90.0) < 1.5,
        "a low object inside the spin circle holds the turn; it turns once the object is removed");

  // A wall 40 cm ahead of the sonar is outside the circle: it never holds a spin.
  freshRobot(); operatorSonarEnabled = true; heading.confirmTurnSides(); settle(400); beginRouteHeading();
  bool held = false;
  tickHook = [&]() {
    if (ultrasonicPingInFlight) { ultrasonicEchoReady = true; ultrasonicEchoWidthUs = 2332; }  // 40 cm everywhere
    held = held || haltWaitingForClearance;
  };
  check(turnLeft(90) && !held, "objects outside the spin circle (40 cm from the sonar) do not hold a turn");
  tickHook = nullptr;
}

static void checkWholeRoute() {
  // The production outbound route with every imperfection at once.
  freshRobot();
  plant.gyroScale = 0.98; plant.gyroBiasDps = 0.03; plant.leftWheelScale = 1.003;
  plant.thetaDeg = 2.0; plant.gyroDeg = 0.0;
  size_t turnsSeen = 0;
  double maxCentring = 0.0;
  tickHook = [&]() {
    maxCentring = std::max(maxCentring, (double)std::fabs(heading.centerTargetDeg()));
    // Each ROUTE turn (90 or 180 degrees, not a centring sidestep's spin) leaves
    // the robot 6 cm off the new hallway's centre line.
    size_t count = 0;
    for (const char* turn : {"[HEADING TURN cmd=+90.0", "[HEADING TURN cmd=-90.0", "[HEADING TURN cmd=+180.0"}) {
      for (size_t at = ESP_Serial.tx.find(turn); at != std::string::npos;
           at = ESP_Serial.tx.find(turn, at + 1)) ++count;
    }
    if (count != turnsSeen) { turnsSeen = count; plant.enterHallway(6.0); }
    if (ESP_Serial.tx.size() > 60000) ESP_Serial.tx.erase(0, 30000), turnsSeen = 0, count = 0;
  };
  settle(400);
  const bool ok = runStart();
  tickHook = nullptr;
  // Commanded: L90, L90, R90, R90, L180 from a 2 degree start = hallway direction 180.
  const double endError = hallway_sim::Plant::wrap(plant.thetaDeg - 180.0);
  std::printf("INFO route: ok=%d end heading error %+.2f deg, last hallway y %+.1f cm, "
              "largest centring command on the long run %.2f deg\n",
              (int)ok, endError, plant.yCm, maxCentring);
  check(ok, "the whole outbound route completes with a 2% gyro scale error, drift and unequal wheels");
  check(maxCentring < 0.5,
        "after the pre-run centring the long run starts centred: its centring never engages (no weaving)");
  check(std::fabs(endError) < 2.0,
        "the route ends within 2 degrees of the true final direction (gyro-only would be off by its scale error and drift)");
}

int main() {
  checkTurnEndpointStop();
  checkTurns();
  checkNoHeadingNoMotion();
  checkCentering();
  checkStraightLifecycle();
  checkVetoAndSlow();
  checkPreRunCentring();
  checkSpinFloorCheck();
  checkLongOutageMidTurn();
  checkHeadingRecoveryOnEveryResume();
  checkWholeRoute();
  if (failures) { std::fprintf(stderr, "%d navigation check(s) failed\n", failures); return 1; }
  return 0;
}
