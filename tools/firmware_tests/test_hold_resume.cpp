// Obstacle holds must end in a resumed route, never a permanent stop (2026-11-12).
//
// The owner reported runs that "completely declared itself stuck/blocked forever".
// These scenarios drive the real executor firmware through the paths that did it:
//  * a sonar obstacle that leaves an open hallway (no echo within ~3 m), which the
//    old code treated as "unknown" forever, so the hold could never be released;
//  * a STOP arriving while the start gate waits for a sonar sample, which made the
//    segment return false and latched a route fault;
//  * a bridge (MPU) restart longer than 10 s in the middle of a straight;
//  * the servo, which was only ever commanded to the forward view.
#include "stubs/Arduino.h"
#include "../../MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.cpp"
#include "../../MAIN_EXECUTOR_CODE/pointsRun.ino"
#include "../../MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.ino"
#include <set>

static FastAccelStepper leftWheel, rightWheel;
static uint32_t mpuSequence = 0;
static bool mpuStreaming = true;
static uint32_t mpuEpoch = 1;
static void mpuLine(uint32_t at) {
  char payload[64];
  std::snprintf(payload, sizeof(payload), "M,%lu,0,0,%lu,3", (unsigned long)++mpuSequence,
                (unsigned long)mpuEpoch);
  uint8_t x = 0;
  for (const char* c = payload; *c; ++c) x ^= (uint8_t)*c;
  char line[72];
  std::snprintf(line, sizeof(line), "%s*%02X", payload, (unsigned)x);
  heading.handleLine(line, at);
}
static void feedMpu() {
  static uint32_t last = 0;
  if (mpuStreaming && fakeNow - last >= 20) { last = fakeNow; mpuLine(fakeNow); }
}
static void primeMpu() { for (int k = 12; k >= 0; --k) mpuLine(fakeNow - (uint32_t)k * 20U); }

static void resetFixture() {
  tickHook = nullptr; backgroundHook = feedMpu; mpuStreaming = true; mpuEpoch = 1;
  heading = HeadingControl();
  fakeNow = 1000;
  operatorStopMarker = 0;
  operatorEmergencyLatched = operatorManualStartRequired = false;
  operatorSonarEnabled = true;
  operatorRebootScheduled = false;
  leftWheel = FastAccelStepper(); rightWheel = FastAccelStepper();
  stepper1 = &leftWheel; stepper2 = &rightWheel;
  fullReset();
  garbyState = GarbyState::RUNNING;
  uartRxBuffer = ""; ESP_Serial.rx.clear(); ESP_Serial.tx.clear();
  uartDiscardUntilNewline = uartCarriageReturnSeen = false;
  frontObstacleLatched = frontEmergencyActive = false;
  latestFrontSampleValid = false;
  frontFilterCount = frontFilterIndex = ultrasonicBlockedCount = ultrasonicClearCount = 0;
  frontSampleSequence = 0; ultrasonicPingInFlight = false;
  heading.confirmTurnSides();
}
static void clearPath() { processESPMessage("GO"); fakeNow += 100; processESPMessage("GO"); primeMpu(); }

static int failures = 0;
static void check(bool ok, const char* label) {
  if (!ok) { std::fprintf(stderr, "FAIL %s\n", label); failures++; }
  else std::printf("PASS %s\n", label);
}

// LiDAR keeps sending the given path command every 100 ms.
struct PathFeed {
  uint32_t last = 0;
  void tick(const char* command) {
    if (fakeNow - last >= 100) { last = fakeNow; processESPMessage(command); }
  }
};

static void answerPing(uint32_t widthUs) {
  if (ultrasonicPingInFlight) { ultrasonicEchoReady = true; ultrasonicEchoWidthUs = widthUs; }
}

int main() {
  // 1. A sonar obstacle walks away and leaves an open hallway: no echo comes back
  //    within the sonar's range. The hold must end once the LiDAR is clear too.
  {
    resetFixture(); clearPath();
    recordFrontSample(30); recordFrontSample(30);
    PathFeed feed; const uint32_t started = fakeNow; bool gaveUp = false;
    tickHook = [&]() {
      feed.tick("GO");  // pings are never answered: nothing within ~3 m
      if (fakeNow - started > 20000 && !gaveUp) { gaveUp = true; resetQueued = true; }
    };
    haltAndWait("SONIC:BLOCKED ahead");
    tickHook = nullptr;
    check(!gaveUp && fakeNow - started < 5000,
          "an obstacle that leaves an open hallway (no echo) releases the sonar hold");
  }

  // 2. Re-enabling the sonar in an open hallway must not hold the robot forever.
  {
    resetFixture();
    garbyState = GarbyState::IDLE;
    manualStartLinkReady = lastConnected = true; lastBridgeRxMs = fakeNow;
    processESPMessage("[OP:11:SONAR_OFF]");
    processESPMessage("[OP:12:SONAR_ON]");
    garbyState = GarbyState::RUNNING;
    clearPath();
    PathFeed feed; const uint32_t started = fakeNow; bool gaveUp = false;
    tickHook = [&]() {
      feed.tick("GO");
      lastBridgeRxMs = fakeNow;
      if (fakeNow - started > 20000 && !gaveUp) { gaveUp = true; resetQueued = true; }
    };
    haltAndWait("SONIC:REARM");
    tickHook = nullptr;
    check(!gaveUp, "a re-enabled sonar with nothing in range releases its rearm latch");
  }

  // 3. A STOP that arrives while the start gate waits for a sonar sample holds the
  //    robot; it must not fail the segment and latch a route fault.
  {
    resetFixture(); clearPath();
    PathFeed feed; const uint32_t started = fakeNow; bool stopped = false, gaveUp = false;
    tickHook = [&]() {
      if (!stopped && !shouldStop && fakeNow - started >= 4) { stopped = true; processESPMessage("STOP"); }
      feed.tick("GO");
      answerPing(7000);  // clear echoes at 120 cm, once the gate has started
      if (fakeNow - started > 30000 && !gaveUp) { gaveUp = true; resetQueued = true; }
    };
    const bool ok = safeMoveDistance(2000, false, false);
    tickHook = nullptr;
    check(ok && stopped && !gaveUp && !routeFaultLatched && leftWheel.position == 2000 &&
          rightWheel.position == 2000,
          "a STOP during the start gate is held and the segment completes");
  }

  // 4. A bridge (MPU) restart that takes 30 s in the middle of a straight. The robot
  //    is stopped the whole time; it must wait and then finish, not latch a fault.
  {
    resetFixture(); clearPath(); beginRouteHeading();
    PathFeed feed; bool cut = false, gaveUp = false, stoppedDuringOutage = true;
    uint32_t cutAt = 0;
    const long start1 = leftWheel.getCurrentPosition();
    tickHook = [&]() {
      feed.tick("GO");
      answerPing(7000);
      if (!cut && leftWheel.getCurrentPosition() - start1 > 4000) {
        cut = true; cutAt = fakeNow; mpuStreaming = false;
      }
      if (cut && fakeNow - cutAt > 1000 && fakeNow - cutAt < 29000 &&
          (leftWheel.isRunning() || rightWheel.isRunning())) stoppedDuringOutage = false;
      if (cut && !mpuStreaming && fakeNow - cutAt >= 30000) { mpuEpoch = 2; mpuStreaming = true; }
      if (cut && fakeNow - cutAt > 120000 && !gaveUp) { gaveUp = true; resetQueued = true; }
    };
    const bool ok = safeMoveDistance(20000, false, true);
    tickHook = nullptr;
    check(cut && ok && !gaveUp && stoppedDuringOutage && !routeFaultLatched,
          "a 30 s bridge/MPU outage mid-straight holds stopped and then completes the straight");
  }

  // 5. The servo sweeps: while driving a straight it looks to both sides of forward.
  {
    resetFixture(); clearPath(); beginRouteHeading();
    PathFeed feed; std::set<int> angles;
    tickHook = [&]() {
      feed.tick("GO");
      answerPing(7000);
      angles.insert(servo.angle);
    };
    const bool ok = safeMoveDistance(30000, false, true);
    tickHook = nullptr;
    const bool left = std::any_of(angles.begin(), angles.end(), [](int a) { return a > DEFAULT_VIEW; });
    const bool right = std::any_of(angles.begin(), angles.end(), [](int a) { return a < DEFAULT_VIEW; });
    check(ok && left && right, "the sonar servo sweeps both sides of forward while driving");
  }

  // 6. Sweep geometry: what an echo at a bearing can mean.
  {
    check(SonarSweep::inPath(0, 50.0f, ULTRASONIC_STOP_DISTANCE_CM) &&
          SonarSweep::inPath(25, 50.0f, ULTRASONIC_STOP_DISTANCE_CM),
          "an echo 50 cm ahead, straight or at the body's corner (+25), is in the path");
    check(!SonarSweep::inPath(70, 101.0f, ULTRASONIC_SLOW_DISTANCE_CM) &&
          !SonarSweep::inPath(-50, 124.0f, ULTRASONIC_SLOW_DISTANCE_CM) &&
          !SonarSweep::inPath(70, 87.0f, ULTRASONIC_STOP_DISTANCE_CM),
          "the 190 cm hallway's side walls and 82 cm pillars at +/-50 and +/-70 are not in the path");
    check(SonarSweep::insideSpin(0, 15.0f) && SonarSweep::insideSpin(-70, 25.0f) &&
          !SonarSweep::insideSpin(70, 101.0f) && !SonarSweep::insideSpin(0, 40.0f),
          "the spin check sees only what the base would sweep (not a wall 40 cm ahead of the sonar)");
    SonarSweep sweep;
    bool forwardEveryOther = sweep.surveySteps() >= 12;
    for (int i = 0; i < sweep.surveySteps(); i += 2) {
      forwardEveryOther = forwardEveryOther && sweep.target(SonarSweep::Pattern::SURVEY) == DEFAULT_VIEW;
      sweep.advance(SonarSweep::Pattern::SURVEY);
      sweep.advance(SonarSweep::Pattern::SURVEY);
    }
    int lowest = 999, highest = -999;
    for (int i = 0; i < sweep.slotCount(); ++i) {
      lowest = std::min(lowest, sweep.slot(i).command);
      highest = std::max(highest, sweep.slot(i).command);
    }
    check(forwardEveryOther && lowest == SCAN_RIGHT && highest == SCAN_LEFT,
          "the survey covers the servo's whole span with forward between every side bearing");

    // Silence is counted over surveys only: a long drive down an open hallway (no
    // echo ahead) must not make the first survey at the next stop report SILENT.
    SonarSweep quiet;
    bool reported = false;
    for (int i = 0; i < 500; ++i) reported = reported || quiet.silentNow(false, false);
    int surveyPings = 0;
    while (!quiet.silentNow(false, true) && surveyPings < 1000) ++surveyPings;
    const bool once = !quiet.silentNow(false, true);
    check(!reported && surveyPings + 1 == 3 * quiet.surveySteps() && once,
          "SONAR:SILENT needs three complete surveys without an echo, is reported once, and driving never counts");
    SonarSweep echoes;
    for (int i = 0; i < 2 * echoes.surveySteps(); ++i) echoes.silentNow(false, true);
    echoes.record(DEFAULT_VIEW - 70, true, 101.0f, 1000);
    bool afterEcho = false;
    for (int i = 0; i < 2 * echoes.surveySteps(); ++i) afterEcho = afterEcho || echoes.silentNow(false, true);
    check(!afterEcho, "one echo at any bearing restarts the silence count");

    // A side echo is an immediate (one-sample) emergency only within 25 cm.
    SonarSweep side;
    side.record(DEFAULT_VIEW + 70, true, 40.0f, 2000);
    const bool farOne = side.sideBlocking(2000);
    side.record(DEFAULT_VIEW + 70, true, 40.0f, 2100);
    const bool farTwo = side.sideBlocking(2100);
    SonarSweep close;
    close.record(DEFAULT_VIEW - 70, true, 20.0f, 3000);
    check(!farOne && farTwo && close.sideBlocking(3000),
          "a near side echo needs two samples; one within 25 cm of the sonar blocks at once");
  }

  // 7. Something low at the body's corner (+25 bearing) while driving: brake, hold,
  //    resume when it is gone, finish the commanded distance.
  {
    resetFixture(); clearPath(); beginRouteHeading();
    PathFeed feed; bool obstacle = true; uint32_t heldAt = 0;
    const long start1 = leftWheel.getCurrentPosition(), start2 = rightWheel.getCurrentPosition();
    tickHook = [&]() {
      feed.tick("GO");
      const bool corner = ultrasonicPingCommand == DEFAULT_VIEW + SonarSweep::FAN_OFFSET_DEG;
      answerPing(obstacle && corner && leftWheel.getCurrentPosition() - start1 > 3000 ? 2915 : 7000);
      if (heldAt == 0 && haltWaitingForClearance) heldAt = fakeNow;
      if (heldAt != 0 && fakeNow - heldAt > 3000) obstacle = false;
    };
    const bool ok = safeMoveDistance(30000, false, true);
    tickHook = nullptr;
    const long mean = ((leftWheel.getCurrentPosition() - start1) + (rightWheel.getCurrentPosition() - start2)) / 2;
    check(ok && heldAt != 0 && !obstacle && !routeFaultLatched && std::labs(mean - 30000) <= 1,
          "a low object at the body's corner holds the straight, which resumes and completes once it is gone");
  }

  // 8. A centred robot in the 190 cm hallway: the sweep sees the side walls and the
  //    open hallway ahead, and none of it holds or slows the robot.
  {
    resetFixture(); clearPath(); beginRouteHeading();
    PathFeed feed; bool held = false, slowed = false;
    tickHook = [&]() {
      feed.tick("GO");
      if (ultrasonicPingInFlight) {
        const int offset = std::abs(ultrasonicPingCommand - DEFAULT_VIEW);
        if (offset >= 10) {  // the side wall, along the beam from the centre line
          const float cm = 95.0f / std::sin(offset * 0.0174533f);
          if (cm < 300.0f) { ultrasonicEchoReady = true; ultrasonicEchoWidthUs = (uint32_t)(cm * 58.31f); }
        }  // straight ahead: nothing within range, no echo
      }
      held = held || haltWaitingForClearance;
      slowed = slowed || motionBaseSpeed != MAX_SPEED;
    };
    const bool ok = safeMoveDistance(30000, false, true);
    tickHook = nullptr;
    check(ok && !held && !slowed, "hallway walls at the sweep's side bearings never hold or slow a straight");
  }

  // 8b. How often the forward bearing is re-read: driving (CRUISE) and held (SURVEY).
  {
    const auto forwardGaps = [](bool driving, uint32_t* worst) {
      resetFixture(); clearPath(); beginRouteHeading();
      PathFeed feed;
      uint32_t lastForward = 0, total = 0, count = 0;
      *worst = 0;
      uint32_t seen = frontSampleSequence;
      tickHook = [&]() {
        feed.tick(driving ? "GO" : "STOP");
        answerPing(7000);
        if (frontSampleSequence != seen) {
          seen = frontSampleSequence;
          if (lastForward != 0) {
            const uint32_t gap = fakeNow - lastForward;
            total += gap; ++count; *worst = std::max(*worst, gap);
          }
          lastForward = fakeNow;
        }
      };
      if (driving) {
        safeMoveDistance(30000, false, true);
      } else {
        const uint32_t until = fakeNow + 6000;
        while (fakeNow < until) { pollESP(); serviceUltrasonic(); delay(5); }
      }
      tickHook = nullptr;
      return count ? (double)total / count : 1e9;
    };
    uint32_t worstDriving = 0, worstHeld = 0;
    const double driving = forwardGaps(true, &worstDriving);
    const double held = forwardGaps(false, &worstHeld);
    std::printf("INFO forward bearing re-read every %.0f ms driving (worst %lu), %.0f ms held (worst %lu)\n",
                driving, (unsigned long)worstDriving, held, (unsigned long)worstHeld);
    check(driving <= 160.0 && worstDriving <= 250 && held <= 260.0 && worstHeld <= 400 &&
          worstHeld < ULTRASONIC_SAMPLE_MAX_AGE_MS,
          "the forward bearing stays fresh: ~0.13 s apart driving, ~0.2 s held, never past its age limit");
  }

  // 9. Buzzer: a beep every 1.2 s while driving, two chirps while held, sparse in
  //    low power, silent at idle and under the emergency stop.
  {
    check(buzzerPatternLevel(BuzzerPattern::RUNNING, 0) && !buzzerPatternLevel(BuzzerPattern::RUNNING, 200) &&
          buzzerPatternLevel(BuzzerPattern::RUNNING, BUZZER_RUN_PERIOD_MS + 10) &&
          buzzerPatternLevel(BuzzerPattern::HOLDING, 0) &&
          !buzzerPatternLevel(BuzzerPattern::HOLDING, BUZZER_HOLD_ON_MS + 10) &&
          buzzerPatternLevel(BuzzerPattern::HOLDING, BUZZER_HOLD_ON_MS + BUZZER_HOLD_GAP_MS + 10) &&
          !buzzerPatternLevel(BuzzerPattern::HOLDING, 600) &&
          buzzerPatternLevel(BuzzerPattern::LOW_POWER, 10) && !buzzerPatternLevel(BuzzerPattern::LOW_POWER, 1000) &&
          !buzzerPatternLevel(BuzzerPattern::OFF, 0),
          "buzzer patterns: beep per period driving, double chirp held, sparse chirp in low power, OFF silent");
    resetFixture(); clearPath(); pollESP();
    const bool running = currentBuzzerPattern() == BuzzerPattern::RUNNING;
    garbyState = GarbyState::IDLE; pollESP();
    const bool idle = currentBuzzerPattern() == BuzzerPattern::OFF;
    garbyState = GarbyState::RUNNING; outboundComplete = true; pollESP();
    const bool parked = currentBuzzerPattern() == BuzzerPattern::OFF;
    outboundComplete = false; operatorEmergencyLatched = true; pollESP();
    const bool stopped = currentBuzzerPattern() == BuzzerPattern::OFF;
    operatorEmergencyLatched = false;
    check(running && idle && parked && stopped,
          "the buzzer runs during a route and is silent idle, parked at the destination and emergency-stopped");
  }

  // 10. A long LiDAR hold drops to low power, chirps sparsely, and restores normal
  //     power on the first clear evidence, before the robot moves again.
  {
    resetFixture(); clearPath(); processESPMessage("STOP");
    PathFeed feed; const uint32_t started = fakeNow;
    bool sawHolding = false, sawLowPattern = false, sawLow = false;
    uint32_t lowAt = 0, normalAt = 0;
    tickHook = [&]() {
      const bool blocked = fakeNow - started < 15000;
      feed.tick(blocked ? "STOP" : "GO");
      answerPing(7000);
      sawHolding = sawHolding || currentBuzzerPattern() == BuzzerPattern::HOLDING;
      sawLowPattern = sawLowPattern || currentBuzzerPattern() == BuzzerPattern::LOW_POWER;
      if (lowPowerHoldActive() && lowAt == 0) lowAt = fakeNow;
      if (lowAt != 0 && !lowPowerHoldActive() && normalAt == 0) normalAt = fakeNow;
      sawLow = sawLow || lowPowerHoldActive();
      if (fakeNow - started > 60000) resetQueued = true;
    };
    haltAndWait("PATH:BLOCKED from LiDAR");
    tickHook = nullptr;
    const size_t low = ESP_Serial.tx.find("[POWER:LOW]");
    const size_t normal = ESP_Serial.tx.find("[POWER:NORMAL]");
    std::printf("INFO low power from %+ld ms to %+ld ms of the hold; it ended at %+ld ms\n",
                (long)(lowAt - started), (long)(normalAt - started), (long)(fakeNow - started));
    check(!resetQueued && sawHolding && sawLow && sawLowPattern && !lowPowerHoldActive() &&
          low != std::string::npos && normal != std::string::npos && normal > low &&
          lowAt - started >= LOW_POWER_AFTER_MS && normalAt != 0 && fakeNow - normalAt >= 150 &&
          fakeNow - started >= 15000,
          "a long hold enters low power after 10 s, and the first clear evidence restores normal power "
          "well before the hold releases");
    pollESP();
    check(currentBuzzerPattern() == BuzzerPattern::RUNNING, "after the hold the running beep returns");
  }

  // 11. The Pi learns what the wheels are doing ([DRIVE] lines) to tell the robot's
  //     own motion from a person's.
  {
    resetFixture(); clearPath(); beginRouteHeading(); ESP_Serial.tx.clear();
    PathFeed feed;
    tickHook = [&]() { feed.tick("GO"); answerPing(7000); };
    const bool ok = safeMoveDistance(20000, false, true);
    for (int i = 0; i < 30; ++i) { delay(10); pollESP(); }
    tickHook = nullptr;
    const long cruise = lroundf((float)MAX_SPEED / HeadingControl::STEPS_PER_CM);
    char expected[32];
    std::snprintf(expected, sizeof(expected), "[DRIVE:F:%ld]", cruise);
    const size_t forward = ESP_Serial.tx.find(expected);
    const size_t stopped = ESP_Serial.tx.rfind("[DRIVE:S]");
    check(ok && forward != std::string::npos && stopped != std::string::npos && stopped > forward,
          "drive reports give the straight's speed in cm/s and the stop that follows");
  }

  if (failures) {
    std::fprintf(stderr, "%d hold/resume check(s) failed\n", failures);
    return 1;
  }
  std::printf("Hold/resume checks PASS\n");
  return 0;
}
