#include "stubs/Arduino.h"
#include "../../MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.cpp"
#include "../../MAIN_EXECUTOR_CODE/pointsRun.ino"
#include "../../MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.ino"
#include "hallway_sim.h"

static FastAccelStepper wheel1, wheel2;
static unsigned failures = 0;
static void check(bool ok, const char* label) {
  if (!ok) { std::fprintf(stderr, "FAIL %s\n", label); ++failures; }
  else std::printf("PASS %s\n", label);
}
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
  fullReset();
  processESPMessage("[BLE CONNECTION ESTABLISHED]");
  ESP_Serial.tx.clear();
}
int main() {
  check(TEST_SPEED_HZ >= 30.0f * HeadingControl::STEPS_PER_CM &&
        RUN_MIN_SPEED_HZ >= 30.0f * HeadingControl::STEPS_PER_CM &&
        HOLD_MAX_SPEED_HZ > TEST_SPEED_HZ,
        "developer requested speeds have a nominal 30 cm/s minimum");
  fixture(); garbyState=GarbyState::RETURNING; fullReset(true);
  processESPMessage("[TEST:100:F:401:0]");
  check(garbyState == GarbyState::IDLE && !motorsRunning(), "oversized developer travel cannot start");
  processESPMessage("[TEST:101:F:400:0]");
  check(ESP_Serial.tx.find("[TEST ACK:101]") != std::string::npos,
        "a finite developer test is accepted separately from a route");
  ESP_Serial.tx.clear(); reportRobotStatus();
  check(!returnComplete && ESP_Serial.tx.find("[RETURN COMPLETE]") == std::string::npos,
        "a developer test clears the parked return arrival before its wheels can move");
  const uint32_t started = fakeNow;
  tickHook = [&]() {
    processESPMessage("[TEST KEEP:101]");
    processESPMessage("STOP"); // only this explicitly disabled LiDAR gate is bypassed
    if (fakeNow - started > 10000) std::exit(2);
  };
  loop(); tickHook = nullptr;
  check(wheel1.getCurrentPosition() == 400 && wheel2.getCurrentPosition() == 400 && !motorsRunning() &&
        ESP_Serial.tx.find("[TEST DONE:101]") != std::string::npos,
        "motor test works without LiDAR, MPU or sonar only when explicitly deselected");
  check(garbyState == GarbyState::IDLE && shouldStop && !pathCommandFresh() &&
        operatorManualStartRequired, "test completion restores normal gates and disarms automatic starts");
  const auto moved = wheel1.getCurrentPosition();
  processESPMessage("[TEST:101:F:400:0]"); loop();
  check(wheel1.getCurrentPosition() == moved, "duplicate developer request cannot replay travel");
  processESPMessage("[TEST:99:F:400:0]"); loop();
  check(wheel1.getCurrentPosition() == moved, "older developer request cannot replay travel");

  fixture(); processESPMessage("[TEST:102:F:400:0]");
  loop();
  check(!motorsRunning() && ESP_Serial.tx.find("[TEST DONE:102]") != std::string::npos,
        "a short finite move may complete inside its initial lease without an extra KEEP");

  fixture(); processESPMessage("[TEST:103:F:400:7]");
  const uint32_t gatedAt = fakeNow;
  tickHook = [&]() { processESPMessage("[TEST KEEP:103]");
    if (fakeNow - gatedAt > 10000) std::exit(3); };
  loop(); tickHook = nullptr;
  check(wheel1.getCurrentPosition() == 0 && wheel2.getCurrentPosition() == 0 && !motorsRunning(),
        "enabled missing LiDAR/MPU/sonar evidence keeps the developer test stationary");

  fixture(); processESPMessage("[TEST:104:1:100:0]");
  tickHook = [&]() { processESPMessage("[TEST KEEP:104]"); processESPMessage("[OP:900:ESTOP]"); };
  loop(); tickHook = nullptr;
  check(operatorEmergencyLatched && !motorsRunning() && shouldStop,
        "operator emergency stop dominates every developer override");
  processESPMessage("[TEST:105:F:400:0]");
  check(ESP_Serial.tx.find("[TEST FAILED:105:EMERGENCY_STOP]") != std::string::npos,
        "developer tests cannot release a latched emergency stop");

  fixture(); processESPMessage("[TEST:106:1:100:0]");
  const uint32_t leftAt = fakeNow;
  tickHook = [&]() { processESPMessage("[TEST KEEP:106]");
    if (fakeNow-leftAt > 10000) std::exit(4); };
  loop(); tickHook = nullptr;
  check(wheel1.getCurrentPosition() == 100 && wheel2.getCurrentPosition() == 0 && !motorsRunning(),
        "a single-motor test moves only its selected channel");

  fixture(); processESPMessage("[TEST:110:F:400:0]");
  processESPMessage("[OP:1000:ESTOP]"); loop();
  check(!developerQueued && !developerExecuting &&
        ESP_Serial.tx.find("[TEST FAILED:110:EMERGENCY_STOP]") != std::string::npos,
        "an emergency stop before execution retires the queued test");
  processESPMessage("[OP:1001:RELEASE]");
  processESPMessage("[TEST:111:I:0:0]");
  tickHook = [&]() { processESPMessage("[TEST KEEP:111]"); };
  loop(); tickHook = nullptr;
  check(ESP_Serial.tx.find("[TEST DONE:111]") != std::string::npos && !motorsRunning(),
        "a cancelled queued test does not block a later deliberate test");

  fixture(); processESPMessage("[TEST:120:F:400:1]");
  const uint32_t blockedAt = fakeNow;
  tickHook = [&]() { processESPMessage("[TEST KEEP:120]"); processESPMessage("STOP");
    if (fakeNow-blockedAt > 10000) std::exit(5); };
  loop(); tickHook = nullptr;
  check(fakeNow-blockedAt <= TEST_LIMIT_MS+50 && !motorsRunning() &&
        wheel1.getCurrentPosition() == 0 && ESP_Serial.tx.find("[TEST FAILED:120:HELD_LIDAR]") != std::string::npos,
        "fresh heartbeats cannot extend a blocked test past its absolute deadline, which names the LiDAR hold");
  processESPMessage("[TEST KEEP:120]"); processESPMessage("[TEST:120:F:400:1]"); loop();
  check(!developerQueued && !developerExecuting && wheel1.getCurrentPosition() == 0,
        "late heartbeat and duplicate request cannot revive an expired test");

  fixture(); processESPMessage("[TEST:130:F:400:0]");
  fakeNow += TEST_LEASE_MS + 1;
  processESPMessage("[TEST KEEP:130]"); loop();
  check(wheel1.getCurrentPosition() == 0 && ESP_Serial.tx.find("LEASE_EXPIRED") != std::string::npos,
        "an expired queued test cannot begin movement or renew with a delayed heartbeat");

  fixture(); processESPMessage("[TEST:140:F:400:0]");
  tickHook = [&]() { processESPMessage("[TEST KEEP:140]"); processESPMessage("[RESET]"); };
  loop(); tickHook = nullptr;
  check(!developerExecuting && !motorsRunning() &&
        ESP_Serial.tx.find("[TEST FAILED:140:RESET_REQUESTED]") != std::string::npos,
        "reset unwinds through test cleanup instead of deleting its active bounds");

  fixture(); processESPMessage("[TEST:150:F:400:0]");
  tickHook = [&]() { processESPMessage("[TEST KEEP:150]"); ESP_Serial.rx += "\x01\n"; };
  loop(); tickHook = nullptr;
  check(!motorsRunning() && ESP_Serial.tx.find("UART_FAULT") != std::string::npos,
        "a UART parser fault stops a test even with LiDAR clearance deselected");

  fixture(); processESPMessage("[TEST:160:F:400:0]");
  wheel2.rejectNext = true;
  tickHook = [&]() { processESPMessage("[TEST KEEP:160]"); };
  loop(); tickHook = nullptr;
  check(!motorsRunning() && wheel1.getCurrentPosition() == 0 && wheel2.getCurrentPosition() == 0,
        "one rejected developer wheel target stops both channels");
  fixture(); processESPMessage("[TEST:161:F:400:0]");
  wheel1.rejectSpeed = true;
  tickHook = [&]() { processESPMessage("[TEST KEEP:161]"); };
  loop(); tickHook = nullptr;
  check(!motorsRunning() && wheel1.getCurrentPosition() == 0 &&
        ESP_Serial.tx.find("SPEED_REJECTED") != std::string::npos,
        "a rejected developer speed cannot reuse an earlier faster profile");

  fixture(); processESPMessage("[TEST:170:F:400:1]");
  processESPMessage("GO"); fakeNow += 100; processESPMessage("GO");
  const uint32_t lidarOnlyAt = fakeNow;
  tickHook = [&]() { processESPMessage("[TEST KEEP:170]"); processESPMessage("GO");
    if (fakeNow-lidarOnlyAt > 5000) std::exit(6); };
  loop(); tickHook = nullptr;
  check(wheel1.getCurrentPosition() == 400 && wheel2.getCurrentPosition() == 400 &&
        ESP_Serial.tx.find("[TEST DONE:170]") != std::string::npos &&
        ESP_Serial.tx.find(":M=0:Y=") != std::string::npos,
        "LiDAR-only testing uses fresh clearance while reporting unavailable MPU measurement");
  check(headingFeedbackRequired() && !pathCommandFresh() && frontObstacleLatched,
        "MPU/path/sonar gates are restored after a LiDAR-only test");

  fixture();
  hallway_sim::Plant plant;
  plant.walls = plant.sendGo = false;
  plant.attach(&wheel1, &wheel2, &ESP_Serial.rx);
  backgroundHook = [&]() { plant.tick(); };
  for (int i=0; i<400; ++i) { delay(1); pollESP(); }
  processESPMessage("[TEST:180:L:10:2]");
  const uint32_t turnAt = fakeNow;
  uint32_t largestSpeed = 0;
  tickHook = [&]() { processESPMessage("[TEST KEEP:180]"); processESPMessage("STOP");
    largestSpeed = max(largestSpeed, max(wheel1.speed, wheel2.speed));
    if (fakeNow-turnAt > 7000) std::exit(7); };
  loop(); tickHook = backgroundHook = nullptr;
  check(ESP_Serial.tx.find("[TEST DONE:180]") != std::string::npos &&
        ESP_Serial.tx.find("align=SKIP") != std::string::npos &&
        fabs(plant.thetaDeg-10.0) <= 1.1 && largestSpeed <= TEST_SPEED_HZ,
        "MPU-only test measures a bounded low-speed turn without wall alignment or LiDAR GO");
  check(ESP_Serial.tx.find(":M=1:Y=") != std::string::npos,
        "the turn result marks a continuous valid MPU measurement explicitly");

  fixture(); processESPMessage("[TEST:190:L:10:0]");
  check(garbyState == GarbyState::IDLE && !motorsRunning(),
        "a turn cannot be requested with MPU supervision deselected");
  processESPMessage("[TEST:190:F:400:0]");
  check(ESP_Serial.tx.find("[TEST FAILED:190:STALE_REQUEST]") != std::string::npos &&
        garbyState == GarbyState::IDLE,
        "a rejected test ID cannot later be reused for another motion");
  processESPMessage("[TEST:191:FORWARD:400:0]");
  processESPMessage("[TEST:191:F:400:0]");
  check(ESP_Serial.tx.find("[TEST FAILED:191:STALE_REQUEST]") != std::string::npos,
        "a malformed action with a canonical ID consumes that ID safely");

  fixture();
  const char* wall = "W,1,0,7,900,1100,150";
  uint8_t checksum=0; for (const char* c=wall; *c; ++c) checksum ^= (uint8_t)*c;
  char wallLine[80]; std::snprintf(wallLine,sizeof(wallLine),"%s*%02X",wall,(unsigned)checksum);
  processESPMessage(wallLine); reportRobotStatus();
  check(ESP_Serial.tx.find("[TEST DATA:M=0:Y=0:W=1:H=150]") != std::string::npos,
        "raw LiDAR wall heading can be inspected independently of an unavailable MPU");
  float wallHeading=0;
  check(!heading.wallMeasurement(fakeNow,nullptr), "diagnostic wall getter handles a missing destination");
  fakeNow += HeadingControl::WALL_MAX_AGE_MS + 1;
  check(!heading.wallMeasurement(fakeNow,&wallHeading), "stale wall measurements never appear current");

  fixture(); processESPMessage("[TEST:200:F:400:0]");
  processESPMessage("[TEST:200:F:100:7]");
  processESPMessage("[TEST:201:F:100:7]");
  processESPMessage("[START:50000]");
  check(developerSensors == 0 && developerAmount == 400 &&
        ESP_Serial.tx.find("[TEST FAILED:200:ID_REUSED]") != std::string::npos &&
        ESP_Serial.tx.find("[TEST FAILED:201:NOT_IDLE]") != std::string::npos &&
        ESP_Serial.tx.find("[START FAILED:50000:NOT_IDLE]") != std::string::npos,
        "a queued test cannot be reconfigured or launch a normal route");
  tickHook = [&]() { processESPMessage("[TEST KEEP:200]"); };
  loop(); tickHook = nullptr;
  check(wheel1.getCurrentPosition() == 400 && ESP_Serial.tx.find("[TEST DONE:200]") != std::string::npos,
        "rejected competing requests preserve the active test identity and travel");
  processESPMessage("[TEST:201:F:100:7]");
  check(!developerQueued && ESP_Serial.tx.find("[TEST FAILED:201:STALE_REQUEST]") != std::string::npos,
        "a test rejected while busy cannot run later when idle");

  fixture(); processESPMessage("[TEST:210:F:400:4]");
  const uint32_t sonarAt = fakeNow;
  recordFrontSample(10);
  tickHook = [&]() { processESPMessage("[TEST KEEP:210]"); recordFrontSample(10);
    if (fakeNow-sonarAt > 10000) std::exit(8); };
  loop(); tickHook = nullptr;
  check(!motorsRunning() && wheel1.getCurrentPosition() == 0 &&
        ESP_Serial.tx.find("[TEST FAILED:210:HELD_SONAR]") != std::string::npos,
        "selected obstacle sonar still holds a test while LiDAR and MPU are deselected, and is named");

  fixture();
  hallway_sim::Plant lossPlant;
  lossPlant.walls = lossPlant.sendGo = false;
  lossPlant.attach(&wheel1,&wheel2,&ESP_Serial.rx);
  backgroundHook = [&]() { lossPlant.tick(); };
  for (int i=0;i<400;++i) { delay(1); pollESP(); }
  lossPlant.streamMpu = false;
  fakeNow += 200;  // consume part of the unchanged 300 ms motion freshness budget
  processESPMessage("[TEST:220:F:400:2]");
  const uint32_t lostAt = fakeNow;
  tickHook = [&]() { processESPMessage("[TEST KEEP:220]");
    if (fakeNow-lostAt > 10000) std::exit(9); };
  loop(); tickHook = backgroundHook = nullptr;
  std::printf("INFO MPU loss: steps=%ld held_ms=%lu failure=%s outcome=%s\n",
              wheel1.getCurrentPosition(), (unsigned long)(fakeNow-lostAt),
              developerFailure.c_str(), developerOutcome.c_str());
  check(!motorsRunning() && wheel1.getCurrentPosition() < 400 && fakeNow - lostAt < 1000 &&
        ESP_Serial.tx.find("[TEST FAILED:220:HELD_MPU]") != std::string::npos,
        "loss of a selected MPU ends a moving test at once and never falls back to wheel counts");


  fixture(); processESPMessage("[TEST:230:2:100:0]");
  ultrasonicAwaitingEcho = ultrasonicPingInFlight = true;
  ultrasonicEchoReady = true;
  tickHook = [&]() { processESPMessage("[TEST KEEP:230]"); };
  loop(); tickHook = nullptr;
  check(wheel1.getCurrentPosition() == 0 && wheel2.getCurrentPosition() == 100 &&
        !ultrasonicAwaitingEcho && !ultrasonicPingInFlight && !ultrasonicEchoReady &&
        frontObstacleLatched && !latestFrontSampleValid && ultrasonicClearCount == 0,
        "motor two is isolated and bypassed sonar is rearmed without any old echo evidence");
  // Owner, 2026-11-15: a lifted turn with every sensor selected stop-started loudly.
  // Lifted wheels: the gyro sees no rotation (gyroScale 0).
  fixture();
  hallway_sim::Plant lifted;
  lifted.walls = lifted.sendGo = false;
  lifted.gyroScale = 0.0;
  lifted.attach(&wheel1, &wheel2, &ESP_Serial.rx);
  backgroundHook = [&]() { lifted.tick(); };
  for (int i=0; i<400; ++i) { delay(1); pollESP(); }
  processESPMessage("[TEST:240:L:10:2]");
  const uint32_t liftedAt = fakeNow;
  int starts = 0; bool wasRunning = false;
  tickHook = [&]() { processESPMessage("[TEST KEEP:240]");
    const bool running = motorsRunning();
    if (running && !wasRunning) ++starts;
    wasRunning = running;
    if (fakeNow-liftedAt > 10000) std::exit(10); };
  loop(); tickHook = backgroundHook = nullptr;
  std::printf("INFO lifted turn: starts=%d steps=%ld/%ld ms=%lu\n", starts,
              wheel1.getCurrentPosition(), wheel2.getCurrentPosition(), (unsigned long)(fakeNow-liftedAt));
  check(ESP_Serial.tx.find("[TEST FAILED:240:NO_ROTATION]") != std::string::npos && starts == 1 &&
        !motorsRunning() && fakeNow - liftedAt < TEST_LIMIT_MS && !routeFaultLatched &&
        heading.stepsPerDeg() == HeadingControl::DEFAULT_STEPS_PER_DEG,
        "a lifted turn spins once, ends as NO_ROTATION, and learns nothing");

  fixture();
  hallway_sim::Plant flicker;
  flicker.walls = false;
  flicker.gyroScale = 0.0;
  flicker.attach(&wheel1, &wheel2, &ESP_Serial.rx);
  backgroundHook = [&]() { flicker.tick(); };
  for (int i=0; i<400; ++i) { delay(1); pollESP(); }
  processESPMessage("[TEST:250:R:10:3]");
  const uint32_t flickerAt = fakeNow;
  starts = 0; wasRunning = false;
  tickHook = [&]() { processESPMessage("[TEST KEEP:250]");
    // LiDAR turn clearance flickering: blocked 150 ms every 300 ms. The old resume
    // rule re-spun 5 times in 8 s on this pattern.
    flicker.sendGo = (fakeNow - flickerAt) % 300 >= 150;
    if (!flicker.sendGo) processESPMessage("STOP");
    const bool running = motorsRunning();
    if (running && !wasRunning) ++starts;
    wasRunning = running;
    if (fakeNow-flickerAt > 10000) std::exit(11); };
  loop(); tickHook = backgroundHook = nullptr;
  std::printf("INFO flickering turn: starts=%d steps=%ld ms=%lu failure=%s\n", starts,
              wheel1.getCurrentPosition(), (unsigned long)(fakeNow-flickerAt), developerFailure.c_str());
  check(starts <= 1 && !motorsRunning() &&
        ESP_Serial.tx.find("[TEST FAILED:250:HELD_LIDAR]") != std::string::npos,
        "a test turn held by flickering LiDAR clearance stops once and never re-spins");

  fixture();
  hallway_sim::Plant smooth;
  smooth.walls = smooth.sendGo = false;
  smooth.attach(&wheel1, &wheel2, &ESP_Serial.rx);
  backgroundHook = [&]() { smooth.tick(); };
  for (int i=0; i<400; ++i) { delay(1); pollESP(); }
  processESPMessage("[TEST:260:L:10:2]");
  unsigned spinApplies = 0;
  tickHook = [&]() { processESPMessage("[TEST KEEP:260]");
    if (spinInProgress) spinApplies = wheel1.applies; };
  wheel1.applies = 0;
  loop(); tickHook = backgroundHook = nullptr;
  std::printf("INFO floor test turn: ramp re-plans during spins=%u (125 before the fix)\n", spinApplies);
  check(ESP_Serial.tx.find("[TEST DONE:260]") != std::string::npos && spinApplies < 40,
        "a floor test turn holds one capped speed instead of re-planning it continuously");

  // Backward (owner, 2026-11-16: "backward was not working at all" on the bench).
  // A lifted route faults at its first turn, so the parking step's backward travel
  // could not be seen anywhere. B runs the same rear scope: [BACK:START], GO:BACK.
  fixture(); processESPMessage("[TEST:300:B:401:0]");
  check(garbyState == GarbyState::IDLE && !motorsRunning(), "oversized backward travel cannot start");
  processESPMessage("[TEST:301:B:400:0]");
  check(ESP_Serial.tx.find("[TEST ACK:301]") != std::string::npos, "a finite backward test is accepted");
  const uint32_t backAt = fakeNow;
  bool scopeSeen = false; uint32_t fastest = 0;
  tickHook = [&]() { processESPMessage("[TEST KEEP:301]");
    processESPMessage("GO");   // in flight as the scope began: never a hold with LiDAR deselected
    processESPMessage("STOP");
    scopeSeen = scopeSeen || (backingActive && motorsRunning());
    fastest = max(fastest, max(wheel1.speed, wheel2.speed));
    if (fakeNow - backAt > 10000) std::exit(12); };
  loop(); tickHook = nullptr;
  std::printf("INFO backward test: steps=%ld/%ld failure=%s\n", wheel1.getCurrentPosition(),
              wheel2.getCurrentPosition(), developerFailure.c_str());
  check(wheel1.getCurrentPosition() == -400 && wheel2.getCurrentPosition() == -400 && !motorsRunning() &&
        ESP_Serial.tx.find("[TEST DONE:301]") != std::string::npos,
        "the backward test turns both wheels backwards by its steps");
  check(scopeSeen && fastest <= TEST_SPEED_HZ && !backingActive &&
        ESP_Serial.tx.find("[BACK:START]") != std::string::npos &&
        ESP_Serial.tx.rfind("[BACK:END]") > ESP_Serial.tx.find("[BACK:START]"),
        "it backs inside the parking rear scope at the capped test speed, and the scope ends with it");
  check(garbyState == GarbyState::IDLE && shouldStop && !pathCommandFresh() && operatorManualStartRequired,
        "a backward test ends at IDLE with the normal gates restored");

  fixture(); processESPMessage("[TEST:310:B:400:1]");
  const uint32_t plainAt = fakeNow;
  tickHook = [&]() { processESPMessage("[TEST KEEP:310]"); processESPMessage("GO");
    if (fakeNow - plainAt > 10000) std::exit(13); };
  loop(); tickHook = nullptr;
  check(wheel1.getCurrentPosition() == 0 && wheel2.getCurrentPosition() == 0 && !motorsRunning() &&
        ESP_Serial.tx.find("[TEST FAILED:310:HELD_REAR]") != std::string::npos,
        "with LiDAR selected a plain forward GO never moves it backwards; the rear check is named");

  fixture(); processESPMessage("[TEST:311:B:400:1]");
  const uint32_t rearAt = fakeNow;
  tickHook = [&]() { processESPMessage("[TEST KEEP:311]");
    processESPMessage(backingActive ? "GO:BACK" : "GO");
    if (fakeNow - rearAt > 10000) std::exit(14); };
  loop(); tickHook = nullptr;
  check(wheel1.getCurrentPosition() == -400 && wheel2.getCurrentPosition() == -400 &&
        ESP_Serial.tx.find("[TEST DONE:311]") != std::string::npos,
        "with LiDAR selected the bridge's rear clearance (GO:BACK) lets it back");

  fixture(); processESPMessage("[TEST:312:B:400:1]");
  const uint32_t blockAt = fakeNow;
  tickHook = [&]() { processESPMessage("[TEST KEEP:312]");
    processESPMessage(wheel1.getCurrentPosition() > -200 ? "GO:BACK" : "STOP:BACK");
    if (fakeNow - blockAt > 10000) std::exit(15); };
  loop(); tickHook = nullptr;
  check(!motorsRunning() && wheel1.getCurrentPosition() < 0 && wheel1.getCurrentPosition() > -400 &&
        ESP_Serial.tx.find("[TEST FAILED:312:HELD_REAR]") != std::string::npos,
        "something appearing behind brakes the backward test once and names the rear check");

  fixture(); processESPMessage("[TEST:313:B:400:4]");
  const uint32_t sonarBackAt = fakeNow;
  recordFrontSample(10);
  tickHook = [&]() { processESPMessage("[TEST KEEP:313]"); recordFrontSample(10);
    if (fakeNow - sonarBackAt > 10000) std::exit(16); };
  loop(); tickHook = nullptr;
  check(wheel1.getCurrentPosition() == -400 && ESP_Serial.tx.find("[TEST DONE:313]") != std::string::npos,
        "the forward sonar faces away from backward travel and does not hold it, as in parking");

  fixture();
  hallway_sim::Plant backPlant;
  backPlant.walls = backPlant.sendGo = false;
  backPlant.gyroScale = 0.0;   // lifted wheels: the gyro sees nothing, which backing does not need
  backPlant.attach(&wheel1, &wheel2, &ESP_Serial.rx);
  backgroundHook = [&]() { backPlant.tick(); };
  for (int i=0; i<400; ++i) { delay(1); pollESP(); }
  processESPMessage("[TEST:314:B:400:2]");
  const uint32_t gyroBackAt = fakeNow;
  tickHook = [&]() { processESPMessage("[TEST KEEP:314]"); processESPMessage("STOP");
    if (fakeNow - gyroBackAt > 10000) std::exit(17); };
  loop(); tickHook = backgroundHook = nullptr;
  check(wheel1.getCurrentPosition() == -400 && wheel2.getCurrentPosition() == -400 &&
        ESP_Serial.tx.find("[TEST DONE:314]") != std::string::npos,
        "a lifted backward test with the gyro selected runs under live MPU supervision");

  // Remote control (owner, 2026-11-16): arrows that drive only while held, no step
  // value, no time limit, no sensor checks; Slow 2351 / Fast 4702 steps/s. Letting go
  // ([TEST END]) stops it; a lost KEEP stream stops it within 500 ms.
  fixture(); processESPMessage("[TEST:400:W:4703:0]");
  check(garbyState == GarbyState::IDLE && !motorsRunning(), "a held drive faster than 4702 steps/s is refused");
  processESPMessage("[TEST:401:W:400:1]");
  check(garbyState == GarbyState::IDLE && !motorsRunning(), "a held drive cannot carry sensor gates it does not use");

  fixture(); processESPMessage("[TEST:402:W:4702:0]");
  check(ESP_Serial.tx.find("[TEST ACK:402]") != std::string::npos, "a held forward drive is accepted");
  const uint32_t heldAt = fakeNow;
  uint32_t heldFastest = 0; bool released = false; long atRelease = 0;
  tickHook = [&]() { processESPMessage("[TEST KEEP:402]"); processESPMessage("STOP");
    heldFastest = max(heldFastest, max(wheel1.speed, wheel2.speed));
    if (!released && fakeNow - heldAt > 9000) {
      released = true; atRelease = wheel1.getCurrentPosition(); processESPMessage("[TEST END:402]"); }
    if (fakeNow - heldAt > 20000) std::exit(20); };
  loop(); tickHook = nullptr;
  std::printf("INFO held forward: steps=%ld/%ld after release=%ld fastest=%u\n", wheel1.getCurrentPosition(),
              wheel2.getCurrentPosition(), wheel1.getCurrentPosition() - atRelease, (unsigned)heldFastest);
  check(released && ESP_Serial.tx.find("[TEST DONE:402]") != std::string::npos && !motorsRunning() &&
        wheel1.getCurrentPosition() > 8000 && wheel1.getCurrentPosition() == wheel2.getCurrentPosition(),
        "a held drive runs past the 8 s test limit, ignoring STOP, until it is let go");
  check(heldFastest > TEST_SPEED_HZ && heldFastest <= 4702 &&
        wheel1.getCurrentPosition() - atRelease <= 200,
        "Fast drives at up to 4702 steps/s and letting go brakes it within a few cm");
  check(garbyState == GarbyState::IDLE && shouldStop && !pathCommandFresh() && operatorManualStartRequired &&
        frontObstacleLatched, "after a held drive every normal gate is back and no route can start by itself");

  fixture(); processESPMessage("[TEST:403:S:2351:0]");
  const uint32_t backHeldAt = fakeNow;
  tickHook = [&]() { processESPMessage("[TEST KEEP:403]");
    if (fakeNow - backHeldAt > 2000) processESPMessage("[TEST END:403]");
    if (fakeNow - backHeldAt > 6000) std::exit(21); };
  loop(); tickHook = nullptr;
  check(wheel1.getCurrentPosition() < -600 && wheel2.getCurrentPosition() < -600 &&
        wheel1.getCurrentPosition() == wheel2.getCurrentPosition() &&
        ESP_Serial.tx.find("[TEST DONE:403]") != std::string::npos,
        "Down drives both wheels backwards while held");

  for (const char* turn : {"A", "D"}) {
    fixture();
    const std::string id = std::string(turn) == "A" ? "404" : "405";
    processESPMessage(("[TEST:" + id + ":" + turn + ":2351:0]").c_str());
    const uint32_t turnHeldAt = fakeNow;
    tickHook = [&]() { processESPMessage(("[TEST KEEP:" + id + "]").c_str());
      if (fakeNow - turnHeldAt > 1500) processESPMessage(("[TEST END:" + id + "]").c_str());
      if (fakeNow - turnHeldAt > 6000) std::exit(22); };
    loop(); tickHook = nullptr;
    const long left = heading.leftMotor() == &wheel1 ? wheel1.getCurrentPosition() : wheel2.getCurrentPosition();
    const long right = heading.leftMotor() == &wheel1 ? wheel2.getCurrentPosition() : wheel1.getCurrentPosition();
    const bool leftTurn = std::string(turn) == "A";
    check(left == -right && (leftTurn ? left < -300 : left > 300) &&
          ESP_Serial.tx.find("[TEST DONE:" + id + "]") != std::string::npos,
          leftTurn ? "Left spins in place to the left (left wheel back, right wheel forward) while held"
                   : "Right spins in place to the right while held");
  }

  fixture(); processESPMessage("[TEST:406:W:4702:0]");
  const uint32_t keptAt = fakeNow; uint32_t lastKeep = fakeNow, lastRunning = 0;
  tickHook = [&]() {
    if (fakeNow - keptAt < 1000) { processESPMessage("[TEST KEEP:406]"); lastKeep = fakeNow; }
    if (motorsRunning()) lastRunning = fakeNow;
    if (fakeNow - keptAt > 5000) std::exit(23); };
  loop(); tickHook = nullptr;
  std::printf("INFO held drive, Pi gone: last ran %ld ms after the last KEEP\n", (long)(lastRunning - lastKeep));
  check(lastRunning > lastKeep && lastRunning - lastKeep <= 550 && !motorsRunning() &&
        ESP_Serial.tx.find("[TEST FAILED:406:LEASE_EXPIRED]") != std::string::npos,
        "if the Pi or the window goes quiet, the held drive stops within its 500 ms lease");

  fixture(); processESPMessage("[TEST:407:W:2351:0]"); processESPMessage("[TEST END:407]");
  tickHook = [&]() { processESPMessage("[TEST KEEP:407]"); };
  loop(); tickHook = nullptr;
  check(wheel1.getCurrentPosition() == 0 && wheel2.getCurrentPosition() == 0 &&
        ESP_Serial.tx.find("[TEST DONE:407]") != std::string::npos,
        "a tap released before the drive started never moves the wheels");

  fixture(); processESPMessage("[TEST:408:W:2351:0]");
  const uint32_t stopAt = fakeNow;
  tickHook = [&]() { processESPMessage("[TEST KEEP:408]"); processESPMessage("[TEST END:999]");
    if (fakeNow - stopAt > 1000) processESPMessage("[OP:1408:ESTOP]");
    if (fakeNow - stopAt > 5000) std::exit(24); };
  loop(); tickHook = nullptr;
  check(operatorEmergencyLatched && !motorsRunning() && wheel1.getCurrentPosition() > 0 &&
        ESP_Serial.tx.find("[TEST FAILED:408:EMERGENCY_STOP]") != std::string::npos,
        "another drive's release is ignored, and the emergency stop still ends a held drive");

  for (int dir : {1, -1}) {
    fixture();
    hallway_sim::Plant floor;
    floor.walls = floor.sendGo = false;
    floor.attach(&wheel1, &wheel2, &ESP_Serial.rx);
    backgroundHook = [&]() { floor.tick(); };
    for (int i=0;i<400;++i) { delay(1); pollESP(); }
    const std::string id=dir>0?"901":"902";
    const std::string request="[TEST:"+id+":"+(dir>0?"L":"R")+":90:2]";
    processESPMessage(request.c_str());
    const uint32_t began=fakeNow;
    uint32_t maxSpeed=0;
    tickHook = [&]() {
      processESPMessage(("[TEST KEEP:"+id+"]").c_str());
      processESPMessage("STOP");  // LiDAR explicitly deselected for this test
      maxSpeed=max(maxSpeed,max(wheel1.speed,wheel2.speed));
      if(fakeNow-began>32000) std::exit(30);
    };
    loop(); tickHook = backgroundHook = nullptr;
    check(ESP_Serial.tx.find("[TEST DONE:"+id+"]")!=std::string::npos &&
          fabs(floor.thetaDeg-dir*90.0)<1.1 && maxSpeed<=TEST_SPEED_HZ &&
          maxSpeed == TEST_SPEED_HZ && fakeNow-began<TEST_TURN90_LIMIT_MS,
          dir>0 ? "left 90 degree test completes on measured rotation with a 30 cm/s speed setting and endpoint braking"
                : "right 90 degree test completes on measured rotation with a 30 cm/s speed setting and endpoint braking");
    check(!motorsRunning() && garbyState==GarbyState::IDLE && shouldStop &&
          !pathCommandFresh() && operatorManualStartRequired && wheel1.forcedStops>0 && wheel2.forcedStops>0,
          "a completed 90 degree test force-stops both wheels and restores route gates at IDLE");
  }

  fixture();
  hallway_sim::Plant lifted90;
  lifted90.walls=lifted90.sendGo=false; lifted90.gyroScale=0.0;
  lifted90.attach(&wheel1,&wheel2,&ESP_Serial.rx);
  backgroundHook=[&]() { lifted90.tick(); };
  for(int i=0;i<400;++i) {delay(1);pollESP();}
  processESPMessage("[TEST:903:R:90:2]");
  const uint32_t lifted90At=fakeNow;
  tickHook=[&]() {processESPMessage("[TEST KEEP:903]");};
  loop();tickHook=backgroundHook=nullptr;
  check(ESP_Serial.tx.find("[TEST FAILED:903:NO_ROTATION]")!=std::string::npos &&
        !motorsRunning() && fakeNow-lifted90At<2000 && !routeFaultLatched,
        "unchanged MPU heading ends a lifted 90 degree test promptly with NO_ROTATION");

  fixture();
  hallway_sim::Plant lease90;
  lease90.walls=lease90.sendGo=false;lease90.attach(&wheel1,&wheel2,&ESP_Serial.rx);
  backgroundHook=[&]() {lease90.tick();};
  for(int i=0;i<400;++i) {delay(1);pollESP();}
  processESPMessage("[TEST:904:L:90:2]");
  const uint32_t lease90At=fakeNow;
  tickHook=[&]() {if(fakeNow-lease90At<1000)processESPMessage("[TEST KEEP:904]");};
  loop();tickHook=backgroundHook=nullptr;
  check(ESP_Serial.tx.find("[TEST FAILED:904:LEASE_EXPIRED]")!=std::string::npos &&
        !motorsRunning() && fakeNow-lease90At<2000,
        "the 90 degree test retains its 800 ms KEEP lease despite its longer absolute limit");

  fixture();processESPMessage("[TEST:905:L:90:2]");
  const uint32_t missing90At=fakeNow;
  tickHook=[&]() {processESPMessage("[TEST KEEP:905]");};
  loop();tickHook=nullptr;
  check(ESP_Serial.tx.find("[TEST FAILED:905:HELD_MPU]")!=std::string::npos &&
        wheel1.getCurrentPosition()==0 && wheel2.getCurrentPosition()==0 &&
        fakeNow-missing90At>=TEST_TURN90_LIMIT_MS && fakeNow-missing90At<TEST_TURN90_LIMIT_MS+100,
        "a missing MPU holds a 90 degree test still and reports HELD_MPU at its 30-second limit");
  fixture();
  for (const char* request : {"[TEST:910:L:90:1]","[TEST:911:R:89:2]",
                              "[TEST:912:L:91:2]","[TEST:913:R:16:2]"}) processESPMessage(request);
  check(!developerQueued && !motorsRunning() && garbyState==GarbyState::IDLE,
        "the 90 degree addition does not allow other large angles or turn without MPU supervision");

  // The MPU can still report rotation after the pulse queues have stopped.
  // A quarter-second delay alone used to report DONE during that rotation.
  fixture();
  hallway_sim::Plant coast;
  coast.walls = coast.sendGo = false;
  coast.attach(&wheel1, &wheel2, &ESP_Serial.rx);
  uint32_t coastAt = 0, quietAt = 0;
  bool spun = false;
  backgroundHook = [&]() {
    const bool running = motorsRunning();
    if (running) spun = true;
    if (spun && !running && coastAt == 0) coastAt = fakeNow;
    if (coastAt != 0 && fakeNow - coastAt < 550) {
      // Additional chassis motion independent of commanded step counters.
      coast.thetaDeg += .002;
      coast.gyroBiasDps = 2.0;
    } else if (coastAt != 0) {
      coast.gyroBiasDps = 0.0;
      if (quietAt == 0) quietAt = fakeNow;
    }
    coast.tick();
  };
  for (int i=0; i<400; ++i) {delay(1); pollESP();}
  processESPMessage("[TEST:920:L:90:2]");
  tickHook = [&]() {processESPMessage("[TEST KEEP:920]");};
  loop(); tickHook = backgroundHook = nullptr;
  check(quietAt != 0 && fakeNow - quietAt >= 200 && !motorsRunning(),
        "a measured turn cannot finish while fresh MPU samples still show chassis rotation");

  uint32_t settlingId = 921;
  for (const std::string cause : {"MPU_DATA_LOST", "EMERGENCY_STOP", "TURN_UNSETTLED"}) {
    const std::string id = std::to_string(settlingId++);
    fixture();
    hallway_sim::Plant settling;
    settling.walls = settling.sendGo = false;
    settling.attach(&wheel1, &wheel2, &ESP_Serial.rx);
    uint32_t stoppedAt = 0;
    bool moved = false;
    backgroundHook = [&]() {
      if (motorsRunning()) moved = true;
      if (moved && !motorsRunning() && !stoppedAt) stoppedAt = fakeNow;
      if (stoppedAt && cause == "TURN_UNSETTLED") settling.gyroBiasDps = 2.0;
      settling.tick();
    };
    for (int i=0; i<400; ++i) {delay(1); pollESP();}
    processESPMessage(("[TEST:" + id + ":L:90:2]").c_str());
    tickHook = [&]() {
      processESPMessage(("[TEST KEEP:" + id + "]").c_str());
      if (stoppedAt && fakeNow - stoppedAt >= 100) {
        if (cause == "MPU_DATA_LOST") settling.streamMpu = false;
        if (cause == "EMERGENCY_STOP") processESPMessage("[OP:1921:ESTOP]");
      }
    };
    loop(); tickHook = backgroundHook = nullptr;
    std::printf("INFO settling %s: stopped=%lu failure=%s outcome=%s\n", cause.c_str(),
                (unsigned long)stoppedAt, developerFailure.c_str(), developerOutcome.c_str());
    check(stoppedAt && !motorsRunning() &&
          ESP_Serial.tx.find("[TEST FAILED:" + id + ":" + cause + "]") != std::string::npos &&
          ESP_Serial.tx.find("[TEST DONE:" + id + "]") == std::string::npos,
          "MPU loss, emergency stop and ongoing rotation cannot pass settled turn confirmation");
  }

  return failures ? 1 : 0;
}
