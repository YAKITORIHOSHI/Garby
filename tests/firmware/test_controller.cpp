#include "stubs/Arduino.h"
#include "../../MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.cpp"
#include "../../MAIN_EXECUTOR_CODE/pointsRun.ino"
#include "../../MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.ino"

static FastAccelStepper leftWheel, rightWheel;
// The executor refuses to move without the bridge's heading stream (2026-11-11).
// These tests are about gates and routes, so a steady, level heading is fed.
static uint32_t mpuSequence=0;
static bool mpuStreaming=true;
static void mpuLine(uint32_t at,long headingCdeg=0){
  char payload[64];std::snprintf(payload,sizeof(payload),"M,%lu,%ld,0,1,3",(unsigned long)++mpuSequence,headingCdeg);
  uint8_t x=0;for(const char* c=payload;*c;++c)x^=(uint8_t)*c;
  char line[72];std::snprintf(line,sizeof(line),"%s*%02X",payload,(unsigned)x);
  heading.handleLine(line,at);
}
static void feedMpu(){
  static uint32_t last=0;
  if(mpuStreaming && fakeNow-last>=20){last=fakeNow;mpuLine(fakeNow);}
}
static void primeMpu(){for(int k=12;k>=0;--k)mpuLine(fakeNow-(uint32_t)k*20U);}
static void resetFixture() {
  tickHook = nullptr; backgroundHook = feedMpu; mpuStreaming = true; heading = HeadingControl();
  fakeNow = 1000;
  operatorStopMarker = 0;
  operatorEmergencyLatched = operatorManualStartRequired = false;
  operatorSonarEnabled = true;
  operatorRebootScheduled = false;
  operatorLastId = operatorLastAction = operatorLastFailure = "";
  operatorLastAccepted = false;
  ESP.restartCount = 0;
  leftWheel = FastAccelStepper(); rightWheel = FastAccelStepper();
  stepper1 = &leftWheel; stepper2 = &rightWheel;
  fullReset();
  resetRequestId = ""; resetOutcome = ResetOutcome::NONE;
  startRequestId = ""; lastStartRequestNumber = 0; startAccepted = false;
  startFailure = ""; manualStartLinkReady = false;
  lastConnected = false; lastBridgeRxMs = 0;
  garbyState = GarbyState::RUNNING;
  uartRxBuffer = ""; ESP_Serial.rx.clear(); ESP_Serial.tx.clear();
  uartDiscardUntilNewline=uartCarriageReturnSeen=false;
  frontObstacleLatched = frontEmergencyActive = false;
  latestFrontSampleValid = false;
  frontFilterCount = frontFilterIndex = ultrasonicBlockedCount = ultrasonicClearCount = 0;
  frontSampleSequence = 0; ultrasonicPingInFlight = false;
}
static void clearPath() {processESPMessage("GO");fakeNow+=100;processESPMessage("GO");primeMpu();}
static unsigned failures=0;
static void check(bool ok,const char* label){if(!ok){std::fprintf(stderr,"FAIL %s\n",label);failures++;}else std::printf("PASS %s\n",label);}
int main() {
  resetFixture(); garbyState=GarbyState::IDLE; ESP_Serial.tx.clear(); reportRobotStatus();
  check(ESP_Serial.tx.find("[RETURN COMPLETE]")==std::string::npos,
        "boot and generic IDLE do not claim a completed return");
  garbyState=GarbyState::RETURNING; fullReset(true); ESP_Serial.tx.clear(); reportRobotStatus();
  check(ESP_Serial.tx.find("[IDLE]\n[RETURN COMPLETE]\n")!=std::string::npos,
        "a successful return retains its completion for reconnect status replay");
  fullReset(); ESP_Serial.tx.clear(); reportRobotStatus();
  check(ESP_Serial.tx.find("[RETURN COMPLETE]")==std::string::npos,
        "supervised reset does not invent return completion");
  garbyState=GarbyState::RETURNING; operatorEmergencyLatched=true; fullReset(true);
  ESP_Serial.tx.clear(); reportRobotStatus();
  check(ESP_Serial.tx.find("[RETURN COMPLETE]")==std::string::npos,
        "an emergency stop prevents a return arrival report");
  resetFixture(); garbyState=GarbyState::RETURNING; routeFaultLatched=true; fullReset(true);
  ESP_Serial.tx.clear(); reportRobotStatus();
  check(ESP_Serial.tx.find("[RETURN COMPLETE]")==std::string::npos,
        "a route fault prevents a return arrival report");
  resetFixture(); garbyState=GarbyState::RETURNING; resetQueued=true; fullReset(true);
  ESP_Serial.tx.clear(); reportRobotStatus();
  check(ESP_Serial.tx.find("[RETURN COMPLETE]")==std::string::npos,
        "a pending abort prevents a return arrival report");
  resetFixture(); fullReset(true); ESP_Serial.tx.clear(); reportRobotStatus();
  check(ESP_Serial.tx.find("[RETURN COMPLETE]")==std::string::npos,
        "an outbound or developer completion cannot report a completed return");
  resetFixture(); garbyState=GarbyState::RETURNING; fullReset(true);
  processESPMessage("[BLE CONNECTION ESTABLISHED]");
  processESPMessage("[START:1234567890123]"); ESP_Serial.tx.clear(); reportRobotStatus();
  check(garbyState==GarbyState::RUNNING && !returnComplete &&
        ESP_Serial.tx.find("[RETURN COMPLETE]")==std::string::npos,
        "a new outbound request clears the retained return arrival before status replay");
  resetFixture();spinSonarCheck=true;ESP_Serial.tx.clear();
  processESPMessage("[BLE CONNECTION ESTABLISHED]");
  check(ESP_Serial.tx.find("[TURN:START]\n")<ESP_Serial.tx.find("[SYSTEM READY]\n"),
        "handshake replays an interrupted turn before announcing readiness");
  spinSonarCheck=false;ESP_Serial.tx.clear();processESPMessage("[ROBOT STATUS]");
  check(ESP_Serial.tx.find("[TURN:END]\n")<ESP_Serial.tx.find("[SYSTEM READY]\n"),
        "status reports explicitly restore non-turn mode before readiness");
  resetFixture();clearPath();leftWheel.runForward();rightWheel.runForward();
  processESPMessage("[OP:1:ESTOP]");
  check(operatorEmergencyLatched && shouldStop && resetQueued &&
        !leftWheel.isRunning() && !rightWheel.isRunning() &&
        ESP_Serial.tx.find("[OP ACK:1:ESTOP]")!=std::string::npos,
        "operator emergency stop force-stops both wheels and acknowledges the latch");
  clearPath();processESPMessage("[RESET]");processESPMessage("[BLE CONNECTION ESTABLISHED]");
  processESPMessage("[START:1234567890001]");
  check(operatorEmergencyLatched && shouldStop && !leftWheel.isRunning() &&
        ESP_Serial.tx.find("EMERGENCY_STOP")!=std::string::npos,
        "GO, legacy reset, reconnect, and manual start cannot erase emergency stop");
  processESPMessage("[OP:2:RELEASE]");
  check(operatorEmergencyLatched && ESP_Serial.tx.find("[OP FAILED:2:NOT_IDLE]")!=std::string::npos,
        "release cannot occur inside a still-active route stack");
  loop();processESPMessage("[OP:3:RELEASE]");
  check(!operatorEmergencyLatched && garbyState==GarbyState::IDLE && shouldStop &&
        operatorManualStartRequired && !pathCommandSeen,
        "release after route unwinds leaves IDLE requiring a fresh deliberate start");
  recordLoadCellSample(5);loop();fakeNow+=100;recordLoadCellSample(5);loop();
  check(garbyState==GarbyState::IDLE,
        "high sensor readings cannot automatically restart a released emergency stop");
  processESPMessage("[OP:3:SONAR_OFF]");
  check(operatorSonarEnabled && ESP_Serial.tx.find("[OP FAILED:3:ID_REUSED]")!=std::string::npos,
        "operator ID cannot be reused for a different action");
  processESPMessage("[OP:2:SONAR_OFF]");
  check(operatorSonarEnabled && ESP_Serial.tx.find("[OP FAILED:2:STALE_REQUEST]")!=std::string::npos,
        "stale operator action cannot override the current mode");
  processESPMessage("[OP:4:SONAR_OFF]");
  check(!operatorSonarEnabled && servo.angle==DEFAULT_VIEW,
        "sonar disable is acknowledged at stationary IDLE and parks the servo");
  ultrasonicPingInFlight=true;cancelUltrasonicPing();
  serviceUltrasonic();recordFrontSample(1);commandServoTracked(SCAN_LEFT);scanAngle(SCAN_RIGHT);
  check(!ultrasonicPingInFlight && !frontObstacleDetected() && !frontDistanceFresh() &&
        servo.angle==DEFAULT_VIEW && readDistanceRaw()==999,
        "disabled sonar cannot ping, provide obstacle readings, or move the servo off center");
  processESPMessage("[OP:5:SONAR_ON]");
  check(operatorSonarEnabled && frontObstacleDetected() && !frontDistanceFresh(),
        "re-enabled sonar fails closed until new clearance confirmations arrive");
  recordFrontSample(100);recordFrontSample(100);
  check(frontObstacleDetected(),"two new sonar clear samples cannot bypass rearm confirmation");
  recordFrontSample(100);
  check(!frontObstacleDetected(),"three new clear samples rearm local obstacle sonar");
  processESPMessage("[OP:6:REBOOT]");
  check(!operatorRebootScheduled && ESP_Serial.tx.find("ESTOP_REQUIRED")!=std::string::npos,
        "MCU reboot requires acknowledged emergency stop");
  processESPMessage("[OP:7:ESTOP]");loop();
  processESPMessage("[OP:8:REBOOT]");
  check(operatorRebootScheduled && operatorEmergencyLatched && ESP.restartCount==0,
        "reboot is committed only while emergency-stopped and is delayed for ACK delivery");
  fakeNow+=750;serviceOperatorReboot();
  check(ESP.restartCount==1 && operatorEmergencyLatched,
        "delayed reboot retains emergency latch for software restart");
  operatorEmergencyLatched = false;restoreOperatorStop();
  check(operatorEmergencyLatched,"RTC no-init marker restores operator stop after RAM initialization");
  resetFixture();garbyState=GarbyState::IDLE;processESPMessage("[BLE CONNECTION ESTABLISHED]");
  processESPMessage("[OP:99999999999999999999:SONAR_OFF]");
  check(!operatorSonarEnabled,"20-digit operator IDs are validated without integer overflow");
  processESPMessage("[OP:01:SONAR_ON]");
  check(!operatorSonarEnabled,"noncanonical operator IDs are ignored");
  resetFixture();clearPath();
  processESPMessage("[OP:1:SONAR_OFF]");
  check(operatorSonarEnabled,"sonar mode changes cannot apply during an active route");
  const auto stoppedRouteStarted=fakeNow;
  bool operatorStopInjected=false;
  tickHook=[&](){
    if(!operatorStopInjected && fakeNow-stoppedRouteStarted>=30){
      operatorStopInjected=true;processESPMessage("[OP:2:ESTOP]");
    }
    processESPMessage("GO");
    if(fakeNow-stoppedRouteStarted>3000){std::fprintf(stderr,"operator stop did not unwind route\n");std::exit(1);}
  };
  check(!safeMoveDistance(10000,false,true) && operatorStopInjected &&
        !leftWheel.isRunning() && !rightWheel.isRunning(),
        "emergency stop inside actual motion aborts route without resuming from queued GO");
  tickHook=nullptr;

  resetFixture();garbyState=GarbyState::IDLE;clearPath();
  processESPMessage("[START:1234567890000]");
  check(garbyState==GarbyState::IDLE && ESP_Serial.tx.find("[START FAILED:1234567890000:NOT_READY]")!=std::string::npos,
        "GO traffic alone cannot replace the Pi/controller startup handshake");
  resetFixture();garbyState=GarbyState::IDLE;
  recordLoadCellSample(std::numeric_limits<float>::quiet_NaN());
  processESPMessage("[BLE CONNECTION ESTABLISHED]");
  check(ESP_Serial.tx.find("[SYSTEM READY]")!=std::string::npos &&
        ESP_Serial.tx.find("[DISPOSAL READY:1]")!=std::string::npos &&
        ESP_Serial.tx.find("[IDLE]")!=std::string::npos,
        "ready executor reports manual disposal without bin, gas or scale readings");
  ESP_Serial.tx.clear();operatorEmergencyLatched=true;reportRobotStatus();
  check(ESP_Serial.tx.find("[DISPOSAL READY:")==std::string::npos,
        "emergency stop withholds manual disposal readiness");
  operatorEmergencyLatched=false;routeFaultLatched=true;ESP_Serial.tx.clear();reportRobotStatus();
  check(ESP_Serial.tx.find("[DISPOSAL READY:")==std::string::npos,
        "route fault withholds manual disposal readiness");
  routeFaultLatched=false;
  ESP_Serial.tx.clear();processESPMessage("[START STATUS:1234567890001]");
  check(garbyState==GarbyState::IDLE && startRequestId.length()==0 &&
        ESP_Serial.tx.find("[START FAILED:1234567890001:NOT_RECEIVED]")!=std::string::npos,
        "querying an undelivered start reports failure without starting the route");
  ESP_Serial.tx.clear();
  processESPMessage("[START:1234567890001]");
  check(garbyState==GarbyState::RUNNING && startAccepted &&
        ESP_Serial.tx.find("[START ACK:1234567890001]")!=std::string::npos,
        "manual start enters the normal outbound route with no scale/bin trigger");
  check(shouldStop && !pathCommandSeen && !leftWheel.isRunning() && !rightWheel.isRunning(),
        "manual start cannot grant path permission or directly move either wheel");
  check(ESP_Serial.tx.find("[DISPOSAL READY:")==std::string::npos,
        "accepted manual disposal withholds eligibility for a second run");
  ESP_Serial.tx.clear();processESPMessage("[START STATUS:1234567890001]");
  check(garbyState==GarbyState::RUNNING &&
        ESP_Serial.tx.find("[START ACK:1234567890001]")!=std::string::npos && shouldStop,
        "a lost manual-start ACK can be queried without changing the motion gate");
  processESPMessage("[START:1234567890001]");
  check(garbyState==GarbyState::RUNNING && shouldStop && startAccepted,
        "duplicate manual start acknowledges its original outcome without changing motion gates");
  fullReset();processESPMessage("[START:1234567890001]");
  check(garbyState==GarbyState::IDLE,"accepted start replay after return cannot start another cycle");
  ESP_Serial.tx.clear();processESPMessage("[START STATUS:1234567890001]");
  check(garbyState==GarbyState::IDLE &&
        ESP_Serial.tx.find("[START ACK:1234567890001]")!=std::string::npos,
        "querying a completed start cannot begin another cycle");
  routeFaultLatched=true;processESPMessage("[START:1234567890002]");
  check(garbyState==GarbyState::IDLE && !startAccepted && startFailure=="ROUTE_FAULT",
        "manual start cannot clear a route fault");
  routeFaultLatched=false;processESPMessage("[START:1234567890001]");
  check(garbyState==GarbyState::IDLE && ESP_Serial.tx.find("STALE_REQUEST")!=std::string::npos,
        "older accepted start remains non-replayable after a newer rejected request");
  garbyState=GarbyState::RETURNING;processESPMessage("[START:1234567890003]");
  check(garbyState==GarbyState::RETURNING && !startAccepted && startFailure=="NOT_IDLE",
        "manual start cannot interrupt a returning route");
  resetFixture();garbyState=GarbyState::IDLE;processESPMessage("[BLE CONNECTION ESTABLISHED]");
  fakeNow+=BRIDGE_RX_STALE_MS+1;processESPMessage("[START:1234567890004]");
  check(garbyState==GarbyState::IDLE && startFailure=="NOT_READY",
        "stale UART cannot be refreshed into readiness by the start command itself");
  processESPMessage("[BLE CONNECTION ESTABLISHED]");processESPMessage("STOP:LINK");
  processESPMessage("[START:1234567890005]");
  check(garbyState==GarbyState::IDLE && startFailure=="NOT_READY",
        "BLE loss revokes the executor manual-start handshake");
  processESPMessage("[BLE CONNECTION ESTABLISHED]");
  const auto lastStartBeforeBad=lastStartRequestNumber;
  for(const char* packet : {"[START:]", "[START:0]", "[START:01]", "[START:abc]", "[START:12345678900060000]", "[START:1234567890006"}) {
    processESPMessage(packet);
  }
  check(garbyState==GarbyState::IDLE && lastStartRequestNumber==lastStartBeforeBad,
        "malformed manual-start IDs cannot change executor state");
  processESPMessage("STOP:PROTOCOL");ESP_Serial.tx.clear();
  lastReadyRecoveryMs=0;serviceBridgeRecovery();
  check(ESP_Serial.tx.find("[MCU READY]")!=std::string::npos,
        "idle executor renews a revoked startup handshake even while UART traffic is fresh");
  processESPMessage("[BLE CONNECTION ESTABLISHED]");processESPMessage("[START:1234567890006]");
  check(startAccepted && garbyState==GarbyState::RUNNING,
        "manual start recovers after a new handshake without a controller reboot");
  resetFixture();garbyState=GarbyState::IDLE;processESPMessage("[BLE CONNECTION ESTABLISHED]");
  stepper2=nullptr;processESPMessage("[START:1234567890007]");
  check(garbyState==GarbyState::IDLE &&
        ESP_Serial.tx.find("[START FAILED:1234567890007:MOTORS_UNAVAILABLE]")!=std::string::npos,
        "uninitialized motor output is rejected instead of accepting an immobile route");
  resetFixture();garbyState=GarbyState::IDLE;processESPMessage("[BLE CONNECTION ESTABLISHED]");
  processESPMessage("STOP:MPU");ESP_Serial.tx.clear();processESPMessage("[ROBOT STATUS]");
  check(shouldStop && ESP_Serial.tx.find("[MOTION:STOP:MPU]")!=std::string::npos,
        "status exposes an MPU motion hold to the Pi without clearing STOP");
  clearPath();recordFrontSample(20);ESP_Serial.tx.clear();processESPMessage("[ROBOT STATUS]");
  check(ESP_Serial.tx.find("[MOTION:LOCAL_OBSTACLE]")!=std::string::npos,
        "status exposes the executor's local sonar hold even with clear LiDAR");
  resetFixture();clearPath();recordFrontSample(20);
  recordFrontSample(120);recordFrontSample(120);
  fakeNow+=ULTRASONIC_SAMPLE_MAX_AGE_MS+1;
  recordFrontSample(120);
  check(frontObstacleDetected(),"clear echoes separated by an outage cannot release obstacle latch");
  recordFrontSample(120);recordFrontSample(120);
  check(!frontObstacleDetected(),"three consecutive fresh echoes recover after outage");
  tickHook=nullptr;
  resetFixture();clearPath();leftWheel.frozen=true;
  const auto frozenStarted=fakeNow;
  tickHook=[&](){processESPMessage("GO");if(fakeNow-frozenStarted>4000){resetQueued=true;}};
  check(!moveToTarget(100000,100000,false,true) && routeFaultLatched &&
        !leftWheel.isRunning() && !rightWheel.isRunning() && fakeNow-frozenStarted<4000,
        "one stalled software step counter stops both wheels and faults the route");
  resetFixture();clearPath();leftWheel.frozen=true;
  commandWheelTargets(100000,100000);
  for(int i=0;i<5;i++) {
    fakeNow+=500;
    commandWheelTargets(100000+i,100000-i);
  }
  check(!serviceMotionProgress() && routeFaultLatched,
        "retargeting a running wheel cannot hide stalled step progress");
  for(int mode=0;mode<1;mode++) {
    resetFixture();clearPath();rightWheel.frozen=true;
    const auto started=fakeNow;
    tickHook=[&](){processESPMessage("GO");if(fakeNow-started>4000) resetQueued=true;};
    const bool completed=moveToTarget(100000,100000,true,false);
    check(!completed && routeFaultLatched && !leftWheel.isRunning() && !rightWheel.isRunning(),
          "adjustment stops both wheels when one executor stalls");
  }
  resetFixture();clearPath();rightWheel.rejectNext=true;
  startStraight();
  check(!leftWheel.isRunning() && !rightWheel.isRunning() && !movingForward,
        "continuous start rejection cannot leave one wheel running");
  resetFixture(); processESPMessage("GO"); check(shouldStop,"one GO cannot release STOP");
  fakeNow+=1000; processESPMessage("GO"); check(shouldStop,"widely separated GO evidence does not accumulate");
  fakeNow+=100; processESPMessage("GO"); check(!shouldStop,"two fresh GO commands release STOP");
  fakeNow+=801; enforcePathWatchdog(); check(shouldStop,"controller independently stops on silence");
  resetFixture(); clearPath(); ESP_Serial.rx="STOP\nGO\nGO\n"; pollESP();
  check(shouldStop && !ESP_Serial.rx.empty(),"queued GO cannot erase STOP before braking");
  pollESP();check(shouldStop,"back-to-back buffered GO commands cannot release STOP");
  fakeNow+=MCU_GO_CONFIRM_MIN_INTERVAL_MS;processESPMessage("GO");
  check(!shouldStop,"a later fresh GO supplies independent clearance evidence");
  resetFixture();clearPath();ESP_Serial.rx=std::string(UART_RX_LINE_MAX+1,'X')+"\nGO\nGO\n";
  pollESP();check(shouldStop && linkFaultActive,"oversized UART line immediately invalidates motion permission");
  pollESP();check(shouldStop,"queued GO after UART overflow cannot bypass new confirmation");
  resetFixture();clearPath();ESP_Serial.rx=std::string("GO")+char(1)+"\nGO\nGO\n";
  pollESP();check(shouldStop && linkFaultActive,"non-printable UART data immediately invalidates permission");
  pollESP();check(shouldStop,"queued GO after corrupt UART cannot erase STOP");
  resetFixture();clearPath();ESP_Serial.rx="GO\r\n";pollESP();
  check(!shouldStop && pathCommandFresh(),"ordinary CRLF UART frames preserve live permission");
  ESP_Serial.rx="G\rO\nGO\nGO\n";pollESP();
  check(shouldStop && linkFaultActive,"embedded carriage return cannot be stripped into valid GO");
  resetFixture(); clearPath(); rightWheel.rejectNext=true;
  check(!startTargetMove(1000,1000) && !leftWheel.isRunning() && !rightWheel.isRunning(),"one rejected motor command stops both wheels");
  // 2026-11-14: one stray forward echo inside the stop distance no longer brakes
  // the robot (a short stop-and-go); two consecutive ones do, as at the sides.
  // Within VERY_CLOSE_DISTANCE_CM a single echo still latches at once.
  resetFixture(); clearPath(); recordFrontSample(150); recordFrontSample(150); recordFrontSample(45);
  check(!frontObstacleDetected(),"a single 45 cm echo after a clear history does not stop on its own");
  recordFrontSample(150);
  check(!frontObstacleDetected(),"a stray echo the next ping does not repeat never latches");
  recordFrontSample(45); recordFrontSample(45);
  check(frontObstacleDetected(),"two consecutive forward echoes inside the stop distance stop");
  resetFixture(); clearPath(); recordFrontSample(150); recordFrontSample(150); recordFrontSample(22);
  check(frontObstacleDetected(),"a single echo within the very-close distance stops at once");
  recordFrontSample(20); recordFrontSample(999,false);
  check(frontObstacleDetected(),"a single ping with no echo does not clear a latched obstacle");
  for(int i=0;i<3;i++){fakeNow+=40;recordFrontSample(120);}
  check(!frontObstacleDetected(),"three valid clear echoes release local latch");
  // 2026-11-12: no echo inside the timeout means nothing within ~3 m (an open
  // hallway). It counts as clear, with the same three-sample confirmation.
  recordFrontSample(20); recordFrontSample(20);
  for(int i=0;i<2;i++){fakeNow+=40;recordFrontSample(999,false);}
  check(frontObstacleDetected(),"two pings with no echo cannot release the latch");
  fakeNow+=40;recordFrontSample(999,false);
  check(!frontObstacleDetected() && frontDistanceFresh() && latestFrontDistance()>=ULTRASONIC_CLEAR_DISTANCE_CM,
        "three pings with no echo in range (open hallway) release it as a far, fresh reading");
  recordFrontSample(0.5f);
  check(!frontObstacleDetected(),"an echo under 1.5 cm is a glitch, not an obstacle or a clear reading");
  // 2026-11-14: the sonar reads only to 1.1 m. A ping gives up after that echo
  // time, and an echo from further away is "nothing in range" (a clear reading).
  check(ULTRASONIC_TIMEOUT_US < 7000UL && ULTRASONIC_TIMEOUT_US > 6000UL,
        "a ping waits only for the echo time of the sonar's 1.1 m range (was 18 ms, about 3 m)");
  resetFixture(); clearPath(); commandServoTracked(DEFAULT_VIEW); fakeNow+=500;
  ultrasonicPingInFlight=true; ultrasonicPingCommand=DEFAULT_VIEW; ultrasonicPingStartedUs=micros();
  ultrasonicEchoReady=true; ultrasonicEchoWidthUs=7000;  // 120 cm
  serviceUltrasonic();
  check(frontDistanceFresh() && latestFrontRawCm>=999.0f && !frontObstacleDetected(),
        "an echo from 120 cm, beyond the sonar's range, counts as nothing in range");
  ultrasonicPingInFlight=false; fakeNow+=500;
  ultrasonicPingInFlight=true; ultrasonicPingCommand=DEFAULT_VIEW; ultrasonicPingStartedUs=micros();
  ultrasonicEchoReady=true; ultrasonicEchoWidthUs=5831;  // 100 cm
  serviceUltrasonic();
  check(std::fabs(latestFrontRawCm-100.0f)<0.5f,"an echo inside the range is read as its distance");
  ultrasonicPingInFlight=false;
  resetFixture(); clearPath(); recordFrontSample(20);
  const auto haltStarted=fakeNow;
  tickHook=[&](){if(fakeNow-haltStarted>200){if(ultrasonicPingInFlight){ultrasonicEchoReady=true;ultrasonicEchoWidthUs=7000;}processESPMessage("GO");}if(fakeNow-haltStarted>2000){std::fprintf(stderr,"HALT never recovered\n");std::exit(1);}};
  haltAndWait("SONIC:BLOCKED");
  check(fakeNow-haltStarted>=200 && fakeNow-haltStarted<2000,"halt continues sensing and resumes on local plus LiDAR clearance");
  resetFixture();clearPath();recordFrontSample(50);
  const auto hysteresisStarted=fakeNow;bool deadbandObserved=false,holdReported=true;
  tickHook=[&](){
    processESPMessage("GO");
    if(fakeNow-hysteresisStarted<300){
      recordFrontSample(70);ESP_Serial.tx.clear();processESPMessage("[ROBOT STATUS]");
      deadbandObserved=true;
      holdReported=holdReported && ESP_Serial.tx.find("[MOTION:LOCAL_OBSTACLE]")!=std::string::npos;
    } else if(ultrasonicPingInFlight){ultrasonicEchoReady=true;ultrasonicEchoWidthUs=7000;}
    if(fakeNow-hysteresisStarted>2000){std::fprintf(stderr,"Hysteresis hold never recovered\n");std::exit(1);}
  };
  haltAndWait("SONIC:BLOCKED");tickHook=nullptr;
  check(deadbandObserved && holdReported,
        "local clearance hysteresis keeps reporting a hold until the halt actually exits");
  ESP_Serial.tx.clear();processESPMessage("[ROBOT STATUS]");
  check(ESP_Serial.tx.find("[MOTION:CLEAR]")!=std::string::npos,
        "confirmed halt recovery clears the displayed motion hold");
  resetFixture(); clearPath(); bool blockedOnce=false; const auto moveStarted=fakeNow;
  tickHook=[&](){auto age=fakeNow-moveStarted;if(age>40 && !blockedOnce){processESPMessage("STOP");recordFrontSample(20);blockedOnce=true;}if(age>250){if(ultrasonicPingInFlight){ultrasonicEchoReady=true;ultrasonicEchoWidthUs=7000;}processESPMessage("GO");}if(age>5000){std::fprintf(stderr,"MOVE never recovered\n");std::exit(1);}};
  check(moveToTarget(1000,1000,false,true) && blockedOnce && leftWheel.position==1000 && rightWheel.position==1000,"blocked move resumes the original target and completes");
  resetFixture(); processESPMessage("[RESET:1234567890000]");
  check(routeFaultLatched && resetOutcome==ResetOutcome::FAILED,"return from incomplete outbound is rejected");
  processESPMessage("[RESET:1234567890000]"); check(garbyState==GarbyState::RUNNING,"retry cannot turn rejected return into recovery");
  resetFixture(); outboundComplete=true; processESPMessage("[RESET:1234567890001]");
  check(resetQueued && resetOutcome==ResetOutcome::ACTIVE,"return accepted only from confirmed destination");
  resetQueued=false; garbyState=GarbyState::RETURNING; shouldStop=false;
  processESPMessage("[RESET:1234567890001]"); check(!shouldStop && !resetQueued,"same return ID is idempotent during return");
  fullReset(); ESP_Serial.tx.clear(); processESPMessage("[RESET:1234567890001]");
  check(ESP_Serial.tx.find("[RESET DONE:1234567890001]")!=std::string::npos,"completed retry returns matching completion without motion");
  resetFixture();garbyState=GarbyState::IDLE;
  recordLoadCellSample(std::numeric_limits<float>::quiet_NaN());loop();
  check(garbyState==GarbyState::IDLE && ESP_Serial.tx.find("LOAD_CELL:-1.00")!=std::string::npos,"invalid weight stays idle and publishes unavailable telemetry");
  fakeNow+=100;recordLoadCellSample(50);loop();
  check(garbyState==GarbyState::IDLE,"out-of-range raw weight cannot contaminate load confirmation");
  fakeNow+=100;recordLoadCellSample(5);loop();
  check(garbyState==GarbyState::IDLE,"one valid weight after invalid data cannot trigger outbound");
  fakeNow+=100;recordLoadCellSample(5);loop();
  check(garbyState==GarbyState::RUNNING,"valid weight recovers after invalid samples without a reset");
  resetFixture();garbyState=GarbyState::IDLE;recordLoadCellSample(5);loop();
  fakeNow+=LOAD_SAMPLE_MAX_AGE_MS+1;loop();
  check(garbyState==GarbyState::IDLE,"stale cached weight cannot start outbound");
  recordLoadCellSample(5);loop();
  check(garbyState==GarbyState::IDLE,"weight after a stale gap starts a new confirmation streak");
  fakeNow+=100;recordLoadCellSample(5);loop();
  check(garbyState==GarbyState::RUNNING,"two new weight samples restore triggering after an outage");
  scale.ready=false;
  check(checkLoad(1),"load helper uses the fresh worker snapshot without waiting on the converter");
  fakeNow+=LOAD_SAMPLE_MAX_AGE_MS+1;
  check(!checkLoad(1),"load helper rejects expired worker samples");
  recordLoadCellSample(std::numeric_limits<float>::quiet_NaN());
  check(!checkLoad(1),"load helper rejects invalid samples");
  struct WorkerStopped {};
  scale.ready=false;scale.units=2;scale.offset=0;
  const uint32_t workerStarted=fakeNow;
  bool unavailableDuringTare=true,workerRecovered=false;
  tickHook=[&](){
    float kilograms;uint32_t sequence;
    const bool valid=getLoadCellSample(kilograms,sequence);
    if(!scale.ready) unavailableDuringTare=unavailableDuringTare && !valid;
    if(fakeNow-workerStarted>=2500) scale.ready=true;
    if(valid){workerRecovered=scale.offset==200000 && kilograms>1;throw WorkerStopped{};}
    if(fakeNow-workerStarted>5000){std::fprintf(stderr,"LOAD worker never recovered\n");std::exit(1);}
  };
  try{loadCellWorkerTask(nullptr);}catch(const WorkerStopped&){}
  tickHook=nullptr;
  check(unavailableDuringTare,"missing converter cannot produce a valid untared weight");
  check(workerRecovered,"background tare retries and publishes only after calibration succeeds");
  for(float raw : {0.01f, 4.0f, -5.0f}) {
    resetFixture();garbyState=GarbyState::IDLE;
    recordLoadCellSample(raw);loop();fakeNow+=100;recordLoadCellSample(raw);loop();
    check(garbyState==GarbyState::IDLE,"empty, below corrected 4kg, or negative load cannot trigger route");
  }
  resetFixture();garbyState=GarbyState::IDLE;
  recordLoadCellSample(4.01f);loop();
  check(garbyState==GarbyState::IDLE,"one corrected 4kg sample cannot trigger route");
  fakeNow+=100;recordLoadCellSample(4.01f);loop();
  check(garbyState==GarbyState::RUNNING,"two corrected 4kg samples trigger route");
  resetFixture();garbyState=GarbyState::IDLE;
  recordLoadCellSample(0.01f);
  Air780.rx=std::string(1000,'A');Serial.rx=std::string(1000,'B');
  loop();
  check(Air780.rx.size()==1000 && Serial.rx.size()==1000,
        "main loop leaves cellular UART and console bytes untouched for the single modem owner");
  Air780.rx.clear();Serial.rx.clear();
  resetFixture();garbyState=GarbyState::IDLE;
  const uint32_t bootStarted=fakeNow;setup();
  check(fakeNow-bootStarted<1000 && !leftWheel.isRunning() && !rightWheel.isRunning(),
        "executor boot returns promptly with no bridge and leaves motors stopped for loop recovery");
  // The hold reason must name a lost MPU reference, not blame the LiDAR.
  resetFixture();processESPMessage("STOP:MPU");
  check(motionHoldReason("LINK:STALE").startsWith("MPU:"),"a lost heading reference is not reported as a LiDAR block");
  processESPMessage("STOP");
  check(motionHoldReason("LINK:STALE")=="PATH:BLOCKED from LiDAR","an ordinary obstacle stop still reports the LiDAR");
  processESPMessage("STOP:STALE");
  check(motionHoldReason("LINK:STALE")=="LINK:STALE","a link stop still reports the link");
  // During a route, silence on the bridge UART uses the ping, not the boot line.
  resetFixture();lastBridgeRxMs=0;lastReadyRecoveryMs=0;ESP_Serial.tx.clear();serviceBridgeRecovery();
  check(ESP_Serial.tx.find("[MCU ALIVE]")!=std::string::npos && ESP_Serial.tx.find("[MCU READY]")==std::string::npos,
        "recovery ping during a route is distinct from the controller boot announcement");
  // Parked at the destination the Pi's own 4 Hz stream keeps path data fresh.
  // Polling at the moving rate only spends BLE airtime and Pi CPU for hours.
  resetFixture();outboundComplete=true;ESP_Serial.tx.clear();
  for(int i=0;i<250;i++) loop();  // about 5 s of the parked 20 ms branch
  {std::size_t polls=0,at=0;while((at=ESP_Serial.tx.find("[REQUEST-STATUS]",at))!=std::string::npos){polls++;at++;}
   check(polls>=1 && polls<=12,"parked at the destination the controller polls status at the idle rate");}
  return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
