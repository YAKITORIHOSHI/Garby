#include "stubs/Arduino.h"
#include "../../NAPHTALI_CODE_V2/NAPHTALI_CODE_V2.cpp"
#include "../../NAPHTALI_CODE_V2/pointsRun.ino"
#include "../../NAPHTALI_CODE_V2/NAPHTALI_CODE_V2.ino"

static FastAccelStepper leftWheel, rightWheel;
static void resetFixture() {
  tickHook = nullptr;
  fakeNow = 1000;
  leftWheel = FastAccelStepper(); rightWheel = FastAccelStepper();
  stepper1 = &leftWheel; stepper2 = &rightWheel;
  fullReset();
  resetRequestId = ""; resetOutcome = ResetOutcome::NONE;
  garbyState = GarbyState::RUNNING;
  uartRxBuffer = ""; ESP_Serial.rx.clear(); ESP_Serial.tx.clear();
  frontObstacleLatched = frontEmergencyActive = false;
  latestFrontSampleValid = false;
  frontFilterCount = frontFilterIndex = ultrasonicBlockedCount = ultrasonicClearCount = 0;
  frontSampleSequence = 0; ultrasonicPingInFlight = false;
}
static void clearPath() {processESPMessage("GO");fakeNow+=100;processESPMessage("GO");}
static void check(bool ok,const char* label){if(!ok){std::fprintf(stderr,"FAIL %s\n",label);std::exit(1);}std::printf("PASS %s\n",label);}
int main() {
  resetFixture(); processESPMessage("GO"); check(shouldStop,"one GO cannot release STOP");
  fakeNow+=1000; processESPMessage("GO"); check(shouldStop,"widely separated GO evidence does not accumulate");
  fakeNow+=100; processESPMessage("GO"); check(!shouldStop,"two fresh GO commands release STOP");
  fakeNow+=801; enforcePathWatchdog(); check(shouldStop,"controller independently stops on silence");
  resetFixture(); clearPath(); ESP_Serial.rx="STOP\nGO\nGO\n"; pollESP();
  check(shouldStop && !ESP_Serial.rx.empty(),"queued GO cannot erase STOP before braking");
  resetFixture(); clearPath(); rightWheel.rejectNext=true;
  check(!startTargetMove(1000,1000) && !leftWheel.isRunning() && !rightWheel.isRunning(),"one rejected motor command stops both wheels");
  resetFixture(); clearPath(); recordFrontSample(150); recordFrontSample(150); recordFrontSample(45);
  check(frontObstacleDetected(),"single close echo stops despite clear median history");
  recordFrontSample(20); recordFrontSample(999,false);
  check(frontObstacleDetected(),"no echo never clears a latched obstacle");
  for(int i=0;i<3;i++){fakeNow+=40;recordFrontSample(120);}
  check(!frontObstacleDetected(),"three valid clear echoes release local latch");
  resetFixture(); clearPath(); recordFrontSample(20);
  const auto haltStarted=fakeNow;
  tickHook=[&](){if(fakeNow-haltStarted>200){if(ultrasonicPingInFlight){ultrasonicEchoReady=true;ultrasonicEchoWidthUs=7000;}processESPMessage("GO");}if(fakeNow-haltStarted>2000){std::fprintf(stderr,"HALT never recovered\n");std::exit(1);}};
  haltAndWait("SONIC:BLOCKED");
  check(fakeNow-haltStarted>=200 && fakeNow-haltStarted<2000,"halt continues sensing and resumes on local plus LiDAR clearance");
  resetFixture(); clearPath(); bool blockedOnce=false; const auto moveStarted=fakeNow;
  tickHook=[&](){auto age=fakeNow-moveStarted;if(age>40 && !blockedOnce){processESPMessage("STOP");recordFrontSample(20);blockedOnce=true;}if(age>250){if(ultrasonicPingInFlight){ultrasonicEchoReady=true;ultrasonicEchoWidthUs=7000;}processESPMessage("GO");}if(age>5000){std::fprintf(stderr,"MOVE never recovered\n");std::exit(1);}};
  check(moveToTarget(1000,1000,false,true) && blockedOnce && leftWheel.position==1000 && rightWheel.position==1000,"blocked move resumes the original target and completes");
  resetFixture(); clearPath(); nudgeLeftContinuous(220,25); const auto nudgedSpeed=leftWheel.speed;
  fakeNow+=100; updateNudge(); check(activeNudge==NudgeDir::LEFT && nudgedSpeed<rightWheel.speed,"nudge persists until its requested duration");
  fakeNow+=121; updateNudge(); check(activeNudge==NudgeDir::NONE && leftWheel.speed==rightWheel.speed,"nudge expiry restores both wheels");
  resetFixture(); processESPMessage("[RESET:1234567890000]");
  check(routeFaultLatched && resetOutcome==ResetOutcome::FAILED,"return from incomplete outbound is rejected");
  processESPMessage("[RESET:1234567890000]"); check(garbyState==GarbyState::RUNNING,"retry cannot turn rejected return into recovery");
  resetFixture(); outboundComplete=true; processESPMessage("[RESET:1234567890001]");
  check(resetQueued && resetOutcome==ResetOutcome::ACTIVE,"return accepted only from confirmed destination");
  resetQueued=false; garbyState=GarbyState::RETURNING; shouldStop=false;
  processESPMessage("[RESET:1234567890001]"); check(!shouldStop && !resetQueued,"same return ID is idempotent during return");
  fullReset(); ESP_Serial.tx.clear(); processESPMessage("[RESET:1234567890001]");
  check(ESP_Serial.tx.find("[RESET DONE:1234567890001]")!=std::string::npos,"completed retry returns matching completion without motion");
}
