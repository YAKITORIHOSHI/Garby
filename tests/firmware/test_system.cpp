#include "stubs/Arduino.h"
#include "stubs/NimBLEDevice.h"
#include "stubs/Wire.h"
namespace bridge {
#include "../../BLE_Receiver-Final/BLE_Receiver-Final.ino"
}
// The two physical boards intentionally use different UART TX pins.
#undef ESP_RX
#undef ESP_TX
namespace controller {
#include "../../MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.cpp"
#include "../../MAIN_EXECUTOR_CODE/pointsRun.ino"
#include "../../MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.ino"
}

static FastAccelStepper wheel1, wheel2;
static uint32_t sequence=0;
// The chassis turns when the wheels differ, and the bridge's gyro sees it. Every
// 10 ms the wheel step difference becomes the MPU6050's Z rate (4900 steps per
// wheel per 90 degrees, motor 1 = left), the real reader polls it and the real
// 50 Hz heading stream runs, as in the bridge's loop. The raw sensor reads 10%
// low, matching the owner's approximate 100 physical / 90 reported observation.
static long physics1=0,physics2=0;
static uint32_t physicsMs=0;
static float injectedRateDps=0;
static void physics(){
  if(fakeNow-physicsMs<10)return;
  const float dt=(fakeNow-physicsMs)*0.001f;
  const long now1=wheel1.getCurrentPosition(),now2=wheel2.getCurrentPosition();
  const float rate=0.5f*(float)((now2-physics2)-(now1-physics1))/(4900.0f/90.0f)/dt+injectedRateDps;
  physics1=now1;physics2=now2;physicsMs=fakeNow;
  Wire.setGyroZ((int16_t)constrain(rate*0.9f*131.0f,-32000.0f,32000.0f));
  // Tests jump the clock; the real loop never leaves the gyro unread for 100 ms.
  if(bridge::mpuHeading.ready() && !bridge::mpuHeading.valid())bridge::startHeadingReference();
  bridge::mpuHeading.poll();
}
static void wireUart(){
  physics();
  bridge::serviceHeadingStream(fakeNow);
  controller::ESP_Serial.rx+=bridge::ESP_Serial.tx; bridge::ESP_Serial.tx.clear();
  controller::pollESP();
  bridge::ESP_Serial.rx+=controller::ESP_Serial.tx; controller::ESP_Serial.tx.clear();
  bridge::pollMcuUart();
}
// The Pi's path packet. Since 2026-11-16 it always carries the rear-travel code R=.
static void pathBody(const char* body){
  std::string packet="P:"+std::to_string(++sequence)+"|"+body;
  bridge::handlePathPacket(packet.c_str());
  const float yaw=bridge::mpuHeading.headingDeg();
  const float rearAngle=yaw-90.0f*std::round(yaw/90.0f);
  const std::string sides="S:"+std::to_string(sequence)+
      "|L=80|R=80|F=200|T=9|CF=200|CL=80|CR=80|WL=80|WR=80|G=0|A="+
      std::to_string(sequence)+"|RD=260|RH="+std::to_string(rearAngle);
  bridge::enqueueBleWrite(sides.c_str(), sides.size());
  bridge::processBleIngress();
  wireUart();wireUart();
}
static void path(bool blocked){pathBody(blocked?"F=O|B=C|T=C|R=C":"F=C|B=C|T=C|R=C");}
static void check(bool ok,const char* label){if(!ok){std::fprintf(stderr,"FAIL %s\n",label);std::exit(1);}std::printf("PASS %s\n",label);}
static std::string notifications(){
  std::string messages;
  for(const auto& frame:bridge::pNotifyChar->notifications) messages+=frame.substr(1);
  return messages;
}
static void initialize(){
  tickHook=nullptr; fakeNow=10000;
  controller::stepper1=&wheel1;controller::stepper2=&wheel2;
  controller::fullReset();controller::garbyState=controller::GarbyState::RUNNING;
  bridge::resetIngressQueues();bridge::resetNavigationState();
  // Each case starts a fresh controller epoch. A real BLE reconnect within a
  // case deliberately retains its heading; resetting this fixture must not.
  bridge::handleMcuLine("[HEADING:END]");
  bridge::deviceConnected=true;bridge::mcuReady=true;
  bridge::mcuUnackedCommands=0;bridge::mcuAckPending=bridge::mcuAckFault=false;
  bridge::ESP_Serial.tx.clear();controller::ESP_Serial.tx.clear();
  // One continuous heading reference, streamed before any path is released.
  Wire=TwoWire{};injectedRateDps=0;
  if(!bridge::mpuHeading.begin(8,9)){std::fprintf(stderr,"MPU fixture failed\n");std::exit(1);}
  bridge::startHeadingReference();
  controller::heading=controller::HeadingControl();
  physics1=wheel1.getCurrentPosition();physics2=wheel2.getCurrentPosition();physicsMs=fakeNow;
  // Production status restores the executor's turn mode after bridge boot.
  controller::reportRobotStatus();wireUart();
  for(int i=0;i<30;i++){fakeNow+=10;wireUart();}
  for(int i=0;i<4;i++){for(int k=0;k<10;k++){fakeNow+=10;wireUart();}path(false);}
}
int main(){
  bridge::setup();initialize();
  check(!controller::shouldStop,"Pi path stream releases both bridge and controller confirmation gates");
  const auto healthPathAt=controller::lastPathCommandMs;
  const auto healthBridgeAt=controller::lastBridgeRxMs;
  bridge::enqueueBleWrite("[HEALTH:1:DXUUU]",16);bridge::processBleIngress();wireUart();wireUart();
  check(controller::healthMonitors[0].value=='D' && controller::healthMonitors[1].value=='X' &&
        controller::lastPathCommandMs==healthPathAt && controller::lastBridgeRxMs==healthBridgeAt &&
        !controller::shouldStop && bridge::mcuUnackedCommands==0,
        "Pi health traverses both boards and acknowledges without refreshing path or readiness evidence");
  check(bridge::mpuHeading.valid() && controller::heading.headingReady(fakeNow),
        "the bridge's continuous heading stream is confirmed by the executor before any route");
  controller::heading.confirmTurnSides();controller::heading.beginRoute(fakeNow);
  const auto headingRunStarted=fakeNow;
  uint32_t headingPathMs=fakeNow;
  bool rightCorrectionSeen=false;
  tickHook=[&](){
    const auto age=fakeNow-headingRunStarted;
    // The chassis is pushed left at 10 deg/s for the first 0.4 s of the segment.
    injectedRateDps=(age>=300 && age<700)?10.0f:0.0f;
    wireUart();
    if(fakeNow-headingPathMs>=100){
      headingPathMs=fakeNow;path(false);
      const std::string sides="S:"+std::to_string(sequence)+"|L=80|R=80|F=200|T=9|CF=200|CL=80|CR=80|WL=80.0|WR=80.0|G=150|A="+std::to_string(sequence);
      bridge::enqueueBleWrite(sides.c_str(),sides.size());bridge::processBleIngress();wireUart();
    }
    if(controller::ultrasonicPingInFlight){controller::ultrasonicEchoReady=true;controller::ultrasonicEchoWidthUs=7000;}
    rightCorrectionSeen=rightCorrectionSeen ||
      (controller::heading.appliedCut()>0.0f && wheel2.speed<wheel1.speed);
    if(age>10000){std::fprintf(stderr,"MPU heading run timed out\n");std::exit(1);}
  };
  check(controller::safeMoveDistance(25000,false,true) && rightCorrectionSeen,
        "real MPU samples flow through the bridge UART into the correct continuous wheel-speed correction");
  check(std::fabs(controller::heading.routeError())<1.5f,
        "the heading hold brings the pushed chassis back to its route heading");
  check(controller::heading.wallLines()>0 && controller::heading.widthKnown() &&
        std::fabs(controller::heading.widthCm()-160.0f)<2.0f,
        "the Pi's wall fit crosses BLE framing, the bridge and the UART into the executor's fusion");
  wireUart();wireUart();
  check(!bridge::straightRunActive && bridge::mpuHeading.valid(),
        "segment completion ends the bridge's segment notice and keeps its continuous heading");
  tickHook=nullptr;initialize();
  const auto started=fakeNow;uint32_t lastPath=fakeNow;bool stopped=false;
  tickHook=[&](){
    auto age=fakeNow-started;
    if(fakeNow-lastPath>=100){lastPath=fakeNow;path(age>=100 && age<500);}
    wireUart();
    if(controller::ultrasonicPingInFlight){controller::ultrasonicEchoReady=true;controller::ultrasonicEchoWidthUs=(age>=100 && age<500)?1166:7000;}
    if(age>=100 && age<500 && !wheel1.isRunning() && !wheel2.isRunning()) stopped=true;
    if(age>5000){std::fprintf(stderr,"system recovery timed out\n");std::exit(1);}
  };
  check(controller::safeMoveDistance(2000,false,true) && stopped,"complete sensing-to-UART-to-motor flow stops and resumes original route target");
  wireUart();wireUart();
  check(!bridge::mcuAckFault && bridge::mcuUnackedCommands==0,"STOP and GO ACKs drain without a permanent communication fault");
  tickHook=nullptr;initialize();
  const auto reconnectStarted=fakeNow;lastPath=fakeNow;bool disconnected=false,reconnected=false;
  bridge::ServerCallbacks callbacks;NimBLEConnInfo info;
  tickHook=[&](){
    auto age=fakeNow-reconnectStarted;
    if(age>=100 && !disconnected){disconnected=true;static_cast<NimBLEServerCallbacks*>(&callbacks)->onDisconnect(bridge::pServer,info,8);bridge::loop();}
    if(age>=500 && !reconnected){reconnected=true;static_cast<NimBLEServerCallbacks*>(&callbacks)->onConnect(bridge::pServer,info);bridge::loop();}
    if(bridge::deviceConnected && fakeNow-lastPath>=100){lastPath=fakeNow;path(false);}
    wireUart();
    if(controller::ultrasonicPingInFlight){controller::ultrasonicEchoReady=true;controller::ultrasonicEchoWidthUs=7000;}
    if(age>5000){std::fprintf(stderr,"reconnection recovery timed out\n");std::exit(1);}
  };
  check(controller::safeMoveDistance(2000,false,true) && disconnected && reconnected,"BLE reconnect resumes interrupted target after fresh confirmations");
  // Reconnecting to a real robot usually takes longer than the controller's 3 s
  // RX-stale window. Its recovery ping must not erase the segment's heading.
  tickHook=nullptr;initialize();
  const uint32_t outageEpoch=bridge::headingEpoch;
  controller::ESP_Serial.println("[HEADING:START]");fakeNow+=10;wireUart();
  check(bridge::straightRunActive && bridge::mpuHeading.valid(),"heading reference exists before the long BLE outage");
  static_cast<NimBLEServerCallbacks*>(&callbacks)->onDisconnect(bridge::pServer,info,8);bridge::loop();
  bridge::ESP_Serial.tx.clear();
  for(int i=0;i<500;i++){fakeNow+=10;if(i%5==0)controller::serviceBridgeRecovery();wireUart();}
  check(bridge::straightRunActive && bridge::mpuHeading.valid() && bridge::headingEpoch==outageEpoch &&
        controller::heading.headingReady(fakeNow),
        "a 5 s BLE outage keeps the heading reference and its stream to the executor");
  static_cast<NimBLEServerCallbacks*>(&callbacks)->onConnect(bridge::pServer,info);bridge::loop();wireUart();
  check(bridge::straightRunActive && bridge::mpuHeading.valid() && bridge::headingEpoch==outageEpoch,
        "reconnecting after a long outage resumes the same heading reference");
  bridge::handleMcuLine("[HEADING:END]");
  tickHook=nullptr;initialize();controller::fullReset();controller::recordLoadCellSample(5);
  controller::loop();fakeNow+=100;controller::recordLoadCellSample(5);controller::loop();
  check(controller::garbyState==controller::GarbyState::RUNNING,"confirmed load starts outbound state without bypassing path gate");
  const long scaleStartPosition1=wheel1.getCurrentPosition();
  const long scaleStartPosition2=wheel2.getCurrentPosition();
  const auto scaleRouteStarted=fakeNow;lastPath=fakeNow;
  bool blockedScaleRouteStayedStopped=true,scaleRouteDrove=false;
  tickHook=[&](){
    const auto age=fakeNow-scaleRouteStarted;
    if(fakeNow-lastPath>=100){lastPath=fakeNow;path(age<1200);}
    wireUart();
    if(controller::ultrasonicPingInFlight){controller::ultrasonicEchoReady=true;controller::ultrasonicEchoWidthUs=7000;}
    if(age<1200) {
      blockedScaleRouteStayedStopped=blockedScaleRouteStayedStopped &&
          wheel1.getCurrentPosition()==scaleStartPosition1 && wheel2.getCurrentPosition()==scaleStartPosition2;
    } else if(wheel1.getCurrentPosition()!=scaleStartPosition1 && wheel2.getCurrentPosition()!=scaleStartPosition2) {
      scaleRouteDrove=true;
    }
    if(age>400000){std::fprintf(stderr,"scale-start route timed out\n");std::exit(1);}
  };
  controller::loop();tickHook=nullptr;wireUart();wireUart();
  check(blockedScaleRouteStayedStopped && scaleRouteDrove && controller::outboundComplete && !controller::routeFaultLatched,
        "scale threshold holds wheels on blocked LiDAR, then drives the full route after confirmed clearance");
  check(bridge::ESP_Serial.rx.empty() && !bridge::backingActive && !controller::backingActive,
        "the full route ends its backing scope on both boards");

  // Reverse parking end to end (2026-11-16): the real bridge and executor. The Pi's
  // R= gates a backing move through GO:BACK; the front group does not.
  auto backingRun=[&](const char* body,uint32_t holdMs,const char* holdBody,uint32_t limitMs)->int{
    initialize();controller::heading.confirmTurnSides();controller::heading.beginRoute(fakeNow);
    const long start1=wheel1.getCurrentPosition(),start2=wheel2.getCurrentPosition();
    const auto t0=fakeNow;uint32_t lastP=fakeNow;bool movedDuringHold=false;
    tickHook=[&](){
      const auto age=fakeNow-t0;
      if(fakeNow-lastP>=100){lastP=fakeNow;pathBody(age<holdMs?holdBody:body);}
      wireUart();
      if(controller::ultrasonicPingInFlight){controller::ultrasonicEchoReady=true;controller::ultrasonicEchoWidthUs=700;}
      if(age<holdMs && age>300 && (wheel1.getCurrentPosition()!=start1 || wheel2.getCurrentPosition()!=start2))
        movedDuringHold=true;
      if(age>limitMs)controller::resetQueued=true;   // end a move that never gets permission
    };
    const bool ok=controller::backIntoPlace(2000);
    tickHook=nullptr;controller::resetQueued=false;wireUart();wireUart();
    const long moved1=wheel1.getCurrentPosition()-start1,moved2=wheel2.getCurrentPosition()-start2;
    if(movedDuringHold)return -1;
    if(!ok)return moved1==0 && moved2==0 ? 0 : -2;
    return moved1==-2000 && moved2==-2000 ? 1 : -3;
  };
  check(backingRun("F=O|B=C|T=O|R=C",0,"",20000)==1,
        "backing completes on a clear rear with an obstacle ahead (the front faces away), sonar echo 12 cm ahead");
  check(backingRun("F=C|B=C|T=C|R=C",1500,"F=C|B=C|T=C|R=O",20000)==1,
        "something behind holds the wheels still; the backing move resumes once the rear is clear");
  check(backingRun("F=C|B=C|T=C",0,"",4000)==0,
        "a Pi without the rear-travel code never lets the robot back up");
  check(backingRun("F=C|B=O|T=C|R=C",0,"",4000)==0,
        "the back group (side clearance) still holds a backing move");

  // Backward developer test end to end (owner, 2026-11-16, wheels lifted on the bench):
  // the request crosses BLE and both UARTs, the executor opens the parking rear scope
  // and the real bridge answers it with GO:BACK only on the Pi's R=C.
  auto backwardTest=[&](const char* body,const char* id,long* d1,long* d2)->std::string{
    initialize();controller::fullReset();controller::garbyState=controller::GarbyState::IDLE;
    controller::recordLoadCellSample(std::numeric_limits<float>::quiet_NaN());
    controller::manualStartLinkReady=false;controller::lastConnected=false;
    bridge::piReady=false;bridge::bleBridgeSignalSent=false;
    bridge::enqueueBleWrite("[RASPI READY]",13);bridge::loop();
    for(int i=0;i<4;i++) wireUart();
    bridge::pNotifyChar->notifications.clear();
    const long start1=wheel1.getCurrentPosition(),start2=wheel2.getCurrentPosition();
    const std::string request=std::string("[TEST:")+id+":B:400:1]";
    const std::string keep=std::string("[TEST KEEP:")+id+"]";
    bridge::enqueueBleWrite(request.c_str(),request.size());bridge::processBleIngress();
    wireUart();wireUart();
    const auto t0=fakeNow;uint32_t lastP=fakeNow,lastK=fakeNow;
    tickHook=[&](){
      if(fakeNow-lastP>=100){lastP=fakeNow;pathBody(body);}
      if(fakeNow-lastK>=200){lastK=fakeNow;bridge::enqueueBleWrite(keep.c_str(),keep.size());bridge::processBleIngress();}
      wireUart();
      if(fakeNow-t0>12000){std::fprintf(stderr,"backward test timed out\n");std::exit(1);}
    };
    controller::loop();tickHook=nullptr;wireUart();wireUart();
    *d1=wheel1.getCurrentPosition()-start1;*d2=wheel2.getCurrentPosition()-start2;
    return notifications();
  };
  long back1=0,back2=0;
  std::string backNotes=backwardTest("F=O|B=C|T=O|R=C","1794830000001",&back1,&back2);
  check(backNotes.find("[TEST DONE:1794830000001]")!=std::string::npos && back1==-400 && back2==-400 &&
        !bridge::backingActive && !controller::backingActive && controller::garbyState==controller::GarbyState::IDLE,
        "a bench backward test crosses both boards and reverses both wheels on a clear rear, front blocked");
  backNotes=backwardTest("F=C|B=C|T=C|R=O","1794830000002",&back1,&back2);
  check(backNotes.find("[TEST FAILED:1794830000002:HELD_REAR]")!=std::string::npos && back1==0 && back2==0,
        "something behind holds the bench backward test still and the monitor is told it was the rear check");

  // Remote control end to end (owner, 2026-11-16): held arrow, no sensor gates, no time
  // limit; letting go is a [TEST END] through BLE, the bridge and the UART.
  auto heldDrive=[&](const char* id,const char* action,uint32_t releaseMs,long* d1,long* d2)->std::string{
    initialize();controller::fullReset();controller::garbyState=controller::GarbyState::IDLE;
    controller::recordLoadCellSample(std::numeric_limits<float>::quiet_NaN());
    controller::manualStartLinkReady=false;controller::lastConnected=false;
    bridge::piReady=false;bridge::bleBridgeSignalSent=false;
    bridge::enqueueBleWrite("[RASPI READY]",13);bridge::loop();
    for(int i=0;i<4;i++) wireUart();
    bridge::pNotifyChar->notifications.clear();
    const long start1=wheel1.getCurrentPosition(),start2=wheel2.getCurrentPosition();
    const std::string request=std::string("[TEST:")+id+":"+action+":4702:0]";
    const std::string keep=std::string("[TEST KEEP:")+id+"]",end=std::string("[TEST END:")+id+"]";
    bridge::enqueueBleWrite(request.c_str(),request.size());bridge::processBleIngress();
    wireUart();wireUart();
    const auto t0=fakeNow;uint32_t lastP=fakeNow,lastK=fakeNow;bool ended=false;
    tickHook=[&](){
      if(fakeNow-lastP>=100){lastP=fakeNow;pathBody("F=O|B=O|T=O|R=O");}   // everything "blocked"
      if(fakeNow-t0<releaseMs && fakeNow-lastK>=100){lastK=fakeNow;bridge::enqueueBleWrite(keep.c_str(),keep.size());bridge::processBleIngress();}
      if(!ended && fakeNow-t0>=releaseMs){ended=true;bridge::enqueueBleWrite(end.c_str(),end.size());bridge::processBleIngress();}
      wireUart();
      if(fakeNow-t0>releaseMs+5000){std::fprintf(stderr,"held drive timed out\n");std::exit(1);}
    };
    controller::loop();tickHook=nullptr;wireUart();wireUart();
    *d1=wheel1.getCurrentPosition()-start1;*d2=wheel2.getCurrentPosition()-start2;
    return notifications();
  };
  long held1=0,held2=0;
  std::string heldNotes=heldDrive("1794830000003","W",10000,&held1,&held2);
  check(heldNotes.find("[TEST DONE:1794830000003]")!=std::string::npos && held1>15000 && held1==held2 &&
        !controller::backingActive && controller::garbyState==controller::GarbyState::IDLE,
        "a held Up arrow drives through both boards past eight seconds, with every LiDAR code blocked, until let go");
  heldNotes=heldDrive("1794830000004","S",1500,&held1,&held2);
  check(heldNotes.find("[TEST DONE:1794830000004]")!=std::string::npos && held1<-2000 && held1==held2,
        "a held Down arrow drives backwards and stops when let go");

  // RunStart end to end (owner, 2026-11-16): a list of steps with no sensor gates. The
  // emergency stop, its release and Continue run all cross BLE, the bridge and the UART.
  {
    initialize();controller::fullReset();controller::garbyState=controller::GarbyState::IDLE;
    controller::recordLoadCellSample(std::numeric_limits<float>::quiet_NaN());
    controller::manualStartLinkReady=false;controller::lastConnected=false;
    bridge::piReady=false;bridge::bleBridgeSignalSent=false;
    bridge::enqueueBleWrite("[RASPI READY]",13);bridge::loop();
    for(int i=0;i<4;i++) wireUart();
    bridge::pNotifyChar->notifications.clear();
    const long s1=wheel1.getCurrentPosition(),s2=wheel2.getCurrentPosition();
    auto runList=[&](const std::string& request,const std::string& id,std::function<bool()> stopNow){
      const std::string keep="[TEST KEEP:"+id+"]";
      bridge::enqueueBleWrite(request.c_str(),request.size());bridge::processBleIngress();
      wireUart();wireUart();
      const auto t0=fakeNow;uint32_t lastP=fakeNow,lastK=fakeNow;bool stopped=false;
      tickHook=[&](){
        if(fakeNow-lastP>=100){lastP=fakeNow;pathBody("F=O|B=O|T=O|R=O");}   // everything "blocked"
        if(!stopped && fakeNow-lastK>=100){lastK=fakeNow;bridge::enqueueBleWrite(keep.c_str(),keep.size());bridge::processBleIngress();}
        if(!stopped && stopNow && stopNow()){stopped=true;bridge::enqueueBleWrite("[OP:801:ESTOP]",14);bridge::processBleIngress();}
        wireUart();
        if(fakeNow-t0>60000){std::fprintf(stderr,"RunStart list timed out\n");std::exit(1);}
      };
      controller::loop();tickHook=nullptr;wireUart();wireUart();
    };
    runList("[TEST:1794830000010:P:3000:6000:F6000,L980,B1000]","1794830000010",
            [&](){return wheel1.getCurrentPosition()-s1>3000;});
    check(controller::operatorEmergencyLatched && bridge::operatorStopLatched && !wheel1.isRunning() &&
          wheel1.getCurrentPosition()-s1<6000 &&
          notifications().find("[TEST FAILED:1794830000010:EMERGENCY_STOP]")!=std::string::npos &&
          notifications().find("[RUN SAVED:1794830000010:0:")!=std::string::npos,
          "a RunStart list with every LiDAR code blocked runs, stops on the emergency stop and reports its saved place");
    bridge::enqueueBleWrite("[OP:802:RELEASE]",16);bridge::processBleIngress();wireUart();wireUart();
    controller::loop();wireUart();wireUart();
    check(!controller::operatorEmergencyLatched && !bridge::operatorStopLatched && controller::runSaved,
          "releasing the stop through both boards keeps the saved list");
    runList("[TEST:1794830000011:C:1794830000010]","1794830000011",nullptr);
    const bool m1Left=controller::heading.motor1IsLeft();
    check(wheel1.getCurrentPosition()-s1==6000+(m1Left?-980:980)-1000 &&
          wheel2.getCurrentPosition()-s2==6000+(m1Left?980:-980)-1000 &&
          notifications().find("[TEST DONE:1794830000011]")!=std::string::npos,
          "Continue run finishes the list exactly through both boards");
  }
  // Measured 90 degree developer turns: real BLE ingress, both UART consumers,
  // continuous bridge MPU samples and the same immediate endpoint as routes.
  for (int dir : {1,-1}) {
    initialize();controller::fullReset();controller::garbyState=controller::GarbyState::IDLE;
    controller::recordLoadCellSample(std::numeric_limits<float>::quiet_NaN());
    controller::manualStartLinkReady=false;controller::lastConnected=false;
    bridge::piReady=false;bridge::bleBridgeSignalSent=false;
    bridge::enqueueBleWrite("[RASPI READY]",13);bridge::loop();
    for(int i=0;i<4;i++)wireUart();
    bridge::pNotifyChar->notifications.clear();
    const float yaw=controller::heading.heading();
    const long turnStart1=wheel1.getCurrentPosition(), turnStart2=wheel2.getCurrentPosition();
    const std::string id=dir>0?"1794830000020":"1794830000021";
    const std::string request="[TEST:"+id+":"+(dir>0?"L":"R")+":90:2]";
    const std::string keep="[TEST KEEP:"+id+"]";
    bridge::enqueueBleWrite(request.c_str(),request.size());bridge::processBleIngress();
    wireUart();wireUart();
    const uint32_t began=fakeNow;uint32_t lastKeep=fakeNow;
    tickHook=[&]() {
      if(fakeNow-lastKeep>=200) {
        lastKeep=fakeNow;bridge::enqueueBleWrite(keep.c_str(),keep.size());bridge::processBleIngress();
      }
      wireUart();
      if(fakeNow-began>32000) {std::fprintf(stderr,"90 degree end-to-end test timed out\n");std::exit(1);}
    };
    controller::loop();tickHook=nullptr;wireUart();wireUart();
    check(notifications().find("[TEST DONE:"+id+"]")!=std::string::npos &&
          fabs(controller::heading.heading()-yaw-dir*90.0f)<1.2f &&
          fabs(0.5f*((wheel2.getCurrentPosition()-turnStart2)-(wheel1.getCurrentPosition()-turnStart1)) /
               (4900.0f/90.0f)-dir*90.0f)<1.2f &&
          fakeNow-began<30000 && !controller::motorsRunning() && !bridge::developerTestPending(),
          dir>0 ? "left 90 degree floor test crosses both boards and reports measured completion"
                : "right 90 degree floor test crosses both boards and reports measured completion");
  }
  initialize();controller::fullReset();controller::recordLoadCellSample(std::numeric_limits<float>::quiet_NaN());
  controller::manualStartLinkReady=false;controller::lastConnected=false;
  bridge::piReady=false;bridge::bleBridgeSignalSent=false;
  bridge::pNotifyChar->notifications.clear();
  bridge::enqueueBleWrite("[RASPI READY]",13);bridge::loop();
  for(int i=0;i<4;i++) wireUart();
  check(notifications().find("[SYSTEM READY]")!=std::string::npos &&
        notifications().find("[IDLE]")!=std::string::npos,
        "Pi readiness round trip confirms both controllers while scale and bin data are unavailable");
  const std::string manualStart="[START:1234567890001]";
  check(notifications().find("[DISPOSAL READY:1]")!=std::string::npos,
        "executor disposal capability crosses the real bridge without any bin threshold");
  bridge::enqueueBleWrite(manualStart.c_str(),manualStart.size());bridge::processBleIngress();
  wireUart();wireUart();
  check(controller::garbyState==controller::GarbyState::RUNNING &&
        notifications().find("[START ACK:1234567890001]")!=std::string::npos,
        "manual start travels Pi-to-bridge-to-executor and acknowledges the same request ID");
  bridge::pNotifyChar->notifications.clear();
  const auto pathAgeBeforeQuery=controller::lastPathCommandMs;
  const std::string query="[START STATUS:1234567890001]";
  bridge::enqueueBleWrite(query.c_str(),query.size());bridge::processBleIngress();
  wireUart();wireUart();
  check(controller::garbyState==controller::GarbyState::RUNNING &&
        notifications().find("[START ACK:1234567890001]")!=std::string::npos &&
        controller::lastPathCommandMs==pathAgeBeforeQuery,
        "lost start outcome is recovered through both UART boards without refreshing path permission");
  lastPath=fakeNow;const auto cycleStarted=fakeNow;
  tickHook=[&](){
    if(fakeNow-lastPath>=100){lastPath=fakeNow;path(false);}
    wireUart();
    if(controller::ultrasonicPingInFlight){controller::ultrasonicEchoReady=true;controller::ultrasonicEchoWidthUs=7000;}
    if(fakeNow-cycleStarted>400000){std::fprintf(stderr,"full cycle timed out\n");std::exit(1);}
  };
  controller::loop();wireUart();wireUart();
  check(controller::outboundComplete && !controller::routeFaultLatched && !wheel1.isRunning() && !wheel2.isRunning(),"entire calibrated outbound route completes and remains stationary");
  bridge::pNotifyChar->notifications.clear();bridge::processAndRelayMessage("[RESET:1234567890002]");wireUart();
  controller::loop();controller::loop();wireUart();wireUart();
  check(controller::garbyState==controller::GarbyState::IDLE && notifications().find("[RESET DONE:1234567890002]")!=std::string::npos,"entire return route reaches IDLE with matching completion through bridge");
  check(controller::returnComplete && notifications().find("[RETURN COMPLETE]")!=std::string::npos,
        "the actual successful return publishes point-A completion through the existing bridge");
  bridge::pNotifyChar->notifications.clear();controller::reportRobotStatus();wireUart();wireUart();
  check(notifications().find("[IDLE]\n[RETURN COMPLETE]")!=std::string::npos,
        "a reconnect status query replays the retained point-A completion through both boards");
  tickHook=nullptr;
  for(int i=0;i<3;i++){fakeNow+=1100;controller::processESPMessage("SENSOR:US=999|MQ4=10000|MQ135=10000|MQ137=10000");controller::loop();}
  check(controller::garbyState==controller::GarbyState::RUNNING,"fresh dangerous gas confirmations start the next cycle after full reset");
  bridge::pNotifyChar->notifications.clear();
  bridge::enqueueBleWrite("[OP:1001:ESTOP]",15);bridge::processBleIngress();wireUart();wireUart();
  controller::loop();wireUart();wireUart();
  check(controller::operatorEmergencyLatched && bridge::operatorStopLatched &&
        bridge::operatorExecutorStopped && controller::garbyState==controller::GarbyState::IDLE &&
        notifications().find("[OP ACK:1001:ESTOP]")!=std::string::npos,
        "operator stop crosses actual BLE ingress/UART and confirms stopped IDLE through notifications");
  bridge::processAndRelayMessage("[OP:1002:SONAR_OFF]");wireUart();wireUart();
  check(!controller::operatorSonarEnabled && controller::servo.angle==DEFAULT_VIEW &&
        notifications().find("[OP ACK:1002:SONAR_OFF]")!=std::string::npos,
        "sonar disable travels both boards and acknowledges the centered servo mode");
  bridge::processAndRelayMessage("[OP:1003:RELEASE]");wireUart();wireUart();
  check(!controller::operatorEmergencyLatched && !bridge::operatorStopLatched &&
        controller::operatorManualStartRequired && controller::shouldStop,
        "end-to-end release unlocks both latches without permitting automatic motion");
  bridge::processAndRelayMessage("[OP:1004:ESTOP]");wireUart();wireUart();controller::loop();
  bridge::processAndRelayMessage("[OP:1005:REBOOT]");wireUart();wireUart();
  check(controller::operatorRebootScheduled && bridge::operatorRebootScheduled &&
        notifications().find("[OP ACK:1005:REBOOT]")!=std::string::npos,
        "coordinated reboot commits both stopped controllers before the Pi acknowledgement");
}
