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
// 50 Hz heading stream runs, as in the bridge's loop.
static long physics1=0,physics2=0;
static uint32_t physicsMs=0;
static float injectedRateDps=0;
static void physics(){
  if(fakeNow-physicsMs<10)return;
  const float dt=(fakeNow-physicsMs)*0.001f;
  const long now1=wheel1.getCurrentPosition(),now2=wheel2.getCurrentPosition();
  const float rate=0.5f*(float)((now2-physics2)-(now1-physics1))/(4900.0f/90.0f)/dt+injectedRateDps;
  physics1=now1;physics2=now2;physicsMs=fakeNow;
  Wire.setGyroZ((int16_t)constrain(rate*131.0f,-32000.0f,32000.0f));
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
static void path(bool blocked){
  std::string packet="P:"+std::to_string(++sequence)+(blocked?"|F=O|B=C|T=C":"|F=C|B=C|T=C");
  bridge::handlePathPacket(packet.c_str());wireUart();wireUart();
}
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
