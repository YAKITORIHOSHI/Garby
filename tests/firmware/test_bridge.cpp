#include "stubs/Arduino.h"
#include "stubs/NimBLEDevice.h"
#include "../../BLE_Receiver-Final/BLE_Receiver-Final.ino"

// Every GO needs a valid heading (2026-11-11). The real loop polls the gyro each
// millisecond; these tests jump the clock, so re-reference before each entry point.
static bool keepMpuFresh=true;
static void settleMpu(){if(keepMpuFresh && mpuHeading.ready() && !mpuHeading.valid())mpuHeading.requestReference();}
static void path(const char* packet){settleMpu();handlePathPacket(packet);}
static void ingress(){settleMpu();processBleIngress();}
static void runLoop(){settleMpu();loop();}
static void check(bool ok,const char* label){if(!ok){std::fprintf(stderr,"FAIL %s\n",label);std::exit(1);}std::printf("PASS %s\n",label);}
static void resetFixture(){fakeNow=10000;resetNavigationState();turnActive=false;turnStateKnown=true;backingActive=false;backStateKnown=true;resetIngressQueues();if(emergencyQueue)xQueueReset(emergencyQueue);operatorStopMarker=0;operatorStopLatched=false;operatorExecutorStopped=false;operatorStopGeneration=0;pendingEmergencyId=pendingReleaseId=pendingRebootId="";operatorRebootScheduled=false;lastEmergencySendMs=0;deviceConnected=true;mcuReady=true;piReady=true;bleBridgeSignalSent=true;bleSessionChanged=false;readyNoticePending=false;mcuBackpressureStopPending=false;mcuUnackedCommands=0;mcuAckPending=mcuAckFault=false;ESP_Serial.tx.clear();}
static std::string notificationText(){pNotifyChar->flushScheduledNotifications();std::string text;for(const auto& frame:pNotifyChar->notifications)text+=frame.substr(1);return text;}
static void checkHealthRelay(){
  resetFixture();processAndRelayMessage("[HEALTH:1:UDXUD]");
  check(ESP_Serial.tx=="[HEALTH:1:UDXUD]\n" && mcuUnackedCommands==1 &&
        !pathStreamSeen && !havePathSeq && stopLatched,
        "allowlisted Pi health relays unchanged and expects the ordinary executor ACK");

  resetFixture();piReady=false;bleBridgeSignalSent=false;clearConfirmCount=1;
  havePathSeq=haveSidesSeq=pathStreamSeen=true;lastPathSeq=24;lastSidesSeq=23;
  lastPathDataReceived=fakeNow-123;const unsigned long pathTime=lastPathDataReceived;
  enqueueBleWrite("[HEALTH:1:UUUUU]",16);ingress();
  check(ESP_Serial.tx=="[HEALTH:1:UUUUU]\n" && !piReady && !bleBridgeSignalSent &&
        !readyNoticePending && mcuReady && stopLatched && clearConfirmCount==1 &&
        havePathSeq && haveSidesSeq && pathStreamSeen && lastPathSeq==24 && lastSidesSeq==23 &&
        lastPathDataReceived==pathTime && !operatorStopLatched,
        "health has no path, readiness, motion confirmation or operator latch side effects");
  resetFixture();operatorStopLatched=true;operatorExecutorStopped=true;
  enqueueBleWrite("[HEALTH:1:XXXXX]",16);ingress();
  check(ESP_Serial.tx=="[HEALTH:1:XXXXX]\n" && operatorStopLatched && operatorExecutorStopped,
        "unknown health states remain diagnostic while an operator emergency is latched");

  const char* malformed[]={"[HEALTH:0:UUUUU]","[HEALTH:2:UUUUU]","[HEALTH:1:UUUU]",
    "[HEALTH:1:UUUUUU]","[HEALTH:1:UUUCU]","[HEALTH:1:UUUuU]",
    "[HEALTH:1:UUUUU]|U=D","[HEALTH:1:UUUUU][HEALTH:1:DDDDD]",
    "[HEALTH:1:UUUUU]\nGO","[HEALTH:1:UUUUU]\rGO"," [HEALTH:1:UUUUU]",
    "[HEALTH:1:UUUUU] ","[HEALTH:1:UUUUU]\n"};
  resetFixture();
  for(const char* packet:malformed)processAndRelayMessage(packet);
  check(ESP_Serial.tx.empty(),"relay revalidates health versions, exact lengths, states and injection attempts");
  for(const char* packet:malformed)enqueueBleWrite(packet,std::strlen(packet));
  const char nulInjection[]="[HEALTH:1:UUUUU]\0GO";
  enqueueBleWrite(nulInjection,sizeof(nulInjection)-1);ingress();
  check(ESP_Serial.tx.find("[HEALTH:")==std::string::npos && ESP_Serial.tx.find("GO")==std::string::npos,
        "BLE health ingress rejects malformed and embedded-NUL packets");
  resetFixture();enqueueBleWrite("[HEALTH:1:UDXUD]",16);
  enqueueBleWrite("P:5|F=O|B=C",11);
  for(int i=0;i<64;i++)for(const char* packet:malformed)enqueueBleWrite(packet,std::strlen(packet));
  ingress();
  check(lastPathSeq==5 && pathStreamSeen && stopLatched &&
        ESP_Serial.tx.find("[HEALTH:1:UDXUD]\n")!=std::string::npos &&
        ESP_Serial.tx.find("GO")==std::string::npos,
        "malformed health flood preserves both queued safety packets and the latest valid snapshot");

  resetFixture();
  for(int i=0;i<64;i++){
    enqueueBleWrite(i%2 ? "[HEALTH:1:UUUUU]" : "[HEALTH:1:DDDDD]",16);
    enqueueBleWrite("SENSOR:US=100",13);
  }
  enqueueBleWrite("[HEALTH:1:UDXUD]",16);
  for(int i=0;i<6;i++)enqueueBleWrite("[START STATUS:10]",17);
  ingress();
  check(ESP_Serial.tx.find("[HEALTH:")==std::string::npos && uxQueueMessagesWaiting(controlQueue)==2,
        "health burst cannot consume control capacity or run ahead of queued controls");
  ingress();
  check(ESP_Serial.tx.find("SENSOR:US=100\n[HEALTH:1:UDXUD]\n")!=std::string::npos &&
        ESP_Serial.tx.find("[HEALTH:1:UUUUU]")==std::string::npos &&
        ESP_Serial.tx.find("[HEALTH:1:DDDDD]")==std::string::npos,
        "independent latest health and sensor mailboxes survive producer bursts");

  resetFixture();enqueueBleWrite("[HEALTH:1:UUUUU]",16);fakeNow+=15000;ingress();
  check(ESP_Serial.tx=="[HEALTH:1:UUUUU]\n","health at the 15-second freshness boundary can relay");
  resetFixture();enqueueBleWrite("[HEALTH:1:UUUUU]",16);fakeNow+=15001;ingress();
  check(ESP_Serial.tx.empty(),"health older than 15 seconds is discarded");
  resetFixture();enqueueBleWrite("[HEALTH:1:UUUUU]",16);connectionSession++;ingress();
  check(ESP_Serial.tx.empty(),"health from a prior BLE session is discarded");
  resetFixture();enqueueBleWrite("[HEALTH:1:UUUUU]",16);resetIngressQueues();ingress();
  check(ESP_Serial.tx.empty(),"ingress reset clears pending health snapshots");
  resetFixture();mcuReady=false;enqueueBleWrite("[HEALTH:1:UUUUU]",16);ingress();
  processAndRelayMessage("[HEALTH:1:UUUUU]");
  check(ESP_Serial.tx.empty() && !mcuReady,"health cannot reach or establish an unready executor");
  resetFixture();deviceConnected=false;processAndRelayMessage("[HEALTH:1:UUUUU]");
  check(ESP_Serial.tx.empty(),"health relay requires a live BLE session");
  resetFixture();mcuUnackedCommands=MCU_MAX_UNACKED_COMMANDS;
  enqueueBleWrite("[HEALTH:1:UUUUU]",16);ingress();
  check(ESP_Serial.tx.empty() && mcuUnackedCommands==MCU_MAX_UNACKED_COMMANDS &&
        mcuBackpressureStopPending,"health cannot bypass the ordinary UART backlog cap");

  resetFixture();path("P:1|F=C|B=C");path("P:2|F=C|B=C");
  const unsigned long healthyPathTime=lastPathDataReceived;
  ESP_Serial.tx.clear();mcuUnackedCommands=0;mcuAckPending=false;
  for(int i=0;i<8;i++){
    fakeNow+=150;enqueueBleWrite("[HEALTH:1:UUUUU]",16);
    for(const char* packet:malformed)enqueueBleWrite(packet,std::strlen(packet));
    runLoop();
    while(mcuUnackedCommands)handleMcuLine("[ESP RECEIVED]");
  }
  check(stopLatched && lastPathDataReceived==healthyPathTime && lastPathSeq==2 &&
        ESP_Serial.tx.find("STOP:STALE")!=std::string::npos &&
        ESP_Serial.tx.find("GO")==std::string::npos,
        "valid and malformed health flood cannot keep the safety path watchdog alive");

  resetFixture();enqueueBleWrite("[HEALTH:1:UUUUU]",16);enqueueBleWrite("[OP:81:ESTOP]",13);
  ingress();
  check(operatorStopLatched && ESP_Serial.tx.find("[OP:81:ESTOP]")!=std::string::npos &&
        ESP_Serial.tx.find("[HEALTH:")==std::string::npos,
        "emergency dispatch clears queued health and wins over telemetry");
  resetFixture();enqueueBleWrite("[HEALTH:1:UUUUU]",16);
  ServerCallbacks healthCallbacks;NimBLEConnInfo healthInfo;
  static_cast<NimBLEServerCallbacks*>(&healthCallbacks)->onDisconnect(pServer,healthInfo,0);runLoop();
  deviceConnected=true;bleSessionChanged=false;ESP_Serial.tx.clear();ingress();
  check(ESP_Serial.tx.empty(),"BLE disconnect clears health before reconnection");
}
// ---- 2026-11-11: continuous heading stream, wall forwarding, turn sweep, slow GO ----
static bool lineChecks(const std::string& line){
  const auto star=line.rfind('*');
  if(star==std::string::npos||line.size()!=star+3)return false;
  uint8_t x=0;for(size_t i=0;i<star;i++)x^=(uint8_t)line[i];
  char hex[3];std::snprintf(hex,sizeof(hex),"%02X",(unsigned)x);
  return line.compare(star+1,2,hex)==0;
}
static std::vector<std::string> uartLines(char kind){
  std::vector<std::string> found;std::istringstream stream(ESP_Serial.tx);std::string line;
  while(std::getline(stream,line))if(line.size()>2&&line[0]==kind&&line[1]==',')found.push_back(line);
  return found;
}
static int lastAllow(){
  const auto lines=uartLines('M');if(lines.empty())return -1;
  const std::string& line=lines.back();
  return line[line.rfind('*')-1]-'0';
}
static void mpuFixture(){
  resetFixture();Wire=TwoWire{};
  check(mpuHeading.begin(8,9),"stationary MPU fixture calibrates through the production reader");
  startHeadingReference();fakeNow+=10;mpuHeading.poll();
  ESP_Serial.tx.clear();mcuUnackedCommands=0;mcuAckPending=mcuAckFault=false;
}
static void spin(unsigned long ms,int16_t raw){
  Wire.setGyroZ(raw);
  for(unsigned long t=0;t<ms;t+=10){fakeNow+=10;mpuHeading.poll();}
  Wire.setGyroZ(0);
}
static uint32_t pathSeq=0;
static void clearPath(const char* body="F=C|B=C|T=C"){
  const std::string packet="P:"+std::to_string(++pathSeq)+"|"+body;
  fakeNow+=10;mpuHeading.poll();handlePathPacket(packet.c_str());
  mcuUnackedCommands=0;mcuAckPending=false;  // the executor ACKs every path command
}
static void sides(const std::string& body){
  clearPath();  // a steering frame is only accepted with its own path sequence
  currentFrameReceivedAtMs=fakeNow;
  handleSidesPacket(("S:"+std::to_string(pathSeq)+"|"+body).c_str());
}
static const char* hallway="L=95|R=95|F=300|FL=134|FR=134|BL=134|BR=134|CF=300|CL=95|CR=95|WL=95.0|WR=94.0|H=-1.25|G=200";

static void checkSteeringAcquisitionAge(){
  mpuFixture();pathSeq=0;clearPath();clearPath();
  sides("L=95|R=95|F=300|CF=300|CL=95|CR=95|G=700|A=1");
  check(currentSteeringAllow(fakeNow)==0,"700 ms old clearance cannot grant a 500 ms steering permission");
  sides("L=95|R=95|F=300|CF=300|CL=95|CR=95|G=400|A=2");
  check(currentSteeringAllow(fakeNow)==3 && currentSteeringAllow(fakeNow+70)==3 &&
        currentSteeringAllow(fakeNow+71)==0,
        "permission expires at acquisition age 500 ms including the 30 ms transit estimate");
  clearPath();currentFrameReceivedAtMs=fakeNow-120;ESP_Serial.tx.clear();
  handleSidesPacket(("S:"+std::to_string(pathSeq)+"|L=95|R=95|F=300|CF=300|CL=95|CR=95|WL=95|H=0|G=400|A=3").c_str());
  check(currentSteeringAllow(fakeNow)==0 && uartLines('W').size()==1 &&
        uartLines('W').back().rfind("W,3,550,",0)==0,
        "bridge mailbox time ages both clearance permission and the wall measurement");
  sides("L=95|R=95|F=300|CF=300|CL=95|CR=95|A=4");
  check(currentSteeringAllow(fakeNow)==0,"clearance without acquisition age grants no steering");
  // Unsigned subtraction retains the age when the acquisition predates boot's
  // millis value (or the clock wraps), instead of creating a fresh lifetime.
  fakeNow=20;currentFrameReceivedAtMs=fakeNow;stopLatched=false;
  evaluateSteeringFrame("L=95|R=95|F=300|CF=300|CL=95|CR=95|G=400|A=5");
  check(currentSteeringAllow(fakeNow)==3 && currentSteeringAllow(fakeNow+71)==0,
        "acquisition age expiry survives unsigned millis wrap");
}

static void checkSteeringPermission(){
  SteeringFrame f;
  f.clearanceValid=f.frontKnown=f.leftKnown=f.rightKnown=true;
  f.clearFront=f.front=300;f.clearLeft=f.clearRight=f.left=f.right=95;
  check(steeringAllowMask(f)==(STEER_ALLOW_LEFT|STEER_ALLOW_RIGHT),"a clear hallway permits steering to both sides");
  SteeringFrame g=f;g.clearLeft=40;
  check(steeringAllowMask(g)==STEER_ALLOW_RIGHT,"a nearest object 40 cm to the left vetoes only steering left");
  g=f;g.clearRight=39;
  check(steeringAllowMask(g)==STEER_ALLOW_LEFT,"a nearest object on the right vetoes only steering right");
  g=f;g.frontLeftKnown=true;g.frontLeft=35;
  check(steeringAllowMask(g)==STEER_ALLOW_RIGHT,"a close front-left diagonal vetoes steering left");
  g=f;g.backRightKnown=true;g.backRight=35;
  check(steeringAllowMask(g)==STEER_ALLOW_LEFT,"a close rear-right diagonal vetoes steering right");
  g=f;g.clearanceValid=false;
  check(steeringAllowMask(g)==0,"missing nearest-object clearance permits no steering");
  g=f;g.frontKnown=false;
  check(steeringAllowMask(g)==0,"a missing forward return permits no steering");
  g=f;g.clearFront=60;
  check(steeringAllowMask(g)==0,"an object 60 cm ahead permits no steering");
  g=f;g.left=22;
  check(steeringAllowMask(g)==0,"a wall median at the hard limit permits no steering");
  g=f;g.leftKnown=g.rightKnown=false;
  check(steeringAllowMask(g)==(STEER_ALLOW_LEFT|STEER_ALLOW_RIGHT),
        "missing wall medians remove centering evidence, not heading-hold permission");

  char line[96];
  SteeringFrame w;w.wallLeftKnown=w.wallRightKnown=w.headingKnown=true;w.wallLeft=95.0f;w.wallRight=94.0f;w.headingDeg=-1.25f;
  check(formatWallLine(line,sizeof(line),7,230,w) && std::string(line).rfind("W,7,230,7,950,940,-125*",0)==0 && lineChecks(line),
        "wall line carries acquisition, age, flags, millimetres, centidegrees and a valid checksum");
  SteeringFrame one;one.wallRightKnown=true;one.wallRight=110.0f;
  check(formatWallLine(line,sizeof(line),8,100,one) && std::string(line).rfind("W,8,100,2,0,1100,0*",0)==0,
        "a single wall is forwarded with only its own flag");
  check(!formatWallLine(line,sizeof(line),9,100,SteeringFrame{}),"a scan without any wall forwards nothing");
  check(!formatWallLine(line,sizeof(line),9,MAX_WALL_AGE_MS+1,w),"wall evidence older than the age limit is not forwarded");
}

static void checkHeadingStreamAndWalls(){
  mpuFixture();
  const uint32_t epoch=headingEpoch;
  check(epoch!=0,"the heading reference has a nonzero epoch");
  fakeNow+=20;serviceHeadingStream(fakeNow);
  auto m=uartLines('M');
  check(m.size()==1 && lineChecks(m[0]) && std::count(m[0].begin(),m[0].end(),',')==5,
        "the 50 Hz heading line has five fields and a valid checksum");
  check(lastAllow()==0 && mcuUnackedCommands==0,
        "the heading stream grants no steering while STOP is latched and expects no ACK");
  serviceHeadingStream(fakeNow+5);
  check(uartLines('M').size()==1,"heading lines are spaced by the stream interval");

  // Clear path twice, then one clean acquisition.
  pathSeq=0;clearPath();clearPath();
  check(!stopLatched && ESP_Serial.tx.find("GO\n")!=std::string::npos,"two clear path packets with a valid heading release GO");
  ESP_Serial.tx.clear();
  sides(std::string(hallway)+"|A=1");
  auto w=uartLines('W');
  check(w.size()==1 && lineChecks(w[0]) && w[0].rfind("W,1,230,7,950,940,-125*",0)==0,
        "a new acquisition forwards its wall fit once, with scan age plus transit");
  fakeNow+=20;mpuHeading.poll();serviceHeadingStream(fakeNow);
  check(lastAllow()==3,"a clean acquisition permits steering both ways in the next heading line");
  sides(std::string(hallway)+"|A=1");
  check(uartLines('W').size()==1,"a repeated acquisition forwards no second wall line");
  spin(STEERING_MAX_AGE_MS,0);sides(std::string(hallway)+"|A=1");fakeNow+=20;mpuHeading.poll();serviceHeadingStream(fakeNow);
  check(lastAllow()==0,"a repeated acquisition cannot refresh the permission's age");
  sides(std::string(hallway)+"|A=2");fakeNow+=20;mpuHeading.poll();serviceHeadingStream(fakeNow);
  check(lastAllow()==3 && uartLines('W').size()==2,"the next independent acquisition restores permission and forwards walls");
  const uint32_t invalidBefore=invalidSteeringFieldCount;
  sides("L=95|R=abc|F=300|CF=300|CL=95|CR=95|A=3");fakeNow+=20;mpuHeading.poll();serviceHeadingStream(fakeNow);
  check(lastAllow()==0 && invalidSteeringFieldCount==invalidBefore+1 && uartLines('W').size()==2,
        "a malformed field withdraws permission and forwards nothing");
  sides(std::string(hallway)+"|A=2");fakeNow+=20;mpuHeading.poll();serviceHeadingStream(fakeNow);
  check(lastAllow()==0,"an out-of-order acquisition cannot restore permission");
  sides("L=95|R=95|F=300|CF=300|CL=30|CR=95|WL=95|H=0.5|G=150|A=4");
  fakeNow+=20;mpuHeading.poll();serviceHeadingStream(fakeNow);
  check(lastAllow()==2 && uartLines('W').back().rfind("W,4,180,5,950,0,50*",0)==0,
        "a thin object on the left vetoes steering left while its wall and heading are still forwarded");
  sides("F=300|CF=300|CL=95|CR=95|G=150|A=5");fakeNow+=20;mpuHeading.poll();serviceHeadingStream(fakeNow);
  check(lastAllow()==3 && uartLines('W').size()==3,
        "open space on both sides keeps heading-hold permission and forwards no wall");
  sides("L=95|R=95|F=300|CF=300|CL=95|CR=95|WL=95|WR=95|H=0|A=6");
  check(uartLines('W').size()==3,"a wall fit without its acquisition age is not forwarded");
  relayStop("STOP");fakeNow+=20;mpuHeading.poll();serviceHeadingStream(fakeNow);
  check(lastAllow()==0,"STOP withdraws steering permission at once");

  // The heading is continuous: segment notices never re-zero it.
  mpuFixture();spin(1000,1310);
  const float before=mpuHeading.headingDeg();
  handleMcuLine("[HEADING:START:120]");handleMcuLine("[HEADING:END]");handleMcuLine("[HEADING:START]");
  fakeNow+=10;mpuHeading.poll();
  check(std::fabs(before-10.0f*MpuHeading::GYRO_SCALE)<0.2f && std::fabs(mpuHeading.headingDeg()-before)<0.15f && headingEpoch==epoch+1,
        "a straight-segment notice keeps the heading and its epoch");
  serviceHeadingStream(fakeNow+20);
  const std::string scaledFields = "," + std::to_string(std::lround(mpuHeading.headingDeg()*100.0f)) + "," +
      std::to_string(std::lround(mpuHeading.rateDps()*100.0f)) + ",";
  check(uartLines('M').back().find(scaledFields)!=std::string::npos,
        "the stream reports the same scaled heading and rate in centidegrees");

  // No valid heading: fault line instead of M, and never GO.
  mpuFixture();pathSeq=0;clearPath();clearPath();ESP_Serial.tx.clear();
  fakeNow+=150;  // the sensor stopped answering in time
  serviceHeadingStream(fakeNow);
  check(ESP_Serial.tx.find("MPU:FAULT\n")!=std::string::npos && uartLines('M').empty(),
        "a stale heading sends MPU:FAULT and no heading line");
  handlePathPacket(("P:"+std::to_string(++pathSeq)+"|F=C|B=C|T=C").c_str());
  check(stopLatched && ESP_Serial.tx.find("STOP:MPU")!=std::string::npos,
        "a clear path cannot release motion without a valid heading, straight segment or not");
  runLoop();
  const uint32_t epochBefore=headingEpoch;
  mpuHeading.beginRecovery();fakeNow+=600;  // blind for longer than the warm-reference limit
  for(int i=0;i<40;i++){fakeNow+=10;loop();}
  check(mpuHeading.valid() && headingEpoch==epochBefore+1,
        "recovery after a long outage starts a new reference under a new epoch");
}

static void checkTurnSweepAndSlow(){
  mpuFixture();pathSeq=0;clearPath();clearPath();ESP_Serial.tx.clear();
  handleMcuLine("[TURN:START]");
  check(turnActive && stopLatched,"a turn notice requires fresh clearance that includes the spin sweep");
  clearPath("F=C|B=C");
  check(stopLatched && ESP_Serial.tx.find("STOP:TURN_UNKNOWN")!=std::string::npos,
        "a path packet without sweep evidence cannot release a turn");
  ESP_Serial.tx.clear();clearPath("F=C|B=C|T=S");
  check(ESP_Serial.tx.find("STOP:TURN_UNKNOWN")!=std::string::npos,"an unknown sweep is not a clear sweep");
  ESP_Serial.tx.clear();clearPath("F=C|B=C|T=O");
  check(ESP_Serial.tx.find("STOP:TURN\n")!=std::string::npos,"an object inside the sweep stops the turn");
  ESP_Serial.tx.clear();clearPath();
  check(stopLatched && ESP_Serial.tx.find("STOP:CLEARING")!=std::string::npos,"one clear sweep is not yet confirmation");
  clearPath();
  check(!stopLatched && ESP_Serial.tx.find("GO\n")!=std::string::npos,"two clear sweeps release the turn");
  ESP_Serial.tx.clear();clearPath("F=O|B=C|T=C");
  check(ESP_Serial.tx.find("STOP\n")!=std::string::npos,"the ordinary front stop still applies during a turn");
  handleMcuLine("[TURN:END]");clearPath("F=C|B=C|T=O");clearPath("F=C|B=C|T=O");
  check(!turnActive && !stopLatched,"outside a turn the sweep does not gate straight travel");
  ESP_Serial.tx.clear();clearPath("F=W|B=C|T=C");
  check(!stopLatched && ESP_Serial.tx=="GO:SLOW\n","the LiDAR caution zone releases motion at caution speed");
  ESP_Serial.tx.clear();clearPath("F=C|B=W|T=C");
  check(stopLatched && ESP_Serial.tx.find("STOP:PROTOCOL")!=std::string::npos,"the slow code is valid for the front only");
  mpuFixture();pathSeq=0;clearPath();clearPath();ESP_Serial.tx.clear();clearPath("F=C|B=C|T=X");
  check(stopLatched && ESP_Serial.tx.find("STOP:PROTOCOL")!=std::string::npos,"an invalid sweep code fails closed");
  mpuFixture();pathSeq=0;clearPath();clearPath();handleMcuLine("[TURN:START]");
  handleMcuLine("[MCU ALIVE]");
  check(turnActive,"a recovery ping during a route keeps the turn gate");
  resetFixture();
}
// Reverse parking (2026-11-16): between [BACK:START] and [BACK:END] GO needs the
// Pi's rear-travel clearance R=C and the back group; the front group is not
// consulted; the grant is GO:BACK, which is the only GO the executor backs on.
static void checkBackingScope(){
  mpuFixture();pathSeq=0;clearPath();clearPath();ESP_Serial.tx.clear();
  handleMcuLine("[BACK:START]");
  check(backingActive && stopLatched,"a backing notice requires fresh clearance that includes the rear travel");
  clearPath("F=C|B=C|T=C");
  check(stopLatched && ESP_Serial.tx.find("STOP:BACK_UNKNOWN")!=std::string::npos,
        "a path packet without rear evidence cannot release a backing move");
  ESP_Serial.tx.clear();clearPath("F=C|B=C|T=C|R=S");
  check(ESP_Serial.tx.find("STOP:BACK_UNKNOWN")!=std::string::npos,"an unknown rear is not a clear rear");
  ESP_Serial.tx.clear();clearPath("F=C|B=C|T=C|R=O");
  check(ESP_Serial.tx.find("STOP:BACK\n")!=std::string::npos,"an object behind stops the backing move");
  ESP_Serial.tx.clear();clearPath("F=C|B=C|T=C|R=C");
  check(stopLatched && ESP_Serial.tx.find("STOP:CLEARING")!=std::string::npos,"one clear rear is not yet confirmation");
  clearPath("F=O|B=C|T=O|R=C");
  check(!stopLatched && ESP_Serial.tx.find("GO:BACK\n")!=std::string::npos &&
        ESP_Serial.tx.find("GO\n")==std::string::npos,
        "two clear rears release backing as GO:BACK; the front and the spin sweep face away");
  ESP_Serial.tx.clear();clearPath("F=W|B=C|T=C|R=C");
  check(ESP_Serial.tx=="GO:BACK\n","a front caution zone does not turn backing into a forward GO:SLOW");
  ESP_Serial.tx.clear();clearPath("F=C|B=O|T=C|R=C");
  check(stopLatched && ESP_Serial.tx.find("STOP\n")!=std::string::npos,
        "the back group (side clearance, missing back data) still holds backing");
  ESP_Serial.tx.clear();clearPath("F=S|B=S");
  check(stopLatched && ESP_Serial.tx.find("STOP:STALE")!=std::string::npos,"a stale scan holds backing");
  ESP_Serial.tx.clear();clearPath("F=C|B=C|T=C|R=X");
  check(stopLatched && ESP_Serial.tx.find("STOP:PROTOCOL")!=std::string::npos,"an invalid rear code fails closed");
  clearPath("F=C|B=C|T=C|R=C");clearPath("F=C|B=C|T=C|R=C");
  check(!stopLatched,"clear rear evidence releases backing again");
  handleMcuLine("[BACK:START]");clearPath("F=C|B=C|T=C|R=C");
  check(!stopLatched,"a status replay of the backing scope does not restart clearance confirmations");
  check(currentSteeringAllow(fakeNow)==0,"no steering permission while backing");
  handleMcuLine("[BACK:END]");ESP_Serial.tx.clear();clearPath("F=O|B=C|T=C|R=C");
  check(!backingActive && stopLatched && ESP_Serial.tx.find("STOP\n")!=std::string::npos,
        "after the backing move the front group holds forward travel again");
  ESP_Serial.tx.clear();clearPath("F=C|B=C|T=C|R=O");clearPath("F=C|B=C|T=C|R=O");
  check(!stopLatched && ESP_Serial.tx.find("GO\n")!=std::string::npos &&
        ESP_Serial.tx.find("GO:BACK")==std::string::npos,
        "outside a backing move the rear does not gate forward travel, and GO is plain");
  // A controller restart ends the scope; a recovery ping makes it unknown until replayed.
  handleMcuLine("[BACK:START]");handleMcuLine("[MCU READY]");
  check(!backingActive && backStateKnown,"a controller restart (idle) ends the backing scope");
  mpuFixture();pathSeq=0;mcuReady=false;handleMcuLine("[BACK:START]");
  handleMcuLine("[MCU ALIVE]");
  check(!backingActive && !backStateKnown,"a rebooted bridge does not know the backing mode from a recovery ping");
  ESP_Serial.tx.clear();handleMcuLine("[TURN:END]");clearPath("F=C|B=C|T=C|R=O");clearPath("F=C|B=C|T=C|R=O");
  check(stopLatched && ESP_Serial.tx.find("GO")==std::string::npos,
        "no GO before the executor replays its backing scope");
  handleMcuLine("[BACK:START]");ESP_Serial.tx.clear();clearPath("F=C|B=C|T=C|R=O");clearPath("F=C|B=C|T=C|R=O");
  check(backingActive && stopLatched && ESP_Serial.tx.find("GO")==std::string::npos,
        "the replay restores the rear gate of an interrupted backing move");
  // The operator's confirmed release ends it with the route.
  operatorStopLatched=true;operatorExecutorStopped=true;pendingReleaseId="7";
  std::strcpy(pendingReleaseAction,"RELEASE");pendingReleaseGeneration=operatorStopGeneration;
  handleMcuLine("[OP ACK:7:RELEASE]");
  check(!operatorStopLatched && !backingActive && backStateKnown,"a confirmed release ends the backing scope");
  resetFixture();
}
// ---- regression checks carried over from the tap-era suite (2026-11-11) ----
// The real loop polls the gyro every few milliseconds. advance() keeps that true,
// and models ongoing BLE traffic unless told otherwise.
static void advance(uint32_t ms,bool bleTraffic=true){
  for(uint32_t t=0;t<ms;t+=10){fakeNow+=9;if(bleTraffic)lastDataReceived=fakeNow;loop();}
}
static void checkTurnSessionRecovery(){
  ServerCallbacks callbacks;NimBLEConnInfo info;
  for(const char* sweep:{"F=C|B=C|T=O","F=C|B=C|T=S","F=C|B=C"}){
    mpuFixture();pathSeq=0;clearPath();clearPath();handleMcuLine("[TURN:START]");
    static_cast<NimBLEServerCallbacks*>(&callbacks)->onDisconnect(pServer,info,8);advance(10);
    static_cast<NimBLEServerCallbacks*>(&callbacks)->onConnect(pServer,info);advance(10);
    handleMcuLine("[MCU ALIVE]");
    ESP_Serial.tx.clear();clearPath(sweep);clearPath(sweep);
    check(turnActive && stopLatched && ESP_Serial.tx.find("GO\n")==std::string::npos,
          "BLE reconnect retains turn clearance for occupied, stale or missing sweep evidence");
    ESP_Serial.tx.clear();clearPath();clearPath();
    check(!stopLatched && ESP_Serial.tx.find("GO\n")!=std::string::npos,
          "an interrupted turn resumes only after fresh clear sweeps");
  }
  mpuFixture();pathSeq=0;mcuReady=false;turnActive=false;
  handleMcuLine("[MCU ALIVE]");
  ESP_Serial.tx.clear();clearPath();clearPath();
  check(stopLatched && ESP_Serial.tx.find("GO\n")==std::string::npos,
        "a rebooted bridge cannot issue GO before learning the live executor's turn mode");
  handleMcuLine("[TURN:START]");ESP_Serial.tx.clear();clearPath();clearPath();
  check(stopLatched && ESP_Serial.tx.find("STOP:BACK_UNKNOWN")!=std::string::npos,
        "a rebooted bridge cannot issue GO before learning the executor's backing mode too");
  handleMcuLine("[BACK:END]");ESP_Serial.tx.clear();  // reportRobotStatus replays both
  clearPath("F=C|B=C|T=O");clearPath("F=C|B=C|T=O");
  check(turnActive && stopLatched && ESP_Serial.tx.find("GO\n")==std::string::npos,
        "the executor's replay restores the occupied turn-sweep gate after bridge reboot");
  clearPath();handleMcuLine("[TURN:START]");handleMcuLine("[BACK:END]");clearPath();
  check(!stopLatched,"repeated turn status does not restart clearance confirmations");
  handleMcuLine("[TURN:END]");clearPath("F=C|B=C|T=O");
  check(!turnActive && !stopLatched,"confirmed turn end restores ordinary straight clearance");
  resetFixture();
}
static void pumpWithPath(uint32_t ms){
  for(uint32_t t=0;t<ms;t+=10){
    fakeNow+=9;lastDataReceived=fakeNow;loop();
    if(t%150==0)clearPath();
  }
}
static size_t ackedLines(const std::string& tx){
  size_t lines=0;std::istringstream stream(tx);std::string line;
  while(std::getline(stream,line))
    if(!(line.rfind("M,",0)==0||line.rfind("W,",0)==0||line=="MPU:FAULT"))lines++;
  return lines;
}
static void checkPortedRegressions(){
  // A rebooted bridge must learn of a live controller from its recovery ping.
  resetFixture();mcuReady=false;
  handleMcuLine("[MCU ALIVE]");
  check(mcuReady,"a rebooted bridge adopts a running controller from its recovery ping");
  mpuFixture();handleMcuLine("[HEADING:START:0]");handleMcuLine("[TURN:START]");
  operatorExecutorStopped=true;pendingReleaseId="42";
  const uint32_t epoch=headingEpoch;
  pNotifyChar->notifications.clear();
  handleMcuLine("[MCU ALIVE]");
  check(straightRunActive && turnActive && mpuHeading.valid() && headingEpoch==epoch &&
        notificationText().empty() && operatorExecutorStopped && pendingReleaseId=="42",
        "a recovery ping is not a reboot: notices, heading and operator state survive, nothing reaches the Pi");
  handleMcuLine("[MCU READY]");
  check(!straightRunActive && !turnActive && mpuHeading.valid() && headingEpoch==epoch,
        "a controller reboot ends its segment and turn notices but not the bridge's heading reference");

  // A path packet can land while a new reference waits for its first sample.
  mpuFixture();stopLatched=false;clearConfirmCount=0;startHeadingReference();
  ESP_Serial.tx.clear();pathSeq=900;handlePathPacket("P:901|F=C|B=C|T=C");
  check(!stopLatched && ESP_Serial.tx.find("STOP:MPU")==std::string::npos,
        "a path packet racing a new heading reference does not stop the robot");
  Wire.registers[0x3a]=0;startHeadingReference();
  for(int i=0;i<12;i++){fakeNow+=10;mpuHeading.poll();}
  handlePathPacket("P:902|F=C|B=C|T=C");
  check(stopLatched,"a new reference that never gets fresh data still holds STOP");
  Wire=TwoWire{};

  // Lost UART lines never get an ACK. The estimate must not ratchet into a wedge.
  uint32_t uartSeq=2000;
  auto uartCycle=[&](bool lineLost){
    advance(150);
    const std::string packet="P:"+std::to_string(++uartSeq)+"|F=C|B=C|T=C";handlePathPacket(packet.c_str());
    if(lineLost){ESP_Serial.tx.clear();}
    else{const size_t lines=ackedLines(ESP_Serial.tx);ESP_Serial.tx.clear();
      for(size_t i=0;i<lines;i++) handleMcuLine("[ESP RECEIVED]");}
    advance(110);
  };
  mpuFixture();bleSessionChanged=false;bleBridgeSignalSent=true;sentConnectedMsg=true;piReady=false;
  uartCycle(false);uartCycle(false);
  for(int i=0;i<8;i++){uartCycle(true);uartCycle(false);}
  for(int i=0;i<6;i++) uartCycle(false);
  advance(150);{const std::string packet="P:"+std::to_string(++uartSeq)+"|F=C|B=C|T=C";ESP_Serial.tx.clear();handlePathPacket(packet.c_str());}
  check(!stopLatched && !mcuAckFault && ESP_Serial.tx.find("GO")!=std::string::npos,
        "eight lost UART lines in one session do not wedge the bridge in STOP:MCU_LINK");
  mpuFixture();bleSessionChanged=false;bleBridgeSignalSent=true;sentConnectedMsg=true;piReady=false;
  uartCycle(false);uartCycle(false);ESP_Serial.tx.clear();
  for(int i=0;i<13;i++){advance(240);const std::string packet="P:"+std::to_string(++uartSeq)+"|F=C|B=C|T=C";handlePathPacket(packet.c_str());}
  check(mcuAckFault && stopLatched && ESP_Serial.tx.find("STOP:MCU_LINK")!=std::string::npos,
        "a controller that stops acknowledging still faults the UART link");
  check(uartLines('M').size()>100,"the heading stream keeps flowing while path commands are held");

  // The facts that separate a power reset from a radio drop must reach the Pi.
  ServerCallbacks diagCallbacks;NimBLEConnInfo diagInfo;
  mpuFixture();bleSessionChanged=false;bridgeDiagPending=bridgeMpuLinePending=false;
  static_cast<NimBLEServerCallbacks*>(&diagCallbacks)->onDisconnect(pServer,diagInfo,0x208);advance(10);
  static_cast<NimBLEServerCallbacks*>(&diagCallbacks)->onConnect(pServer,diagInfo);advance(10);
  pNotifyChar->notifications.clear();
  enqueueBleWrite("[RASPI READY]",13);advance(10);
  check(notificationText().find("[BRIDGE DIAG")==std::string::npos,
        "diagnostics wait out the connection burst: live, sending them inside it made NimBLE reject a control write");
  pNotifyChar->notifications.clear();advance(BRIDGE_DIAG_SETTLE_MS);
  {const std::string text=notificationText();
   check(text.find("[BRIDGE DIAG fw=" BRIDGE_FIRMWARE_ID " reset=")!=std::string::npos &&
         text.find("last_drop=0x208")!=std::string::npos && text.find("sup_ms=4000")!=std::string::npos &&
         text.find("[BRIDGE MPU")==std::string::npos,
         "bridge reports firmware, reset cause, last drop reason and link timing, without the MPU line in that burst");}
  pNotifyChar->notifications.clear();advance(BRIDGE_DIAG_SETTLE_MS+20);
  {const std::string text=notificationText();
   check(text.find("[BRIDGE MPU VALID bias=+0.000dps tilt=0.0deg refresh=")!=std::string::npos &&
         text.find("epoch="+std::to_string(headingEpoch))!=std::string::npos,
         "the MPU line follows on its own: bias, tilt, refresh count and heading epoch reach the Pi");}
  const auto diagLines=[](){
    const std::string text=notificationText();int lines=0;
    for(size_t at=text.find("[BRIDGE DIAG");at!=std::string::npos;at=text.find("[BRIDGE DIAG",at+1))lines++;
    return lines;};
  pNotifyChar->notifications.clear();
  static_cast<NimBLEServerCallbacks*>(&diagCallbacks)->onConnParamsUpdate(diagInfo);advance(10);
  check(diagLines()==0,"a connection parameter update does not send diagnostics straight away");
  static_cast<NimBLEServerCallbacks*>(&diagCallbacks)->onConnParamsUpdate(diagInfo);advance(10);
  advance(BRIDGE_DIAG_SETTLE_MS);advance(BRIDGE_DIAG_SETTLE_MS);
  check(diagLines()==1,"several reports in one settling window send a single diagnostics line");
  pNotifyChar->notifications.clear();
  static_cast<NimBLEServerCallbacks*>(&diagCallbacks)->onConnParamsUpdate(diagInfo);advance(10);
  static_cast<NimBLEServerCallbacks*>(&diagCallbacks)->onDisconnect(pServer,diagInfo,0x208);advance(10);
  advance(BRIDGE_DIAG_SETTLE_MS);
  check(diagLines()==0,"a report pending when the link drops is not sent into a session that is not ready");
  static_cast<NimBLEServerCallbacks*>(&diagCallbacks)->onConnect(pServer,diagInfo);advance(10);

  // On battery there is no console: losing the MPU must hold STOP, say why and
  // recover by itself. Live, this fault stopped the first straight segment every time.
  mpuFixture();bleSessionChanged=false;deviceConnected=true;piReady=true;
  bridgeDiagPending=bridgeMpuLinePending=false;
  pathSeq=0;clearPath();clearPath();
  const uint32_t faultsBefore=mpuFaultCount,epochBefore=headingEpoch;
  ESP_Serial.tx.clear();pNotifyChar->notifications.clear();
  Wire.registers[0x3a]=0;Wire.registers[0x6b]=0x40; // a supply dip leaves the MPU reset and asleep
  pumpWithPath(150);
  check(mpuFaultCount==faultsBefore+1 && !mpuHeading.ready() && stopLatched &&
        ESP_Serial.tx.find("STOP:MPU")!=std::string::npos && ESP_Serial.tx.find("MPU:FAULT\n")!=std::string::npos,
        "a lost MPU stops the robot, withdraws readiness and tells the executor at once");
  ESP_Serial.tx.clear();pumpWithPath(1000);
  check(stopLatched && !mpuHeading.ready() && ESP_Serial.tx.find("GO")==std::string::npos &&
        uartLines('M').empty(),
        "a clear path cannot restart motion while the MPU is gone, and no heading line is sent");
  pumpWithPath(BRIDGE_DIAG_SETTLE_MS+20);
  check(notificationText().find("mpu=NOT_FRESH/NOT_FRESH")!=std::string::npos,
        "the Pi log is told why the MPU stopped the robot");
  Wire.registers[0x3a]=1;ESP_Serial.tx.clear();pNotifyChar->notifications.clear();
  pumpWithPath(600);
  check(mpuHeading.ready() && mpuHeading.valid() && Wire.registers[0x6b]==0x01 &&
        std::fabs(mpuHeading.biasDps())<0.001f && headingEpoch==epochBefore+1,
        "the bridge reconfigures the MPU, keeps its measured bias and starts a new reference under a new epoch");
  check(!stopLatched && ESP_Serial.tx.find("GO")!=std::string::npos && mpuFaultCount==faultsBefore+1,
        "motion resumes after an MPU fault with no console and no reset");
  pumpWithPath(BRIDGE_DIAG_SETTLE_MS+20);
  check(notificationText().find("mpu=OK/NOT_FRESH")!=std::string::npos,
        "the Pi log shows the recovery and keeps the last fault reason");

  resetFixture();Wire=TwoWire{};Wire.registers[0x3f]=0;
  check(!mpuHeading.begin(8,9),"a boot calibration can fail");
  ESP_Serial.tx.clear();pathSeq=0;pumpWithPath(500);
  check(stopLatched && ESP_Serial.tx.find("STOP:MPU")!=std::string::npos,
        "a failed boot calibration still holds STOP");
  Wire.registers[0x3f]=0x40;ESP_Serial.tx.clear();
  pumpWithPath(MpuHeading::RECOVERY_RETRY_MS+400+MpuHeading::CALIBRATION_SAMPLES*MpuHeading::SAMPLE_INTERVAL_MS);
  check(mpuHeading.ready() && mpuHeading.valid() && !stopLatched &&
        ESP_Serial.tx.find("GO")!=std::string::npos && !uartLines('M').empty(),
        "a failed boot calibration recovers with no console and no reset, and starts the heading stream");
  Wire=TwoWire{};check(mpuHeading.begin(8,9),"MPU fixture restored");
}

static void checkSteeringFrameGates(){
  mpuFixture();pathSeq=0;clearPath();clearPath();
  const uint32_t before=invalidSteeringFieldCount;
  sides("L=95|L=96|R=95|F=300|CF=300|CL=95|CR=95|G=100|A=1");fakeNow+=20;mpuHeading.poll();serviceHeadingStream(fakeNow);
  check(lastAllow()==0 && invalidSteeringFieldCount==before+1,"a duplicated wall field withdraws permission and counts as corrupt");
  sides("L=95|R=95|F=300|CF=300|CF=200|CL=95|CR=95|G=100|A=2");fakeNow+=20;mpuHeading.poll();serviceHeadingStream(fakeNow);
  check(lastAllow()==0,"a duplicated clearance field permits no steering");
  for(const char* acquisition:{"A=abc","A=99999999999","A=3|A=4",""}){
    const uint32_t count=invalidSteeringFieldCount;
    sides(std::string("L=95|R=95|F=300|CF=300|CL=95|CR=95|WL=95|WR=95|H=0|G=100")+(*acquisition?"|":"")+acquisition);
    fakeNow+=20;mpuHeading.poll();serviceHeadingStream(fakeNow);
    check(lastAllow()==0 && invalidSteeringFieldCount==count+1 && uartLines('W').empty(),
          (std::string("an invalid or missing acquisition (")+acquisition+") fails closed and forwards nothing").c_str());
  }
  const uint32_t clean=invalidSteeringFieldCount;
  sides("F=300|CF=300|CL=95|CR=95|G=100|A=5");
  check(invalidSteeringFieldCount==clean,"omitted walls are not counted as a corrupt steering frame");
  // A steering frame belongs to its own path packet. An older one is ignored.
  ESP_Serial.tx.clear();clearPath();
  currentFrameReceivedAtMs=fakeNow;
  handleSidesPacket(("S:"+std::to_string(pathSeq-1)+"|"+std::string(hallway)+"|A=6").c_str());
  check(uartLines('W').empty(),"a steering frame older than the latest path packet is ignored");
  // A frame left in the mailbox past its age limit can neither steer nor be fused.
  clearPath();ESP_Serial.tx.clear();
  const std::string frame="S:"+std::to_string(pathSeq)+"|"+std::string(hallway)+"|A=7";
  enqueueBleWrite(frame.c_str(),frame.size());
  for(uint32_t t=0;t<=SIDES_FRAME_MAX_AGE_MS;t+=10){fakeNow+=10;mpuHeading.poll();}
  processBleIngress();
  check(uartLines('W').empty() && steeringAllow==0,"a steering frame older than its mailbox limit is dropped");
  sides(std::string(hallway)+"|A=8");spin(20,0);serviceHeadingStream(fakeNow);
  check(lastAllow()==3,"permission fixture: a clean acquisition permits steering");
  operatorStopLatched=true;spin(20,0);serviceHeadingStream(fakeNow);
  check(lastAllow()==0,"an operator emergency stop withdraws steering permission");
  operatorStopLatched=false;
  ServerCallbacks callbacks;NimBLEConnInfo info;
  static_cast<NimBLEServerCallbacks*>(&callbacks)->onDisconnect(pServer,info,8);advance(10);
  check(steeringAllow==0 && stopLatched,"a BLE disconnect withdraws steering permission");
  static_cast<NimBLEServerCallbacks*>(&callbacks)->onConnect(pServer,info);advance(10);
}

static void checkStillBiasRefresh(){
  // Drift appears after boot: 13 counts = 0.099 deg/s. Bias is re-measured only
  // while the wheels are held, never while the executor may be moving.
  // The fixture rewinds the clock between cases; real millis() never does.
  mpuFixture();pathSeq=0;stopLatchSeen=false;executorParked=false;Wire.setGyroZ(13);
  const uint32_t startRefreshes=mpuHeading.biasRefreshes();
  advance(1000+1600);
  check(mpuHeading.biasRefreshes()>startRefreshes && std::fabs(mpuHeading.biasDps()-0.0992f*MpuHeading::GYRO_SCALE)<0.005f,
        "a STOP held for over a second re-measures the gyro bias");
  handleMcuLine("[RUNNING]");
  uint32_t refreshes=mpuHeading.biasRefreshes();
  pumpWithPath(3000);
  check(!stopLatched && mpuHeading.biasRefreshes()==refreshes,
        "a robot that may be moving (GO flowing, executor RUNNING) never re-measures bias");
  handleMcuLine("[IDLE]");pumpWithPath(1000+1600);
  check(!stopLatched && mpuHeading.biasRefreshes()>refreshes,
        "an executor reporting IDLE re-measures bias even while the path is clear");
  handleMcuLine("[RETURNING]");refreshes=mpuHeading.biasRefreshes();pumpWithPath(2600);
  check(mpuHeading.biasRefreshes()==refreshes,"a return route ends bias refresh at once");
  handleMcuLine("[OUTBOUND COMPLETE]");pumpWithPath(2600);
  check(mpuHeading.biasRefreshes()>refreshes,"parked at the destination, bias refresh resumes");
  refreshes=mpuHeading.biasRefreshes();
  for(int i=0;i<15;i++)pumpWithPath(1000);  // no state heartbeat for 15 s
  const uint32_t afterExpiry=mpuHeading.biasRefreshes();
  pumpWithPath(3000);
  check(mpuHeading.biasRefreshes()==afterExpiry,"a parked state without its heartbeat expires");
  handleMcuLine("[IDLE]");handleMcuLine("[TURN:START]");refreshes=mpuHeading.biasRefreshes();advance(3000);
  check(stopLatched && mpuHeading.biasRefreshes()==refreshes,"no bias refresh inside a turn, stopped or parked");
  handleMcuLine("[TURN:END]");
  mpuFixture();Serial.tx.clear();Serial.rx="H";loop();
  check(Serial.tx.find("[MPU] VALID heading=+0.00 deg")!=std::string::npos &&
        Serial.tx.find("epoch="+std::to_string(headingEpoch))!=std::string::npos,
        "the USB console prints heading, rate, bias and epoch on H for the yaw-sign bench check");
  Wire=TwoWire{};
}
static void checkDeveloperRelay(){
  resetFixture();
  const char* request="[TEST:501:F:400:1]";
  enqueueBleWrite(request,std::strlen(request));ingress();
  check(ESP_Serial.tx==std::string(request)+"\n",
        "finite developer commands cross the production BLE control ingress and UART");
  resetFixture();operatorStopLatched=true;
  processAndRelayMessage("[TEST:502:F:400:0]");
  check(ESP_Serial.tx.empty(),"developer requests cannot cross a latched emergency stop");

  const auto command=[](const char* raw){currentFrameReceivedAtMs=fakeNow;processAndRelayMessage(raw);};
  resetFixture();
  const char* malformed[]={"[TEST:0:F:400:1]","[TEST:01:F:400:1]","[TEST:12345678901234567:F:400:1]",
    "[TEST:503:F:401:1]","[TEST:503:1:101:0]","[TEST:503:2:0:0]","[TEST:503:L:10:0]",
    "[TEST:503:R:16:2]","[TEST:503:L:89:2]","[TEST:503:R:91:2]","[TEST:503:L:90:1]",
    "[TEST:503:I:1:7]","[TEST:503:F:400:8]","[TEST:503:F:400:01]",
    "[TEST:503:F:400:1:0]","[TEST:503:F:400:1]extra","[TEST:503:FF:400:1]"};
  for(const char* raw:malformed)command(raw);
  command("[TEST:503:B:401:1]");command("[TEST:503:B:0:1]");
  check(ESP_Serial.tx.empty() && !developerTestPending(),
        "developer parser rejects oversized travel, turn-without-MPU, overflow, noncanonical and injected fields");
  resetFixture();command("[TEST:503:B:400:1]");
  check(ESP_Serial.tx=="[TEST:503:B:400:1]\n" && developerTestPending(),
        "a finite backward test (reverse parking bench check) reaches the executor");
  resetFixture();
  piReady=false;command("[TEST:504:F:400:1]");
  check(ESP_Serial.tx.empty() && !developerTestPending(),"developer requests require the full Pi/UART handshake");

  resetFixture();command("[TEST:505:F:400:1]");
  const auto sent=ESP_Serial.tx;
  handleMcuLine("[IDLE]");
  check(developerTestPending(),"a pre-test IDLE queued before ACK cannot cancel test correlation");
  command("[TEST:505:F:400:1]");command("[TEST:505:F:400:7]");
  check(ESP_Serial.tx==sent && developerTestSensors==1 && !developerMpuBypassed(),
        "repeated test writes are never replayed and cannot replace the original sensor mask");
  handleMcuLine("[TEST ACK:999]");command("[TEST KEEP:505]");
  check(!developerMpuBypassed(),"a KEEP plus an unrelated executor ACK cannot bypass MPU");
  handleMcuLine("[TEST ACK:505:EXTRA]");
  check(!developerTestAcknowledged,"a malformed ACK cannot activate developer overrides");
  handleMcuLine("[TEST ACK:505]");
  check(developerMpuBypassed(),"only the matching executor ACK and live KEEP enable selected MPU isolation");
  handleMcuLine("[TEST DONE:505:EXTRA]");handleMcuLine("[TEST FAILED:505:]");
  check(developerMpuBypassed(),"malformed terminal reports cannot retire a valid test correlation");
  const auto beforeReset=ESP_Serial.tx;
  command("[RESET]");command("[RESET:50001]");command("[START:50002]");
  check(ESP_Serial.tx==beforeReset && developerMpuBypassed(),
        "return/reset and route start cannot reach an executor with an active developer test");
  fakeNow+=801;command("[TEST KEEP:505]");
  check(!developerTestPending() && !developerMpuBypassed(),"a delayed KEEP cannot revive an expired test lease");
  handleMcuLine("[TEST ACK:505]");
  check(!developerMpuBypassed(),"an ACK arriving after lease expiry cannot revive overrides");

  resetFixture();command("[TEST:506:F:400:1]");handleMcuLine("[TEST ACK:506]");
  check(!developerMpuBypassed(),"executor acceptance without an actual Pi KEEP never relaxes MPU gating");
  enqueueBleWrite("[TEST KEEP:506]",15);fakeNow+=301;ingress();
  check(!developerTestKeepSeen && !developerMpuBypassed(),"KEEP expires at 300 ms in the production BLE ingress");
  command("[TEST KEEP:506]");
  check(developerMpuBypassed(),"a new current KEEP can renew a still-live lease");
  handleMcuLine("[TEST DONE:506]");
  check(!developerTestPending() && stopLatched && clearConfirmCount==0,
        "test completion restores MPU gating and fresh clearance confirmations");

  resetFixture();command("[TEST:507:F:400:1]");handleMcuLine("[TEST ACK:507]");
  command("[TEST KEEP:507]");connectionSession++;
  check(!developerMpuBypassed(),"a BLE session change invalidates developer permission immediately");
  handleMcuLine("[TEST ACK:507]");
  check(!developerTestPending(),"a prior-session ACK cannot establish a new developer lease");

  resetFixture();command("[TEST:508:F:400:1]");handleMcuLine("[TEST ACK:508]");command("[TEST KEEP:508]");
  latchOperatorStop();
  check(!developerMpuBypassed(),"emergency ingress vetoes developer MPU isolation before queued work drains");
  ESP_Serial.tx.clear();
  check(!sendMcuLine("[TEST KEEP:508]",true) && ESP_Serial.tx.empty(),
        "UART write boundary refuses developer KEEP after emergency stop");
  resetFixture();command("[TEST:509:F:400:1]");handleMcuLine("[TEST ACK:509]");command("[TEST KEEP:509]");
  relayStop("STOP:MCU_LINK");
  check(!developerTestPending(),"UART faults terminate developer overrides");

  resetFixture();command("[TEST:510:F:400:1]");handleMcuLine("[TEST ACK:510]");command("[TEST KEEP:510]");
  handleMcuLine("[IDLE]");
  check(!developerTestPending(),"IDLE after acceptance ends developer scope");
  resetFixture();command("[TEST:511:F:400:1]");handleMcuLine("[TEST ACK:511]");command("[TEST KEEP:511]");
  handleMcuLine("[MCU READY]");
  check(!developerTestPending(),"executor restart ends developer scope without changing the gyro reference");

  resetFixture();command("[TEST:512:F:400:1]");handleMcuLine("[TEST ACK:512]");
  const uint32_t began=fakeNow;
  for(unsigned i=0;i<39;i++){
    fakeNow+=200;command("[TEST KEEP:512]");
    while(mcuUnackedCommands)handleMcuLine("[ESP RECEIVED]");
  }
  check(developerMpuBypassed(),"fresh KEEPs maintain an accepted finite test before its hard limit");
  fakeNow=began+8000;command("[TEST KEEP:512]");
  check(!developerTestPending() && !developerMpuBypassed(),"even perfect KEEPs cannot extend a test past eight seconds");

  for (const char* side : {"L", "R"}) {
    resetFixture();
    const std::string request=std::string("[TEST:513:")+side+":90:2]";
    command(request.c_str());handleMcuLine("[TEST ACK:513]");
    check(developerTestPending() && developerTestSensors==2 && !developerMpuBypassed() &&
          ESP_Serial.tx.find(request+"\n")!=std::string::npos,
          "an explicit 90 degree turn crosses BLE/UART and cannot bypass the MPU");
    const uint32_t turnBegan=fakeNow;
    for (unsigned i=0;i<149;++i) {
      fakeNow+=200;command("[TEST KEEP:513]");
      while(mcuUnackedCommands)handleMcuLine("[ESP RECEIVED]");
    }
    check(developerTestFresh(fakeNow) && fakeNow-turnBegan>DEVELOPER_LIMIT_MS,
          "fresh KEEPs preserve a 90 degree turn beyond the small-test eight-second limit");
    fakeNow=turnBegan+30000;command("[TEST KEEP:513]");
    check(!developerTestPending() && developerTestLimitMs==DEVELOPER_LIMIT_MS,
          "even perfect KEEPs cannot extend a 90 degree turn past 30 seconds or retain its limit");
  }

  // Remote control (owner, 2026-11-16): held drives have no step value and no time
  // limit; the Pi's [TEST END] (the arrow let go) is relayed as a stop, never refused.
  resetFixture();command("[TEST:519:W:4703:0]");command("[TEST:519:W:400:1]");command("[TEST:519:X:400:0]");
  check(ESP_Serial.tx.empty() && !developerTestPending(),
        "held drives above 4702 steps/s, with sensor gates or unknown arrows are refused");
  resetFixture();command("[TEST:520:W:4702:0]");handleMcuLine("[TEST ACK:520]");
  check(ESP_Serial.tx=="[TEST:520:W:4702:0]\n","a held forward drive reaches the executor");
  const uint32_t heldBegan=fakeNow;
  for(unsigned i=0;i<120;i++){
    fakeNow+=100;command("[TEST KEEP:520]");
    while(mcuUnackedCommands)handleMcuLine("[ESP RECEIVED]");
  }
  check(fakeNow-heldBegan>=12000 && developerTestPending() && developerMpuBypassed(),
        "a held drive outlasts the eight-second test limit while its KEEPs arrive");
  ESP_Serial.tx.clear();command("[TEST END:999]");
  check(ESP_Serial.tx.empty(),"another drive's release is not relayed");
  operatorStopLatched=true;command("[TEST END:520]");operatorStopLatched=false;
  check(ESP_Serial.tx=="[TEST END:520]\n","letting go is relayed even while a stop is latched");
  handleMcuLine("[TEST DONE:520]");
  check(!developerTestPending(),"the executor's DONE ends the held drive's scope");
  resetFixture();command("[TEST:521:A:2351:0]");
  enqueueBleWrite("[TEST END:521]",14);currentFrameReceivedAtMs=fakeNow;fakeNow+=5000;ingress();
  check(ESP_Serial.tx.find("[TEST END:521]")!=std::string::npos,
        "a release that waited in the BLE queue is still delivered: it can only stop");

  // RunStart (owner, 2026-11-16): the Developer window's step list. No sensor gates, no
  // time limit; the KEEP lease and the emergency stop bound it, as for the remote.
  resetFixture();
  for(const char* raw:{"[TEST:530:P:8001:4800:F100]","[TEST:530:P:9:4800:F100]","[TEST:530:P:2351:99:F100]",
                       "[TEST:530:P:2351:20001:F100]","[TEST:530:P:2351:4800:F0]","[TEST:530:P:2351:4800:F100001]",
                       "[TEST:530:P:2351:4800:X100]","[TEST:530:P:2351:4800:F100,]","[TEST:530:P:2351:4800:F100:1]",
                       "[TEST:530:P:2351:4800]","[TEST:530:C:]","[TEST:530:U:7:1]","[TEST:530:C:07]"})command(raw);
  std::string many="[TEST:530:P:2351:4800:F1";for(int i=0;i<20;i++)many+=",F1";command((many+"]").c_str());
  check(ESP_Serial.tx.empty() && !developerTestPending(),
        "RunStart lists out of bounds, with unknown steps or more than 20 steps are refused");
  std::string longest="[TEST:1234567890123456:P:8000:20000:F100000";
  for(int i=0;i<19;i++)longest+=",C100000";longest+="]";
  resetFixture();command(longest.c_str());
  check(longest.size()<=BLE_FRAME_MAX_BYTES && ESP_Serial.tx==longest+"\n" && developerTestPending(),
        "the longest RunStart list fits one BLE frame and reaches the executor");
  resetFixture();command("[TEST:531:P:4000:9000:F40000,L980]");handleMcuLine("[TEST ACK:531]");
  check(ESP_Serial.tx=="[TEST:531:P:4000:9000:F40000,L980]\n","a RunStart list reaches the executor");
  const uint32_t listBegan=fakeNow;
  for(unsigned i=0;i<150;i++){
    fakeNow+=200;command("[TEST KEEP:531]");
    while(mcuUnackedCommands)handleMcuLine("[ESP RECEIVED]");
  }
  check(fakeNow-listBegan>=30000 && developerTestPending() && developerMpuBypassed(),
        "a RunStart list outlasts the eight-second test limit while its KEEPs arrive");
  fakeNow+=801;command("[TEST KEEP:531]");
  check(!developerTestPending(),"a RunStart list without KEEPs loses its scope like any test");
  resetFixture();command("[TEST:532:C:531]");
  check(ESP_Serial.tx=="[TEST:532:C:531]\n" && developerTestPending(),"Continue run of a saved list is relayed");
  resetFixture();command("[TEST:533:U:531]");
  check(ESP_Serial.tx=="[TEST:533:U:531]\n" && developerTestPending(),"Reverse to start of a saved list is relayed");
  resetFixture();operatorStopLatched=true;command("[TEST:534:C:531]");operatorStopLatched=false;
  check(ESP_Serial.tx.empty() && !developerTestPending(),"Continue cannot cross a latched emergency stop");
  resetFixture();
  enqueueBleWrite("[RUN CLEAR:531]",15);ingress();
  check(ESP_Serial.tx=="[RUN CLEAR:531]\n" && !developerTestPending(),
        "Discard crosses the BLE control ingress and only forgets the saved list");
  resetFixture();command("[RUN CLEAR:0531]");command("[RUN CLEAR:]");
  check(ESP_Serial.tx.empty(),"a malformed Discard is dropped");
  resetFixture();pNotifyChar->notifications.clear();handleMcuLine("[RUN SAVED:531:1:420:4000:9000:F40000,L980]");
  check(notificationText().find("[RUN SAVED:531:1:420:4000:9000:F40000,L980]")!=std::string::npos,
        "the executor's saved list reaches the Pi");

  mpuFixture();mpuHeading.beginRecovery();keepMpuFresh=false;pathSeq=0;
  handlePathPacket("P:1|F=C|B=C|T=C");
  check(ESP_Serial.tx.find("STOP:MPU")!=std::string::npos,"ordinary motion still refuses a missing MPU");
  command("[TEST:513:F:400:1]");handleMcuLine("[TEST ACK:513]");command("[TEST KEEP:513]");
  ESP_Serial.tx.clear();handlePathPacket("P:2|F=C|B=C|T=C");handlePathPacket("P:3|F=C|B=C|T=C");
  check(ESP_Serial.tx.find("GO\n")!=std::string::npos && !stopLatched,
        "LiDAR-only test can receive confirmed genuine path clearance while MPU is absent");
  handleMcuLine("[TEST DONE:513]");ESP_Serial.tx.clear();handlePathPacket("P:4|F=C|B=C|T=C");
  check(stopLatched && ESP_Serial.tx.find("STOP:MPU")!=std::string::npos,
        "the same missing MPU holds ordinary motion again immediately after the test");
  keepMpuFresh=true;

  mpuFixture();command("[TEST:514:F:400:0]");handleMcuLine("[TEST ACK:514]");handleMcuLine("[TESTING]");
  const uint32_t refreshes=mpuHeading.biasRefreshes();Wire.setGyroZ(13);
  for(unsigned i=0;i<260;i++){
    fakeNow+=10;
    if(i%20==0)command("[TEST KEEP:514]");
    while(mcuUnackedCommands)handleMcuLine("[ESP RECEIVED]");
    loop();
  }
  check(!executorParked && developerTestPending() && mpuHeading.biasRefreshes()==refreshes,
        "ignored path STOP and TESTING never permit moving gyro-bias remeasurement");
  handleMcuLine("[TEST DONE:514]");
  check(!stopLatchSeen && !executorParked,"test end restarts stationary dwell before bias refresh");

  mpuFixture();mpuHeading.beginRecovery();command("[TEST:515:F:400:1]");handleMcuLine("[TEST ACK:515]");
  for(unsigned i=0;i<50;i++){
    fakeNow+=10;
    if(i%10==0)command("[TEST KEEP:515]");
    while(mcuUnackedCommands)handleMcuLine("[ESP RECEIVED]");
    loop();
  }
  check(!mpuHeading.ready(),"MPU recovery calibration cannot measure bias while a selected test may move");
  handleMcuLine("[TEST FAILED:515:LEASE_EXPIRED]");
  check(!developerTestPending(),"a valid matching failure retires test correlation");

  pNotifyChar->flushScheduledNotifications();pNotifyChar->notifications.clear();
  resetFixture();operatorStopLatched=true;handleMcuLine("[TEST READY:1]");
  check(notificationText().empty(),"developer eligibility is suppressed by emergency stop");
  operatorStopLatched=false;piReady=false;handleMcuLine("[TEST READY:1]");
  check(notificationText().empty(),"developer eligibility is suppressed before Pi/UART readiness");
  piReady=true;handleMcuLine("[TEST READY:1]");
  check(notificationText().find("[TEST READY:1]")!=std::string::npos,
        "ready developer-capable firmware advertises its test eligibility to the Pi");
  Wire=TwoWire{};
}

static void checkRearWallTransport(){
  mpuFixture();
  clearPath();
  currentFrameReceivedAtMs=fakeNow;
  const uint32_t pathAt=lastPathDataReceived;
  const uint8_t pending=mcuUnackedCommands;
  handleSidesPacket(("S:"+std::to_string(pathSeq)+
      "|A=1|G=200|RD=160.5|RH=-3.25|F=300|CF=300|CL=95|CR=95").c_str());
  auto lines=uartLines('D');
  check(lines.size()==1 && lines[0].rfind("D,1,230,1,1605,-325*",0)==0 && lineChecks(lines[0]),
        "rear wall distance, sign and source age cross the real S parser into checksummed D");
  check(lastPathDataReceived==pathAt && mcuUnackedCommands==pending,
        "rear measurement neither refreshes path age nor expects an executor ACK");
  sides("A=1|G=0|RD=160.5|RH=-3.25");
  check(uartLines('D').size()==1,"a repeated acquisition never refreshes rear evidence");
  sides("A=2|G=100");
  lines=uartLines('D');
  check(lines.size()==2 && lines.back().rfind("D,2,130,0,0,0*",0)==0,
        "a scan without a rear wall explicitly withdraws the old fit");
  sides("A=3|G=100|RD=nan|RH=0");
  check(uartLines('D').back().rfind("D,3,130,0,0,0*",0)==0,
        "a malformed optional rear fit cannot become an alignment");
  sides("A=4|G=900|RD=160|RH=0");
  check(uartLines('D').size()==3,"over-age rear evidence is not forwarded");
}

int main(){
  setup();check(NimBLEDevice::getAdvertising()->scanResponse && pServer->autoAdvertise,"scan response and automatic advertising recovery are configured");
  check(NimBLEDevice::getPower()==9,"NimBLE power API receives dBm rather than the numeric ESP-IDF enum");
  checkDeveloperRelay();
  checkSteeringAcquisitionAge();
  checkHealthRelay();
  checkSteeringPermission();
  checkHeadingStreamAndWalls();
  checkTurnSweepAndSlow();
  checkBackingScope();
  checkRearWallTransport();
  checkTurnSessionRecovery();
  checkPortedRegressions();
  checkSteeringFrameGates();
  checkStillBiasRefresh();
  resetFixture();piReady=false;bleBridgeSignalSent=false;sendConnectedNotice();
  check(ESP_Serial.tx.empty(),"BLE connection alone does not announce complete startup to the executor");
  pNotifyChar->notifications.clear();processAndRelayMessage("[START:1234567890000]");
  check(ESP_Serial.tx.empty() && notificationText().find("[START FAILED:1234567890000:NOT_READY]")!=std::string::npos,
        "manual start is rejected before Pi readiness and executor handshake");
  resetFixture();mcuReady=false;processAndRelayMessage("[START:1234567890001]");
  check(ESP_Serial.tx.empty(),"manual start cannot reach an unready executor");
  resetFixture();mcuAckFault=true;processAndRelayMessage("[START:1234567890002]");
  check(ESP_Serial.tx.empty(),"manual start cannot cross an unhealthy UART");
  resetFixture();mcuUnackedCommands=MCU_MAX_UNACKED_COMMANDS;
  pNotifyChar->notifications.clear();processAndRelayMessage("[START:1234567890002]");
  check(ESP_Serial.tx.empty() && notificationText().find("[START FAILED:1234567890002:UART_BUSY]")!=std::string::npos,
        "UART backpressure explicitly rejects a start instead of silently discarding it");
  resetFixture();processAndRelayMessage("[START STATUS:1234567890002]");
  check(ESP_Serial.tx=="[START STATUS:1234567890002]\n",
        "start outcome query reaches the executor without becoming a START command");
  resetFixture();mcuAckFault=true;pNotifyChar->notifications.clear();
  processAndRelayMessage("[START STATUS:1234567890002]");
  check(notificationText().find("[START FAILED:")==std::string::npos,
        "an unavailable outcome query cannot falsely reject an already accepted start");
  resetFixture();mcuUnackedCommands=MCU_MAX_UNACKED_COMMANDS;pNotifyChar->notifications.clear();
  processAndRelayMessage("[START STATUS:1234567890002]");
  check(notificationText().find("[START FAILED:")==std::string::npos,
        "UART congestion during an outcome query is not a terminal start rejection");
  resetFixture();const std::string start="[START:1234567890003]";
  enqueueBleWrite(start.c_str(),start.size());ingress();
  check(ESP_Serial.tx==start+"\n" && mcuUnackedCommands==1,
        "ready start command crosses ordered BLE ingress and expects an executor ACK");
  resetFixture();enqueueBleWrite(start.c_str(),start.size());fakeNow+=PATH_FRAME_MAX_AGE_MS+1;ingress();
  check(ESP_Serial.tx.find("[START:")==std::string::npos && stopLatched,
        "expired BLE start is discarded instead of executing later");
  resetFixture();enqueueBleWrite(start.c_str(),start.size());connectionSession++;ingress();
  check(ESP_Serial.tx.empty(),"start from a prior BLE or controller epoch is discarded");
  resetFixture();deviceConnected=false;sendMcuLine(start.c_str(),true);
  check(ESP_Serial.tx.empty(),"start is rechecked for a live BLE link at UART transmission");
  resetFixture();pNotifyChar->notifications.clear();handleMcuLine("[START ACK:1234567890003]");
  check(notificationText()=="[START ACK:1234567890003]\n",
        "executor start acknowledgement returns to Pi through bounded notifications");
  resetFixture();pNotifyChar->notifications.clear();handleMcuLine("[DISPOSAL READY:1]");
  check(notificationText()=="[DISPOSAL READY:1]\n" && stopLatched && !pathStreamSeen,
        "manual disposal capability is relayed without supplying path evidence");
  for(int unavailable=0;unavailable<4;unavailable++){
    resetFixture();pNotifyChar->notifications.clear();
    if(unavailable==0)piReady=false;
    if(unavailable==1)mcuAckFault=true;
    if(unavailable==2)operatorStopLatched=true;
    if(unavailable==3)mcuBackpressureStopPending=true;
    handleMcuLine("[DISPOSAL READY:1]");
    check(notificationText().empty(),"unready or emergency-stopped bridge withholds disposal readiness");
  }
  resetFixture();
  for(const char* malformed:{"[START:0]","[START:01]","[START STATUS:01]"})
    processAndRelayMessage(malformed);
  check(ESP_Serial.tx.empty(),"bridge rejects noncanonical disposal IDs before UART relay");
  piReady=false;pNotifyChar->notifications.clear();handleMcuLine("[SYSTEM READY]");
  check(notificationText().empty(),"stale controller readiness cannot bypass Pi readiness");
  resetFixture();
  deviceConnected=false;NimBLEDevice::stopAdvertising();fakeNow+=2001;runLoop();
  check(NimBLEDevice::getAdvertising()->isAdvertising(),"inactive advertising is restarted from loop");
  resetFixture(); path("P:1|F=C|B=C");check(stopLatched,"one path packet cannot release bridge STOP");
  path("P:2|F=C|B=C");check(!stopLatched,"second fresh clear path releases bridge STOP");
  path("P:1|F=C|B=C");check(stopLatched,"out-of-order path latches STOP");
  resetFixture();path("P:4294967295|F=C|B=C");path("P:0|F=C|B=C");check(!stopLatched,"path sequence wraps safely");
  path("P:1|F=C|B=C|F=O");check(stopLatched,"duplicate path fields fail closed");
  resetFixture();const char injection[]="SENSOR:US=100\nGO\nGO";enqueueBleWrite(injection,sizeof(injection)-1);ingress();
  check(ESP_Serial.tx.find("GO")==std::string::npos && stopLatched,"telemetry cannot inject UART motion commands");
  Wire=TwoWire{};Wire.readFailure=true;
  check(!mpuHeading.begin(8,9),"missing MPU cannot initialize");
  path("P:1|F=C|B=C");path("P:2|F=C|B=C");
  check(stopLatched,"clear LiDAR cannot start motion without a calibrated MPU");
  Wire=TwoWire{};check(mpuHeading.begin(8,9),"repaired stationary MPU can calibrate");
  mpuHeading.requestReference();handleMcuLine("[HEADING:START:12]");handleMcuLine("[TURN:START]");fakeNow+=10;mpuHeading.poll();
  handleMcuLine("[MCU READY]");
  check(!straightRunActive && !turnActive && mpuHeading.valid(),
        "controller reboot ends its segment notices but keeps the bridge's continuous heading");
  resetFixture();processAndRelayMessage("[RESET:1234567890000]");
  check(ESP_Serial.tx=="[RESET:1234567890000]\n","return retry is relayed without injecting STOP");
  resetFixture();ServerCallbacks callbacks;NimBLEConnInfo info;
  static_cast<NimBLEServerCallbacks*>(&callbacks)->onDisconnect(pServer,info,8);check(ESP_Serial.tx.empty(),"disconnect callback performs no UART work");
  runLoop();check(ESP_Serial.tx.find("STOP:LINK")!=std::string::npos && lastDisconnectReason==8,"loop asserts STOP after BLE disconnect and retains reason for diagnostics");
  resetFixture();for(int i=1;i<=10;i++){std::string p="P:"+std::to_string(i)+"|F=C|B=C";enqueueBleWrite(p.c_str(),p.size());}
  ingress();check(stopLatched && uxQueueMessagesWaiting(controlQueue)==0,"overflow discards uncertain path backlog and stops");
  resetFixture();stopLatched=false;pathStreamSeen=true;lastPathDataReceived=fakeNow;fakeNow+=851;runLoop();
  check(stopLatched,"path timeout stops even with BLE connected");
  resetFixture();const int oldParams=pServer->paramUpdates;
  static_cast<NimBLEServerCallbacks*>(&callbacks)->onConnect(pServer,info);runLoop();fakeNow+=1001;runLoop();
  check(pServer->paramUpdates==oldParams,"connection parameter changes wait for Pi readiness");
  enqueueBleWrite("[RASPI READY]",13);runLoop();
  check(pServer->paramUpdates==oldParams+1,"ready Pi gets connection parameter tuning after setup grace");
  resetFixture();pNotifyChar->notifications.clear();sendNotification("[RESET DONE:1234567890001]");
  std::string reconstructed;bool bounded=true;bool first=true;
  for(const auto& frame:pNotifyChar->notifications){bounded=bounded && frame.size()<=20 && frame[0]==(first?'~':'+');reconstructed+=frame.substr(1);first=false;}
  check(bounded && reconstructed=="[RESET DONE:1234567890001]\n","long return outcome is delivered in default-MTU-safe notification chunks");
  pNotifyChar->notifications.clear();
  sendNotification("[START ACK:1234567890003]");
  check(notificationText()=="[START ACK:1234567890003]\n" && pNotifyChar->scheduledAttributeUpdates==0,
        "fragmented start outcome retains each payload while NimBLE attribute updates are deferred");
  pNotifyChar->notifications.clear();const uint32_t oldNotifyFailures=notificationFailureCount;
  pNotifyChar->rejectNextNotification=true;sendNotification("[START ACK:1234567890003]");
  check(pNotifyChar->notifications.empty() && notificationFailureCount==oldNotifyFailures+1,
        "notification queue rejection is counted without sending a dangling continuation");
  resetFixture();const int oldDrops=pServer->disconnects;
  static_cast<NimBLEServerCallbacks*>(&callbacks)->onConnect(pServer,info);runLoop();fakeNow+=11000;runLoop();
  check(pServer->disconnects==oldDrops && stopLatched,"slow initial GATT setup keeps motion stopped without premature BLE disconnect");
  fakeNow+=35000;runLoop();check(pServer->disconnects==oldDrops+1,"silent incomplete BLE setup eventually disconnects");

  resetFixture();stopLatched=false;
  const char emergency[]="[OP:100:ESTOP]";
  enqueueBleWrite(emergency,sizeof(emergency)-1);
  check(operatorStopLatched && ESP_Serial.tx.empty(),
        "BLE callback immediately latches operator stop without UART or Tk-style cross-thread work");
  sendMcuLine("GO",true);sendMcuLine("GO:SLOW",true);sendMcuLine("[RESET]",true);
  check(ESP_Serial.tx.empty(),"immediate operator latch blocks UART GO, slow GO and legacy resets");
  for(int i=0;i<12;i++)enqueueBleWrite("P:1|F=C|B=C",11);
  ingress();
  check(ESP_Serial.tx.find(emergency)!=std::string::npos && operatorStopLatched && stopLatched,
        "dedicated emergency mailbox survives control queue overflow and ingress fault clearing");
  handleMcuLine("[OP ACK:100:ESTOP]");
  check(operatorExecutorStopped,"matching executor stop acknowledgment establishes delivery evidence");
  ESP_Serial.tx.clear();path("P:2|F=C|B=C");path("P:3|F=C|B=C");
  processAndRelayMessage("[START:1234567890004]");processAndRelayMessage("[RESET:1234567890004]");
  check(stopLatched && ESP_Serial.tx.find("GO")==std::string::npos &&
        ESP_Serial.tx.find("[START:")==std::string::npos && ESP_Serial.tx.find("[RESET:")==std::string::npos,
        "fresh clear paths, manual starts and return requests cannot bypass operator stop");
  mcuUnackedCommands=0;mcuAckPending=mcuAckFault=mcuBackpressureStopPending=false;
  processAndRelayMessage("[OP:101:RELEASE]");
  check(pendingReleaseId=="101" && operatorStopLatched,"sending RELEASE alone never clears bridge latch");
  handleMcuLine("[OP ACK:999:RELEASE]");
  check(operatorStopLatched,"unmatched release acknowledgment cannot unlock motion");
  handleMcuLine("[OP ACK:101:RELEASE]");
  check(!operatorStopLatched && stopLatched && clearConfirmCount==0,
        "matching release clears operator latch but resets all ordinary motion confirmations");

  resetFixture();processAndRelayMessage("[OP:200:ESTOP]");handleMcuLine("[OP ACK:200:ESTOP]");
  processAndRelayMessage("[OP:201:RELEASE]");
  enqueueBleWrite("[OP:202:ESTOP]",14);handleMcuLine("[OP ACK:201:RELEASE]");
  check(operatorStopLatched,"new callback-side emergency invalidates an older in-flight RELEASE acknowledgment");
  ingress();handleMcuLine("[OP STATUS:1:ESTOP=0:SONAR=1]");
  check(operatorStopLatched && !operatorExecutorStopped,"older status cannot clear or confirm a pending emergency");
  handleMcuLine("[OP ACK:202:ESTOP]");
  check(operatorExecutorStopped,"latest emergency acknowledgment confirms the new stop generation");

  resetFixture();mcuReady=false;deviceConnected=false;
  processAndRelayMessage("[OP:300:ESTOP]");
  check(operatorStopLatched && ESP_Serial.tx.find("[OP:300:ESTOP]")!=std::string::npos,
        "emergency reaches UART before readiness and remains valid after BLE disconnect");
  resetNavigationState();check(operatorStopLatched,"navigation and reconnect reset never clears operator stop");
  resetFixture();mcuUnackedCommands=MCU_MAX_UNACKED_COMMANDS;
  processAndRelayMessage("[OP:400:ESTOP]");
  check(ESP_Serial.tx.find("[OP:400:ESTOP]")!=std::string::npos,"emergency bypasses ordinary UART backlog cap");
  resetFixture();processAndRelayMessage("[OP:0500:ESTOP]");processAndRelayMessage("[OP:123456789012345678901:ESTOP]");
  check(!operatorStopLatched,"noncanonical and oversized operator IDs do not become accepted commands");
  processAndRelayMessage("[OP:12345678901234567890:ESTOP]");
  check(operatorStopLatched,"full 20-digit operator ID is handled without integer overflow");

  resetFixture();pNotifyChar->notifications.clear();processAndRelayMessage("[OP:600:REBOOT]");
  check(ESP_Serial.tx.empty() && notificationText().find("STOP_NOT_CONFIRMED")!=std::string::npos,
        "reboot is rejected without acknowledged executor emergency stop");
  processAndRelayMessage("[OP:601:ESTOP]");handleMcuLine("[OP ACK:601:ESTOP]");
  mcuUnackedCommands=0;mcuAckPending=mcuAckFault=mcuBackpressureStopPending=false;
  processAndRelayMessage("[OP:602:REBOOT]");
  check(!operatorRebootScheduled,"reboot transmission alone never schedules bridge restart");
  handleMcuLine("[OP ACK:999:REBOOT]");check(!operatorRebootScheduled,"unmatched reboot ACK cannot restart bridge");
  handleMcuLine("[OP ACK:602:REBOOT]");
  const int restartBefore=ESP.restartCount;
  check(operatorRebootScheduled && notificationText().find("[OP ACK:602:REBOOT]")!=std::string::npos,
        "matching executor commit is reported before delayed bridge reboot");
  fakeNow=operatorRebootAtMs-1;runLoop();check(ESP.restartCount==restartBefore,"bridge leaves notification drain time before reboot");
  fakeNow=operatorRebootAtMs;runLoop();
  check(ESP.restartCount==restartBefore+1 && operatorStopLatched,"bridge restart is delayed and preserves RTC operator stop");
  operatorStopLatched=false;restoreOperatorStop();
  check(operatorStopLatched,"no-init RTC marker restores the latch after ordinary RAM reinitialization");
  operatorStopMarker=0;restoreOperatorStop();
  check(!operatorStopLatched,"cleared RTC marker does not reinstate a released emergency");
  resetFixture();operatorStopLatched=true;handleMcuLine("[OP STATUS:1:ESTOP=1:SONAR=1]");
  check(operatorExecutorStopped,"post-reboot executor status recovers confirmed stop evidence without an old request ID");
  resetFixture();processAndRelayMessage("[OP:700:ESTOP]");handleMcuLine("[OP ACK:700:ESTOP]");
  processAndRelayMessage("[OP:701:REBOOT]");
  fakeNow+=5001;handleMcuLine("[OP ACK:701:REBOOT]");
  check(!operatorRebootScheduled,"late UART ACK cannot commit reboot before the ingress timeout runs");
  ingress();
  check(!pendingRebootId.length() && operatorStopLatched && !operatorRebootScheduled,
        "missing reboot outcome expires correlation while keeping emergency stop");
  handleMcuLine("[OP ACK:701:REBOOT]");
  check(!operatorRebootScheduled,"expired reboot ACK cannot restart the bridge");
  processAndRelayMessage("[OP:702:RELEASE]");
  check(pendingReleaseId=="702","expired reboot permits a new deliberate recovery action");

  // Continue run (2026-11-14): the same confirmed-stop gate and correlated ACK as
  // RELEASE; the executor then runs the interrupted route behind a fresh STOP.
  resetFixture();pNotifyChar->notifications.clear();processAndRelayMessage("[OP:800:CONTINUE]");
  check(ESP_Serial.tx.find("[OP:800:CONTINUE]")==std::string::npos &&
        notificationText().find("[OP FAILED:800:STOP_NOT_CONFIRMED]")!=std::string::npos,
        "continue is rejected without an acknowledged executor emergency stop");
  processAndRelayMessage("[OP:801:ESTOP]");handleMcuLine("[OP ACK:801:ESTOP]");
  mcuUnackedCommands=0;mcuAckPending=mcuAckFault=mcuBackpressureStopPending=false;
  ESP_Serial.tx.clear();processAndRelayMessage("[OP:802:CONTINUE]");
  check(ESP_Serial.tx.find("[OP:802:CONTINUE]")!=std::string::npos && pendingReleaseId=="802" &&
        operatorStopLatched,"sending CONTINUE relays it to the executor but never clears the bridge latch");
  handleMcuLine("[OP ACK:802:RELEASE]");
  check(operatorStopLatched,"an acknowledgement for a different action cannot complete a pending continue");
  handleMcuLine("[OP ACK:802:CONTINUE]");
  check(!operatorStopLatched && stopLatched && clearConfirmCount==0 && pendingReleaseId.length()==0,
        "matching continue clears the operator latch but keeps STOP until fresh path confirmations");
  resetFixture();processAndRelayMessage("[OP:900:ESTOP]");handleMcuLine("[OP ACK:900:ESTOP]");
  mcuUnackedCommands=0;mcuAckPending=mcuAckFault=mcuBackpressureStopPending=false;
  processAndRelayMessage("[OP:901:CONTINUE]");
  enqueueBleWrite("[OP:902:ESTOP]",14);handleMcuLine("[OP ACK:901:CONTINUE]");
  check(operatorStopLatched,"a newer emergency stop invalidates an in-flight continue acknowledgement");
  handleMcuLine("[OP STATUS:1:ESTOP=1:SONAR=1:RESUME=1]");
  check(operatorStopLatched,"the extended executor status still restores/confirms stop evidence");

  // Reverse to start (2026-11-14): the same confirmed-stop gate and correlated ACK
  // as RELEASE and CONTINUE; the executor then drives back behind a fresh STOP.
  resetFixture();pNotifyChar->notifications.clear();processAndRelayMessage("[OP:950:REVERSE]");
  check(ESP_Serial.tx.find("[OP:950:REVERSE]")==std::string::npos &&
        notificationText().find("[OP FAILED:950:STOP_NOT_CONFIRMED]")!=std::string::npos,
        "reverse is rejected without an acknowledged executor emergency stop");
  processAndRelayMessage("[OP:951:ESTOP]");handleMcuLine("[OP ACK:951:ESTOP]");
  mcuUnackedCommands=0;mcuAckPending=mcuAckFault=mcuBackpressureStopPending=false;
  ESP_Serial.tx.clear();processAndRelayMessage("[OP:952:REVERSE]");
  check(ESP_Serial.tx.find("[OP:952:REVERSE]")!=std::string::npos && pendingReleaseId=="952" &&
        operatorStopLatched,"sending REVERSE relays it to the executor but never clears the bridge latch");
  handleMcuLine("[OP ACK:952:CONTINUE]");
  check(operatorStopLatched,"an acknowledgement for a different action cannot complete a pending reverse");
  handleMcuLine("[OP FAILED:952:NO_ROUTE]");
  check(operatorStopLatched && pendingReleaseId.length()==0,
        "a refused reverse leaves the operator stop latched");
  processAndRelayMessage("[OP:953:REVERSE]");handleMcuLine("[OP ACK:953:REVERSE]");
  check(!operatorStopLatched && stopLatched && clearConfirmCount==0 && pendingReleaseId.length()==0,
        "matching reverse clears the operator latch but keeps STOP until fresh path confirmations");
  executorParked=true;handleMcuLine("[REVERSING]");
  check(!executorParked,"the executor's REVERSING state ends the parked gyro-bias window like any route");
  resetFixture();processAndRelayMessage("[OP:960:ESTOP]");handleMcuLine("[OP ACK:960:ESTOP]");
  mcuUnackedCommands=0;mcuAckPending=mcuAckFault=mcuBackpressureStopPending=false;
  processAndRelayMessage("[OP:961:REVERSE]");
  enqueueBleWrite("[OP:962:ESTOP]",14);handleMcuLine("[OP ACK:961:REVERSE]");
  check(operatorStopLatched,"a newer emergency stop invalidates an in-flight reverse acknowledgement");
  handleMcuLine("[OP STATUS:1:ESTOP=1:SONAR=1:RESUME=1:REV=1]");
  check(operatorStopLatched,"the status with the reverse field still restores/confirms stop evidence");
  pNotifyChar->notifications.clear();handleMcuLine("[LEG:OF:1/15:S:5]");handleMcuLine("[LEG:0]");
  check(notificationText().find("[LEG:OF:1/15:S:5]")!=std::string::npos &&
        notificationText().find("[LEG:0]")!=std::string::npos,
        "route leg notices are relayed to the Pi unchanged");
}
