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
static void resetFixture(){fakeNow=10000;resetNavigationState();resetIngressQueues();if(emergencyQueue)xQueueReset(emergencyQueue);operatorStopMarker=0;operatorStopLatched=false;operatorExecutorStopped=false;operatorStopGeneration=0;pendingEmergencyId=pendingReleaseId=pendingRebootId="";operatorRebootScheduled=false;lastEmergencySendMs=0;deviceConnected=true;mcuReady=true;piReady=true;bleBridgeSignalSent=true;bleSessionChanged=readyNoticePending=false;mcuBackpressureStopPending=false;mcuUnackedCommands=0;mcuAckPending=mcuAckFault=false;ESP_Serial.tx.clear();}
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
  check(std::fabs(before-10.0f)<0.2f && std::fabs(mpuHeading.headingDeg()-before)<0.1f && headingEpoch==epoch+1,
        "a straight-segment notice keeps the heading and its epoch");
  serviceHeadingStream(fakeNow+20);
  check(uartLines('M').back().find(",1000,")!=std::string::npos || uartLines('M').back().find(",999,")!=std::string::npos
        || uartLines('M').back().find(",1001,")!=std::string::npos,"the stream reports the integrated heading in centidegrees");

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
// ---- regression checks carried over from the tap-era suite (2026-11-11) ----
// The real loop polls the gyro every few milliseconds. advance() keeps that true,
// and models ongoing BLE traffic unless told otherwise.
static void advance(uint32_t ms,bool bleTraffic=true){
  for(uint32_t t=0;t<ms;t+=10){fakeNow+=9;if(bleTraffic)lastDataReceived=fakeNow;loop();}
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
  mpuFixture();bleSessionChanged=false;bleBridgeSignalSent=sentConnectedMsg=true;piReady=false;
  uartCycle(false);uartCycle(false);
  for(int i=0;i<8;i++){uartCycle(true);uartCycle(false);}
  for(int i=0;i<6;i++) uartCycle(false);
  advance(150);{const std::string packet="P:"+std::to_string(++uartSeq)+"|F=C|B=C|T=C";ESP_Serial.tx.clear();handlePathPacket(packet.c_str());}
  check(!stopLatched && !mcuAckFault && ESP_Serial.tx.find("GO")!=std::string::npos,
        "eight lost UART lines in one session do not wedge the bridge in STOP:MCU_LINK");
  mpuFixture();bleSessionChanged=false;bleBridgeSignalSent=sentConnectedMsg=true;piReady=false;
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
  check(mpuHeading.biasRefreshes()>startRefreshes && std::fabs(mpuHeading.biasDps()-0.0992f)<0.005f,
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
int main(){
  setup();check(NimBLEDevice::getAdvertising()->scanResponse && pServer->autoAdvertise,"scan response and automatic advertising recovery are configured");
  check(NimBLEDevice::getPower()==9,"NimBLE power API receives dBm rather than the numeric ESP-IDF enum");
  checkHealthRelay();
  checkSteeringPermission();
  checkHeadingStreamAndWalls();
  checkTurnSweepAndSlow();
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
}
