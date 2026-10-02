#include "stubs/Arduino.h"
#include "stubs/NimBLEDevice.h"
namespace bridge {
#include "../../BLE_Receiver-Final/BLE_Receiver-Final.ino"
}
namespace controller {
#include "../../NAPHTALI_CODE_V2/NAPHTALI_CODE_V2.cpp"
#include "../../NAPHTALI_CODE_V2/pointsRun.ino"
#include "../../NAPHTALI_CODE_V2/NAPHTALI_CODE_V2.ino"
}

static FastAccelStepper wheel1, wheel2;
static uint32_t sequence=0;
static void wireUart(){
  controller::ESP_Serial.rx+=bridge::ESP_Serial.tx; bridge::ESP_Serial.tx.clear();
  controller::pollESP();
  bridge::ESP_Serial.rx+=controller::ESP_Serial.tx; controller::ESP_Serial.tx.clear();
  bridge::pollMcuUart();
}
static void path(bool blocked){
  std::string packet="P:"+std::to_string(++sequence)+(blocked?"|F=O|B=C":"|F=C|B=C");
  bridge::handlePathPacket(packet.c_str());wireUart();wireUart();
}
static void check(bool ok,const char* label){if(!ok){std::fprintf(stderr,"FAIL %s\n",label);std::exit(1);}std::printf("PASS %s\n",label);}
static void initialize(){
  tickHook=nullptr; fakeNow=10000;
  controller::stepper1=&wheel1;controller::stepper2=&wheel2;
  controller::fullReset();controller::garbyState=controller::GarbyState::RUNNING;
  bridge::resetIngressQueues();bridge::resetNavigationState();
  bridge::deviceConnected=true;bridge::mcuReady=true;
  bridge::mcuUnackedCommands=0;bridge::mcuAckPending=bridge::mcuAckFault=false;
  bridge::ESP_Serial.tx.clear();controller::ESP_Serial.tx.clear();
  for(int i=0;i<4;i++){fakeNow+=100;path(false);}
}
int main(){
  bridge::setup();initialize();
  check(!controller::shouldStop,"Pi path stream releases both bridge and controller confirmation gates");
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
  check(!bridge::mcuAckFault && bridge::mcuUnackedCommands==0,"STOP and nudge ACKs drain without a permanent communication fault");
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
}
