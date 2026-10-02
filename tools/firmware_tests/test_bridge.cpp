#include "stubs/Arduino.h"
#include "stubs/NimBLEDevice.h"
#include "../../BLE_Receiver-Final/BLE_Receiver-Final.ino"

static void check(bool ok,const char* label){if(!ok){std::fprintf(stderr,"FAIL %s\n",label);std::exit(1);}std::printf("PASS %s\n",label);}
static void resetFixture(){fakeNow=10000;resetNavigationState();resetIngressQueues();deviceConnected=true;mcuReady=true;mcuUnackedCommands=0;mcuAckPending=mcuAckFault=false;ESP_Serial.tx.clear();}
int main(){
  setup(); resetFixture(); handlePathPacket("P:1|F=C|B=C");check(stopLatched,"one path packet cannot release bridge STOP");
  handlePathPacket("P:2|F=C|B=C");check(!stopLatched,"second fresh clear path releases bridge STOP");
  handlePathPacket("P:1|F=C|B=C");check(stopLatched,"out-of-order path latches STOP");
  resetFixture();handlePathPacket("P:4294967295|F=C|B=C");handlePathPacket("P:0|F=C|B=C");check(!stopLatched,"path sequence wraps safely");
  handlePathPacket("P:1|F=C|B=C|F=O");check(stopLatched,"duplicate path fields fail closed");
  resetFixture();const char injection[]="SENSOR:US=100\nGO\nGO";enqueueBleWrite(injection,sizeof(injection)-1);processBleIngress();
  check(ESP_Serial.tx.find("GO")==std::string::npos && stopLatched,"telemetry cannot inject UART motion commands");
  resetFixture();stopLatched=false;gracePassed=rampPassed=true;
  nudgeConfirmDir=-1;nudgeConfirmCnt=2;smoothedError=40;emaInitialized=true;corridorBaselineWidth=100;
  unsigned long ms;unsigned int cut;String dir;
  computeNudgeCommand("L=30|R=70|F=200|T=0",ms,cut,dir);
  check(dir=="NUDGE_RIGHT" && ms>0,"side threat overrides old EMA direction in transmitted nudge");
  ESP_Serial.tx.clear();sendComputedNudge("L=30|R=70|F=200|T=0");
  check(ESP_Serial.tx.empty(),"cooldown sends no premature STABLE cancellation");
  resetFixture();processAndRelayMessage("[RESET:1234567890000]");
  check(ESP_Serial.tx=="[RESET:1234567890000]\n","return retry is relayed without injecting STOP");
  resetFixture();ServerCallbacks callbacks;NimBLEConnInfo info;
  static_cast<NimBLEServerCallbacks*>(&callbacks)->onDisconnect(pServer,info,8);check(ESP_Serial.tx.empty(),"disconnect callback performs no UART work");
  loop();check(ESP_Serial.tx.find("STOP:LINK")!=std::string::npos && lastDisconnectReason==8,"loop asserts STOP after BLE disconnect and retains reason for diagnostics");
  resetFixture();for(int i=1;i<=10;i++){std::string p="P:"+std::to_string(i)+"|F=C|B=C";enqueueBleWrite(p.c_str(),p.size());}
  processBleIngress();check(stopLatched && uxQueueMessagesWaiting(controlQueue)==0,"overflow discards uncertain path backlog and stops");
  resetFixture();stopLatched=false;pathStreamSeen=true;lastPathDataReceived=fakeNow;fakeNow+=851;loop();
  check(stopLatched,"path timeout stops even with BLE connected");
}
