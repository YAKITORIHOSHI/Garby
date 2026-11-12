#pragma once
#include "Arduino.h"
class NimBLEConnInfo {public: uint16_t getConnHandle(){return 1;} uint16_t getConnInterval(){return 24;} uint16_t getConnTimeout(){return 400;} uint16_t getConnLatency(){return 0;} uint16_t getMTU(){return 23;} };
class NimBLEServer;
class NimBLECharacteristic;
class NimBLEServerCallbacks {public: virtual void onConnect(NimBLEServer*,NimBLEConnInfo&){} virtual void onDisconnect(NimBLEServer*,NimBLEConnInfo&,int){} virtual void onConnParamsUpdate(NimBLEConnInfo&){} };
class NimBLECharacteristicCallbacks {public: virtual void onWrite(NimBLECharacteristic*,NimBLEConnInfo&){} };
class NimBLECharacteristic {
 public:
  std::string value;
  std::vector<std::string> notifications;
  size_t scheduledAttributeUpdates=0;
  size_t explicitPayloadNotifications=0;
  bool rejectNextNotification=false;
  void setValue(const uint8_t* p,size_t n){value.assign((const char*)p,n);}
  // NimBLE's no-payload notify schedules a later characteristic read; it does
  // not snapshot the value here. The explicit-payload overload copies now.
  bool notify(){scheduledAttributeUpdates++;return true;}
  bool notify(const uint8_t* p,size_t n,uint16_t){
    if(rejectNextNotification){rejectNextNotification=false;return false;}
    explicitPayloadNotifications++;notifications.emplace_back((const char*)p,n);return true;
  }
  void flushScheduledNotifications(){while(scheduledAttributeUpdates){notifications.push_back(value);scheduledAttributeUpdates--;}}
  void setCallbacks(NimBLECharacteristicCallbacks*){}
  std::string getValue(){return value;}
};
namespace NIMBLE_PROPERTY {constexpr int WRITE=1,WRITE_NR=2,READ=4,NOTIFY=8;}
class NimBLEService {public: NimBLECharacteristic* createCharacteristic(const char*,int){return new NimBLECharacteristic;} };
class NimBLEServer {public: bool autoAdvertise=false; int paramUpdates=0,disconnects=0; void updateConnParams(uint16_t,int,int,int,int){paramUpdates++;} void advertiseOnDisconnect(bool enable){autoAdvertise=enable;} void disconnect(uint16_t){disconnects++;} void setCallbacks(NimBLEServerCallbacks*){} NimBLEService* createService(const char*){return new NimBLEService;} };
class NimBLEUUID {public: explicit NimBLEUUID(const char*){} };
class NimBLEAdvertisementData {public: void setFlags(int){} void setCompleteServices(NimBLEUUID){} void setName(const char*){} };
class NimBLEAdvertising {public: bool advertising=false,scanResponse=false; void enableScanResponse(bool enabled){scanResponse=enabled;} bool isAdvertising(){return advertising;} void addServiceUUID(const char*){} void setMinInterval(int){} void setMaxInterval(int){} void setAdvertisementData(NimBLEAdvertisementData){} void setScanResponseData(NimBLEAdvertisementData){} };
class FakeAddress {public: std::string toString(){return "host";} };
#define ESP_PWR_LVL_P9 12
#define BLE_HS_ADV_F_DISC_GEN 1
#define BLE_HS_ADV_F_BREDR_UNSUP 2
class NimBLEDevice {public: static inline int powerDbm=0; static void init(const char*){} static void setMTU(int){} static void setPower(int value){powerDbm=value;} static NimBLEServer* createServer(){return new NimBLEServer;} static NimBLEAdvertising* getAdvertising(){static NimBLEAdvertising a;return &a;} static bool startAdvertising(){getAdvertising()->advertising=true;return true;} static void stopAdvertising(){getAdvertising()->advertising=false;} static FakeAddress getAddress(){return {};} static int getPower(){return powerDbm;} };
