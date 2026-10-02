#pragma once
#include "Arduino.h"
class NimBLEConnInfo {public: uint16_t getConnHandle(){return 1;} };
class NimBLEServer;
class NimBLECharacteristic;
class NimBLEServerCallbacks {public: virtual void onConnect(NimBLEServer*,NimBLEConnInfo&){} virtual void onDisconnect(NimBLEServer*,NimBLEConnInfo&,int){} virtual void onConnParamsUpdate(NimBLEConnInfo&){} };
class NimBLECharacteristicCallbacks {public: virtual void onWrite(NimBLECharacteristic*,NimBLEConnInfo&){} };
class NimBLECharacteristic {public: std::string value; std::vector<std::string> notifications; void setValue(const uint8_t* p,size_t n){value.assign((const char*)p,n);} void notify(){notifications.push_back(value);} void setCallbacks(NimBLECharacteristicCallbacks*){} std::string getValue(){return value;} };
namespace NIMBLE_PROPERTY {constexpr int WRITE=1,WRITE_NR=2,READ=4,NOTIFY=8;}
class NimBLEService {public: NimBLECharacteristic* createCharacteristic(const char*,int){return new NimBLECharacteristic;} };
class NimBLEServer {public: void updateConnParams(uint16_t,int,int,int,int){} void disconnect(uint16_t){} void setCallbacks(NimBLEServerCallbacks*){} NimBLEService* createService(const char*){return new NimBLEService;} };
class NimBLEUUID {public: explicit NimBLEUUID(const char*){} };
class NimBLEAdvertisementData {public: void setFlags(int){} void setCompleteServices(NimBLEUUID){} void setName(const char*){} };
class NimBLEAdvertising {public: void addServiceUUID(const char*){} void setMinInterval(int){} void setMaxInterval(int){} void setAdvertisementData(NimBLEAdvertisementData){} void setScanResponseData(NimBLEAdvertisementData){} };
class FakeAddress {public: std::string toString(){return "host";} };
#define ESP_PWR_LVL_P9 9
#define BLE_HS_ADV_F_DISC_GEN 1
#define BLE_HS_ADV_F_BREDR_UNSUP 2
class NimBLEDevice {public: static void init(const char*){} static void setMTU(int){} static void setPower(int){} static NimBLEServer* createServer(){return new NimBLEServer;} static NimBLEAdvertising* getAdvertising(){static NimBLEAdvertising a;return &a;} static void startAdvertising(){} static void stopAdvertising(){} static FakeAddress getAddress(){return {};} static int getPower(){return 9;} };
