#pragma once
// Host substitutes for peripherals, time, and RTOS. Production algorithms are
// included unchanged by the tests; electrical behavior and ramps are not modeled.
#include <algorithm>
#include <cassert>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>
using std::isfinite;

inline uint32_t fakeNow = 1000;
inline std::function<void()> tickHook;
inline unsigned long millis() { return fakeNow; }
inline unsigned long micros() { return fakeNow * 1000U; }
inline void delay(unsigned long n) { fakeNow += (uint32_t)n; if (tickHook) tickHook(); }
inline void delayMicroseconds(unsigned long) {}
inline bool isDigit(char c){return std::isdigit((unsigned char)c)!=0;}
template<class A, class B> auto min(A a, B b) { using C = std::common_type_t<A,B>; return std::min(C(a), C(b)); }
template<class A, class B> auto max(A a, B b) { using C = std::common_type_t<A,B>; return std::max(C(a), C(b)); }
template<class A, class B, class C> auto constrain(A a, B b, C c) { using T = std::common_type_t<A,B,C>; return std::clamp(T(a), T(b), T(c)); }
class String {
 public:
  std::string text;
  String() = default;
  String(const char* s) : text(s ? s : "") {}
  String(std::string s) : text(std::move(s)) {}
  String(char c) : text(1,c) {}
  template<class T, std::enable_if_t<std::is_arithmetic_v<T>, int> = 0>
  String(T v) { std::ostringstream s; s << v; text = s.str(); }
  String(float v, int digits) { std::ostringstream s; s << std::fixed << std::setprecision(digits) << v; text = s.str(); }
  unsigned int length() const { return (unsigned int)text.size(); }
  const char* c_str() const { return text.c_str(); }
  char charAt(unsigned int i) const { return i < text.size() ? text[i] : 0; }
  char operator[](unsigned int i) const { return charAt(i); }
  bool startsWith(const String& s) const { return text.rfind(s.text, 0) == 0; }
  bool endsWith(const String& s) const { return text.size() >= s.text.size() && text.compare(text.size()-s.text.size(), s.text.size(), s.text) == 0; }
  int indexOf(const String& s, int start=0) const { auto p = text.find(s.text, max(start,0)); return p == std::string::npos ? -1 : (int)p; }
  String substring(int a) const { return substring(a, (int)text.size()); }
  String substring(int a, int b) const { a = constrain(a,0,(int)text.size()); b = constrain(b,a,(int)text.size()); return text.substr(a,b-a); }
  void trim() { auto a=text.find_first_not_of(" \t\r\n"); if(a==std::string::npos) text.clear(); else text=text.substr(a,text.find_last_not_of(" \t\r\n")-a+1); }
  void replace(char a, char b) { std::replace(text.begin(),text.end(),a,b); }
  void toUpperCase() { for(auto& c:text) c=(char)std::toupper((unsigned char)c); }
  void remove(unsigned int a) { if(a <= text.size()) text.erase(a); }
  void reserve(size_t n) { text.reserve(n); }
  float toFloat() const { return (float)std::atof(c_str()); }
  long toInt() const { return std::atol(c_str()); }
  String& operator+=(const String& s) { text+=s.text; return *this; }
  friend String operator+(const String& a,const String& b) { return a.text+b.text; }
  friend bool operator==(const String& a,const String& b) { return a.text==b.text; }
  friend bool operator!=(const String& a,const String& b) { return !(a==b); }
  friend std::ostream& operator<<(std::ostream& os, const String& s) { return os<<s.text; }
};
class HardwareSerial {
 public:
  std::string tx, rx;
  explicit HardwareSerial(int=0) {}
  void begin(int,int=0,int=0,int=0) {}
  void setTimeout(int) {}
  void setRxBufferSize(int) {}
  void setTxBufferSize(int) {}
  int available() const { return (int)rx.size(); }
  int read() { if(rx.empty()) return -1; char c=rx.front(); rx.erase(0,1); return (unsigned char)c; }
  void write(int c) { tx+=(char)c; }
  template<class T> void print(const T& v) { std::ostringstream s; s<<v; tx+=s.str(); }
  template<class T> void println(const T& v) { print(v); tx+='\n'; }
  void println() { tx+='\n'; }
  template<class... T> void printf(const char* fmt,T... args) { char b[2048]; std::snprintf(b,sizeof(b),fmt,args...); tx+=b; }
};
inline HardwareSerial Serial;
class Servo { public: void setPeriodHertz(int) {} void attach(int,int,int) {} void write(int) {} };
class HX711 { public: void begin(int,int) {} bool wait_ready_timeout(int) {return false;} void set_scale(float) {} void tare(int) {} long get_offset(){return 0;} bool is_ready(){return false;} float get_units(int){return 0;} };
enum MoveResultCode { MOVE_OK=0, MOVE_ERR_SPEED_IS_UNDEFINED=1 };
class FastAccelStepper {
 public:
  long position=0, target=0;
  uint32_t speed=5600, lastUpdate=1000;
  bool running=false, rejectNext=false;
  void update() { if(running) { auto distance=(long)((fakeNow-lastUpdate)*speed/1000U); if(target>=position) position=min(position+distance,target); else position=max(position-distance,target); running=position!=target; } lastUpdate=fakeNow; }
  void setDirectionPin(int,bool=true) {}
  void setSpeedInHz(uint32_t n) { update(); speed=n; }
  void setAcceleration(uint32_t) {}
  void applySpeedAcceleration() {}
  MoveResultCode moveTo(long n) { update(); if(rejectNext) {rejectNext=false;return MOVE_ERR_SPEED_IS_UNDEFINED;} target=n; running=target!=position; return MOVE_OK; }
  MoveResultCode runForward() {return moveTo(1000000000);}
  bool isRunning() {update();return running;}
  long getCurrentPosition() {update();return position;}
  void stopMove() {update();running=false;target=position;}
  void forceStopAndNewPosition(long n) {position=target=n;running=false;}
};
class FastAccelStepperEngine { public: void init(){} FastAccelStepper* stepperConnectToPin(int){return nullptr;} };
#define SERIAL_8N1 0
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define CHANGE 2
#define IRAM_ATTR
inline void setCpuFrequencyMhz(int){}
inline void pinMode(int,int){}
inline void digitalWrite(int,int){}
inline int digitalRead(int){return 0;}
inline int digitalPinToInterrupt(int p){return p;}
inline void attachInterrupt(int,void(*)(),int){}
inline unsigned long pulseIn(int,int,unsigned long){return 0;}
using portMUX_TYPE=int;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)0)
#define portEXIT_CRITICAL(x) ((void)0)
#define portENTER_CRITICAL_ISR(x) ((void)0)
#define portEXIT_CRITICAL_ISR(x) ((void)0)
#define pdMS_TO_TICKS(x) (x)
#define pdPASS 1
#define portMAX_DELAY 0
inline void vTaskDelay(unsigned long n){delay(n);}
inline void vTaskDelete(void*){}
inline int xTaskCreate(void(*)(void*),const char*,int,void*,int,void*){return pdPASS;}
struct StaticQueue_t { size_t depth=0,size=0; std::deque<std::vector<uint8_t>> items; };
using QueueHandle_t=StaticQueue_t*;
inline QueueHandle_t xQueueCreateStatic(size_t depth,size_t size,uint8_t*,StaticQueue_t* q){q->depth=depth;q->size=size;return q;}
inline int xQueueSendToBack(QueueHandle_t q,const void* p,int){if(q->items.size()>=q->depth)return 0;auto b=(const uint8_t*)p;q->items.emplace_back(b,b+q->size);return pdPASS;}
inline int xQueueReceive(QueueHandle_t q,void* p,int){if(q->items.empty())return 0;memcpy(p,q->items.front().data(),q->size);q->items.pop_front();return pdPASS;}
inline void xQueueOverwrite(QueueHandle_t q,const void* p){q->items.clear();xQueueSendToBack(q,p,0);}
inline void xQueueReset(QueueHandle_t q){q->items.clear();}
inline size_t uxQueueMessagesWaiting(QueueHandle_t q){return q->items.size();}
using StaticSemaphore_t=int;
using SemaphoreHandle_t=int*;
inline SemaphoreHandle_t xSemaphoreCreateMutexStatic(int* p){return p;}
inline void xSemaphoreTake(int*,int){}
inline void xSemaphoreGive(int*){}
using esp_err_t=int;
#define ESP_OK 0
#define ESP_ERR_NVS_NO_FREE_PAGES 1
#define ESP_ERR_NVS_NEW_VERSION_FOUND 2
inline int nvs_flash_init(){return ESP_OK;}
inline void nvs_flash_erase(){}
