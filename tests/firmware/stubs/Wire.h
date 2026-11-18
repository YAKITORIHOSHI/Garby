#pragma once
#include "Arduino.h"
#include <array>

// Deterministic register-level MPU bus. Tests exercise the production reader,
// including probe failures, partial reads, byte order and signed gyro samples.
class TwoWire {
 public:
  bool busReady = true, readFailure = false, writeFailure = false;
  uint8_t deviceAddress = 0x68, selectedAddress = 0, selectedRegister = 0;
  size_t shortRead = 0;
  int sda = -1, scl = -1;
  uint32_t clockHz = 0, timeoutMs = 0;
  std::array<uint8_t, 256> registers{};
  std::vector<uint8_t> outgoing;
  std::deque<uint8_t> incoming;
  TwoWire() { registers[0x75] = 0x68; registers[0x3f] = 0x40; registers[0x3a] = 1; }
  bool begin(int data, int clock, uint32_t hz) {
    sda = data; scl = clock; clockHz = hz; return busReady;
  }
  bool end() { return true; }
  void setTimeOut(uint32_t ms) { timeoutMs = ms; }
  void beginTransmission(uint8_t address) { selectedAddress = address; outgoing.clear(); }
  size_t write(uint8_t value) { outgoing.push_back(value); return 1; }
  uint8_t endTransmission(bool = true) {
    if (selectedAddress != deviceAddress || writeFailure || outgoing.empty()) return 2;
    selectedRegister = outgoing[0];
    for (size_t i = 1; i < outgoing.size(); ++i) registers[selectedRegister + i - 1] = outgoing[i];
    return 0;
  }
  size_t requestFrom(uint8_t address, size_t count) {
    incoming.clear();
    if (address != deviceAddress || readFailure) return 0;
    const size_t availableCount = shortRead ? std::min(count, shortRead) : count;
    for (size_t i = 0; i < availableCount; ++i) incoming.push_back(registers[(uint8_t)(selectedRegister + i)]);
    return availableCount;
  }
  int available() const { return (int)incoming.size(); }
  int read() { if (incoming.empty()) return -1; int value = incoming.front(); incoming.pop_front(); return value; }
  void setGyroZ(int16_t raw) { registers[0x47] = (uint16_t)raw >> 8; registers[0x48] = (uint16_t)raw & 0xff; }
};
inline TwoWire Wire;
