#pragma once
#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

// ============================================================
// BRIDGE STEERING TUNABLES (2026-11-11 navigation review)
// ============================================================
// The bridge no longer decides centering and never sends steering taps. Per LiDAR
// acquisition it:
//   1. validates the Pi's S frame and derives a per-side steering permission from
//      NEAREST-object clearance (CF/CL/CR and the sector minima), and
//   2. forwards the Pi's wall line fit (WL/WR/H) with its age to the executor as
//      one W line. The executor fuses it with the gyro (HeadingControl.h).
//   3. forwards the unmasked rear fit RD/RH as a D line for parking only,
//      including an explicit unavailable fit on acquisitions without that wall.
// Its 50 Hz M line carries calibrated yaw and the permission bits. Neither line
// is ACKed, refreshes path freshness, or can release a STOP.
// All distances are centimetres from the Pi.

static constexpr bool     ENABLE_LIDAR_CENTERING    = true;   // false = never forward W lines (gyro heading hold only)
static constexpr bool     YAW_POSITIVE_IS_LEFT      = true;   // +Z up: positive gyro Z turns left. false reverses the gyro

static constexpr uint32_t HEADING_STREAM_INTERVAL_MS = 20;    // M lines, 50 Hz
static constexpr uint32_t MPU_FAULT_REPEAT_MS       = 200;    // MPU:FAULT while no valid heading
static constexpr uint32_t STEERING_MAX_AGE_MS       = 500;    // acquisition age, including Pi/bridge queues and transit estimate
static constexpr uint32_t BLE_TRANSIT_ESTIMATE_MS   = 30;     // Pi write to bridge ingress, added to the scan age
static constexpr uint32_t MAX_WALL_AGE_MS           = 900;    // older wall evidence is not forwarded

// Nearest-object vetoes, unchanged from the tap design they replace.
static constexpr float    FRONT_STEER_SUPPRESS_CM   = 60.0f;  // anything this close ahead: no steering at all
static constexpr float    SIDE_STEER_AVOID_CM       = 40.0f;  // no steering toward a side this close
static constexpr float    SIDE_STEER_HARD_STOP_CM   = 22.0f;  // a wall median this close: no steering at all

static constexpr uint8_t  STEER_ALLOW_LEFT  = 0x01;
static constexpr uint8_t  STEER_ALLOW_RIGHT = 0x02;

// One S frame, already parsed. "Known" means present and a positive finite number.
struct SteeringFrame {
  bool clearanceValid = false;   // CF, CL, CR all present and positive
  float clearFront = 0, clearLeft = 0, clearRight = 0;
  bool frontKnown = false;       // F (front sector median)
  float front = 0;
  bool leftKnown = false, rightKnown = false;
  float left = 0, right = 0;     // L/R sector medians
  bool frontLeftKnown = false, frontRightKnown = false, backLeftKnown = false, backRightKnown = false;
  float frontLeft = 0, frontRight = 0, backLeft = 0, backRight = 0;
  bool wallLeftKnown = false, wallRightKnown = false, headingKnown = false;
  float wallLeft = 0, wallRight = 0, headingDeg = 0;   // WL, WR, H
  bool rearWallKnown = false;
  float rearDistance = 0, rearHeading = 0;            // RD, RH; parking only
  bool ageKnown = false;
  uint32_t ageMs = 0;            // G
};

// Bit 0 permits steering LEFT, bit 1 RIGHT. A veto never adds a turn the other way:
// the executor simply cannot slow the wheel that would steer toward that side.
inline uint8_t steeringAllowMask(const SteeringFrame& f) {
  if (!f.clearanceValid || !f.frontKnown) return 0;
  if (f.front <= FRONT_STEER_SUPPRESS_CM || f.clearFront <= FRONT_STEER_SUPPRESS_CM) return 0;
  if ((f.leftKnown && f.left <= SIDE_STEER_HARD_STOP_CM) ||
      (f.rightKnown && f.right <= SIDE_STEER_HARD_STOP_CM)) return 0;
  const bool leftUnsafe = (f.leftKnown && f.left <= SIDE_STEER_AVOID_CM) ||
      f.clearLeft <= SIDE_STEER_AVOID_CM ||
      (f.frontLeftKnown && f.frontLeft <= SIDE_STEER_AVOID_CM) ||
      (f.backLeftKnown && f.backLeft <= SIDE_STEER_AVOID_CM);
  const bool rightUnsafe = (f.rightKnown && f.right <= SIDE_STEER_AVOID_CM) ||
      f.clearRight <= SIDE_STEER_AVOID_CM ||
      (f.frontRightKnown && f.frontRight <= SIDE_STEER_AVOID_CM) ||
      (f.backRightKnown && f.backRight <= SIDE_STEER_AVOID_CM);
  return (leftUnsafe ? 0 : STEER_ALLOW_LEFT) | (rightUnsafe ? 0 : STEER_ALLOW_RIGHT);
}

inline uint8_t lineChecksum(const char* text) {
  uint8_t check = 0;
  for (const char* p = text; *p; ++p) check ^= (uint8_t)*p;
  return check;
}

// W,<acquisition>,<age ms>,<flags>,<WL mm>,<WR mm>,<H cdeg>*<xor hex>
// flags: bit0 WL valid, bit1 WR valid, bit2 H valid. Returns false if there is
// nothing worth forwarding or the line would not fit.
inline bool formatWallLine(char* out, size_t size, uint32_t acquisition, uint32_t ageMs,
                           const SteeringFrame& f) {
  if (out == nullptr || size < 16 || ageMs > MAX_WALL_AGE_MS) return false;
  const bool left = f.wallLeftKnown && f.wallLeft <= 400.0f;
  const bool right = f.wallRightKnown && f.wallRight <= 400.0f;
  const bool heading = f.headingKnown && fabsf(f.headingDeg) <= 45.0f;
  if (!left && !right && !heading) return false;
  const unsigned flags = (left ? 1U : 0U) | (right ? 2U : 0U) | (heading ? 4U : 0U);
  char payload[80];
  const int used = snprintf(payload, sizeof(payload), "W,%lu,%lu,%u,%ld,%ld,%ld",
      (unsigned long)acquisition, (unsigned long)ageMs, flags,
      left ? lroundf(f.wallLeft * 10.0f) : 0L, right ? lroundf(f.wallRight * 10.0f) : 0L,
      heading ? lroundf(f.headingDeg * 100.0f) : 0L);
  if (used <= 0 || (size_t)used >= sizeof(payload)) return false;
  const int total = snprintf(out, size, "%s*%02X", payload, (unsigned)lineChecksum(payload));
  return total > 0 && (size_t)total < size;
}

// D,<acquisition>,<age ms>,<valid 0|1>,<rear distance mm>,<rear H cdeg>*<xor>
// Missing fits are forwarded too: the latest acquisition explicitly withdraws
// the old rear wall. This measurement never authorizes wheel movement.
inline bool formatRearWallLine(char* out, size_t size, uint32_t acquisition, uint32_t ageMs,
                               const SteeringFrame& f) {
  if (!out || ageMs > MAX_WALL_AGE_MS) return false;
  const bool valid = f.rearWallKnown && f.rearDistance >= 10.0f &&
      f.rearDistance <= 300.0f && fabsf(f.rearHeading) <= 20.0f;
  char payload[80];
  const int used = snprintf(payload, sizeof(payload), "D,%lu,%lu,%u,%ld,%ld",
      (unsigned long)acquisition, (unsigned long)ageMs, valid ? 1U : 0U,
      valid ? lroundf(f.rearDistance * 10.0f) : 0L,
      valid ? lroundf(f.rearHeading * 100.0f) : 0L);
  if (used <= 0 || (size_t)used >= sizeof(payload)) return false;
  const int total = snprintf(out, size, "%s*%02X", payload, (unsigned)lineChecksum(payload));
  return total > 0 && (size_t)total < size;
}
