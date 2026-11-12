#pragma once
// A kinematic hallway for the real executor firmware (2026-11-11 navigation review).
//
// It turns the two host steppers' step counts into chassis motion, and gives the
// executor what the S3 bridge would send on the UART: the 50 Hz gyro line (with a
// configurable bias and scale error), GO / GO:SLOW path commands, and one wall
// line per LiDAR scan, delayed like the real Pi -> BLE -> bridge path.
//
// It is a MODEL. Wheels never slip, the stepper stub has no ramp, walls are ideal
// lines with Gaussian noise. It can show that the control laws converge and that
// the gates hold; it cannot show physical straightness, stopping distance or which
// motor is really the left wheel.
#include <cmath>
#include <cstdio>
#include <deque>
#include <string>
#include <utility>
#include <vector>

namespace hallway_sim {

struct Gap { double fromCm, toCm; };

struct Plant {
  // ---- robot ----
  double stepsPerCm = 1600.0 / (3.14159265 * 6.5);  // the header's unverified geometry
  double stepsPerDeg = 4900.0 / 90.0;               // steps per wheel per chassis degree
  bool motor1IsLeft = true;
  double leftWheelScale = 1.0;      // 1.005 = left wheel 0.5% larger: a steady pull right
  // ---- gyro ----
  double gyroScale = 1.0;           // measured / true rotation
  double gyroBiasDps = 0.0;
  bool streamMpu = true;
  uint32_t epoch = 1;
  // ---- path commands ----
  bool sendGo = true;
  bool slowZone = false;
  int allow = 3;
  // ---- walls ----
  bool walls = true;
  double widthCm = 190.0;           // the measured 95 + 95 cm hallway
  double distanceNoiseCm = 1.0, headingNoiseDeg = 0.3;
  double wallHeadingErrorDeg = 0.0;  // a door leaf or clutter fitted instead of the wall
  bool swapWalls = false;           // LiDAR left/right reversed against the MPU (a sign error)
  std::vector<Gap> leftGaps, rightGaps;   // along-hallway ranges with no wall on that side
  uint32_t scanPeriodMs = 174, lidarLatencyMs = 250;
  // ---- state ----
  double xCm = 0, yCm = 0;          // along the hallway, lateral (+ = left of centre)
  double thetaDeg = 0;              // true chassis heading, world frame, + = left
  double axisDeg = 0;               // hallway direction, world frame
  double gyroDeg = 0, rateDps = 0;
  double maxAbsY = 0;

  FastAccelStepper* m1 = nullptr;
  FastAccelStepper* m2 = nullptr;
  std::string* rx = nullptr;        // the executor's UART receive buffer

  void attach(FastAccelStepper* motor1, FastAccelStepper* motor2, std::string* uartRx) {
    m1 = motor1; m2 = motor2; rx = uartRx;
    last1 = m1->getCurrentPosition(); last2 = m2->getCurrentPosition();
    lastMs = lastM = lastScan = lastGo = fakeNow;
  }

  // The robot now stands in a hallway whose direction is the nearest right angle
  // to its heading (after a turn), at the given lateral offset.
  void enterHallway(double lateralCm) {
    axisDeg = 90.0 * std::round(thetaDeg / 90.0);
    xCm = 0; yCm = lateralCm; maxAbsY = std::fabs(lateralCm);
  }
  // Heading relative to the nearest right-angle hallway direction.
  double headingVsHallway() const {
    const double relative = wrap(thetaDeg - axisDeg);
    return relative - 90.0 * std::round(relative / 90.0);
  }

  void tick() {
    if (!m1 || !m2 || !rx) return;
    const uint32_t now = fakeNow;
    const uint32_t dt = now - lastMs;
    if (dt == 0) return;
    lastMs = now;
    const long p1 = m1->getCurrentPosition(), p2 = m2->getCurrentPosition();
    const double d1 = (double)(p1 - last1), d2 = (double)(p2 - last2);
    last1 = p1; last2 = p2;
    const double left = (motor1IsLeft ? d1 : d2) * leftWheelScale;
    const double right = motor1IsLeft ? d2 : d1;
    const double turned = 0.5 * (right - left) / stepsPerDeg;
    const double forward = 0.5 * (left + right) / stepsPerCm;
    thetaDeg += turned;
    const double relative = (thetaDeg - axisDeg) * 0.017453292519943295;
    xCm += forward * std::cos(relative);
    yCm += forward * std::sin(relative);
    if (std::fabs(yCm) > maxAbsY) maxAbsY = std::fabs(yCm);
    const double measured = turned * gyroScale + gyroBiasDps * dt * 0.001;
    gyroDeg += measured;
    rateDps = measured / (dt * 0.001);

    if (streamMpu && now - lastM >= 20) {
      lastM = now;
      char payload[64];
      std::snprintf(payload, sizeof(payload), "M,%lu,%ld,%ld,%lu,%d", (unsigned long)++sequence,
                    std::lround(wrap(gyroDeg) * 100.0), std::lround(clamp(rateDps, -249.0, 249.0) * 100.0),
                    (unsigned long)epoch, allow);
      send(payload);
    }
    if (sendGo && now - lastGo >= 150) {
      lastGo = now;
      *rx += slowZone ? "GO:SLOW\n" : "GO\n";
    }
    if (walls && now - lastScan >= scanPeriodMs) {
      lastScan = now;
      const double relDeg = wrap(thetaDeg - axisDeg);
      const double heading = relDeg - 90.0 * std::round(relDeg / 90.0);
      const bool left = !inGap(leftGaps), right = !inGap(rightGaps);
      const bool haveHeading = (left || right) && std::fabs(heading) <= 20.0;
      const unsigned flags = (left ? 1U : 0U) | (right ? 2U : 0U) | (haveHeading ? 4U : 0U);
      ++acquisition;
      if (flags != 0) {
        double leftCm = widthCm / 2 - yCm + noise(distanceNoiseCm);
        double rightCm = widthCm / 2 + yCm + noise(distanceNoiseCm);
        if (swapWalls) std::swap(leftCm, rightCm);
        const unsigned shownFlags = swapWalls
            ? ((flags & 4U) | ((flags & 1U) << 1) | ((flags & 2U) >> 1)) : flags;
        char payload[80];
        const bool showLeft = (shownFlags & 1U) != 0, showRight = (shownFlags & 2U) != 0;
        std::snprintf(payload, sizeof(payload), "W,%lu,%lu,%u,%ld,%ld,%ld", (unsigned long)acquisition,
                      (unsigned long)lidarLatencyMs, shownFlags,
                      showLeft ? std::lround(leftCm * 10.0) : 0L,
                      showRight ? std::lround(rightCm * 10.0) : 0L,
                      haveHeading ? std::lround((heading + wallHeadingErrorDeg + noise(headingNoiseDeg)) * 100.0) : 0L);
        pending.push_back({now + lidarLatencyMs, checked(payload)});
      }
    }
    while (!pending.empty() && (int32_t)(now - pending.front().first) >= 0) {
      *rx += pending.front().second;
      pending.pop_front();
    }
  }

  static double wrap(double deg) {
    while (deg > 180.0) deg -= 360.0;
    while (deg < -180.0) deg += 360.0;
    return deg;
  }

 private:
  long last1 = 0, last2 = 0;
  uint32_t lastMs = 0, lastM = 0, lastScan = 0, lastGo = 0, sequence = 0, acquisition = 0;
  uint32_t random = 12345;
  std::deque<std::pair<uint32_t, std::string>> pending;

  static double clamp(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
  bool inGap(const std::vector<Gap>& gaps) const {
    for (const Gap& gap : gaps) if (xCm >= gap.fromCm && xCm <= gap.toCm) return true;
    return false;
  }
  double uniform() {
    random = random * 1664525U + 1013904223U;
    return ((random >> 8) + 0.5) / 16777216.0;
  }
  double noise(double sigma) {  // deterministic Gaussian (Box-Muller)
    if (sigma <= 0) return 0;
    return sigma * std::sqrt(-2.0 * std::log(uniform())) * std::cos(6.283185307179586 * uniform());
  }
  static std::string checked(const char* payload) {
    uint8_t x = 0;
    for (const char* c = payload; *c; ++c) x ^= (uint8_t)*c;
    char line[112];
    std::snprintf(line, sizeof(line), "%s*%02X\n", payload, (unsigned)x);
    return line;
  }
  void send(const char* payload) { *rx += checked(payload); }
};

}  // namespace hallway_sim
