#pragma once
// Obstacle sonar sweep for the executor (2026-11-12).
//
// The HC-SR04 sits low on the front of the base, under the LiDAR's scan plane
// (the LiDAR is on top of the bin), so it is the only sensor that sees chair
// legs, bins, boxes, feet and small children. Until 2026-11-12 the servo was
// only ever commanded to the forward view: it never moved, and the sonar saw a
// single cone straight ahead. This class decides where the servo points and
// turns every echo into evidence. The .cpp owns the servo, trigger and ISR.
//
//   CRUISE  driving straight: forward, +25, forward, -25 ... The forward beam
//           gets every other ping; the +/-25 beams cover the body's corners.
//   SURVEY  stopped inside a route (holds, start gates, before turns): sweeps
//           the servo's whole span, +/-25, +/-50, +/-70 degrees with forward
//           between each, so the bearing a hold usually waits on is re-read
//           every ~0.2 s (host model). With the beam's own width that covers
//           about 165 degrees in front.
//   FORWARD spinning in place, and parked/idle: forward only.
//
// Bearings: 0 = straight ahead, + = left. Ranges are from the sonar. The beam
// is roughly 25-30 degrees wide, so an echo's bearing is uncertain: path checks
// take the beam edge nearest the robot's path (conservative), the spin check
// the edge nearest the spin centre.
//
// A ping that returns no echo inside the timeout (ULTRASONIC_MAX_RANGE_CM, 1.1 m
// since 2026-11-14; about 3 m before), or an echo from further away, means nothing
// is in range. In an open hallway that is the normal reading, so it counts as clear.
// Until 2026-11-12 it was "unknown", which could never release a hold once an
// obstacle had gone: the robot stayed stopped forever.
#include <Arduino.h>
#include <math.h>
#include <stdio.h>

class SonarSweep {
 public:
  enum class Pattern : uint8_t { FORWARD, CRUISE, SURVEY };

  // ---- mounting (servo commands from MAIN_EXECUTOR_CODE.h) ----
  // Higher command = further left (left = 140, right = 0, PINOUTS.txt).
  static constexpr int FORWARD_CMD = DEFAULT_VIEW;
  static constexpr int RIGHT_LIMIT_CMD = SCAN_RIGHT;
  static constexpr int LEFT_LIMIT_CMD = SCAN_LEFT;
  // ---- geometry, cm (unmeasured defaults, same chassis as the Pi) ----
  static constexpr float SONAR_AHEAD_OF_CENTRE_CM = 30.0f;  // front edge of a 60 cm base
  static constexpr float HALF_WIDTH_CM = 30.0f;              // 60 cm chassis (owner)
  static constexpr float CORRIDOR_MARGIN_CM = 8.0f;
  static constexpr float BEAM_EDGE_DEG = 12.0f;              // echo bearing uncertainty
  static constexpr float SPIN_RADIUS_CM = 50.0f;             // half diagonal 42.4 + 8, as the Pi's turn sweep
  // ---- station (2026-11-14) ----
  // Distance from the sonar to the station wall when the robot stands at the end
  // of the last straight of the return route, before its final turn. NOT
  // MEASURED: negative means unknown, and then a station step that faces the
  // wall is not held by the forward sonar at all. Measure it with a tape and set
  // it: an echo more than STATION_SONAR_MARGIN_CM nearer than the wall should
  // be then holds the robot.
  static constexpr float STATION_PARK_GAP_CM = -1.0f;
  static constexpr float STATION_SONAR_MARGIN_CM = 15.0f;
  // ---- evidence ----
  static constexpr float STOP_AHEAD_CM = ULTRASONIC_STOP_DISTANCE_CM;
  static constexpr float SLOW_AHEAD_CM = ULTRASONIC_SLOW_DISTANCE_CM;
  static constexpr float EMERGENCY_CM = VERY_CLOSE_DISTANCE_CM;
  static constexpr uint8_t BLOCK_CONFIRM = 2;     // same bearing, consecutive
  static constexpr uint8_t CLEAR_CONFIRM = 2;
  static constexpr uint32_t WIDE_FRESH_MS = 2500;  // +/-50, +/-70 count only this long (not swept while driving)
  static constexpr uint32_t SPIN_FRESH_MS = 3000;
  // Three complete surveys (the side bearings face the hallway walls) without one
  // echo at any bearing: report SONAR:SILENT. Driving time never counts toward it.
  static constexpr uint8_t SILENT_SURVEYS = 3;

  static constexpr int MAX_SLOTS = 9;
  static constexpr int MAX_SURVEY_STEPS = 2 * MAX_SLOTS;
  static constexpr int FAN_OFFSET_DEG = 25;

  struct Slot {
    int offsetDeg = 0;
    int command = FORWARD_CMD;
    bool sampled = false;
    bool echo = false;
    float cm = 999.0f;
    uint32_t ms = 0;
    uint8_t nearStreak = 0, clearStreak = 0;
    bool blocking = false;
  };

  SonarSweep() { begin(); }

  void begin() {
    static const int candidates[] = {-90, -70, -50, -FAN_OFFSET_DEG, 0, FAN_OFFSET_DEG, 50, 70, 90};
    count_ = 0;
    for (int offset : candidates) {
      const int command = FORWARD_CMD + offset;
      if (command < RIGHT_LIMIT_CMD || command > LEFT_LIMIT_CMD || count_ >= MAX_SLOTS) continue;
      slots_[count_] = Slot();
      slots_[count_].offsetDeg = offset;
      slots_[count_].command = command;
      ++count_;
    }
    // Survey order: forward between every side bearing, sides alternating and
    // widening: 0, +25, 0, -25, 0, +50, 0, -50, 0, +70, 0, -70 (servo span allowing).
    surveyCount_ = 0;
    static const int widths[] = {FAN_OFFSET_DEG, 50, 70, 90};
    for (int width : widths) {
      for (int sign = 1; sign >= -1; sign -= 2) {
        const Slot* s = findOffset(sign * width);
        if (s == nullptr || surveyCount_ + 2 > MAX_SURVEY_STEPS) continue;
        surveyOrder_[surveyCount_++] = FORWARD_CMD;
        surveyOrder_[surveyCount_++] = s->command;
      }
    }
    if (surveyCount_ == 0) surveyOrder_[surveyCount_++] = FORWARD_CMD;
    surveyIndex_ = 0;
    cruiseStep_ = 0;
    lastEchoMs_ = 0;
    quietSurveySamples_ = 0;
    silentReported_ = false;
  }

  // Forget every sample (sonar switched off/on): nothing old can clear or block.
  void reset() { begin(); }

  int slotCount() const { return count_; }
  const Slot& slot(int i) const { return slots_[i]; }

  // The servo command for the current step of a pattern.
  int target(Pattern pattern) const {
    switch (pattern) {
      case Pattern::CRUISE: {
        static const int order[4] = {0, FAN_OFFSET_DEG, 0, -FAN_OFFSET_DEG};
        const int command = FORWARD_CMD + order[cruiseStep_ & 3U];
        return command < RIGHT_LIMIT_CMD || command > LEFT_LIMIT_CMD ? FORWARD_CMD : command;
      }
      case Pattern::SURVEY:
        return surveyOrder_[surveyIndex_];
      default:
        return FORWARD_CMD;
    }
  }

  // Move to the pattern's next step after a ping has completed.
  void advance(Pattern pattern) {
    if (pattern == Pattern::CRUISE) {
      cruiseStep_ = (uint8_t)((cruiseStep_ + 1U) & 3U);
    } else if (pattern == Pattern::SURVEY) {
      surveyIndex_ = (surveyIndex_ + 1) % surveyCount_;
    }
  }

  // Steps in one complete survey (every bearing once).
  int surveySteps() const { return surveyCount_; }

  // Restart the survey from its beginning (forward first).
  void restartSurvey() { surveyIndex_ = 0; }

  // One completed ping at a servo command: an echo at cm, or no echo in range.
  void record(int command, bool echo, float cm, uint32_t now) {
    Slot* s = find(command);
    if (s == nullptr) return;
    if (echo && !(isfinite(cm) && cm >= 1.5f && cm <= 999.0f)) return;  // glitch: no evidence
    s->sampled = true;
    s->echo = echo;
    s->cm = echo ? cm : 999.0f;
    s->ms = now;
    if (echo) {
      lastEchoMs_ = now;
      quietSurveySamples_ = 0;
      silentReported_ = false;
    }
    // Emergency: an echo within EMERGENCY_CM of the sonar that can be in the path.
    const bool emergency = echo && cm <= EMERGENCY_CM && inPath(s->offsetDeg, cm, STOP_AHEAD_CM);
    if (emergency || (echo && inPath(s->offsetDeg, cm, STOP_AHEAD_CM))) {
      if (s->nearStreak < 255U) ++s->nearStreak;
      s->clearStreak = 0;
      if (emergency || s->nearStreak >= BLOCK_CONFIRM) s->blocking = true;
    } else {
      if (s->clearStreak < 255U) ++s->clearStreak;
      s->nearStreak = 0;
      if (s->clearStreak >= CLEAR_CONFIRM) s->blocking = false;
    }
  }

  // Something low in the path at a side bearing (the forward bearing keeps its
  // own latch in the .cpp). Fan bearings hold until re-sampled clear; the wide
  // ones are only swept while stopped, so they count while fresh.
  bool sideBlocking(uint32_t now) const {
    for (int i = 0; i < count_; ++i) {
      const Slot& s = slots_[i];
      if (s.offsetDeg == 0 || !s.blocking) continue;
      if (abs(s.offsetDeg) <= FAN_OFFSET_DEG || now - s.ms <= WIDE_FRESH_MS) return true;
    }
    return false;
  }

  // A fresh side echo inside the caution distance of the path.
  bool sideSlow(uint32_t now) const {
    for (int i = 0; i < count_; ++i) {
      const Slot& s = slots_[i];
      if (s.offsetDeg == 0 || abs(s.offsetDeg) > FAN_OFFSET_DEG || !s.sampled || !s.echo) continue;
      if (now - s.ms <= WIDE_FRESH_MS && inPath(s.offsetDeg, s.cm, SLOW_AHEAD_CM)) return true;
    }
    return false;
  }

  // Spin-in-place clearance at floor level: every bearing sampled since sinceMs
  // and none inside the spin circle. A point inside the circle stays inside it
  // while the robot rotates in place, so samples taken before the spin hold.
  enum class Spin : uint8_t { CLEAR, BLOCKED, UNKNOWN };
  Spin spinClearance(uint32_t sinceMs, uint32_t now) const {
    bool complete = true;
    for (int i = 0; i < count_; ++i) {
      const Slot& s = slots_[i];
      if (!s.sampled || now - s.ms > SPIN_FRESH_MS) {
        complete = false;
        continue;
      }
      if (s.echo && insideSpin(s.offsetDeg, s.cm)) return Spin::BLOCKED;
      if ((int32_t)(s.ms - sinceMs) < 0) complete = false;
    }
    return complete ? Spin::CLEAR : Spin::UNKNOWN;
  }

  // Call after record() for every completed ping. True once when SILENT_SURVEYS
  // complete surveys have passed without one echo at any bearing (unplugged,
  // broken, or a truly empty space). It never blocks motion. Only survey pings
  // count: driving down an open hallway hears nothing ahead, and that is normal.
  bool silentNow(bool echo, bool surveying) {
    if (echo || !surveying || silentReported_) return false;
    if (quietSurveySamples_ < 0xFFFFU) ++quietSurveySamples_;
    if (quietSurveySamples_ < (uint32_t)SILENT_SURVEYS * (uint32_t)surveyCount_) return false;
    silentReported_ = true;
    return true;
  }
  uint32_t lastEchoMs() const { return lastEchoMs_; }

  // "-70:101 -50:--- ..." for a USB log line (--- = no echo, ? = not sampled).
  void summary(char* out, size_t size) const {
    if (out == nullptr || size == 0) return;
    size_t used = 0;
    out[0] = '\0';
    for (int i = 0; i < count_ && used + 12 < size; ++i) {
      const Slot& s = slots_[i];
      int written;
      if (!s.sampled) written = snprintf(out + used, size - used, "%s%+d:?", used ? " " : "", s.offsetDeg);
      else if (!s.echo) written = snprintf(out + used, size - used, "%s%+d:---", used ? " " : "", s.offsetDeg);
      else written = snprintf(out + used, size - used, "%s%+d:%.0f%s", used ? " " : "", s.offsetDeg, s.cm,
                              s.blocking ? "!" : "");
      if (written < 0) break;
      used += (size_t)written;
    }
  }

  // ---- geometry (public for tests) ----
  // Can an echo at this bearing and range lie in the robot's path within
  // `aheadCm` of the sonar? Uses the beam edge nearest straight ahead.
  static bool inPath(int offsetDeg, float cm, float aheadCm) {
    const float edge = fmaxf(0.0f, fabsf((float)offsetDeg) - BEAM_EDGE_DEG) * DEG_TO_RAD;
    const float lateral = cm * sinf(edge);
    const float ahead = cm * cosf(edge);
    return lateral <= HALF_WIDTH_CM + CORRIDOR_MARGIN_CM && ahead <= aheadCm;
  }
  // Can it lie inside the circle the body sweeps when spinning in place?
  static bool insideSpin(int offsetDeg, float cm) {
    const float edge = fminf(180.0f, fabsf((float)offsetDeg) + BEAM_EDGE_DEG) * DEG_TO_RAD;
    const float s = SONAR_AHEAD_OF_CENTRE_CM;
    const float d2 = s * s + cm * cm + 2.0f * s * cm * cosf(edge);
    return d2 <= SPIN_RADIUS_CM * SPIN_RADIUS_CM;
  }

 private:
  Slot* find(int command) {
    for (int i = 0; i < count_; ++i) if (slots_[i].command == command) return &slots_[i];
    return nullptr;
  }
  const Slot* findOffset(int offsetDeg) const {
    for (int i = 0; i < count_; ++i) if (slots_[i].offsetDeg == offsetDeg) return &slots_[i];
    return nullptr;
  }

  Slot slots_[MAX_SLOTS];
  int count_ = 0;
  int surveyOrder_[MAX_SURVEY_STEPS] = {};
  int surveyCount_ = 0;
  int surveyIndex_ = 0;
  uint8_t cruiseStep_ = 0;
  uint32_t lastEchoMs_ = 0;
  uint32_t quietSurveySamples_ = 0;
  bool silentReported_ = false;
};
