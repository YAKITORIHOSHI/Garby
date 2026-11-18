#pragma once
// Continue a route after the operator's emergency stop (owner, 2026-11-14).
//
// The routes are step lists (pointsRun.ino). While a route runs, this tracks the
// step in progress and, for a straight, the wheel positions where its travel
// began. The operator's emergency stop saves what is left of the route: the
// step, the straight travel still to drive (mean of both wheels, as the straight
// itself measures it), whether a turn had already set its target in the route
// heading frame, and the S3 heading-reference count. "Release stop" discards
// it (the route ends at IDLE, as before); "Continue run" re-arms it, and the
// route function picks up at that step instead of starting again.
//
// Nothing here grants motion. The continued route passes the same gates as any
// route: fresh GO, no STOP, sonar, MPU heading and the spin sweep before turns.
// A controller restart loses the saved point (RAM only), so Continue is then
// unavailable.
//
// "Reverse to start" (owner, 2026-11-14) uses the same record the other way:
// for a calibration run that stopped short, the robot turns around and drives
// FORWARD back along the travel it covered, to the start of the route, and
// faces the way it started (pointsRun.ino, runReverse). While it does, the
// record keeps saying where on the route the robot is, so an emergency stop
// during the way back can be continued or reversed again. The record also
// carries what is needed to find the route's start heading again:
// anchorFromTarget, the angle from the heading the robot is holding (the route
// target) to the heading the route started with.
#include <stdint.h>

// BACKING (owner, 2026-11-16, reverse parking): drive the step's wheel steps
// BACKWARDS into the parking spot at the end of a route (backIntoPlace). Its leg
// travel counts the steps driven backwards, so Continue resumes it the same way.
enum class RouteOp : uint8_t { BUMPS, STRAIGHT, LONG_HALLWAY, TURN_LEFT, TURN_RIGHT, BACKING };

// Station steps (owner, 2026-11-14: "Don't be bothered with walls during the
// first and parking, garby's station can be found at the side of hallways").
// STATION: this step is driven at the station, beside its walls. The executor
// says so in its [LEG:..] notice, and the Pi then leaves returns that belong to
// a long straight wall out of its LiDAR holds; a turn skips the wall alignment.
// WALL_AHEAD_*: the station wall is ahead of the sonar when the step is driven
// forward along the route (FWD) or on the way back of a reverse run (REV), so a
// sonar echo from it is expected (SonarSweep.h, STATION_PARK_GAP_CM).
static constexpr uint8_t STEP_STATION = 0x01;
static constexpr uint8_t STEP_WALL_AHEAD_FWD = 0x02;
static constexpr uint8_t STEP_WALL_AHEAD_REV = 0x04;

struct RouteStep {
  RouteOp op;
  int32_t amount;     // wheel steps, or degrees for a turn
  uint8_t flags = 0;  // STEP_* bits
};

class RouteResume {
 public:
  enum Route : uint8_t { NONE = 0, OUTBOUND = 1, RETURN = 2 };

  struct Point {
    uint8_t route = NONE;
    uint8_t step = 0;           // the step to run first
    int32_t remaining = -1;     // straight travel still to drive; -1 = the step's full amount
    bool turnCommitted = false; // the turn's target is already in the route frame
    bool anchored = false;      // the route heading frame exists (else start the route again)
    uint32_t reanchors = 0;     // heading reference count when it stopped
    bool reversed = false;      // stopped on the way back of a reverse run
    float anchorFromTarget = 0.0f;  // route start heading minus the route target, degrees
  };

  // ---- live progress (the route runner) ----
  void beginRoute(uint8_t route, bool reverse = false) {
    live_ = Live();
    live_.route = route;
    live_.reverse = reverse;
  }
  void endRoute() { live_ = Live(); }
  void anchored() { live_.anchored = true; }
  // anchorFromTarget: the route's start heading minus the route target while
  // this step runs (before a turn step adds its angle). stepTurn: the angle a
  // turn step adds to the route target (+ = left), 0 for a straight.
  void beginStep(uint8_t index, int32_t planned, float anchorFromTarget = 0.0f,
                 float stepTurn = 0.0f) {
    live_.step = index;
    live_.planned = planned;
    live_.amount = planned;
    live_.legActive = false;
    live_.turnCommitted = false;
    live_.anchorFromTarget = anchorFromTarget;
    live_.stepTurn = stepTurn;
  }
  // The way back through one step of a reverse run. forwardRemaining is where
  // the robot stands in the step, as a forward route would count it: the travel
  // from here to the step's end.
  void beginReverseStep(uint8_t index, int32_t amount, int32_t forwardRemaining,
                        float anchorFromTarget) {
    live_.step = index;
    live_.planned = forwardRemaining;
    live_.amount = amount;
    live_.legActive = false;
    live_.turnCommitted = false;
    live_.anchorFromTarget = anchorFromTarget;
    live_.stepTurn = 0.0f;
  }
  // The straight travel of the current step starts here (after any pre-run
  // centring, whose sidesteps are not route travel). backward: the leg is driven
  // backwards (a BACKING step), so falling step counts are its progress.
  void beginLeg(int32_t steps, long position1, long position2, bool backward = false) {
    live_.legActive = true;
    live_.legSteps = steps;
    live_.legStart1 = position1;
    live_.legStart2 = position2;
    live_.legBackward = backward;
  }
  // A route turn added its angle to the route heading frame.
  void commitTurn() {
    if (live_.route != NONE) live_.turnCommitted = true;
  }
  // Every step has completed: the robot is parked at the end of the route.
  void routeDone(uint8_t stepCount, float anchorFromTarget = 0.0f) {
    live_.step = stepCount;
    live_.legActive = false;
    live_.turnCommitted = false;
    live_.planned = -1;
    live_.parked = true;
    live_.anchorFromTarget = anchorFromTarget;
    live_.stepTurn = 0.0f;
  }
  bool routeActive() const { return live_.route != NONE; }
  bool reverseActive() const { return live_.route != NONE && live_.reverse; }
  // Straight travel still to drive in the leg in progress; -1 without one.
  int32_t legLeft(long position1, long position2) const {
    if (live_.route == NONE || !live_.legActive) return -1;
    const int64_t left = (int64_t)live_.legSteps - legDone(position1, position2);
    return left < 0 ? 0 : (int32_t)left;
  }

  // ---- the operator's emergency stop ----
  // Save what is left of the live route. With no live route, an armed point
  // (Continue accepted, route not yet restarted) is kept for another Continue.
  void capture(long position1, long position2, uint32_t reanchors) {
    if (live_.route == NONE) {
      armed_ = false;
      reverseArmed_ = false;
      return;
    }
    Point point;
    point.route = live_.route;
    point.step = live_.step;
    point.anchored = live_.anchored;
    point.reanchors = reanchors;
    point.turnCommitted = live_.turnCommitted;
    point.reversed = live_.reverse;
    // A committed turn already added its angle to the route target.
    point.anchorFromTarget =
        live_.anchorFromTarget - (live_.turnCommitted ? live_.stepTurn : 0.0f);
    if (live_.legActive) {
      int64_t left = (int64_t)live_.legSteps - legDone(position1, position2);
      if (left < 0) left = 0;
      if (live_.reverse) {
        // Still to drive back = travel already made into this step.
        const int64_t forwardLeft = (int64_t)live_.amount - left;
        point.remaining = forwardLeft < 0 ? 0 : (int32_t)forwardLeft;
      } else {
        point.remaining = (int32_t)left;
      }
    } else {
      point.remaining = live_.planned;
    }
    saved_ = point;
    haveSaved_ = true;
    armed_ = false;
    reverseArmed_ = false;
  }

  bool available() const { return haveSaved_; }
  // Reverse needs the route's heading frame: a route stopped before it was
  // anchored has not moved, and has nothing to drive back.
  bool reverseAvailable() const { return haveSaved_ && saved_.anchored; }
  const Point& saved() const { return saved_; }
  // Release, a new route or a reset: the stopped route will not continue.
  void discard() {
    haveSaved_ = false;
    armed_ = false;
    reverseArmed_ = false;
    saved_ = Point();
  }
  // Continue accepted: the next run of that route resumes from the saved point.
  // A point saved on the way back of a reverse run continues that way back.
  bool arm() {
    if (!haveSaved_) return false;
    if (saved_.reversed) return armReverse();
    armed_ = true;
    reverseArmed_ = false;
    return true;
  }
  // Reverse accepted: the reverse runner takes the saved point.
  bool armReverse() {
    if (!reverseAvailable()) return false;
    reverseArmed_ = true;
    armed_ = false;
    return true;
  }
  bool armedFor(uint8_t route) const {
    return haveSaved_ && armed_ && !reverseArmed_ && saved_.route == route;
  }
  bool reverseArmed() const { return haveSaved_ && reverseArmed_; }
  Point takeReverse() {
    Point point;
    if (reverseArmed()) {
      point = saved_;
      discard();
    }
    return point;
  }
  // The route runner takes the armed point. Anything else gets a fresh start.
  Point take(uint8_t route) {
    Point point;
    if (armedFor(route)) {
      point = saved_;
      discard();
    }
    return point;
  }

 private:
  // Mean travel of both wheels in the leg's own direction of travel.
  int64_t legDone(long position1, long position2) const {
    const int64_t moved = ((int64_t)(position1 - live_.legStart1) +
                           (int64_t)(position2 - live_.legStart2)) / 2;
    return live_.legBackward ? -moved : moved;
  }

  struct Live {
    uint8_t route = NONE;
    uint8_t step = 0;
    int32_t planned = -1;
    int32_t amount = 0;         // the step's whole straight travel
    bool anchored = false;
    bool legActive = false;
    int32_t legSteps = 0;
    long legStart1 = 0, legStart2 = 0;
    bool legBackward = false;   // a BACKING leg: falling step counts are progress
    bool turnCommitted = false;
    bool parked = false;
    bool reverse = false;       // the way back of a reverse run
    float anchorFromTarget = 0.0f;
    float stepTurn = 0.0f;
  };
  Live live_;
  Point saved_;
  bool haveSaved_ = false;
  bool armed_ = false;
  bool reverseArmed_ = false;
};
