// .ino (Extension)
// ============================
// RUN DRAFT END-TO-END
// ============================

// The routes are step lists (2026-11-14) so the operator's "Continue run" can
// pick a route up at the step where the emergency stop interrupted it
// (RouteResume.h). The step counts and turn angles are the calibrated values of
// the earlier hand-written sequences, unchanged and in the same order.
//
// BUMPS n
//   Short calibration bumps, nudge disabled: n steps in moves of 100.
// STRAIGHT n
//   safeMoveDistance(n, false, false). Every straight longer than FINISH_STEPS
//   holds the MPU route heading.
// LONG_HALLWAY n  (2026-11-12)
//   Stand still and let the LiDAR walls put the robot on the centre line and
//   axis first (centerBeforeLongRun), then drive the straight with wall fusion.
//   The sidesteps' forward travel is taken off the straight, so the run still
//   ends where its calibrated step count says.
// TURN_LEFT / TURN_RIGHT degrees
//   MPU-measured spins, absolute along the route (2026-11-11: the old
//   4900/5000-step turns are 90 degrees, the 9800-step turn is 180). Each turn
//   is then checked against the LiDAR walls and corrected to the hallway.

//
// Station steps (owner, 2026-11-14): the station is at the hallway wall. The
// outbound route leaves it straight out to the hallway centre; the return route
// drives straight at it and turns round beside it. Those steps carry
// STEP_STATION (RouteResume.h): the Pi leaves the station's walls out of its
// LiDAR holds while they run, and the turn beside the wall skips the wall
// alignment. STEP_WALL_AHEAD_* marks where the sonar faces the station wall:
// driving the return's last steps forward, and driving the outbound's first
// steps on the way back of a reverse run. The outbound's first turn is at the
// hallway centre and is an ordinary turn.

static const int32_t CALIBRATION_BUMP_STEPS = 100;
static const int32_t CALIBRATION_BUMPS = 5 * CALIBRATION_BUMP_STEPS;

static const uint8_t LEAVING_STATION = STEP_STATION | STEP_WALL_AHEAD_REV;
static const uint8_t PARKING = STEP_STATION | STEP_WALL_AHEAD_FWD;

static const RouteStep OUTBOUND_ROUTE[] = {
  {RouteOp::BUMPS, CALIBRATION_BUMPS, LEAVING_STATION},
  {RouteOp::STRAIGHT, 8000, LEAVING_STATION},
  {RouteOp::TURN_LEFT, 90},
  {RouteOp::BUMPS, CALIBRATION_BUMPS},
  {RouteOp::LONG_HALLWAY, 595500},
  {RouteOp::TURN_LEFT, 90},
  {RouteOp::BUMPS, CALIBRATION_BUMPS},
  {RouteOp::STRAIGHT, 6000},
  {RouteOp::TURN_RIGHT, 90},
  {RouteOp::BUMPS, CALIBRATION_BUMPS},
  {RouteOp::STRAIGHT, 25500},
  {RouteOp::TURN_RIGHT, 90},
  {RouteOp::BUMPS, CALIBRATION_BUMPS},
  {RouteOp::STRAIGHT, 10600},
  {RouteOp::TURN_LEFT, 180},
};

static const RouteStep RETURN_ROUTE[] = {
  {RouteOp::BUMPS, CALIBRATION_BUMPS},
  {RouteOp::STRAIGHT, 9500},
  {RouteOp::TURN_LEFT, 90},
  {RouteOp::BUMPS, CALIBRATION_BUMPS},
  {RouteOp::STRAIGHT, 25500},
  {RouteOp::TURN_LEFT, 90},
  {RouteOp::BUMPS, CALIBRATION_BUMPS},
  {RouteOp::STRAIGHT, 6000},
  {RouteOp::TURN_RIGHT, 90},
  {RouteOp::BUMPS, CALIBRATION_BUMPS},
  {RouteOp::LONG_HALLWAY, 595700},
  {RouteOp::TURN_RIGHT, 90},
  {RouteOp::BUMPS, CALIBRATION_BUMPS, PARKING},
  {RouteOp::STRAIGHT, 7000, PARKING},
  {RouteOp::TURN_LEFT, 180, PARKING},
};

static const uint8_t OUTBOUND_STEPS = sizeof(OUTBOUND_ROUTE) / sizeof(OUTBOUND_ROUTE[0]);
static const uint8_t RETURN_STEPS = sizeof(RETURN_ROUTE) / sizeof(RETURN_ROUTE[0]);

static bool routePause(unsigned long durationMs = 100UL) {
  responsiveDelay(durationMs);
  return !resetQueued;
}

static void beginRouteLeg(int32_t steps) {
  routeResume.beginLeg(steps, stepper1 ? stepper1->getCurrentPosition() : 0,
                       stepper2 ? stepper2->getCurrentPosition() : 0);
}

static bool runCalibrationBumps(int32_t steps) {
  beginRouteLeg(steps);
  for (int32_t done = 0; done < steps; done += CALIBRATION_BUMP_STEPS) {
    const int32_t bump = steps - done < CALIBRATION_BUMP_STEPS ? steps - done : CALIBRATION_BUMP_STEPS;
    if (!safeMoveDistance(bump, true, false)) return false;
    responsiveDelay(2);
  }
  return true;
}

static bool runLongHallway(int32_t steps) {
  int32_t progress = 0;
  // Pre-run centring is for a long run (both full hallway runs are ~595k steps). A
  // continued run's short tail keeps the straight's own wall centring instead: its
  // sidesteps could travel further forward than the tail itself.
  if (steps >= HeadingControl::PRECENTER_MIN_STEPS && !centerBeforeLongRun(&progress)) return false;
  // At most the straight itself (until 2026-11-14, progress over a quarter of the
  // straight was ignored, which only a short continued tail could reach).
  if (progress < 0) progress = 0;
  if (progress > steps) progress = steps;
  beginRouteLeg(steps - progress);
  return safeMoveDistance(steps - progress, false, true);
}

static bool runRouteStep(const RouteStep& step, int32_t amount, bool turnCommitted) {
  switch (step.op) {
    case RouteOp::BUMPS:
      return runCalibrationBumps(amount);
    case RouteOp::STRAIGHT:
      beginRouteLeg(amount);
      return amount <= 0 || safeMoveDistance(amount, false, false);
    case RouteOp::LONG_HALLWAY:
      return amount <= 0 || runLongHallway(amount);
    case RouteOp::TURN_LEFT:
      return turnCommitted ? resumeRouteTurn() : safeTurnLeft((float)amount);
    case RouteOp::TURN_RIGHT:
      return turnCommitted ? resumeRouteTurn() : safeTurnRight((float)amount);
  }
  return false;
}

static bool isStraightStep(const RouteStep& step) {
  return step.op == RouteOp::BUMPS || step.op == RouteOp::STRAIGHT ||
         step.op == RouteOp::LONG_HALLWAY;
}

// The angle a step adds to the route heading (+ = left).
static float stepTurnDeg(const RouteStep& step) {
  if (step.op == RouteOp::TURN_LEFT) return (float)step.amount;
  if (step.op == RouteOp::TURN_RIGHT) return -(float)step.amount;
  return 0.0f;
}

// The route heading at the start of step `index`, relative to the route's start.
static float headingBeforeStep(const RouteStep* steps, uint8_t index) {
  float total = 0.0f;
  for (uint8_t i = 0; i < index; ++i) total += stepTurnDeg(steps[i]);
  return total;
}

// Straight travel toward the station wall that follows step `index` in the
// direction being driven (the sonar's expected-wall distance needs it).
static int32_t wallAheadBeyond(const RouteStep* steps, uint8_t count, uint8_t index, bool reverse) {
  int32_t total = 0;
  if (reverse) {
    for (int16_t i = (int16_t)index - 1; i >= 0; --i) {
      if (!isStraightStep(steps[i]) || !(steps[i].flags & STEP_WALL_AHEAD_REV)) break;
      total += steps[i].amount;
    }
  } else {
    for (uint8_t i = index + 1U; i < count; ++i) {
      if (!isStraightStep(steps[i]) || !(steps[i].flags & STEP_WALL_AHEAD_FWD)) break;
      total += steps[i].amount;
    }
  }
  return total;
}

// One route, from its first step or from the point "Continue run" armed.
static bool runRoute(uint8_t route, const RouteStep* steps, uint8_t count, const char* name) {
  const RouteResume::Point resume = routeResume.take(route);
  const bool continuing = resume.route == route && resume.anchored && resume.step <= count;
  const uint8_t first = continuing ? resume.step : 0;
  routeResume.beginRoute(route);
  bool finished = false;
  struct LiveScope {
    bool* finished;
    ~LiveScope() {
      if (!*finished) routeResume.endRoute();
      routeLegEnd();
    }
  } scope{&finished};

  if (continuing) {
    Serial.printf("[%s] Continuing at step %u of %u (straight left %ld, turn set %d)\n", name,
                  (unsigned)first, (unsigned)count, (long)resume.remaining, resume.turnCommitted ? 1 : 0);
    ESP_Serial.printf("[ROUTE CONTINUE:%u/%u]\n", (unsigned)first, (unsigned)count);
  }

  // Request initial LiDAR/sensor status
  requestStatus();
  if (!routePause()) return false;
  if (first < count) {
    // No MPU heading: do not move. A continued route keeps its heading frame.
    if (continuing ? !resumeRouteHeading(resume.reanchors) : !beginRouteHeading()) return false;
  }
  routeResume.anchored();

  for (uint8_t i = first; i < count; ++i) {
    const bool resumed = continuing && i == first;
    const int32_t amount = resumed && resume.remaining >= 0 ? resume.remaining : steps[i].amount;
    routeResume.beginStep(i, amount, -headingBeforeStep(steps, i), stepTurnDeg(steps[i]));
    routeLegBegin(route, false, i, count, steps[i].op, steps[i].flags,
                  wallAheadBeyond(steps, count, i, false));
    if (!runRouteStep(steps[i], amount, resumed && resume.turnCommitted)) return false;
    // A pause after every step but the last, as the hand-written routes had.
    if (i + 1U < count && !routePause()) return false;
  }
  routeResume.routeDone(count, -headingBeforeStep(steps, count));
  finished = true;
  return true;
}

// ============================
// REVERSE TO START (owner, 2026-11-14)
// ============================
// For calibration runs: "the step value did not reach the end point, therefore
// the purpose of this reverse is let garby go back to point A on itself to start
// over". After an emergency stop the robot turns around and drives FORWARD back
// along the travel it covered, step by step with every turn mirrored, then faces
// the way the route started. Every gate of a route applies (fresh GO, no STOP,
// sonar, MPU heading, spin sweep), and it keeps driving forward, so the forward
// sonar, the front box and the moving-object prediction all face the way it goes.
//
// Travel to undo: every completed straight step in full, and the interrupted
// one by what it had covered. Headings are absolute: each leg is driven at its
// heading on the way out plus 180 degrees, so a stop in the middle of a turn
// needs no special case. Calibration bumps are driven back as plain travel.
// Not undone: the forward travel of pre-run centring sidesteps when the stop
// came during the centring itself (some tens of centimetres at most).
//
// It reaches the start only as well as the wheels repeat their steps: slip on
// the way out is not known and is not corrected.
static float nearestHeading(float targetDeg) {
  return heading.heading() + HeadingControl::wrap180(targetDeg - heading.heading());
}

static bool runReverseRoute(uint8_t route, const RouteStep* steps, uint8_t count, const char* name) {
  const RouteResume::Point point = routeResume.takeReverse();
  if (point.route != route || !point.anchored) return false;
  routeResume.beginRoute(route, true);
  bool finished = false;
  struct LiveScope {
    bool* finished;
    ~LiveScope() {
      routeResume.endRoute();
      routeLegEnd();
      (void)finished;
    }
  } scope{&finished};

  const uint8_t stoppedAt = point.step > count ? count : point.step;
  Serial.printf("[%s] Reversing from step %u of %u (straight left %ld)\n", name,
                (unsigned)stoppedAt, (unsigned)count, (long)point.remaining);
  ESP_Serial.printf("[ROUTE REVERSE:%u/%u]\n", (unsigned)stoppedAt, (unsigned)count);

  requestStatus();
  if (!routePause()) return false;
  // The heading frame of the stopped route; a new S3 reference needs the walls.
  if (!resumeRouteHeading(point.reanchors)) return false;
  routeResume.anchored();
  // The heading the route started with, in the present heading frame.
  float anchor = heading.routeTarget() + point.anchorFromTarget;

  for (int16_t j = (int16_t)(stoppedAt >= count ? count - 1 : stoppedAt); j >= 0; --j) {
    const RouteStep& step = steps[j];
    if (!isStraightStep(step)) continue;
    int32_t done = step.amount;
    if ((uint8_t)j == point.step) {
      const int32_t left = point.remaining < 0 ? step.amount : point.remaining;
      done = step.amount - left;
    }
    if (done > step.amount) done = step.amount;
    if (done <= 0) continue;
    const float legOffset = headingBeforeStep(steps, (uint8_t)j);
    const float target = nearestHeading(anchor + legOffset + 180.0f);
    // The record and the notice say where the robot is before anything moves.
    heading.setRouteTarget(target);
    routeResume.beginReverseStep((uint8_t)j, step.amount, step.amount - done, -180.0f - legOffset);
    routeLegBegin(route, true, (uint8_t)j, count, step.op, step.flags,
                  wallAheadBeyond(steps, count, (uint8_t)j, true));
    if (fabsf(target - heading.heading()) > HeadingControl::TURN_TOLERANCE_DEG) {
      if (!spinToRouteHeading(target, !(step.flags & STEP_STATION))) return false;
      if (!routePause()) return false;
    }
    beginRouteLeg(done);
    if (!safeMoveDistance(done, false, step.op == RouteOp::LONG_HALLWAY)) return false;
    // The wall alignment and the straight may have corrected the route frame.
    anchor = heading.routeTarget() - legOffset - 180.0f;
    if (!routePause()) return false;
  }

  // Back at the start: face the way the route started.
  const float startHeading = nearestHeading(anchor);
  heading.setRouteTarget(startHeading);
  routeResume.beginReverseStep(0, isStraightStep(steps[0]) ? steps[0].amount : 0,
                               isStraightStep(steps[0]) ? steps[0].amount : 0, 0.0f);
  routeLegBegin(route, true, 0, count, steps[0].op, steps[0].flags, 0);
  if (fabsf(startHeading - heading.heading()) > HeadingControl::TURN_TOLERANCE_DEG &&
      !spinToRouteHeading(startHeading, !(steps[0].flags & STEP_STATION))) return false;
  finished = true;
  return true;
}

// The reverse run of whichever route the operator's "Reverse to start" armed.
bool runReverse(uint8_t* routeOut) {
  const uint8_t route = routeResume.reverseArmed() ? routeResume.saved().route
                                                   : (uint8_t)RouteResume::NONE;
  if (routeOut) *routeOut = route;
  Serial.println("[REVERSE] Entered runReverse()");
  if (route == RouteResume::OUTBOUND)
    return runReverseRoute(route, OUTBOUND_ROUTE, OUTBOUND_STEPS, "REVERSE");
  if (route == RouteResume::RETURN)
    return runReverseRoute(route, RETURN_ROUTE, RETURN_STEPS, "REVERSE");
  return false;
}

bool runStart() {
  Serial.println("[RUNSTART] Entered runStart()");   // confirms this function is actually reached
  if (!runRoute(RouteResume::OUTBOUND, OUTBOUND_ROUTE, OUTBOUND_STEPS, "RUNSTART")) return false;
  Serial.println("[RUNSTART] runStart() finished all moves");
  return true;
}

bool returnToPointB() {
  Serial.println("[RETURNTOB] Entered returnToPointB()");
  if (!runRoute(RouteResume::RETURN, RETURN_ROUTE, RETURN_STEPS, "RETURNTOB")) return false;
  Serial.println("[RETURNTOB] returnToPointB() finished all moves");
  return true;
}
