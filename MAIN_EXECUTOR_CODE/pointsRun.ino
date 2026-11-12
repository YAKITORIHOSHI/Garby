// .ino (Extension)
// ============================
// RUN DRAFT END-TO-END
// ============================

// safeMoveDistance(steps, isADJ, isNudgeEnabled)
//   Every straight longer than FINISH_STEPS holds the MPU route heading.
//   isNudgeEnabled=true also fuses the LiDAR walls on that leg: it steers to the
//   measured hallway direction and centre instead of the gyro heading alone.
// safeTurnLeft/Right(degrees)
//   MPU-measured spins, absolute along the route (2026-11-11: the old
//   4900/5000-step turns are 90 degrees, the 9800-step turn is 180). Each turn
//   is then checked against the LiDAR walls and corrected to the hallway.
// runLongHallway(steps)  (2026-11-12)
//   The long hallway runs: stand still and let the LiDAR walls put the robot on
//   the centre line and axis first (centerBeforeLongRun), then drive the straight
//   with wall fusion. Step counts and turn angles are unchanged.

static bool routePause(unsigned long durationMs = 100UL) {
  responsiveDelay(durationMs);
  return !resetQueued;
}

static bool runCalibrationBumps() {
  for (int i = 0; i < 5; i++) {
    if (!safeMoveDistance(100, true, false)) return false;
    responsiveDelay(2);
  }
  return routePause();
}

// The sidesteps' forward travel along the hallway is taken off the straight, so
// the run still ends where its calibrated step count says.
static bool runLongHallway(int32_t steps) {
  int32_t progress = 0;
  if (!centerBeforeLongRun(&progress)) return false;
  if (progress < 0 || progress > steps / 4) progress = 0;
  return safeMoveDistance(steps - progress, false, true);
}

bool runStart() {
  Serial.println("[RUNSTART] Entered runStart()");   // confirms this function is actually reached

  // Request initial LiDAR/sensor status
  requestStatus();
  if (!routePause()) return false;
  if (!beginRouteHeading()) return false;  // no MPU heading: do not move

  // Short calibration bumps — nudge disabled
  if (!runCalibrationBumps()) return false;

  if (!safeMoveDistance(8000, false, false)) return false;
  if (!routePause()) return false;
  if (!safeTurnLeft(90)) return false;
  if (!routePause()) return false;

  if (!runCalibrationBumps()) return false;

  if (!runLongHallway(595500)) return false;
  if (!routePause()) return false;
  if (!safeTurnLeft(90)) return false;
  if (!routePause()) return false;

  if (!runCalibrationBumps()) return false;

  if (!safeMoveDistance(6000, false, false)) return false;
  if (!routePause()) return false;
  if (!safeTurnRight(90)) return false;
  if (!routePause()) return false;

  if (!runCalibrationBumps()) return false;

  if (!safeMoveDistance(25500, false, false)) return false;
  if (!routePause()) return false;
  if (!safeTurnRight(90)) return false;
  if (!routePause()) return false;

  if (!runCalibrationBumps()) return false;

  if (!safeMoveDistance(10600, false, false)) return false;
  if (!routePause()) return false;
  if (!safeTurnLeft(180)) return false;

  Serial.println("[RUNSTART] runStart() finished all moves");
  return true;
}

bool returnToPointB() {
  Serial.println("[RETURNTOB] Entered returnToPointB()");

  // Request initial status for the return trip
  requestStatus();
  if (!routePause()) return false;
  if (!beginRouteHeading()) return false;  // no MPU heading: do not move

  if (!runCalibrationBumps()) return false;

  if (!safeMoveDistance(9500, false, false)) return false;
  if (!routePause()) return false;
  if (!safeTurnLeft(90)) return false;
  if (!routePause()) return false;

  if (!runCalibrationBumps()) return false;

  if (!safeMoveDistance(25500, false, false)) return false;
  if (!routePause()) return false;
  if (!safeTurnLeft(90)) return false;
  if (!routePause()) return false;

  if (!runCalibrationBumps()) return false;

  if (!safeMoveDistance(6000, false, false)) return false;
  if (!routePause()) return false;
  if (!safeTurnRight(90)) return false;
  if (!routePause()) return false;

  if (!runCalibrationBumps()) return false;
  if (!runLongHallway(595700)) return false;
  if (!routePause()) return false;
  if (!safeTurnRight(90)) return false;
  if (!routePause()) return false;

  if (!runCalibrationBumps()) return false;

  if (!safeMoveDistance(7000, false, false)) return false;
  if (!routePause()) return false;
  if (!safeTurnLeft(180)) return false;
  if (!routePause()) return false;

  Serial.println("[RETURNTOB] returnToPointB() finished all moves");
  return true;
}
