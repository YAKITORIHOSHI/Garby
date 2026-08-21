// .ino (Extension)
// ============================
// RUN DRAFT END-TO-END
// ============================

// safeMoveDistance(steps, isADJ, isNudgeEnabled)
//   isNudgeEnabled=false on the short calibration bumps so they do not
//   consume steering commands intended for the following hallway segment.

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

bool runStart() {
  Serial.println("[RUNSTART] Entered runStart()");   // confirms this function is actually reached

  // Request initial LiDAR/sensor status
  requestStatus();
  if (!routePause()) return false;

  // Short calibration bumps — nudge disabled
  if (!runCalibrationBumps()) return false;

  if (!safeMoveDistance(8000, false, false)) return false;
  if (!routePause()) return false;
  if (!safeTurnLeft(4900)) return false;
  if (!routePause()) return false;

  if (!runCalibrationBumps()) return false;

  if (!safeMoveDistance(595500, false, true)) return false;
  if (!routePause()) return false;
  if (!safeTurnLeft(5000)) return false;
  if (!routePause()) return false;

  if (!runCalibrationBumps()) return false;

  if (!safeMoveDistance(6000, false, false)) return false;
  if (!routePause()) return false;
  if (!safeTurnRight(4900)) return false;
  if (!routePause()) return false;

  if (!runCalibrationBumps()) return false;

  if (!safeMoveDistance(25500, false, false)) return false;
  if (!routePause()) return false;
  if (!safeTurnRight(4900)) return false;
  if (!routePause()) return false;

  if (!runCalibrationBumps()) return false;

  if (!safeMoveDistance(10600, false, false)) return false;
  if (!routePause()) return false;
  if (!safeTurnLeft(9800)) return false;

  Serial.println("[RUNSTART] runStart() finished all moves");
  return true;
}

bool returnToPointB() {
  Serial.println("[RETURNTOB] Entered returnToPointB()");

  // Request initial status for the return trip
  requestStatus();
  if (!routePause()) return false;

  if (!runCalibrationBumps()) return false;

  if (!safeMoveDistance(9500, false, false)) return false;
  if (!routePause()) return false;
  if (!safeTurnLeft(4900)) return false;
  if (!routePause()) return false;

  if (!runCalibrationBumps()) return false;

  if (!safeMoveDistance(25500, false, false)) return false;
  if (!routePause()) return false;
  if (!safeTurnLeft(4900)) return false;
  if (!routePause()) return false;

  if (!runCalibrationBumps()) return false;

  if (!safeMoveDistance(6000, false, false)) return false;
  if (!routePause()) return false;
  if (!safeTurnRight(4900)) return false;
  if (!routePause()) return false;

  if (!runCalibrationBumps()) return false;
  if (!safeMoveDistance(595700, false, true)) return false;
  if (!routePause()) return false;
  if (!safeTurnRight(4900)) return false;
  if (!routePause()) return false;

  if (!runCalibrationBumps()) return false;

  if (!safeMoveDistance(7000, false, false)) return false;
  if (!routePause()) return false;
  if (!safeTurnLeft(9800)) return false;
  if (!routePause()) return false;

  Serial.println("[RETURNTOB] returnToPointB() finished all moves");
  return true;
} 
