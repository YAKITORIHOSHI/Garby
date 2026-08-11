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
    safeMoveDistance(100, true, false);
    if (resetQueued) return false;
    responsiveDelay(2);
  }
  return routePause();
}

void runStart() {
  Serial.println("[RUNSTART] Entered runStart()");   // confirms this function is actually reached

  // Request initial LiDAR/sensor status
  requestStatus();
  if (!routePause()) return;

  // Short calibration bumps — nudge disabled
  if (!runCalibrationBumps()) return;

  safeMoveDistance(8000, false, false);
  if (!routePause()) return;
  safeTurnLeft(4900);
  if (!routePause()) return;

  if (!runCalibrationBumps()) return;

  safeMoveDistance(595500, false, true);
  if (!routePause()) return;
  safeTurnLeft(5000);
  if (!routePause()) return;

  if (!runCalibrationBumps()) return;

  safeMoveDistance(6000, false, false);
  if (!routePause()) return;
  safeTurnRight(4900);
  if (!routePause()) return;

  if (!runCalibrationBumps()) return;

  safeMoveDistance(25500, false, false);
  if (!routePause()) return;
  safeTurnRight(4900);
  if (!routePause()) return;

  if (!runCalibrationBumps()) return;

  safeMoveDistance(10600, false, false);
  if (!routePause()) return;
  safeTurnLeft(9800);

  Serial.println("[RUNSTART] runStart() finished all moves");
}

void returnToPointB() {
  Serial.println("[RETURNTOB] Entered returnToPointB()");

  // Request initial status for the return trip
  requestStatus();
  if (!routePause()) return;

  if (!runCalibrationBumps()) return;

  safeMoveDistance(9500, false, false);
  if (!routePause()) return;
  safeTurnLeft(4900);
  if (!routePause()) return;

  if (!runCalibrationBumps()) return;

  safeMoveDistance(25500, false, false);
  if (!routePause()) return;
  safeTurnLeft(4900);
  if (!routePause()) return;

  if (!runCalibrationBumps()) return;

  safeMoveDistance(6000, false, false);
  if (!routePause()) return;
  safeTurnRight(4900);
  if (!routePause()) return;

  if (!runCalibrationBumps()) return;
  safeMoveDistance(595700, false, true);
  if (!routePause()) return;
  safeTurnRight(4900);
  if (!routePause()) return;

  if (!runCalibrationBumps()) return;

  safeMoveDistance(7000, false, false);
  if (!routePause()) return;
  safeTurnLeft(9800);
  if (!routePause()) return;

  Serial.println("[RETURNTOB] returnToPointB() finished all moves");
} 
