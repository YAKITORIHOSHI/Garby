#include "NAPHTALI_CODE_V2.h"

// ============================================================
// GLOBAL OBJECT DEFINITIONS
// ============================================================
HX711             scale;
HardwareSerial    ESP_Serial(1);
HardwareSerial    Air780(2);
Servo             servo;
FastAccelStepperEngine engine   = FastAccelStepperEngine();
FastAccelStepper*      stepper1 = NULL;
FastAccelStepper*      stepper2 = NULL;

// ============================================================
// STATIC / MISC VARS
// ============================================================
uint32_t lastPrintMs  = 0;

// ============================================================
// GLOBAL STATE DEFINITIONS
// ============================================================
LidarZones zones;
bool          lidarBlockedActive   = false;
bool          lidarControlled      = false;
unsigned long lidarBlockedStart    = 0;
unsigned long lidarLastPeriodicSMS = 0;
unsigned long lidarLastLRScan      = 0;
int           blockedCount         = 0;
bool          idleMode             = false;
bool          lastConnected        = false;
bool          movingForward        = false;
bool          blockedSMSSent       = false;
bool          loadcellSMSSent      = false;
bool          buzzerState          = false;
unsigned long lastBeepTime         = 0;
float frontDistance = 0.0f;
float leftDistance  = 0.0f;
float rightDistance = 0.0f;
bool isTrashbinFull = false;
// Fail closed: motion is not allowed until fresh GO packets are confirmed.
bool shouldStop     = true;
bool sensorTripped  = false;
bool pathCommandSeen = false;
bool linkFaultActive = true;
unsigned long lastPathCommandMs = 0;
unsigned long lastSensorPacketMs = 0;
uint8_t clearPathCommandCount = 0;

ParseData     path;
ReceivedDatas data;

GarbyState garbyState  = GarbyState::IDLE;
bool       resetQueued = false;

// ── Non-blocking nudge ───────────────────────────────────────
NudgeDir      activeNudge     = NudgeDir::NONE;
unsigned long nudgeStartMs    = 0;
unsigned long nudgeDurationMs = 0;
unsigned long nudgeHoldStartMs = 0;
unsigned long nudgeSettleUntilMs = 0;  // block new nudges until this timestamp

unsigned long lastIdlePrintMs = 0;
#define IDLE_PRINT_INTERVAL_MS   5000UL

static unsigned long lastBridgeRxMs = 0;
static unsigned long lastStatusRequestMs = 0;
static unsigned long lastReadyRecoveryMs = 0;
static uint32_t motionBaseSpeed = MAX_SPEED;

static portMUX_TYPE ultrasonicMux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool ultrasonicAwaitingEcho = false;
static volatile bool ultrasonicEchoRiseSeen = false;
static volatile bool ultrasonicEchoReady = false;
static volatile uint32_t ultrasonicEchoRiseUs = 0;
static volatile uint32_t ultrasonicEchoWidthUs = 0;
static bool ultrasonicPingInFlight = false;
static uint32_t ultrasonicPingStartedUs = 0;
static unsigned long lastUltrasonicPingMs = 0;
static unsigned long lastFrontSampleMs = 0;
static uint32_t frontSampleSequence = 0;
static float latestFrontRawCm = 999.0f;
static float frontFilterWindow[3] = {999.0f, 999.0f, 999.0f};
static uint8_t frontFilterIndex = 0;
static uint8_t frontFilterCount = 0;
static uint8_t ultrasonicBlockedCount = 0;
static uint8_t ultrasonicClearCount = 0;
static bool frontObstacleLatched = false;
static bool frontEmergencyActive = false;

// Defined with the ultrasonic/servo implementation below. These declarations
// let every motor loop service safety sensing without blocking on pulseIn().
static void commandServoTracked(int angle);
static bool servoSettled();
static void setStraightBaseSpeed(uint32_t requestedSpeed);
static void cancelActiveNudge(bool addSettleTime);

static volatile bool smsWorkerBusy = false;

static void smsWorkerTask(void* arg) {
  String* message = static_cast<String*>(arg);
  if (message != nullptr) {
    sendSMS(CONTACT_NUMBER, *message);
    delete message;
  }
  smsWorkerBusy = false;
  vTaskDelete(nullptr);
}

void queueSMSAlert(const String& message) {
  if (smsWorkerBusy) return;
  String* copy = new String(message);
  if (copy == nullptr) return;
  smsWorkerBusy = true;
  if (xTaskCreate(smsWorkerTask, "garby-sms", 6144, copy, 1, nullptr) != pdPASS) {
    smsWorkerBusy = false;
    delete copy;
  }
}

bool smsAlertBusy() {
  return smsWorkerBusy;
}

// ============================================================
// NEW: requestStatus() – sends request to RasPi via BLE bridge
// ============================================================
void requestStatus() {
  const unsigned long now = millis();
  if (lastStatusRequestMs != 0 &&
      now - lastStatusRequestMs < REQUEST_STATUS_MIN_INTERVAL_MS) {
    return;
  }
  lastStatusRequestMs = now;
  ESP_Serial.println("[REQUEST-STATUS]");
  static unsigned long lastRequestLogMs = 0;
  if (now - lastRequestLogMs >= 5000UL) {
    lastRequestLogMs = now;
    Serial.println("[REQ] Requesting fresh path status");
  }
}

void serviceBridgeRecovery() {
  const unsigned long now = millis();
  const bool rxStale = lastBridgeRxMs == 0 ||
                       now - lastBridgeRxMs > BRIDGE_RX_STALE_MS;
  if (!rxStale) return;

  lastConnected = false;
  if (lastReadyRecoveryMs != 0 &&
      now - lastReadyRecoveryMs < BRIDGE_RECOVERY_INTERVAL_MS) {
    return;
  }
  lastReadyRecoveryMs = now;
  ESP_Serial.println("[MCU READY]");
}

void responsiveDelay(unsigned long durationMs) {
  const unsigned long started = millis();
  while (millis() - started < durationMs) {
    pollESP();
    serviceUltrasonic();
    serviceBridgeRecovery();
    enforcePathWatchdog();
    updateNudge();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// ============================================================
// parseNudgeCmd — parse the BLE bridge's pre-computed command
// ============================================================
// The BLE bridge now does the SIDES: parsing, error math, dead-zone
// check, and front-suppression scaling, and sends us a tiny pre-
// digested "N:<ms>|<NUDGE_LEFT|NUDGE_RIGHT|STABLE>" line. This routine
// just cracks it open and stores the result. moveToTarget() consumes
// it through consumeNudgeCmd() and applies the nudge directly — no
// float math, no per-field indexOf hunt, no clamp logic in the
// controller's hot loop anymore.
//
// Returns true if `buf` was a well-formed N: command we recognised.
bool ParseData::parseNudgeCmd(const String& buf) {
  String s = buf;
  s.trim();
  if (!s.startsWith("N:")) return false;

  // "N:<ms>|<dir>" or "N:<ms>:<intensity>|<dir>"
  int colonEnd = 2;                                  // just after "N:"
  int pipe      = s.indexOf('|', colonEnd);
  if (pipe < 0) return false;

  String numStr = s.substring(colonEnd, pipe);
  String dirStr = s.substring(pipe + 1);
  numStr.trim();
  dirStr.trim();

  unsigned long ms = 0;
  unsigned int intensity = 0;

  int subColon = numStr.indexOf(':');
  if (subColon >= 0) {
    const long parsedMs = numStr.substring(0, subColon).toInt();
    const long parsedIntensity = numStr.substring(subColon + 1).toInt();
    if (parsedMs < 0 || parsedIntensity < 0) return false;
    ms = (unsigned long)parsedMs;
    intensity = (unsigned int)parsedIntensity;
  } else {
    const long parsedMs = numStr.toInt();
    if (parsedMs < 0) return false;
    ms = (unsigned long)parsedMs;
    intensity = 0;
  }

  NudgeCmdDir dir = NudgeCmdDir::STABLE;
  if      (dirStr == "NUDGE_LEFT")  dir = NudgeCmdDir::NUDGE_LEFT;
  else if (dirStr == "NUDGE_RIGHT") dir = NudgeCmdDir::NUDGE_RIGHT;
  else if (dirStr == "STABLE")      dir = NudgeCmdDir::STABLE;
  else                              return false;

  if (ms > NUDGE_MAX_HOLD_MS) ms = NUDGE_MAX_HOLD_MS;
  if (intensity > NUDGE_MAX_CUT_PCT) intensity = NUDGE_MAX_CUT_PCT;
  if (dir == NudgeCmdDir::STABLE) {
    ms = 0;
    intensity = 0;
  }
  cmdMs        = ms;
  cmdIntensity = intensity;
  cmdDir       = dir;
  cmdReceivedMs = millis();
  newCmd       = true;
  return true;
}

void ParseData::printAllState() {
  const char* dirStr = "STABLE";
  if (cmdDir == NudgeCmdDir::NUDGE_LEFT)  dirStr = "NUDGE_LEFT";
  if (cmdDir == NudgeCmdDir::NUDGE_RIGHT) dirStr = "NUDGE_RIGHT";
  Serial.println("=== ParseData ===");
  Serial.printf("  NudgeCmd: %s  ms=%lu  intensity=%u  newCmd=%d\n",
                dirStr, cmdMs, cmdIntensity, (int)newCmd);
}

// ============================================================
// PARSE DATA — ReceivedDatas (unchanged)
// ============================================================
void ReceivedDatas::setUS(int val) {
  us.value = val;
  if      (val < 20) us.status = UltrasonicStatus::EMPTY;
  else if (val < 40) us.status = UltrasonicStatus::HALFWAY;
  else               us.status = UltrasonicStatus::FULL;
}
void ReceivedDatas::setMQ4(int val) {
  mq4.value = val;
  if      (val < 400) mq4.status = MQ4Status::NORMAL;
  else if (val < 700) mq4.status = MQ4Status::WARNING;
  else                mq4.status = MQ4Status::DANGER;
}
void ReceivedDatas::setMQ135(int val) {
  mq135.value = val;
  if      (val < 300) mq135.status = MQ135Status::CLEAN;
  else if (val < 500) mq135.status = MQ135Status::MODERATE;
  else if (val < 700) mq135.status = MQ135Status::POOR;
  else                mq135.status = MQ135Status::VERY_POOR;
}
void ReceivedDatas::setMQ137(int val) {
  mq137.value = val;
  if      (val < 400) mq137.status = MQ137Status::NORMAL;
  else if (val < 700) mq137.status = MQ137Status::WARNING;
  else                mq137.status = MQ137Status::DANGER;
}
bool ReceivedDatas::parse(const String& raw) {
  if (!raw.startsWith("SENSOR:")) return false;
  String body = raw.substring(7);
  body.trim();
  int segStart = 0;
  while (segStart < (int)body.length()) {
    int    pipe = body.indexOf('|', segStart);
    String seg  = (pipe == -1) ? body.substring(segStart)
                               : body.substring(segStart, pipe);
    seg.trim();
    segStart = (pipe == -1) ? (int)body.length() : pipe + 1;
    int eq = seg.indexOf('=');
    if (eq == -1) return false;
    String key = seg.substring(0, eq);
    int    val = seg.substring(eq + 1).toInt();
    key.trim();
    if      (key == "US")    setUS(val);
    else if (key == "MQ4")   setMQ4(val);
    else if (key == "MQ135") setMQ135(val);
    else if (key == "MQ137") setMQ137(val);
  }
  return true;
}
int ReceivedDatas::getUSValue()    { return us.value;    }
int ReceivedDatas::getMQ4Value()   { return mq4.value;   }
int ReceivedDatas::getMQ135Value() { return mq135.value; }
int ReceivedDatas::getMQ137Value() { return mq137.value; }
ReceivedDatas::UltrasonicStatus ReceivedDatas::getUSStatus()    { return us.status;    }
ReceivedDatas::MQ4Status        ReceivedDatas::getMQ4Status()   { return mq4.status;   }
ReceivedDatas::MQ135Status      ReceivedDatas::getMQ135Status() { return mq135.status; }
ReceivedDatas::MQ137Status      ReceivedDatas::getMQ137Status() { return mq137.status; }
void ReceivedDatas::printAll() {
  Serial.println("=== ReceivedDatas ===");
  Serial.printf("  US:    val=%d  status=%d\n", us.value,    (int)us.status);
  Serial.printf("  MQ4:   val=%d  status=%d\n", mq4.value,   (int)mq4.status);
  Serial.printf("  MQ135: val=%d  status=%d\n", mq135.value, (int)mq135.status);
  Serial.printf("  MQ137: val=%d  status=%d\n", mq137.value, (int)mq137.status);
}

bool sensorReadDecisionMaker(ReceivedDatas& d) {
  if (d.getMQ4Status()   == ReceivedDatas::MQ4Status::DANGER)      return true;
  if (d.getMQ135Status() == ReceivedDatas::MQ135Status::VERY_POOR) return true;
  if (d.getMQ137Status() == ReceivedDatas::MQ137Status::DANGER)     return true;
  int secondHighCount = 0;
  if (d.getMQ4Status()   == ReceivedDatas::MQ4Status::WARNING)      secondHighCount++;
  if (d.getMQ135Status() == ReceivedDatas::MQ135Status::POOR)       secondHighCount++;
  if (d.getMQ137Status() == ReceivedDatas::MQ137Status::WARNING)    secondHighCount++;
  return (secondHighCount >= 3);
}

// ============================================================
// STATE MACHINE HELPERS
// ============================================================

// ── pollESP ──────────────────────────────────────────────────
// Reads all pending messages from ESP_Serial in non-blocking fashion,
// updates global path/sensor/reset state, and sends ACK_MSG back.
static String uartRxBuffer = "";
static String lastStopReason = "";
static bool uartRxReserved = false;
static bool uartDiscardUntilNewline = false;

static void noteBridgeRx() {
  lastBridgeRxMs = millis();
  lastConnected = true;
}

bool pathCommandFresh() {
  return pathCommandSeen &&
         (millis() - lastPathCommandMs <= PATH_COMMAND_TIMEOUT_MS);
}

void enforcePathWatchdog() {
  if (garbyState == GarbyState::IDLE) return;
  if (pathCommandFresh()) return;
  if (!shouldStop || !linkFaultActive) {
    Serial.println("[SAFETY] Path command stale/missing -- latching STOP");
  }
  shouldStop = true;
  linkFaultActive = true;
  clearPathCommandCount = 0;
  path.reset();
}

static void processESPMessage(const String& espMsg) {
    if (espMsg == "[BLE CONNECTION ESTABLISHED]") {
      noteBridgeRx();
      shouldStop = true;
      linkFaultActive = true;
      pathCommandSeen = false;
      clearPathCommandCount = 0;
      requestStatus();
      return;
    }

    if (espMsg == "[RESET]") {
      noteBridgeRx();
      // The Pi deliberately retries RESET until it receives [IDLE]. Once the
      // return route is already active, that retry is an idempotent keepalive,
      // not a request to abort the route and report a false completion.
      if (garbyState == GarbyState::RETURNING) {
        ESP_Serial.println(ACK_MSG);
        return;
      }
      shouldStop = true;
      linkFaultActive = true;
      clearPathCommandCount = 0;
      path.reset();
      if (garbyState == GarbyState::RUNNING) {
        resetQueued = true;
        Serial.println("[RESET] Explicit return/reset queued");
      } else {
        fullReset();
      }
      ESP_Serial.println(ACK_MSG);
      return;
    }

    if (espMsg.startsWith("STOP")) {
      noteBridgeRx();
      const bool changed = !shouldStop || espMsg != lastStopReason;
      shouldStop = true;
      pathCommandSeen = true;
      lastPathCommandMs = millis();
      clearPathCommandCount = 0;
      linkFaultActive = espMsg.indexOf("LINK") >= 0 ||
                        espMsg.indexOf("STALE") >= 0 ||
                        espMsg.indexOf("WAITING") >= 0 ||
                        espMsg.indexOf("PROTOCOL") >= 0;
      path.reset();
      activeNudge = NudgeDir::NONE;
      nudgeHoldStartMs = 0;
      lastStopReason = espMsg;
      if (changed) Serial.println("[STOP] " + espMsg);
      ESP_Serial.println(ACK_MSG);
      return;
    }

    if (espMsg == "GO") {
      noteBridgeRx();
      pathCommandSeen = true;
      lastPathCommandMs = millis();
      if (clearPathCommandCount < MCU_GO_CONFIRM_PACKETS) {
        clearPathCommandCount++;
      }
      if (clearPathCommandCount >= MCU_GO_CONFIRM_PACKETS) {
        if (shouldStop) Serial.println("[GO] Fresh path confirmed -- motion enabled");
        shouldStop = false;
        linkFaultActive = false;
        lastStopReason = "";
      }
      ESP_Serial.println(ACK_MSG);
      return;
    }

    if (espMsg.startsWith("N:")) {
      noteBridgeRx();
      bool ok = false;
      if (!shouldStop && pathCommandFresh()) {
        ok = path.parseNudgeCmd(espMsg);
      } else {
        path.reset();
      }
      if (!ok && !shouldStop) {
        Serial.println("[WARN] Bad or unsafe N: command: " + espMsg);
      }
      ESP_Serial.println(ACK_MSG);
      return;
    }

    if (espMsg.startsWith("SENSOR:")) {
      noteBridgeRx();
      bool ok = data.parse(espMsg);
      if (ok) {
        lastSensorPacketMs = millis();
        sensorTripped = sensorReadDecisionMaker(data);
      }
      else Serial.println("[WARN] Bad SENSOR message: " + espMsg);
      ESP_Serial.println(ACK_MSG);
      return;
    }

    static unsigned long lastIgnoredLogMs = 0;
    if (millis() - lastIgnoredLogMs >= 2000UL) {
      lastIgnoredLogMs = millis();
      Serial.println("[ESP] Ignored: " + espMsg);
    }
}

void pollESP() {
  if (!uartRxReserved) {
    uartRxBuffer.reserve(UART_RX_LINE_MAX + 1U);
    lastStopReason.reserve(96);
    uartRxReserved = true;
  }

  uint16_t byteBudget = UART_RX_BUDGET_BYTES;
  while (byteBudget-- > 0 && ESP_Serial.available()) {
    const char c = (char)ESP_Serial.read();
    if (c == '\r') continue;

    if (uartDiscardUntilNewline) {
      if (c == '\n') {
        uartDiscardUntilNewline = false;
        static unsigned long lastOverflowLogMs = 0;
        if (millis() - lastOverflowLogMs >= 2000UL) {
          lastOverflowLogMs = millis();
          Serial.println("[WARN] Oversized ESP UART line discarded");
        }
      }
      continue;
    }

    if (c != '\n') {
      if (uartRxBuffer.length() < UART_RX_LINE_MAX) {
        uartRxBuffer += c;
      } else {
        uartRxBuffer.remove(0);
        uartDiscardUntilNewline = true;
      }
      continue;
    }

    uartRxBuffer.trim();
    if (uartRxBuffer.length() > 0) processESPMessage(uartRxBuffer);
    // remove(0) retains the reserved allocation and prevents long-run heap churn.
    uartRxBuffer.remove(0);
  }
}

void haltAndWait(const String& reason) {
  activeBrakeStopMotors();
  movingForward = false;
  path.reset();
  commandServoTracked(DEFAULT_VIEW);

  if (!blockedSMSSent) {
    queueSMSAlert("[GARBY] Blocked: " + reason);
    blockedSMSSent = true;
  }

  unsigned long lastSMS = millis();
  unsigned long lastBeep = millis();
  unsigned long lastRequestTime = 0;
  unsigned long lastLocalCheck = 0;
  uint8_t clearanceCount = 0;
  float localDist = 0.0f;
  unsigned long evaluatedPathCommandMs = 0;
  const bool requiresLocal = reason.indexOf("HUMAN") >= 0 ||
                             reason.indexOf("SONIC") >= 0;

  Serial.println("[HALT] Waiting for confirmed clearance: " + reason);
  while (true) {
    const unsigned long now = millis();
    if (now - lastRequestTime >= 100UL) {
      requestStatus();
      lastRequestTime = now;
    }

    pollESP();
    serviceUltrasonic();
    serviceBridgeRecovery();
    enforcePathWatchdog();

    if (resetQueued) {
      blockedSMSSent = false;
      return;
    }

    bool newClearanceEvidence = false;
    bool localClear = true;
    if (requiresLocal && now - lastLocalCheck >= 100UL) {
      lastLocalCheck = now;
      localDist = latestFrontDistance();
      localClear = frontDistanceFresh() &&
                   !frontObstacleDetected() &&
                   localDist > ULTRASONIC_CLEAR_DISTANCE_CM;
      newClearanceEvidence = true;
    } else if (requiresLocal) {
      localClear = frontDistanceFresh() &&
                   !frontObstacleDetected() &&
                   localDist > ULTRASONIC_CLEAR_DISTANCE_CM;
    } else if (lastPathCommandMs != evaluatedPathCommandMs) {
      evaluatedPathCommandMs = lastPathCommandMs;
      newClearanceEvidence = true;
    }

    if (newClearanceEvidence) {
      const bool lidarClear = pathCommandFresh() && !shouldStop;
      const bool allClear = requiresLocal ? (lidarClear && localClear) : lidarClear;
      if (allClear) {
        if (clearanceCount < ULTRASONIC_CLEAR_CONFIRM_SAMPLES) clearanceCount++;
      } else {
        clearanceCount = 0;
      }
    }

    if (clearanceCount >= ULTRASONIC_CLEAR_CONFIRM_SAMPLES) {
      Serial.println("[HALT] Clearance confirmed -- resuming");
      blockedSMSSent = false;
      return;
    }

    if (now - lastBeep >= 5000UL) {
      lastBeep = now;
      xTaskCreate(buzzerTask, "garby-beep", 1536, nullptr, 1, nullptr);
    }
    if (now - lastSMS >= 30000UL) {
      lastSMS = now;
      queueSMSAlert("[GARBY] Still blocked: " + reason);
    }

    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// ── movementGate ───────────────────────────────────────────────
void movementGate(bool isNudgeEnabled) {
  (void)isNudgeEnabled;
  commandServoTracked(DEFAULT_VIEW);
  requestStatus();
  const unsigned long start = millis();
  do {
    pollESP();
    serviceUltrasonic();
    serviceBridgeRecovery();
    enforcePathWatchdog();
    if (!shouldStop && pathCommandFresh()) {
      // Give the centered sonar one bounded opportunity to refresh before
      // moving. LiDAR remains the fail-closed authority if sonar has no echo.
      const unsigned long sampleWaitStarted = millis();
      while (!frontDistanceFresh() &&
             millis() - sampleWaitStarted < ULTRASONIC_SAMPLE_MAX_AGE_MS) {
        pollESP();
        serviceUltrasonic();
        enforcePathWatchdog();
        if (shouldStop || resetQueued) break;
        vTaskDelay(pdMS_TO_TICKS(2));
      }
      if (frontObstacleDetected()) {
        haltAndWait("SONIC:BLOCKED before motion");
      }
      return;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  } while (millis() - start < 400UL);

  haltAndWait(linkFaultActive ? "LINK:WAITING_FOR_FRESH_PATH"
                              : "PATH:BLOCKED from LiDAR");
}

void safeMoveDistance(int32_t steps, bool isADJ, bool isNudgeEnabled) {
  movementGate(isNudgeEnabled);
  if (resetQueued || shouldStop || !pathCommandFresh()) return;
  moveDistance(steps, isADJ, isNudgeEnabled);
}

void safeTurnLeft(int32_t step) {
  movementGate(false);
  if (!resetQueued && !shouldStop) turnLeft(step);
}

void safeTurnRight(int32_t step) {
  movementGate(false);
  if (!resetQueued && !shouldStop) turnRight(step);
}

void fullReset() {
  shouldStop = true;
  pathCommandSeen = false;
  linkFaultActive = true;
  lastPathCommandMs = 0;
  lastSensorPacketMs = 0;
  clearPathCommandCount = 0;
  sensorTripped = false;
  movingForward = false;
  blockedSMSSent = false;
  loadcellSMSSent = false;
  resetQueued = false;
  path.reset();
  activeNudge = NudgeDir::NONE;
  nudgeHoldStartMs = 0;
  nudgeSettleUntilMs = 0;
  garbyState = GarbyState::IDLE;
  Serial.println("[RESET] State cleared -- IDLE and fail-closed");
  ESP_Serial.println("[IDLE]");
  queueSMSAlert("[GARBY] Returned to base. Ready for next cycle.");
}

// ── printIdleUptime ───────────────────────────────────────────
void printIdleUptime() {
  unsigned long now = millis();
  if (now - lastIdlePrintMs < IDLE_PRINT_INTERVAL_MS) return;
  lastIdlePrintMs = now;
  unsigned long uptimeSec = now / 1000UL;
  unsigned long h = uptimeSec / 3600;
  unsigned long m = (uptimeSec % 3600) / 60;
  unsigned long s = uptimeSec % 60;
  Serial.printf("[IDLE] Up %02lu:%02lu:%02lu — waiting for trigger...\n", h, m, s);
}

// ============================================================
// MOTOR MOVEMENT FUNCTIONS
// ============================================================
static bool motorsRunning() {
  return stepper1 && stepper2 && (stepper1->isRunning() || stepper2->isRunning());
}

static void clearNudgeState() {
  activeNudge = NudgeDir::NONE;
  nudgeStartMs = 0;
  nudgeDurationMs = 0;
  nudgeHoldStartMs = 0;
  nudgeSettleUntilMs = millis() + NUDGE_SETTLE_MS;
}

static void controlledStopMotors(uint32_t deceleration,
                                 unsigned long timeoutMs) {
  if (!stepper1 || !stepper2) return;
  stepper1->setAcceleration(deceleration);
  stepper2->setAcceleration(deceleration);
  stepper1->applySpeedAcceleration();
  stepper2->applySpeedAcceleration();
  stepper1->stopMove();
  stepper2->stopMove();

  const unsigned long started = millis();
  while (motorsRunning() && millis() - started < timeoutMs) {
    pollESP();
    serviceUltrasonic();
    delay(1);
  }
  if (motorsRunning()) {
    // Last-resort hard stop only if the controlled ramp did not complete.
    stepper1->forceStopAndNewPosition(stepper1->getCurrentPosition());
    stepper2->forceStopAndNewPosition(stepper2->getCurrentPosition());
  }
  movingForward = false;
  clearNudgeState();
}

void turnRight(int32_t step) {
  if (!stepper1 || !stepper2) return;
  commandServoTracked(DEFAULT_VIEW);
  const long target1 = stepper1->getCurrentPosition() - step;
  const long target2 = stepper2->getCurrentPosition() + step;
  stepper1->setSpeedInHz(TURN_SPEED);
  stepper1->setAcceleration(ACCELERATION);
  stepper2->setSpeedInHz(TURN_SPEED);
  stepper2->setAcceleration(ACCELERATION);
  stepper1->moveTo(target1);
  stepper2->moveTo(target2);
  while (motorsRunning()) {
    pollESP();
    serviceUltrasonic();
    serviceBridgeRecovery();
    enforcePathWatchdog();
    if (resetQueued || shouldStop) {
      activeBrakeStopMotors();
      if (resetQueued) break;
      haltAndWait("PATH:BLOCKED during turn");
      if (resetQueued) break;
      stepper1->setSpeedInHz(TURN_SPEED);
      stepper2->setSpeedInHz(TURN_SPEED);
      stepper1->setAcceleration(ACCELERATION);
      stepper2->setAcceleration(ACCELERATION);
      stepper1->moveTo(target1);
      stepper2->moveTo(target2);
      continue;
    }
    if (frontObstacleDetected()) {
      activeBrakeStopMotors();
      haltAndWait("SONIC:BLOCKED during turn");
      if (resetQueued) break;
      stepper1->setSpeedInHz(TURN_SPEED);
      stepper2->setSpeedInHz(TURN_SPEED);
      stepper1->setAcceleration(ACCELERATION);
      stepper2->setAcceleration(ACCELERATION);
      stepper1->moveTo(target1);
      stepper2->moveTo(target2);
    }
    delay(1);
  }
}

void turnLeft(int32_t step) {
  if (!stepper1 || !stepper2) return;
  commandServoTracked(DEFAULT_VIEW);
  const long target1 = stepper1->getCurrentPosition() + step;
  const long target2 = stepper2->getCurrentPosition() - step;
  stepper1->setSpeedInHz(TURN_SPEED);
  stepper1->setAcceleration(ACCELERATION);
  stepper2->setSpeedInHz(TURN_SPEED);
  stepper2->setAcceleration(ACCELERATION);
  stepper1->moveTo(target1);
  stepper2->moveTo(target2);
  while (motorsRunning()) {
    pollESP();
    serviceUltrasonic();
    serviceBridgeRecovery();
    enforcePathWatchdog();
    if (resetQueued || shouldStop) {
      activeBrakeStopMotors();
      if (resetQueued) break;
      haltAndWait("PATH:BLOCKED during turn");
      if (resetQueued) break;
      stepper1->setSpeedInHz(TURN_SPEED);
      stepper2->setSpeedInHz(TURN_SPEED);
      stepper1->setAcceleration(ACCELERATION);
      stepper2->setAcceleration(ACCELERATION);
      stepper1->moveTo(target1);
      stepper2->moveTo(target2);
      continue;
    }
    if (frontObstacleDetected()) {
      activeBrakeStopMotors();
      haltAndWait("SONIC:BLOCKED during turn");
      if (resetQueued) break;
      stepper1->setSpeedInHz(TURN_SPEED);
      stepper2->setSpeedInHz(TURN_SPEED);
      stepper1->setAcceleration(ACCELERATION);
      stepper2->setAcceleration(ACCELERATION);
      stepper1->moveTo(target1);
      stepper2->moveTo(target2);
    }
    delay(1);
  }
}

void emergencyStopMotors() {
  if (stepper1) stepper1->forceStopAndNewPosition(stepper1->getCurrentPosition());
  if (stepper2) stepper2->forceStopAndNewPosition(stepper2->getCurrentPosition());
  movingForward = false;
  clearNudgeState();
}

void smoothDecelStopMotors() {
  controlledStopMotors(GENTLE_STOP_DECEL, 1800UL);
}

void activeBrakeStopMotors() {
  // Fast enough for a 95 cm LiDAR stop margin, but without the previous
  // counter-torque reverse pulse that could jerk or spill the payload.
  controlledStopMotors(SAFETY_STOP_DECEL, 650UL);
}

void startStraight() {
  enforcePathWatchdog();
  if (!stepper1 || !stepper2 || shouldStop || !pathCommandFresh()) return;
  motionBaseSpeed = MAX_SPEED;
  const uint32_t motor1Speed = (uint32_t)(motionBaseSpeed *
    (1.0f + constrain(MOTOR1_TRIM_PCT, -10.0f, 10.0f) / 100.0f));
  const uint32_t motor2Speed = (uint32_t)(motionBaseSpeed *
    (1.0f + constrain(MOTOR2_TRIM_PCT, -10.0f, 10.0f) / 100.0f));
  stepper1->setSpeedInHz(motor1Speed);
  stepper1->setAcceleration(ACCELERATION);
  stepper2->setSpeedInHz(motor2Speed);
  stepper2->setAcceleration(ACCELERATION);
  stepper1->runForward();
  stepper2->runForward();
  movingForward = true;
}

static void startTargetMove(long target1, long target2) {
  enforcePathWatchdog();
  if (!stepper1 || !stepper2 || shouldStop || !pathCommandFresh()) return;
  motionBaseSpeed = MAX_SPEED;
  const uint32_t motor1Speed = (uint32_t)(motionBaseSpeed *
    (1.0f + constrain(MOTOR1_TRIM_PCT, -10.0f, 10.0f) / 100.0f));
  const uint32_t motor2Speed = (uint32_t)(motionBaseSpeed *
    (1.0f + constrain(MOTOR2_TRIM_PCT, -10.0f, 10.0f) / 100.0f));
  stepper1->setSpeedInHz(motor1Speed);
  stepper1->setAcceleration(ACCELERATION);
  stepper2->setSpeedInHz(motor2Speed);
  stepper2->setAcceleration(ACCELERATION);
  stepper1->moveTo(target1);
  stepper2->moveTo(target2);
  movingForward = true;
}

void restoreStraight() {
  setStraightBaseSpeed(MAX_SPEED);
}

static uint32_t trimmedMotorSpeed(uint32_t baseSpeed, float trimPct) {
  trimPct = constrain(trimPct, -10.0f, 10.0f);
  return (uint32_t)((float)baseSpeed * (1.0f + trimPct / 100.0f));
}

static void restoreWheelSpeeds() {
  if (!stepper1 || !stepper2 || shouldStop) return;
  stepper1->setSpeedInHz(trimmedMotorSpeed(motionBaseSpeed, MOTOR1_TRIM_PCT));
  stepper1->setAcceleration(ACCELERATION);
  stepper1->applySpeedAcceleration();
  stepper2->setSpeedInHz(trimmedMotorSpeed(motionBaseSpeed, MOTOR2_TRIM_PCT));
  stepper2->setAcceleration(ACCELERATION);
  stepper2->applySpeedAcceleration();
}

static void cancelActiveNudge(bool addSettleTime) {
  if (activeNudge != NudgeDir::NONE) restoreWheelSpeeds();
  activeNudge = NudgeDir::NONE;
  nudgeStartMs = 0;
  nudgeDurationMs = 0;
  nudgeHoldStartMs = 0;
  if (addSettleTime) nudgeSettleUntilMs = millis() + NUDGE_SETTLE_MS;
}

static void setStraightBaseSpeed(uint32_t requestedSpeed) {
  if (!stepper1 || !stepper2 || shouldStop) return;
  requestedSpeed = constrain(requestedSpeed, CAUTION_SPEED, MAX_SPEED);
  if (motionBaseSpeed == requestedSpeed) return;

  // A speed-zone change is safety-critical; cancel a steering tap before
  // applying the same new base to both wheels.
  if (activeNudge != NudgeDir::NONE) cancelActiveNudge(true);
  motionBaseSpeed = requestedSpeed;
  restoreWheelSpeeds();
}

static float nudgeCutFraction(float delayMs, unsigned int intensityPct,
                              float directionBoost = 1.0f) {
  // Older bridges omit intensity. Their proportional duration still carries
  // useful error magnitude, so map it to a gentle execution level.
  if (intensityPct == 0) {
    const float ratio = constrain(delayMs / (float)NUDGE_MAX_HOLD_MS, 0.0f, 1.0f);
    intensityPct = (unsigned int)(NUDGE_MIN_CUT_PCT + ratio * 13.0f);
  }
  intensityPct = constrain(intensityPct, NUDGE_MIN_CUT_PCT, NUDGE_MAX_CUT_PCT);
  float cut = ((float)intensityPct / 100.0f) * directionBoost;
  return constrain(cut,
                   (float)NUDGE_MIN_CUT_PCT / 100.0f,
                   (float)NUDGE_MAX_CUT_PCT / 100.0f);
}

// ── nudgeLeftContinuous ───────────────────────────────────────
// Small, one-shot correction: slows the left wheel briefly so the
// robot drifts left, then auto-reverts via updateNudge(). Every fresh
// SIDES packet that's still off-center simply calls this again.
void nudgeLeftContinuous(float delayMs, unsigned int intensityPct) {
  if (shouldStop || !pathCommandFresh()) return;
  if (activeNudge == NudgeDir::NONE &&
      (long)(millis() - nudgeSettleUntilMs) < 0) return;
  if (activeNudge == NudgeDir::LEFT) return;  // never extend a tap indefinitely
  if (activeNudge == NudgeDir::RIGHT) {
    restoreWheelSpeeds();
    clearNudgeState();
    return;
  }

  const float cutPct = nudgeCutFraction(delayMs, intensityPct,
                                         NUDGE_LEFT_BOOST_FACTOR);
  const uint32_t leftBase = trimmedMotorSpeed(motionBaseSpeed, MOTOR1_TRIM_PCT);
  uint32_t targetSpeed = (uint32_t)(leftBase * (1.0f - cutPct));

  // Apply a bounded acceleration so each correction is a smooth tap
  stepper1->setSpeedInHz(targetSpeed);
  stepper1->setAcceleration(NUDGE_ACCELERATION);
  stepper1->applySpeedAcceleration();
  static unsigned long lastLogMs = 0;
  if (millis() - lastLogMs >= 500UL) {
    lastLogMs = millis();
    Serial.printf("<-- nudge left (dur=%.0f ms, cut=%.1f%%)\n", delayMs, cutPct * 100.0f);
  }
  activeNudge      = NudgeDir::LEFT;
  nudgeStartMs     = millis();
  nudgeDurationMs  = (unsigned long)delayMs;
  nudgeHoldStartMs = millis();
}

// ── nudgeRightContinuous ──────────────────────────────────────
void nudgeRightContinuous(float delayMs, unsigned int intensityPct) {
  if (shouldStop || !pathCommandFresh()) return;
  if (activeNudge == NudgeDir::NONE &&
      (long)(millis() - nudgeSettleUntilMs) < 0) return;
  if (activeNudge == NudgeDir::RIGHT) return;
  if (activeNudge == NudgeDir::LEFT) {
    restoreWheelSpeeds();
    clearNudgeState();
    return;
  }

  const float cutPct = nudgeCutFraction(delayMs, intensityPct);
  const uint32_t rightBase = trimmedMotorSpeed(motionBaseSpeed, MOTOR2_TRIM_PCT);
  uint32_t targetSpeed = (uint32_t)(rightBase * (1.0f - cutPct));

  // Apply a bounded acceleration so each correction is a smooth tap
  stepper2->setSpeedInHz(targetSpeed);
  stepper2->setAcceleration(NUDGE_ACCELERATION);
  stepper2->applySpeedAcceleration();
  static unsigned long lastLogMs = 0;
  if (millis() - lastLogMs >= 500UL) {
    lastLogMs = millis();
    Serial.printf("--> nudge right (dur=%.0f ms, cut=%.1f%%)\n", delayMs, cutPct * 100.0f);
  }
  activeNudge      = NudgeDir::RIGHT;
  nudgeStartMs     = millis();
  nudgeDurationMs  = (unsigned long)delayMs;
  nudgeHoldStartMs = millis();
}


// ── updateNudge ───────────────────────────────────────────────
void updateNudge() {
  if (shouldStop) {
    activeNudge = NudgeDir::NONE;
    return;
  }
  if (activeNudge == NudgeDir::NONE) return;
  const unsigned long allowedDuration = min(nudgeDurationMs, NUDGE_MAX_HOLD_MS);
  if (millis() - nudgeStartMs >= allowedDuration) {
    restoreWheelSpeeds();   // always restore BOTH wheels, never just one
    activeNudge        = NudgeDir::NONE;
    nudgeHoldStartMs   = 0;
    nudgeSettleUntilMs = millis() + NUDGE_SETTLE_MS;  // block new nudges until chassis settles
  }
}

// ── moveToTarget ──────────────────────────────────────────────
static int servoCommandAngle = DEFAULT_VIEW;
static unsigned long servoReadyAtMs = 0;

static void commandServoTracked(int angle) {
  if (angle < SCAN_RIGHT) angle = SCAN_RIGHT;
  if (angle > SCAN_LEFT) angle = SCAN_LEFT;
  const int delta = abs(angle - servoCommandAngle);
  servo.write(angle);
  servoCommandAngle = angle;
  unsigned long settle = (unsigned long)delta * SERVO_MS_PER_DEG;
  if (settle < SERVO_SETTLE_MIN_MS) settle = SERVO_SETTLE_MIN_MS;
  servoReadyAtMs = millis() + settle;
}

static bool servoSettled() {
  return (long)(millis() - servoReadyAtMs) >= 0;
}

static void IRAM_ATTR ultrasonicEchoISR() {
  if (!ultrasonicAwaitingEcho) return;
  const uint32_t nowUs = micros();
  const bool levelHigh = digitalRead(ECHO_PIN) == HIGH;

  portENTER_CRITICAL_ISR(&ultrasonicMux);
  if (levelHigh) {
    ultrasonicEchoRiseUs = nowUs;
    ultrasonicEchoRiseSeen = true;
  } else if (ultrasonicEchoRiseSeen) {
    ultrasonicEchoWidthUs = nowUs - ultrasonicEchoRiseUs;
    ultrasonicEchoReady = true;
    ultrasonicEchoRiseSeen = false;
    ultrasonicAwaitingEcho = false;
  }
  portEXIT_CRITICAL_ISR(&ultrasonicMux);
}

static float medianOfThree(float a, float b, float c) {
  if (a > b) { const float t = a; a = b; b = t; }
  if (b > c) { const float t = b; b = c; c = t; }
  if (a > b) { const float t = a; a = b; b = t; }
  return b;
}

static void recordFrontSample(float distanceCm) {
  if (!isfinite(distanceCm) || distanceCm < 1.5f || distanceCm > 999.0f) {
    distanceCm = 999.0f;
  }

  latestFrontRawCm = distanceCm;
  frontFilterWindow[frontFilterIndex] = distanceCm;
  frontFilterIndex = (frontFilterIndex + 1U) % 3U;
  if (frontFilterCount < 3U) frontFilterCount++;

  if (frontFilterCount == 1U) {
    frontDistance = distanceCm;
  } else if (frontFilterCount == 2U) {
    frontDistance = min(frontFilterWindow[0], frontFilterWindow[1]);
  } else {
    frontDistance = medianOfThree(frontFilterWindow[0],
                                  frontFilterWindow[1],
                                  frontFilterWindow[2]);
  }
  lastFrontSampleMs = millis();
  frontSampleSequence++;

  if (distanceCm <= VERY_CLOSE_DISTANCE_CM) {
    frontEmergencyActive = true;
    frontObstacleLatched = true;
    ultrasonicBlockedCount = ULTRASONIC_BLOCK_CONFIRM_SAMPLES;
    ultrasonicClearCount = 0;
    return;
  }

  if (distanceCm > VERY_CLOSE_DISTANCE_CM + 10.0f) {
    frontEmergencyActive = false;
  }

  if (distanceCm <= ULTRASONIC_STOP_DISTANCE_CM) {
    if (ultrasonicBlockedCount < 255U) ultrasonicBlockedCount++;
    ultrasonicClearCount = 0;
    if (ultrasonicBlockedCount >= ULTRASONIC_BLOCK_CONFIRM_SAMPLES) {
      frontObstacleLatched = true;
    }
  } else {
    ultrasonicBlockedCount = 0;
    if (distanceCm >= ULTRASONIC_CLEAR_DISTANCE_CM) {
      if (ultrasonicClearCount < 255U) ultrasonicClearCount++;
      if (ultrasonicClearCount >= ULTRASONIC_CLEAR_CONFIRM_SAMPLES) {
        frontObstacleLatched = false;
        frontEmergencyActive = false;
      }
    } else {
      ultrasonicClearCount = 0;
    }
  }
}

static void startUltrasonicPing() {
  portENTER_CRITICAL(&ultrasonicMux);
  ultrasonicEchoRiseSeen = false;
  ultrasonicEchoReady = false;
  ultrasonicAwaitingEcho = true;
  portEXIT_CRITICAL(&ultrasonicMux);

  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  ultrasonicPingStartedUs = micros();
  ultrasonicPingInFlight = true;
  lastUltrasonicPingMs = millis();
}

void initUltrasonicMonitor() {
  digitalWrite(TRIG_PIN, LOW);
  portENTER_CRITICAL(&ultrasonicMux);
  ultrasonicAwaitingEcho = false;
  ultrasonicEchoRiseSeen = false;
  ultrasonicEchoReady = false;
  portEXIT_CRITICAL(&ultrasonicMux);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), ultrasonicEchoISR, CHANGE);
  commandServoTracked(DEFAULT_VIEW);
}

void serviceUltrasonic() {
  bool echoReady = false;
  uint32_t echoWidthUs = 0;
  portENTER_CRITICAL(&ultrasonicMux);
  if (ultrasonicEchoReady) {
    echoReady = true;
    echoWidthUs = ultrasonicEchoWidthUs;
    ultrasonicEchoReady = false;
  }
  portEXIT_CRITICAL(&ultrasonicMux);

  if (echoReady) {
    ultrasonicPingInFlight = false;
    const float distanceCm = (float)echoWidthUs * 0.0343f * 0.5f;
    recordFrontSample(distanceCm);
  } else if (ultrasonicPingInFlight &&
             (uint32_t)(micros() - ultrasonicPingStartedUs) >= ULTRASONIC_TIMEOUT_US) {
    portENTER_CRITICAL(&ultrasonicMux);
    ultrasonicAwaitingEcho = false;
    ultrasonicEchoRiseSeen = false;
    ultrasonicEchoReady = false;
    portEXIT_CRITICAL(&ultrasonicMux);
    ultrasonicPingInFlight = false;
    recordFrontSample(999.0f);
  }

  const bool centered = abs(servoCommandAngle - DEFAULT_VIEW) <=
                        SERVO_CENTER_CONE_DEG;
  if (!ultrasonicPingInFlight && centered && servoSettled() &&
      millis() - lastUltrasonicPingMs >= ULTRASONIC_PING_INTERVAL_MS) {
    startUltrasonicPing();
  }
}

bool frontObstacleDetected() {
  // Once confirmed, remain fail-closed until confirmed clear samples arrive.
  return frontObstacleLatched;
}

bool frontEmergencyObstacleDetected() {
  return frontEmergencyActive;
}

bool frontDistanceFresh() {
  return lastFrontSampleMs != 0 &&
         millis() - lastFrontSampleMs <= ULTRASONIC_SAMPLE_MAX_AGE_MS;
}

float latestFrontDistance() {
  return frontDistanceFresh() ? frontDistance : 999.0f;
}

void moveToTarget(long target1, long target2, bool isADJ, bool isNudgeEnabled) {
  if (!stepper1 || !stepper2) return;
  commandServoTracked(DEFAULT_VIEW);

  if (isADJ) {
    stepper1->setSpeedInHz(MAX_SPEED);
    stepper1->setAcceleration(ACCELERATION);
    stepper2->setSpeedInHz(MAX_SPEED);
    stepper2->setAcceleration(ACCELERATION);
    stepper1->moveTo(target1);
    stepper2->moveTo(target2);
    while (motorsRunning()) {
      pollESP();
      serviceUltrasonic();
      serviceBridgeRecovery();
      enforcePathWatchdog();
      if (resetQueued || shouldStop) {
        activeBrakeStopMotors();
        if (resetQueued) break;
        haltAndWait("PATH:BLOCKED during adjustment");
        if (resetQueued) break;
        startTargetMove(target1, target2);
        continue;
      }
      if (frontObstacleDetected()) {
        activeBrakeStopMotors();
        haltAndWait("SONIC:BLOCKED during adjustment");
        if (resetQueued) break;
        startTargetMove(target1, target2);
      }
      delay(1);
    }
    movingForward = false;
    return;
  }

  startTargetMove(target1, target2);
  if (!movingForward) return;
  Serial.println("[MOVE] Targeted straight run started");

  unsigned long lastRequestTime = 0;

  while (motorsRunning()) {
    const unsigned long now = millis();

    if (now - lastRequestTime >= 100UL) {
      requestStatus();
      lastRequestTime = now;
    }

    pollESP();
    serviceUltrasonic();
    serviceBridgeRecovery();
    enforcePathWatchdog();

    if (resetQueued) {
      smoothDecelStopMotors();
      Serial.println("[MOVE] Explicit reset -- controlled stop");
      break;
    }

    if (shouldStop) {
      activeBrakeStopMotors();
      const float localDist = latestFrontDistance();
      const String reason = (frontObstacleDetected() ||
                             localDist <= ULTRASONIC_STOP_DISTANCE_CM)
        ? "SONIC:BLOCKED (centered ultrasonic obstacle)"
        : (linkFaultActive ? "LINK:STALE" : "PATH:BLOCKED from LiDAR");
      haltAndWait(reason);
      if (resetQueued) break;
      startTargetMove(target1, target2);
      commandServoTracked(DEFAULT_VIEW);
      lastRequestTime = millis();
      continue;
    }

    ParseData::NudgeCmdDir cmdDir = ParseData::NudgeCmdDir::STABLE;
    unsigned long cmdMs = 0;
    unsigned int cmdIntensity = 0;
    if (isNudgeEnabled && path.consumeNudgeCmd(cmdDir, cmdMs, cmdIntensity)) {
      if (cmdDir == ParseData::NudgeCmdDir::STABLE) {
        cancelActiveNudge(false);
      } else if (cmdMs > 0) {
        if (cmdDir == ParseData::NudgeCmdDir::NUDGE_LEFT)
          nudgeLeftContinuous((float)cmdMs, cmdIntensity);
        else if (cmdDir == ParseData::NudgeCmdDir::NUDGE_RIGHT)
          nudgeRightContinuous((float)cmdMs, cmdIntensity);
      }
    } else if (!isNudgeEnabled) {
      path.reset();
    }
    updateNudge();

    if (frontDistanceFresh()) {
      const float filteredDistance = latestFrontDistance();
      if (filteredDistance <= ULTRASONIC_SLOW_DISTANCE_CM) {
        setStraightBaseSpeed(CAUTION_SPEED);
      } else if (filteredDistance >= ULTRASONIC_RESUME_SPEED_DISTANCE_CM) {
        setStraightBaseSpeed(MAX_SPEED);
      }
    }

    if (frontObstacleDetected()) {
      activeBrakeStopMotors();
      xTaskCreate(buzzerTask, "garby-obstacle", 1536, nullptr, 2, nullptr);
      haltAndWait("SONIC:BLOCKED in center path");
      if (resetQueued) break;
      startTargetMove(target1, target2);
      commandServoTracked(DEFAULT_VIEW);
      continue;
    }

    delay(1);
  }

  if (motorsRunning()) smoothDecelStopMotors();
  else clearNudgeState();
  movingForward = false;
  commandServoTracked(DEFAULT_VIEW);
}

void moveDistance(int32_t steps, bool isADJ, bool isNudgeEnabled) {
  if (!stepper1 || !stepper2) return;
  long t1 = stepper1->getCurrentPosition() + steps;
  long t2 = stepper2->getCurrentPosition() + steps;
  moveToTarget(t1, t2, isADJ, isNudgeEnabled);
}

// ============================================================
// SERVO + ULTRASONIC
// ============================================================
static void cancelUltrasonicPing() {
  portENTER_CRITICAL(&ultrasonicMux);
  ultrasonicAwaitingEcho = false;
  ultrasonicEchoRiseSeen = false;
  ultrasonicEchoReady = false;
  portEXIT_CRITICAL(&ultrasonicMux);
  ultrasonicPingInFlight = false;
}

float readDistanceRaw() {
  // Synchronous compatibility helper for stopped/manual scans only. Motion
  // safety uses serviceUltrasonic() and never blocks on pulseIn().
  cancelUltrasonicPing();
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  const unsigned long duration = pulseIn(ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);
  if (duration == 0) return 999.0f;
  return (float)duration * 0.0343f / 2.0f;
}

float getDistance() {
  commandServoTracked(DEFAULT_VIEW);
  const uint32_t startingSequence = frontSampleSequence;
  const unsigned long started = millis();
  while ((frontSampleSequence == startingSequence || !frontDistanceFresh()) &&
         millis() - started < 350UL) {
    pollESP();
    serviceUltrasonic();
    serviceBridgeRecovery();
    vTaskDelay(pdMS_TO_TICKS(1));
  }
  return latestFrontDistance();
}

float scanAngle(int angle) {
  cancelUltrasonicPing();
  commandServoTracked(angle);
  const unsigned long deadline = millis() + 350UL;
  while (!servoSettled() && (long)(deadline - millis()) > 0) {
    pollESP();
    delay(1);
  }
  const float distance = readDistanceRaw();
  commandServoTracked(DEFAULT_VIEW);
  return distance;
}

// ============================================================
// SMS / AIR780E
// ============================================================
String sendAT(const String& cmd, unsigned long timeout) {
  while (Air780.available()) Air780.read();
  Air780.println(cmd);
  Serial.print(">> " + cmd + " ");
  String response = "";
  unsigned long start = millis();
  while (millis() - start < timeout) {
    while (Air780.available()) {
      char c = (char)Air780.read();
      if ((c >= 32 && c <= 126) || c == '\r' || c == '\n' || c == '\t') {
        response += c;
      }
    }
    if (response.indexOf("OK") != -1 || response.indexOf("ERROR") != -1) break;
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  String cleanResp = response;
  cleanResp.trim();
  Serial.println("<< " + cleanResp);
  return response;
}
void powerOnAir780() {
  Serial.println("[Air780E] Checking if alive...");
  pinMode(PWRKEY_PIN, OUTPUT);
  digitalWrite(PWRKEY_PIN, HIGH);
  for (int i = 0; i < 5; i++) {
    String r = sendAT("AT", 1000);
    if (r.indexOf("OK") != -1) {
      Serial.println("[Air780E] Already on.");
      return;
    }
    delay(500);
  }
  Serial.println("[Air780E] Sending power-on pulse...");
  digitalWrite(PWRKEY_PIN, LOW);
  delay(1500);
  digitalWrite(PWRKEY_PIN, HIGH);
  Serial.println("[Air780E] Waiting for boot...");
  delay(8000);
  Serial.println("[Air780E] Power-on done.");
}
bool waitForModule(int maxAttempts) {
  for (int i = 0; i < maxAttempts; i++) {
    String r = sendAT("AT", 2000);
    if (r.indexOf("OK") != -1) {
      Serial.println("[Air780E] Module responsive!");
      return true;
    }
    Serial.printf("[Air780E] Attempt %d/%d\n", i + 1, maxAttempts);
    delay(1000);
  }
  Serial.println("[Air780E] ERROR: No response.");
  return false;
}
bool waitForNetwork(int maxAttempts) {
  for (int i = 0; i < maxAttempts; i++) {
    String resp = sendAT("AT+CREG?", 3000);
    if (resp.indexOf(",1") != -1 || resp.indexOf(",5") != -1) {
      Serial.println("[Air780E] Registered!");
      return true;
    }
    Serial.printf("[Air780E] Network attempt %d/%d\n", i + 1, maxAttempts);
    delay(2000);
  }
  Serial.println("[Air780E] WARNING: Not registered.");
  return false;
}
void sendSMS(const String& phoneNumber, const String& message) {
  Serial.println("[SMS] Sending to " + phoneNumber);
  String resp = sendAT("AT+CMGF=1", 3000);
  if (resp.indexOf("OK") == -1) { Serial.println("[SMS] Text mode failed."); return; }
  while (Air780.available()) Air780.read();
  Air780.println("AT+CMGS=\"" + phoneNumber + "\"");
  String prompt = "";
  unsigned long start = millis();
  while (millis() - start < 5000) {
    while (Air780.available()) {
      char c = (char)Air780.read();
      if ((c >= 32 && c <= 126) || c == '\r' || c == '\n' || c == '>') prompt += c;
    }
    if (prompt.indexOf(">") != -1) break;
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  if (prompt.indexOf(">") == -1) {
    Serial.println("[SMS] No '>' prompt. Aborting.");
    Air780.write(0x1B);
    return;
  }
  Air780.print(message);
  Air780.write(0x1A);
  String result = "";
  start = millis();
  while (millis() - start < 15000) {
    while (Air780.available()) {
      char c = (char)Air780.read();
      if ((c >= 32 && c <= 126) || c == '\r' || c == '\n') result += c;
    }
    if (result.indexOf("+CMGS") != -1 || result.indexOf("ERROR") != -1) break;
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  if (result.indexOf("+CMGS") != -1) Serial.println("[SMS] Sent!");
  else                                Serial.println("[SMS] Failed: " + result);
}



// ============================================================
// SENSOR DECISION HANDLERS (unchanged)
// ============================================================
bool handlePath() {
  if (shouldStop) { Serial.println("[STOP] Path blocked"); return true; }
  return false;
}
bool handleSensor() {
  if (sensorTripped) { Serial.println("[ALERT] Gas danger!"); return true; }
  switch (data.getUSStatus()) {
    case ReceivedDatas::UltrasonicStatus::FULL:
      Serial.println("[BIN] Full — return to base");
      return true;
    case ReceivedDatas::UltrasonicStatus::HALFWAY:
      Serial.println("[BIN] Halfway");
      break;
    default: break;
  }
  return false;
}

// ============================================================
// UTILITY (unchanged)
// ============================================================
void flushESPSerial() {
  while (ESP_Serial.available()) ESP_Serial.read();
}
void buzzerTask(void *pvParameters) {
  (void)pvParameters;
  if (buzzerState) {
    vTaskDelete(NULL);
    return;
  }
  buzzerState = true;
  digitalWrite(BUZZER_PIN, HIGH);
  vTaskDelay(pdMS_TO_TICKS(500));
  digitalWrite(BUZZER_PIN, LOW);
  vTaskDelay(pdMS_TO_TICKS(100));
  buzzerState = false;
  vTaskDelete(NULL);
}
bool checkLoad(float threshold) {
  if (!scale.is_ready()) return false;
  float w = scale.get_units(1) - 0.010f;
  return (w >= threshold);
}
