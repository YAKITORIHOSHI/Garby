#include "NAPHTALI_CODE_V2.h"

// ============================================================
// GLOBAL OBJECT DEFINITIONS
// ============================================================
HX711             scale;
HardwareSerial    ESP_Serial(1);
HardwareSerial    Air780(2);
Servo             servo;
FastAccelStepperEngine engine   = FastAccelStepperEngine();
FastAccelStepper*      stepper1 = nullptr;
FastAccelStepper*      stepper2 = nullptr;

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
static bool latestFrontSampleValid = false;
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
static volatile bool modemServiceBusy = false;
static volatile bool modemReadyFlag = false;

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
  // SMS is operational telemetry only. Never let an unavailable modem delay
  // motion safety, BLE/UART handshakes, or the main state machine.
  if (!modemReadyFlag || modemServiceBusy || smsWorkerBusy) return;
  String* copy = new String(message);
  if (copy == nullptr) return;
  smsWorkerBusy = true;
  if (xTaskCreate(smsWorkerTask, "garby-sms", 6144, copy, 1, nullptr) != pdPASS) {
    smsWorkerBusy = false;
    delete copy;
  }
}

bool smsAlertBusy() {
  return smsWorkerBusy || modemServiceBusy;
}

bool modemReady() {
  return modemReadyFlag;
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
static bool parseUnsignedToken(const String& token, unsigned long maxValue,
                               unsigned long& out) {
  if (token.length() == 0) return false;
  unsigned long value = 0;
  for (size_t i = 0; i < token.length(); ++i) {
    const char c = token.charAt(i);
    if (c < '0' || c > '9') return false;
    const unsigned long digit = (unsigned long)(c - '0');
    if (value > (maxValue - digit) / 10UL) return false;
    value = value * 10UL + digit;
  }
  out = value;
  return true;
}

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
  unsigned long parsedMs = 0;
  unsigned long parsedIntensity = 0;
  if (subColon >= 0) {
    if (numStr.indexOf(':', subColon + 1) >= 0) return false;
    if (!parseUnsignedToken(numStr.substring(0, subColon),
                            NUDGE_MAX_HOLD_MS, parsedMs)) return false;
    if (!parseUnsignedToken(numStr.substring(subColon + 1),
                            100UL, parsedIntensity)) return false;
    ms = parsedMs;
    intensity = (unsigned int)parsedIntensity;
  } else {
    if (!parseUnsignedToken(numStr, NUDGE_MAX_HOLD_MS, parsedMs)) return false;
    ms = parsedMs;
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
  if      (val == 999) us.status = UltrasonicStatus::UNAVAILABLE;
  else if (val < 20)   us.status = UltrasonicStatus::EMPTY;
  else if (val < 40)   us.status = UltrasonicStatus::HALFWAY;
  else                 us.status = UltrasonicStatus::FULL;
}
void ReceivedDatas::setMQ4(int val) {
  mq4.value = val;
  if      (val < 0)   mq4.status = MQ4Status::UNAVAILABLE;
  else if (val < 400) mq4.status = MQ4Status::NORMAL;
  else if (val < 700) mq4.status = MQ4Status::WARNING;
  else                mq4.status = MQ4Status::DANGER;
}
void ReceivedDatas::setMQ135(int val) {
  mq135.value = val;
  if      (val < 0)   mq135.status = MQ135Status::UNAVAILABLE;
  else if (val < 300) mq135.status = MQ135Status::CLEAN;
  else if (val < 500) mq135.status = MQ135Status::MODERATE;
  else if (val < 700) mq135.status = MQ135Status::POOR;
  else                mq135.status = MQ135Status::VERY_POOR;
}
void ReceivedDatas::setMQ137(int val) {
  mq137.value = val;
  if      (val < 0)   mq137.status = MQ137Status::UNAVAILABLE;
  else if (val < 400) mq137.status = MQ137Status::NORMAL;
  else if (val < 700) mq137.status = MQ137Status::WARNING;
  else                mq137.status = MQ137Status::DANGER;
}
static bool parseSensorNumber(const String& text,
                              float minimum,
                              float maximum,
                              int& out) {
  String value = text;
  value.trim();
  if (value.length() == 0) return false;

  bool hasDigit = false;
  for (unsigned int i = 0; i < value.length(); ++i) {
    const char c = value.charAt(i);
    if (isDigit(c)) {
      hasDigit = true;
      continue;
    }
    if (c != '+' && c != '-' && c != '.' && c != 'e' && c != 'E') {
      return false;
    }
  }
  if (!hasDigit) return false;

  char* end = nullptr;
  const char* start = value.c_str();
  const float parsed = strtof(start, &end);
  if (end == start || end == nullptr || *end != '\0' || !isfinite(parsed) ||
      parsed < minimum || parsed > maximum) {
    return false;
  }
  out = (int)lroundf(parsed);
  return true;
}

bool ReceivedDatas::parse(const String& raw) {
  if (!raw.startsWith("SENSOR:")) return false;
  String body = raw.substring(7);
  body.trim();
  if (body.length() == 0) return false;

  int nextUS = 0, nextMQ4 = 0, nextMQ135 = 0, nextMQ137 = 0;
  bool seenUS = false, seenMQ4 = false, seenMQ135 = false, seenMQ137 = false;
  int segStart = 0;
  while (segStart < (int)body.length()) {
    int pipe = body.indexOf('|', segStart);
    String seg = (pipe == -1) ? body.substring(segStart)
                              : body.substring(segStart, pipe);
    seg.trim();
    segStart = (pipe == -1) ? (int)body.length() : pipe + 1;
    if (seg.length() == 0) return false;

    const int eq = seg.indexOf('=');
    if (eq <= 0 || seg.indexOf('=', eq + 1) != -1) return false;
    String key = seg.substring(0, eq);
    String value = seg.substring(eq + 1);
    key.trim();

    if (key == "US") {
      if (seenUS || !parseSensorNumber(value, 0.0f, 999.0f, nextUS)) return false;
      seenUS = true;
    } else if (key == "MQ4") {
      if (seenMQ4 || !parseSensorNumber(value, -1.0f, 100000.0f, nextMQ4)) return false;
      seenMQ4 = true;
    } else if (key == "MQ135") {
      if (seenMQ135 || !parseSensorNumber(value, -1.0f, 100000.0f, nextMQ135)) return false;
      seenMQ135 = true;
    } else if (key == "MQ137") {
      if (seenMQ137 || !parseSensorNumber(value, -1.0f, 100000.0f, nextMQ137)) return false;
      seenMQ137 = true;
    } else {
      return false;
    }
  }

  // Commit atomically only after the complete telemetry packet validates.
  if (!seenUS || !seenMQ4 || !seenMQ135 || !seenMQ137) return false;
  setUS(nextUS);
  setMQ4(nextMQ4);
  setMQ135(nextMQ135);
  setMQ137(nextMQ137);
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
  const unsigned long start = millis();
  do {
    // Re-request while waiting. requestStatus() is internally rate-limited to
    // 80 ms, allowing the bridge/Pi to satisfy both clearance-confirmation
    // layers without relying on the slower periodic status cadence.
    requestStatus();
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
  } while (millis() - start < MOTION_GATE_TIMEOUT_MS);

  haltAndWait(linkFaultActive ? "LINK:WAITING_FOR_FRESH_PATH"
                              : "PATH:BLOCKED from LiDAR");
}

bool safeMoveDistance(int32_t steps, bool isADJ, bool isNudgeEnabled) {
  movementGate(isNudgeEnabled);
  if (resetQueued || shouldStop || !pathCommandFresh()) return false;
  return moveDistance(steps, isADJ, isNudgeEnabled);
}

bool safeTurnLeft(int32_t step) {
  movementGate(false);
  if (resetQueued || shouldStop || !pathCommandFresh()) return false;
  return turnLeft(step);
}

bool safeTurnRight(int32_t step) {
  movementGate(false);
  if (resetQueued || shouldStop || !pathCommandFresh()) return false;
  return turnRight(step);
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

bool turnRight(int32_t step) {
  if (!stepper1 || !stepper2 || shouldStop || !pathCommandFresh()) return false;
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
      if (resetQueued) return false;
      haltAndWait("PATH:BLOCKED during turn");
      if (resetQueued) return false;
      if (shouldStop || !pathCommandFresh()) return false;
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
      if (resetQueued) return false;
      if (shouldStop || !pathCommandFresh()) return false;
      stepper1->setSpeedInHz(TURN_SPEED);
      stepper2->setSpeedInHz(TURN_SPEED);
      stepper1->setAcceleration(ACCELERATION);
      stepper2->setAcceleration(ACCELERATION);
      stepper1->moveTo(target1);
      stepper2->moveTo(target2);
    }
    delay(1);
  }
  movingForward = false;
  return stepper1->getCurrentPosition() == target1 &&
         stepper2->getCurrentPosition() == target2;
}

bool turnLeft(int32_t step) {
  if (!stepper1 || !stepper2 || shouldStop || !pathCommandFresh()) return false;
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
      if (resetQueued) return false;
      haltAndWait("PATH:BLOCKED during turn");
      if (resetQueued) return false;
      if (shouldStop || !pathCommandFresh()) return false;
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
      if (resetQueued) return false;
      if (shouldStop || !pathCommandFresh()) return false;
      stepper1->setSpeedInHz(TURN_SPEED);
      stepper2->setSpeedInHz(TURN_SPEED);
      stepper1->setAcceleration(ACCELERATION);
      stepper2->setAcceleration(ACCELERATION);
      stepper1->moveTo(target1);
      stepper2->moveTo(target2);
    }
    delay(1);
  }
  movingForward = false;
  return stepper1->getCurrentPosition() == target1 &&
         stepper2->getCurrentPosition() == target2;
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

static bool startTargetMove(long target1, long target2) {
  enforcePathWatchdog();
  if (!stepper1 || !stepper2 || shouldStop || !pathCommandFresh()) return false;
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
  const bool targetAlreadyReached =
    stepper1->getCurrentPosition() == target1 &&
    stepper2->getCurrentPosition() == target2;
  if (!targetAlreadyReached && !motorsRunning()) {
    movingForward = false;
    Serial.println("[MOTOR] Target command was not accepted");
    return false;
  }
  return true;
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
    // A confirmed reversal is already filtered by the bridge hysteresis and
    // cooldown. Restore the base speeds, then apply the new direction now;
    // dropping the reversal here made the robot ignore the correction and
    // look permanently stuck near a wall.
    restoreWheelSpeeds();
    activeNudge = NudgeDir::NONE;
    nudgeStartMs = 0;
    nudgeDurationMs = 0;
    nudgeHoldStartMs = 0;
    nudgeSettleUntilMs = millis();
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
    // See the left-nudge reversal path above. Do not consume a valid opposite
    // correction without executing it.
    restoreWheelSpeeds();
    activeNudge = NudgeDir::NONE;
    nudgeStartMs = 0;
    nudgeDurationMs = 0;
    nudgeHoldStartMs = 0;
    nudgeSettleUntilMs = millis();
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
    cancelActiveNudge(false);
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

static void recordFrontSample(float distanceCm, bool validEcho = true) {
  const bool valid = validEcho && isfinite(distanceCm) &&
                     distanceCm >= 1.5f && distanceCm <= 999.0f;
  lastFrontSampleMs = millis();
  frontSampleSequence++;
  latestFrontSampleValid = valid;

  if (!valid) {
    // No echo is UNKNOWN, not a clear reading. Preserve any existing obstacle
    // latch/counters so sensor silence can never release a prior safety STOP.
    latestFrontRawCm = 999.0f;
    return;
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
    recordFrontSample(999.0f, false);
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
  return latestFrontSampleValid && lastFrontSampleMs != 0 &&
         millis() - lastFrontSampleMs <= ULTRASONIC_SAMPLE_MAX_AGE_MS;
}

float latestFrontDistance() {
  return frontDistanceFresh() ? frontDistance : 999.0f;
}

bool moveToTarget(long target1, long target2, bool isADJ, bool isNudgeEnabled) {
  if (!stepper1 || !stepper2 || shouldStop || !pathCommandFresh()) return false;
  commandServoTracked(DEFAULT_VIEW);

  if (isADJ) {
    stepper1->setSpeedInHz(MAX_SPEED);
    stepper1->setAcceleration(ACCELERATION);
    stepper2->setSpeedInHz(MAX_SPEED);
    stepper2->setAcceleration(ACCELERATION);
    stepper1->moveTo(target1);
    stepper2->moveTo(target2);
    if (stepper1->getCurrentPosition() != target1 ||
        stepper2->getCurrentPosition() != target2) {
      if (!motorsRunning()) {
        Serial.println("[MOTOR] Adjustment command was not accepted");
        movingForward = false;
        return false;
      }
    }
    while (motorsRunning()) {
      pollESP();
      serviceUltrasonic();
      serviceBridgeRecovery();
      enforcePathWatchdog();
      if (resetQueued || shouldStop) {
        activeBrakeStopMotors();
        if (resetQueued) return false;
        haltAndWait("PATH:BLOCKED during adjustment");
        if (resetQueued || shouldStop || !pathCommandFresh()) return false;
        if (!startTargetMove(target1, target2)) return false;
        continue;
      }
      if (frontObstacleDetected()) {
        activeBrakeStopMotors();
        haltAndWait("SONIC:BLOCKED during adjustment");
        if (resetQueued || shouldStop || !pathCommandFresh()) return false;
        if (!startTargetMove(target1, target2)) return false;
      }
      delay(1);
    }
    movingForward = false;
    return stepper1->getCurrentPosition() == target1 &&
           stepper2->getCurrentPosition() == target2;
  }

  if (!startTargetMove(target1, target2)) return false;
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
      return false;
    }

    if (shouldStop) {
      activeBrakeStopMotors();
      const float localDist = latestFrontDistance();
      const String reason = (frontObstacleDetected() ||
                             localDist <= ULTRASONIC_STOP_DISTANCE_CM)
        ? "SONIC:BLOCKED (centered ultrasonic obstacle)"
        : (linkFaultActive ? "LINK:STALE" : "PATH:BLOCKED from LiDAR");
      haltAndWait(reason);
      if (resetQueued || shouldStop || !pathCommandFresh()) return false;
      if (!startTargetMove(target1, target2)) return false;
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
      if (resetQueued || shouldStop || !pathCommandFresh()) return false;
      if (!startTargetMove(target1, target2)) return false;
      commandServoTracked(DEFAULT_VIEW);
      continue;
    }

    delay(1);
  }

  if (motorsRunning()) smoothDecelStopMotors();
  else clearNudgeState();
  movingForward = false;
  commandServoTracked(DEFAULT_VIEW);
  return stepper1->getCurrentPosition() == target1 &&
         stepper2->getCurrentPosition() == target2;
}

bool moveDistance(int32_t steps, bool isADJ, bool isNudgeEnabled) {
  if (!stepper1 || !stepper2) return false;
  long t1 = stepper1->getCurrentPosition() + steps;
  long t2 = stepper2->getCurrentPosition() + steps;
  return moveToTarget(t1, t2, isADJ, isNudgeEnabled);
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
        if (response.length() < 768U) response += c;
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
static void modemInitializationTask(void*) {
  modemServiceBusy = true;
  modemReadyFlag = false;

  powerOnAir780();
  if (!waitForModule(10)) {
    Serial.println("[Air780E] WARNING: Module not responding; SMS remains disabled.");
  } else {
    sendAT("AT+CGSN");
    sendAT("AT+CSQ");
    sendAT("AT+CIMI");
    sendAT("AT+COPS?");
    sendAT("AT+CFUN=1", 5000);
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (!waitForNetwork(10)) {
      Serial.println("[Air780E] Network unavailable; control system remains operational.");
    }
    sendAT("AT+CSQ");
    modemReadyFlag = true;
  }

  modemServiceBusy = false;
  if (modemReadyFlag) {
    Serial.println("[Air780E] Background initialization complete.");
    queueSMSAlert("[GARBY] Restarted and ready.");
  }
  vTaskDelete(nullptr);
}

void startModemInitialization() {
  if (modemServiceBusy || modemReadyFlag) return;
  modemServiceBusy = true;
  if (xTaskCreate(modemInitializationTask, "garby-modem-init", 7168,
                  nullptr, 1, nullptr) != pdPASS) {
    modemServiceBusy = false;
    Serial.println("[Air780E] WARNING: Could not start modem init task; SMS disabled.");
  }
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
      if ((c >= 32 && c <= 126) || c == '\r' || c == '\n' || c == '>') {
        if (prompt.length() < 256U) prompt += c;
      }
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
      if ((c >= 32 && c <= 126) || c == '\r' || c == '\n') {
        if (result.length() < 768U) result += c;
      }
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
    vTaskDelete(nullptr);
    return;
  }
  buzzerState = true;
  digitalWrite(BUZZER_PIN, HIGH);
  vTaskDelay(pdMS_TO_TICKS(500));
  digitalWrite(BUZZER_PIN, LOW);
  vTaskDelay(pdMS_TO_TICKS(100));
  buzzerState = false;
  vTaskDelete(nullptr);
}
bool checkLoad(float threshold) {
  if (!scale.is_ready()) return false;
  float w = scale.get_units(1) - 0.010f;
  return (w >= threshold);
}
