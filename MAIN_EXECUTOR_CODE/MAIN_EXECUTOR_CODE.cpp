#include "MAIN_EXECUTOR_CODE.h"

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
HeadingControl         heading;

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

ReceivedDatas data;

GarbyState garbyState  = GarbyState::IDLE;
bool       resetQueued = false;
bool       routeFaultLatched = false;
bool       outboundComplete = false;
static String resetRequestId;
enum class ResetOutcome { NONE, ACTIVE, DONE, FAILED };
static ResetOutcome resetOutcome = ResetOutcome::NONE;

static void sendResetOutcome(const String& id, ResetOutcome outcome) {
  const char* label = outcome == ResetOutcome::DONE ? "DONE" :
                      outcome == ResetOutcome::FAILED ? "FAILED" : "ACK";
  ESP_Serial.println("[RESET " + String(label) + ":" + id + "]");
}

static void queueRouteFaultNotice();
void reportRouteFault() {
  ESP_Serial.println("[ROUTE FAULT]");
  // An operator stop uses the route latch only to unwind the current stack.
  // Its emergency notice must not invent a separate navigation failure.
  queueRouteFaultNotice();
  if (resetOutcome == ResetOutcome::ACTIVE) {
    resetOutcome = ResetOutcome::FAILED;
    sendResetOutcome(resetRequestId, resetOutcome);
  }
}

unsigned long lastIdlePrintMs = 0;
#define IDLE_PRINT_INTERVAL_MS   5000UL

static unsigned long lastBridgeRxMs = 0;
static bool manualStartLinkReady = false;
static String startRequestId;
static uint64_t lastStartRequestNumber = 0;
static bool startAccepted = false;
static String startFailure;
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
// Every completed sonar sample at any bearing: a hold's clearance evidence clock.
static uint32_t sonarEvidenceSequence = 0;

// Sonar sweep (2026-11-12): where the servo points and what each bearing saw.
static SonarSweep sonarSweep;
static int ultrasonicPingCommand = DEFAULT_VIEW;    // servo command the ping in flight was aimed at
static unsigned long ultrasonicEchoHighSinceMs = 0;
static bool spinInProgress = false;                 // spinning in place: forward-only pings
static bool spinSonarCheck = false;                 // inside a turn: floor-level spin clearance applies
static bool surveyPassActive = false;               // a low-power hold's periodic survey
static unsigned long lastSurveyPassMs = 0;
static int surveyPassSteps = 0;

// Low-power hold and buzzer state (2026-11-12).
static bool lowPowerActive = false;
static bool headingHoldActive = false;              // stopped, waiting for the MPU stream
static void enterLowPower(const char* why);
static void exitLowPower();
static void serviceBuzzerPattern();
static void serviceDriveReport();

// Retained over ESP.restart() so a coordinated reboot cannot release a stop.
// A power cycle starts in the normal fail-closed startup state.
RTC_NOINIT_ATTR static uint32_t operatorStopMarker;
static constexpr uint32_t OPERATOR_STOP_MARKER = 0x47425354UL;
static bool operatorEmergencyLatched = false;
static void restoreOperatorStop() {
  operatorEmergencyLatched = operatorStopMarker == OPERATOR_STOP_MARKER;
}
static bool operatorManualStartRequired = false;
static bool operatorSonarEnabled = true;
void initOperatorControls() { restoreOperatorStop(); }
bool operatorAutomaticStartAllowed() { return !operatorManualStartRequired; }
static String operatorLastId, operatorLastAction, operatorLastFailure;
static bool operatorLastAccepted = false;
static bool operatorRebootScheduled = false;
static unsigned long operatorRebootAtMs = 0;
static bool motorsRunning();
static void cancelUltrasonicPing();
static void sendOperatorStatus();
static void serviceOperatorReboot();

// Defined with the ultrasonic/servo implementation below. These declarations
// let every motor loop service safety sensing without blocking on pulseIn().
static void commandServoTracked(int angle);
static bool servoSettled();
static void cancelActiveNudge(bool addSettleTime);

static volatile bool smsWorkerBusy = false;
static volatile bool modemServiceBusy = false;
static volatile bool modemReadyFlag = false;
static volatile bool modemWorkerStarted = false;
static portMUX_TYPE smsMux = portMUX_INITIALIZER_UNLOCKED;
static constexpr uint32_t SMS_SPACING_MS = 20000UL;
static constexpr uint32_t SMS_WINDOW_MS = 600000UL;
static constexpr uint32_t SMS_RETRY_MS = 30000UL;
static constexpr uint8_t SMS_QUEUE_SIZE = 10;
// These messages use the GSM default alphabet only. 140 characters also leave
// room below one SMS if a future printable extension character needs escaping.
static constexpr uint16_t SMS_MESSAGE_MAX = 140;
struct SmsQueuedNotice {
  bool used = false;
  bool requireFaultSubmission = false;
  SmsNoticeKind kind = SmsNoticeKind::LEGACY;
  char state = 'E';
  char message[SMS_MESSAGE_MAX + 1] = {};
  uint32_t createdAt = 0, eligibleAt = 0, version = 0;
  uint8_t attempts = 0;
};
struct SmsQueueState {
  SmsQueuedNotice entries[SMS_QUEUE_SIZE];
  char submittedState[(uint8_t)SmsNoticeKind::COUNT] = {};
  char terminalState[(uint8_t)SmsNoticeKind::COUNT] = {};
  uint8_t transitionAttempts[(uint8_t)SmsNoticeKind::COUNT][2] = {};
  uint32_t transitionEligibleAt[(uint8_t)SmsNoticeKind::COUNT][2] = {};
  uint32_t attemptAt[6] = {};
  uint8_t attemptCount = 0;
  uint32_t lastAttemptAt = 0, nextVersion = 0;
  bool attempted = false;
  bool inFlight = false;
  SmsNoticeKind inFlightKind = SmsNoticeKind::LEGACY;
  char inFlightState = 'E';
};
static SmsQueueState smsQueueState;
struct NoticeMonitor { char value = 'X'; uint32_t since = 0; };
static NoticeMonitor healthMonitors[5], linkMonitor;
static bool healthPacketSeen = false;
static uint32_t healthPacketAtMs = 0;
static bool bridgeLinkObserved = false, bridgeLinkUp = false;
static bool routeFaultNoticeRaised = false;
static bool blockedNoticeActive = false;
static uint32_t blockedNoticeAtMs = 0;

static portMUX_TYPE loadCellMux = portMUX_INITIALIZER_UNLOCKED;
static float sampledLoadKg = -1.0f;
static uint32_t loadSampleSequence = 0;
static uint32_t lastLoadSampleMs = 0;
static bool loadSampleValid = false;

static void recordLoadCellSample(float readingKg) {
  float corrected = readingKg - 0.010f;
  if (corrected > -0.05f && corrected < 0.05f) corrected = 0.0f;
  const bool valid = isfinite(corrected) && corrected >= 0.0f && corrected <= LOAD_VALID_MAX_KG;
  portENTER_CRITICAL(&loadCellMux);
  sampledLoadKg = valid ? corrected : -1.0f;
  loadSampleValid = valid;
  lastLoadSampleMs = (uint32_t)millis();
  loadSampleSequence++;
  portEXIT_CRITICAL(&loadCellMux);
}

bool getLoadCellSample(float& kilograms, uint32_t& sequence) {
  portENTER_CRITICAL(&loadCellMux);
  kilograms = sampledLoadKg;
  sequence = loadSampleSequence;
  const bool valid = loadSampleValid &&
      (uint32_t)((uint32_t)millis() - lastLoadSampleMs) <= LOAD_SAMPLE_MAX_AGE_MS;
  portEXIT_CRITICAL(&loadCellMux);
  return valid;
}

static void loadCellWorkerTask(void*) {
  // HX711 reads/taring may wait for the converter. Keep all such waits out of
  // boot readiness, UART, sonar, and the motor executor.
  scale.set_scale(CALIBRATION_FACTOR);
  int64_t zeroSum = 0;
  uint8_t zeroCount = 0;
  uint32_t tareStarted = (uint32_t)millis();
  uint32_t lastTareWarning = tareStarted;
  bool tared = false;
  for (;;) {
    if (scale.is_ready()) {
      if (!tared) {
        zeroSum += scale.read();
        if (++zeroCount == 5) {
          scale.set_offset((long)(zeroSum / zeroCount));
          tared = true;
          Serial.printf("[LoadCell] Ready and tared. Zero factor: %ld\n", scale.get_offset());
        }
      } else {
        recordLoadCellSample(scale.get_units(1));
      }
    }
    const uint32_t now = (uint32_t)millis();
    if (!tared && (uint32_t)(now - tareStarted) >= 2000UL) {
      zeroSum = 0;
      zeroCount = 0;
      tareStarted = now;
      if ((uint32_t)(now - lastTareWarning) >= 5000UL) {
        lastTareWarning = now;
        Serial.println("[LoadCell] Tare unavailable; retrying in background, weight remains unavailable");
      }
    }
    vTaskDelay(pdMS_TO_TICKS(tared ? 100 : 10));
  }
}

void startLoadCellMonitor() {
  // The HX711 library can yield indefinitely if DOUT disappears during a read.
  // Keep that worker below the control loop and let idle/watchdog work run too.
  if (xTaskCreate(loadCellWorkerTask, "garby-load", 3072, nullptr, 0, nullptr) != pdPASS) {
    Serial.println("[LoadCell] Monitor could not start; weight remains unavailable");
  }
}

static uint8_t smsPriority(const SmsQueuedNotice& entry) {
  if (entry.kind == SmsNoticeKind::EMERGENCY) return 3;
  if (entry.kind == SmsNoticeKind::ROUTE_FAULT && entry.state == 'D') return 2;
  return entry.state == 'D' ? 1 : 0;
}

static bool enqueueSmsNotice(SmsNoticeKind kind, const char* message, char state,
                             bool requireFaultSubmission) {
  if ((uint8_t)kind >= (uint8_t)SmsNoticeKind::COUNT || !message || !message[0] ||
      (state != 'E' && state != 'D' && state != 'U')) return false;
  size_t length = 0;
  // Exclude non-GSM ASCII characters and all GSM extension characters. Every
  // accepted character therefore occupies exactly one default-alphabet septet.
  for (; message[length]; ++length) {
    const unsigned char c = (unsigned char)message[length];
    if (length >= SMS_MESSAGE_MAX || c < 32 || c > 126 ||
        c == '`' || c == '^' || c == '{' || c == '}' || c == '[' || c == ']' ||
        c == '\\' || c == '~' || c == '|') return false;
  }
  const uint32_t now = (uint32_t)millis();
  portENTER_CRITICAL(&smsMux);
  int slot = -1;
  for (uint8_t i = 0; i < SMS_QUEUE_SIZE; ++i) {
    auto& entry = smsQueueState.entries[i];
    if (entry.used && (uint32_t)(now - entry.createdAt) >= SMS_WINDOW_MS) entry.used = false;
    if (entry.used && entry.kind == kind) {
      if (entry.state == state && strcmp(entry.message, message) == 0) {
        portEXIT_CRITICAL(&smsMux);
        return true;
      }
      slot = i;
      break;
    }
    if (!entry.used && slot < 0) slot = i;
  }
  const bool oppositeInFlight = smsQueueState.inFlight &&
      smsQueueState.inFlightKind == kind && smsQueueState.inFlightState != state;
  if (state != 'E' && smsQueueState.terminalState[(uint8_t)kind] == state && !oppositeInFlight) {
    // A fault that returns before its queued recovery is sent invalidates that
    // recovery. If the recovery is already on the UART, retain a restoring D.
    for (auto& entry : smsQueueState.entries)
      if (entry.used && entry.kind == kind) entry.used = false;
    portEXIT_CRITICAL(&smsMux); return true;
  }
  if (slot < 0) {
    SmsQueuedNotice incoming;
    incoming.kind = kind; incoming.state = state;
    const uint8_t incomingPriority = smsPriority(incoming);
    uint8_t lowestPriority = incomingPriority;
    uint32_t oldestAge = 0;
    for (uint8_t i = 0; i < SMS_QUEUE_SIZE; ++i) {
      const auto& entry = smsQueueState.entries[i];
      const uint32_t age = (uint32_t)(now - entry.createdAt);
      const uint8_t priority = smsPriority(entry);
      if (priority < incomingPriority &&
          (slot < 0 || priority < lowestPriority || (priority == lowestPriority && age > oldestAge))) {
        slot = i; oldestAge = age; lowestPriority = priority;
      }
    }
  }
  if (slot < 0) { portEXIT_CRITICAL(&smsMux); return false; }
  auto& entry = smsQueueState.entries[slot];
  entry = SmsQueuedNotice{};
  entry.used = true; entry.kind = kind; entry.state = state;
  entry.requireFaultSubmission = requireFaultSubmission;
  memcpy(entry.message, message, length + 1);
  entry.createdAt = entry.eligibleAt = now;
  if (state != 'E') {
    const uint8_t stateIndex = state == 'D' ? 0 : 1;
    entry.attempts = smsQueueState.transitionAttempts[(uint8_t)kind][stateIndex];
    if (entry.attempts)
      entry.eligibleAt = smsQueueState.transitionEligibleAt[(uint8_t)kind][stateIndex];
  }
  entry.version = ++smsQueueState.nextVersion;
  portEXIT_CRITICAL(&smsMux);
  return true;
}

bool queueSMSNotice(SmsNoticeKind kind, const char* message, char state) {
  return enqueueSmsNotice(kind, message, state, false);
}

bool queueSMSAlert(const String& message) {
  return queueSMSNotice(SmsNoticeKind::LEGACY, message.c_str());
}

static void resetSmsTransition(SmsNoticeKind kind, char state);
static void queueRouteFaultNotice() {
  if (!operatorEmergencyLatched && !routeFaultNoticeRaised) {
    resetSmsTransition(SmsNoticeKind::ROUTE_FAULT, 'U');
    routeFaultNoticeRaised = queueSMSNotice(SmsNoticeKind::ROUTE_FAULT,
        "GARBY: Route fault. Motors stopped; supervised recovery is required.", 'D');
  }
}

static void cancelPendingSms(SmsNoticeKind kind, char state = 0) {
  portENTER_CRITICAL(&smsMux);
  for (auto& entry : smsQueueState.entries)
    if (entry.used && entry.kind == kind && (!state || entry.state == state)) entry.used = false;
  portEXIT_CRITICAL(&smsMux);
}

static char submittedSmsState(SmsNoticeKind kind) {
  portENTER_CRITICAL(&smsMux);
  const char result = smsQueueState.submittedState[(uint8_t)kind];
  portEXIT_CRITICAL(&smsMux);
  return result;
}

static void queuePairedRecovery(SmsNoticeKind kind, const char* message) {
  portENTER_CRITICAL(&smsMux);
  const bool knownFault = smsQueueState.submittedState[(uint8_t)kind] == 'D';
  const bool submittingFault = smsQueueState.inFlight && smsQueueState.inFlightKind == kind &&
      (smsQueueState.inFlightState == 'D' ||
       (kind == SmsNoticeKind::BLOCKED && smsQueueState.inFlightState == 'E'));
  portEXIT_CRITICAL(&smsMux);
  // This records the current local state while an older fault is on the UART.
  // The worker releases the recovery only after proving that D was submitted.
  if (knownFault || submittingFault) enqueueSmsNotice(kind, message, 'U', true);
}

static void resetSmsTransition(SmsNoticeKind kind, char state) {
  // Unknown/stale data never calls this. Only sustained evidence of the
  // opposite state can give a failed/uncertain transition a fresh budget.
  const uint8_t index = state == 'D' ? 0 : 1;
  portENTER_CRITICAL(&smsMux);
  if (smsQueueState.terminalState[(uint8_t)kind] == state)
    smsQueueState.terminalState[(uint8_t)kind] = 0;
  smsQueueState.transitionAttempts[(uint8_t)kind][index] = 0;
  smsQueueState.transitionEligibleAt[(uint8_t)kind][index] = 0;
  portEXIT_CRITICAL(&smsMux);
}

bool serviceSmsQueueOnce(uint32_t now) {
  // Only the persistent modem task calls this service in production. Enqueue
  // never waits for it, and no critical section includes a UART transaction.
  if (!modemReadyFlag) return false;
  SmsQueuedNotice selected;
  int slot = -1;
  portENTER_CRITICAL(&smsMux);
  uint8_t live = 0;
  for (uint8_t i = 0; i < smsQueueState.attemptCount; ++i)
    if ((uint32_t)(now - smsQueueState.attemptAt[i]) < SMS_WINDOW_MS)
      smsQueueState.attemptAt[live++] = smsQueueState.attemptAt[i];
  smsQueueState.attemptCount = live;
  for (auto& entry : smsQueueState.entries)
    if (entry.used && (uint32_t)(now - entry.createdAt) >= SMS_WINDOW_MS) entry.used = false;
  if (live >= 6 || (smsQueueState.attempted &&
                   (uint32_t)(now - smsQueueState.lastAttemptAt) < SMS_SPACING_MS)) {
    portEXIT_CRITICAL(&smsMux); return false;
  }
  for (uint8_t i = 0; i < SMS_QUEUE_SIZE; ++i) {
    auto& entry = smsQueueState.entries[i];
    if (entry.used && entry.requireFaultSubmission &&
        smsQueueState.submittedState[(uint8_t)entry.kind] != 'D') entry.used = false;
    if (!entry.used || (int32_t)(now - entry.eligibleAt) < 0) continue;
    if (slot < 0 || smsPriority(entry) > smsPriority(selected) ||
        (smsPriority(entry) == smsPriority(selected) &&
         (uint32_t)(now - entry.createdAt) > (uint32_t)(now - selected.createdAt))) {
      slot = i; selected = entry;
    }
  }
  if (slot < 0) { portEXIT_CRITICAL(&smsMux); return false; }
  smsQueueState.attemptAt[smsQueueState.attemptCount++] = now;
  smsQueueState.lastAttemptAt = now; smsQueueState.attempted = true;
  smsQueueState.inFlight = true;
  smsQueueState.inFlightKind = selected.kind;
  smsQueueState.inFlightState = selected.state;
  if (selected.state != 'E')
    ++smsQueueState.transitionAttempts[(uint8_t)selected.kind][selected.state == 'D' ? 0 : 1];
  portEXIT_CRITICAL(&smsMux);
  smsWorkerBusy = true;
  const SmsSubmitResult result = sendSMS(CONTACT_NUMBER, selected.message);
  smsWorkerBusy = false;
  portENTER_CRITICAL(&smsMux);
  smsQueueState.inFlight = false;
  const char submittedState = selected.state == 'E' && selected.kind == SmsNoticeKind::BLOCKED ? 'D' : selected.state;
  if (result == SmsSubmitResult::SUBMITTED && submittedState != 'E')
    smsQueueState.submittedState[(uint8_t)selected.kind] = submittedState;
  if (selected.state != 'E') {
    const uint8_t index = selected.state == 'D' ? 0 : 1;
    if (result == SmsSubmitResult::RETRYABLE_FAILURE && selected.attempts == 0)
      smsQueueState.transitionEligibleAt[(uint8_t)selected.kind][index] = (uint32_t)millis() + SMS_RETRY_MS;
    else smsQueueState.terminalState[(uint8_t)selected.kind] = selected.state;
  }
  auto& entry = smsQueueState.entries[slot];
  // A newer transition can replace an in-flight notice. Its state and retry
  // budget belong to that newer transition and must survive the old outcome.
  if (entry.used && entry.version == selected.version) {
    if (result == SmsSubmitResult::RETRYABLE_FAILURE && selected.attempts == 0) {
      entry.attempts = 1;
      entry.eligibleAt = (uint32_t)millis() + SMS_RETRY_MS;
    } else entry.used = false;
  }
  portEXIT_CRITICAL(&smsMux);
  return true;
}

static void observeNotice(NoticeMonitor& monitor, char value, uint32_t now) {
  if (monitor.value != value) { monitor.value = value; monitor.since = now; }
}

bool parseHealthStatus(const String& packet) {
  if (packet.length() != 16 || !packet.startsWith("[HEALTH:1:") || packet.charAt(15) != ']') return false;
  for (uint8_t i = 0; i < 5; ++i) {
    const char value = packet.charAt(10 + i);
    if (value != 'U' && value != 'D' && value != 'X') return false;
  }
  const uint32_t now = (uint32_t)millis();
  if (healthPacketSeen && (uint32_t)(now - healthPacketAtMs) > 15000UL)
    for (auto& monitor : healthMonitors) observeNotice(monitor, 'X', now);
  healthPacketSeen = true; healthPacketAtMs = now;
  for (uint8_t i = 0; i < 5; ++i) observeNotice(healthMonitors[i], packet.charAt(10 + i), now);
  return true;
}

static void monitorNotice(NoticeMonitor& monitor, SmsNoticeKind kind,
                          uint32_t now, uint32_t downMs, uint32_t upMs,
                          const char* downMessage, const char* upMessage) {
  if (monitor.value == 'X') { cancelPendingSms(kind); return; }
  if (monitor.value == 'D') {
    cancelPendingSms(kind, 'U');
    if ((uint32_t)(now - monitor.since) >= downMs) {
      resetSmsTransition(kind, 'U');
      queueSMSNotice(kind, downMessage, 'D');
    }
  } else {
    cancelPendingSms(kind, 'D');
    if ((uint32_t)(now - monitor.since) >= upMs) {
      resetSmsTransition(kind, 'D');
      if (submittedSmsState(kind) == 'D') queueSMSNotice(kind, upMessage, 'U');
    }
  }
}

void serviceNoticeMonitoring(uint32_t now) {
  if (healthPacketSeen && (uint32_t)(now - healthPacketAtMs) > 15000UL)
    for (auto& monitor : healthMonitors) observeNotice(monitor, 'X', now);
  static const SmsNoticeKind kinds[5] = {SmsNoticeKind::INTERNET, SmsNoticeKind::CLOUD,
      SmsNoticeKind::LIDAR, SmsNoticeKind::SENSORS, SmsNoticeKind::THERMAL};
  static const uint32_t faultMs[5] = {30000, 60000, 15000, 30000, 10000};
  static const char* const downMessages[5] = {
      "GARBY: Internet unavailable on the Pi. Local robot safety remains active.",
      "GARBY: Cloud connection unavailable on the Pi. Local robot safety remains active.",
      "GARBY: LiDAR data unavailable or unhealthy. Check the Pi and LiDAR.",
      "GARBY: Bin sensor UART data unavailable or invalid. Check the sensor controller.",
      "GARBY: Thermal warning on the Pi: high temperature or active throttling."};
  static const char* const upMessages[5] = {"GARBY: Internet connection recovered.",
      "GARBY: Cloud connection recovered.", "GARBY: LiDAR data recovered.",
      "GARBY: Bin sensor UART data recovered.", "GARBY: Thermal condition recovered on the Pi."};
  for (uint8_t i = 0; i < 5; ++i)
    monitorNotice(healthMonitors[i], kinds[i], now, faultMs[i], i == 4 ? 30000UL : 10000UL,
                  downMessages[i], upMessages[i]);
  const bool freshLink = bridgeLinkObserved && bridgeLinkUp &&
      (uint32_t)(now - lastBridgeRxMs) <= BRIDGE_RX_STALE_MS;
  observeNotice(linkMonitor, bridgeLinkObserved ? (freshLink ? 'U' : 'D') : 'X', now);
  monitorNotice(linkMonitor, SmsNoticeKind::LINK, now, 10000UL, 10000UL,
      "GARBY: Pi/controller link unavailable. Check BLE and the controller connection.",
      "GARBY: Pi/controller link recovered. Motion still requires fresh safety clearance.");
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
  // A parser/protocol fault can revoke the startup handshake while ordinary
  // path traffic keeps UART fresh. Re-handshake at IDLE so manual start can
  // recover without resetting the controllers or a moving heading segment.
  const bool needsStartupHandshake = garbyState == GarbyState::IDLE &&
                                     !manualStartLinkReady;
  if (!rxStale && !needsStartupHandshake) return;

  lastConnected = false;
  manualStartLinkReady = false;
  if (lastReadyRecoveryMs != 0 &&
      now - lastReadyRecoveryMs < BRIDGE_RECOVERY_INTERVAL_MS) {
    return;
  }
  lastReadyRecoveryMs = now;
  // [MCU READY] tells the bridge this controller restarted, so it drops the
  // straight-run heading. That is fine at IDLE, where the startup handshake also
  // needs renewing. During a route, silence usually just means a BLE outage, so
  // only re-announce ourselves in case the bridge itself rebooted.
  ESP_Serial.println(garbyState == GarbyState::IDLE ? "[MCU READY]" : "[MCU ALIVE]");
}

void responsiveDelay(unsigned long durationMs) {
  const unsigned long started = millis();
  while (millis() - started < durationMs) {
    pollESP();
    serviceUltrasonic();
    serviceBridgeRecovery();
    enforcePathWatchdog();
    vTaskDelay(pdMS_TO_TICKS(2));
  }
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
static String lastReportedMotion = "";
static bool haltWaitingForClearance = false;
static bool haltRequiresLocal = false;
static bool uartRxReserved = false;
static bool uartDiscardUntilNewline = false;
static bool uartCarriageReturnSeen = false;
static bool goEvidenceSeen = false;
static uint32_t lastGoEvidenceMs = 0;
static void reportMotionStatus(bool force = false);

static void resetGoEvidence() {
  clearPathCommandCount = 0;
  goEvidenceSeen = false;
}

// Name what actually holds the robot: a lost MPU reference needs supervised
// alignment, and reporting it as a LiDAR block sends the operator the wrong way.
static String motionHoldReason(const char* linkReason) {
  if (lastStopReason.indexOf("MPU") >= 0) return "MPU:HEADING_UNAVAILABLE (bridge MPU recovering)";
  if (lastStopReason.indexOf("TURN") >= 0) return "PATH:TURN_CLEARANCE from LiDAR";
  return linkFaultActive ? linkReason : "PATH:BLOCKED from LiDAR";
}

static void latchUartFault() {
  // Parser context only invalidates permission. The motion loop owns braking,
  // avoiding recursive pollESP() calls from controlledStopMotors().
  shouldStop = true;
  manualStartLinkReady = false;
  linkFaultActive = true;
  pathCommandSeen = false;
  resetGoEvidence();
}

static void noteBridgeRx() {
  const uint32_t now = (uint32_t)millis();
  if (bridgeLinkObserved && bridgeLinkUp &&
      (uint32_t)(now - lastBridgeRxMs) > BRIDGE_RX_STALE_MS)
    observeNotice(linkMonitor, 'D', now);
  lastBridgeRxMs = now;
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
  resetGoEvidence();
  reportMotionStatus();
}

static void sendOperatorStatus() {
  ESP_Serial.println("[OP STATUS:1:ESTOP=" + String(operatorEmergencyLatched ? 1 : 0) +
                     ":SONAR=" + String(operatorSonarEnabled ? 1 : 0) + "]");
}

static void serviceOperatorReboot() {
  if (!operatorRebootScheduled ||
      (long)(millis() - operatorRebootAtMs) < 0) return;
  emergencyStopMotors();
  ESP.restart();
}

static bool manualStartConnectionReady() {
  return manualStartLinkReady && lastConnected &&
         millis() - lastBridgeRxMs <= BRIDGE_RX_STALE_MS;
}

static void reportMotionStatus(bool force) {
  String reason;
  if (operatorEmergencyLatched) reason = "EMERGENCY_STOP";
  else if (routeFaultLatched) reason = "ROUTE_FAULT";
  else if (!stepper1 || !stepper2) reason = "MOTORS_UNAVAILABLE";
  else if (!pathCommandFresh()) reason = "WAITING_FOR_PATH";
  else if (shouldStop) reason = lastStopReason.length() ? lastStopReason : String("WAITING_FOR_PATH");
  else if (frontObstacleDetected() || (haltWaitingForClearance && haltRequiresLocal)) reason = "LOCAL_OBSTACLE";
  else if (haltWaitingForClearance) reason = "CONFIRMING_CLEARANCE";
  else reason = "CLEAR";
  if (force || reason != lastReportedMotion) {
    lastReportedMotion = reason;
    ESP_Serial.println("[MOTION:" + reason + "]");
  }
}

static void reportRobotStatus() {
  // Replay the whole turn scope, including holds, before announcing readiness.
  // A restarted bridge has no RAM history of the original TURN:START notice.
  ESP_Serial.println(spinSonarCheck ? "[TURN:START]" : "[TURN:END]");
  // This capability/heartbeat crosses both UART directions and BLE. It
  // never waits for a load-cell, bin sensor, modem, or cloud reading.
  if (manualStartConnectionReady()) {
    ESP_Serial.println("[SYSTEM READY]");
    // Capability + fresh idle eligibility only. This never grants GO and
    // deliberately does not depend on bin level, gas or load readings.
    if (garbyState == GarbyState::IDLE && !outboundComplete &&
        !routeFaultLatched && !resetQueued && !operatorEmergencyLatched &&
        stepper1 && stepper2) ESP_Serial.println("[DISPOSAL READY:1]");
  }
  if (routeFaultLatched) ESP_Serial.println("[ROUTE FAULT]");
  else if (outboundComplete) ESP_Serial.println("[OUTBOUND COMPLETE]");
  else if (garbyState == GarbyState::RETURNING) ESP_Serial.println("[RETURNING]");
  else if (garbyState == GarbyState::RUNNING) ESP_Serial.println("[RUNNING]");
  else ESP_Serial.println("[IDLE]");
  reportMotionStatus(true);
  sendOperatorStatus();
}

bool serviceOperatorStop() {
  if (!operatorEmergencyLatched) return false;
  emergencyStopMotors();
  const bool changed = garbyState != GarbyState::IDLE || outboundComplete;
  garbyState = GarbyState::IDLE;
  outboundComplete = false;
  resetQueued = false;
  routeFaultLatched = false;
  shouldStop = true;
  if (changed) reportRobotStatus();
  return true;
}

static void sendStartOutcome() {
  if (startAccepted) ESP_Serial.println("[START ACK:" + startRequestId + "]");
  else ESP_Serial.println("[START FAILED:" + startRequestId + ":" + startFailure + "]");
}

static bool validOperatorId(const String& id) {
  if (id.length() == 0 || id.length() > 20 || id.charAt(0) == '0') return false;
  for (unsigned int i = 0; i < id.length(); ++i) {
    if (id.charAt(i) < '0' || id.charAt(i) > '9') return false;
  }
  return true;
}

static void sendOperatorOutcome(const String& id, const String& action,
                                bool accepted, const String& failure) {
  ESP_Serial.println(accepted ? "[OP ACK:" + id + ":" + action + "]" :
                               "[OP FAILED:" + id + ":" + failure + "]");
  sendOperatorStatus();
  ESP_Serial.println(ACK_MSG);
}

static void latchOperatorEmergency() {
  const bool firstStop = !operatorEmergencyLatched;
  operatorStopMarker = OPERATOR_STOP_MARKER;
  operatorEmergencyLatched = true;
  operatorManualStartRequired = true;
  shouldStop = true;
  resetQueued = true;  // unwind any active route before allowing RELEASE
  routeFaultLatched = true;
  pathCommandSeen = false;
  linkFaultActive = true;
  resetGoEvidence();
  emergencyStopMotors();  // force-stop is nonblocking and does not poll UART
  if (firstStop)
    queueSMSNotice(SmsNoticeKind::EMERGENCY,
        "GARBY: Emergency stop active. Motors stopped; operator release and a fresh start are required.", 'D');
  commandServoTracked(DEFAULT_VIEW);
  if (resetOutcome == ResetOutcome::ACTIVE) {
    resetOutcome = ResetOutcome::FAILED;
    sendResetOutcome(resetRequestId, resetOutcome);
  }
}

static void processOperatorCommand(const String& espMsg) {
  if (espMsg == "[OP STATUS]") {
    sendOperatorStatus();
    ESP_Serial.println(ACK_MSG);
    return;
  }
  if (!espMsg.endsWith("]")) return;
  const int colon = espMsg.indexOf(':', 4);
  if (colon < 0) return;
  const String id = espMsg.substring(4, colon);
  const String action = espMsg.substring(colon + 1, espMsg.length() - 1);
  if (!validOperatorId(id)) return;
  if (action != "ESTOP" && action != "RELEASE" && action != "SONAR_OFF" &&
      action != "SONAR_ON" && action != "REBOOT") {
    sendOperatorOutcome(id, action, false, "UNKNOWN_ACTION");
    return;
  }
  // Even a delayed/replayed emergency request must fail on the safe side.
  if (action == "ESTOP") latchOperatorEmergency();
  if (id == operatorLastId) {
    sendOperatorOutcome(id, action, operatorLastAccepted && action == operatorLastAction,
                        action == operatorLastAction ? operatorLastFailure : String("ID_REUSED"));
    return;
  }
  if (operatorLastId.length() &&
      (id.length() < operatorLastId.length() ||
       (id.length() == operatorLastId.length() &&
        strcmp(id.c_str(), operatorLastId.c_str()) < 0))) {
    sendOperatorOutcome(id, action, false, "STALE_REQUEST");
    return;
  }
  operatorLastId = id;
  operatorLastAction = action;
  operatorLastAccepted = false;
  operatorLastFailure = "";
  const bool stationaryIdle = garbyState == GarbyState::IDLE && !motorsRunning();
  if (action == "ESTOP") {
    operatorLastAccepted = true;
  } else if (operatorRebootScheduled) {
    operatorLastFailure = "REBOOT_PENDING";
  } else if (!manualStartConnectionReady()) {
    operatorLastFailure = "NOT_READY";
  } else if (!stationaryIdle) {
    operatorLastFailure = "NOT_IDLE";
  } else if (action == "RELEASE") {
    if (!operatorEmergencyLatched) operatorLastFailure = "NOT_STOPPED";
    else {
      operatorStopMarker = 0;
      operatorEmergencyLatched = false;
      operatorManualStartRequired = true;
      // Releasing the latch grants no GO evidence and cannot restart a route.
      shouldStop = true;
      pathCommandSeen = false;
      linkFaultActive = true;
      resetQueued = false;
      routeFaultLatched = false;
      resetGoEvidence();
      operatorLastAccepted = true;
      queueSMSNotice(SmsNoticeKind::EMERGENCY,
          "GARBY: Emergency stop released. Robot remains stopped until a deliberate fresh start.", 'U');
    }
  } else if (action == "REBOOT") {
    if (!operatorEmergencyLatched) operatorLastFailure = "ESTOP_REQUIRED";
    else {
      operatorRebootScheduled = true;
      operatorRebootAtMs = millis() + 750UL;
      operatorLastAccepted = true;
    }
  } else {
    // This controls only the executor's obstacle sonar, not bin-level sonar.
    operatorSonarEnabled = action == "SONAR_ON";
    cancelUltrasonicPing();
    digitalWrite(TRIG_PIN, LOW);
    latestFrontSampleValid = false;
    lastFrontSampleMs = 0;
    frontFilterCount = frontFilterIndex = 0;
    ultrasonicBlockedCount = ultrasonicClearCount = 0;
    frontObstacleLatched = operatorSonarEnabled;  // fresh clear streak to rearm
    frontEmergencyActive = false;
    sonarSweep.reset();  // no bearing's old echo can block or clear the new mode
    commandServoTracked(DEFAULT_VIEW);
    operatorLastAccepted = true;
  }
  noteBridgeRx();
  sendOperatorOutcome(id, action, operatorLastAccepted, operatorLastFailure);
}

// LiDAR caution zone (bridge "GO:SLOW"): something is in the swept corridor
// beyond the stop distance. Slowing is immediate; full speed returns only after
// several plain GO packets, so the zone edge cannot make the speed flap.
#define LIDAR_SLOW_RELEASE_PACKETS 3
static bool lidarSlowActive = false;
static uint8_t lidarNormalGoCount = 0;

static void processESPMessage(const String& espMsg) {
    // The 50 Hz MPU stream is heading evidence only: no ACK, no link or path
    // freshness, and it can never release a STOP.
    if (heading.handleLine(espMsg.c_str(), (uint32_t)millis())) return;
    if (espMsg.startsWith("[HEALTH:")) {
      // Operational telemetry is ACKed for bridge flow control, but never
      // counts as path evidence, a startup handshake, or bridge readiness.
      if (parseHealthStatus(espMsg)) ESP_Serial.println(ACK_MSG);
      return;
    }
    if (espMsg == "[OP STATUS]" || espMsg.startsWith("[OP:")) {
      processOperatorCommand(espMsg);
      return;
    }

    if (espMsg == "[ROBOT STATUS]") {
      reportRobotStatus();
      return;
    }
    if (espMsg == "[BLE CONNECTION ESTABLISHED]") {
      noteBridgeRx();
      bridgeLinkObserved = bridgeLinkUp = true;
      observeNotice(linkMonitor, 'U', (uint32_t)millis());
      shouldStop = true;
      linkFaultActive = true;
      pathCommandSeen = false;
      resetGoEvidence();
      manualStartLinkReady = true;
      reportRobotStatus();
      requestStatus();
      static bool readyBuzzerScheduled = false;
      if (!readyBuzzerScheduled) {
        readyBuzzerScheduled = true;
        buzzerAlert(BuzzerAlert::READY);
      }
      return;
    }

    const bool startQuery = espMsg.startsWith("[START STATUS:");
    if (espMsg.startsWith("[START:") || startQuery) {
      if (!espMsg.endsWith("]")) return;
      const String id = espMsg.substring(startQuery ? 14 : 7, espMsg.length() - 1);
      if (id.length() == 0 || id.length() > 16) return;
      uint64_t requestNumber = 0;
      for (unsigned int i = 0; i < id.length(); ++i) {
        if (id.charAt(i) < '0' || id.charAt(i) > '9') return;
        requestNumber = requestNumber * 10U + (uint64_t)(id.charAt(i) - '0');
      }
      if (requestNumber == 0 || id.charAt(0) == '0') return;
      if (startQuery) {
        // A read-only recovery path for a lost GATT write/notification. Never
        // refresh motion permission, consume a new ID, or repeat a route.
        if (id == startRequestId) sendStartOutcome();
        else ESP_Serial.println("[START FAILED:" + id + ":NOT_RECEIVED]");
        reportRobotStatus();
        ESP_Serial.println(ACK_MSG);
        return;
      }
      if (id == startRequestId) {
        // Remember across fullReset(): replaying a completed start cannot
        // launch another route after the robot has returned to base.
        sendStartOutcome();
        reportRobotStatus();
        ESP_Serial.println(ACK_MSG);
        return;
      }
      if (requestNumber <= lastStartRequestNumber) {
        ESP_Serial.println("[START FAILED:" + id + ":STALE_REQUEST]");
        ESP_Serial.println(ACK_MSG);
        return;
      }
      const bool connected = manualStartConnectionReady();
      noteBridgeRx();
      lastStartRequestNumber = requestNumber;
      startRequestId = id;
      startAccepted = false;
      if (operatorEmergencyLatched) startFailure = "EMERGENCY_STOP";
      else if (!connected) startFailure = "NOT_READY";
      else if (routeFaultLatched || resetQueued) startFailure = "ROUTE_FAULT";
      else if (garbyState != GarbyState::IDLE || outboundComplete) startFailure = "NOT_IDLE";
      else if (!stepper1 || !stepper2) startFailure = "MOTORS_UNAVAILABLE";
      else {
        // Same route and motion gates as the automatic trigger. Keep STOP,
        // path freshness, and local obstacle sensing in charge of the wheels.
        outboundComplete = false;
        garbyState = GarbyState::RUNNING;
        startAccepted = true;
        operatorManualStartRequired = false;
        startFailure = "";
        Serial.println("[STATE] IDLE -> RUNNING (manual disposal; bin/gas/load triggers bypassed)");
        queueSMSNotice(SmsNoticeKind::TRIP, "GARBY: Manual disposal requested. Motion awaits safety clearance.");
      }
      sendStartOutcome();
      reportRobotStatus();
      ESP_Serial.println(ACK_MSG);
      return;
    }

    if (espMsg == "[RESET]" || espMsg.startsWith("[RESET:")) {
      noteBridgeRx();
      const bool correlated = espMsg != "[RESET]";
      String id;
      if (correlated) {
        if (!espMsg.endsWith("]")) return;
        id = espMsg.substring(7, espMsg.length() - 1);
        // Millisecond epoch IDs fit in 16 decimal digits. Validate each digit
        // without truncating the value into the ESP32's 32-bit unsigned long.
        if (id.length() == 0 || id.length() > 16) return;
        for (unsigned int i = 0; i < id.length(); ++i) {
          if (id.charAt(i) < '0' || id.charAt(i) > '9') return;
        }
        if (id == resetRequestId && resetOutcome != ResetOutcome::NONE) {
          sendResetOutcome(id, resetOutcome);
          ESP_Serial.println(ACK_MSG);
          return;
        }
      }
      if (operatorEmergencyLatched) {
        if (correlated) sendResetOutcome(id, ResetOutcome::FAILED);
        ESP_Serial.println(ACK_MSG);
        return;
      }
      if (garbyState == GarbyState::RETURNING && !routeFaultLatched) {
        if (correlated) sendResetOutcome(id, ResetOutcome::FAILED);
        ESP_Serial.println(ACK_MSG);
        return;
      }
      shouldStop = true;
      linkFaultActive = true;
      resetGoEvidence();
      if (correlated) {
        resetRequestId = id;
        resetOutcome = ResetOutcome::ACTIVE;
      }
      if (garbyState == GarbyState::RUNNING && outboundComplete && !routeFaultLatched) {
        resetQueued = true;
        if (correlated) sendResetOutcome(id, ResetOutcome::ACTIVE);
        Serial.println("[RESET] Explicit return/reset queued");
      } else if (!correlated && routeFaultLatched) {
        // Local supervised recovery after physically returning the robot to
        // base. App retries cannot take this recovery branch.
        emergencyStopMotors();
        fullReset();
      } else if (correlated || garbyState == GarbyState::RUNNING) {
        if (garbyState == GarbyState::RUNNING) {
          routeFaultLatched = true;
          resetQueued = true;  // abort the incomplete segment, never run return
        }
        resetOutcome = ResetOutcome::FAILED;
        if (correlated) sendResetOutcome(id, resetOutcome);
        ESP_Serial.println("[RETURN REJECTED]");
      } else {
        emergencyStopMotors();
        fullReset();
      }
      ESP_Serial.println(ACK_MSG);
      return;
    }

    if (espMsg.startsWith("STOP")) {
      noteBridgeRx();
      if (espMsg.indexOf("LINK") >= 0 && bridgeLinkObserved) {
        bridgeLinkUp = false;
        observeNotice(linkMonitor, 'D', (uint32_t)millis());
      }
      const bool changed = !shouldStop || espMsg != lastStopReason;
      shouldStop = true;
      pathCommandSeen = true;
      lastPathCommandMs = millis();
      resetGoEvidence();
      linkFaultActive = espMsg.indexOf("LINK") >= 0 ||
                        espMsg.indexOf("STALE") >= 0 ||
                        espMsg.indexOf("WAITING") >= 0 ||
                        espMsg.indexOf("PROTOCOL") >= 0;
      if (espMsg.indexOf("LINK") >= 0 || espMsg.indexOf("PROTOCOL") >= 0) {
        manualStartLinkReady = false;
      }
      lastStopReason = espMsg;
      if (changed) Serial.println("[STOP] " + espMsg);
      reportMotionStatus();
      ESP_Serial.println(ACK_MSG);
      return;
    }

    if (espMsg == "GO" || espMsg == "GO:SLOW") {
      noteBridgeRx();
      if (espMsg == "GO:SLOW") {
        lidarSlowActive = true;
        lidarNormalGoCount = 0;
      } else if (lidarSlowActive && ++lidarNormalGoCount >= LIDAR_SLOW_RELEASE_PACKETS) {
        lidarSlowActive = false;
      }
      if (!pathCommandFresh()) {
        resetGoEvidence();
      }
      pathCommandSeen = true;
      lastPathCommandMs = millis();
      const uint32_t now = (uint32_t)millis();
      if ((!goEvidenceSeen ||
           (uint32_t)(now - lastGoEvidenceMs) >= MCU_GO_CONFIRM_MIN_INTERVAL_MS) &&
          clearPathCommandCount < MCU_GO_CONFIRM_PACKETS) {
        clearPathCommandCount++;
        goEvidenceSeen = true;
        lastGoEvidenceMs = now;
      }
      if (clearPathCommandCount >= MCU_GO_CONFIRM_PACKETS && !routeFaultLatched &&
          !operatorEmergencyLatched) {
        if (shouldStop) Serial.println("[GO] Fresh path confirmed -- motion enabled");
        shouldStop = false;
        linkFaultActive = false;
        lastStopReason = "";
      }
      reportMotionStatus();
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
  serviceOperatorReboot();
  // Wheel positions bridge the gaps between MPU samples.
  if (stepper1 && stepper2) heading.attach(stepper1, stepper2);
  if (!uartRxReserved) {
    uartRxBuffer.reserve(UART_RX_LINE_MAX + 1U);
    lastStopReason.reserve(96);
    uartRxReserved = true;
  }

  uint16_t byteBudget = UART_RX_BUDGET_BYTES;
  while (byteBudget-- > 0 && ESP_Serial.available()) {
    const char c = (char)ESP_Serial.read();

    if (uartDiscardUntilNewline) {
      if (c == '\n') {
        uartDiscardUntilNewline = false;
        uartCarriageReturnSeen = false;
        static unsigned long lastOverflowLogMs = 0;
        if (millis() - lastOverflowLogMs >= 2000UL) {
          lastOverflowLogMs = millis();
          Serial.println("[WARN] Damaged ESP UART line discarded");
        }
      }
      continue;
    }

    if (c == '\r' && !uartCarriageReturnSeen) {
      uartCarriageReturnSeen = true;
      continue;
    }
    if (c != '\n') {
      if (uartCarriageReturnSeen || (unsigned char)c < 32U || (unsigned char)c > 126U) {
        uartRxBuffer.remove(0);
        uartDiscardUntilNewline = true;
        uartCarriageReturnSeen = false;
        latchUartFault();
        break;
      }
      if (uartRxBuffer.length() < UART_RX_LINE_MAX) {
        uartRxBuffer += c;
      } else {
        uartRxBuffer.remove(0);
        uartDiscardUntilNewline = true;
        uartCarriageReturnSeen = false;
        latchUartFault();
        break;
      }
      continue;
    }

    uartCarriageReturnSeen = false;
    uartRxBuffer.trim();
    const bool stopBoundary = uartRxBuffer.startsWith("STOP") ||
                              uartRxBuffer.startsWith("[RESET") ||
                              uartRxBuffer.startsWith("[OP:") ||
                              uartRxBuffer == "[BLE CONNECTION ESTABLISHED]";
    if (uartRxBuffer.length() > 0) processESPMessage(uartRxBuffer);
    // remove(0) retains the reserved allocation and prevents long-run heap churn.
    uartRxBuffer.remove(0);
    // Let the moving loop brake before later queued GO lines can clear STOP.
    if (stopBoundary) break;
  }
  // pollESP runs in every motion and clearance loop as well as loop().
  // Health timing and heading-stream freshness therefore continue during long
  // routes and obstacle holds.
  heading.serviceLink((uint32_t)millis());
  serviceNoticeMonitoring((uint32_t)millis());
  serviceBuzzerPattern();
  serviceDriveReport();
}

class ClearanceWaitNotice {
 public:
  explicit ClearanceWaitNotice(bool needsLocal) {
    haltWaitingForClearance = true;
    haltRequiresLocal = needsLocal;
  }
  ~ClearanceWaitNotice() {
    haltWaitingForClearance = haltRequiresLocal = false;
    reportMotionStatus();
  }
};

// Sonar clearance for a hold to end. Driving: a fresh forward sample beyond the
// clear distance and no side bearing blocking. Inside a turn: nothing inside the
// circle the base sweeps (frontObstacleDetected() checks exactly that there).
static bool sonarClearForResume() {
  if (!operatorSonarEnabled) return true;
  if (frontObstacleDetected()) return false;
  if (spinSonarCheck) return true;
  return frontDistanceFresh() && latestFrontDistance() >= ULTRASONIC_CLEAR_DISTANCE_CM;
}

// Every way out of a hold restores normal power first.
struct LowPowerScope {
  ~LowPowerScope() { exitLowPower(); }
};

static bool holdForHeading(bool verifyDirection = true);

void haltAndWait(const String& reason) {
  activeBrakeStopMotors();
  movingForward = false;
  commandServoTracked(DEFAULT_VIEW);

  blockedNoticeActive = true;
  blockedNoticeAtMs = (uint32_t)millis();
  if (!blockedSMSSent && !operatorEmergencyLatched && !resetQueued) {
    blockedSMSSent = queueSMSNotice(SmsNoticeKind::BLOCKED,
        "GARBY: Movement held by an obstacle or missing safety clearance.", 'D');
  }

  unsigned long lastSMS = millis();
  unsigned long lastRequestTime = 0;
  unsigned long lastLocalCheck = 0;
  unsigned long lastDiagnosticMs = 0;
  unsigned long lastClearEvidenceMs = millis();
  uint8_t clearanceCount = 0;
  unsigned long evaluatedPathCommandMs = 0;
  bool requiresLocal = frontObstacleDetected() ||
                             reason.indexOf("HUMAN") >= 0 ||
                             reason.indexOf("SONIC") >= 0;
  ClearanceWaitNotice clearanceNotice(requiresLocal);
  LowPowerScope power;

  Serial.println("[HALT] Waiting for confirmed clearance: " + reason);
  reportMotionStatus();
  while (true) {
    const unsigned long now = millis();
    // Fewer polls in low power: the Pi still streams path data at 4 Hz itself.
    const unsigned long pollMs = lowPowerActive ? LOW_POWER_STATUS_INTERVAL_MS : 100UL;
    if (now - lastRequestTime >= pollMs) {
      requestStatus();
      lastRequestTime = now;
    }

    pollESP();
    serviceUltrasonic();
    serviceBridgeRecovery();
    enforcePathWatchdog();

    if (frontObstacleDetected()) {
      requiresLocal = haltRequiresLocal = true;
    }

    if (resetQueued || routeFaultLatched) {
      blockedSMSSent = false;
      blockedNoticeActive = false;
      cancelPendingSms(SmsNoticeKind::BLOCKED);
      return;
    }

    // Return to the common motion gate even while STOP/sonar still holds us.
    // It owns heading recovery and its deadline; this clearance-only wait must
    // never hide an MPU outage or resume the wheels by itself.
    if (!heading.headingReady((uint32_t)millis())) return;

    bool newClearanceEvidence = false;
    bool localClear = true;
    if (requiresLocal && sonarEvidenceSequence != lastLocalCheck) {
      lastLocalCheck = sonarEvidenceSequence;
      localClear = sonarClearForResume();
      newClearanceEvidence = true;
    } else if (requiresLocal) {
      localClear = sonarClearForResume();
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

    // Low power only while nothing suggests the path is clearing. The first
    // clear evidence restores normal power, so the confirmations and the
    // restart run at full rate with the drivers energised.
    if (clearanceCount > 0) {
      lastClearEvidenceMs = now;
      exitLowPower();
    } else if (now - lastClearEvidenceMs >= LOW_POWER_AFTER_MS) {
      enterLowPower(reason.c_str());
    }

    if (clearanceCount >= ULTRASONIC_CLEAR_CONFIRM_SAMPLES) {
      Serial.println("[HALT] Clearance confirmed -- resuming");
      blockedSMSSent = false;
      blockedNoticeActive = false;
      cancelPendingSms(SmsNoticeKind::BLOCKED);
      queuePairedRecovery(SmsNoticeKind::BLOCKED,
          "GARBY: Obstacle hold cleared. Motion may resume with fresh safety clearance.");
      return;
    }

    if (now - lastDiagnosticMs >= 5000UL) {
      lastDiagnosticMs = now;
      Serial.printf("[HALT] path=%s sonar=%s distance=%.1fcm confirmations=%u/%u%s\n",
                    !pathCommandFresh() ? "STALE" : shouldStop ? "BLOCKED" : "CLEAR",
                    frontDistanceFresh() ? "VALID" : "UNKNOWN",
                    latestFrontDistance(), clearanceCount, ULTRASONIC_CLEAR_CONFIRM_SAMPLES,
                    lowPowerActive ? " power=LOW" : "");
    }

    if (now - lastSMS >= 600000UL) {
      lastSMS = now;
      queueSMSNotice(SmsNoticeKind::BLOCKED,
          "GARBY: Still blocked after a further 10 minutes. Check obstacles and controller safety status.");
    }

    vTaskDelay(pdMS_TO_TICKS(lowPowerActive ? 40 : 20));
  }
}

// Motion is permitted: fresh GO evidence and no local sonar obstacle.
static bool motionPermitted() {
  return !shouldStop && pathCommandFresh() && !frontObstacleDetected();
}

static String holdReasonNow(const char* where) {
  if (!shouldStop && pathCommandFresh() && frontObstacleDetected())
    return String("SONIC:BLOCKED ") + where;
  return motionHoldReason("LINK:WAITING_FOR_FRESH_PATH") + " " + where;
}

// Hold until motion is permitted (2026-11-12). Only an explicit reset, a latched
// route fault or the operator's emergency stop ends this without permission: a
// blocked or stale path is waited out (low power after a while) and the route
// then continues. Callers used to treat a STOP that arrived at the wrong moment
// (just after a hold, or while the start gate waited for a sonar sample) as a
// failed segment, which latched a route fault and kept the robot stopped for good.
static bool waitUntilMotionPermitted(const char* where) {
  while (!resetQueued && !routeFaultLatched && !operatorEmergencyLatched) {
    if (!heading.headingReady((uint32_t)millis())) {
      activeBrakeStopMotors();
      movingForward = false;
      if (!holdForHeading()) return false;
      continue;
    }
    if (motionPermitted()) return true;
    haltAndWait(holdReasonNow(where));
  }
  return false;
}

// ── movementGate ───────────────────────────────────────────────
bool movementGate(bool isNudgeEnabled) {
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
    if (resetQueued || operatorEmergencyLatched || routeFaultLatched) return false;
    if (!shouldStop && pathCommandFresh()) break;
    vTaskDelay(pdMS_TO_TICKS(10));
  } while (millis() - start < MOTION_GATE_TIMEOUT_MS);

  // Give the centred sonar one bounded opportunity to refresh before moving: it
  // is the only sensor below the LiDAR plane. No echo counts as a sample, so
  // this only waits for a ping that is already due.
  const unsigned long sampleWaitStarted = millis();
  while (operatorSonarEnabled && !frontDistanceFresh() &&
         millis() - sampleWaitStarted < ULTRASONIC_SAMPLE_MAX_AGE_MS) {
    pollESP();
    serviceUltrasonic();
    enforcePathWatchdog();
    if (resetQueued) return false;
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  return waitUntilMotionPermitted("before motion");
}

bool safeMoveDistance(int32_t steps, bool isADJ, bool isNudgeEnabled) {
  if (!movementGate(isNudgeEnabled)) return false;
  return moveDistance(steps, isADJ, isNudgeEnabled);
}

bool safeTurnLeft(float degrees) {
  return turnLeft(degrees);  // the turn waits for its own fresh clearance
}

bool safeTurnRight(float degrees) {
  return turnRight(degrees);
}

void fullReset() {
  const bool clearedRouteFault = routeFaultNoticeRaised && !operatorEmergencyLatched;
  shouldStop = true;
  lidarSlowActive = false;
  lidarNormalGoCount = 0;
  pathCommandSeen = false;
  linkFaultActive = true;
  lastPathCommandMs = 0;
  lastSensorPacketMs = 0;
  resetGoEvidence();
  sensorTripped = false;
  movingForward = false;
  blockedSMSSent = false;
  loadcellSMSSent = false;
  resetQueued = false;
  routeFaultLatched = false;
  routeFaultNoticeRaised = false;
  outboundComplete = false;
  garbyState = GarbyState::IDLE;
  Serial.println("[RESET] State cleared -- IDLE and fail-closed");
  ESP_Serial.println("[IDLE]");
  if (resetOutcome == ResetOutcome::ACTIVE) {
    resetOutcome = ResetOutcome::DONE;
    sendResetOutcome(resetRequestId, resetOutcome);
  }
  if (clearedRouteFault) {
    cancelPendingSms(SmsNoticeKind::ROUTE_FAULT);
    resetSmsTransition(SmsNoticeKind::ROUTE_FAULT, 'D');
    queuePairedRecovery(SmsNoticeKind::ROUTE_FAULT,
        "GARBY: Route fault cleared by reset. Verify the robot position before starting.");
  }
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
}

// Detect a pulse executor that reports running but makes no software progress.
// This cannot detect wheel slip or a mechanically stalled motor without encoders.
static long progressPosition[2] = {0, 0};
static unsigned long progressAtMs[2] = {0, 0};
static void resetMotionProgress() {
  progressPosition[0] = stepper1->getCurrentPosition();
  progressPosition[1] = stepper2->getCurrentPosition();
  progressAtMs[0] = progressAtMs[1] = millis();
}
static bool serviceMotionProgress() {
  FastAccelStepper* wheels[2] = {stepper1, stepper2};
  for (unsigned i = 0; i < 2; ++i) {
    const long position = wheels[i]->getCurrentPosition();
    if (!wheels[i]->isRunning() || position != progressPosition[i]) {
      progressPosition[i] = position;
      progressAtMs[i] = millis();
    } else if (millis() - progressAtMs[i] >= MOTOR_PROGRESS_TIMEOUT_MS) {
      emergencyStopMotors();
      routeFaultLatched = true;
      shouldStop = true;
      Serial.println("[MOTOR] No step-counter progress; route fault latched");
      reportRouteFault();
      return false;
    }
  }
  return true;
}

static bool commandWheelTargets(long target1, long target2) {
  // A live steering retarget must not refresh a stalled wheel's deadline.
  if (!motorsRunning()) resetMotionProgress();
  const auto result1 = stepper1->moveTo(target1);
  const auto result2 = stepper2->moveTo(target2);
  if (result1 != MOVE_OK || result2 != MOVE_OK) {
    // Never leave one wheel running when its partner rejected the command.
    emergencyStopMotors();
    Serial.println("[MOTOR] Wheel target rejected; both wheels stopped");
    return false;
  }
  return true;
}

// ── MPU-measured turns (merged from MPU_and_Motor-Latest/Calib) ─────────
// Turn targets are absolute along the route: each turn aims at the route's
// start heading +/- the sum of all turns so far, so heading drift picked up on
// a straight is removed by the next turn. Every halt, sonar and route-fault
// rule of the old step turns still applies between and inside spins.
enum class SpinResult : uint8_t { REACHED, WRONG_WAY, DATA_LOST, STEP_LIMIT, HALTED, FAILED };

// The bridge adds the LiDAR turn-sweep clearance (P frame T=) to its GO
// decision only between these notices. GO evidence received before
// [TURN:START] ignored that clearance, so the turn needs fresh confirmations.
class TurnSegmentNotice {
 public:
  TurnSegmentNotice() {
    ESP_Serial.println("[TURN:START]");
    if (!shouldStop) lastStopReason = "STOP:TURN_CLEARANCE";
    shouldStop = true;
    resetGoEvidence();
    // A turn moves no part of the base forward: from here the sonar evidence is
    // the circle the base sweeps, at floor level (the LiDAR's sweep is above it).
    spinSonarCheck = true;
    reportMotionStatus();
  }
  ~TurnSegmentNotice() {
    spinSonarCheck = false;
    ESP_Serial.println("[TURN:END]");
  }
};

// Spinning in place: the sonar pings forward only (the rotation itself sweeps it).
struct SpinInProgress {
  SpinInProgress() { spinInProgress = true; }
  ~SpinInProgress() { spinInProgress = false; }
};

// Before a spin the robot stands still while the sonar surveys its whole span,
// so the spin-circle check has every bearing (bounded wait; the hold that may
// follow keeps surveying).
#define SPIN_SURVEY_WAIT_MS 2500UL
static void waitForSpinSurvey() {
  if (!operatorSonarEnabled || operatorEmergencyLatched) return;
  const uint32_t since = (uint32_t)millis();
  while (sonarSweep.spinClearance(since, (uint32_t)millis()) == SonarSweep::Spin::UNKNOWN &&
         (uint32_t)millis() - since < SPIN_SURVEY_WAIT_MS) {
    pollESP();
    serviceUltrasonic();
    serviceBridgeRecovery();
    enforcePathWatchdog();
    if (resetQueued || routeFaultLatched) return;
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

static void serviceTurnSafety() {
  pollESP();
  serviceUltrasonic();
  serviceBridgeRecovery();
  enforcePathWatchdog();
}

static bool turnMustHalt() {
  return resetQueued || routeFaultLatched || shouldStop || !pathCommandFresh() ||
         frontObstacleDetected();
}

static bool setSpinSpeed(uint32_t speedHz) {
  if (stepper1->setSpeedInHz(speedHz) != 0 || stepper2->setSpeedInHz(speedHz) != 0)
    return false;
  stepper1->setAcceleration(ACCELERATION);
  stepper2->setAcceleration(ACCELERATION);
  stepper1->applySpeedAcceleration();
  stepper2->applySpeedAcceleration();
  return true;
}

static bool waitSpinStopped() {
  while (motorsRunning()) {
    if (!serviceMotionProgress()) return false;
    serviceTurnSafety();
    if (resetQueued || shouldStop) {
      activeBrakeStopMotors();
      return true;
    }
    delay(1);
  }
  return true;
}

// One spin in place toward the target, stopping on the MPU heading.
static SpinResult spinToward(float targetDeg, uint32_t maxSpeedHz) {
  SpinInProgress spinning;
  const float startHeading = heading.heading();
  const float startError = targetDeg - startHeading;
  const int dir = startError > 0.0f ? 1 : -1;  // + = left
  const long start1 = stepper1->getCurrentPosition();
  const long start2 = stepper2->getCurrentPosition();
  const float stepLimit = fabsf(startError) * heading.stepsPerDeg() *
                          HeadingControl::TURN_STEP_LIMIT_FACTOR + 300.0f;
  FastAccelStepper* left = heading.leftMotor();
  FastAccelStepper* right = heading.rightMotor();
  uint32_t speed = heading.turnProfileSpeed(fabsf(startError),
      heading.turnSideConfirmed() ? maxSpeedHz : HeadingControl::TURN_CORRECTION_SPEED_HZ,
      ACCELERATION);
  if (!setSpinSpeed(speed)) {
    emergencyStopMotors();
    return SpinResult::FAILED;
  }
  resetMotionProgress();
  const MoveResultCode leftResult = dir > 0 ? left->runBackward() : left->runForward();
  const MoveResultCode rightResult = dir > 0 ? right->runForward() : right->runBackward();
  if (leftResult != MOVE_OK || rightResult != MOVE_OK) {
    emergencyStopMotors();
    Serial.println("[MOTOR] Spin command rejected; both wheels stopped");
    return SpinResult::FAILED;
  }

  SpinResult result = SpinResult::REACHED;
  while (true) {
    if (!serviceMotionProgress()) return SpinResult::FAILED;
    serviceTurnSafety();
    if (turnMustHalt()) {
      activeBrakeStopMotors();
      return SpinResult::HALTED;
    }
    const uint32_t now = (uint32_t)millis();
    if (!heading.sampleFresh(now, HeadingControl::TURN_DATA_TIMEOUT_MS)) {
      result = SpinResult::DATA_LOST;
      break;
    }
    const float turned = (heading.heading() - startHeading) * dir;
    if (turned < -HeadingControl::TURN_WRONG_WAY_DEG) {
      result = SpinResult::WRONG_WAY;
      break;
    }
    const float spunSteps = 0.5f * (
        fabsf((float)(stepper1->getCurrentPosition() - start1)) +
        fabsf((float)(stepper2->getCurrentPosition() - start2)));
    if (spunSteps > stepLimit) {
      result = SpinResult::STEP_LIMIT;
      break;
    }
    const float remainingDeg = (targetDeg - heading.turnHeadingNow()) * dir;
    if (remainingDeg <= HeadingControl::TURN_STOP_DEG) break;
    // Until a turn proves which motor is left, creep for the first degrees: a
    // wrong guess then backs off about 1 degree instead of an 8-degree jerk.
    const bool sideUnproven = !heading.turnSideConfirmed() &&
                              turned < HeadingControl::TURN_WRONG_WAY_DEG;
    const uint32_t wanted = heading.turnProfileSpeed(remainingDeg,
        sideUnproven ? HeadingControl::TURN_CORRECTION_SPEED_HZ : maxSpeedHz, ACCELERATION);
    if (wanted != speed) {
      if (!setSpinSpeed(wanted)) {
        emergencyStopMotors();
        return SpinResult::FAILED;
      }
      speed = wanted;
    }
    delay(1);
  }
  stepper1->stopMove();
  stepper2->stopMove();
  if (!waitSpinStopped()) return SpinResult::FAILED;
  if (turnMustHalt()) return SpinResult::HALTED;
  return result;
}

static bool waitForTurnHeading(uint32_t timeoutMs) {
  const unsigned long started = millis();
  while (!heading.headingReady((uint32_t)millis())) {
    if (millis() - started >= timeoutMs) return false;
    serviceTurnSafety();
    if (resetQueued || routeFaultLatched || shouldStop) return false;
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  return true;
}

static int measureWallAxis(float* axisOut, uint32_t timeoutMs = HeadingControl::ALIGN_WAIT_MS);

// Motion without heading feedback is not allowed (2026-11-11). The wheels are at
// rest here. Wait for the stream, in low power after LOW_POWER_AFTER_MS; only
// HEADING_RECOVERY_MS without it fails the route (10 s until 2026-11-12, which
// turned every S3 brownout restart into a permanent stop). When the stream comes
// back as a new S3 reference after a long hold, the walls must confirm the route
// direction before the robot moves on: someone may have turned it meanwhile.
static bool holdForHeading(bool verifyDirection) {
  const unsigned long started = millis();
  const uint32_t anchorsBefore = heading.reanchors();
  Serial.println("[HEADING] MPU stream missing; holding until it returns");
  ESP_Serial.println("[HEADING LOST holding]");
  struct HoldScope {
    HoldScope() { headingHoldActive = true; }
    ~HoldScope() { headingHoldActive = false; }
  } hold;
  LowPowerScope power;
  bool recovered = false, verificationPending = false, reportedMissingWalls = false;
  unsigned long lastRequestMs = 0;
  while (!resetQueued && !routeFaultLatched && !operatorEmergencyLatched) {
    const unsigned long held = millis() - started;
    if (held >= HeadingControl::HEADING_RECOVERY_MS) {
      Serial.println("[HEADING] Heading recovery/direction confirmation timed out; route fault");
      ESP_Serial.println("[HEADING LOST fault]");
      return false;
    }
    if (held >= LOW_POWER_AFTER_MS) enterLowPower("heading recovery");
    if (millis() - lastRequestMs >= (lowPowerActive ? LOW_POWER_STATUS_INTERVAL_MS : 100UL)) {
      requestStatus();
      lastRequestMs = millis();
    }
    serviceTurnSafety();
    if (resetQueued || routeFaultLatched || operatorEmergencyLatched) return false;
    if (!heading.headingReady((uint32_t)millis())) {
      vTaskDelay(pdMS_TO_TICKS(lowPowerActive ? 20 : 2));
      continue;
    }
    if (!recovered) ESP_Serial.println("[HEADING LOST recovered]");
    recovered = true;
    verificationPending = verificationPending ||
        (verifyDirection && millis() - started >= HeadingControl::HEADING_VERIFY_AFTER_MS &&
         heading.reanchors() != anchorsBefore);
    if (!verificationPending) return true;

    // W measurements flow even during STOP. Do not wait for GO here: ordinary
    // obstacle holds remain independent, while direction evidence shares the
    // original five-minute recovery budget. Missing/disagreeing walls keep us
    // stationary, and no measurement pair may span a new heading reference.
    const uint32_t anchorsAtCheck = heading.reanchors();
    const uint32_t elapsed = (uint32_t)(millis() - started);
    if (elapsed >= HeadingControl::HEADING_RECOVERY_MS) continue;
    const uint32_t remaining = HeadingControl::HEADING_RECOVERY_MS - elapsed;
    float axis = 0.0f;
    const int measured = measureWallAxis(&axis, min(remaining, HeadingControl::ALIGN_WAIT_MS));
    if (measured == -2) return false;
    if (millis() - started >= HeadingControl::HEADING_RECOVERY_MS) continue;
    if (!heading.headingReady((uint32_t)millis()) || heading.reanchors() != anchorsAtCheck) continue;
    if (measured != 1) {
      if (!reportedMissingWalls) ESP_Serial.println("[HEADING LOST verify=WAIT_WALLS]");
      reportedMissingWalls = true;
      continue;
    }
    // Hallways meet at right angles, and an outage in the middle of a turn may
    // leave the old hallway's walls in view: compare with the nearest 90 degrees
    // of the route and correct only that residual.
    const float offRoute = HeadingControl::wrap180(axis - heading.routeTarget());
    const float offGrid = offRoute - 90.0f * roundf(offRoute / 90.0f);
    if (fabsf(offGrid) > HeadingControl::ALIGN_MAX_CORRECTION_DEG) {
      Serial.printf("[HEADING] After the outage the walls are %+.1f deg off the route grid: "
                    "direction unknown; route fault\n", offGrid);
      ESP_Serial.printf("[HEADING LOST verify=FAIL walls=%+.1f]\n", offGrid);
      return false;
    }
    heading.setRouteTarget(heading.routeTarget() + offGrid);
    ESP_Serial.printf("[HEADING LOST verify=OK walls=%+.1f]\n", offGrid);
    return true;
  }
  return false;
}

// Hallway direction from two independent LiDAR scans acquired after the spin
// stopped. 1 = measured, 0 = no walls, -1 = scans disagree, -2 = stop requested.
static int measureWallAxis(float* axisOut, uint32_t timeoutMs) {
  const uint32_t since = (uint32_t)millis();
  float first = 0.0f;
  uint32_t firstAcquisition = 0;
  bool haveFirst = false;
  while ((uint32_t)millis() - since < timeoutMs) {
    serviceTurnSafety();
    if (resetQueued || routeFaultLatched || operatorEmergencyLatched) return -2;
    float axis = 0.0f;
    uint32_t acquisition = 0;
    if (heading.wallAxisSince(since, (uint32_t)millis(), &axis, &acquisition) &&
        (!haveFirst || acquisition != firstAcquisition)) {
      if (!haveFirst) {
        first = axis;
        firstAcquisition = acquisition;
        haveFirst = true;
      } else {
        if (fabsf(axis - first) > HeadingControl::ALIGN_AGREE_DEG) return -1;
        *axisOut = 0.5f * (axis + first);
        return 1;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  return 0;
}

enum class AlignResult : uint8_t { NO_WALLS, INCONSISTENT, ALIGNED, CORRECTED, HALTED, FAILED };
static const char* alignText(AlignResult result) {
  switch (result) {
    case AlignResult::NO_WALLS:     return "NO_WALLS";
    case AlignResult::INCONSISTENT: return "INCONSISTENT";
    case AlignResult::ALIGNED:      return "OK";
    case AlignResult::CORRECTED:    return "CORRECTED";
    case AlignResult::HALTED:       return "HALTED";
    default:                        return "FAILED";
  }
}

// After the gyro says the turn is done, the walls say where the hallway really
// is. That removes the gyro's scale error and accumulated drift from the route:
// the next straight and the next turn start from the measured hallway axis.
// Without two agreeing scans nothing is changed.
static AlignResult alignToWalls(float gyroTargetDeg, float* beforeDeg, float* afterDeg) {
  *beforeDeg = *afterDeg = 0.0f;
  bool corrected = false;
  for (uint8_t pass = 0;; ++pass) {
    float axis = 0.0f;
    const int measured = measureWallAxis(&axis);
    if (measured == -2) return AlignResult::HALTED;
    if (measured <= 0) {
      if (corrected) return AlignResult::CORRECTED;
      return measured == 0 ? AlignResult::NO_WALLS : AlignResult::INCONSISTENT;
    }
    if (fabsf(axis - gyroTargetDeg) > HeadingControl::ALIGN_MAX_CORRECTION_DEG) {
      return corrected ? AlignResult::CORRECTED : AlignResult::INCONSISTENT;
    }
    const float residual = heading.heading() - axis;
    if (pass == 0) *beforeDeg = residual;
    *afterDeg = residual;
    heading.setRouteTarget(axis);
    if (fabsf(residual) <= HeadingControl::ALIGN_TOLERANCE_DEG) {
      return corrected ? AlignResult::CORRECTED : AlignResult::ALIGNED;
    }
    if (pass >= HeadingControl::ALIGN_MAX_PASSES) return AlignResult::CORRECTED;
    if (shouldStop || !pathCommandFresh() || frontObstacleDetected()) return AlignResult::HALTED;
    const SpinResult spin = spinToward(axis, HeadingControl::TURN_CORRECTION_SPEED_HZ);
    if (spin == SpinResult::HALTED) return AlignResult::HALTED;
    if (spin == SpinResult::DATA_LOST) return AlignResult::INCONSISTENT;
    if (spin != SpinResult::REACHED) return AlignResult::FAILED;
    corrected = true;
    responsiveDelay(HeadingControl::TURN_SETTLE_MS);
  }
}

// alignAfter: check the result against two wall scans and adopt the measured
// hallway direction (every route turn). The pre-run sidestep's first spin, which
// ends 35 degrees off the walls on purpose, skips it.
static bool rotateToHeading(float targetDeg, float commandedDeg, bool alignAfter = true) {
  if (!stepper1 || !stepper2) return false;
  heading.attach(stepper1, stepper2);
  commandServoTracked(DEFAULT_VIEW);
  waitForSpinSurvey();
  const float startHeading = heading.heading();
  const long start1 = stepper1->getCurrentPosition();
  const long start2 = stepper2->getCurrentPosition();
  bool flipped = false, clean = true;
  uint8_t passes = 0, spins = 0;
  while (passes < HeadingControl::TURN_MAX_PASSES) {
    if (resetQueued || routeFaultLatched) return false;
    const float frameBeforeGate = heading.routeTarget();
    if (!waitUntilMotionPermitted("during turn")) return false;
    targetDeg += heading.routeTarget() - frameBeforeGate;
    if (!waitForTurnHeading(HeadingControl::MPU_WAIT_MS)) {
      if (resetQueued || routeFaultLatched) return false;
      if (shouldStop || !pathCommandFresh()) continue;
      // Never finish a turn by wheel steps: hold, then fault if it stays blind.
      const float frameBefore = heading.routeTarget();
      if (!holdForHeading()) return false;
      // A post-outage wall check may have corrected the route frame: the turn's
      // target moves with it.
      targetDeg += heading.routeTarget() - frameBefore;
      clean = false;
      continue;
    }
    const float error = targetDeg - heading.heading();
    if (fabsf(error) <= HeadingControl::TURN_TOLERANCE_DEG) break;
    const uint32_t maxSpeed = (passes == 0 || fabsf(error) > 10.0f)
        ? HeadingControl::TURN_MAX_SPEED_HZ : HeadingControl::TURN_CORRECTION_SPEED_HZ;
    ++spins;
    const SpinResult result = spinToward(targetDeg, maxSpeed);
    if (result == SpinResult::HALTED) continue;  // the hold above owns the wait
    if (result == SpinResult::FAILED) return false;
    ++passes;
    if (result == SpinResult::WRONG_WAY) {
      // The motor-side guess was wrong; swap it once and retry.
      if (flipped || heading.turnSideConfirmed()) {
        Serial.println("[HEADING] TURN went the wrong way twice; route fault");
        return false;
      }
      heading.swapTurnSides();
      flipped = true;
      clean = false;
      responsiveDelay(HeadingControl::TURN_SETTLE_MS);
      continue;
    }
    if (result == SpinResult::STEP_LIMIT) {
      // 1.5x the expected steps without reaching the heading: slipping or lifted
      // wheels. The robot's direction is unknown, so never continue the route.
      Serial.println("[HEADING] TURN step limit: wheels slipping or not rotating the chassis");
      return false;
    }
    if (result == SpinResult::REACHED) heading.confirmTurnSides();
    else clean = false;  // DATA_LOST: the next pass re-reads the heading
    responsiveDelay(HeadingControl::TURN_SETTLE_MS);
  }
  movingForward = false;
  const float finalError = heading.heading() - targetDeg;
  const float gyroTurned = heading.heading() - startHeading;
  if (clean) {
    heading.learnStepsPerDeg(heading.heading() - startHeading,
                             (int32_t)(stepper1->getCurrentPosition() - start1),
                             (int32_t)(stepper2->getCurrentPosition() - start2));
  }
  if (flipped && heading.turnSideConfirmed()) {
    Serial.printf("[HEADING] first spin went the wrong way; set MOTOR1_IS_LEFT_GUESS = %s\n",
                  heading.motor1IsLeft() ? "true" : "false");
  }
  if (fabsf(finalError) > 5.0f) {
    Serial.println("[HEADING] TURN ended over 5 degrees from its target; route fault");
    ESP_Serial.printf("[HEADING TURN cmd=%+.1f gyro=%+.1f err=%+.2f align=NONE fault=1]\n",
                      commandedDeg, gyroTurned, finalError);
    return false;
  }
  float wallBefore = 0.0f, wallAfter = 0.0f;
  if (!alignAfter) {
    ESP_Serial.printf("[HEADING TURN cmd=%+.1f gyro=%+.1f err=%+.2f align=SKIP spins=%u]\n",
                      commandedDeg, gyroTurned, finalError, (unsigned)spins);
    return !resetQueued && !routeFaultLatched;
  }
  responsiveDelay(HeadingControl::TURN_SETTLE_MS);
  const AlignResult align = alignToWalls(targetDeg, &wallBefore, &wallAfter);
  // Commanded vs achieved. "gyro" is what the MPU measured; "wall_before" is how
  // far the walls say the robot was from the hallway direction when the gyro
  // called the turn done, and "wall_after" what remained after the correction.
  Serial.printf("[HEADING] TURN commanded %+.1f: gyro turned %+.1f (error %+.2f), walls %s "
                "%+.2f -> %+.2f deg, %u spin(s), %.1f steps/deg\n",
                commandedDeg, gyroTurned, finalError, alignText(align), wallBefore, wallAfter,
                (unsigned)spins, heading.stepsPerDeg());
  ESP_Serial.printf("[HEADING TURN cmd=%+.1f gyro=%+.1f err=%+.2f align=%s wall_before=%+.2f "
                    "wall_after=%+.2f spins=%u side=%s spd=%.1f]\n",
                    commandedDeg, gyroTurned, finalError, alignText(align), wallBefore, wallAfter,
                    (unsigned)spins, heading.motor1IsLeft() ? "M1L" : "M1R", heading.stepsPerDeg());
  return align != AlignResult::FAILED && !resetQueued && !routeFaultLatched;
}

bool turnLeft(float degrees) {
  if (!stepper1 || !stepper2 || !(degrees > 0.0f)) return false;
  TurnSegmentNotice notice;
  if (!movementGate(false)) return false;
  heading.addTurn(degrees);
  return rotateToHeading(heading.routeTarget(), degrees);
}

bool turnRight(float degrees) {
  if (!stepper1 || !stepper2 || !(degrees > 0.0f)) return false;
  TurnSegmentNotice notice;
  if (!movementGate(false)) return false;
  heading.addTurn(-degrees);
  return rotateToHeading(heading.routeTarget(), -degrees);
}

// A spin to an absolute heading that is not a route turn (pre-run centring). The
// route frame is unchanged unless alignAfter adopts the measured hallway axis.
static bool spinToHeading(float targetDeg, bool alignAfter) {
  if (!stepper1 || !stepper2) return false;
  TurnSegmentNotice notice;
  const float frameBeforeGate = heading.routeTarget();
  if (!movementGate(false)) return false;
  targetDeg += heading.routeTarget() - frameBeforeGate;
  return rotateToHeading(targetDeg, targetDeg - heading.heading(), alignAfter);
}

// A route anchors its heading frame here. Without a confirmed MPU stream the
// route does not start: turns and straights both need heading feedback.
bool beginRouteHeading() {
  if (stepper1 && stepper2) heading.attach(stepper1, stepper2);
  const unsigned long started = millis();
  while (!heading.headingReady((uint32_t)millis()) &&
         millis() - started < HeadingControl::START_MPU_WAIT_MS) {
    pollESP();
    serviceUltrasonic();
    serviceBridgeRecovery();
    enforcePathWatchdog();
    if (resetQueued) break;
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  bool ready = heading.headingReady((uint32_t)millis());
  // A bridge restarting just as the route starts is waited out like one during
  // the route (low power, bounded by HEADING_RECOVERY_MS), not refused at once.
  if (!ready && !resetQueued && !operatorEmergencyLatched) ready = holdForHeading(false);
  heading.beginRoute((uint32_t)millis());
  Serial.printf("[HEADING] Route frame %s (%lu packets OK, %lu rejected)\n",
                ready ? "anchored on MPU" : "NOT anchored: no MPU stream, route refused",
                (unsigned long)heading.accepted(), (unsigned long)heading.rejected());
  ESP_Serial.printf("[HEADING ROUTE:%s ok=%lu bad=%lu]\n", ready ? "MPU" : "NO_MPU",
                    (unsigned long)heading.accepted(), (unsigned long)heading.rejected());
  return ready;
}

// ============================================================
// PRE-RUN CENTRING (2026-11-12, the owner's idea)
// ============================================================
// Before the long hallway run the robot stands still and the LiDAR walls put it
// on the hallway's centre line and axis: the straight then starts with nothing to
// correct, and its own centring (which engages at 5 cm) does not weave down the
// hallway correcting a start error. A turn that came out a few degrees or a first
// leg that ended off centre is fixed here, standing still, where it costs least.
struct HallwayFix {
  float lateralCm = 0.0f;   // + = left of centre
  float axisDeg = 0.0f;     // hallway direction, gyro frame
  float widthCm = 0.0f;
};

// Two independent wall scans taken while standing still: both walls and the
// hallway heading present, a plausible width, agreeing within PRECENTER_AGREE_CM
// and ALIGN_AGREE_DEG. 1 = measured, 0 = no usable walls, -2 = stop requested.
static int measureHallwayFix(HallwayFix* out) {
  const uint32_t since = (uint32_t)millis();
  HeadingControl::WallView first;
  bool haveFirst = false;
  while ((uint32_t)millis() - since < HeadingControl::PRECENTER_WAIT_MS) {
    pollESP();
    serviceUltrasonic();
    serviceBridgeRecovery();
    enforcePathWatchdog();
    if (resetQueued || routeFaultLatched) return -2;
    HeadingControl::WallView view;
    if (heading.wallViewSince(since, (uint32_t)millis(), &view) && view.both() &&
        view.hasHeading() && view.widthCm() >= HeadingControl::WIDTH_MIN_CM &&
        view.widthCm() <= HeadingControl::WIDTH_MAX_CM &&
        (!haveFirst || view.acquisition != first.acquisition)) {
      if (haveFirst &&
          fabsf(view.lateralCm() - first.lateralCm()) <= HeadingControl::PRECENTER_AGREE_CM &&
          fabsf(HeadingControl::wrap180(view.axisDeg() - first.axisDeg())) <=
              HeadingControl::ALIGN_AGREE_DEG) {
        out->lateralCm = 0.5f * (view.lateralCm() + first.lateralCm());
        out->axisDeg = first.axisDeg() + 0.5f * HeadingControl::wrap180(view.axisDeg() - first.axisDeg());
        out->widthCm = 0.5f * (view.widthCm() + first.widthCm());
        return 1;
      }
      first = view;  // no pair yet, or it disagreed: the newest scan starts a new pair
      haveFirst = true;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  return 0;
}

// False only when the route must stop (reset, fault, a failed spin or move).
// No usable walls is not an error: the straight's own centring remains.
// *progressSteps = forward travel along the hallway the sidesteps used, which the
// caller takes off the straight so the run still ends at its calibrated point.
bool centerBeforeLongRun(int32_t* progressSteps) {
  if (progressSteps) *progressSteps = 0;
  if (!stepper1 || !stepper2) return false;
  const float theta = HeadingControl::PRECENTER_ANGLE_DEG * DEG_TO_RAD;
  float distanceScale = 1.0f;     // actual/assumed wheel travel, learned from a sidestep
  float previousLateral = 0.0f;
  float previousScale = 1.0f;
  bool moved = false;
  for (uint8_t pass = 0; pass <= HeadingControl::PRECENTER_MAX_PASSES; ++pass) {
    responsiveDelay(HeadingControl::TURN_SETTLE_MS);  // stand still before measuring
    if (resetQueued || routeFaultLatched) return false;
    HallwayFix fix;
    const int measured = measureHallwayFix(&fix);
    if (measured == -2) return false;
    if (measured != 1) {
      ESP_Serial.println(moved ? "[HEADING CENTER stop=NO_WALLS]" : "[HEADING CENTER skip=NO_WALLS]");
      return true;
    }
    const float offRoute = HeadingControl::wrap180(fix.axisDeg - heading.routeTarget());
    if (fabsf(offRoute) > HeadingControl::ALIGN_MAX_CORRECTION_DEG) {
      ESP_Serial.printf("[HEADING CENTER skip=WALLS_OFF_ROUTE walls=%+.1f]\n", offRoute);
      return true;
    }
    if (fabsf(fix.lateralCm) > HeadingControl::PRECENTER_MAX_CM) {
      ESP_Serial.printf("[HEADING CENTER skip=OFFSET y=%+.1f w=%.0f]\n", fix.lateralCm, fix.widthCm);
      return true;
    }
    if (moved) {
      // What the last sidestep achieved against what it asked for.
      const float wanted = -previousLateral;
      const float got = fix.lateralCm - previousLateral;
      // Beyond the walls' noise it moved the wrong way, or hardly at all: a sign
      // or calibration problem to find on the bench, never to keep correcting.
      const bool wrongWay = (got * wanted < 0.0f && fabsf(got) > 1.5f) ||
                            (fabsf(wanted) >= 6.0f && fabsf(got) < 0.25f * fabsf(wanted));
      if (wrongWay) {
        ESP_Serial.printf("[HEADING CENTER stop=WRONG_WAY y=%+.1f was=%+.1f]\n",
                          fix.lateralCm, previousLateral);
        return true;
      }
      if (fabsf(wanted) >= HeadingControl::PRECENTER_LEARN_MIN_CM && got * wanted > 0.0f) {
        const float k = constrain(got / (wanted * previousScale), 0.5f, 2.0f);
        distanceScale = 1.0f / k;
        ESP_Serial.printf("[HEADING CENTER learn k=%.2f]\n", k);
      }
    }
    heading.setRouteTarget(fix.axisDeg);
    const float headingError = HeadingControl::wrap180(heading.heading() - fix.axisDeg);
    ESP_Serial.printf("[HEADING CENTER pass=%u y=%+.1f hdg=%+.2f w=%.0f]\n",
                      (unsigned)(pass + 1U), fix.lateralCm, headingError, fix.widthCm);
    if (fabsf(fix.lateralCm) <= HeadingControl::PRECENTER_TOLERANCE_CM) {
      // On the centre line: face the hallway if the turn left more than the
      // alignment tolerance, then the straight starts with nothing to correct.
      if (fabsf(headingError) > HeadingControl::ALIGN_TOLERANCE_DEG &&
          !spinToHeading(fix.axisDeg, true)) {
        return false;
      }
      ESP_Serial.printf("[HEADING CENTER done y=%+.1f sidesteps=%u]\n", fix.lateralCm, (unsigned)pass);
      return true;
    }
    if (pass == HeadingControl::PRECENTER_MAX_PASSES) break;
    // Sidestep toward the centre: + lateral (left of centre) turns right (-yaw).
    const float side = fix.lateralCm > 0.0f ? -1.0f : 1.0f;
    const float diagonalCm = fabsf(fix.lateralCm) / sinf(theta) * distanceScale;
    const int32_t steps = (int32_t)lroundf(diagonalCm * HeadingControl::STEPS_PER_CM);
    if (diagonalCm < HeadingControl::PRECENTER_MIN_MOVE_CM || steps < 50) break;
    ESP_Serial.printf("[HEADING CENTER move=%s %.1fcm at %.0f deg]\n", side < 0.0f ? "R" : "L",
                      diagonalCm, HeadingControl::PRECENTER_ANGLE_DEG);
    if (!spinToHeading(fix.axisDeg + side * HeadingControl::PRECENTER_ANGLE_DEG, false)) return false;
    if (!safeMoveDistance(steps, true, false)) return false;
    if (!spinToHeading(fix.axisDeg, true)) return false;
    if (progressSteps) *progressSteps += (int32_t)lroundf((float)steps * cosf(theta));
    previousLateral = fix.lateralCm;
    previousScale = distanceScale;
    moved = true;
  }
  ESP_Serial.println("[HEADING CENTER stop=PASSES]");
  return true;
}

void emergencyStopMotors() {
  if (stepper1) stepper1->forceStopAndNewPosition(stepper1->getCurrentPosition());
  if (stepper2) stepper2->forceStopAndNewPosition(stepper2->getCurrentPosition());
  movingForward = false;
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
  const auto result1 = stepper1->runForward();
  const auto result2 = stepper2->runForward();
  if (result1 != MOVE_OK || result2 != MOVE_OK) {
    emergencyStopMotors();
    Serial.println("[MOTOR] Continuous start rejected; both wheels stopped");
    return;
  }
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
  if (!commandWheelTargets(target1, target2)) return false;
  movingForward = true;
  return true;
}

static uint32_t trimmedMotorSpeed(uint32_t baseSpeed, float trimPct) {
  trimPct = constrain(trimPct, -10.0f, 10.0f);
  return (uint32_t)((float)baseSpeed * (1.0f + trimPct / 100.0f));
}

// ── servo tracking ────────────────────────────────────────────
static int servoCommandAngle = DEFAULT_VIEW;
static unsigned long servoReadyAtMs = 0;

static void commandServoTracked(int angle) {
  if (!operatorSonarEnabled || operatorEmergencyLatched) angle = DEFAULT_VIEW;
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

// One forward-bearing sample. validEcho=false is a ping with no echo inside the
// timeout (about 3 m): nothing in range, so it is a far, clear reading. Until
// 2026-11-12 it was "unknown" and reset the clear streak, so an obstacle that
// walked away into an open hallway could never release the latch and the robot
// stayed held for good (also after the operator re-enabled the sonar). Release
// still takes ULTRASONIC_CLEAR_CONFIRM_SAMPLES consecutive clear samples, and a
// hold still needs fresh LiDAR clearance as well. An echo under 1.5 cm is a
// glitch and is ignored.
static void recordFrontSample(float distanceCm, bool validEcho = true) {
  if (!operatorSonarEnabled) return;
  if (validEcho && !(isfinite(distanceCm) && distanceCm >= 1.5f && distanceCm <= 999.0f)) return;
  if (!validEcho) distanceCm = 999.0f;
  // Confirmation and filtering must never combine samples across an outage.
  if (!frontDistanceFresh()) {
    ultrasonicClearCount = ultrasonicBlockedCount = 0;
    frontFilterCount = frontFilterIndex = 0;
  }
  lastFrontSampleMs = millis();
  frontSampleSequence++;
  sonarEvidenceSequence++;
  latestFrontSampleValid = true;

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
  ultrasonicPingCommand = servoCommandAngle;
}

void initUltrasonicMonitor() {
  digitalWrite(TRIG_PIN, LOW);
  portENTER_CRITICAL(&ultrasonicMux);
  ultrasonicAwaitingEcho = false;
  ultrasonicEchoRiseSeen = false;
  ultrasonicEchoReady = false;
  portEXIT_CRITICAL(&ultrasonicMux);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), ultrasonicEchoISR, CHANGE);
  sonarSweep.reset();
  commandServoTracked(DEFAULT_VIEW);
}

// Which sweep the sonar runs now (SonarSweep.h). parked = forward only, slowly.
static SonarSweep::Pattern sonarPatternNow(bool* parked) {
  *parked = false;
  // The emergency stop parks the servo forward (commandServoTracked forces it).
  if (operatorEmergencyLatched) {
    *parked = true;
    return SonarSweep::Pattern::FORWARD;
  }
  if (spinInProgress) return SonarSweep::Pattern::FORWARD;
  if (movingForward) return SonarSweep::Pattern::CRUISE;
  const bool routeActive = !routeFaultLatched &&
      ((garbyState == GarbyState::RUNNING && !outboundComplete) ||
       garbyState == GarbyState::RETURNING);
  if (!routeActive) {
    // Idle at base or parked at the destination: no reason to wear the servo.
    *parked = true;
    return SonarSweep::Pattern::FORWARD;
  }
  if (lowPowerActive) {
    // One survey pass every few seconds keeps a long hold aware of its sides.
    const unsigned long now = millis();
    if (!surveyPassActive && now - lastSurveyPassMs >= LOW_POWER_SURVEY_EVERY_MS) {
      surveyPassActive = true;
      surveyPassSteps = 0;
      lastSurveyPassMs = now;
      sonarSweep.restartSurvey();
    }
    if (surveyPassActive) return SonarSweep::Pattern::SURVEY;
    *parked = true;
    return SonarSweep::Pattern::FORWARD;
  }
  surveyPassActive = false;
  return SonarSweep::Pattern::SURVEY;
}

static void sonarSampleDone(int command, bool echo, float cm, SonarSweep::Pattern pattern) {
  const uint32_t now = (uint32_t)millis();
  if (command == DEFAULT_VIEW) recordFrontSample(cm, echo);
  else sonarEvidenceSequence++;
  sonarSweep.record(command, echo, cm, now);
  if (command == sonarSweep.target(pattern)) {
    sonarSweep.advance(pattern);
    if (surveyPassActive && pattern == SonarSweep::Pattern::SURVEY &&
        ++surveyPassSteps >= sonarSweep.surveySteps()) {
      surveyPassActive = false;
    }
  }
  // A survey pass completed: one USB line for bench checks (never over BLE).
  static unsigned long lastSummaryMs = 0;
  if (pattern == SonarSweep::Pattern::SURVEY && command == SonarSweep::RIGHT_LIMIT_CMD &&
      now - lastSummaryMs >= 5000UL) {
    lastSummaryMs = now;
    char line[120];
    sonarSweep.summary(line, sizeof(line));
    Serial.printf("[SONAR SWEEP %s]\n", line);
  }
  if (sonarSweep.silentNow(echo, pattern == SonarSweep::Pattern::SURVEY)) {
    // Three full surveys without an echo at any bearing: unplugged, broken or a
    // truly empty space. It cannot hold the robot (no echo is not an obstacle).
    Serial.println("[SONAR] No echo at any bearing in three full surveys: check the sensor");
    ESP_Serial.println("[SONAR:SILENT]");
  }
}

void serviceUltrasonic() {
  if (!operatorSonarEnabled) return;
  bool echoReady = false;
  uint32_t echoWidthUs = 0;
  portENTER_CRITICAL(&ultrasonicMux);
  if (ultrasonicEchoReady) {
    echoReady = true;
    echoWidthUs = ultrasonicEchoWidthUs;
    ultrasonicEchoReady = false;
  }
  portEXIT_CRITICAL(&ultrasonicMux);

  bool parked = false;
  const SonarSweep::Pattern pattern = sonarPatternNow(&parked);
  if (echoReady) {
    ultrasonicPingInFlight = false;
    sonarSampleDone(ultrasonicPingCommand, true, (float)echoWidthUs * 0.0343f * 0.5f, pattern);
  } else if (ultrasonicPingInFlight &&
             (uint32_t)(micros() - ultrasonicPingStartedUs) >= ULTRASONIC_TIMEOUT_US) {
    portENTER_CRITICAL(&ultrasonicMux);
    ultrasonicAwaitingEcho = false;
    ultrasonicEchoRiseSeen = false;
    ultrasonicEchoReady = false;
    portEXIT_CRITICAL(&ultrasonicMux);
    ultrasonicPingInFlight = false;
    sonarSampleDone(ultrasonicPingCommand, false, 999.0f, pattern);
  }
  if (ultrasonicPingInFlight) return;

  // Aim first; ping once the servo has settled there.
  const int aim = sonarSweep.target(pattern);
  if (servoCommandAngle != aim) {
    commandServoTracked(aim);
    return;
  }
  const unsigned long now = millis();
  const unsigned long interval = parked ? ULTRASONIC_PARKED_PING_INTERVAL_MS
                                        : ULTRASONIC_PING_INTERVAL_MS;
  if (!servoSettled() || now - lastUltrasonicPingMs < interval) return;
  // The module ignores a trigger while ECHO is still high from its last ping
  // (some hold it ~38 ms or longer when nothing echoes). Wait for it; if it never
  // drops, count the ping as no echo so a hold cannot wait on a stuck line.
  if (digitalRead(ECHO_PIN) == HIGH) {
    if (ultrasonicEchoHighSinceMs == 0) {
      ultrasonicEchoHighSinceMs = now;
    } else if (now - ultrasonicEchoHighSinceMs >= ULTRASONIC_ECHO_STUCK_MS) {
      ultrasonicEchoHighSinceMs = 0;
      lastUltrasonicPingMs = now;
      static unsigned long lastStuckLogMs = 0;
      if (lastStuckLogMs == 0 || now - lastStuckLogMs >= 30000UL) {
        lastStuckLogMs = now;
        Serial.println("[SONAR] ECHO line stuck high: check the sensor wiring");
        ESP_Serial.println("[SONAR:FAULT]");
      }
      sonarSampleDone(aim, false, 999.0f, pattern);
    }
    return;
  }
  ultrasonicEchoHighSinceMs = 0;
  startUltrasonicPing();
}

bool frontObstacleDetected() {
  if (!operatorSonarEnabled) return false;
  const uint32_t now = (uint32_t)millis();
  // Inside a turn nothing moves forward: only what lies inside the circle the
  // base sweeps can be hit (a wall 50 cm ahead of the sonar cannot).
  if (spinSonarCheck) {
    return sonarSweep.spinClearance(0, now) == SonarSweep::Spin::BLOCKED;
  }
  // Once confirmed, remain fail-closed until confirmed clear samples arrive.
  // A fresh single echo inside the stop envelope also stops immediately; the
  // confirmation latch remains responsible for persistence and recovery. The
  // +/-25 (and, while fresh, wider) bearings add what is low beside the path.
  return frontObstacleLatched ||
         (frontDistanceFresh() && latestFrontRawCm <= ULTRASONIC_STOP_DISTANCE_CM) ||
         sonarSweep.sideBlocking(now);
}

bool frontEmergencyObstacleDetected() {
  return operatorSonarEnabled && frontEmergencyActive;
}

bool frontDistanceFresh() {
  return operatorSonarEnabled && latestFrontSampleValid && lastFrontSampleMs != 0 &&
         millis() - lastFrontSampleMs <= ULTRASONIC_SAMPLE_MAX_AGE_MS;
}

float latestFrontDistance() {
  return frontDistanceFresh() ? frontDistance : 999.0f;
}

// Keep the bridge's LiDAR centering scoped to one forward segment. The
// destructor pairs END with START on every exit; obstacle waits inside the
// segment keep it. START carries this controller's error from the route heading
// (centidegrees), so the bridge judges centering against the same target.
class HeadingSegmentNotice {
 public:
  explicit HeadingSegmentNotice(bool enabled) : enabled_(enabled) {
    if (enabled_ && heading.headingReady((uint32_t)millis())) {
      const long errorCdeg = constrain(lroundf(heading.routeError() * 100.0f), -18000L, 18000L);
      ESP_Serial.printf("[HEADING:START:%ld]\n", errorCdeg);
    } else {
      ESP_Serial.println(enabled_ ? "[HEADING:START]" : "[HEADING:END]");
    }
  }
  ~HeadingSegmentNotice() {
    if (enabled_) ESP_Serial.println("[HEADING:END]");
  }
 private:
  bool enabled_;
};

// Short, reverse and calibration moves: one plain ramp, both wheels the same
// step count, no steering. Every obstacle/stop hold still applies.
static bool movePlain(long target1, long target2, const char* label) {
  HeadingSegmentNotice headingNotice(false);
  commandServoTracked(DEFAULT_VIEW);
  stepper1->setSpeedInHz(MAX_SPEED);
  stepper1->setAcceleration(ACCELERATION);
  stepper2->setSpeedInHz(MAX_SPEED);
  stepper2->setAcceleration(ACCELERATION);
  if (!commandWheelTargets(target1, target2)) return false;
  movingForward = true;
  // Resume after a hold: wait for permission, restart toward the same targets. A
  // STOP that lands between the two just holds again; only a rejected wheel
  // command, an explicit reset or a latched fault ends the move.
  auto resume = [&]() -> bool {
    while (true) {
      if (!waitUntilMotionPermitted(label)) return false;
      if (startTargetMove(target1, target2)) return true;
      if (motionPermitted()) return false;  // permitted, yet the wheels refused
    }
  };
  while (motorsRunning()) {
    if (!serviceMotionProgress()) return false;
    pollESP();
    serviceUltrasonic();
    serviceBridgeRecovery();
    enforcePathWatchdog();
    if (resetQueued) {
      activeBrakeStopMotors();
      return false;
    }
    if (shouldStop || frontObstacleDetected() ||
        !heading.sampleFresh((uint32_t)millis(), HeadingControl::HEADING_LOSS_HALT_MS)) {
      activeBrakeStopMotors();
      if (!resume()) return false;
      continue;
    }
    delay(1);
  }
  movingForward = false;
  return stepper1->getCurrentPosition() == target1 &&
         stepper2->getCurrentPosition() == target2;
}

// A forward straight longer than FINISH_STEPS: cruise with the MPU route-heading
// hold, then hand the last FINISH_STEPS to the stepper ramp. Both wheels travel
// the same remaining distance, so the mean step count ends exactly and the
// steering difference already applied is kept. Halts resume from the remaining
// mean distance, so a stop never replays or loses commanded travel.
static bool driveStraight(int32_t steps, bool centering) {
  heading.attach(stepper1, stepper2);
  HeadingSegmentNotice headingNotice(centering);
  commandServoTracked(DEFAULT_VIEW);
  const long start1 = stepper1->getCurrentPosition();
  const long start2 = stepper2->getCurrentPosition();
  heading.beginStraight((uint32_t)millis(), centering);
  heading.setProbeReserve(heading.probeReserveSteps(MAX_SPEED, FINISH_STEPS));
  motionBaseSpeed = MAX_SPEED;
  bool cruising = false;
  bool sonarSlowHold = false;
  bool realigned = false;
  unsigned long lastFusionReportMs = millis();
  auto remaining = [&]() -> int32_t {
    const int64_t travelled = ((int64_t)(stepper1->getCurrentPosition() - start1) +
                               (int64_t)(stepper2->getCurrentPosition() - start2)) / 2;
    return (int32_t)((int64_t)steps - travelled);
  };
  auto base1 = [&]() { return trimmedMotorSpeed(motionBaseSpeed, MOTOR1_TRIM_PCT); };
  auto base2 = [&]() { return trimmedMotorSpeed(motionBaseSpeed, MOTOR2_TRIM_PCT); };
  auto finish = [&](bool ok) {
    realigned = heading.endStraight((uint32_t)millis());
    movingForward = false;
    return ok;
  };
  auto startPhase = [&]() -> bool {
    enforcePathWatchdog();
    if (!motionPermitted()) return false;
    const int32_t left = remaining();
    if (left <= 0) return true;
    stepper1->setSpeedInHz(base1());
    stepper1->setAcceleration(ACCELERATION);
    stepper2->setSpeedInHz(base2());
    stepper2->setAcceleration(ACCELERATION);
    if (left > FINISH_STEPS) {
      if (!motorsRunning()) resetMotionProgress();
      const auto result1 = stepper1->runForward();
      const auto result2 = stepper2->runForward();
      if (result1 != MOVE_OK || result2 != MOVE_OK) {
        emergencyStopMotors();
        Serial.println("[MOTOR] Continuous start rejected; both wheels stopped");
        return false;
      }
      cruising = true;
      heading.resumeCruise((uint32_t)millis());
    } else {
      if (!commandWheelTargets(stepper1->getCurrentPosition() + left,
                               stepper2->getCurrentPosition() + left)) return false;
      cruising = false;
    }
    movingForward = true;
    return true;
  };
  // Start or restart from the remaining mean distance once motion is permitted.
  // A STOP that lands in between just holds again (2026-11-12): only a rejected
  // wheel command, an explicit reset or a latched fault ends the straight.
  auto resume = [&]() -> bool {
    while (true) {
      if (!waitUntilMotionPermitted("during straight")) return false;
      if (startPhase()) return true;
      if (motionPermitted()) return false;  // permitted, yet the wheels refused
    }
  };
  if (!resume()) return finish(false);
#if GARBY_VERBOSE_DIAGNOSTICS
  Serial.printf("[MOVE] Straight %ld steps, heading hold %s\n", (long)steps,
                heading.steeringOn() ? "ON" : "pending probe");
#endif

  unsigned long lastRequestTime = 0;
  while (motorsRunning()) {
    if (!serviceMotionProgress()) return finish(false);
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
      heading.pauseCruise((uint32_t)millis());
      smoothDecelStopMotors();
      Serial.println("[MOVE] Explicit reset -- controlled stop");
      return finish(false);
    }

    // STOP and the sonar have priority over steering. The hold beeps its own
    // pattern and drops to low power if it lasts; the straight then resumes.
    if (shouldStop || frontObstacleDetected()) {
      heading.pauseCruise((uint32_t)millis());
      activeBrakeStopMotors();
      if (!resume()) return finish(false);
      commandServoTracked(DEFAULT_VIEW);
      lastRequestTime = millis();
      continue;
    }

    // Heading feedback lost while moving (the bridge also sends STOP:MPU for a
    // sensor fault): brake, wait for it, then require fresh path clearance.
    if (!heading.sampleFresh((uint32_t)millis(), HeadingControl::HEADING_LOSS_HALT_MS)) {
      heading.pauseCruise((uint32_t)millis());
      activeBrakeStopMotors();
      if (!holdForHeading()) return finish(false);
      // The same permission as the start of any move (and its own hold if the
      // path lapsed meanwhile), then the remaining distance.
      if (!resume()) return finish(false);
      lastRequestTime = millis();
      continue;
    }

    {
      // Caution speed for the sonar's slow band or the LiDAR's caution zone.
      if (!operatorSonarEnabled) {
        sonarSlowHold = false;
      } else {
        // The +/-25 degree bearings watch the body's corners: something low
        // there within the caution distance slows the robot as well.
        const bool sideSlow = sonarSweep.sideSlow((uint32_t)now);
        if (frontDistanceFresh()) {
          const float filteredDistance = latestFrontDistance();
          if (filteredDistance <= ULTRASONIC_SLOW_DISTANCE_CM || sideSlow) sonarSlowHold = true;
          else if (filteredDistance >= ULTRASONIC_RESUME_SPEED_DISTANCE_CM) sonarSlowHold = false;
        } else if (sideSlow) {
          sonarSlowHold = true;
        }
      }
      const uint32_t wanted = (sonarSlowHold || lidarSlowActive) ? CAUTION_SPEED : MAX_SPEED;
      if (wanted != motionBaseSpeed) {
        // A speed-zone change is safety-critical: drop any probe and apply the
        // same new base to both wheels before steering resumes from it.
        motionBaseSpeed = wanted;
        heading.pauseCruise((uint32_t)millis());
        stepper1->setSpeedInHz(base1());
        stepper1->setAcceleration(ACCELERATION);
        stepper1->applySpeedAcceleration();
        stepper2->setSpeedInHz(base2());
        stepper2->setAcceleration(ACCELERATION);
        stepper2->applySpeedAcceleration();
      }
    }

    if (centering && now - lastFusionReportMs >= 3000UL) {
      lastFusionReportMs = now;
      char line[176];
      if (heading.fusionReport((uint32_t)now, line, sizeof(line))) ESP_Serial.println(line);
    }

    if (cruising) {
      const int32_t left = remaining();
      if (left <= FINISH_STEPS) {
        heading.pauseCruise((uint32_t)millis());
        const int32_t rest = left > 0 ? left : 0;
        stepper1->setSpeedInHz(base1());
        stepper2->setSpeedInHz(base2());
        if (!commandWheelTargets(stepper1->getCurrentPosition() + rest,
                                 stepper2->getCurrentPosition() + rest)) return finish(false);
        cruising = false;
      } else if (!heading.serviceCruise((uint32_t)millis(), base1(), base2(), left, ACCELERATION)) {
        emergencyStopMotors();
        Serial.println("[MOTOR] Steering speed rejected; both wheels stopped");
        return finish(false);
      }
    }
    delay(1);
  }

  if (motorsRunning()) smoothDecelStopMotors();
  commandServoTracked(DEFAULT_VIEW);
  const bool mpu = heading.headingReady((uint32_t)millis());
  const float error = heading.routeError();
  const bool hold = heading.steeringOn();
  const int32_t left = remaining();
  finish(true);
  if (mpu) {
    Serial.printf("[HEADING] STRAIGHT %ld steps: heading error %+.1f deg, hold %s\n",
                  (long)steps, error, hold ? "ON" : "OFF");
  } else {
    Serial.printf("[HEADING] STRAIGHT %ld steps: no MPU data, hold OFF\n", (long)steps);
  }
  ESP_Serial.printf("[HEADING STRAIGHT steps=%ld err=%+.1f hold=%d mpu=%d lidar_scans=%u fused=%u "
                    "realign=%+.2f width=%.0f]\n",
                    (long)steps, mpu ? error : 0.0f, hold ? 1 : 0, mpu ? 1 : 0,
                    (unsigned)heading.segmentWallScans(), (unsigned)heading.segmentAxisUpdates(),
                    realigned ? heading.lastRealignDeg() : 0.0f,
                    heading.widthKnown() ? heading.widthCm() : 0.0f);
  return left == 0;
}

bool moveToTarget(long target1, long target2, bool isADJ, bool isNudgeEnabled) {
  if (!stepper1 || !stepper2) return false;
  if (!waitUntilMotionPermitted("before motion")) return false;
  const int64_t delta1 = (int64_t)target1 - stepper1->getCurrentPosition();
  const int64_t delta2 = (int64_t)target2 - stepper2->getCurrentPosition();
  if (!isADJ && delta1 == delta2 && delta1 > FINISH_STEPS && delta1 <= INT32_MAX) {
    return driveStraight((int32_t)delta1, isNudgeEnabled);
  }
  return movePlain(target1, target2, isADJ ? "adjustment" : "short move");
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
  if (!operatorSonarEnabled) return 999.0f;
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
  if (!operatorSonarEnabled) return 999.0f;
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
  if (!operatorSonarEnabled || operatorEmergencyLatched) {
    commandServoTracked(DEFAULT_VIEW);
    return 999.0f;
  }
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
struct ModemReply {
  String text;
  bool ok = false, error = false, prompt = false, reference = false;
};

static ModemReply readModemReply(uint32_t timeout, bool needPrompt = false, bool needSubmission = false) {
  ModemReply reply;
  reply.text.reserve(768);
  String line;
  line.reserve(128);
  bool lineOverflow = false;
  const uint32_t started = (uint32_t)millis();
  while ((uint32_t)((uint32_t)millis() - started) < timeout) {
    uint16_t budget = 256;
    while (budget-- && Air780.available()) {
      const char c = (char)Air780.read();
      if ((c < 32 || c > 126) && c != '\r' && c != '\n' && c != '\t') continue;
      // Leave bounded room for a final response even after a long URC burst.
      if (reply.text.length() < 600U) reply.text += c;
      if (needPrompt && c == '>') {
        line.trim();
        if (!lineOverflow && line.length() == 0) { reply.prompt = true; return reply; }
      }
      if (c == '\r') continue;
      if (c != '\n') {
        if (line.length() < 128U) line += c;
        else lineOverflow = true;
        continue;
      }
      line.trim();
      if (!lineOverflow) {
        if (line.startsWith("+CMGS:")) {
          String reference = line.substring(6); reference.trim();
          bool digits = reference.length() > 0;
          for (unsigned int i = 0; i < reference.length(); ++i)
            if (reference.charAt(i) < '0' || reference.charAt(i) > '9') digits = false;
          if (digits) reply.reference = true;
        }
        reply.error = line == "ERROR" || line.startsWith("+CMS ERROR:") || line.startsWith("+CME ERROR:");
        reply.ok = line == "OK" && (!needSubmission || reply.reference);
        if (reply.ok || reply.error) {
          // Ensure sendAT callers see the actual terminal line after truncation.
          reply.text += "\n" + line + "\n";
          return reply;
        }
      }
      line.remove(0); lineOverflow = false;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  return reply;
}

static bool atResponseLine(const String& response, const char* expected) {
  unsigned int start = 0;
  while (start < response.length()) {
    int end = response.indexOf("\n", start);
    if (end < 0) end = response.length();
    String line = response.substring(start, end); line.trim();
    if (line == expected) return true;
    start = (unsigned int)end + 1U;
  }
  return false;
}

static void discardModemInput() {
  // Bounded even if a failing device emits continuous noise.
  uint16_t budget = 1024;
  while (budget-- && Air780.available()) Air780.read();
}

String sendAT(const String& cmd, unsigned long timeout) {
  discardModemInput();
  Air780.println(cmd);
#if GARBY_VERBOSE_DIAGNOSTICS
  Serial.println(">> " + cmd);
#endif
  const ModemReply reply = readModemReply((uint32_t)timeout);
#if GARBY_VERBOSE_DIAGNOSTICS
  Serial.println("<< " + reply.text);
#endif
  return reply.text;
}

void powerOnAir780() {
  pinMode(PWRKEY_PIN, OUTPUT);
  digitalWrite(PWRKEY_PIN, HIGH);
  for (int i = 0; i < 3; ++i) {
    if (atResponseLine(sendAT("AT", 1000), "OK")) return;
    vTaskDelay(pdMS_TO_TICKS(500));
  }
  Serial.println("[Air780E] Checking power-on in background");
  digitalWrite(PWRKEY_PIN, LOW);
  vTaskDelay(pdMS_TO_TICKS(1500));
  digitalWrite(PWRKEY_PIN, HIGH);
  vTaskDelay(pdMS_TO_TICKS(8000));
}

bool waitForModule(int maxAttempts) {
  for (int i = 0; i < maxAttempts; ++i) {
    if (atResponseLine(sendAT("AT", 2000), "OK")) return true;
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  return false;
}

static bool registeredResponse(const String& response) {
  if (!atResponseLine(response, "OK")) return false;
  unsigned int start = 0;
  while (start < response.length()) {
    int end = response.indexOf("\n", start);
    if (end < 0) end = response.length();
    String line = response.substring(start, end); line.trim();
    if (line.startsWith("+CREG:")) {
      const int comma = line.indexOf(',');
      if (comma >= 0) {
        String registration = line.substring(comma + 1);
        const int extra = registration.indexOf(',');
        if (extra >= 0) registration = registration.substring(0, extra);
        registration.trim();
        if (registration == "1" || registration == "5") return true;
      }
    }
    start = (unsigned int)end + 1U;
  }
  return false;
}

bool waitForNetwork(int maxAttempts) {
  for (int i = 0; i < maxAttempts; ++i) {
    if (registeredResponse(sendAT("AT+CREG?", 3000))) return true;
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
  return false;
}

static void modemInitializationTask(void*) {
  bool firstInitialization = true, startupNoticeQueued = false;
  uint32_t retryAt = (uint32_t)millis(), retryDelay = 30000UL;
  uint32_t lastNetworkCheck = (uint32_t)millis();
  for (;;) {
    const uint32_t now = (uint32_t)millis();
    if (!modemReadyFlag && (int32_t)(now - retryAt) >= 0) {
      modemServiceBusy = true;
      // Only the first attempt may pulse PWRKEY. Later retries never power-cycle
      // a module that has merely lost service or produced an uncertain result.
      if (firstInitialization) { powerOnAir780(); firstInitialization = false; }
      const bool responding = waitForModule(3);
      if (responding) {
        sendAT("ATE0", 1000);
        sendAT("AT+CFUN=1", 3000);
      }
      modemReadyFlag = responding && waitForNetwork(3);
      modemServiceBusy = false;
      if (modemReadyFlag) {
        retryDelay = 30000UL;
        lastNetworkCheck = (uint32_t)millis();
        Serial.println("[Air780E] SMS network registered");
        if (!startupNoticeQueued)
          startupNoticeQueued = queueSMSNotice(SmsNoticeKind::STARTUP,
              "GARBY: Controller restarted. Cellular SMS registered; robot motion remains subject to safety checks.");
      } else {
        retryAt = (uint32_t)millis() + retryDelay;
        retryDelay = min(retryDelay * 2UL, 300000UL);
        Serial.println("[Air780E] SMS service unavailable; retrying in background");
      }
    }
    if (modemReadyFlag) {
      serviceSmsQueueOnce((uint32_t)millis());
      if ((uint32_t)((uint32_t)millis() - lastNetworkCheck) >= 60000UL) {
        modemServiceBusy = true;
        modemReadyFlag = waitForNetwork(1);
        modemServiceBusy = false;
        lastNetworkCheck = (uint32_t)millis();
        if (!modemReadyFlag) retryAt = lastNetworkCheck + 30000UL;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void startModemInitialization() {
  // The flag changes before task creation: no main-loop or console code can
  // acquire the UART during the scheduler handoff to its sole modem owner.
  static bool attempted = false;
  static uint32_t lastAttempt = 0;
  const uint32_t now = (uint32_t)millis();
  portENTER_CRITICAL(&smsMux);
  if (modemWorkerStarted || (attempted && (uint32_t)(now - lastAttempt) < 30000UL)) {
    portEXIT_CRITICAL(&smsMux); return;
  }
  attempted = true; lastAttempt = now;
  modemWorkerStarted = true;
  portEXIT_CRITICAL(&smsMux);
  if (xTaskCreate(modemInitializationTask, "garby-modem", 7168,
                  nullptr, 1, nullptr) != pdPASS) {
    modemWorkerStarted = false;
    Serial.println("[Air780E] Could not start modem owner; retrying later");
  }
}

SmsSubmitResult sendSMS(const String& phoneNumber, const String& message) {
  // Only the persistent modem owner calls this function in production.
  // Queue validation has already bounded and sanitized the message body.
  const String mode = sendAT("AT+CMGF=1", 3000);
  if (!atResponseLine(mode, "OK")) {
    Serial.println("[SMS] Text mode unavailable before submission");
    return SmsSubmitResult::RETRYABLE_FAILURE;
  }
  discardModemInput();
  Air780.println("AT+CMGS=\"" + phoneNumber + "\"");
  const ModemReply prompt = readModemReply(5000, true);
  if (!prompt.prompt) {
    Air780.write(0x1B);
    Serial.println("[SMS] No message prompt; body was not submitted");
    return SmsSubmitResult::RETRYABLE_FAILURE;
  }
  Air780.print(message);
  Air780.write(0x1A);
  const ModemReply result = readModemReply(15000, false, true);
  if (result.reference && result.ok) {
    Serial.println("[SMS] Submitted to cellular network; handset delivery is unconfirmed");
    return SmsSubmitResult::SUBMITTED;
  }
  if (result.error) {
    Serial.println("[SMS] Modem rejected submission; no automatic post-body resend");
    return SmsSubmitResult::FAILED;
  }
  Serial.println("[SMS] Submission result unknown; no automatic resend");
  return SmsSubmitResult::INDETERMINATE;
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
// ============================================================
// BUZZER (2026-11-12)
// ============================================================
// One persistent low-priority task plays the pattern for the robot's state, so
// the beeping never depends on which motion loop is running. It used to start a
// new task for every beep, and only while held.
static volatile BuzzerPattern buzzerPattern = BuzzerPattern::OFF;
static volatile uint32_t buzzerPatternStartMs = 0;
static volatile BuzzerAlert buzzerPendingAlert = BuzzerAlert::NONE;
#define BUZZER_READY_ON_MS 400UL

bool buzzerPatternLevel(BuzzerPattern pattern, uint32_t msIntoPattern) {
  switch (pattern) {
    case BuzzerPattern::RUNNING:
      return msIntoPattern % BUZZER_RUN_PERIOD_MS < BUZZER_RUN_ON_MS;
    case BuzzerPattern::HOLDING: {
      const uint32_t t = msIntoPattern % BUZZER_HOLD_PERIOD_MS;
      return t < BUZZER_HOLD_ON_MS ||
             (t >= BUZZER_HOLD_ON_MS + BUZZER_HOLD_GAP_MS &&
              t < 2UL * BUZZER_HOLD_ON_MS + BUZZER_HOLD_GAP_MS);
    }
    case BuzzerPattern::LOW_POWER:
      return msIntoPattern % BUZZER_LOW_POWER_PERIOD_MS < BUZZER_LOW_POWER_ON_MS;
    default:
      return false;
  }
}

static void buzzerWrite(bool on) {
  digitalWrite(BUZZER_PIN, (on == (BUZZER_ACTIVE_HIGH != 0)) ? HIGH : LOW);
}

static void buzzerTaskLoop(void* parameters) {
  (void)parameters;
  bool level = false;
  bool alertActive = false;
  uint32_t alertUntilMs = 0;
  buzzerWrite(false);
  for (;;) {
    const uint32_t now = (uint32_t)millis();
    if (buzzerPendingAlert != BuzzerAlert::NONE) {
      buzzerPendingAlert = BuzzerAlert::NONE;
      alertActive = true;
      alertUntilMs = now + BUZZER_READY_ON_MS;
    }
    bool want;
    if (alertActive && (int32_t)(alertUntilMs - now) > 0) {
      want = true;
    } else {
      alertActive = false;
      want = buzzerPatternLevel(buzzerPattern, now - buzzerPatternStartMs);
    }
    if (want != level) {
      level = want;
      buzzerWrite(level);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void startBuzzer() {
  static bool started = false;
  if (started) return;
  buzzerWrite(false);
  started = xTaskCreate(buzzerTaskLoop, "garby-buzzer", 2048, nullptr, 1, nullptr) == pdPASS;
  if (!started) Serial.println("[BUZZER] Task could not start; buzzer silent");
}

void buzzerAlert(BuzzerAlert alert) { buzzerPendingAlert = alert; }

BuzzerPattern currentBuzzerPattern() { return buzzerPattern; }

// Called from pollESP(), which every motion and hold loop runs. Driving a route:
// RUNNING. Held by an obstacle or waiting for the heading: HOLDING. Low power:
// LOW_POWER. Idle, parked, faulted or emergency-stopped: silent.
static void serviceBuzzerPattern() {
  BuzzerPattern want = BuzzerPattern::OFF;
  const bool routeActive = !operatorEmergencyLatched && !routeFaultLatched &&
      ((garbyState == GarbyState::RUNNING && !outboundComplete) ||
       garbyState == GarbyState::RETURNING);
  if (routeActive) {
    if (lowPowerActive) want = BuzzerPattern::LOW_POWER;
    else if (haltWaitingForClearance || headingHoldActive) want = BuzzerPattern::HOLDING;
    else want = BuzzerPattern::RUNNING;
  }
  if (want != buzzerPattern) {
    // Restart the phase so a new pattern begins with its first beep.
    buzzerPatternStartMs = (uint32_t)millis();
    buzzerPattern = want;
  }
}

// ============================================================
// LOW-POWER HOLD (2026-11-12)
// ============================================================
static void motorDriversEnable(bool on) {
#if MOTOR_ENABLE_PIN >= 0
  const bool level = MOTOR_ENABLE_ACTIVE_LOW ? !on : on;
  digitalWrite(MOTOR_ENABLE_PIN, level ? HIGH : LOW);
  if (on) delay(MOTOR_ENABLE_SETTLE_MS);
#else
  (void)on;
#endif
}

void initPowerControl() {
#if MOTOR_ENABLE_PIN >= 0
  pinMode(MOTOR_ENABLE_PIN, OUTPUT);
#endif
  motorDriversEnable(true);
}

static void enterLowPower(const char* why) {
  if (lowPowerActive) return;
  lowPowerActive = true;
  motorDriversEnable(false);
  setCpuFrequencyMhz(LOW_POWER_CPU_MHZ);
  Serial.printf("[POWER] Low-power hold: %s\n", why ? why : "held");
  ESP_Serial.println("[POWER:LOW]");
}

static void exitLowPower() {
  if (!lowPowerActive) return;
  setCpuFrequencyMhz(EXECUTOR_CPU_MHZ);
  motorDriversEnable(true);
  lowPowerActive = false;
  surveyPassActive = false;
  Serial.println("[POWER] Normal power");
  ESP_Serial.println("[POWER:NORMAL]");
}

bool lowPowerHoldActive() { return lowPowerActive; }

// ============================================================
// DRIVE REPORT (2026-11-12)
// ============================================================
// What the wheels are doing, for the Pi's moving-object prediction (it must tell
// the robot's own motion from a person's): [DRIVE:S] stopped, [DRIVE:F:<cm/s>]
// straight (signed), [DRIVE:T:<deg/s>] spinning (+ = left). Sent on a change and
// every 2 s; it is telemetry only and grants nothing.
static void serviceDriveReport() {
  static unsigned long lastSampleMs = 0, lastSentMs = 0;
  static char lastMode = 0;
  static long lastValue = 0;
  const unsigned long now = millis();
  if (!stepper1 || !stepper2 || now - lastSampleMs < 100UL) return;
  lastSampleMs = now;
  const float s1 = (float)stepper1->getCurrentSpeedInMilliHz(false) * 0.001f;
  const float s2 = (float)stepper2->getCurrentSpeedInMilliHz(false) * 0.001f;
  char mode;
  long value = 0;
  if (fabsf(s1) < 40.0f && fabsf(s2) < 40.0f) {
    mode = 'S';
  } else if ((s1 >= 0.0f) == (s2 >= 0.0f)) {
    mode = 'F';
    value = lroundf(0.5f * (s1 + s2) / HeadingControl::STEPS_PER_CM);
  } else {
    mode = 'T';
    const float left = heading.motor1IsLeft() ? s1 : s2;
    const float right = heading.motor1IsLeft() ? s2 : s1;
    value = lroundf(0.5f * (right - left) / heading.stepsPerDeg());
  }
  const bool changed = mode != lastMode || labs(value - lastValue) >= (mode == 'T' ? 10L : 8L);
  if (!changed && now - lastSentMs < 2000UL) return;
  if (changed && mode == lastMode && now - lastSentMs < 150UL) return;
  lastMode = mode;
  lastValue = value;
  lastSentMs = now;
  if (mode == 'S') ESP_Serial.println("[DRIVE:S]");
  else ESP_Serial.printf("[DRIVE:%c:%ld]\n", mode, value);
}
bool checkLoad(float threshold) {
  float kilograms;
  uint32_t sequence;
  return isfinite(threshold) && threshold >= 0.0f &&
      getLoadCellSample(kilograms, sequence) && kilograms >= threshold;
}
