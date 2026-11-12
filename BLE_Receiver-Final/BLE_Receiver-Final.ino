// Target: ESP32-S3 Dev Module (44 header pins; use GPIO/IO labels).
// MPU6050: 3V3/GND, SDA=IO8, SCL=IO9; level mounting with +Z upward.
// UART to controller: RX=IO18, TX=IO17. Native USB keeps IO19/IO20.
#include <Arduino.h>
#if defined(ARDUINO_ARCH_ESP32) && !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Build the BLE bridge for ESP32-S3 Dev Module (esp32:esp32:esp32s3)."
#endif
#include "MpuHeading.h"
#include "SteeringAssist.h"
#include <HardwareSerial.h>
#include <NimBLEDevice.h>
#include <nvs_flash.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#if defined(ARDUINO_ARCH_ESP32)
#include <esp_system.h>
#endif
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

HardwareSerial ESP_Serial(1);

// ===== BLE UUIDs =====
#define SERVICE_UUID      "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define WRITE_CHAR_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define NOTIFY_CHAR_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a9"

// ===== UART to Controller ESP32 =====
#define ESP_RX 18
#define ESP_TX 17
#define MPU_SDA_PIN 8
#define MPU_SCL_PIN 9
// Normalizes the sensor so positive heading always means LEFT everywhere below.
static constexpr bool MPU_REVERSE_Z = !YAW_POSITIVE_IS_LEFT;
static MpuHeading mpuHeading;
// Executor notices. Neither changes the heading reference: the bridge keeps one
// continuous reference from boot and the executor owns its segment/route frames.
static bool straightRunActive = false;
static bool turnActive = false;
static bool haveSteeringAcquisition = false;
static uint32_t lastSteeringAcquisition = 0;
static uint8_t steeringAllow = 0;               // STEER_ALLOW_* of the newest acquisition
static uint32_t steeringAllowAtMs = 0;
static uint32_t currentFrameReceivedAtMs = 0;   // BLE ingress time of the frame being processed
static uint32_t headingSequence = 0;
static uint32_t headingEpoch = 0;               // numbers the S3 heading reference; never 0
static uint32_t lastHeadingSendMs = 0;
static uint32_t lastMpuFaultSendMs = 0;
static uint32_t wallLinesSent = 0;
static uint32_t stopLatchedSinceMs = 0;
static bool stopLatchSeen = false;
static bool bridgeMpuLinePending = false;
static uint32_t bridgeMpuLineDueMs = 0;
// The executor's own state line ([IDLE], [OUTBOUND COMPLETE], [ROUTE FAULT] mean
// its wheels are not commanded). It is sent on every change and as a 5 s
// heartbeat; a stale one is treated as unknown.
static bool executorParked = false;
static uint32_t executorParkedSinceMs = 0;
static uint32_t executorStateAtMs = 0;

// ===== Safety watchdogs =====
// Path freshness is safety-critical and therefore uses a much shorter timeout
// than general BLE liveness. Any stale path stream produces STOP, never GO or
// an automatic return movement.
#define PATH_DATA_TIMEOUT_MS        850UL
#define LINK_DATA_TIMEOUT_MS      10000UL
#define BLE_SETUP_TIMEOUT_MS      45000UL
#define ADVERTISING_RETRY_MS       2000UL
#define STALE_STOP_REPEAT_MS        500UL
#define CLEAR_CONFIRM_PACKETS          2
#define VERBOSE_PROTOCOL_LOGS           0

// BLE callbacks run in the NimBLE host task. Keep that task deterministic:
// copy writes into fixed-size queues and do parsing/UART work from loop().
// Requested MTU is 247; cap writes at 244 bytes and notifications at 20 bytes
// because the central may keep the default ATT MTU of 23.
#define BLE_FRAME_MAX_BYTES           244U
#define BRIDGE_DIAG_SETTLE_MS         2000UL
#define CONTROL_QUEUE_DEPTH              8U
#define CONTROL_DRAIN_BUDGET              4U
#define PATH_FRAME_MAX_AGE_MS          650UL
#define SIDES_FRAME_MAX_AGE_MS         500UL
#define SENSOR_FRAME_MAX_AGE_MS       2000UL
#define HEALTH_FRAME_MAX_AGE_MS      15000UL
#define MCU_UART_LINE_MAX_BYTES         256U
#define MCU_UART_RX_BYTE_BUDGET         384U
#define MCU_ACK_TIMEOUT_MS             3000UL
#define MCU_MAX_UNACKED_COMMANDS           8U
#define MCU_ACK_RESYNC_QUIET_MS          100UL
#define TRANSPORT_STATS_INTERVAL_MS   10000UL
#define BRIDGE_CPU_FREQUENCY_MHZ         160
#define BRIDGE_TX_POWER_DBM                9
#define BRIDGE_FIRMWARE_ID       "NAV-20261111"

// Steering is continuous in the executor (HeadingControl.h). Bridge steering
// tunables, the permission rules and the W line format are in SteeringAssist.h.
// Motion is never released before the bias is measured and the heading is valid.
#define STILL_BIAS_SETTLE_MS            1000UL  // STOP held, or executor parked, this long before bias refresh
#define EXECUTOR_STATE_MAX_AGE_MS      12000UL  // two missed heartbeats: parked state unknown

// ===== BLE globals =====
NimBLEServer*          pServer     = nullptr;
NimBLECharacteristic* pNotifyChar = nullptr;
volatile bool deviceConnected     = false;
volatile bool sentConnectedMsg    = false;
volatile bool bleBridgeSignalSent = false;  // Track if we told the MCU about BLE link
volatile bool mcuReady            = false;  // Fail closed until explicit [MCU READY]
static volatile bool bleSessionChanged = false;
static volatile bool readyNoticePending = false;
static volatile int lastDisconnectReason = 0;
static volatile bool bleWriteSeen = false;
static volatile unsigned long connectionStartedMs = 0;
static bool piReady = false;
static bool connectionParamsPending = false;
static volatile bool connectionInfoPending = false;
static volatile uint16_t connectionIntervalUnits = 0;
static volatile uint16_t connectionTimeoutUnits = 0;
static volatile uint16_t connectionLatency = 0;
static const char* bootResetLabel = "HOST_TEST";

uint16_t      activeConnHandle = 0;
volatile unsigned long lastDataReceived = 0;  // any bounded, non-empty BLE write
unsigned long lastPathDataReceived = 0;  // valid, in-order path packet only
unsigned long lastStaleStopSent = 0;
bool          pathStreamSeen = false;
bool          stopLatched = true;
uint8_t       clearConfirmCount = 0;
uint32_t      lastPathSeq = 0;
uint32_t      lastSidesSeq = 0;
bool          havePathSeq = false;
bool          haveSidesSeq = false;

// ── Fixed-capacity BLE ingress ───────────────────────────────
// Safety/control packets retain order. Steering and telemetry are latest-value
// mailboxes, so a producer burst cannot build an unbounded stale backlog.
struct BleFrame {
  uint32_t receivedAtMs;
  uint32_t session;
  uint16_t length;
  char payload[BLE_FRAME_MAX_BYTES + 1U];
};

static StaticQueue_t controlQueueState;
static StaticQueue_t sidesQueueState;
static StaticQueue_t sensorQueueState;
static StaticQueue_t healthQueueState;
static uint8_t controlQueueStorage[CONTROL_QUEUE_DEPTH * sizeof(BleFrame)];
static uint8_t sidesQueueStorage[sizeof(BleFrame)];
static uint8_t sensorQueueStorage[sizeof(BleFrame)];
static uint8_t healthQueueStorage[sizeof(BleFrame)];
static QueueHandle_t controlQueue = nullptr;
static QueueHandle_t sidesQueue   = nullptr;
static QueueHandle_t sensorQueue  = nullptr;
static QueueHandle_t healthQueue  = nullptr;

static volatile uint32_t connectionSession       = 0;
static volatile uint32_t ingressFaultCount       = 0;
static volatile uint32_t controlOverflowCount    = 0;
static volatile uint32_t oversizeFrameCount      = 0;
static volatile uint32_t unknownFrameCount       = 0;
static volatile uint32_t replacedSidesCount      = 0;
static volatile uint32_t replacedSensorCount     = 0;
static volatile uint32_t mcuLineOverflowCount    = 0;
static volatile uint32_t mcuBackpressureCount    = 0;
static volatile uint32_t invalidSteeringFieldCount = 0;
static uint32_t notificationFailureCount         = 0;
// Field diagnostics. The USB console is normally unattended on the robot, so the
// facts that separate a power reset from a radio drop must reach the Pi's log.
static volatile bool connParamsReportPending     = false;
static bool bridgeDiagPending                    = false;
static uint32_t bridgeDiagDueMs                  = 0;
static uint32_t linkDropCount                    = 0;
static unsigned long lastLinkDurationMs          = 0;
static uint32_t mpuFaultCount                    = 0;
static uint32_t mcuAckResyncCount                = 0;
static unsigned long lastMcuSendMs               = 0;
static unsigned long lastMcuAckMs                = 0;
static uint32_t handledIngressFaultCount         = 0;

// ── Controller UART delivery health ─────────────────────────
// The executor already returns the legacy generic [ESP RECEIVED] ACK. We use
// it only as a liveness signal; no new UART or BLE protocol field is required.
static volatile bool mcuAckPending       = false;
static volatile bool mcuAckFault         = false;
static volatile bool mcuBackpressureStopPending = false;
static volatile uint8_t mcuUnackedCommands = 0;
static volatile unsigned long mcuAckPendingSince = 0;
static StaticSemaphore_t mcuTxMutexState;
static SemaphoreHandle_t mcuTxMutex = nullptr;

// Operator STOP survives a software restart; normal path/session recovery is
// deliberately unable to clear it. Only a correlated executor RELEASE does so.
// RTC_DATA_ATTR is initialized on ordinary restart. NOINIT retains a marker;
// validate it before restoring RAM state. An accidental cold-boot match is
// conservative: it can only request STOP, never permission to move.
RTC_NOINIT_ATTR static volatile uint32_t operatorStopMarker;
static constexpr uint32_t OPERATOR_STOP_MARKER = 0x47525350U;
static volatile bool operatorStopLatched = false;
static portMUX_TYPE operatorMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t operatorStopGeneration = 0;
static volatile bool operatorExecutorStopped = false;
static StaticQueue_t emergencyQueueState;
static uint8_t emergencyQueueStorage[sizeof(BleFrame)];
static QueueHandle_t emergencyQueue = nullptr;
static String pendingEmergencyId;
static uint32_t pendingEmergencyGeneration = 0;
static unsigned long lastEmergencySendMs = 0;
static String pendingReleaseId;
static uint32_t pendingReleaseGeneration = 0;
static String pendingRebootId;
static uint32_t pendingRebootGeneration = 0;
static bool operatorRebootScheduled = false;
static unsigned long operatorRebootAtMs = 0;
static unsigned long pendingRebootSentAtMs = 0;

// Keep sketch types above the first function: Arduino inserts auto-prototypes
// there, including declarations of functions whose parameters use BleFrame.
static bool lineIsGo(const char* line) {
  return strcmp(line, "GO") == 0 || strcmp(line, "GO:SLOW") == 0;
}

static void noteMcuCommandQueued(uint8_t count) {
  if (!mcuAckPending) {
    mcuAckPending = true;
    mcuAckPendingSince = millis();
  }
  lastMcuSendMs = millis();
  const uint16_t next = (uint16_t)mcuUnackedCommands + count;
  mcuUnackedCommands = (next > 255U) ? 255U : (uint8_t)next;
}

static bool commandRequiresLiveBle(const char* line) {
  if (lineIsGo(line)) return true;
  if (strncmp(line, "[START:", 7) == 0) return true;
  if (strncmp(line, "[START STATUS:", 14) == 0) return true;
  return strncmp(line, "[OP:", 4) == 0 && strstr(line, ":ESTOP]") == nullptr;
}

static bool commandIsSafetyPriority(const char* line) {
  return strncmp(line, "STOP", 4) == 0 ||
         strncmp(line, "[RESET", 6) == 0 ||
         (strncmp(line, "[OP:", 4) == 0 && strstr(line, ":ESTOP]") != nullptr);
}

static bool sendMcuLine(const char* line, bool expectsAck) {
  if (line == nullptr || line[0] == '\0') return false;
  if (mcuTxMutex != nullptr) xSemaphoreTake(mcuTxMutex, portMAX_DELAY);
  // Re-check after acquiring the UART lock. If disconnect raced a queued GO or
  // start, the disconnecting task wins and the stale motion command is dropped.
  if (commandRequiresLiveBle(line) && !deviceConnected) {
    if (mcuTxMutex != nullptr) xSemaphoreGive(mcuTxMutex);
    return false;
  }
  if (operatorStopLatched && (lineIsGo(line) ||
      strncmp(line, "[START:", 7) == 0 || strncmp(line, "[RESET", 6) == 0)) {
    if (mcuTxMutex != nullptr) xSemaphoreGive(mcuTxMutex);
    return false;
  }
  if (expectsAck && !commandIsSafetyPriority(line) &&
      mcuUnackedCommands >= MCU_MAX_UNACKED_COMMANDS) {
    // Never grow a stale UART backlog if the executor pauses. Safety commands
    // bypass this cap; ordinary GO/telemetry is dropped and loop() emits an
    // ordered STOP immediately afterward.
    stopLatched = true;
    clearConfirmCount = 0;
    mcuBackpressureStopPending = true;
    mcuBackpressureCount++;
    if (mcuTxMutex != nullptr) xSemaphoreGive(mcuTxMutex);
    return false;
  }
  ESP_Serial.println(line);
  if (expectsAck) noteMcuCommandQueued(1U);
  if (mcuTxMutex != nullptr) xSemaphoreGive(mcuTxMutex);
  return true;
}

static bool sendMcuLine(const String& line, bool expectsAck) {
  return sendMcuLine(line.c_str(), expectsAck);
}

static void sendMcuStop(const char* reason) {
  if (reason == nullptr || reason[0] == '\0') return;
  if (mcuTxMutex != nullptr) xSemaphoreTake(mcuTxMutex, portMAX_DELAY);
  ESP_Serial.println(reason);
  noteMcuCommandQueued(1U);
  if (mcuTxMutex != nullptr) xSemaphoreGive(mcuTxMutex);
}

static void relayStop(const char* reason);

static void resetNavigationState() {
  stopLatched            = true;
  clearConfirmCount      = 0;
  pathStreamSeen         = false;
  havePathSeq            = false;
  haveSidesSeq           = false;
  lastPathSeq            = 0;
  lastSidesSeq           = 0;
  lastPathDataReceived   = millis();
  lastStaleStopSent      = 0;
  haveSteeringAcquisition = false;
  lastSteeringAcquisition = 0;
  steeringAllow          = 0;
  steeringAllowAtMs      = 0;
  turnActive             = false;
}

void sendNotification(const char* msg) {
  if (!deviceConnected || pNotifyChar == nullptr || msg == nullptr) return;
  const size_t length = strnlen(msg, BLE_FRAME_MAX_BYTES + 1U);
  if (length == 0 || length > BLE_FRAME_MAX_BYTES) {
    oversizeFrameCount++;
    return;
  }
  // Default ATT MTU allows only 20 notification bytes. A requested MTU of 247
  // does not prove the Pi negotiated it. '~' starts a line, '+' continues it,
  // and newline ends it, so long reset IDs cannot truncate.
  const uint32_t session = connectionSession;
  const uint16_t handle = activeConnHandle;
  size_t offset = 0;
  do {
    if (!deviceConnected || session != connectionSession) return;
    uint8_t frame[20];
    frame[0] = offset == 0 ? '~' : '+';
    size_t used = 1;
    while (used < sizeof(frame) && offset < length) frame[used++] = msg[offset++];
    if (offset == length && used < sizeof(frame)) {
      frame[used++] = '\n';
      offset++;
    }
    pNotifyChar->setValue(frame, used);
    // NimBLE 2.5.x notify() without data schedules a future read of the
    // characteristic. Overwriting it for the next fragment can make queued
    // notifications all contain the last fragment. The explicit payload is
    // copied into a NimBLE mbuf now, and returns real enqueue failures.
    if (!pNotifyChar->notify(frame, used, handle)) {
      notificationFailureCount++;
      return;
    }
  } while (offset <= length);
  // Status requests and telemetry are intentionally repetitive. Printing each
  // notification can occupy the console long enough to disturb scheduling.
  if (VERBOSE_PROTOCOL_LOGS || strcmp(msg, "CONNECTED...") == 0) {
    Serial.print(">>> BLE SENT: ");
    Serial.println(msg);
  }
}

// reset=BROWNOUT with a small up_s points at supply; a long up_s with
// last_drop=0x208 (supervision timeout) points at radio or the Pi side.
static void sendBridgeDiag() {
  char line[BLE_FRAME_MAX_BYTES + 1U];
  snprintf(line, sizeof(line),
           "[BRIDGE DIAG fw=%s reset=%s up_s=%lu drops=%lu last_drop=0x%X last_link_s=%lu "
           "itvl_ms=%u sup_ms=%lu mpu_faults=%lu mpu=%s/%s uart_resync=%lu notify_fail=%lu]",
           BRIDGE_FIRMWARE_ID, bootResetLabel, (unsigned long)(millis() / 1000UL),
           (unsigned long)linkDropCount, (unsigned int)lastDisconnectReason,
           (unsigned long)(lastLinkDurationMs / 1000UL),
           (unsigned int)(connectionIntervalUnits * 5U / 4U),
           (unsigned long)connectionTimeoutUnits * 10UL,
           (unsigned long)mpuFaultCount, mpuHeading.problem(), mpuHeading.lastFault(),
           (unsigned long)mcuAckResyncCount,
           (unsigned long)notificationFailureCount);
  sendNotification(line);
  // The MPU line follows later: this burst already uses most NimBLE buffers.
  bridgeMpuLinePending = true;
  bridgeMpuLineDueMs = millis() + BRIDGE_DIAG_SETTLE_MS;
}

// One DIAG line takes about 8 notifications and NimBLE has only 12 message
// buffers. Inside the connection burst that exhausts them, and NimBLE then
// rejects the Pi's writes with INSUFFICIENT_RESOURCE. So a report waits for the
// burst to drain, and requests made while it waits merge into one line.
static void requestBridgeDiag() {
  if (bridgeDiagPending) return;
  bridgeDiagPending = true;
  bridgeDiagDueMs = millis() + BRIDGE_DIAG_SETTLE_MS;
}

// Heading reference, bias and tilt in one short line, after the DIAG line.
static void sendBridgeMpuLine() {
  char line[112];
  snprintf(line, sizeof(line),
           "[BRIDGE MPU %s bias=%+.3fdps tilt=%.1fdeg refresh=%lu epoch=%lu walls=%lu]",
           mpuHeading.valid() ? "VALID" : "INVALID", mpuHeading.biasDps(), mpuHeading.tiltDeg(),
           (unsigned long)mpuHeading.biasRefreshes(), (unsigned long)headingEpoch,
           (unsigned long)wallLinesSent);
  sendNotification(line);
}

// ── sendConnectedNotice ─────────────────────────────────────
// Notifies the RasPi and MCU that a BLE link is up.
void sendConnectedNotice() {
  if (deviceConnected) {
    if (!sentConnectedMsg) {
      sendNotification("CONNECTED...");
      sentConnectedMsg = true;
    }
    // Inform MCU over UART once MCU is ready and BLE link is established
    if (piReady && mcuReady && !bleBridgeSignalSent) {
      sendNotification("[BRIDGE BUILD:" BRIDGE_FIRMWARE_ID "]");
      sendNotification((String("[BRIDGE BOOT:") + bootResetLabel + "]").c_str());
      sendMcuLine("[BLE CONNECTION ESTABLISHED]", false);
      relayStop("STOP:WAITING_DATA");
      bleBridgeSignalSent = true;
      Serial.println(">>> BLE established; MCU held until fresh path data");
    }
  }
}

// ============================================================
// SIDES PARSER + NUDGE DECISION  (offloaded from controller)
// ============================================================
// Extracts "KEY=VAL" fields from a pipe-delimited substring. Missing and
// malformed are distinct: an optional missing legacy field may use the existing
// fallback, while a present malformed field must fail stable.
enum class FieldParseStatus : uint8_t { MISSING, VALID, INVALID };

// Explicit declarations prevent the Arduino .ino preprocessor from emitting
// these custom-return-type prototypes before FieldParseStatus is declared.
static FieldParseStatus extractField(const String& s,
                                     const char* key,
                                     float& out);
static FieldParseStatus extractFieldEither(const String& s,
                                           const char* compactKey,
                                           const char* legacyKey,
                                           float& out);

static bool parseFiniteDecimal(const String& text, float& out) {
  String value = text;
  value.trim();
  if (value.length() == 0) return false;

  // Limit accepted syntax to a decimal/sign/exponent representation. strtof()
  // alone also accepts tokens such as hexadecimal floats on some toolchains.
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

  errno = 0;
  char* parseEnd = nullptr;
  const char* start = value.c_str();
  const float parsed = strtof(start, &parseEnd);
  if (parseEnd == start || parseEnd == nullptr || *parseEnd != '\0' ||
      errno == ERANGE || !isfinite(parsed)) {
    return false;
  }
  out = parsed;
  return true;
}

static FieldParseStatus extractField(const String& s,
                                     const char* key,
                                     float& out) {
  FieldParseStatus result = FieldParseStatus::MISSING;
  int searchStart = 0;
  while (searchStart < (int)s.length()) {
    int idx = s.indexOf(key, searchStart);
    if (idx < 0) return result;
    if (idx == 0 || s.charAt(idx - 1) == '|') {
      int vStart = idx + strlen(key);  // points just after "KEY="
      int vEnd   = s.indexOf('|', vStart);
      if (vEnd < 0) vEnd = s.length();
      String v = s.substring(vStart, vEnd);
      if (result != FieldParseStatus::MISSING || !parseFiniteDecimal(v, out)) {
        return FieldParseStatus::INVALID;
      }
      result = FieldParseStatus::VALID;
    }
    searchStart = idx + 1;
  }
  return result;
}

static FieldParseStatus extractFieldEither(const String& s,
                                           const char* compactKey,
                                           const char* legacyKey,
                                           float& out) {
  const FieldParseStatus compact = extractField(s, compactKey, out);
  if (compact != FieldParseStatus::MISSING) return compact;
  return extractField(s, legacyKey, out);
}

static bool parseUint32Decimal(const String& text, uint32_t& out);

static bool extractAcquisition(const String& body, uint32_t& acquisition) {
  int start = body.startsWith("A=") ? 0 : body.indexOf("|A=");
  if (start < 0) return false;
  if (start > 0) ++start;
  int end = body.indexOf('|', start);
  if (end < 0) end = body.length();
  if (body.indexOf("|A=", start + 1) >= 0) return false;
  return parseUint32Decimal(body.substring(start + 2, end), acquisition);
}

// One S frame per independent LiDAR acquisition becomes (a) the per-side
// steering permission carried by every M line and (b) one W line with the wall
// line fit for the executor's gyro/LiDAR fusion. A repeated acquisition earns
// nothing and refreshes no age; a malformed frame removes the permission.
static void withdrawSteering() {
  steeringAllow = 0;
  invalidSteeringFieldCount++;
}

static void evaluateSteeringFrame(const String& sidesVal) {
  if (sidesVal.indexOf('=') == -1) {
    steeringAllow = 0;
    return;
  }
  uint32_t acquisition = 0;
  const bool acquisitionValid = extractAcquisition(sidesVal, acquisition);
  if (!acquisitionValid) {
    withdrawSteering();
    return;
  }
  const bool newAcquisition = !haveSteeringAcquisition ||
      (int32_t)(acquisition - lastSteeringAcquisition) > 0;
  if (!newAcquisition) {
    // The same acquisition polled again cannot refresh evidence; an older one
    // arriving late means the stream is out of order.
    if (acquisition != lastSteeringAcquisition) withdrawSteering();
    return;
  }
  // Even a rejected scan consumes its acquisition ID. A later stale scan must
  // not become "new" because the intervening geometry was unusable.
  lastSteeringAcquisition = acquisition;
  haveSteeringAcquisition = true;

  SteeringFrame f;
  struct Field { const char* compact; const char* legacy; float* value; bool* known; };
  bool frontBackKnown = false, tiltKnown = false;
  float back = 0.0f, tilt = 0.0f;
  const Field fields[] = {
    {"L=", "LEFT=", &f.left, &f.leftKnown}, {"R=", "RIGHT=", &f.right, &f.rightKnown},
    {"F=", "FRONT=", &f.front, &f.frontKnown}, {"B=", "BACK=", &back, &frontBackKnown},
    {"FL=", "FRONT_LEFT=", &f.frontLeft, &f.frontLeftKnown},
    {"FR=", "FRONT_RIGHT=", &f.frontRight, &f.frontRightKnown},
    {"BL=", "BACK_LEFT=", &f.backLeft, &f.backLeftKnown},
    {"BR=", "BACK_RIGHT=", &f.backRight, &f.backRightKnown},
    {"WL=", "WALL_LEFT=", &f.wallLeft, &f.wallLeftKnown},
    {"WR=", "WALL_RIGHT=", &f.wallRight, &f.wallRightKnown},
  };
  for (const Field& field : fields) {
    const FieldParseStatus status = extractFieldEither(sidesVal, field.compact, field.legacy, *field.value);
    if (status == FieldParseStatus::INVALID ||
        (status == FieldParseStatus::VALID && !(*field.value > 0.0f))) {
      withdrawSteering();
      return;
    }
    *field.known = status == FieldParseStatus::VALID;
  }
  // Signed fields: heading may be negative, tilt is ignored but must parse.
  const FieldParseStatus heading = extractField(sidesVal, "H=", f.headingDeg);
  const FieldParseStatus tiltStatus = extractFieldEither(sidesVal, "T=", "TILT=", tilt);
  float age = 0.0f;
  const FieldParseStatus ageStatus = extractField(sidesVal, "G=", age);
  if (heading == FieldParseStatus::INVALID || tiltStatus == FieldParseStatus::INVALID ||
      ageStatus == FieldParseStatus::INVALID ||
      (ageStatus == FieldParseStatus::VALID && !(age >= 0.0f && age <= 60000.0f))) {
    withdrawSteering();
    return;
  }
  (void)tiltKnown;
  f.headingKnown = heading == FieldParseStatus::VALID;
  f.ageKnown = ageStatus == FieldParseStatus::VALID;
  f.ageMs = f.ageKnown ? (uint32_t)lroundf(age) : 0U;
  f.clearanceValid =
    extractField(sidesVal, "CF=", f.clearFront) == FieldParseStatus::VALID &&
    extractField(sidesVal, "CL=", f.clearLeft) == FieldParseStatus::VALID &&
    extractField(sidesVal, "CR=", f.clearRight) == FieldParseStatus::VALID &&
    f.clearFront > 0 && f.clearLeft > 0 && f.clearRight > 0;

  steeringAllow = steeringAllowMask(f);
  steeringAllowAtMs = (uint32_t)millis();

  // The wall fit is measurement only. It is forwarded with its total age so the
  // executor can line it up with the gyro heading at the moment of the scan.
  if (ENABLE_LIDAR_CENTERING && f.ageKnown) {
    const uint32_t queuedMs = (uint32_t)millis() - currentFrameReceivedAtMs;
    const uint32_t totalAgeMs = f.ageMs + (queuedMs < 1000U ? queuedMs : 1000U) +
                                BLE_TRANSIT_ESTIMATE_MS;
    char line[96];
    if (formatWallLine(line, sizeof(line), acquisition, totalAgeMs, f) &&
        sendMcuLine(line, false)) {
      wallLinesSent++;
    }
  }
}

// Permission expires with its acquisition, and STOP always withdraws it.
static uint8_t currentSteeringAllow(uint32_t nowMs) {
  if (stopLatched || operatorStopLatched || turnActive) return 0;
  if (nowMs - steeringAllowAtMs > STEERING_MAX_AGE_MS) return 0;
  return steeringAllow;
}

// 50 Hz calibrated yaw for the executor: M,<seq>,<heading cdeg>,<rate cdeg/s>,
// <epoch>,<allow>*<xor>. Without a valid heading it sends MPU:FAULT instead.
static void serviceHeadingStream(uint32_t nowMs) {
  if (mpuHeading.valid()) {
    if (nowMs - lastHeadingSendMs < HEADING_STREAM_INTERVAL_MS) return;
    lastHeadingSendMs = nowMs;
    if (++headingSequence == 0) ++headingSequence;
    char payload[64];
    snprintf(payload, sizeof(payload), "M,%lu,%ld,%ld,%lu,%u",
             (unsigned long)headingSequence, lroundf(mpuHeading.headingDeg() * 100.0f),
             lroundf(mpuHeading.rateDps() * 100.0f), (unsigned long)headingEpoch,
             (unsigned)currentSteeringAllow(nowMs));
    char line[72];
    snprintf(line, sizeof(line), "%s*%02X", payload, (unsigned)lineChecksum(payload));
    sendMcuLine(line, false);
    return;
  }
  if (nowMs - lastMpuFaultSendMs >= MPU_FAULT_REPEAT_MS) {
    lastMpuFaultSendMs = nowMs;
    sendMcuLine("MPU:FAULT", false);
  }
}

static void startHeadingReference() {
  mpuHeading.requestReference();
  if (++headingEpoch == 0) ++headingEpoch;
}

// ============================================================
// SAFETY-FIRST COMPACT TRANSPORT
// ============================================================
// Pi -> bridge:
//   P:<seq>|F=<C|W|O|H|S>|B=<C|O|H|S>[|T=<C|O|S>]  (path; acknowledged write)
//       W = clear to the stop distance, caution zone ahead -> GO:SLOW
//       T = spin-sweep clearance; required to be C between [TURN:START]/[TURN:END]
//   S:<seq>|L=..|R=..|F=..|...|CF=..|CL=..|CR=..|A=<acq>[|WL=..][|WR=..][|H=..]|G=<ms>
//       (steering; latest value). CF/CL/CR give the steering permission bits;
//       WL/WR/H (wall line fit) and G (scan age) become the executor's W line.
//   SENSOR:...                         (telemetry only; never changes path)
//   [HEALTH:1:UUUUU]                   (Internet/cloud/LiDAR/sensor UART/thermal;
//                                       U=up, D=down, X=unknown; telemetry only)
// C=clear, O=generic obstacle, H=legacy human-tagged obstacle, S=stale stream.
// Production Pi LiDAR blockages use O. Its UART ultrasonic value describes
// trash fill level, not a person detector, so it must never promote O to H.
// H remains accepted only for backward compatibility; both O and H fail closed.

static bool parseUint32Decimal(const String& text, uint32_t& out) {
  if (text.length() == 0) return false;
  uint32_t value = 0;
  for (unsigned int i = 0; i < text.length(); ++i) {
    const char c = text.charAt(i);
    if (c < '0' || c > '9') return false;
    const uint32_t digit = (uint32_t)(c - '0');
    if (value > (UINT32_MAX - digit) / 10U) return false;
    value = value * 10U + digit;
  }
  out = value;
  return true;
}

static bool parseSequencedPacket(const String& raw,
                                 char packetType,
                                 uint32_t& seq,
                                 String& body) {
  if (raw.length() < 4 || raw.charAt(0) != packetType || raw.charAt(1) != ':') {
    return false;
  }
  int pipe = raw.indexOf('|', 2);
  if (pipe < 0) return false;
  const String seqText = raw.substring(2, pipe);
  if (!parseUint32Decimal(seqText, seq)) return false;
  body = raw.substring(pipe + 1);
  body.trim();
  return body.length() > 0;
}

static bool sequenceIsNewer(uint32_t seq, uint32_t previous) {
  return (int32_t)(seq - previous) > 0;
}

static String extractSegment(const String& raw, const char* prefix) {
  int idx = raw.indexOf(prefix);
  if (idx < 0) return "";
  int valStart = idx + strlen(prefix);
  int valEnd = raw.indexOf('|', valStart);
  if (valEnd < 0) valEnd = raw.length();
  String val = raw.substring(valStart, valEnd);
  val.trim();
  return val;
}

static void relayStop(const char* reason) {
  stopLatched = true;
  clearConfirmCount = 0;
  sendMcuStop(reason);
}

static bool validPathCode(const String& code) {
  return code == "C" || code == "O" || code == "H" || code == "S";
}

// F may also be W: clear to the stop distance, but something in the swept
// corridor ahead, so drive at caution speed. T is the spin-sweep clearance and
// is optional; it only matters between [TURN:START] and [TURN:END].
static bool parsePathBody(const String& body, String& frontCode, String& backCode,
                          String& turnCode) {
  if (body.endsWith("|")) return false;
  bool haveFront = false;
  bool haveBack = false;
  bool haveTurn = false;
  turnCode = "";
  int start = 0;
  while (start < body.length()) {
    int end = body.indexOf('|', start);
    if (end < 0) end = body.length();
    String token = body.substring(start, end);
    token.trim();
    if (token.startsWith("F=")) {
      if (haveFront || token.length() != 3) return false;
      frontCode = token.substring(2);
      haveFront = true;
    } else if (token.startsWith("B=")) {
      if (haveBack || token.length() != 3) return false;
      backCode = token.substring(2);
      haveBack = true;
    } else if (token.startsWith("T=")) {
      if (haveTurn || token.length() != 3) return false;
      turnCode = token.substring(2);
      haveTurn = true;
    } else {
      return false;
    }
    start = end + 1;
  }
  return haveFront && haveBack &&
         (validPathCode(frontCode) || frontCode == "W") && validPathCode(backCode) &&
         (!haveTurn || turnCode == "C" || turnCode == "O" || turnCode == "S");
}

static void handlePathPacket(const String& raw) {
  uint32_t seq = 0;
  String body;
  if (!parseSequencedPacket(raw, 'P', seq, body)) {
    relayStop("STOP:PROTOCOL");
    return;
  }
  if (havePathSeq && !sequenceIsNewer(seq, lastPathSeq)) {
    if (VERBOSE_PROTOCOL_LOGS) Serial.println(">>> Rejected stale PATH packet");
    relayStop("STOP:SEQUENCE");
    return;
  }

  String frontCode;
  String backCode;
  String turnCode;
  if (!parsePathBody(body, frontCode, backCode, turnCode)) {
    relayStop("STOP:PROTOCOL");
    return;
  }

  lastPathSeq = seq;
  havePathSeq = true;
  pathStreamSeen = true;
  lastPathDataReceived = millis();

  if (operatorStopLatched) {
    relayStop("STOP:OPERATOR");
    return;
  }

  const bool slow = frontCode == "W";
  const bool blocked = (frontCode != "C" && !slow) || (backCode != "C");
  // H changes only the diagnostic STOP reason. Generic O has identical motion
  // safety behavior and is the expected production LiDAR blockage code.
  const bool human = (frontCode == "H") || (backCode == "H");
  const bool stale = (frontCode == "S") || (backCode == "S");

  if (blocked) {
    if (stale) relayStop("STOP:STALE");
    else if (human) relayStop("STOP:HUMAN");
    else relayStop("STOP");
    return;
  }

  // Between [TURN:START] and [TURN:END] the spin sweep must be clear as well.
  // A Pi that sends no T, or an unknown sweep, is never a clear turn.
  if (turnActive && turnCode != "C") {
    relayStop(turnCode == "O" ? "STOP:TURN" : "STOP:TURN_UNKNOWN");
    return;
  }

  // Every motion needs heading feedback: straights hold it and turns are
  // measured by it. The heading is continuous from boot, so only a bounded
  // reference request after recovery may briefly lack a sample.
  if (!mpuHeading.ready() || (!mpuHeading.valid() && !mpuHeading.referenceSettling())) {
    relayStop("STOP:MPU");
    return;
  }

  // Do not release motion while the bridge has no evidence that the executor
  // is consuming UART commands. A later ACK plus fresh clear packets recovers.
  if (mcuAckFault) {
    relayStop("STOP:MCU_LINK");
    return;
  }

  if (stopLatched) {
    clearConfirmCount++;
    if (clearConfirmCount < CLEAR_CONFIRM_PACKETS) {
      sendMcuLine("STOP:CLEARING", true);
      return;
    }
    stopLatched = false;
    clearConfirmCount = 0;
  }
  sendMcuLine(slow ? "GO:SLOW" : "GO", true);
}

static void handleSidesPacket(const String& raw) {
  uint32_t seq = 0;
  String body;
  if (!parseSequencedPacket(raw, 'S', seq, body)) return;
  if (haveSidesSeq && !sequenceIsNewer(seq, lastSidesSeq)) return;
  // A sides packet older than the most recently accepted safety packet must
  // never steer after a newer STOP has arrived. Equal sequence is expected.
  if (!havePathSeq || seq != lastPathSeq) return;

  lastSidesSeq = seq;
  haveSidesSeq = true;
  // Measurements still flow while stopped (a turn's wall alignment happens with
  // the wheels at rest). Permission is withdrawn by STOP in currentSteeringAllow().
  evaluateSteeringFrame(body);
}

static void handleLegacyCombined(const String& raw) {
  // Backward-compatible parser. It is deliberately isolated so SENSOR and
  // control packets can never fall through to a false GO.
  String pathVal = extractSegment(raw, "PATH:");
  String backPathVal = extractSegment(raw, "BACK_PATH:");
  if (pathVal.length() == 0 || backPathVal.length() == 0) {
    relayStop("STOP:PROTOCOL");
    return;
  }

  const bool frontClear = pathVal == "CLEAR";
  const bool frontBlocked = pathVal.startsWith("BLOCKED");
  const bool backClear = backPathVal == "CLEAR";
  const bool backBlocked = backPathVal.startsWith("BLOCKED");
  if ((!frontClear && !frontBlocked) || (!backClear && !backBlocked)) {
    relayStop("STOP:PROTOCOL");
    return;
  }

  uint32_t seq = havePathSeq ? lastPathSeq + 1U : 1U;
  String f = frontBlocked ?
             (pathVal.indexOf("HUMAN_DETECTED") >= 0 ? "H" : "O") : "C";
  String b = backBlocked ? "O" : "C";
  String compactPath = "P:" + String(seq) + "|F=" + f + "|B=" + b;
  handlePathPacket(compactPath);

  int sidesIdx = raw.indexOf("SIDES:");
  if (sidesIdx >= 0) {
    String sidesBody = raw.substring(sidesIdx + 6);
    sidesBody.trim();
    evaluateSteeringFrame(sidesBody);
  }
}

// A bounded parser also runs at ingress, before the ordinary queue can fill.
// No heap allocation, UART, notifications or navigation work in the callback.
static bool parseOperatorCommand(const char* raw, char* id, char* action) {
  if (strncmp(raw, "[OP:", 4) != 0) return false;
  size_t at = 4, digits = 0;
  while (raw[at] >= '0' && raw[at] <= '9') {
    if (digits == 20U) return false;
    id[digits++] = raw[at++];
  }
  if (digits == 0 || id[0] == '0' || raw[at++] != ':') return false;
  id[digits] = '\0';
  size_t length = 0;
  while (raw[at] && raw[at] != ']') {
    if (length == 11U) return false;
    action[length++] = raw[at++];
  }
  if (length == 0 || raw[at] != ']' || raw[at + 1] != '\0') return false;
  action[length] = '\0';
  return strcmp(action, "ESTOP") == 0 || strcmp(action, "RELEASE") == 0 ||
         strcmp(action, "SONAR_OFF") == 0 || strcmp(action, "SONAR_ON") == 0 ||
         strcmp(action, "REBOOT") == 0;
}

static void latchOperatorStop() {
  portENTER_CRITICAL(&operatorMux);
  operatorStopMarker = OPERATOR_STOP_MARKER;
  operatorStopLatched = true;
  operatorExecutorStopped = false;
  operatorStopGeneration++;
  portEXIT_CRITICAL(&operatorMux);
}

static void restoreOperatorStop() {
  operatorStopLatched = operatorStopMarker == OPERATOR_STOP_MARKER;
}

static void operatorFailure(const char* id, const char* reason) {
  sendNotification((String("[OP FAILED:") + id + ":" + reason + "]").c_str());
}

static bool processOperatorCommand(const String& raw) {
  if (raw == "[OP STATUS]") {
    if (mcuReady) sendMcuLine(raw, false);
    return true;
  }
  if (!raw.startsWith("[OP:")) return false;
  char id[21] = {}, action[12] = {};
  if (!parseOperatorCommand(raw.c_str(), id, action)) return true;
  if (strcmp(action, "ESTOP") == 0) {
    latchOperatorStop();
    pendingEmergencyId = id;
    pendingEmergencyGeneration = operatorStopGeneration;
    pendingReleaseId = "";
    pendingRebootId = "";
    relayStop("STOP:OPERATOR");
    // Safety delivery is allowed before startup and through UART backpressure.
    sendMcuLine(raw, true);
    lastEmergencySendMs = millis();
    return true;
  }
  if (!deviceConnected || !piReady || !mcuReady || !bleBridgeSignalSent ||
      mcuAckFault || mcuBackpressureStopPending) {
    operatorFailure(id, "NOT_READY");
    return true;
  }
  if (operatorRebootScheduled || pendingRebootId.length() != 0) {
    operatorFailure(id, "REBOOT_PENDING");
    return true;
  }
  if (strcmp(action, "RELEASE") == 0 || strcmp(action, "REBOOT") == 0) {
    if (!operatorStopLatched || !operatorExecutorStopped) {
      operatorFailure(id, "STOP_NOT_CONFIRMED");
      return true;
    }
    if (strcmp(action, "RELEASE") == 0) {
      pendingReleaseId = id;
      pendingReleaseGeneration = operatorStopGeneration;
    } else {
      pendingRebootId = id;
      pendingRebootGeneration = operatorStopGeneration;
    }
  }
  if (!sendMcuLine(raw, true)) {
    if (pendingReleaseId == id) pendingReleaseId = "";
    if (pendingRebootId == id) pendingRebootId = "";
    operatorFailure(id, "UART_BUSY");
  } else if (strcmp(action, "REBOOT") == 0) {
    pendingRebootSentAtMs = millis();
  }
  return true;
}

static bool isHealthPacket(const char* bytes, size_t length) {
  if (bytes == nullptr || length != 16U ||
      memcmp(bytes, "[HEALTH:1:", 10U) != 0 || bytes[15] != ']') return false;
  for (size_t i = 10U; i < 15U; ++i) {
    if (bytes[i] != 'U' && bytes[i] != 'D' && bytes[i] != 'X') return false;
  }
  return true;
}

static void processAndRelayMessage(const char* incoming) {
  if (incoming == nullptr) return;
  String raw(incoming);
  raw.trim();
  if (raw.length() == 0) return;
  if (raw.startsWith("[HEALTH")) {
    // Revalidate the original line even if this caller bypassed BLE ingress.
    // Health cannot establish readiness, path freshness or permission to move.
    if (deviceConnected && mcuReady &&
        isHealthPacket(incoming, strnlen(incoming, 17U))) {
      sendMcuLine(incoming, true);
    }
    return;
  }
  if (processOperatorCommand(raw)) return;

  const bool startQuery = raw.startsWith("[START STATUS:");
  if (raw.startsWith("[START:") || startQuery) {
    if (!raw.endsWith("]")) return;
    const String id = raw.substring(startQuery ? 14 : 7, raw.length() - 1);
    if (id.length() == 0 || id.length() > 16) return;
    for (unsigned int i = 0; i < id.length(); ++i) {
      if (id.charAt(i) < '0' || id.charAt(i) > '9') return;
    }
    if (!startQuery && operatorStopLatched) {
      sendNotification(("[START FAILED:" + id + ":EMERGENCY_STOP]").c_str());
      return;
    }
    if (!deviceConnected || !piReady || !mcuReady || !bleBridgeSignalSent ||
        mcuAckFault || mcuBackpressureStopPending) {
      // A query cannot establish that the original START failed. Preserve the
      // pending result when transport is temporarily unavailable; the Pi can
      // query again within its deadline without repeating motion.
      if (!startQuery) sendNotification(("[START FAILED:" + id + ":NOT_READY]").c_str());
      return;
    }
    if (!sendMcuLine(raw, true)) {
      if (!startQuery) sendNotification(("[START FAILED:" + id + ":UART_BUSY]").c_str());
    }
    return;
  }
  if (!mcuReady) return;

  if (raw.startsWith("SENSOR:")) {
    sendMcuLine(raw, true);
    return;
  }
  if (raw == "[RESET]" || raw.startsWith("[RESET:")) {
    // The controller owns reset idempotency and brakes before a new return.
    // Repeated delivery must not STOP an already-running return every 3 s.
    sendMcuLine(raw, true);
    return;
  }
  if (raw == "[RASPI READY]") return;
  if (raw.startsWith("P:")) {
    handlePathPacket(raw);
    return;
  }
  if (raw.startsWith("S:")) {
    handleSidesPacket(raw);
    return;
  }
  if (raw.indexOf("PATH:") >= 0) {
    handleLegacyCombined(raw);
    return;
  }

  if (raw.indexOf("PATH") >= 0 || raw.startsWith("P")) {
    relayStop("STOP:PROTOCOL");
  }
  Serial.println(">>> Ignored unknown BLE packet: " + raw);
}

static bool frameStartsWith(const BleFrame& frame, const char* prefix) {
  const size_t prefixLength = strlen(prefix);
  return frame.length >= prefixLength &&
         memcmp(frame.payload, prefix, prefixLength) == 0;
}

static bool trimByte(char c) {
  return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

static void trimFrame(BleFrame& frame) {
  size_t first = 0;
  size_t end = frame.length;
  while (first < end && trimByte(frame.payload[first])) first++;
  while (end > first && trimByte(frame.payload[end - 1U])) end--;
  const size_t trimmedLength = end - first;
  if (first > 0 && trimmedLength > 0) {
    memmove(frame.payload, frame.payload + first, trimmedLength);
  }
  frame.length = (uint16_t)trimmedLength;
  frame.payload[trimmedLength] = '\0';
}

static void resetIngressQueues() {
  if (controlQueue != nullptr) xQueueReset(controlQueue);
  if (sidesQueue != nullptr)   xQueueReset(sidesQueue);
  if (sensorQueue != nullptr)  xQueueReset(sensorQueue);
  if (healthQueue != nullptr)  xQueueReset(healthQueue);
}

static void enqueueBleWrite(const char* bytes, size_t length) {
  if (bytes == nullptr || length == 0) return;
  if (length > BLE_FRAME_MAX_BYTES) {
    oversizeFrameCount++;
    ingressFaultCount++;
    return;
  }

  BleFrame frame = {};
  frame.receivedAtMs = millis();
  frame.session = connectionSession;
  frame.length = (uint16_t)length;
  memcpy(frame.payload, bytes, length);
  frame.payload[length] = '\0';
  trimFrame(frame);
  if (frame.length == 0) return;
  // Reject malformed health envelopes before generic control classification
  // or protocol-fault handling. They cannot fill safety queues or replace the
  // last valid health snapshot. Validate the untrimmed bytes and exact length.
  const bool healthFrame = frameStartsWith(frame, "[HEALTH");
  if (healthFrame && !isHealthPacket(bytes, length)) {
    unknownFrameCount++;
    return;
  }
  // A BLE frame is exactly one printable protocol line. Embedded newlines
  // could otherwise inject UART GO/reset commands through SENSOR telemetry.
  for (size_t i = 0; i < frame.length; ++i) {
    const unsigned char c = (unsigned char)frame.payload[i];
    if (c < 32U || c > 126U) {
      ingressFaultCount++;
      return;
    }
  }

  // A bounded, non-empty GATT write is sufficient for link liveness. Path
  // freshness is tracked separately and only advances after full validation.
  lastDataReceived = frame.receivedAtMs;
  bleWriteSeen = true;

  if (healthFrame) {
    if (healthQueue != nullptr) xQueueOverwrite(healthQueue, &frame);
    else unknownFrameCount++;
    return;
  }

  char operatorId[21] = {}, operatorAction[12] = {};
  if (parseOperatorCommand(frame.payload, operatorId, operatorAction) &&
      strcmp(operatorAction, "ESTOP") == 0) {
    latchOperatorStop();
    if (emergencyQueue != nullptr) xQueueOverwrite(emergencyQueue, &frame);
    else ingressFaultCount++;
    return;
  }

  const bool isControl = frameStartsWith(frame, "P:") ||
                         frameStartsWith(frame, "[RESET") ||
                         frameStartsWith(frame, "[START:") ||
                         frameStartsWith(frame, "[START STATUS:") ||
                         frameStartsWith(frame, "[OP:") ||
                         strcmp(frame.payload, "[OP STATUS]") == 0 ||
                         strstr(frame.payload, "PATH:") != nullptr;
  if (isControl) {
    if (controlQueue == nullptr) {
      ingressFaultCount++;
      return;
    }
    if (xQueueSendToBack(controlQueue, &frame, 0) != pdPASS) {
      BleFrame discarded = {};
      xQueueReceive(controlQueue, &discarded, 0);
      xQueueSendToBack(controlQueue, &frame, 0);
      controlOverflowCount++;
      ingressFaultCount++;
    }
    return;
  }

  if (frameStartsWith(frame, "S:")) {
    if (sidesQueue != nullptr) {
      if (uxQueueMessagesWaiting(sidesQueue) > 0) replacedSidesCount++;
      xQueueOverwrite(sidesQueue, &frame);
    } else {
      ingressFaultCount++;
    }
    return;
  }

  if (frameStartsWith(frame, "SENSOR:")) {
    if (sensorQueue != nullptr) {
      if (uxQueueMessagesWaiting(sensorQueue) > 0) replacedSensorCount++;
      xQueueOverwrite(sensorQueue, &frame);
    } else {
      ingressFaultCount++;
    }
    return;
  }

  // These messages intentionally have no state effect. The ready token only
  // proves Pi-side startup and link liveness, already recorded above.
  if (strcmp(frame.payload, "[RASPI READY]") == 0) {
    readyNoticePending = true;
    return;
  }
  unknownFrameCount++;
}

static bool frameIsCurrent(const BleFrame& frame, unsigned long maxAgeMs) {
  return deviceConnected && frame.session == connectionSession &&
         (millis() - frame.receivedAtMs <= maxAgeMs);
}

static void processBleIngress() {
  if (pendingRebootId.length() && !operatorRebootScheduled &&
      millis() - pendingRebootSentAtMs > 5000UL) {
    operatorFailure(pendingRebootId.c_str(), "ACK_TIMEOUT");
    pendingRebootId = "";  // Retire correlation only; never release or retry motion.
  }
  BleFrame emergency = {};
  if (emergencyQueue != nullptr && xQueueReceive(emergencyQueue, &emergency, 0) == pdPASS) {
    // An emergency never expires or disappears with a BLE session change.
    // Drop older queued releases/starts before dispatching its stop command.
    resetIngressQueues();
    processAndRelayMessage(emergency.payload);
  }
  if (operatorStopLatched && !operatorExecutorStopped && pendingEmergencyId.length() &&
      millis() - lastEmergencySendMs >= STALE_STOP_REPEAT_MS) {
    relayStop("STOP:OPERATOR");
    sendMcuLine("[OP:" + pendingEmergencyId + ":ESTOP]", true);
    lastEmergencySendMs = millis();
  }
  const uint32_t faultSnapshot = ingressFaultCount;
  if (faultSnapshot != handledIngressFaultCount) {
    handledIngressFaultCount = faultSnapshot;
    stopLatched = true;
    clearConfirmCount = 0;
    if (mcuReady) relayStop("STOP:PROTOCOL");
    resetIngressQueues();
    return;
  }

  BleFrame frame = {};
  uint8_t processedControls = 0;
  const uint32_t ingressWindowMs = PATH_FRAME_MAX_AGE_MS;
  while (processedControls < CONTROL_DRAIN_BUDGET &&
         controlQueue != nullptr &&
         xQueueReceive(controlQueue, &frame, 0) == pdPASS) {
    processedControls++;
    if (!deviceConnected || frame.session != connectionSession) continue;
    if (!frameIsCurrent(frame, ingressWindowMs)) {
      if (mcuReady) relayStop("STOP:STALE");
      continue;
    }
    if (VERBOSE_PROTOCOL_LOGS) {
      Serial.print(">>> BLE GOT: ");
      Serial.println(frame.payload);
    }
    currentFrameReceivedAtMs = frame.receivedAtMs;  // a legacy combined frame may carry walls
    processAndRelayMessage(frame.payload);
  }

  // Never spend the current iteration on steering/telemetry while older
  // safety commands remain queued.
  if (controlQueue != nullptr && uxQueueMessagesWaiting(controlQueue) > 0) return;

  if (sidesQueue != nullptr && xQueueReceive(sidesQueue, &frame, 0) == pdPASS) {
    if (frameIsCurrent(frame, SIDES_FRAME_MAX_AGE_MS)) {
      if (VERBOSE_PROTOCOL_LOGS) {
        Serial.print(">>> BLE GOT: ");
        Serial.println(frame.payload);
      }
      currentFrameReceivedAtMs = frame.receivedAtMs;
      processAndRelayMessage(frame.payload);
    } else {
      // Loop starvation made this frame old: it can neither steer nor be fused.
      steeringAllow = 0;
    }
  }

  if (sensorQueue != nullptr && xQueueReceive(sensorQueue, &frame, 0) == pdPASS &&
      frameIsCurrent(frame, SENSOR_FRAME_MAX_AGE_MS)) {
    processAndRelayMessage(frame.payload);
  }
  // Health is the lowest-priority mailbox and never shares sensor capacity.
  if (healthQueue != nullptr && xQueueReceive(healthQueue, &frame, 0) == pdPASS &&
      frameIsCurrent(frame, HEALTH_FRAME_MAX_AGE_MS)) {
    processAndRelayMessage(frame.payload);
  }
}

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pSvr, NimBLEConnInfo& connInfo) override {
    connectionSession++;
    deviceConnected      = true;
    activeConnHandle = connInfo.getConnHandle();
    lastDataReceived = millis();
    connectionStartedMs = lastDataReceived;
    bleWriteSeen = false;
    bleSessionChanged = true;
    connectionIntervalUnits = connInfo.getConnInterval();
    connectionTimeoutUnits = connInfo.getConnTimeout();
    connectionLatency = connInfo.getConnLatency();
    connectionInfoPending = true;
  }

  void onDisconnect(NimBLEServer* pSvr,
                    NimBLEConnInfo& connInfo,
                    int reason) override {
    lastDisconnectReason = reason;
    deviceConnected     = false;
    connectionSession++;
    bleSessionChanged = true;
  }

  void onConnParamsUpdate(NimBLEConnInfo& connInfo) override {
    // No console/UART I/O from the BLE host task.
    connectionIntervalUnits = connInfo.getConnInterval();
    connectionTimeoutUnits = connInfo.getConnTimeout();
    connectionLatency = connInfo.getConnLatency();
    connectionInfoPending = true;
    connParamsReportPending = true;
  }
};

// ── WriteCallbacks ───────────────────────────────────────────
class WriteCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* pChar,
               NimBLEConnInfo& connInfo) override {
    // Copy only. String parsing, float math, logs, and UART writes stay out of
    // the NimBLE host callback so acknowledged safety writes are never delayed
    // by application work.
    const auto received = pChar->getValue();
    enqueueBleWrite(received.c_str(), received.length());
  }
};

void setup() {
  restoreOperatorStop();
  // 160 MHz is ample for a 4 Hz navigation stream and materially reduces
  // bridge heat/power. NimBLE host callbacks now do bounded copies only.
  setCpuFrequencyMhz(BRIDGE_CPU_FREQUENCY_MHZ);
  // Keep the ESP32 brownout detector enabled. Undefined execution during a
  // supply dip is less safe than a clean reboot with motor drivers stopped.

  Serial.begin(115200);
  delay(500);
  Serial.println("[BRIDGE] Firmware " BRIDGE_FIRMWARE_ID);
#if defined(ARDUINO_ARCH_ESP32)
  const esp_reset_reason_t resetReason = esp_reset_reason();
  switch (resetReason) {
    case ESP_RST_POWERON: bootResetLabel = "POWERON"; break;
    case ESP_RST_SW: bootResetLabel = "SOFTWARE"; break;
    case ESP_RST_PANIC: bootResetLabel = "PANIC"; break;
    case ESP_RST_INT_WDT: bootResetLabel = "INT_WATCHDOG"; break;
    case ESP_RST_TASK_WDT: bootResetLabel = "TASK_WATCHDOG"; break;
    case ESP_RST_WDT: bootResetLabel = "WATCHDOG"; break;
    case ESP_RST_BROWNOUT: bootResetLabel = "BROWNOUT"; break;
    default: bootResetLabel = "OTHER"; break;
  }
  Serial.printf("[BRIDGE] Reset=%d (%s), heap=%lu min_heap=%lu\n",
                (int)resetReason, bootResetLabel,
                (unsigned long)esp_get_free_heap_size(),
                (unsigned long)esp_get_minimum_free_heap_size());
#endif

  controlQueue = xQueueCreateStatic(
    CONTROL_QUEUE_DEPTH, sizeof(BleFrame),
    controlQueueStorage, &controlQueueState);
  sidesQueue = xQueueCreateStatic(
    1, sizeof(BleFrame), sidesQueueStorage, &sidesQueueState);
  sensorQueue = xQueueCreateStatic(
    1, sizeof(BleFrame), sensorQueueStorage, &sensorQueueState);
  healthQueue = xQueueCreateStatic(
    1, sizeof(BleFrame), healthQueueStorage, &healthQueueState);
  emergencyQueue = xQueueCreateStatic(
    1, sizeof(BleFrame), emergencyQueueStorage, &emergencyQueueState);
  mcuTxMutex = xSemaphoreCreateMutexStatic(&mcuTxMutexState);
  if (controlQueue == nullptr || sidesQueue == nullptr ||
      sensorQueue == nullptr || healthQueue == nullptr ||
      emergencyQueue == nullptr || mcuTxMutex == nullptr) {
    Serial.println("[BRIDGE] ERROR: static transport initialization failed");
  }

  esp_err_t nvsErr = nvs_flash_init();
  if (nvsErr == ESP_ERR_NVS_NO_FREE_PAGES || nvsErr == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    Serial.println("[BRIDGE] NVS partition stale/corrupt — erasing and re-init.");
    nvs_flash_erase();
    nvsErr = nvs_flash_init();
  }
  if (nvsErr != ESP_OK) {
    Serial.printf("[BRIDGE] WARNING: nvs_flash_init() failed: 0x%x\n", nvsErr);
  }

  // Fixed UART buffers absorb the bounded per-loop burst without making a BLE
  // callback wait for physical serial transmission.
  ESP_Serial.setRxBufferSize(512);
  ESP_Serial.setTxBufferSize(512);
  ESP_Serial.begin(115200, SERIAL_8N1, ESP_RX, ESP_TX);
  ESP_Serial.setTimeout(50);
  Serial.println("[BRIDGE] ESP32-S3 UART ready: RX=IO18 TX=IO17.");
#if defined(ARDUINO_ARCH_ESP32)
  // A rebooted S3 must never repeat its previous epoch: the executor would read
  // the new zero heading as a real rotation.
  headingEpoch = esp_random() & 0x7FFFFFFFUL;
#endif
  if (mpuHeading.begin(MPU_SDA_PIN, MPU_SCL_PIN, MPU_REVERSE_Z)) {
    startHeadingReference();
    Serial.printf("[MPU] Calibrated SDA=IO8 SCL=IO9 bias=%.3f deg/s tilt=%.1f deg; heading streaming.\n",
                  mpuHeading.biasDps(), mpuHeading.tiltDeg());
  } else {
    Serial.printf("[MPU] Init/calibration failed (%s); holding STOP and retrying. Check wiring, keep level/still.\n",
                  mpuHeading.problem());
  }
  delay(500);

  NimBLEDevice::init("GarbyESP32");
  NimBLEDevice::setMTU(247);
  // NimBLE 2.x takes dBm here, not esp_power_level_t. ESP_PWR_LVL_P9 is
  // numerically 12 on ESP-IDF, which accidentally requested 12 dBm.
  NimBLEDevice::setPower(BRIDGE_TX_POWER_DBM);
  // Leave the controller's configured modem sleep enabled. Disabling it here
  // kept the radio awake continuously and added heat without reducing the
  // application-level 250 ms command cadence.

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());
  pServer->advertiseOnDisconnect(true);

  NimBLEService* pService = pServer->createService(SERVICE_UUID);

  NimBLECharacteristic* pWriteChar = pService->createCharacteristic(
    WRITE_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  pWriteChar->setCallbacks(new WriteCallbacks());

  pNotifyChar = pService->createCharacteristic(
    NOTIFY_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  // NimBLE-Arduino 2.x starts all registered services when advertising
  // starts. NimBLEService::start() is a deprecated no-op in 2.5.x.

  NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
  pAdv->enableScanResponse(true);
  pAdv->addServiceUUID(SERVICE_UUID);
  pAdv->setMinInterval(160);  // 100 ms
  pAdv->setMaxInterval(240);  // 150 ms

  NimBLEAdvertisementData advData;
  advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);
  advData.setCompleteServices(NimBLEUUID(SERVICE_UUID));
  pAdv->setAdvertisementData(advData);

  NimBLEAdvertisementData scanData;
  scanData.setName("GarbyESP32");
  pAdv->setScanResponseData(scanData);
  if (!NimBLEDevice::startAdvertising()) {
    Serial.println("[BRIDGE] Advertising start failed; loop will retry");
  }
  lastDataReceived = millis();  // start the watchdog window from boot

  Serial.print("[BRIDGE] MAC: ");
  Serial.println(NimBLEDevice::getAddress().toString().c_str());
  Serial.print("[BRIDGE] TX Power: ");
  Serial.println(NimBLEDevice::getPower());
  Serial.println("[BRIDGE] BLE ready. Advertising...");
}

unsigned long lastCheck = 0;
static char mcuRxBuffer[MCU_UART_LINE_MAX_BYTES + 1U] = {};
static size_t mcuRxLength = 0;
static bool mcuRxDiscarding = false;

static void handleMcuLine(const char* mcuMsg) {
  if (mcuMsg == nullptr || mcuMsg[0] == '\0') return;

  const String operatorMessage(mcuMsg);
  if (operatorMessage.startsWith("[OP ACK:") && operatorMessage.endsWith("]")) {
    const int split = operatorMessage.indexOf(':', 8);
    if (split > 8) {
      const String id = operatorMessage.substring(8, split);
      const String action = operatorMessage.substring(split + 1, operatorMessage.length() - 1);
      if (action == "ESTOP" && id == pendingEmergencyId) {
        portENTER_CRITICAL(&operatorMux);
        if (operatorStopGeneration == pendingEmergencyGeneration) operatorExecutorStopped = true;
        portEXIT_CRITICAL(&operatorMux);
      } else if (action == "RELEASE" && id == pendingReleaseId) {
        bool released = false;
        portENTER_CRITICAL(&operatorMux);
        if (operatorStopGeneration == pendingReleaseGeneration && !operatorRebootScheduled) {
          operatorStopMarker = 0;
          operatorStopLatched = false;
          operatorExecutorStopped = false;
          released = true;
        }
        portEXIT_CRITICAL(&operatorMux);
        pendingReleaseId = "";
        if (released) {
          pendingEmergencyId = "";
          resetNavigationState();
          relayStop("STOP:WAITING_DATA");
        }
      } else if (action == "REBOOT" && id == pendingRebootId &&
                 operatorStopLatched && operatorExecutorStopped &&
                 operatorStopGeneration == pendingRebootGeneration &&
                 millis() - pendingRebootSentAtMs <= 5000UL) {
        // ACK is enqueued for BLE before the delayed restart. RTC STOP stays
        // set on both MCUs; readiness alone cannot restart an interrupted route.
        operatorRebootScheduled = true;
        operatorRebootAtMs = millis() + 1000UL;
      }
    }
    sendNotification(mcuMsg);
    return;
  }
  if (operatorMessage.startsWith("[OP FAILED:") && operatorMessage.endsWith("]")) {
    const int split = operatorMessage.indexOf(':', 11);
    const String id = operatorMessage.substring(11, split);
    if (id == pendingReleaseId) pendingReleaseId = "";
    if (id == pendingRebootId) pendingRebootId = "";
    sendNotification(mcuMsg);
    return;
  }
  if (operatorMessage.startsWith("[OP STATUS:1:ESTOP=1:SONAR=")) {
    portENTER_CRITICAL(&operatorMux);
    if (!operatorStopLatched) {
      operatorStopMarker = OPERATOR_STOP_MARKER;
      operatorStopLatched = true;
      operatorStopGeneration++;
    }
    // At software boot the original request ID is gone. An explicit executor
    // status recovers stop evidence. During an in-flight ESTOP only its ACK
    // can confirm delivery, so an older queued status cannot confirm it.
    if (!pendingEmergencyId.length()) operatorExecutorStopped = true;
    portEXIT_CRITICAL(&operatorMux);
    relayStop("STOP:OPERATOR");
  }

  // These notices are local UART events, never commands accepted from BLE.
  // None of them re-zeroes the heading: the executor keeps its own route frame.
  if (strncmp(mcuMsg, "[HEADING:START", 14) == 0) {
    straightRunActive = true;
    Serial.println(mcuMsg);
    return;
  }
  if (strcmp(mcuMsg, "[HEADING:END]") == 0) {
    straightRunActive = false;
    return;
  }
  if (strcmp(mcuMsg, "[TURN:START]") == 0) {
    // GO evidence from before the turn ignored the spin sweep. Confirm afresh.
    turnActive = true;
    stopLatched = true;
    clearConfirmCount = 0;
    return;
  }
  if (strcmp(mcuMsg, "[TURN:END]") == 0) {
    turnActive = false;
    return;
  }

  // During a route the controller sends [MCU ALIVE] when it has heard nothing for
  // a while, which includes every BLE outage of more than 3 s. That is not a
  // controller reboot, so it must not discard the straight-run heading or operator
  // state. Only a bridge that has itself rebooted, and so does not yet know the
  // controller, needs the handshake below.
  const bool recoveryPing = strcmp(mcuMsg, "[MCU ALIVE]") == 0;
  if (recoveryPing && mcuReady) return;
  if (recoveryPing || strcmp(mcuMsg, "[MCU READY]") == 0) {
    operatorExecutorStopped = false;
    pendingReleaseId = "";
    pendingRebootId = "";
    // A restarted controller drops its own route frame. The bridge's heading
    // reference is continuous and stays valid. Its parked state is unknown
    // until its next state line.
    straightRunActive = false;
    turnActive = false;
    executorParked = false;
    // Treat every READY as a controller boot epoch, even if BLE stayed up.
    // This fixes the case where a rebooted executor waits forever because the
    // bridge remembered an earlier handshake.
    mcuReady = true;
    mcuAckPending = false;
    mcuAckFault = false;
    mcuBackpressureStopPending = false;
    mcuUnackedCommands = 0;
    connectionSession++;
    resetIngressQueues();
    resetNavigationState();
    Serial.println("[BRIDGE] MCU reported [MCU READY]");
    sendNotification("[MCU READY]");
    if (deviceConnected && piReady) {
      sendMcuLine("[BLE CONNECTION ESTABLISHED]", false);
      relayStop("STOP:WAITING_DATA");
      bleBridgeSignalSent = true;
      sendMcuLine("[OP STATUS]", false);
      Serial.println("[BRIDGE] MCU re-handshaked; awaiting fresh path data");
    } else {
      bleBridgeSignalSent = false;
    }
    return;
  }

  // ACKs remain local and generic for full backward compatibility. They prove
  // UART consumer liveness; recovery still requires fresh clear path packets.
  if (strcmp(mcuMsg, "[ESP RECEIVED]") == 0) {
    const bool recovered = mcuAckFault;
    lastMcuAckMs = millis();
    // UART is ordered, so one generic ACK proves one queued command was
    // consumed. Do not let a single ACK erase an entire outstanding backlog.
    if (mcuUnackedCommands > 0) mcuUnackedCommands--;
    mcuAckPending = (mcuUnackedCommands > 0);
    if (mcuAckPending) mcuAckPendingSince = millis();
    mcuAckFault = false;
    if (recovered) {
      stopLatched = true;
      clearConfirmCount = 0;
      Serial.println("[BRIDGE] MCU UART ACK recovered; STOP remains latched");
    }
    return;
  }
  if (strcmp(mcuMsg, "[IDLE]") == 0 || strcmp(mcuMsg, "[OUTBOUND COMPLETE]") == 0 ||
      strcmp(mcuMsg, "[ROUTE FAULT]") == 0) {
    if (!executorParked) executorParkedSinceMs = (uint32_t)millis();
    executorParked = true;
    executorStateAtMs = (uint32_t)millis();
  } else if (strcmp(mcuMsg, "[RUNNING]") == 0 || strcmp(mcuMsg, "[RETURNING]") == 0) {
    // Sent before the route's first step; the bias window ends here.
    executorParked = false;
    executorStateAtMs = (uint32_t)millis();
  }
  if (strcmp(mcuMsg, "[SYSTEM READY]") == 0 &&
      (!deviceConnected || !piReady || !mcuReady || !bleBridgeSignalSent || mcuAckFault)) {
    return;
  }
  if (VERBOSE_PROTOCOL_LOGS) {
    Serial.print("[BRIDGE] MCU->BLE: ");
    Serial.println(mcuMsg);
  }
  sendNotification(mcuMsg);
}

static void pollMcuUart() {
  size_t byteBudget = MCU_UART_RX_BYTE_BUDGET;
  while (byteBudget-- > 0 && ESP_Serial.available()) {
    const char c = (char)ESP_Serial.read();
    if (c == '\r') continue;
    if (c != '\n') {
      if (mcuRxDiscarding) continue;
      if (mcuRxLength < MCU_UART_LINE_MAX_BYTES) {
        mcuRxBuffer[mcuRxLength++] = c;
      } else {
        mcuRxLength = 0;
        mcuRxDiscarding = true;
        mcuLineOverflowCount++;
      }
      continue;
    }

    if (mcuRxDiscarding) {
      mcuRxDiscarding = false;
      mcuRxLength = 0;
      continue;
    }

    size_t first = 0;
    size_t end = mcuRxLength;
    while (first < end && trimByte(mcuRxBuffer[first])) first++;
    while (end > first && trimByte(mcuRxBuffer[end - 1U])) end--;
    const size_t lineLength = end - first;
    if (first > 0 && lineLength > 0) {
      memmove(mcuRxBuffer, mcuRxBuffer + first, lineLength);
    }
    mcuRxBuffer[lineLength] = '\0';
    mcuRxLength = 0;
    handleMcuLine(mcuRxBuffer);
  }
}

static void enforceMcuAckWatchdog(unsigned long now) {
  if (mcuBackpressureStopPending) {
    mcuBackpressureStopPending = false;
    mcuAckFault = true;
    relayStop("STOP:MCU_LINK");
    Serial.println("[BRIDGE] MCU UART backlog capped; motion held");
    return;
  }
  if (!deviceConnected || !mcuReady || !mcuAckPending || mcuAckFault) return;
  // A line lost to EMI never gets its ACK, so without this the estimate only ever
  // grows and eventually holds STOP:MCU_LINK for the rest of the session. The
  // controller answered after our last send and then went quiet: nothing is really
  // outstanding. A controller that stops answering never satisfies this condition.
  if ((long)(lastMcuAckMs - lastMcuSendMs) >= 0 &&
      now - lastMcuAckMs >= MCU_ACK_RESYNC_QUIET_MS) {
    mcuUnackedCommands = 0;
    mcuAckPending = false;
    mcuAckResyncCount++;
    return;
  }
  if (now - mcuAckPendingSince < MCU_ACK_TIMEOUT_MS) return;

  mcuAckFault = true;
  relayStop("STOP:MCU_LINK");
  Serial.println("[BRIDGE] MCU ACK timeout; motion held until UART recovery");
}

static void reportTransportStats(unsigned long now) {
  static unsigned long lastReportMs = 0;
  static uint32_t lastControlOverflow = 0;
  static uint32_t lastOversize = 0;
  static uint32_t lastUnknown = 0;
  static uint32_t lastSidesReplaced = 0;
  static uint32_t lastSensorReplaced = 0;
  static uint32_t lastMcuOverflow = 0;
  static uint32_t lastMcuBackpressure = 0;
  static uint32_t lastInvalidSteering = 0;
  static uint32_t lastNotificationFailures = 0;
  if (now - lastReportMs < TRANSPORT_STATS_INTERVAL_MS) return;
  lastReportMs = now;

  const uint32_t controlOverflow = controlOverflowCount;
  const uint32_t oversize = oversizeFrameCount;
  const uint32_t unknown = unknownFrameCount;
  const uint32_t sidesReplaced = replacedSidesCount;
  const uint32_t sensorReplaced = replacedSensorCount;
  const uint32_t mcuOverflow = mcuLineOverflowCount;
  const uint32_t mcuBackpressure = mcuBackpressureCount;
  const uint32_t invalidSteering = invalidSteeringFieldCount;
  const uint32_t notificationFailures = notificationFailureCount;
  if (controlOverflow == lastControlOverflow && oversize == lastOversize &&
      unknown == lastUnknown && sidesReplaced == lastSidesReplaced &&
      sensorReplaced == lastSensorReplaced && mcuOverflow == lastMcuOverflow &&
      mcuBackpressure == lastMcuBackpressure &&
      invalidSteering == lastInvalidSteering &&
      notificationFailures == lastNotificationFailures) {
    return;
  }

  Serial.printf("[BRIDGE] transport totals: ctrl_overflow=%lu oversize=%lu "
                "unknown=%lu sides_replaced=%lu sensor_replaced=%lu "
                "mcu_line_overflow=%lu mcu_backpressure=%lu "
                "invalid_sides=%lu notify_failures=%lu\n",
                (unsigned long)controlOverflow, (unsigned long)oversize,
                (unsigned long)unknown, (unsigned long)sidesReplaced,
                (unsigned long)sensorReplaced, (unsigned long)mcuOverflow,
                (unsigned long)mcuBackpressure,
                (unsigned long)invalidSteering,
                (unsigned long)notificationFailures);
  lastControlOverflow = controlOverflow;
  lastOversize = oversize;
  lastUnknown = unknown;
  lastSidesReplaced = sidesReplaced;
  lastSensorReplaced = sensorReplaced;
  lastMcuOverflow = mcuOverflow;
  lastMcuBackpressure = mcuBackpressure;
  lastInvalidSteering = invalidSteering;
  lastNotificationFailures = notificationFailures;
}

void loop() {
  if (operatorRebootScheduled && (long)(millis() - operatorRebootAtMs) >= 0) {
    ESP.restart();
    return;
  }
  // All navigation state and UART writes have one owner: this loop. Callbacks
  // only invalidate the session immediately and post bounded work here.
  if (bleSessionChanged) {
    bleSessionChanged = false;
    sentConnectedMsg = false;
    bleBridgeSignalSent = false;
    piReady = false;
    connectionParamsPending = deviceConnected;
    resetNavigationState();
    mcuAckPending = false;
    mcuAckFault = false;
    mcuBackpressureStopPending = false;
    mcuUnackedCommands = 0;
    if (deviceConnected) {
      sendConnectedNotice();
    } else {
      linkDropCount++;
      lastLinkDurationMs = millis() - connectionStartedMs;
      resetIngressQueues();
      if (mcuReady) relayStop("STOP:LINK");
      NimBLEDevice::startAdvertising();
      Serial.printf("[BRIDGE] Link lost reason=0x%X connected_ms=%lu "
                    "write_idle_ms=%lu notify_failures=%lu; STOP asserted, advertising restarted\n",
                    (unsigned int)lastDisconnectReason,
                    (unsigned long)(millis() - connectionStartedMs),
                    (unsigned long)(millis() - lastDataReceived),
                    (unsigned long)notificationFailureCount);
    }
  }
  if (connectionInfoPending) {
    connectionInfoPending = false;
    Serial.printf("[BRIDGE] BLE parameters interval=%.2fms latency=%u supervision=%lums\n",
                  connectionIntervalUnits * 1.25,
                  (unsigned int)connectionLatency,
                  (unsigned long)connectionTimeoutUnits * 10UL);
  }
  if (readyNoticePending) {
    readyNoticePending = false;
    piReady = deviceConnected;
    sentConnectedMsg = false;
    sendConnectedNotice();
    connParamsReportPending = false;
    bridgeDiagPending = false;  // One left from the last session must not land in this burst.
    requestBridgeDiag();
    if (mcuReady) sendMcuLine("[ROBOT STATUS]", false);
  }
  if (connParamsReportPending && piReady) {
    connParamsReportPending = false;
    requestBridgeDiag();
  }
  if (bridgeDiagPending && piReady && (long)(millis() - bridgeDiagDueMs) >= 0) {
    bridgeDiagPending = false;
    sendBridgeDiag();
  } else if (bridgeMpuLinePending && piReady && (long)(millis() - bridgeMpuLineDueMs) >= 0) {
    bridgeMpuLinePending = false;
    sendBridgeMpuLine();
  }
  pollMcuUart();
  // Bias refresh needs the wheels held: a STOP latched for a while, or the
  // executor reporting a parked state for a while, and never inside a turn.
  {
    const uint32_t nowMs = (uint32_t)millis();
    if (stopLatched && !stopLatchSeen) stopLatchedSinceMs = nowMs;
    stopLatchSeen = stopLatched;
    if (executorParked && nowMs - executorStateAtMs > EXECUTOR_STATE_MAX_AGE_MS) executorParked = false;
    const bool heldByStop = stopLatched && nowMs - stopLatchedSinceMs >= STILL_BIAS_SETTLE_MS;
    const bool parked = executorParked && nowMs - executorParkedSinceMs >= STILL_BIAS_SETTLE_MS;
    mpuHeading.setStill(!turnActive && (heldByStop || parked));
  }
  const MpuHeading::Sample mpuSample = mpuHeading.poll();
  if (mpuSample == MpuHeading::Sample::FAULT) {
    mpuFaultCount++;
    if (mcuReady) relayStop("STOP:MPU");
    Serial.printf("[MPU] MPU FAULT %s; holding STOP, then recovering.\n",
                  mpuHeading.lastFault());
    mpuHeading.beginRecovery();
    requestBridgeDiag();
  }
  // STOP:MPU holds the robot throughout, so this measures a stationary sensor. On
  // battery there is no console: without this, one bad moment needs a board reset.
  if (!mpuHeading.ready()) {
    const char* const previousProblem = mpuHeading.problem();
    if (mpuHeading.serviceRecovery()) {
      // A warm recovery within 500 ms keeps the reference and its epoch. Any
      // other recovery starts a new reference; the new epoch tells the executor
      // to continue from its own last heading instead of reading a jump.
      if (!mpuHeading.valid()) startHeadingReference();
      Serial.printf("[MPU] Recovered, bias=%.3f deg/s.\n", mpuHeading.biasDps());
      requestBridgeDiag();
    } else if (strcmp(previousProblem, mpuHeading.problem()) != 0) {
      requestBridgeDiag();
    }
  }
  serviceHeadingStream((uint32_t)millis());
  while (Serial.available()) {
    const char command = (char)Serial.read();
    if ((command == 'R' || command == 'r') && stopLatched && mpuHeading.ready()) {
      startHeadingReference();
      Serial.println("[MPU] Operator requested a new heading reference while stopped.");
    }
    if (command == 'H' || command == 'h') {
      // Bench check of the yaw sign: turn the robot left by hand, heading must rise.
      Serial.printf("[MPU] %s heading=%+.2f deg rate=%+.2f deg/s bias=%+.3f deg/s tilt=%.1f deg epoch=%lu\n",
                    mpuHeading.valid() ? "VALID" : "INVALID", mpuHeading.headingDeg(),
                    mpuHeading.rateDps(), mpuHeading.biasDps(), mpuHeading.tiltDeg(),
                    (unsigned long)headingEpoch);
    }
  }
  processBleIngress();

  const unsigned long now = millis();
  if (deviceConnected && piReady && connectionParamsPending &&
      now - connectionStartedMs >= 1000UL) {
    connectionParamsPending = false;
    if (pServer) pServer->updateConnParams(activeConnHandle, 24, 40, 0, 400);
  }
  static unsigned long lastAdvertisingRetryMs = 0;
  if (!deviceConnected && now - lastAdvertisingRetryMs >= ADVERTISING_RETRY_MS) {
    lastAdvertisingRetryMs = now;
    if (!NimBLEDevice::getAdvertising()->isAdvertising() &&
        !NimBLEDevice::startAdvertising()) {
      Serial.println("[BRIDGE] Advertising retry failed; check BLE initialization logs");
    }
  }
  static unsigned long lastRobotStatusMs = 0;
  if (deviceConnected && piReady && mcuReady && now - lastRobotStatusMs >= 5000UL) {
    lastRobotStatusMs = now;
    sendMcuLine("[ROBOT STATUS]", false);
    sendMcuLine("[OP STATUS]", false);
  }
  if (now - lastCheck >= 1000) {
    lastCheck = now;
    sendConnectedNotice();
  }

  if (deviceConnected && mcuReady &&
      now - lastPathDataReceived >= PATH_DATA_TIMEOUT_MS &&
      now - lastStaleStopSent >= STALE_STOP_REPEAT_MS) {
    relayStop(pathStreamSeen ? "STOP:STALE" : "STOP:WAITING_DATA");
    lastStaleStopSent = now;
  }

  enforceMcuAckWatchdog(now);

  const unsigned long silentLimit = bleWriteSeen ? LINK_DATA_TIMEOUT_MS : BLE_SETUP_TIMEOUT_MS;
  if (deviceConnected && now - lastDataReceived >= silentLimit) {
    if (mcuReady) relayStop("STOP:LINK");
    Serial.println(">>> BLE application silent; holding STOP and reconnecting");
    if (pServer != nullptr) {
      pServer->disconnect(activeConnHandle);
    }
    lastDataReceived = now;
  }

  reportTransportStats(now);
  delay(1);
}
