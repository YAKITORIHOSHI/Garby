#include <Arduino.h>
#include <HardwareSerial.h>
#include <NimBLEDevice.h>
#include <nvs_flash.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_bt_main.h"

HardwareSerial ESP_Serial(1);

// ===== BLE UUIDs =====
#define SERVICE_UUID      "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define WRITE_CHAR_UUID   "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define NOTIFY_CHAR_UUID  "beb5483e-36e1-4688-b7f5-ea07361b26a9"

// ===== UART to Controller ESP32 =====
#define ESP_RX 18
#define ESP_TX 19

// ===== Safety watchdogs =====
// Path freshness is safety-critical and therefore uses a much shorter timeout
// than general BLE liveness. Any stale path stream produces STOP, never GO or
// an automatic return movement.
#define PATH_DATA_TIMEOUT_MS        650UL
#define LINK_DATA_TIMEOUT_MS       8000UL
#define STALE_STOP_REPEAT_MS        500UL
#define CLEAR_CONFIRM_PACKETS          3
#define VERBOSE_PROTOCOL_LOGS           0

// BLE callbacks run in the NimBLE host task. Keep that task deterministic:
// copy writes into fixed-size queues and do parsing/UART work from loop().
// The negotiated MTU is 247, which leaves at most 244 application bytes.
#define BLE_FRAME_MAX_BYTES           244U
#define CONTROL_QUEUE_DEPTH              8U
#define CONTROL_DRAIN_BUDGET              4U
#define PATH_FRAME_MAX_AGE_MS          250UL
#define SIDES_FRAME_MAX_AGE_MS         200UL
#define SENSOR_FRAME_MAX_AGE_MS       2000UL
#define MCU_UART_LINE_MAX_BYTES         256U
#define MCU_UART_RX_BYTE_BUDGET         384U
#define MCU_ACK_TIMEOUT_MS             3000UL
#define MCU_MAX_UNACKED_COMMANDS           8U
#define TRANSPORT_STATS_INTERVAL_MS   10000UL
#define BRIDGE_CPU_FREQUENCY_MHZ         160

// ============================================================
// NUDGE DECISION CONFIGURATION (offloaded from controller)
// ============================================================
// The bridge computes nudge decisions from SIDES data and sends
// pre-digested N: commands to the controller. To avoid aggressive
// startup turns and continuous pulls, three mechanisms are layered:
//   1. STARTUP_GRACE_PACKETS — ignore the first N packets after
//      connection, while LiDAR readings settle.
//   2. NUDGE_COOLDOWN_MS — minimum time between nudge activations.
//   3. Proportional tap duration — small drift gets a short tap.
//   4. Direction confirmation — a direction must persist across several
//      fresh packets before a short correction is allowed.
#define NUDGE_DEAD_ZONE_CM               15.0f
#define NUDGE_HYSTERESIS_CM              10.0f
#define NUDGE_DURATION_MIN_MS             35UL
#define NUDGE_DURATION_MAX_MS             75UL
#define NUDGE_INTENSITY_MIN_PCT            8U
#define NUDGE_INTENSITY_MAX_PCT           22U
#define NUDGE_ERROR_SCALE_CM              60.0f
#define NUDGE_COOLDOWN_MS                1100UL
#define NUDGE_CONFIRM_PACKETS               5
#define STARTUP_GRACE_PACKETS               8
// After grace, ramp from the valid minimum tap toward the computed tap over
// this many fresh accepted packets. Zero remains reserved for safety/stable.
#define STARTUP_RAMP_PACKETS              8
// Combined error weights: lateral centering + heading correction (tilt boost)
#define LATERAL_WEIGHT                   0.85f
#define HEADING_WEIGHT                   0.15f
// Corridor protrusion filtering (fire extinguishers, wall pillars, passing people)
#define CORRIDOR_PROTRUSION_THRESHOLD_CM 12.0f
#define CORRIDOR_OPENING_THRESHOLD_CM    25.0f
#define ERROR_EMA_ALPHA                  0.15f
// Front-aware: front <= SUPPRESS -> nudge fully suppressed
#define FRONT_NUDGE_SUPPRESS_CM          60.0f
#define FRONT_NUDGE_WARN_CM             120.0f

// ===== BLE globals =====
NimBLEServer*          pServer     = nullptr;
NimBLECharacteristic* pNotifyChar = nullptr;
volatile bool deviceConnected     = false;
volatile bool sentConnectedMsg    = false;
volatile bool bleBridgeSignalSent = false;  // Track if we told the MCU about BLE link
volatile bool mcuReady            = false;  // Track if MCU has completed setup()

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
static uint8_t controlQueueStorage[CONTROL_QUEUE_DEPTH * sizeof(BleFrame)];
static uint8_t sidesQueueStorage[sizeof(BleFrame)];
static uint8_t sensorQueueStorage[sizeof(BleFrame)];
static QueueHandle_t controlQueue = nullptr;
static QueueHandle_t sidesQueue   = nullptr;
static QueueHandle_t sensorQueue  = nullptr;

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

static void noteMcuCommandQueued(uint8_t count) {
  if (!mcuAckPending) {
    mcuAckPending = true;
    mcuAckPendingSince = millis();
  }
  const uint16_t next = (uint16_t)mcuUnackedCommands + count;
  mcuUnackedCommands = (next > 255U) ? 255U : (uint8_t)next;
}

static bool commandRequiresLiveBle(const char* line) {
  if (strcmp(line, "GO") == 0) return true;
  return strncmp(line, "N:", 2) == 0 && strncmp(line, "N:0", 3) != 0;
}

static bool commandIsSafetyPriority(const char* line) {
  return strncmp(line, "STOP", 4) == 0 ||
         strncmp(line, "N:0", 3) == 0 ||
         strcmp(line, "[RESET]") == 0;
}

static void sendMcuLine(const char* line, bool expectsAck) {
  if (line == nullptr || line[0] == '\0') return;
  if (mcuTxMutex != nullptr) xSemaphoreTake(mcuTxMutex, portMAX_DELAY);
  // Re-check after acquiring the UART lock. If disconnect raced a queued GO or
  // nudge, the disconnecting task wins and the stale motion command is dropped.
  if (commandRequiresLiveBle(line) && !deviceConnected) {
    if (mcuTxMutex != nullptr) xSemaphoreGive(mcuTxMutex);
    return;
  }
  if (expectsAck && !commandIsSafetyPriority(line) &&
      mcuUnackedCommands >= MCU_MAX_UNACKED_COMMANDS) {
    // Never grow a stale UART backlog if the executor pauses. Safety commands
    // bypass this cap; ordinary GO/nudge/telemetry is dropped and loop() emits
    // an ordered STOP pair immediately afterward.
    stopLatched = true;
    clearConfirmCount = 0;
    mcuBackpressureStopPending = true;
    mcuBackpressureCount++;
    if (mcuTxMutex != nullptr) xSemaphoreGive(mcuTxMutex);
    return;
  }
  ESP_Serial.println(line);
  if (expectsAck) noteMcuCommandQueued(1U);
  if (mcuTxMutex != nullptr) xSemaphoreGive(mcuTxMutex);
}

static void sendMcuLine(const String& line, bool expectsAck) {
  sendMcuLine(line.c_str(), expectsAck);
}

static void sendMcuStopPair(const char* reason) {
  if (reason == nullptr || reason[0] == '\0') return;
  if (mcuTxMutex != nullptr) xSemaphoreTake(mcuTxMutex, portMAX_DELAY);
  ESP_Serial.println(reason);
  ESP_Serial.println("N:0:0|STABLE");
  noteMcuCommandQueued(2U);
  if (mcuTxMutex != nullptr) xSemaphoreGive(mcuTxMutex);
}

static void relayStop(const char* reason);

// ── Nudge state (cooldown + confirmation + startup grace + ramp + EMA corridor filter) ──
unsigned long lastNudgeFireMs       = 0;
int           lastFireDir           = 0;
int           nudgeConfirmDir       = 0;
int           nudgeConfirmCnt       = 0;
int           startupGraceCnt       = 0;
bool          gracePassed           = false;
int           startupRampCnt        = 0;
bool          rampPassed            = false;
static float  corridorBaselineWidth = 0.0f;
static float  smoothedError         = 0.0f;
static bool   emaInitialized        = false;

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
  lastNudgeFireMs        = 0;
  lastFireDir            = 0;
  nudgeConfirmDir        = 0;
  nudgeConfirmCnt        = 0;
  startupGraceCnt        = 0;
  gracePassed            = false;
  startupRampCnt         = 0;
  rampPassed             = false;
  corridorBaselineWidth  = 0.0f;
  smoothedError          = 0.0f;
  emaInitialized         = false;
}

void sendNotification(const char* msg) {
  if (!deviceConnected || pNotifyChar == nullptr || msg == nullptr) return;
  const size_t length = strnlen(msg, BLE_FRAME_MAX_BYTES + 1U);
  if (length == 0 || length > BLE_FRAME_MAX_BYTES) {
    oversizeFrameCount++;
    return;
  }
  pNotifyChar->setValue((const uint8_t*)msg, length);
  pNotifyChar->notify();
  // Status requests and telemetry are intentionally repetitive. Printing each
  // notification can occupy the console long enough to disturb scheduling.
  if (VERBOSE_PROTOCOL_LOGS || strcmp(msg, "CONNECTED...") == 0) {
    Serial.print(">>> BLE SENT: ");
    Serial.println(msg);
  }
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
    if (mcuReady && !bleBridgeSignalSent) {
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
  int searchStart = 0;
  while (searchStart < (int)s.length()) {
    int idx = s.indexOf(key, searchStart);
    if (idx < 0) return FieldParseStatus::MISSING;
    if (idx == 0 || s.charAt(idx - 1) == '|') {
      int vStart = idx + strlen(key);  // points just after "KEY="
      int vEnd   = s.indexOf('|', vStart);
      if (vEnd < 0) vEnd = s.length();
      String v = s.substring(vStart, vEnd);
      return parseFiniteDecimal(v, out) ? FieldParseStatus::VALID
                                        : FieldParseStatus::INVALID;
    }
    searchStart = idx + 1;
  }
  return FieldParseStatus::MISSING;
}

static FieldParseStatus extractFieldEither(const String& s,
                                           const char* compactKey,
                                           const char* legacyKey,
                                           float& out) {
  const FieldParseStatus compact = extractField(s, compactKey, out);
  if (compact != FieldParseStatus::MISSING) return compact;
  return extractField(s, legacyKey, out);
}

// Compute a pre-digested nudge command from a SIDES payload.
// Output used in the new "N:<ms>:<intensity>|NUDGE_LEFT|NUDGE_RIGHT|STABLE" line.
//   ms        = scaled nudge duration (0 if suppressed)
//   intensity = scaled speed cut percentage bounded by NUDGE_INTENSITY_MAX_PCT
//   dir       = one of "STABLE", "NUDGE_LEFT", "NUDGE_RIGHT"
static void computeNudgeCommand(const String& sidesVal,
                                unsigned long& ms,
                                unsigned int& intensity,
                                String& dir) {
  ms        = 0;
  intensity = 0;
  dir       = "STABLE";

  if (sidesVal.indexOf('=') == -1) {
    String v = sidesVal;
    v.replace('-', '_');
    v.toUpperCase();
    if      (v == "NUDGE_LEFT")  dir = "NUDGE_LEFT";
    else if (v == "NUDGE_RIGHT") dir = "NUDGE_RIGHT";
    else                         dir = "STABLE";
    if (dir != "STABLE") {
      ms        = NUDGE_DURATION_MIN_MS;
      intensity = NUDGE_INTENSITY_MIN_PCT;
    }
    return;
  }

  float left = 0.0f, right = 0.0f, front = 0.0f, back = 0.0f;
  float frontLeft = 0.0f, frontRight = 0.0f;
  float backLeft  = 0.0f, backRight  = 0.0f;
  float tiltVal = 0.0f;
  const FieldParseStatus fieldL =
    extractFieldEither(sidesVal, "L=",  "LEFT=",        left);
  const FieldParseStatus fieldR =
    extractFieldEither(sidesVal, "R=",  "RIGHT=",       right);
  const FieldParseStatus fieldF =
    extractFieldEither(sidesVal, "F=",  "FRONT=",       front);
  const FieldParseStatus fieldB =
    extractFieldEither(sidesVal, "B=",  "BACK=",        back);
  const FieldParseStatus fieldFL =
    extractFieldEither(sidesVal, "FL=", "FRONT_LEFT=",  frontLeft);
  const FieldParseStatus fieldFR =
    extractFieldEither(sidesVal, "FR=", "FRONT_RIGHT=", frontRight);
  const FieldParseStatus fieldBL =
    extractFieldEither(sidesVal, "BL=", "BACK_LEFT=",   backLeft);
  const FieldParseStatus fieldBR =
    extractFieldEither(sidesVal, "BR=", "BACK_RIGHT=",  backRight);
  const FieldParseStatus fieldTilt =
    extractFieldEither(sidesVal, "T=",  "TILT=",        tiltVal);

  const bool requiredInvalid =
    fieldL != FieldParseStatus::VALID ||
    fieldR != FieldParseStatus::VALID;
  const bool optionalMalformed =
    fieldF == FieldParseStatus::INVALID ||
    fieldB == FieldParseStatus::INVALID ||
    fieldFL == FieldParseStatus::INVALID ||
    fieldFR == FieldParseStatus::INVALID ||
    fieldBL == FieldParseStatus::INVALID ||
    fieldBR == FieldParseStatus::INVALID ||
    fieldTilt == FieldParseStatus::INVALID;
  const bool nonPositiveDistance =
    (fieldL == FieldParseStatus::VALID && left <= 0.0f) ||
    (fieldR == FieldParseStatus::VALID && right <= 0.0f) ||
    (fieldF == FieldParseStatus::VALID && front <= 0.0f) ||
    (fieldB == FieldParseStatus::VALID && back <= 0.0f) ||
    (fieldFL == FieldParseStatus::VALID && frontLeft <= 0.0f) ||
    (fieldFR == FieldParseStatus::VALID && frontRight <= 0.0f) ||
    (fieldBL == FieldParseStatus::VALID && backLeft <= 0.0f) ||
    (fieldBR == FieldParseStatus::VALID && backRight <= 0.0f);

  if (requiredInvalid || optionalMalformed || nonPositiveDistance) {
    // A corrupted sample must also break direction confirmation; otherwise
    // valid samples on either side of it could incorrectly count as consecutive.
    nudgeConfirmDir = 0;
    nudgeConfirmCnt = 0;
    invalidSteeringFieldCount++;
    return;  // outputs were initialized to N:0:0|STABLE
  }

  const bool okF  = fieldF  == FieldParseStatus::VALID;
  const bool okFL = fieldFL == FieldParseStatus::VALID;
  const bool okFR = fieldFR == FieldParseStatus::VALID;
  const bool okBL = fieldBL == FieldParseStatus::VALID;
  const bool okBR = fieldBR == FieldParseStatus::VALID;
  const bool okTilt = fieldTilt == FieldParseStatus::VALID;

  // ── Clamp inputs to sane ranges ────────────────────────────
  if (left  < 5.0f)   left  = 5.0f;
  if (right < 5.0f)   right = 5.0f;
  if (left  > 200.0f) left  = 200.0f;
  if (right > 200.0f) right = 200.0f;
  if (okF) {
    if (front < 5.0f)   front = 5.0f;
    if (front > 400.0f) front = 400.0f;
  }
  if (okFL) {
    if (frontLeft < 5.0f)   frontLeft = 5.0f;
    if (frontLeft > 400.0f) frontLeft = 400.0f;
  }
  if (okFR) {
    if (frontRight < 5.0f)   frontRight = 5.0f;
    if (frontRight > 400.0f) frontRight = 400.0f;
  }
  if (okBL) {
    if (backLeft < 5.0f)   backLeft = 5.0f;
    if (backLeft > 400.0f) backLeft = 400.0f;
  }
  if (okBR) {
    if (backRight < 5.0f)   backRight = 5.0f;
    if (backRight > 400.0f) backRight = 400.0f;
  }

  // ── Corridor baseline and transient geometry rejection ─────
  float currentWidth = left + right;
  if (corridorBaselineWidth <= 0.0f) {
    corridorBaselineWidth = currentWidth;
  }

  const float widthDrop = corridorBaselineWidth - currentWidth;
  const float widthRise = currentWidth - corridorBaselineWidth;

  // Only adapt the baseline while the measurement is near the established
  // corridor width. Freezing it through a pillar, passer, or doorway prevents
  // the transient from becoming the new center target.
  if (fabsf(currentWidth - corridorBaselineWidth) < CORRIDOR_PROTRUSION_THRESHOLD_CM) {
    corridorBaselineWidth = 0.98f * corridorBaselineWidth + 0.02f * currentWidth;
  } else if (widthRise > 0.0f && widthRise < CORRIDOR_OPENING_THRESHOLD_CM) {
    corridorBaselineWidth = 0.995f * corridorBaselineWidth + 0.005f * currentWidth;
  }
  if (corridorBaselineWidth > 220.0f) corridorBaselineWidth = 220.0f;

  // lateralError = LEFT - RIGHT (positive -> move toward the left clearance).
  float lateralError = left - right;
  if (widthDrop > CORRIDOR_PROTRUSION_THRESHOLD_CM && widthDrop < 60.0f) {
    lateralError *= 0.25f;
  }
  if (widthRise > CORRIDOR_OPENING_THRESHOLD_CM) {
    // Open doors and side junctions should not pull the robot out of the
    // established hallway centerline.
    lateralError *= 0.20f;
  }
  if (lateralError > 50.0f) lateralError = 50.0f;
  if (lateralError < -50.0f) lateralError = -50.0f;

  float rawError;
  if (okTilt) {
    // Multi-Point Angle of Vision heading tilt calculation
    rawError = LATERAL_WEIGHT * lateralError + HEADING_WEIGHT * tiltVal;
  } else if (okFL && okFR && okBL && okBR) {
    float headingError = 0.5f * (frontLeft - frontRight) + 0.5f * (backRight - backLeft);
    if (headingError > 20.0f) headingError = 20.0f;
    if (headingError < -20.0f) headingError = -20.0f;
    rawError = LATERAL_WEIGHT * lateralError + HEADING_WEIGHT * headingError;
  } else if (okFL && okFR) {
    float headingError = frontLeft - frontRight;
    if (headingError > 20.0f) headingError = 20.0f;
    if (headingError < -20.0f) headingError = -20.0f;
    rawError = LATERAL_WEIGHT * lateralError + HEADING_WEIGHT * headingError;
  } else {
    rawError = lateralError;  // fall back to lateral-only if diagonal data missing
  }

  // ── Exponential Moving Average (EMA) smoothing ──────────────
  if (!emaInitialized) {
    smoothedError  = rawError;
    emaInitialized = true;
  } else {
    smoothedError = ERROR_EMA_ALPHA * rawError + (1.0f - ERROR_EMA_ALPHA) * smoothedError;
  }

  float error = smoothedError;

  // ── Front suppression modifier (0.0..1.0) ───────────────────
  float frontMod = 1.0f;
  if (okF && front > 0.0f) {
    if (front <= FRONT_NUDGE_SUPPRESS_CM) {
      frontMod = 0.0f;
    } else if (front < FRONT_NUDGE_WARN_CM) {
      frontMod = (front - FRONT_NUDGE_SUPPRESS_CM) /
                 (FRONT_NUDGE_WARN_CM - FRONT_NUDGE_SUPPRESS_CM);
    }
  }
  if (frontMod <= 0.01f) { ms = 0; intensity = 0; dir = "STABLE"; return; }

  // ── Direction + dead-zone + Hysteresis (anti-hunting on long runs) ──
  float absErr = fabsf(error);
  int wantDir = 0;
  float requiredThreshold = NUDGE_DEAD_ZONE_CM;

  if (lastFireDir != 0) {
    bool isReverseDir = (error > 0.0f && lastFireDir < 0) || (error < 0.0f && lastFireDir > 0);
    if (isReverseDir) {
      requiredThreshold += NUDGE_HYSTERESIS_CM;
    }
  }

  if (absErr > requiredThreshold) wantDir = (error > 0.0f) ? 1 : -1;

  // ── Collision Prevention Interlock ─────────────────────────
  // Suppress nudging toward a side wall if robot is already close to that wall (<= 20cm),
  // preventing steering into opposite walls or obstacles during evasion nudges.
  if (wantDir > 0 && left <= 20.0f)  wantDir = 0;
  if (wantDir < 0 && right <= 20.0f) wantDir = 0;

  // ── Direction confirmation (anti-zigzag) ───────────────────
  if (wantDir != 0 && wantDir == nudgeConfirmDir) {
    nudgeConfirmCnt++;
  } else {
    nudgeConfirmDir = wantDir;
    nudgeConfirmCnt = (wantDir != 0) ? 1 : 0;
  }
  bool confirmed = (wantDir != 0) && (nudgeConfirmCnt >= NUDGE_CONFIRM_PACKETS);

  // ── Cooldown check ─────────────────────────────────────────
  bool periodOk = (millis() - lastNudgeFireMs >= NUDGE_COOLDOWN_MS);

  if (!confirmed || !periodOk || wantDir == 0) {
    ms = 0; intensity = 0; dir = "STABLE"; return;
  }

  // ── Proportional duration & intensity scaling ───────────────
  float scale = (absErr - NUDGE_DEAD_ZONE_CM) / NUDGE_ERROR_SCALE_CM;
  if (scale > 1.0f) scale = 1.0f;

  unsigned long baseDur = NUDGE_DURATION_MIN_MS +
    (unsigned long)((float)(NUDGE_DURATION_MAX_MS - NUDGE_DURATION_MIN_MS) * scale);
  unsigned long dur = (unsigned long)((float)baseDur * frontMod);

  unsigned int baseIntensity = NUDGE_INTENSITY_MIN_PCT +
    (unsigned int)((float)(NUDGE_INTENSITY_MAX_PCT - NUDGE_INTENSITY_MIN_PCT) * scale);
  unsigned int calcIntensity = (unsigned int)((float)baseIntensity * frontMod);

  if (dur >= NUDGE_DURATION_MIN_MS) {
    ms        = dur;
    intensity = calcIntensity;
    dir       = (error > 0.0f) ? "NUDGE_LEFT" : "NUDGE_RIGHT";
    lastNudgeFireMs    = millis();
    lastFireDir        = (error > 0.0f) ? 1 : -1;
    nudgeConfirmCnt    = 0;  // require fresh confirmation for next tap
  } else {
    ms = 0; intensity = 0; dir = "STABLE";
  }
}

// ============================================================
// SAFETY-FIRST COMPACT TRANSPORT
// ============================================================
// Pi -> bridge:
//   P:<seq>|F=<C|O|H|S>|B=<C|O|H|S>  (path; acknowledged write)
//   S:<seq>|L=..|R=..|F=..|...        (steering; latest value)
//   SENSOR:...                         (telemetry only; never changes path)
// C=clear, O=generic obstacle, H=legacy human-tagged obstacle, S=stale stream.
// Production Pi LiDAR blockages use O. Its UART ultrasonic value describes
// trash fill level, not a person detector, so it must never promote O to H.
// H remains accepted only for backward compatibility; both O and H fail closed.

static bool parseSequencedPacket(const String& raw,
                                 char packetType,
                                 uint32_t& seq,
                                 String& body) {
  if (raw.length() < 4 || raw.charAt(0) != packetType || raw.charAt(1) != ':') {
    return false;
  }
  int pipe = raw.indexOf('|', 2);
  if (pipe < 0) return false;
  String seqText = raw.substring(2, pipe);
  if (seqText.length() == 0) return false;
  for (unsigned int i = 0; i < seqText.length(); ++i) {
    if (!isDigit(seqText.charAt(i))) return false;
  }
  seq = (uint32_t)strtoul(seqText.c_str(), nullptr, 10);
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

static void sendStableNudge() {
  sendMcuLine("N:0:0|STABLE", true);
}

static void relayStop(const char* reason) {
  stopLatched = true;
  clearConfirmCount = 0;
  sendMcuStopPair(reason);
}

static bool validPathCode(const String& code) {
  return code == "C" || code == "O" || code == "H" || code == "S";
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
    return;
  }

  String frontCode = extractSegment(body, "F=");
  String backCode = extractSegment(body, "B=");
  if (!validPathCode(frontCode) || !validPathCode(backCode)) {
    relayStop("STOP:PROTOCOL");
    return;
  }

  lastPathSeq = seq;
  havePathSeq = true;
  pathStreamSeen = true;
  lastPathDataReceived = millis();

  const bool blocked = (frontCode != "C") || (backCode != "C");
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
      sendStableNudge();
      return;
    }
    stopLatched = false;
    clearConfirmCount = 0;
  }
  sendMcuLine("GO", true);
}

static void sendComputedNudge(const String& sidesBody) {
  unsigned long nudgeMs = 0;
  unsigned int nudgeIntensity = 0;
  String nudgeDir = "STABLE";

  if (stopLatched) {
    sendStableNudge();
    return;
  }

  if (!gracePassed) {
    startupGraceCnt++;
    if (startupGraceCnt >= STARTUP_GRACE_PACKETS) {
      gracePassed = true;
      startupRampCnt = 0;
    }
    // All grace packets are suppression-only, including the packet that
    // completes grace. The next fresh packet is ramp packet 1.
    sendStableNudge();
    return;
  }

  // Advance on every fresh accepted post-grace SIDES packet, even when the
  // current geometry is stable or front suppression prevents a correction.
  // This guarantees completion in exactly STARTUP_RAMP_PACKETS samples rather
  // than consuming one cooldown interval per ramp step.
  float rampScale = 1.0f;
  if (!rampPassed) {
    if (startupRampCnt < STARTUP_RAMP_PACKETS) startupRampCnt++;
    rampScale = (float)startupRampCnt / (float)STARTUP_RAMP_PACKETS;
    if (startupRampCnt >= STARTUP_RAMP_PACKETS) {
      startupRampCnt = STARTUP_RAMP_PACKETS;
      rampPassed = true;
    }
  }

  computeNudgeCommand(sidesBody, nudgeMs, nudgeIntensity, nudgeDir);

  if (nudgeMs == 0 || nudgeDir == "STABLE") {
    // Preserve all stop/front/dead-zone/cooldown suppression as an explicit
    // stable command. Zero intensity is valid only for this stable case.
    nudgeMs = 0;
    nudgeIntensity = 0;
    nudgeDir = "STABLE";
  } else {
    // Normalize the computed tap before applying startup scaling. Scaling the
    // span above the valid minima means a confirmed tap is never consumed and
    // then discarded, and intensity=0 can never trigger executor legacy mode.
    if (nudgeMs < NUDGE_DURATION_MIN_MS) nudgeMs = NUDGE_DURATION_MIN_MS;
    if (nudgeMs > NUDGE_DURATION_MAX_MS) nudgeMs = NUDGE_DURATION_MAX_MS;
    if (nudgeIntensity < NUDGE_INTENSITY_MIN_PCT) {
      nudgeIntensity = NUDGE_INTENSITY_MIN_PCT;
    }
    if (nudgeIntensity > NUDGE_INTENSITY_MAX_PCT) {
      nudgeIntensity = NUDGE_INTENSITY_MAX_PCT;
    }

    if (rampScale < 1.0f) {
      const unsigned long durationAboveMin =
        nudgeMs - NUDGE_DURATION_MIN_MS;
      const unsigned int intensityAboveMin =
        nudgeIntensity - NUDGE_INTENSITY_MIN_PCT;
      nudgeMs = NUDGE_DURATION_MIN_MS +
        (unsigned long)((float)durationAboveMin * rampScale);
      nudgeIntensity = NUDGE_INTENSITY_MIN_PCT +
        (unsigned int)((float)intensityAboveMin * rampScale);
    }
  }

  char command[48];
  snprintf(command, sizeof(command), "N:%lu:%u|%s",
           nudgeMs, nudgeIntensity, nudgeDir.c_str());
  sendMcuLine(command, true);
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
  if (!pathStreamSeen || stopLatched) {
    sendStableNudge();
    return;
  }
  sendComputedNudge(body);
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

  uint32_t seq = havePathSeq ? lastPathSeq + 1U : 1U;
  String f = pathVal.startsWith("BLOCKED") ?
             (pathVal.indexOf("HUMAN_DETECTED") >= 0 ? "H" : "O") : "C";
  String b = backPathVal.startsWith("BLOCKED") ? "O" : "C";
  String compactPath = "P:" + String(seq) + "|F=" + f + "|B=" + b;
  handlePathPacket(compactPath);

  int sidesIdx = raw.indexOf("SIDES:");
  if (sidesIdx >= 0) {
    String sidesBody = raw.substring(sidesIdx + 6);
    sidesBody.trim();
    sendComputedNudge(sidesBody);
  }
}

static void processAndRelayMessage(const char* incoming) {
  if (incoming == nullptr) return;
  String raw(incoming);
  raw.trim();
  if (raw.length() == 0 || !mcuReady) return;

  if (raw.startsWith("SENSOR:")) {
    sendMcuLine(raw, true);
    return;
  }
  if (raw == "[RESET]") {
    relayStop("STOP:RESET");
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

  // A bounded, non-empty GATT write is sufficient for link liveness. Path
  // freshness is tracked separately and only advances after full validation.
  lastDataReceived = frame.receivedAtMs;

  const bool isControl = frameStartsWith(frame, "P:") ||
                         strcmp(frame.payload, "[RESET]") == 0 ||
                         strstr(frame.payload, "PATH:") != nullptr;
  if (isControl) {
    if (controlQueue == nullptr ||
        xQueueSendToBack(controlQueue, &frame, 0) != pdPASS) {
      // Preserve the newest physical state, discard the oldest queued state,
      // and force a fail-closed stop before any replacement can be executed.
      BleFrame discarded = {};
      if (controlQueue != nullptr) {
        xQueueReceive(controlQueue, &discarded, 0);
        xQueueSendToBack(controlQueue, &frame, 0);
      }
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
  if (strcmp(frame.payload, "[RASPI READY]") == 0) return;
  unknownFrameCount++;
}

static bool frameIsCurrent(const BleFrame& frame, unsigned long maxAgeMs) {
  return deviceConnected && frame.session == connectionSession &&
         (millis() - frame.receivedAtMs <= maxAgeMs);
}

static void processBleIngress() {
  const uint32_t faultSnapshot = ingressFaultCount;
  if (faultSnapshot != handledIngressFaultCount) {
    handledIngressFaultCount = faultSnapshot;
    stopLatched = true;
    clearConfirmCount = 0;
    if (mcuReady) relayStop("STOP:PROTOCOL");
    return;
  }

  BleFrame frame = {};
  uint8_t processedControls = 0;
  while (processedControls < CONTROL_DRAIN_BUDGET &&
         controlQueue != nullptr &&
         xQueueReceive(controlQueue, &frame, 0) == pdPASS) {
    processedControls++;
    if (!deviceConnected || frame.session != connectionSession) continue;
    if (!frameIsCurrent(frame, PATH_FRAME_MAX_AGE_MS)) {
      if (mcuReady) relayStop("STOP:STALE");
      continue;
    }
    if (VERBOSE_PROTOCOL_LOGS) {
      Serial.print(">>> BLE GOT: ");
      Serial.println(frame.payload);
    }
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
      processAndRelayMessage(frame.payload);
    } else if (deviceConnected && mcuReady && !stopLatched) {
      // A stale steering tap must never be replayed after loop starvation.
      sendStableNudge();
    }
  }

  if (sensorQueue != nullptr && xQueueReceive(sensorQueue, &frame, 0) == pdPASS &&
      frameIsCurrent(frame, SENSOR_FRAME_MAX_AGE_MS)) {
    processAndRelayMessage(frame.payload);
  }
}

class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pSvr, NimBLEConnInfo& connInfo) override {
    connectionSession++;
    resetIngressQueues();
    deviceConnected      = true;
    sentConnectedMsg     = false;
    bleBridgeSignalSent  = false;

    activeConnHandle = connInfo.getConnHandle();
    lastDataReceived = millis();
    resetNavigationState();
    mcuAckPending = false;
    mcuAckFault = false;
    mcuBackpressureStopPending = false;
    mcuUnackedCommands = 0;

    Serial.print(">>> CONNECTED: ");
    Serial.println(connInfo.getAddress().toString().c_str());
    NimBLEDevice::stopAdvertising();

    // Negotiate stable connection parameters.
    // interval 12-24 = 15-30 ms, timeout 400 = 4000 ms.
    pServer->updateConnParams(connInfo.getConnHandle(), 12, 24, 0, 400);

    // If MCU has already finished setup, inform MCU that BLE link is established
    if (mcuReady) {
      sendMcuLine("[BLE CONNECTION ESTABLISHED]", false);
      relayStop("STOP:WAITING_DATA");
      bleBridgeSignalSent = true;
      Serial.println(">>> BLE established; MCU held until fresh path data");
    } else {
      Serial.println(">>> BLE connected, waiting for [MCU READY] signal from MCU...");
    }
  }

  void onDisconnect(NimBLEServer* pSvr,
                    NimBLEConnInfo& connInfo,
                    int reason) override {
    deviceConnected     = false;
    bleBridgeSignalSent = false;
    connectionSession++;
    resetIngressQueues();
    Serial.printf(">>> DISCONNECTED  reason=0x%02X\n", reason);
    
    resetNavigationState();
    if (mcuReady) {
      relayStop("STOP:LINK");
    }
    // ACK timing is meaningful only during an active BLE control session.
    mcuAckPending = false;
    mcuBackpressureStopPending = false;
    mcuUnackedCommands = 0;
    Serial.println(">>> Link lost; MCU latched in STOP");

    NimBLEDevice::startAdvertising();
    Serial.println(">>> Advertising restarted.");
  }

  void onConnParamsUpdate(NimBLEConnInfo& connInfo) override {
    Serial.printf(">>> ConnParams: interval=%u latency=%u timeout=%u\n",
                  connInfo.getConnInterval(),
                  connInfo.getConnLatency(),
                  connInfo.getConnTimeout());
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
  // 160 MHz is ample for a 4 Hz navigation stream and materially reduces
  // bridge heat/power. NimBLE host callbacks now do bounded copies only.
  setCpuFrequencyMhz(BRIDGE_CPU_FREQUENCY_MHZ);
  // Keep the ESP32 brownout detector enabled. Undefined execution during a
  // supply dip is less safe than a clean reboot with motor drivers stopped.

  Serial.begin(115200);
  delay(500);

  controlQueue = xQueueCreateStatic(
    CONTROL_QUEUE_DEPTH, sizeof(BleFrame),
    controlQueueStorage, &controlQueueState);
  sidesQueue = xQueueCreateStatic(
    1, sizeof(BleFrame), sidesQueueStorage, &sidesQueueState);
  sensorQueue = xQueueCreateStatic(
    1, sizeof(BleFrame), sensorQueueStorage, &sensorQueueState);
  mcuTxMutex = xSemaphoreCreateMutexStatic(&mcuTxMutexState);
  if (controlQueue == nullptr || sidesQueue == nullptr ||
      sensorQueue == nullptr || mcuTxMutex == nullptr) {
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
  Serial.println("[BRIDGE] UART to MCU ready.");
  delay(500);

  NimBLEDevice::init("GarbyESP32");
  NimBLEDevice::setMTU(247);
  NimBLEDevice::setPower(ESP_PWR_LVL_P3);
  // Leave the controller's configured modem sleep enabled. Disabling it here
  // kept the radio awake continuously and added heat without reducing the
  // application-level 250 ms command cadence.

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  NimBLEService* pService = pServer->createService(SERVICE_UUID);

  NimBLECharacteristic* pWriteChar = pService->createCharacteristic(
    WRITE_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
  pWriteChar->setCallbacks(new WriteCallbacks());

  pNotifyChar = pService->createCharacteristic(
    NOTIFY_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
  pAdv->addServiceUUID(SERVICE_UUID);
  NimBLEAdvertisementData scanData;
  scanData.setName("GarbyESP32");
  pAdv->setScanResponseData(scanData);
  NimBLEDevice::startAdvertising();
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

  if (strcmp(mcuMsg, "[MCU READY]") == 0) {
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
    if (deviceConnected) {
      sendMcuLine("[BLE CONNECTION ESTABLISHED]", false);
      relayStop("STOP:WAITING_DATA");
      bleBridgeSignalSent = true;
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
    // The legacy ACK has no sequence, so treat any ACK as proof that the UART
    // consumer is draining again instead of pretending to match it to a line.
    mcuUnackedCommands = 0;
    mcuAckPending = false;
    mcuAckFault = false;
    if (recovered) {
      stopLatched = true;
      clearConfirmCount = 0;
      Serial.println("[BRIDGE] MCU UART ACK recovered; STOP remains latched");
    }
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
  if (controlOverflow == lastControlOverflow && oversize == lastOversize &&
      unknown == lastUnknown && sidesReplaced == lastSidesReplaced &&
      sensorReplaced == lastSensorReplaced && mcuOverflow == lastMcuOverflow &&
      mcuBackpressure == lastMcuBackpressure &&
      invalidSteering == lastInvalidSteering) {
    return;
  }

  Serial.printf("[BRIDGE] transport totals: ctrl_overflow=%lu oversize=%lu "
                "unknown=%lu sides_replaced=%lu sensor_replaced=%lu "
                "mcu_line_overflow=%lu mcu_backpressure=%lu "
                "invalid_sides=%lu\n",
                (unsigned long)controlOverflow, (unsigned long)oversize,
                (unsigned long)unknown, (unsigned long)sidesReplaced,
                (unsigned long)sensorReplaced, (unsigned long)mcuOverflow,
                (unsigned long)mcuBackpressure,
                (unsigned long)invalidSteering);
  lastControlOverflow = controlOverflow;
  lastOversize = oversize;
  lastUnknown = unknown;
  lastSidesReplaced = sidesReplaced;
  lastSensorReplaced = sensorReplaced;
  lastMcuOverflow = mcuOverflow;
  lastMcuBackpressure = mcuBackpressure;
  lastInvalidSteering = invalidSteering;
}

void loop() {
  pollMcuUart();
  processBleIngress();

  const unsigned long now = millis();
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

  if (deviceConnected && now - lastDataReceived >= LINK_DATA_TIMEOUT_MS) {
    if (mcuReady) relayStop("STOP:LINK");
    Serial.println(">>> BLE link silent for 8s; holding STOP and reconnecting");
    if (pServer != nullptr) {
      pServer->disconnect(activeConnHandle);
    }
    lastDataReceived = now;
  }

  reportTransportStats(now);
  delay(1);
}
