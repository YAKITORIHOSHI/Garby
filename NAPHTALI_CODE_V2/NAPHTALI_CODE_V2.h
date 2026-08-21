#pragma once
// ============================================================
// LIBRARIES
// ============================================================
#include <ESP32Servo.h>
#include <FastAccelStepper.h>
#include <HardwareSerial.h>
#include <HX711.h>
#include <Arduino.h>

// 160 MHz is ample for UART parsing and hardware-timed step pulses, while
// reducing heat and current draw compared with forcing this executor to 240 MHz.
#define EXECUTOR_CPU_MHZ 160

// ============================================================
// LOAD CELL
// ============================================================
#define LOADCELL_DOUT_PIN  33
#define LOADCELL_SCK_PIN   32
constexpr float LOAD_TRIGGER_KG = 1.0f;
const float  CALIBRATION_FACTOR  = 100000.0f;

// ============================================================
// ESP32 WROOM (LiDAR BLE bridge)
// ============================================================
#define ESP_RX  18
#define ESP_TX  19

// ============================================================
// SIM MODULE — Air780E
// ============================================================
#define AIR_RX      16
#define AIR_TX      17
#define PWRKEY_PIN  26

// ============================================================
// SERVO
// ============================================================
#define SERVO_PIN    23
// Avoid commanding hobby-servo mechanical end stops; endpoint binding can
// draw enough current to throttle the controller and corrupt sonar readings.
#define SCAN_LEFT    120
#define SCAN_RIGHT    40
#define DEFAULT_VIEW  80
#define SERVO_SETTLE_MIN_MS  35UL
#define SERVO_MS_PER_DEG      2UL
#define SERVO_CENTER_CONE_DEG 18

// ============================================================
// BUZZER
// ============================================================
#define BUZZER_PIN   25

// ============================================================
// ULTRASONIC SENSOR
// ============================================================
#define TRIG_PIN  5
#define ECHO_PIN  4
const float  ULTRASONIC_TRASH_LEVEL = 65.0f;
#define ULTRASONIC_TIMEOUT_US           8000UL
#define ULTRASONIC_PING_INTERVAL_MS       40UL
#define ULTRASONIC_BLOCK_CONFIRM_SAMPLES    2
#define ULTRASONIC_CLEAR_CONFIRM_SAMPLES    3
#define ULTRASONIC_SAMPLE_MAX_AGE_MS      180UL
#define ULTRASONIC_SLOW_DISTANCE_CM       90.0f
#define ULTRASONIC_STOP_DISTANCE_CM       60.0f
#define ULTRASONIC_CLEAR_DISTANCE_CM      72.0f
#define ULTRASONIC_RESUME_SPEED_DISTANCE_CM 105.0f
#define VERY_CLOSE_DISTANCE_CM            25.0f
// LiDAR is the single steering authority. Side-sonar steering can be enabled
// only after hardware testing proves it will not fight the center controller.
#define ENABLE_ULTRASONIC_SIDE_NUDGE         0

// ============================================================
// STEPPER MOTORS (FastAccelStepper)
// ============================================================
#define STEP_PIN1  21
#define DIR_PIN1   22
#define STEP_PIN2  13
#define DIR_PIN2   12

const uint32_t MAX_SPEED       = 5600;
const uint32_t CAUTION_SPEED   = 3900;
const uint32_t TURN_SPEED      = 3000;
const uint32_t ACCELERATION    = 4800;
const uint32_t NUDGE_SPEED     = 5000;
const uint32_t GENTLE_STOP_DECEL = 6500;
const uint32_t SAFETY_STOP_DECEL = 14000;
const uint32_t NUDGE_ACCELERATION = 6500;
#define PATH_COMMAND_TIMEOUT_MS 800UL
#define MCU_GO_CONFIRM_PACKETS     2
#define MOTION_GATE_TIMEOUT_MS    900UL
const int32_t  FAR          = 10000000;
const int      STEP_VAL     = 3000;
const uint32_t NUDGE_ACCEL  = (uint32_t)(ACCELERATION * NUDGE_SPEED / MAX_SPEED);

// Apply measured wheel calibration here only after a straight-line test.
// Positive values speed that motor up; negative values slow it down.
const float MOTOR1_TRIM_PCT = 0.0f;
const float MOTOR2_TRIM_PCT = 0.0f;

extern uint32_t lastPrintMs;

// ============================================================
// WHEEL GEOMETRY
// ============================================================
#define STEPS_PER_REV      1600
#define WHEEL_DIAMETER_MM  65.0f
#define WHEELBASE_MM       150.0f

// ============================================================
// OBSTACLE & TRASHBIN CONFIGURATION
// ============================================================
#define OBSTACLE_DISTANCE    ULTRASONIC_STOP_DISTANCE_CM
#define TRASHBIN_TARGET_CM   25.0f   // cm (trashbin proximity threshold)
#define TRASHBIN_MARGIN_CM   4.0f   // cm (margin of error +-4cm: 21.0cm to 29.0cm)
#define MAX_BLOCKED_COUNT    10

// ============================================================
// CONTACT NUMBER
// ============================================================
#define CONTACT_NUMBER  "+639242473078"

// ============================================================
// NUDGE CONFIGURATION (execution only — bridge computes decisions)
// ============================================================
// The BLE bridge handles ALL nudge computation: error, dead-zone,
// front-suppression, direction confirmation, cooldown, proportional
// duration.  The MCU only executes pre-digested N:<ms>|<dir> commands.
// The constants below are execution-side safety caps only.

// Symmetric execution limits. Mechanical bias should be calibrated from a
// measured straight-line test, not compensated with a large hard-coded boost.
const float NUDGE_RIGHT_SPEED_FACTOR = 0.82f;
const float NUDGE_LEFT_SPEED_FACTOR  = 0.82f;
const float NUDGE_LEFT_BOOST_FACTOR  = 1.00f;
const float NUDGE_SPEED_FACTOR       = 0.82f;
#define NUDGE_MIN_CUT_PCT  5U
#define NUDGE_MAX_CUT_PCT 24U

// Hard ceiling on how long a nudge can stay active in the SAME direction.
#define NUDGE_MAX_HOLD_MS  110UL

// Minimum straight-run settling time AFTER a nudge ends before a new
// nudge can be accepted.
#define NUDGE_SETTLE_MS    220UL
#define NUDGE_COMMAND_MAX_AGE_MS 250UL

// ============================================================
// COMMUNICATION / SENSOR FRESHNESS
// ============================================================
#define REQUEST_STATUS_MIN_INTERVAL_MS  80UL
#define BRIDGE_RECOVERY_INTERVAL_MS    2000UL
#define BRIDGE_RX_STALE_MS             3000UL
#define SENSOR_DATA_TIMEOUT_MS         3000UL
#define UART_RX_LINE_MAX                 255U
#define UART_RX_BUDGET_BYTES             192U

// ============================================================
// ACK PROTOCOL
// ============================================================
#define ACK_MSG  "[ESP RECEIVED]"

// ============================================================
// PARSE DATAS (bridge-offloaded — N: commands only)
// ============================================================
// The BLE bridge handles ALL parsing of PATH/BACK_PATH/SIDES data
// from the RasPi.  The MCU only receives pre-digested execution
// commands: STOP, GO, N:<ms>:<intensity>|<dir>, SENSOR:..., and [RESET].
// ParseData therefore only needs to understand N: commands.
class ParseData {
  public:
    enum class NudgeCmdDir { STABLE, NUDGE_LEFT, NUDGE_RIGHT };

  private:
    NudgeCmdDir   cmdDir       = NudgeCmdDir::STABLE;
    unsigned long cmdMs        = 0;
    unsigned int  cmdIntensity = 0;
    unsigned long cmdReceivedMs = 0;
    bool          newCmd       = false;

  public:
    bool parseNudgeCmd(const String& buf);
    bool hasNewCmd() const { return newCmd; }
    bool consumeNudgeCmd(NudgeCmdDir& outDir, unsigned long& outMs, unsigned int& outIntensity) {
      if (newCmd && millis() - cmdReceivedMs > NUDGE_COMMAND_MAX_AGE_MS) {
        reset();
        return false;
      }
      outDir       = cmdDir;
      outMs        = cmdMs;
      outIntensity = cmdIntensity;
      bool had     = newCmd;
      newCmd       = false;
      return had;
    }
    bool consumeNudgeCmd(NudgeCmdDir& outDir, unsigned long& outMs) {
      unsigned int dummy;
      return consumeNudgeCmd(outDir, outMs, dummy);
    }
    NudgeCmdDir   getCmdDir()       const { return cmdDir; }
    unsigned long getCmdMs()        const { return cmdMs; }
    unsigned int  getCmdIntensity() const { return cmdIntensity; }
    void reset() {
      cmdDir = NudgeCmdDir::STABLE;
      cmdMs = 0;
      cmdIntensity = 0;
      cmdReceivedMs = 0;
      newCmd = false;
    }
    void printAllState();
};

class ReceivedDatas {
  public:
    enum class UltrasonicStatus { UNAVAILABLE, EMPTY, HALFWAY, FULL };
    enum class MQ4Status        { UNAVAILABLE, NORMAL, WARNING, DANGER };
    enum class MQ135Status      { UNAVAILABLE, CLEAN, MODERATE, POOR, VERY_POOR };
    enum class MQ137Status      { UNAVAILABLE, NORMAL, WARNING, DANGER };

  private:
    struct Ultrasonic_Data { int value = 999; UltrasonicStatus status = UltrasonicStatus::UNAVAILABLE; };
    struct MQ4_Data        { int value = -1; MQ4Status        status = MQ4Status::UNAVAILABLE;  };
    struct MQ135_Data      { int value = -1; MQ135Status      status = MQ135Status::UNAVAILABLE; };
    struct MQ137_Data      { int value = -1; MQ137Status      status = MQ137Status::UNAVAILABLE; };

    Ultrasonic_Data us;
    MQ4_Data        mq4;
    MQ135_Data      mq135;
    MQ137_Data      mq137;

  public:
    void setUS   (int val);
    void setMQ4  (int val);
    void setMQ135(int val);
    void setMQ137(int val);

    bool parse(const String& raw);

    int              getUSValue();    UltrasonicStatus getUSStatus();
    int              getMQ4Value();   MQ4Status        getMQ4Status();
    int              getMQ135Value(); MQ135Status      getMQ135Status();
    int              getMQ137Value(); MQ137Status      getMQ137Status();

    void printAll();
};

bool sensorReadDecisionMaker(ReceivedDatas& d);

extern ParseData     path;
extern ReceivedDatas data;

// ============================================================
// LIDAR ZONE STRUCT
// ============================================================
struct LidarZones {
  float front      = 999.0f;
  float frontLeft  = 999.0f;
  float frontRight = 999.0f;
  float left       = 999.0f;
  float right      = 999.0f;
  float back       = 999.0f;
  float backLeft   = 999.0f;
  float backRight  = 999.0f;
};

// ============================================================
// ROBOT STATE MACHINE
// ============================================================
enum class GarbyState { IDLE, RUNNING, RETURNING };
extern GarbyState garbyState;
extern bool       resetQueued;

// ============================================================
// NON-BLOCKING NUDGE STATE (execution-side only)
// ============================================================
enum class NudgeDir { NONE, LEFT, RIGHT };
extern NudgeDir      activeNudge;
extern unsigned long nudgeStartMs;
extern unsigned long nudgeDurationMs;
extern unsigned long nudgeHoldStartMs;

// ============================================================
// GLOBAL OBJECT DECLARATIONS
// ============================================================
extern HX711             scale;
extern HardwareSerial    ESP_Serial;
extern HardwareSerial    Air780;
extern Servo             servo;
extern FastAccelStepperEngine engine;
extern FastAccelStepper* stepper1;
extern FastAccelStepper* stepper2;

// ============================================================
// GLOBAL STATE DECLARATIONS
// ============================================================
extern LidarZones    zones;
extern bool          lidarBlockedActive;
extern bool          lidarControlled;
extern unsigned long lidarBlockedStart;
extern unsigned long lidarLastPeriodicSMS;
extern unsigned long lidarLastLRScan;
extern int           blockedCount;
extern bool          idleMode;
extern bool          lastConnected;
extern bool          movingForward;
extern bool          blockedSMSSent;
extern bool          loadcellSMSSent;
extern bool          buzzerState;
extern unsigned long lastBeepTime;
extern float frontDistance;
extern float leftDistance;
extern float rightDistance;
extern bool isTrashbinFull;
extern bool shouldStop;
extern bool sensorTripped;
extern bool pathCommandSeen;
extern bool linkFaultActive;
extern unsigned long lastPathCommandMs;
extern unsigned long lastSensorPacketMs;
extern uint8_t clearPathCommandCount;

extern unsigned long lastIdlePrintMs;

// ============================================================
// FUNCTION PROTOTYPES
// ============================================================
// Motor control
bool turnRight(int32_t step = STEP_VAL);
bool turnLeft (int32_t step = STEP_VAL);
void emergencyStopMotors();
void smoothDecelStopMotors();
void activeBrakeStopMotors();
void startStraight();
void restoreStraight();


// ---- NEW: request status from RasPi ----
void requestStatus();

void nudgeLeftContinuous (float delayMs, unsigned int intensityPct = 0);
void nudgeRightContinuous(float delayMs, unsigned int intensityPct = 0);
void updateNudge();

bool moveToTarget(long target1, long target2, bool isADJ = false, bool isNudgeEnabled = true);
bool moveDistance(int32_t steps = FAR, bool isADJ = false, bool isNudgeEnabled = true);
bool runStart();
bool returnToPointB();

// Sensor
float readDistanceRaw();
float getDistance();
float scanAngle(int angle);
void initUltrasonicMonitor();
void serviceUltrasonic();
bool frontObstacleDetected();
bool frontEmergencyObstacleDetected();
bool frontDistanceFresh();
float latestFrontDistance();

// SMS / Air780E
String sendAT(const String& cmd, unsigned long timeout = 3000);
void   powerOnAir780();
bool   waitForModule (int maxAttempts = 20);
bool   waitForNetwork(int maxAttempts = 20);
void   sendSMS(const String& phoneNumber, const String& message);
void   queueSMSAlert(const String& message);
bool   smsAlertBusy();
bool   modemReady();
void   startModemInitialization();

// Path / sensor handlers
bool handlePath();
bool handleSensor();

// State machine helpers
void pollESP();
bool pathCommandFresh();
void enforcePathWatchdog();
void haltAndWait(const String& reason);
void movementGate(bool isNudgeEnabled = true);

bool safeMoveDistance(int32_t steps, bool isADJ = false, bool isNudgeEnabled = true);
bool safeTurnLeft (int32_t step = STEP_VAL);
bool safeTurnRight(int32_t step = STEP_VAL);
void fullReset();
void printIdleUptime();
void serviceBridgeRecovery();
void responsiveDelay(unsigned long durationMs);

// Utility
void flushESPSerial();
void buzzerTask(void *pvParameters);
bool checkLoad(float threshold = LOAD_TRIGGER_KG);
