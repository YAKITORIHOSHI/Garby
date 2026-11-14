#pragma once
// ============================================================
// LIBRARIES
// ============================================================
#include <ESP32Servo.h>
#include <FastAccelStepper.h>
#include <HardwareSerial.h>
#include <HX711.h>
#include <Arduino.h>

// Main executor clock used for UART parsing and hardware-timed step pulses.
#define EXECUTOR_CPU_MHZ 240

// Routine motion/modem dumps are opt-in. State transitions and faults stay on.
#ifndef GARBY_VERBOSE_DIAGNOSTICS
#define GARBY_VERBOSE_DIAGNOSTICS 0
#endif

// ============================================================
// LOAD CELL
// ============================================================
#define LOADCELL_DOUT_PIN  33
#define LOADCELL_SCK_PIN   32
constexpr float LOAD_TRIGGER_KG = 4.0f;
constexpr float LOAD_VALID_MAX_KG = 20.0f;
#define LOAD_SAMPLE_MAX_AGE_MS 1000UL
const float  CALIBRATION_FACTOR  = 100000.0f;

void startLoadCellMonitor();
bool getLoadCellSample(float& kilograms, uint32_t& sequence);

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
#define SCAN_LEFT    140
#define SCAN_RIGHT    0
#define DEFAULT_VIEW  70
#define SERVO_SETTLE_MIN_MS  35UL
#define SERVO_MS_PER_DEG      2UL
#define SERVO_CENTER_CONE_DEG 18

// ============================================================
// BUZZER (2026-11-12: one persistent task plays a pattern per robot state)
// ============================================================
#define BUZZER_PIN   25
// Active buzzer (or its driver transistor) on while the pin is HIGH.
#define BUZZER_ACTIVE_HIGH 1
// Driving a route: a short beep every BUZZER_RUN_PERIOD_MS so people in the
// hallway hear GARBY coming. Held by an obstacle: two chirps ("waiting for you").
// Low-power hold: one short chirp, less often.
#define BUZZER_RUN_ON_MS            90UL
#define BUZZER_RUN_PERIOD_MS      1200UL
#define BUZZER_HOLD_ON_MS           60UL
#define BUZZER_HOLD_GAP_MS         110UL
#define BUZZER_HOLD_PERIOD_MS     2000UL
#define BUZZER_LOW_POWER_ON_MS      50UL
#define BUZZER_LOW_POWER_PERIOD_MS 5000UL

enum class BuzzerPattern : uint8_t { OFF, RUNNING, HOLDING, LOW_POWER };
enum class BuzzerAlert : uint8_t { NONE, READY };
// True when the buzzer should sound `msIntoPattern` after the pattern began.
bool buzzerPatternLevel(BuzzerPattern pattern, uint32_t msIntoPattern);
void startBuzzer();
void buzzerAlert(BuzzerAlert alert);
BuzzerPattern currentBuzzerPattern();

// ============================================================
// LOW-POWER HOLD (2026-11-12)
// ============================================================
// A hold that has seen no clearance for LOW_POWER_AFTER_MS slows everything that
// does not decide when to resume: CPU clock, status polling, sonar pinging and the
// buzzer. The wheels are already stopped and every resume gate is unchanged; the
// first clear evidence restores normal power before the confirmations finish.
#define LOW_POWER_AFTER_MS              10000UL
#define LOW_POWER_CPU_MHZ                  80   // APB stays 80 MHz: UART, steppers, servo unchanged
#define LOW_POWER_STATUS_INTERVAL_MS      400UL
#define LOW_POWER_SURVEY_EVERY_MS        4000UL
// Stepper driver ENABLE is not wired (PINOUTS.txt). Wiring it to a free GPIO and
// setting this cuts the drivers' holding current during a long hold, by far the
// largest saving. -1 = not wired.
#define MOTOR_ENABLE_PIN                   -1
#define MOTOR_ENABLE_ACTIVE_LOW             1
#define MOTOR_ENABLE_SETTLE_MS             30UL

// ============================================================
// ULTRASONIC SENSOR
// ============================================================
#define TRIG_PIN  5
#define ECHO_PIN  4
#define ULTRASONIC_TIMEOUT_US          18000UL
#define ULTRASONIC_PING_INTERVAL_MS       40UL
#define ULTRASONIC_PARKED_PING_INTERVAL_MS 250UL   // forward-only while idle or parked
#define ULTRASONIC_ECHO_STUCK_MS          500UL   // ECHO held high this long = no echo
#define ULTRASONIC_BLOCK_CONFIRM_SAMPLES    2
#define ULTRASONIC_CLEAR_CONFIRM_SAMPLES    3
// The sweep revisits the forward bearing every ~0.1 s driving and ~0.2 s (worst
// ~0.3 s) while surveying, so a forward sample stays usable this long (was 180 ms,
// forward only).
#define ULTRASONIC_SAMPLE_MAX_AGE_MS      600UL
#define ULTRASONIC_SLOW_DISTANCE_CM       90.0f
#define ULTRASONIC_STOP_DISTANCE_CM       60.0f
#define ULTRASONIC_CLEAR_DISTANCE_CM      72.0f
#define ULTRASONIC_RESUME_SPEED_DISTANCE_CM 105.0f
#define VERY_CLOSE_DISTANCE_CM            25.0f
// LiDAR is the single steering authority. Side-sonar steering can be enabled
// only after hardware testing proves it will not fight the center controller.
#define ENABLE_ULTRASONIC_SIDE_NUDGE         0

// Where the servo points the sonar and what each echo means (2026-11-12).
#include "SonarSweep.h"

// ============================================================
// STEPPER MOTORS (FastAccelStepper)
// ============================================================
#define STEP_PIN1  21
#define DIR_PIN1   22
#define STEP_PIN2  13
#define DIR_PIN2   12

// Which DIR level makes each motor's step count rise. A rising count is the software
// forward contract in routes and steering. The owner's 2026-11-09 floor report says
// the November 7 true/false build still drove backward, so invert the pair here only.
// The 2026-11-11 Calib floor test used this same false/true pair. Which motor is the
// left wheel is no longer a fixed assumption: HeadingControl starts from a guess and
// the MPU confirms or swaps it on the first turn and the steering probe.
const bool MOTOR1_DIR_HIGH_COUNTS_UP = false;
const bool MOTOR2_DIR_HIGH_COUNTS_UP = true;
const char EXECUTOR_BUILD_ID[] = "RECONNECT-20261114";

const uint32_t MAX_SPEED       = 5600;
const uint32_t CAUTION_SPEED   = 3900;
const uint32_t TURN_SPEED      = 3000;
const uint32_t ACCELERATION    = 4800;
const uint32_t GENTLE_STOP_DECEL = 6500;
const uint32_t SAFETY_STOP_DECEL = 14000;
#define MOTOR_PROGRESS_TIMEOUT_MS 2000UL
#define PATH_COMMAND_TIMEOUT_MS 800UL
#define MCU_GO_CONFIRM_PACKETS     2
#define MCU_GO_CONFIRM_MIN_INTERVAL_MS 80UL
#define MOTION_GATE_TIMEOUT_MS    900UL
const int32_t  FAR          = 10000000;

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
// CONTACT NUMBER
// ============================================================
#define CONTACT_NUMBER  "+639242473078"

// ============================================================
// MPU HEADING CONTROL (MPU_and_Motor-Latest/Calib, merged 2026-11-11)
// ============================================================
// The S3 bridge streams calibrated yaw (M lines) and the Pi's LiDAR wall fit
// (W lines). This controller turns by MPU-measured angle, checks each turn
// against the walls, and steers every long straight continuously: to the
// measured hallway direction and centre when walls are seen, to the gyro route
// heading otherwise. The old N:<ms>:<cut>|<dir> taps are no longer used.
// Tunables and the line formats are in HeadingControl.h.

// ============================================================
// COMMUNICATION / SENSOR FRESHNESS
// ============================================================
#define REQUEST_STATUS_MIN_INTERVAL_MS  80UL
#define PARKED_STATUS_INTERVAL_MS      500UL
#define BRIDGE_RECOVERY_INTERVAL_MS    2000UL
#define BRIDGE_RX_STALE_MS             3000UL
#define SENSOR_DATA_TIMEOUT_MS         3000UL
#define UART_RX_LINE_MAX                 255U
#define UART_RX_BUDGET_BYTES             512U
#define MODEM_PASSTHROUGH_BUDGET_BYTES    128U

// Straights longer than this cruise with heading hold, then hand the last
// FINISH_STEPS to the stepper ramp so the mean step count ends exactly.
// 3267 steps stop 5600 steps/s at 4800 steps/s^2; 280 steps are margin.
const int32_t FINISH_STEPS = (int32_t)((uint64_t)MAX_SPEED * MAX_SPEED /
                                       (2ULL * ACCELERATION)) + (int32_t)(MAX_SPEED / 20U);

// ============================================================
// ACK PROTOCOL
// ============================================================
#define ACK_MSG  "[ESP RECEIVED]"

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
// Stationary route-fault latch: set when a route segment fails so the route is
// never replayed from the robot's now-unknown physical position. Cleared by an
// explicit [RESET] (supervised physical recovery) or on the next IDLE entry.
extern bool       routeFaultLatched;
extern bool       outboundComplete;
void reportRouteFault();

#include "HeadingControl.h"

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
extern HeadingControl    heading;

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
// Turns are MPU-measured angles in degrees, absolute along the route.
bool turnRight(float degrees);
bool turnLeft (float degrees);
void emergencyStopMotors();
void smoothDecelStopMotors();
void activeBrakeStopMotors();
void startStraight();


// ---- NEW: request status from RasPi ----
void requestStatus();

// Every forward straight longer than FINISH_STEPS holds the MPU route heading.
// isNudgeEnabled additionally fuses the LiDAR walls: hallway direction and centring.
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
void initOperatorControls();
bool serviceOperatorStop();
bool operatorAutomaticStartAllowed();
bool frontObstacleDetected();
bool frontEmergencyObstacleDetected();
bool frontDistanceFresh();
float latestFrontDistance();

// SMS / Air780E
enum class SmsNoticeKind : uint8_t {
  STARTUP, TRIP, ROUTE, EMERGENCY, ROUTE_FAULT, LINK,
  INTERNET, CLOUD, LIDAR, SENSORS, THERMAL, BLOCKED, LEGACY, COUNT
};
enum class SmsSubmitResult : uint8_t {
  SUBMITTED, RETRYABLE_FAILURE, FAILED, INDETERMINATE
};
String sendAT(const String& cmd, unsigned long timeout = 3000);
void   powerOnAir780();
bool   waitForModule (int maxAttempts = 20);
bool   waitForNetwork(int maxAttempts = 20);
SmsSubmitResult sendSMS(const String& phoneNumber, const String& message);
bool   queueSMSAlert(const String& message);
bool   queueSMSNotice(SmsNoticeKind kind, const char* message, char state = 'E');
bool   serviceSmsQueueOnce(uint32_t now);
bool   parseHealthStatus(const String& packet);
void   serviceNoticeMonitoring(uint32_t now);
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
// False only for an explicit reset, a latched route fault or the operator stop.
// A blocked or stale path is waited out (2026-11-12), never a route failure.
bool movementGate(bool isNudgeEnabled = true);

bool safeMoveDistance(int32_t steps, bool isADJ = false, bool isNudgeEnabled = true);
bool safeTurnLeft (float degrees);
bool safeTurnRight(float degrees);
bool beginRouteHeading();
// Pre-run centring before a long hallway straight (2026-11-12): stand still,
// measure the walls, face the hallway axis and sidestep onto the centre line.
// False only when the route must stop; *progressSteps = forward travel used.
bool centerBeforeLongRun(int32_t* progressSteps);
// Low-power hold support (motor-driver enable pin, if wired) and its state.
void initPowerControl();
bool lowPowerHoldActive();
void fullReset();
void printIdleUptime();
void serviceBridgeRecovery();
void responsiveDelay(unsigned long durationMs);

// Utility
void flushESPSerial();
bool checkLoad(float threshold = LOAD_TRIGGER_KG);
