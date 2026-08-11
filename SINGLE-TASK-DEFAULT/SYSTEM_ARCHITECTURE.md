# GARBY System Architecture

Last updated: 2026-08-11

This document describes the current GARBY autonomous waste-collection robot after the integration and safety review. It covers the stack from hardware execution up to the Android app and Firebase Realtime Database.

Use `DEPLOYMENT_AND_ACCEPTANCE.md` for the ordered hardware rollout and
`VALIDATION_RESULTS.md` for the completed automated checks.

## 1. System Purpose

GARBY is an autonomous waste-collection robot that:

- waits at base until load or gas conditions require dispatch;
- navigates planned corridor segments using the main ESP32 motor controller;
- receives path safety and lane-centering decisions from a BLE bridge ESP32;
- uses the Raspberry Pi for LiDAR processing, Firebase sync, app reset relay, and BLE transport;
- shows telemetry and reset status in the Android app.

The safety rule is fail-closed: missing, stale, malformed, out-of-order, or disconnected path data must stop motion. The Android app never authorizes motion. It only monitors and sends a reset/return intent that the Pi and firmware must still execute safely.

## 2. Component Map

```text
Android App
  Firebase RTDB read/write
        |
        v
Raspberry Pi
  ROS 2 YDLidar reader
  Pi serial sensor reader
  Firebase heartbeat/sensor writer
  Firebase reset command bridge
  BLE client
        |
        v
ESP32 BLE Bridge
  NimBLE server
  Path packet parser
  Nudge calculator
  UART relay to MCU
        |
        v
Main ESP32 MCU
  Stepper execution
  Servo + ultrasonic local obstacle check
  HX711 load-cell reading
  Air780E SMS alerts
  Robot state machine
```

## 3. Main ESP32 MCU

Source: `NAPHTALI_CODE_V2/`

Active files are the root sketch files:

- `NAPHTALI_CODE_V2.ino`
- `NAPHTALI_CODE_V2.h`
- `NAPHTALI_CODE_V2.cpp`
- `pointsRun.ino`

The `src/` folder is a managed backup/copy and is not the active edit target.

### 3.1 Main Responsibilities

The MCU is the execution layer. It does not parse raw LiDAR `PATH`, `BACK_PATH`, or `SIDES` payloads. It receives only these commands from the BLE bridge over UART:

- `STOP`, `STOP:HUMAN`, `STOP:STALE`, `STOP:LINK`, `STOP:WAITING_DATA`, `STOP:PROTOCOL`
- `GO`
- `N:<ms>:<intensity>|<NUDGE_LEFT|NUDGE_RIGHT|STABLE>`
- `SENSOR:US=..|MQ4=..|MQ137=..|MQ135=..`
- `[RESET]`

### 3.2 State Machine

```text
IDLE
  - request path/sensor status every 500 ms
  - read HX711 load cell
  - send LOAD_CELL:<kg> to the Pi through BLE bridge notifications
  - dispatch when load >= 1.0 kg or gas danger is confirmed

RUNNING
  - execute outbound route in pointsRun.ino
  - request path status every 100 ms during motion
  - stop on stale/missing/blocked path
  - consume bridge-computed nudge taps only on route segments that enable nudging
  - queue RETURNING when [RESET] is received

RETURNING
  - execute return route
  - call fullReset()
  - notify Pi/app with [IDLE]
```

### 3.3 Motor Configuration

Current motor constants:

| Constant | Value | Purpose |
| --- | ---: | --- |
| `MAX_SPEED` | 5600 Hz | Straight cruise, approximately 0.71 m/s |
| `CAUTION_SPEED` | 3900 Hz | Approach speed, approximately 0.50 m/s |
| `TURN_SPEED` | 3000 Hz | Turn motion speed |
| `ACCELERATION` | 4800 Hz/s | Normal acceleration |
| `GENTLE_STOP_DECEL` | 6500 Hz/s | Planned stop deceleration |
| `SAFETY_STOP_DECEL` | 14000 Hz/s | Obstacle/stale-path stop |
| `NUDGE_ACCELERATION` | 6500 Hz/s | Short steering tap acceleration |
| `PATH_COMMAND_TIMEOUT_MS` | 800 ms | Final MCU path freshness watchdog |

These values were lowered from the previous more aggressive setup to reduce missed steps, uncomfortable motor noise, and brownout-like behavior under load.

### 3.4 Servo + Ultrasonic

Current servo/sonar settings:

| Constant | Value |
| --- | ---: |
| `SCAN_RIGHT` | 40 degrees |
| `DEFAULT_VIEW` | 80 degrees |
| `SCAN_LEFT` | 120 degrees |
| `SERVO_SETTLE_MIN_MS` | 35 ms |
| `SERVO_MS_PER_DEG` | 2 ms/degree |
| `ULTRASONIC_TIMEOUT_US` | 8000 us |
| `ULTRASONIC_PING_INTERVAL_MS` | 40 ms |
| `ULTRASONIC_SLOW_DISTANCE_CM` | 90 cm |
| `ULTRASONIC_STOP_DISTANCE_CM` | 60 cm, two samples |
| `VERY_CLOSE_DISTANCE_CM` | 25 cm, one sample |
| `ULTRASONIC_CLEAR_DISTANCE_CM` | 72 cm, three samples |

During motion and turns the servo stays at `DEFAULT_VIEW` so the ultrasonic
beam continuously covers the chassis centerline. Echo timing is captured by an
interrupt; the motor loop no longer blocks in `pulseIn()`. A confirmed obstacle
remains latched until three clear samples arrive, and the robot also needs fresh
LiDAR `GO` permission before resuming. Manual stopped scans retain 40/80/120
degree support. The same confirmed 60 cm latch is enforced during straight
segments, turns, and short adjustment moves. LiDAR remains the only steering
authority.

### 3.5 Load Cell Integration

The MCU reads one ready HX711 sample at a time without the previous multi-sample
blocking delay. Once per second when a filtered value is available, it sends:

```text
LOAD_CELL:<kg>
```

The BLE bridge forwards this as a BLE notification to the Pi, and the Pi writes it into Firebase:

- `/MCU/VALUES/LOAD_CELL`
- `/RASPI/VALUES/LOAD_CELL/WEIGHT_IN_KG`
- `/devices/garby-bin-01/sensors/weight`

### 3.6 Reset Completion

`fullReset()` now sends:

```text
[IDLE]
```

This is the completion signal for the Pi reset bridge. Android waits for Firebase reset status `done`, which the Pi writes only after it receives `[IDLE]`.

Pi reset retries are idempotent while the MCU is already `RETURNING`; a repeated
`[RESET]` is acknowledged but cannot abort the route or produce an early
`[IDLE]`.

## 4. ESP32 BLE Bridge

Source: `BLE_Receiver-Final/BLE_Receiver-Final.ino`

### 4.1 Main Responsibilities

The BLE bridge is the protocol and steering brain:

- receives compact path and side packets from the Pi over BLE;
- validates packet sequence numbers;
- rejects stale/out-of-order path and side packets;
- converts path status into `STOP` or `GO`;
- computes nudge taps from side-wall and diagonal LiDAR measurements;
- forwards `SENSOR`, `[RESET]`, and MCU notifications.

### 4.2 BLE and UART

| Link | Settings |
| --- | --- |
| Pi to bridge | BLE, device name `GarbyESP32` |
| BLE write UUID | `beb5483e-36e1-4688-b7f5-ea07361b26a8` |
| BLE notify UUID | `beb5483e-36e1-4688-b7f5-ea07361b26a9` |
| Bridge to MCU | UART1, 115200 baud, GPIO 18 RX / 19 TX |

### 4.3 Compact Packet Protocol

Pi to bridge:

```text
P:<seq>|F=<C|O|H|S>|B=<C|O|H|S>
S:<seq>|L=..|R=..|F=..|B=..|FL=..|FR=..|BL=..|BR=..|T=..
SENSOR:US=..|MQ4=..|MQ137=..|MQ135=..
[RESET]
[RASPI READY]
```

Codes:

- `C` = clear
- `O` = obstacle
- `H` = legacy human-tagged obstacle code, accepted only for protocol compatibility
- `S` = stale LiDAR stream

Bridge to MCU:

```text
STOP...
GO
N:<ms>:<intensity>|<dir>
SENSOR:...
[RESET]
```

### 4.4 Bridge Watchdogs

| Watchdog | Value | Action |
| --- | ---: | --- |
| Queued path too old | 250 ms | Drop it and fail closed |
| Queued sides too old | 200 ms | Drop it; never replay a stale tap |
| Valid path data stale | 650 ms | Repeat `STOP:STALE` or `STOP:WAITING_DATA` |
| BLE link silent | 8000 ms | Send `STOP:LINK` and reconnect/advertise |
| MCU generic ACK missing | 3000 ms | Send `STOP:MCU_LINK` |
| Stale stop repeat | 500 ms | Reassert stop while stale |

### 4.5 Nudge Calculation

Current nudge constants:

| Constant | Value |
| --- | ---: |
| Dead zone | 15 cm |
| Reverse hysteresis | 10 cm |
| Duration | 35 to 75 ms |
| Intensity | 8 to 22 percent speed cut |
| Error scale | 60 cm |
| Cooldown | 1100 ms |
| Confirmation | 5 packets |
| Lateral weight | 0.85 |
| Heading weight | 0.15 |
| EMA alpha | 0.15 |

The bridge also suppresses nudging:

- toward a wall already at or below 20 cm;
- when front clearance is low;
- during startup grace/ramp;
- while the MCU is stop-latched;
- when a required side distance is missing or any transmitted numeric field is
  malformed, non-finite, or non-positive.

Malformed steering input produces `N:0:0|STABLE` and resets direction
confirmation, so valid samples on either side of corruption cannot be counted
as one continuous correction request.

Eight accepted side samples are fully suppressed after startup, followed by an
eight-sample ramp. The ramp advances on every fresh side packet rather than on
cooldown firings, and every nonzero correction stays explicitly within 35-75 ms
and 8-22 percent. This gives proportional intensity without a long weak-control
window or aggressive startup cliff.

## 5. Raspberry Pi

Source: `RasPi/final_w_serial.py`

### 5.1 Main Responsibilities

The Pi is the supervisor and integration node:

- starts and monitors the YDLidar ROS 2 driver;
- subscribes to `/scan`;
- builds compact path and side packets;
- reads the Pi-side serial sensor board;
- writes sensor values, heartbeat, and load-cell notifications to Firebase;
- relays Android reset requests from Firebase to the robot via BLE;
- owns the BLE client connection to `GarbyESP32`.

### 5.2 LiDAR Processing

Eight named sectors are tracked for safety, each with a centered steering slice:

- `FRONT`
- `FRONT_LEFT`
- `LEFT`
- `BACK_LEFT`
- `BACK`
- `BACK_RIGHT`
- `RIGHT`
- `FRONT_RIGHT`

Safety thresholds:

| Area | Threshold |
| --- | ---: |
| Front path | 95 cm |
| Back path | 35 cm |
| LiDAR stale | 0.8 s |

Each valid scan point is classified once into exactly one nearest 45-degree
safety sector, giving gap-free 360-degree collision coverage. Steering uses a
separate centered 22-degree slice for cleaner wall medians, with its containing
safety sector as the fallback when the narrow slice is sparse. Safety uses a
low-percentile proximity estimate that rejects one isolated speck. Close
front/back obstacles bypass temporal outlier confirmation, and heading tilt is
clamped to plus or minus 20 cm before transmission.

The Pi-side ultrasonic reading is the trash-level sensor and is never used to
classify a corridor obstacle. LiDAR therefore reports a generic obstacle. The
separate servo-mounted front sonar is enforced locally by the executor, where
it can stop motion without waiting for a Pi/BLE/Firebase round trip.

### 5.3 Pi Serial Sensor Board

The Pi reads `/dev/ttyAMA0` at 9600 baud.

Expected lines:

```text
ULTRASONIC:<cm>
MQ4:<value>
MQ137:<value>
MQ135:<value>
```

Each sensor is tracked independently. Live values are coalesced into atomic
updates at no more than one five-second cadence. If a sensor is explicitly
unavailable, the serial port disconnects, or that sensor becomes stale for eight
seconds, the Pi writes its sentinel exactly once for that outage:

```text
ULTRASONIC = 999
MQ4 = -1
MQ137 = -1
MQ135 = -1
```

Android treats these sentinels as stale/offline, not as safe values.
The first valid recovery is written immediately and rearms one future outage
transition. UART reconnects use a bounded 1-30 second backoff and blocking
250 ms reads, so an unplugged sensor board cannot create a CPU hot-loop.

### 5.4 BLE Client

The Pi uses a coalescing BLE queue:

- latest path replaces older path;
- latest sides replaces older sides;
- latest sensor replaces older sensor;
- control messages remain queued;
- urgent safety/reset messages go to the front.

Path packets use acknowledged GATT writes. Steering and telemetry use the serialized shared write lock so concurrent BLE writes do not corrupt the client state.

### 5.5 Firebase Reset Bridge

Android writes reset intent to:

```text
/devices/garby-bin-01/commands/reset
/APP/isReadyToReset
```

The Pi reset bridge:

1. polls Firebase every two seconds;
2. rejects stale reset commands older than 120 seconds;
3. rejects reset commands more than 30 seconds in the future;
4. rejects timestamp-less old `pending` reset commands;
5. relays fresh reset requests as BLE `[RESET]`;
6. marks Firebase status `ack`;
7. repeats `[RESET]` every 3 seconds while in flight;
8. waits for MCU notification `[IDLE]`;
9. marks Firebase status `done` and clears `/APP/isReadyToReset`.

This prevents old pending database commands from replaying when the Pi restarts.

### 5.6 Thermal and Link Diagnostics

The 30-second heartbeat also publishes change-only Pi CPU temperature,
throttling flags, BLE state, LiDAR health, and sensor-UART health. Normal
deployment should use `python3 final_w_serial.py --headless` to avoid GUI render
load. The LiDAR supervisor restarts an exited or unhealthy driver with bounded
backoff; stale scan data still produces an immediate fail-closed path packet.
If neither Pi thermal source is readable, thermal fields remain unknown rather
than falsely publishing `THERMAL OK`.

## 6. Firebase Realtime Database

The project now writes both legacy paths and app-friendly device paths.

### 6.1 Legacy Paths

```text
/RASPI/STATES/launchTime
/RASPI/STATES/recent_uptime
/RASPI/STATES/lastSeen
/RASPI/VALUES/ULTRASONIC_SENSOR/CM_DISTANCE
/RASPI/VALUES/MQ135_SENSOR/AIR_QUALITY
/RASPI/VALUES/MQ137/AMMONIA
/RASPI/VALUES/MQ4_SENSOR/METHANE
/RASPI/VALUES/LOAD_CELL/WEIGHT_IN_KG
/MCU/VALUES/LOAD_CELL
```

All live sensor values include `updatedAt` epoch milliseconds.

### 6.2 Device Paths Used by Android

```text
/devices/garby-bin-01/status/lastSeen
/devices/garby-bin-01/status/recent_uptime
/devices/garby-bin-01/status/cpuTemperatureC
/devices/garby-bin-01/status/thermalWarning
/devices/garby-bin-01/status/throttledFlags
/devices/garby-bin-01/status/bleConnected
/devices/garby-bin-01/status/lidarHealthy
/devices/garby-bin-01/status/sensorSerialConnected
/devices/garby-bin-01/sensors/level
/devices/garby-bin-01/sensors/weight
/devices/garby-bin-01/sensors/mq135
/devices/garby-bin-01/sensors/mq137
/devices/garby-bin-01/sensors/mq4
/devices/garby-bin-01/commands/reset
```

Sensor shape:

```json
{
  "value": 0,
  "unit": "cm",
  "sensorType": "ULTRASONIC_SENSOR",
  "updatedAt": 1786360000000
}
```

Reset command shape:

```json
{
  "requestedAt": 1786360000000,
  "requestedBy": "firebase-auth-uid",
  "status": "pending"
}
```

Known reset statuses:

- `pending`
- `ack`
- `done`
- `failed`

### 6.3 Database Write Optimization

All Pi writes are coalesced into root-level atomic multi-location updates on a
dedicated worker. Network failures retry with bounded 1-30 second backoff and
never block LiDAR/BLE safety threads. Heartbeat/health is 30 seconds, live
sensors are at most every five seconds, and load-cell writes are change-driven
at 0.05 kg with a 30-second refresh.

The checked-in database template preserves every path in the supplied export
but intentionally leaves all runtime FCM token values empty. Android populates
the three compatibility token paths atomically when it obtains a current token.

## 7. Android App

Source: `Garby_MobileApp/`

### 7.1 Main Responsibilities

The Android app:

- signs in with Firebase anonymous auth;
- reads cloud connection state;
- reads robot heartbeat and sensor values;
- displays stale/offline states instead of substituting safe zeroes;
- sends explicit reset intent;
- waits for correlated reset completion.

It is not a motion controller.

### 7.2 Dashboard

Dashboard telemetry:

- robot online/offline from `/RASPI/STATES/lastSeen` or `/devices/garby-bin-01/status/lastSeen`;
- trash-level ultrasonic distance in centimeters;
- load-cell weight in kg;
- MQ135 air quality;
- MQ137 ammonia;
- MQ4 methane;
- Pi CPU temperature/throttling and BLE/LiDAR/sensor-board link health when the
  updated bridge has published those optional fields.

Missing timestamps are stale. Sentinel values are stale/offline:

- ultrasonic `999`;
- gas `-1`.

The app subscribes to the exact supplied legacy `/RASPI/VALUES/...` records and
the device mirrors without listening to the entire database tree. It chooses
the newest valid copy, does not mistake `updatedAt` for a sensor value, treats
missing timestamps as stale, and does not label raw ultrasonic centimeters as
a percentage.

### 7.3 Reset Flow

Reset flow:

1. app requires a validated Android network connection;
2. app requires Firebase `.info/connected`;
3. app requires an authenticated Firebase user;
4. app atomically writes the structured reset command with
   `ServerValue.TIMESTAMP` and `/APP/isReadyToReset = true`;
5. app waits up to 60 seconds for a matching reset command to become `done` or `failed`;
6. old terminal statuses are rejected using `requestedAt` and `requestedBy` correlation.

### 7.4 App Fixes From Review

- notification deep links no longer bypass authentication;
- failed sign-in no longer routes to the dashboard as authenticated;
- reset no longer falls back to a hard-coded operator id;
- missing `updatedAt` no longer appears fresh;
- missing robot `lastSeen` no longer appears online;
- weight thresholds now match the 1.0 kg firmware dispatch threshold;
- gas thresholds now match the MCU MQ warning/danger ranges more closely;
- removed a dead custom executor utility with unbounded queues;
- the legacy compatibility flag can no longer turn a structured `pending` or
  `ack` reset into a false completion;
- FCM compatibility paths are updated atomically and token values are not
  written to logs.

## 8. Function-Level Rundown

### MCU

| Function | Role |
| --- | --- |
| `setup()` | Initializes UARTs, Air780E, steppers, servo, ultrasonic pins, HX711, and BLE bridge handshake |
| `loop()` | Runs the IDLE/RUNNING/RETURNING state machine |
| `pollESP()` | Non-blocking UART parser for bridge commands |
| `enforcePathWatchdog()` | Latches stop if fresh path commands stop arriving |
| `movementGate()` | Requires fresh `GO` before a route segment starts |
| `moveToTarget()` | Executes straight segment, consumes nudge taps, checks ultrasonic center obstacle |
| `activeBrakeStopMotors()` | Fast controlled safety stop |
| `fullReset()` | Clears state, returns to IDLE, notifies `[IDLE]` |
| `runStart()` | Outbound route |
| `returnToPointB()` | Return route |

### BLE Bridge

| Function | Role |
| --- | --- |
| `processAndRelayMessage()` | Routes BLE writes by message type |
| `handlePathPacket()` | Validates path sequence and sends STOP/GO |
| `handleSidesPacket()` | Validates sides sequence and computes nudge only for current path sequence |
| `computeNudgeCommand()` | Lane-centering and anti-zigzag algorithm |
| `relayStop()` | Stop latch plus stable nudge |
| `handleMcuLine()` | Forwards MCU notifications to Pi |
| `loop()` | UART drain, connected notice, stale path/link watchdogs |

### Raspberry Pi

| Function | Role |
| --- | --- |
| `receiveSerial()` | Reads sensor serial and writes Firebase sensor frames |
| `raspi_heartbeat_loop()` | Updates robot online heartbeat |
| `firebase_reset_command_loop()` | Relays app reset intent and rejects stale replay |
| `on_demand_status_responder()` | Responds to MCU `[REQUEST-STATUS]` with path, sides, and sensor packets |
| `ble_notify_handler()` | Reacts to MCU requests, load-cell notifications, and `[IDLE]` reset completion |
| `LidarDistanceReader.scan_callback()` | Builds gap-free safety-sector and narrow steering histories in one pass |
| `LidarDistanceReader.build_status_packets()` | Emits compact `P:` and `S:` packets |

### Android

| Class | Role |
| --- | --- |
| `AuthViewModel` | Firebase anonymous auth and RTDB prewarm |
| `GarbyRealtimeDb` | RTDB gateway for telemetry, heartbeat, reset command, FCM token |
| `DeviceViewModel` | Dashboard state and freshness re-evaluation |
| `MainDashboard` | Live telemetry UI |
| `ResetViewModel` | Reset send, correlation, timeout, and result state |
| `ResetTrashbinScreen` | Reset confirmation and progress UI |

## 9. Known Remaining Physical Validation

The code has been statically reviewed and optimized, but these still require robot hardware validation:

- wheels-lifted app reset: pending -> ack -> return -> done;
- stale old Firebase pending reset is rejected and does not move the robot;
- sensor board unplugged: database shows ultrasonic `999`, gas `-1`, app shows offline/stale;
- long straight corridor: no nudge flip-flop or wall hunting;
- person/object crossing: LiDAR stops before contact and the independent local
  center sonar stops at 60 cm without relying on the trash-level sensor;
- motor temperature/noise at `MAX_SPEED = 5600`;
- centered servo alignment at 80 degrees and stopped/manual sweep reliability
  at 40/80/120 degrees;
- ESP32-safe 3.3 V ultrasonic echo level and measured stopping distance at the
  actual payload;
- 30-60 minute Pi/ESP32/motor-driver thermal run, including throttling flags and
  stable power-supply checks;
- load-cell value appears in Firebase/app while idle.
