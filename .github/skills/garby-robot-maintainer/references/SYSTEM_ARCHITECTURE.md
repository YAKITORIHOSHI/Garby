# GARBY Autonomous Waste Collection Robot - Safety-First System Architecture

Document status: coordinated software baseline for the Raspberry Pi, BLE bridge ESP32, and main-controller ESP32.

This document describes the behavior implemented by the patched project files. When source code and this document disagree, treat the mismatch as a release defect: inspect the deployed source, correct the implementation or documentation, and validate all affected nodes together.

## Table of contents

1. [System purpose and safety priority](#1-system-purpose-and-safety-priority)
2. [Three-node control ownership](#2-three-node-control-ownership)
3. [Runtime architecture](#3-runtime-architecture)
4. [Communication interfaces](#4-communication-interfaces)
5. [Raspberry Pi processing](#5-raspberry-pi-processing)
6. [BLE bridge safety behavior](#6-ble-bridge-safety-behavior)
7. [Hallway centering and anti-zigzag control](#7-hallway-centering-and-anti-zigzag-control)
8. [Main-controller motion safety](#8-main-controller-motion-safety)
9. [Servo and local ultrasonic subsystem](#9-servo-and-local-ultrasonic-subsystem)
10. [Motor control and braking](#10-motor-control-and-braking)
11. [Robot state machine](#11-robot-state-machine)
12. [Load, gas, Firebase, GUI, and SMS](#12-load-gas-firebase-gui-and-sms)
13. [Scheduling and optimization rules](#13-scheduling-and-optimization-rules)
14. [Startup and recovery sequences](#14-startup-and-recovery-sequences)
15. [Validation and release requirements](#15-validation-and-release-requirements)
16. [Change-management rule](#16-change-management-rule)

## 1. System purpose and safety priority

GARBY is an autonomous waste-collection robot intended to travel through corridors, monitor load and environmental sensors, avoid people and obstacles, remain centered using 360-degree LiDAR, and report status through Firebase, a local GUI, and cellular SMS.

The primary design priority is preventing uncontrolled motion. Navigation convenience, automatic return, throughput, and smooth centering are secondary to these rules:

1. Missing, stale, malformed, disconnected, or out-of-order path data means STOP.
2. Only path-safety data may release STOP.
3. A previous STOP requires repeated fresh clearance evidence before motion resumes.
4. Steering is disabled while motion permission is absent or stale.
5. Communication loss holds the robot stationary; it does not initiate a blind return.
6. Software safety does not replace a hardwired emergency-stop or measured stopping-distance tests.

## 2. Three-node control ownership

GARBY is distributed across three processing nodes with deliberately separated responsibilities.

### 2.1 Raspberry Pi 4 - sensing and status production

Primary file: `RaspberryPi/final_w_serial.py`

Responsibilities:

- Start and supervise the YDLidar ROS 2 driver.
- Subscribe to `/scan` and convert each scan into eight directional distance sectors.
- Produce sequenced path-safety packets and matching steering-geometry packets.
- Mark the path stale when fresh LiDAR scans stop arriving.
- Read serial sensor telemetry from `/dev/ttyAMA0` at 9600 baud.
- Update Firebase Realtime Database.
- Run the optional Tkinter monitoring GUI.
- Maintain the BLE client and send data through a bounded coalescing mailbox.

The Raspberry Pi does not directly command motor speed or motor direction.

### 2.2 ESP32 BLE bridge - sole navigation parser and safety latch

Primary file: `BLE_Receiver-Final/BLE_Receiver-Final.ino`

Responsibilities:

- Act as the NimBLE server named `GarbyESP32`.
- Strictly dispatch recognized BLE packet types.
- Accept only fresh sequenced path packets.
- Latch STOP and require repeated clear path packets before sending GO.
- Reject stale or sequence-mismatched steering packets.
- Compute hallway-centering nudge direction, duration, and intensity.
- Hold STOP on BLE disconnect, path timeout, protocol error, or waiting-for-data state.
- Forward sensor telemetry and explicit reset commands without allowing them to alter path state.
- Communicate with the main controller over UART at 115200 baud.

The bridge is the only component that interprets LiDAR lane geometry into nudge commands.

### 2.3 Main-controller ESP32 - guarded execution

Primary files:

- `NAPHTALI_CODE_V2/NAPHTALI_CODE_V2.ino`
- `NAPHTALI_CODE_V2/NAPHTALI_CODE_V2.cpp`
- `NAPHTALI_CODE_V2/NAPHTALI_CODE_V2.h`
- `NAPHTALI_CODE_V2/pointsRun.ino`

Responsibilities:

- Start fail-closed and require fresh confirmed GO commands before motion.
- Execute stepper movement and short precomputed nudge commands.
- Independently enforce path freshness.
- Perform controlled deceleration for normal and safety stops.
- Operate the front servo and ultrasonic sensor as a command-settle-ping state machine.
- Read the HX711 load cell.
- Evaluate received gas telemetry for state transitions.
- Queue SMS work to a separate FreeRTOS task so modem delays do not block safety polling.
- Manage the `IDLE`, `RUNNING`, and `RETURNING` states.

The main controller does not parse raw LiDAR geometry or decide hallway-centering direction.

## 3. Runtime architecture

```text
+--------------------------------------------------------------------------------+
| Raspberry Pi 4                                                                 |
|                                                                                |
|  YDLidar ROS 2 /scan --> LidarDistanceReader                                   |
|                              |                                                 |
|                              +--> P:<seq>|F=..|B=..   path safety               |
|                              +--> S:<seq>|...          steering geometry        |
|                                                                                |
|  /dev/ttyAMA0 sensor reader --> SENSOR:US=..|MQ4=..|MQ137=..|MQ135=..          |
|  Firebase + GUI                                                               |
|                                                                                |
|  CoalescingBleQueue --> serialized Bleak GATT writer                           |
+-----------------------------------|--------------------------------------------+
                                    | BLE write / notify
                                    v
+--------------------------------------------------------------------------------+
| ESP32 BLE Bridge - GarbyESP32                                                  |
|                                                                                |
|  Strict packet dispatcher                                                      |
|  Sequenced path parser --> STOP latch --> STOP / GO                            |
|  Sequenced geometry parser --> anti-zigzag controller --> N: command           |
|  Path watchdog 1.2 s | Link watchdog 8 s | disconnect STOP                     |
|  SENSOR passthrough | explicit RESET passthrough after STOP                    |
+-----------------------------------|--------------------------------------------+
                                    | UART 115200
                                    v
+--------------------------------------------------------------------------------+
| Main-controller ESP32                                                          |
|                                                                                |
|  Non-blocking UART parser --> independent path watchdog 1.5 s                  |
|  Motion gate + GO confirmation --> FastAccelStepper                            |
|  Controlled braking --> motor outputs                                          |
|  Servo scan state machine + local ultrasonic stop                              |
|  HX711 load cell | gas-state evaluation | Air780E SMS worker                   |
|  IDLE --> RUNNING --> stationary destination --> explicit RESET --> RETURNING   |
+--------------------------------------------------------------------------------+
```

## 4. Communication interfaces

### 4.1 Raspberry Pi to BLE bridge

BLE service and characteristics:

- Service UUID: `4fafc201-1fb5-459e-8fcc-c5c9c331914b`
- Write characteristic: `beb5483e-36e1-4688-b7f5-ea07361b26a8`
- Notify characteristic: `beb5483e-36e1-4688-b7f5-ea07361b26a9`

#### Path-safety packet

```text
P:<seq>|F=<code>|B=<code>
```

Fields:

- `<seq>`: unsigned 32-bit monotonically increasing sequence number.
- `F`: front path state.
- `B`: back path state.

Codes:

- `C`: clear
- `O`: obstacle
- `H`: human-tagged obstacle
- `S`: stale or unavailable LiDAR/path stream

Path packets are safety/control traffic and use acknowledged GATT writes.

#### Steering-geometry packet

```text
S:<seq>|L=..|R=..|F=..|B=..|FL=..|FR=..|BL=..|BR=..|T=..
```

or:

```text
S:<seq>|STABLE
```

The sequence must exactly match the newest accepted path packet. Geometry from an older or newer unmatched sequence is ignored.

Abbreviations:

- `L`, `R`: left and right side distances
- `F`, `B`: front and back distances
- `FL`, `FR`, `BL`, `BR`: diagonal distances
- `T`: precomputed diagonal heading/tilt estimate

#### Sensor telemetry packet

```text
SENSOR:US=..|MQ4=..|MQ137=..|MQ135=..
```

This is telemetry only. It must never produce GO, clear a STOP latch, update the path timestamp, or enable steering.

Sentinel values:

- Ultrasonic unavailable: `999`
- Gas sensor unavailable: `-1`

#### Control packets

- `[RASPI READY]`: BLE-session readiness indication.
- `[RESET]`: explicit return/reset request. The bridge first relays `STOP:RESET`, then forwards `[RESET]` to the main controller.

### 4.2 BLE bridge to main controller

Safety commands:

- `STOP`
- `STOP:HUMAN`
- `STOP:STALE`
- `STOP:LINK`
- `STOP:WAITING_DATA`
- `STOP:PROTOCOL`
- `STOP:RESET`
- `STOP:CLEARING`
- `GO`

Steering command:

```text
N:<milliseconds>:<intensity>|<NUDGE_LEFT|NUDGE_RIGHT|STABLE>
```

Other messages:

- `SENSOR:...`
- `[RESET]`
- `[BLE CONNECTION ESTABLISHED]`

### 4.3 Main controller to BLE bridge

- `[MCU READY]`: hardware initialization is complete.
- `[REQUEST-STATUS]`: request fresh path, steering, and periodic telemetry.
- `[ESP RECEIVED]`: local UART acknowledgment; the bridge does not forward it over BLE.
- `[OUTBOUND COMPLETE]`: informational notification.

## 5. Raspberry Pi processing

### 5.1 LiDAR sectors

The LiDAR is divided into eight 22-degree cones:

| Sector | Center angle |
|---|---:|
| FRONT | 180 degrees |
| FRONT_LEFT | 225 degrees |
| LEFT | 270 degrees |
| BACK_LEFT | 315 degrees |
| BACK | 0 degrees |
| BACK_RIGHT | 45 degrees |
| RIGHT | 90 degrees |
| FRONT_RIGHT | 135 degrees |

Front group: `FRONT`, `FRONT_LEFT`, `FRONT_RIGHT`.

Back group: `BACK`, `BACK_LEFT`, `BACK_RIGHT`.

### 5.2 Obstacle processing

Configured stop thresholds:

- Front: 95 cm
- Back: 35 cm

Front obstacles at 100 cm or less are accepted immediately instead of waiting for outlier confirmation. This reduces detection latency.

For front, back, and diagonal obstacle safety, the closest valid point in the cone is retained. For left and right hallway-centering distances, the 25th percentile is used instead of the absolute minimum so one leg, bin rim, or wall fixture is less likely to create a false steering correction.

Distance histories use median smoothing with a depth of six and two-frame confirmation for large non-emergency jumps.

### 5.3 LiDAR freshness

`LIDAR_STALE_TIMEOUT_S = 0.8`

When the scan stream is older than 0.8 seconds, the Pi emits:

```text
P:<seq>|F=S|B=S
S:<seq>|STABLE
```

The watchdog actively produces a stale safety state; it does not merely log a warning.

### 5.4 BLE scheduling and queue behavior

The Pi uses a bounded `CoalescingBleQueue` with a maximum of 12 entries.

- A newer path packet replaces an older queued path packet.
- A newer steering packet replaces an older queued steering packet.
- A newer telemetry packet replaces an older queued telemetry packet.
- Urgent safety packets are inserted at the front.
- Control packets are retained subject to the queue bound.

All GATT writes share one asyncio lock. Path, reset, and readiness packets use acknowledged writes. Steering and telemetry may use write without response.

Sensor telemetry is limited to 1 Hz. Firebase sensor updates are batched to reduce blocking network work.

Runtime front/back safety-disable controls are not honored while the robot is actively requesting status unless `ALLOW_RUNTIME_SAFETY_DISABLE` is intentionally changed and separately validated.

## 6. BLE bridge safety behavior

### 6.1 Strict packet dispatch

The bridge dispatches messages by recognized prefix:

1. `SENSOR:` -> telemetry passthrough and return.
2. `[RESET]` -> STOP, then reset passthrough and return.
3. `[RASPI READY]` -> readiness handling and return.
4. `P:` -> sequenced path parser and return.
5. `S:` -> sequenced steering parser and return.
6. Legacy combined `PATH:` packet -> isolated compatibility parser and return.
7. Unknown packet -> log and ignore.

No unrelated packet may fall through to the path-clear branch.

### 6.2 Sequence handling

A path packet is accepted only if its sequence is newer than the previous accepted sequence. Comparison uses signed subtraction so unsigned wraparound remains usable.

A steering packet is accepted only when:

- It is newer than the previously accepted steering packet.
- A path packet has already been accepted.
- Its sequence exactly equals the newest path sequence.

This prevents delayed steering from acting after a newer STOP.

### 6.3 STOP latch and clearance

The bridge starts with `stopLatched = true`.

Any obstacle, human, stale code, protocol error, path timeout, link timeout, disconnect, reset request, or waiting-for-data condition latches STOP and emits a stable nudge.

A clear path does not immediately produce GO. The bridge requires three fresh clear path packets. Until then it emits `STOP:CLEARING`.

### 6.4 Watchdogs

| Watchdog | Value | Behavior |
|---|---:|---|
| Path-data timeout | 1200 ms | Emit repeated `STOP:STALE` or `STOP:WAITING_DATA`. |
| Stale STOP repeat | 500 ms | Refresh STOP while stale. |
| General link timeout | 8000 ms | Emit `STOP:LINK` and reconnect BLE. |

The 8-second timeout is a connection-recovery timeout, not the motion-safety timeout.

### 6.5 Disconnect behavior

On BLE disconnect, the bridge:

1. Resets navigation state.
2. Sends `STOP:LINK` to the main controller.
3. Sends `N:0:0|STABLE`.
4. Restarts advertising.

It does not send `[RESET]` and does not command an automatic return.

## 7. Hallway centering and anti-zigzag control

The bridge computes the nudge command. The main controller only executes it.

### 7.1 Error model

Lateral centering uses left/right wall distance. Heading uses diagonal geometry and the optional `T` field. The controller applies:

- Lateral weight: 0.75
- Heading weight: 0.25
- EMA alpha: 0.20
- Clamped heading contribution
- Corridor-width baseline
- Wall-protrusion attenuation
- Corridor-opening attenuation

The design reduces false corrections from open doors, pillars, wall equipment, bin edges, and passers at the side.

### 7.2 Nudge eligibility and tuning

| Parameter | Value |
|---|---:|
| Dead zone | 12 cm |
| Direction-reversal hysteresis | 8 cm |
| Direction confirmation | 4 packets |
| Cooldown | 800 ms |
| Duration | 35 to 90 ms |
| Requested speed cut | 8 to 28 percent |
| Startup grace | 8 packets |
| Startup ramp | 8 packets |
| Front full suppression | 60 cm or less |
| Front warning scaling | 60 to 120 cm |

A nudge is also suppressed if it would steer toward a side wall that is already too close.

### 7.3 Main-controller execution limits

- Left and right default factors are symmetric at 0.82.
- Maximum execution-side cut is 30 percent.
- Nudge acceleration is 12000 steps/s^2.
- Maximum same-direction hold is 250 ms.
- Post-nudge settle interval is 300 ms.

The controller rejects all nudge commands while STOP is active or path data is stale.

LiDAR is the only steering authority by default. `ENABLE_ULTRASONIC_SIDE_NUDGE` is set to `0`.

## 8. Main-controller motion safety

### 8.1 Fail-closed startup and motion gate

The controller initializes with:

- `shouldStop = true`
- no accepted path command
- link-fault state active

Before every safe movement or turn, `movementGate()` requests status and waits briefly for fresh confirmed clearance. If clearance is not established, it enters the halt state.

### 8.2 Independent path watchdog

`PATH_COMMAND_TIMEOUT_MS = 1500`

Outside IDLE, the controller latches STOP when no fresh STOP/GO path command has been received within 1.5 seconds. It clears pending nudge state.

### 8.3 Controller GO confirmation

`MCU_GO_CONFIRM_PACKETS = 2`

The bridge already requires three clear path packets. The controller then requires two GO messages before setting `shouldStop = false`.

A STOP immediately resets the controller clear counter and nudge state.

### 8.4 Halt and resume

`haltAndWait()` keeps requesting status and polling UART.

For LiDAR-only path stops, the routine requires three fresh path-clearance evidence updates.

For human or local-sonar stops, it requires:

- fresh LiDAR clearance;
- local distance greater than `OBSTACLE_DISTANCE + 5 cm`;
- three confirmed clearance samples.

An explicit reset exits the halt loop into the state-machine reset/return path.

## 9. Servo and local ultrasonic subsystem

### 9.1 Mechanical limits and scan pattern

- Right endpoint: 30 degrees
- Center: 80 degrees
- Left endpoint: 130 degrees
- Center cone: plus or minus 18 degrees

Scan pattern:

```text
center -> right -> center -> left -> repeat
```

The endpoints avoid commanding common hobby servos directly into mechanical end stops.

### 9.2 Command-settle-ping timing

Every sample follows:

1. Command a bounded angle.
2. Calculate settling time from angular travel.
3. Wait until the tracked settle deadline.
4. Trigger the ultrasonic pulse.
5. Process the reading.
6. Advance to the next scan state.

Constants:

- Minimum servo settling time: 25 ms
- Settling estimate: 2 ms per degree
- Ping interval: 60 ms
- Echo timeout: 12000 microseconds

### 9.3 Local stop logic

- Immediate STOP at 15 cm or less in the center cone.
- Two center samples at 50 cm or less are required for a normal local obstacle STOP.
- Three clear samples are required before local-obstacle release.
- Side samples do not steer while `ENABLE_ULTRASONIC_SIDE_NUDGE = 0`.

The previous multi-second nested left/front/right rescan loop and automatic reverse retreat are not part of this baseline.

### 9.4 Electrical requirement

Use a dedicated regulated 5 V servo supply sized for stall current, connect grounds, and place approximately 470 to 1000 microfarads of bulk capacitance near the servo connector. Software timing cannot correct supply collapse or ground noise.

## 10. Motor control and braking

Configured drive values:

| Parameter | Value |
|---|---:|
| Maximum step rate | 8250 steps/s |
| Normal acceleration | 8250 steps/s^2 |
| Turn speed | 4500 steps/s |
| Gentle-stop deceleration | 10000 steps/s^2 |
| Safety-stop deceleration | 16000 steps/s^2 |
| Wheel steps per revolution | 1600 |
| Wheel diameter | 65 mm |
| Wheelbase | 150 mm |

Stopping modes:

1. Normal route endpoint: allow target-position motion to decelerate normally.
2. Gentle explicit stop: controlled deceleration with an 1800 ms completion timeout.
3. Safety obstacle stop: controlled deceleration with an 800 ms completion timeout.
4. Hard stop fallback: use `forceStopAndNewPosition()` only when controlled stopping does not finish before its timeout.

No counter-torque reverse pulse is used.

At the configured maximum step rate and wheel geometry, theoretical speed is approximately 1.05 m/s. A 16000 steps/s^2 deceleration gives a theoretical motor-ramp stopping phase near 0.52 seconds and 0.27 m. These figures exclude sensing latency, communication latency, wheel slip, motor torque limits, battery condition, payload shift, and mechanical compliance. Hardware acceptance must use measured worst-case total distance.

## 11. Robot state machine

### 11.1 IDLE

- Keep motors stopped and fail-closed.
- Poll BLE/UART continuously.
- Request status approximately every 500 ms to keep telemetry and link state current.
- Read and confirm load-cell threshold.
- Confirm gas alerts before transition.
- Transition to RUNNING after the configured load or gas condition is confirmed.

### 11.2 RUNNING

- Execute `runStart()` once.
- `runStart()` calls `safeMoveDistance()` and therefore requires fresh motion permission.
- After outbound completion, stop gently and remain stationary.
- Continue requesting status.
- Do not execute `runStart()` repeatedly.
- Transition to RETURNING only after an explicit `[RESET]` request.

### 11.3 RETURNING

- Execute `returnToPointB()` through the same motion gate and safety checks.
- On completion, call `fullReset()` and return to IDLE fail-closed.

### 11.4 Route limitation

The current `pointsRun.ino` uses a positive long movement for both outbound and return functions:

- `runStart()`: 595500 steps
- `returnToPointB()`: 595700 steps

The physical meaning cannot be proven from source alone. The chassis may turn before return, follow another route, or require reversed motor direction. Verify both functions with wheels lifted before any floor test and update this document after the physical route is confirmed.

## 12. Load, gas, Firebase, GUI, and SMS

### 12.1 Load and gas trigger

The controller evaluates the HX711 load cell locally. It receives gas values through `SENSOR:` telemetry and evaluates MQ4, MQ135, and MQ137 status thresholds.

Load and gas conditions require repeated confirmation before transitioning from IDLE to RUNNING.

### 12.2 Firebase and GUI

The Pi publishes sensor and state information to Firebase and optionally displays the eight LiDAR sectors and sensor telemetry in Tkinter. Neither Firebase nor the GUI is a motion authority.

### 12.3 SMS isolation

Air780E operations can take seconds. Operational alerts are queued to a separate FreeRTOS worker. The main loop avoids simultaneous modem passthrough while the worker owns the modem.

SMS failure must not block STOP processing or prevent the robot from remaining stationary.

## 13. Scheduling and optimization rules

- Keep LiDAR scan processing single-pass.
- Use bounded queues for latest-value traffic.
- Coalesce path, steering, and telemetry independently.
- Send path safety before steering and telemetry.
- Serialize GATT writes with one lock.
- Keep sensor telemetry at 1 Hz unless a measured requirement justifies more.
- Keep GUI refresh near 5 Hz rather than tying it to the LiDAR callback.
- Avoid explicit garbage collection in the scan callback.
- Keep UART readers non-blocking and buffers bounded.
- Suppress high-rate request and ACK logs in normal operation.
- Keep modem and Firebase waits outside motion-safety loops.
- Use overflow-safe `millis()` subtraction for timeouts.

## 14. Startup and recovery sequences

### 14.1 Normal startup

1. Main controller initializes motors, servo, local sensors, load cell, and modem.
2. Main controller sends `[MCU READY]` repeatedly.
3. Bridge starts BLE advertising and waits for the Pi.
4. Pi connects and sends `[RASPI READY]`.
5. Bridge informs the controller with `[BLE CONNECTION ESTABLISHED]` and `STOP:WAITING_DATA`.
6. Controller remains stopped and requests status.
7. Pi emits fresh path and steering packets.
8. Bridge requires three clear path packets.
9. Controller requires two GO messages.
10. Motion becomes eligible only after both layers confirm clearance.

### 14.2 LiDAR stale

1. Pi detects scan age greater than 0.8 seconds and emits stale path codes.
2. Bridge emits `STOP:STALE` and stable steering.
3. Controller latches STOP and brakes.
4. If Pi stale packets do not arrive, bridge path timeout stops within 1.2 seconds.
5. If bridge commands do not arrive, controller path timeout stops within 1.5 seconds.

### 14.3 BLE disconnect

1. Bridge disconnect callback emits `STOP:LINK`.
2. Controller latches STOP and rejects nudges.
3. Bridge restarts advertising.
4. Reconnection begins in `STOP:WAITING_DATA`.
5. Fresh path and repeated clear confirmations are required again.

### 14.4 Explicit reset/return

1. Pi sends `[RESET]` intentionally.
2. Bridge emits `STOP:RESET` before forwarding reset.
3. Controller performs a controlled stop and queues the explicit state transition.
4. RETURNING uses the normal movement gate and live perception.

## 15. Validation and release requirements

### 15.1 Software validation

- Run `python3 -m py_compile` for `final_w_serial.py`.
- Compile the BLE bridge with the exact installed NimBLE-Arduino and ESP32 board core.
- Compile the main controller with the exact FastAccelStepper, ESP32Servo, HX711, and ESP32 board core.
- Test telemetry while STOP is active; STOP must remain active.
- Test stale and out-of-order path and steering packets.
- Test three bridge clear packets and two controller GO confirmations.
- Test nudge rejection while stopped and stale.
- Test queue coalescing with an older clear packet followed by a newer STOP.
- Test BLE disconnect, LiDAR loss, malformed packet, and reset behavior.

### 15.2 Hardware validation

Start with wheels lifted, then reduced-speed empty floor tests, then maximum-payload tests.

Required checks:

- Hardwired latching emergency-stop removes motor enable or motor power.
- Motor drivers default disabled during reset or brownout.
- Servo power and endpoints do not brown out or bind.
- Ultrasonic readings are checked at known distances.
- STOP and resume confirmation are verified with wheels lifted.
- Braking distance is measured across speed, payload, battery, and floor conditions.
- Hallway centering is tested through open doors, pillars, wall fixtures, people at the side, narrow sections, and wide sections.
- Return direction is verified independently.

Do not claim human-safe operation from source review, host syntax tests, or theoretical braking calculations alone.

## 16. Change-management rule

Any material runtime change must update this file in the same release. This includes:

- packet formats or baud rates;
- node ownership;
- watchdogs, STOP/GO, disconnect, reset, and resume behavior;
- LiDAR filtering and centering parameters;
- servo or ultrasonic timing and thresholds;
- motor speed, braking, or route behavior;
- state transitions;
- deployment or validation requirements.

Deploy coordinated Pi, bridge, and controller revisions together whenever their protocol or safety assumptions change.
