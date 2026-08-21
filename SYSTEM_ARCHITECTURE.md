# GARBY Autonomous Waste-Collection Robot — Coordinated System Architecture

**Release baseline:** 2026-08-21 coordinated communication/stability repair  
**Status:** source-coordinated and host-tested; **hardware acceptance still required**

This file is the authoritative description of the deployable GARBY runtime. If a comment, old thesis note, editor file, or assistant instruction disagrees with the source files listed below, treat that disagreement as a defect and verify the source before deployment.

## 1. Active deployment set

| Node | Active files | Ownership |
| --- | --- | --- |
| Raspberry Pi 4 | `RasPi/final_w_serial.py`, `RasPi/bridge_core.py` | LiDAR acquisition, path-state production, steering geometry, BLE client, sensor UART, optional Firebase/GUI |
| BLE bridge ESP32 | `BLE_Receiver-Final/BLE_Receiver-Final.ino` | BLE server, strict protocol dispatch, path STOP latch, sequence validation, hallway-centering decision, UART forwarding |
| Main/controller ESP32 | `NAPHTALI_CODE_V2/NAPHTALI_CODE_V2.ino`, `.cpp`, `.h`, `pointsRun.ino` | guarded motor execution, local front sonar, load cell, gas-state evaluation, asynchronous modem/SMS, route state machine |

`RasPi/final_w_serial-simulator.py` is a simulator/test producer, not the production Pi entry point.

## 2. Safety ownership and invariants

1. Unknown, missing, stale, malformed, disconnected, or out-of-order **path** information never authorizes movement.
2. Only a validated `P:` path packet can ultimately lead to `GO`.
3. `SENSOR:` telemetry, Firebase, GUI state, notifications, and steering geometry cannot clear STOP.
4. STOP is latched at the BLE bridge and motion starts fail-closed at the main controller.
5. The bridge requires repeated clear path evidence; the controller independently requires repeated `GO` evidence.
6. Steering geometry is sequence-bound to the newest accepted path packet and is ignored while STOP is latched.
7. BLE/LiDAR loss keeps GARBY stationary. Link loss never starts an automatic return.
8. LiDAR is the hallway-centering authority. Local servo ultrasonic is near-field stop/slowdown protection; side-sonar steering is disabled.
9. Cellular/Firebase/GUI work is operational telemetry and may fail without blocking the motion-safety communication path.
10. Software does not replace a hardwired emergency stop, power integrity, or measured stopping-distance validation.

## 3. Runtime data flow

```text
YDLidar /scan
   |
   v
Raspberry Pi final_w_serial.py
   |  P:<seq>|F=...|B=...      safety, acknowledged BLE write
   |  S:<seq>|...              steering geometry, latest-value
   |  SENSOR:...               telemetry only
   v
ESP32 BLE bridge
   |  strict dispatch + seq checks + STOP latch
   |  STOP... / GO             safety permission
   |  N:<ms>:<pct>|<dir>       precomputed steering tap
   |  SENSOR:... / [RESET]
   v  UART 115200
Main ESP32
   |  independent freshness gate + local sonar
   v
FastAccelStepper motor execution
```

The sensor-board UART into the Pi remains **9600 baud** by default. The bridge-to-main-controller UART is **115200 baud**. These are different interfaces and must not be confused.

## 4. Raspberry Pi behavior

### 4.1 Startup and dependencies

The production Pi source now includes `RasPi/bridge_core.py`. The previous package contained only version-specific `.pyc` cache files for that module, which could make `final_w_serial.py` fail at import before BLE or LiDAR started.

Critical runtime dependencies are ROS 2/rclpy + `sensor_msgs`, YDLidar ROS 2 driver, `pyserial`, and `bleak`. `firebase-admin` is optional: if it is missing or Firebase is unavailable, safety/BLE/LiDAR operation continues.

Normal headless start:

```bash
python3 final_w_serial.py --headless
```

### 4.2 LiDAR coordinate convention and mount correction

Software convention:

| Direction | Angle |
| --- | ---: |
| FRONT | 0° |
| FRONT_LEFT | 45° |
| LEFT | 90° |
| BACK_LEFT | 135° |
| BACK | 180° |
| BACK_RIGHT | 225° |
| RIGHT | 270° |
| FRONT_RIGHT | 315° |

Physical mounting is intentionally **hardware-unverified**. Configure `GARBY_LIDAR_YAW_OFFSET_DEG` rather than editing sector code. Default is `0`. A value of `180` recreates the opposite-facing convention found in old documentation.

### 4.3 Scan processing

- One `/scan` ROS subscription uses `qos_profile_sensor_data`; duplicate subscriptions were removed.
- Safety partitions the full circle into gap-free nearest 45° sectors.
- Steering uses centered 22° slices and median wall geometry.
- Positive `+inf` LaserScan samples mean “no return within sensor range” and are represented at `range_max`; NaN/negative infinity remain invalid.
- Collision safety uses the **closest valid return** in each safety sector.
- Current front/back scan quality must contain at least three samples in every required front/back safety sector. Fresh-but-incomplete safety data becomes `S`, not `C`.
- Before the **first valid scan**, path state is stale/unknown. There is no safety startup grace that can report CLEAR blindly.

### 4.4 Pi safety timing

| Setting | Value |
| --- | ---: |
| Front LiDAR stop threshold | 95 cm |
| Back LiDAR stop threshold | 35 cm |
| LiDAR stale timeout | 0.8 s |
| Normal path/steering period | 0.25 s |
| Sensor telemetry period | 1.0 s |
| BLE queue bound | 12 latest/control entries |

The driver supervisor may have startup/restart grace for process management, but the **path safety output does not**.

### 4.5 Pi BLE transport

Service/characteristics:

- Service: `4fafc201-1fb5-459e-8fcc-c5c9c331914b`
- Write: `beb5483e-36e1-4688-b7f5-ea07361b26a8`
- Notify: `beb5483e-36e1-4688-b7f5-ea07361b26a9`
- Server name: `GarbyESP32`

All GATT writes use one asyncio lock. Path/control messages use acknowledged writes. The queue coalesces old path, steering, and telemetry values so old CLEAR state cannot build up behind a new STOP. An unexpected connection-task exception is caught and rescheduled; a transient failed path write requests a **fresh sequence** instead of replaying an old path packet.

## 5. Pi → bridge protocol

### 5.1 Path packet

```text
P:<uint32-seq>|F=<C|O|H|S>|B=<C|O|H|S>
```

- `C` clear
- `O` obstacle
- `H` legacy human-tagged obstacle; motion behavior is still STOP
- `S` stale/unavailable/unknown

The bridge parses the body as exact `F=` and `B=` tokens, rejects duplicates/unknown fields, and rejects decimal sequence overflow.

### 5.2 Steering packet

```text
S:<same-seq>|L=..|R=..|F=..|B=..|FL=..|FR=..|BL=..|BR=..|T=..
```

or

```text
S:<same-seq>|STABLE
```

A steering packet is usable only when its sequence exactly equals the newest accepted `P:` sequence.

### 5.3 Telemetry packet

```text
SENSOR:US=..|MQ4=..|MQ137=..|MQ135=..
```

Sentinels:

- `US=999`: unavailable trash-level ultrasonic telemetry
- gas sensor `-1`: unavailable

These sentinel values are represented as **UNAVAILABLE** in the main-controller telemetry model; they are not interpreted as “full” or “danger”.

### 5.4 Control tokens

- `[RASPI READY]`: Pi BLE session readiness/liveness token
- `[RESET]`: explicit request to stop and transition to the deliberate return/reset workflow

## 6. BLE bridge behavior

### 6.1 Fail-closed startup

`mcuReady` begins `false`; the bridge will not treat the controller as executable until it receives `[MCU READY]`. Every `[MCU READY]` is treated as a new controller boot epoch: queued ingress/navigation state is reset, the controller is told `[BLE CONNECTION ESTABLISHED]`, and `STOP:WAITING_DATA` is asserted until fresh path evidence arrives.

### 6.2 Bridge watchdogs and queues

| Setting | Value |
| --- | ---: |
| Valid path timeout | 850 ms |
| General BLE silence/reconnect timeout | 10 s |
| Repeated stale STOP | 500 ms |
| Path frame max age at ingress | 650 ms |
| Steering frame max age | 500 ms |
| Sensor frame max age | 2000 ms |
| Clear packets needed at bridge | 2 |
| MCU ACK timeout | 3000 ms |
| Max tracked unacked MCU commands | 8 |

BLE callbacks only copy bounded frames. Parsing, float math, logging, and UART writes run from `loop()`. Safety/control frames are processed before steering and telemetry. Steering and telemetry use latest-value mailboxes.

### 6.3 STOP release sequence

From a latched STOP:

1. clear path packet #1 → `STOP:CLEARING`
2. clear path packet #2 → bridge releases latch and emits `GO` #1
3. next fresh clear path packet → `GO` #2; main controller can then release its own latch

Thus a normal fully-latched recovery requires at least **three fresh path packets** across both layers. The main movement gate now waits 900 ms and repeatedly requests status (request throttled to 80 ms), so this confirmation sequence can complete without the old 400 ms false timeout.

### 6.4 BLE disconnect

Disconnect resets navigation state, sends `STOP:LINK` when the MCU is ready, clears pending UART ACK accounting, and resumes advertising. It does **not** issue `[RESET]` and does not initiate a return route.

## 7. Hallway centering

The bridge alone converts LiDAR wall geometry into a nudge command.

| Parameter | Value |
| --- | ---: |
| Lateral weight | 0.85 |
| Heading weight | 0.15 |
| EMA alpha | 0.15 |
| Dead zone | 15 cm |
| Reversal hysteresis | 10 cm |
| Direction confirmation | 5 packets |
| Nudge cooldown | 1100 ms |
| Nudge duration | 35–75 ms |
| Requested cut | 8–22% |
| Startup steering grace | 8 packets |
| Startup ramp | next 8 packets |
| Front full suppression | ≤60 cm |
| Front warning scaling | 60–120 cm |

The main controller applies stricter execution caps: maximum 24% cut, maximum 110 ms same-direction hold, and a 220 ms settle window. `ENABLE_ULTRASONIC_SIDE_NUDGE` remains `0`. When a valid, bridge-confirmed direction reversal arrives, the controller restores both wheel speeds and applies the new tap immediately; it no longer drops the correction and waits for another packet.

## 8. Main-controller communication and motion gate

- `shouldStop = true` at boot.
- `PATH_COMMAND_TIMEOUT_MS = 800` independently stops movement if the bridge stops refreshing STOP/GO state.
- `MCU_GO_CONFIRM_PACKETS = 2`.
- `MOTION_GATE_TIMEOUT_MS = 900`.
- Motion gate repeatedly sends `[REQUEST-STATUS]`; requests are rate-limited to 80 ms.
- Turns and distance moves re-check `pathCommandFresh()` before starting and continue polling UART/watchdogs while moving.
- Motion primitives return a completion result. A missing/unaccepted motor command or aborted segment cannot be reported as `[OUTBOUND COMPLETE]`; the state remains active until the route really completes or an explicit reset is received.
- Malformed `N:` numeric fields are rejected with strict digit/overflow checks rather than permissive `String.toInt()` parsing.
- Main UART line input is bounded and discards oversize lines until newline.

## 9. Local servo ultrasonic

Servo geometry:

- right: **40°**
- center: **80°**
- left: **120°**
- center cone: ±18°

Current automatic safety sampling remains centered; side steering is disabled.

| Setting | Value |
| --- | ---: |
| Echo timeout | 8000 µs |
| Ping period | 40 ms |
| Slowdown | ≤90 cm |
| Normal STOP | 2 valid samples at ≤60 cm |
| Immediate emergency latch | ≤25 cm |
| Clear threshold | ≥72 cm |
| Clear confirmation | 3 valid samples |
| Full-speed resume zone | ≥105 cm |
| Fresh sample age | 180 ms |

A timeout/no echo is **UNKNOWN**, not CLEAR. It cannot increment the clear counter or release a prior local obstacle latch. If the robot is halted because of local sonar, resume still needs valid local clear evidence plus fresh LiDAR permission.

## 10. Motor and braking configuration

| Setting | Value |
| --- | ---: |
| Normal max step rate | 5600 steps/s |
| Caution step rate | 3900 steps/s |
| Turn rate | 3000 steps/s |
| Normal acceleration | 4800 steps/s² |
| Gentle-stop deceleration | 6500 steps/s² |
| Safety-stop deceleration | 14000 steps/s² |
| Wheel steps/rev | 1600 |
| Wheel diameter | 65 mm |
| Wheelbase | 150 mm |

Safety stopping requests controlled deceleration first; force stop is a timeout fallback. No deliberate reverse-torque braking pulse is used.

These numbers are **not proof of stopping distance**. Floor traction, load, battery voltage, driver current, wheel slip, and mechanical compliance must be measured on the real chassis.

## 11. Modem, load, gas, Firebase, and GUI

The Air780E modem no longer gates the MCU communication handshake. `[MCU READY]` is emitted first, then modem initialization runs in a FreeRTOS background task. SMS requests are ignored while the modem is unavailable/busy; modem delays cannot block STOP processing.

Load and gas are IDLE→RUNNING triggers, with repeated confirmation. The load trigger is `LOAD_TRIGGER_KG = 1.0 kg` and requires two filtered samples; it starts the route state machine but does not bypass the independent fresh-path motion gate. Gas telemetry becomes non-triggering when stale/unavailable. Firebase is asynchronous/coalesced on the Pi and is not a motion authority. GUI controls cannot override unavailable LiDAR data while active-operation safety is enforced.

## 12. State machine

### IDLE

- fail-closed motor state
- request status periodically
- collect load/gas evidence
- start outbound workflow only after trigger confirmation

### RUNNING

- execute `runStart()` once
- only a successful `runStart()` completion is reported as outbound complete; interrupted/incomplete segments remain in RUNNING
- every movement passes the fresh-path gate
- after route completion, remain stationary
- transition to RETURNING only on explicit `[RESET]`

### RETURNING

- execute `returnToPointB()` using the same live safety gates
- finish with `fullReset()` to IDLE only after the return route reports success

## 13. Route limitation — physical direction is hardware-unverified

`pointsRun.ino` contains turns plus long positive step moves for both outbound and return portions. Source alone cannot prove which real-world direction the chassis travels after each turn. **Do not change signs based on function names or old documentation.** Verify the full sequence with wheels lifted and record the observed direction before floor tests.

## 14. Required deployment order

1. Compile/flash the main-controller sketch (`NAPHTALI_CODE_V2`).
2. Compile/flash the BLE bridge (`BLE_Receiver-Final`).
3. Deploy the Pi `RasPi/` source, including `bridge_core.py`.
4. Start the Pi bridge in headless mode.
5. With wheels lifted, verify `[MCU READY]` → `[BLE CONNECTION ESTABLISHED]` → `STOP:WAITING_DATA` → repeated clear evidence → GO sequence.
6. Only then perform reduced-speed supervised physical tests described in `DEPLOYMENT_AND_ACCEPTANCE.md`.

## 15. Release rule

Any change to protocol strings, UUIDs, baud rates, watchdogs, STOP/GO logic, LiDAR orientation/filtering, nudge tuning, sonar thresholds, braking, state transitions, or route behavior must update this file and be revalidated as a coordinated three-node release.
