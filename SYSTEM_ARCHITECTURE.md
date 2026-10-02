# GARBY system architecture

Release: 2026-10-02. Software builds and behavioral tests are documented in
`VALIDATION_RESULTS.md`; physical acceptance remains hardware-unverified.
This file describes the active source. Older audit reports are historical.

## Components and ownership

| Component | Active source | Responsibility |
| --- | --- | --- |
| Raspberry Pi | `RasPi/final_w_serial.py`, `bridge_core.py` | ROS 2 LiDAR safety, wall geometry, BLE central, sensor serial input, asynchronous Firebase telemetry and correlated return relay |
| BLE ESP32 | `BLE_Receiver-Final/BLE_Receiver-Final.ino` | BLE GATT server, path validation and STOP latch, centering decisions, controller UART transport |
| Main ESP32 | `NAPHTALI_CODE_V2.ino/.cpp/.h`, `pointsRun.ino` | Motor targets, local sonar protection, load/gas triggers, route state, modem/SMS worker |
| Android | `Garby_MobileApp/app/` | Authenticated Firebase dashboard, freshness display, explicit return request and matching completion |

The Android-directory Python file is a compatibility launcher into `RasPi/`;
it no longer contains another implementation. The Pi simulator is a manual
test producer, not production firmware. `Test/Test.ino` is a serial-only sketch.

```text
LaserScan -> Pi path + steering -> BLE bridge -> UART -> main ESP32 -> motors
Sensor board -> Pi telemetry -> bridge -> main ESP32 gas trigger
Main ESP32 load/state -> bridge notifications -> Pi -> Firebase -> Android
Android return intent -> Firebase -> Pi -> bridge -> main ESP32
Main ESP32 matching outcome -> bridge -> Pi -> Firebase -> Android
```

Motion remains gated independently of the app, Firebase, GUI, and SMS.
FastAccelStepper executes commanded steps without wheel encoders: it is open
loop motor control. LiDAR provides the outer centering correction; motor pulse
counts alone do not prove physical distance or heading.

## Intended complete cycle

1. Boot stationary. The main MCU initializes peripherals and sends `[MCU READY]`
   before the background modem starts. The bridge starts fail closed.
2. The Pi obtains usable scans and sends fresh path packets. Two clear packets
   release the bridge latch; the MCU needs two GO commands. A fully latched
   recovery therefore needs at least three Pi packets across both layers.
3. While IDLE, two filtered load samples at or above 1.0 kg or three dangerous
   gas telemetry packets trigger outbound execution. Every segment needs live
   path permission; triggering the route never bypasses that gate.
4. `runStart()` executes the calibrated route. An obstacle or link loss brakes
   the motors and enters a monitoring wait. Local sonar and LiDAR continue to
   be serviced. Fresh clearance resumes the **original absolute wheel targets**,
   rather than replaying the full segment or restarting the whole route.
5. Only completed targets produce `[OUTBOUND COMPLETE]`. The robot remains
   stationary at the destination, and the Pi publishes readiness to the app.
6. An explicit operator return request with a unique ID starts `returnToPointB()`.
   The return uses the same motion gates and obstacle recovery.
7. Only a completed return calls `fullReset()` and reports a matching DONE.
   A failed segment latches a stationary route fault and reports FAILED.

An ordinary obstacle wait is recoverable. A rejected motor command or incomplete
route is a fault requiring supervised physical recovery. These conditions must
not be treated as equivalent.

## Safety and freshness

- Startup, absent scans, invalid geometry, missing sectors, stale path packets,
  sequence violations, communication loss, and uncertain ingress produce STOP.
- Only a validated path packet can produce GO. Telemetry cannot authorize motion.
- Steering must match the latest accepted path sequence and is suppressed in STOP.
- Freshness limits are fixed; CPU throttling does not extend permission to drive
  on older data. BLE retries can be patient while the robot stays stopped.
- A STOP UART line ends the current receive batch so subsequent queued GO lines
  cannot erase it before the moving loop brakes.
- A GO arriving after an expired gap starts a new confirmation streak.
- BLE/LiDAR loss never starts an automatic return.
- GUI front/back bypasses remain disabled in production, including while IDLE.
- A new return is accepted only after a completed outbound route. Return from
  an interrupted outbound or fault is rejected, rather than driving from an
  unknown physical position.

| Parameter | Value |
| --- | ---: |
| Pi scan stale limit | 0.8 s |
| Periodic path/steering production | 0.25 s; requests may refresh sooner |
| Pi BLE mailbox | 12 entries; latest path/steering/telemetry; monotonic path replacement |
| Bridge path watchdog | 850 ms |
| Bridge ingress path age | 650 ms |
| Bridge steering age | 500 ms |
| Bridge sensor age | 2000 ms |
| General BLE reconnect watchdog | 10 s; never a motion freshness limit |
| Repeated stale STOP | 500 ms |
| Main path watchdog | 800 ms |
| Movement gate | 900 ms, then continuously monitored clearance wait |
| Status request throttle | 80 ms |
| Bridge control queue | 8 frames; overflow clears uncertain backlog and asserts STOP |
| MCU ACK timeout / backlog cap | 3000 ms / 8 ordinary commands |

Individual watchdogs are not a proof of total stopping time. Sensor age,
transport delay, receive scheduling, braking, wheel slip, and load must be
included in physical measurements.

## LiDAR

Software angles are FRONT | 0°, FRONT_LEFT 45°, LEFT 90°, BACK_LEFT 135°,
BACK 180°, BACK_RIGHT 225°, RIGHT 270°, FRONT_RIGHT 315°.
`GARBY_LIDAR_YAW_OFFSET_DEG` corrects the physical mount; default 0 is
hardware-unverified. Verify it before floor operation.

Collision processing covers the full circle with nearest 45° sectors, using the
closest valid return. Steering uses median estimates in centered 22° slices,
falling back to the wider sector when the slice is sparse. Positive infinity
means no return within `range_max`; NaN/negative infinity are invalid.
At least three samples must exist in each front, rear, and side sector.
An invalid side sector blocks both directions. Invalid angular/range metadata
cannot crash the ROS callback or become CLEAR.

| Envelope | Threshold |
| --- | ---: |
| Front sectors | 95 cm |
| Rear sectors | 35 cm |
| Side hard envelope | 22 cm, blocks both travel directions |

One sensor-data subscription with depth 1 avoids a backlog of old scans.
Scan history and path snapshots share a lock. Periodic and urgent path messages
share one mailbox; an older clear packet cannot overwrite a newer STOP, including
across uint32 sequence wrap. No unbounded urgent-write futures are spawned.

## BLE and UART protocol

BLE name: `GarbyESP32`.
Service: `4fafc201-1fb5-459e-8fcc-c5c9c331914b`.
Write: `beb5483e-36e1-4688-b7f5-ea07361b26a8`.
Notify: `beb5483e-36e1-4688-b7f5-ea07361b26a9`.
Requested connection interval: 30–50 ms, supervision timeout 4 s.

```text
Pi -> bridge:
P:<uint32-seq>|F=<C|O|H|S>|B=<C|O|H|S>
S:<same-seq>|L=..|R=..|F=..|B=..|FL=..|FR=..|BL=..|BR=..|T=..
S:<same-seq>|STABLE
SENSOR:US=..|MQ4=..|MQ137=..|MQ135=..
[RASPI READY]
[RESET:<requestedAt-ms>]

Bridge -> main, newline-delimited UART at 115200 baud:
STOP[:reason]
GO
N:<duration-ms>:<percent-cut>|<NUDGE_LEFT|NUDGE_RIGHT|STABLE>
SENSOR:...
[BLE CONNECTION ESTABLISHED]
[ROBOT STATUS]
[RESET:<requestedAt-ms>]

Main -> Pi through bridge notifications:
[MCU READY], [RUNNING], [OUTBOUND COMPLETE], [RETURNING], [ROUTE FAULT], [IDLE]
[RESET ACK:<id>], [RESET DONE:<id>], [RESET FAILED:<id>]
LOAD_CELL:<kg>
[REQUEST-STATUS]
```

`C` = clear, `O` = obstacle, `H` = legacy human-tagged obstacle, `S` = stale/unknown.
Strict path fields reject duplicates, unknown fields, uint32 overflow, and
trailing delimiters. Old or duplicate path sequences assert `STOP:SEQUENCE`.
Legacy combined `PATH:` input remains isolated for old bench tools; production
and the sequenced simulator use `P:`/`S:` only.

Each GATT payload is a single printable line, maximum 244 bytes. Embedded NUL,
newline, or control bytes are rejected, so SENSOR cannot inject UART commands.
All Pi writes share one asyncio lock; path, readiness, and correlated return
writes are acknowledged. Steering/telemetry exceeding the characteristic's
negotiated write-without-response limit also use acknowledged writes; this
handles a 20-byte limit during MTU negotiation or on older BlueZ. Failed safety
writes request a fresh snapshot.
Bridge BLE callbacks copy/post bounded data only. Parsing, UART, notifications,
navigation resets, and connection-parameter work run in `loop()`.

Sensor-board UART into the Pi is **9600 baud**, separate from the 115200 bridge
UART. Its firmware is not provided here. `US=999` and gas `-1` mean UNAVAILABLE,
not full/danger/clearance. Trash-level ultrasonic telemetry is not path sonar.

## Centering and nudging

| Parameter | Value |
| --- | ---: |
| Lateral / heading weights | 0.85 / 0.15 |
| EMA | 0.15 |
| Dead zone / reversal hysteresis | 12 / 8 cm |
| Direction confirmation | 3 packets; strong-signal fast path retained |
| Cooldown | 1000 ms |
| Requested tap | 180–260 ms, 20–34% cut |
| MCU cut / hold cap | 35% / 260 ms |
| MCU post-tap settle | 220 ms |
| Startup grace / ramp | 8 / 8 packets |
| Front suppress / warning zone | ≤60 cm / 60–120 cm |
| Side avoidance / hard envelope | 40 / 22 cm |

The selected avoidance direction, rather than the old EMA sign, determines the
transmitted command. Duration scaling cannot underflow when avoidance overrides
a small lateral error. Cooldown/confirmation wait emits no new tap and does not
cancel a tap already running. Invalid/unsafe/centered geometry still sends STABLE
to cancel a correction. STOP clears direction confirmation and stale EMA state.
The MCU restores both wheel speeds at tap expiry and applies bounded corrections
symmetrically. Ultrasonic side nudging stays disabled (`ENABLE_ULTRASONIC_SIDE_NUDGE=0`).

## Local sonar and recovery

Servo endpoints are right 40°, center 80°, left 120°; production automatic
sampling remains centered, with a ±18° center cone. The servo must settle before
pinging. Echo capture uses an interrupt; safety loops do not call blocking scans.

| Parameter | Value |
| --- | ---: |
| Echo listening window | 18000 µs, roughly 3.1 m at the nominal sound speed |
| Ping period | 40 ms |
| Fresh echo age | 180 ms |
| Slowdown / full-speed resume | 90 / 105 cm |
| Immediate close echo STOP | ≤60 cm, using raw echo rather than delayed median |
| Persistent obstacle confirmation | 2 valid blocked echoes |
| Immediate emergency latch | ≤25 cm |
| Clear threshold / confirmation | ≥72 cm / 3 valid echoes |

A timeout/no echo is UNKNOWN. It never clears a previous local obstacle latch,
and breaks consecutive-clear confirmation. A blocked wait continues UART,
watchdogs, sonar sampling, and status requests. It counts distinct new sonar
samples and needs fresh LiDAR GO as well. An obstacle discovered during a
LiDAR-only wait activates the local-clear requirement too.
If no valid clear echoes return, the robot stays stopped and prints five-second
diagnostics indicating path state, sonar validity, distance, and confirmations.
The 18000 µs window replaces the old 8000 µs window that could not receive an
echo beyond approximately 1.4 m; it must be checked on the installed sonar.

## Motors and routes

Main step/direction pins: wheel 1 = 21/22; wheel 2 = 13/12.
Main and bridge UART pins: RX18/TX19, crossed TX-to-RX with common ground.
Servo23, sonar trig5/echo4, HX711 data33/clock32, buzzer25,
Air780 RX16/TX17/PWRKEY26. Verify electrical levels and boot-strapping pins.

| Parameter | Value |
| --- | ---: |
| Normal / caution / turn speed | 5600 / 3900 / 3000 steps/s |
| Normal acceleration | 4800 steps/s² |
| Gentle / safety stop deceleration | 6500 / 14000 steps/s² |
| Nudge acceleration | 9000 steps/s² |
| Wheel steps/rev / diameter / wheelbase | 1600 / 65 mm / 150 mm |
| Motor trim | 0% each, pending chassis calibration |

Both `moveTo()` results are checked. One rejected wheel command stops both
wheels; success is not guessed from immediate `isRunning()` state. Target moves
and turns poll safety continuously. Safety stops use controlled deceleration,
with hard force stop as a timeout fallback; no reverse braking pulse is used.

`pointsRun.ino` contains physically calibrated turn/distance sequences, including
long positive moves in outbound and return. Signs and numbers were preserved:
the physical route direction is hardware-unverified and must not be inferred
from function names. Wheel pulse counts cannot detect lost steps or wheel slip.

## Firebase, app, and return identity

Firebase writes for telemetry are background, coalesced multi-location updates.
Sensor values publish roughly every 5 s; steady weight refreshes every 30 s;
the Pi heartbeat is every 30 s. The MCU modem initializes in a background task;
SMS cannot block motion safety. Delivery requires actual network/SIM acceptance.

The app writes `devices/garby-bin-01/commands/reset` with server `requestedAt`,
operator `requestedBy`, and status `pending`. The Pi polls every 2 s, accepts a
new pending request only within 120 s (future tolerance 30 s), and relays
`[RESET:<requestedAt>]`. The MCU validates and remembers the ID. Duplicate IDs
return the remembered outcome and do not brake/restart an active return.
Before ACK the Pi retries the same ID every 3 s; after ACK it queries every 10 s.
This recovers a lost completion notification through the MCU's remembered
outcome, including after BLE reconnect, without replaying the route.

ACK means the **MCU accepted** the return, not that the Pi queued a message.
DONE requires matching MCU completion after the route succeeds; uncorrelated
`[IDLE]` never completes an app command. Firmware/route failure yields matching
FAILED. Accepted returns have a 15-minute completion window; they are not
rejected at the 120-second acceptance limit mid-route. A timeout means completion
is unverified, and never implies physical arrival. The app uses the same
15-minute window, replacing its former 60-second timeout.

Firebase command transitions are conditional transactions on the expected ID;
an old completion cannot overwrite a newer operator request. The app tracks the
exact server-issued request timestamp, including successive requests by the
same operator. Structured status
is authoritative; the obsolete APP boolean cannot resurrect terminal commands.
The app requires a live connection, fresh heartbeat, BLE and LiDAR health, and
confirmed destination readiness to enable return. Cached sensor/status data
becomes stale after 90 s, with 30 s future-clock tolerance.

Manual `[RESET]` is reserved for local supervised recovery: after a route fault,
physically reposition the robot to its known base, then issue it to clear state.
It must not be used to blindly return from an incomplete route. Correlated app
retries cannot enter this recovery branch. Deploy Pi and both ESP32s together;
old firmware cannot acknowledge the new correlated return workflow.

FCM receiving exists in the app, but this repository does not include a deployed
cloud notification sender or production RTDB authorization rules. Live Firebase
permissions, anonymous-auth configuration, push delivery, and SIM/SMS delivery
remain external acceptance items, not verified build results.

## Verification and release

Run `python -B tools/verify_project.py` for Python syntax/tests, executable firmware
behavioral tests, source audit, ESP32 target builds, Android tests/builds/lint.
Exports go to `artifacts/firmware/`; Android APKs remain in Gradle outputs.
`--host-only` explicitly skips target and Android builds.

Deployment order: main controller, BLE bridge, complete Pi `RasPi/` directory,
then Android. Follow `DEPLOYMENT_AND_ACCEPTANCE.md` with wheels lifted. A hardwired
emergency stop and measured stopping distance are required before claiming safe
operation around people. All changes to motion, protocol, or timing must update
this architecture and its maintainer reference copy in the same release.
