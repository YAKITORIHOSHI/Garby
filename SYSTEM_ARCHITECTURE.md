# GARBY system architecture

Turn recovery update (2026-11-14, build `RECONNECT-20261114`): BLE-session resets invalidate
path/steering freshness while retaining the executor's turn scope. After bridge boot, a
running executor's `[MCU ALIVE]` does not prove straight mode. GO is withheld until an
authoritative `[TURN:START]` or `[TURN:END]` arrives. The executor replays its complete turn
scope, including stopped holds, before status readiness; repeated START is idempotent.
An explicit controller `[MCU READY]` or confirmed operator RELEASE establishes idle mode.
Occupied, stale or absent `T=` continues to stop an interrupted turn after reconnection.

Release: 2026-10-04. Software builds and behavioral tests are documented in
`VALIDATION_RESULTS.md`; physical acceptance remains hardware-unverified.
This file describes the active source. Older audit reports are historical.

## Components and ownership

| Component | Active source | Responsibility |
| --- | --- | --- |
| Raspberry Pi | `RasPi/final_w_serial.py`, `bridge_core.py`, `ble_support.py`, `runtime_support.py` | ROS 2 LiDAR safety, wall geometry, BLE central, exclusive runtime ownership, sensor serial input, asynchronous Firebase telemetry and correlated return relay |
| BLE ESP32-S3 | `BLE_Receiver-Final/BLE_Receiver-Final.ino`, `MpuHeading.h`, `SteeringAssist.h` | BLE GATT server, path validation and STOP latch, MPU/rate feedback, confirmed LiDAR centering, controller UART transport |
| Main ESP32 | `MAIN_EXECUTOR_CODE.ino/.cpp/.h`, `pointsRun.ino` | Motor targets, local sonar protection, load/gas triggers, route state, modem/SMS worker |
| Android | `Garby_MobileApp/app/` | Authenticated Firebase dashboard, confirmed manual-disposal intent, explicit return request and matching completion |

The Android project contains no Pi code (its old compatibility launcher was
removed on 2026-11-14; run `RasPi/final_w_serial.py`). The Pi simulator is a manual
test producer, not production firmware. `Test/Test.ino` is a serial-only sketch.

```text
LaserScan -> Pi path + clearance + wall line fit -> BLE ESP32-S3 -> UART -> main ESP32 -> motors
MPU6050 -> BLE ESP32-S3 continuous heading (M, 50 Hz) + wall fits (W) -> main ESP32 fusion and steering
Sensor board -> Pi telemetry -> bridge -> main ESP32 gas trigger
Main ESP32 load/state -> bridge notifications -> Pi -> Firebase -> Android
Android return intent -> Firebase -> Pi -> bridge -> main ESP32
Android confirmed disposal intent -> Firebase claim -> Pi START -> bridge -> main ESP32
Main ESP32 matching outcome -> bridge -> Pi -> Firebase -> Android
```

Motion remains gated independently of the app, Firebase, GUI, and SMS.
FastAccelStepper executes commanded steps without wheel encoders: it is open
loop motor control. The MPU6050 supplies relative heading for MPU-measured turns
and continuous straight-line steering; LiDAR wall line fits measure the hallway
direction and lateral offset (2026-11-11, `docs/navigation-2026-11-11.md`), and
LiDAR also handles obstacles/path clearance. Wheel pulse counts do not prove motion.

## Intended complete cycle

1. Boot stationary. The main MCU initializes peripherals and sends `[MCU READY]`
   before the background modem starts. Load-cell taring and reads run in their
   own worker and cannot delay readiness. The bridge starts fail closed.
2. The Pi obtains usable scans and sends fresh path packets. Two clear packets
   release the bridge latch; the MCU needs two GO commands. A fully latched
   recovery therefore needs at least three Pi packets across both layers.
3. While IDLE, two filtered load samples at or above 4.0 kg or three dangerous
   gas telemetry packets trigger outbound execution. Every segment needs live
   path permission; triggering the route never bypasses that gate. An operator
   can also request manual start after the Pi/bridge/executor handshake and a
   fresh IDLE report, bypassing the scale and bin/gas trigger thresholds.
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
- MCU GO confirmations need at least 80 ms between them; a buffered burst cannot
  supply both confirmations. A GO after an expired gap starts a new streak.
- Oversized or non-printable UART input invalidates path permission immediately
  and discards the damaged line; queued GO cannot bypass new confirmation.
  Both LF and CRLF delimiters are accepted; embedded carriage returns are rejected.
- BLE/LiDAR loss never starts an automatic return.
- GUI All-front/All-back toggles keep numeric readings live and bypass the
  selected group's distance thresholds. Either or both may be read only without
  adding a global motion/Start interlock or cancelling a pending Start. Missing
  or stale coverage, the 22 cm side envelope, emergency stop and MPU/nearest
  steering-clearance gates remain authoritative. Both toggles default to checks on
  when the Pi process starts.
- The BLE writer converts a queued path to STOP when its Pi-side LiDAR evidence
  deadline has expired. This check happens before the GATT write; the wire
  protocol does not carry source acquisition time end to end.
- LiDAR and sonar report generic range obstacles, not human identity. A person
  is only detected if a valid return intersects the LiDAR scan plane and meets
  current coverage/range rules. No person classifier or guaranteed stopping
  distance is established by software tests.
- Manual start changes IDLE to the normal outbound route without granting GO.
  The bridge requires Pi readiness, MCU readiness and a healthy UART. The executor
  independently requires its current connection handshake, IDLE and no route fault.
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
| Initial BLE setup / established silence watchdog | 45 s before first valid write / 10 s afterwards; never motion freshness limits |
| Pi discovery / connection / notification / write deadlines | 12 s / 30 s / 8 s / 2 s |
| Inactive advertising retry / MCU state query | 2 s / 5 s |
| Repeated stale STOP | 500 ms |
| Main path watchdog | 800 ms |
| Minimum MCU GO confirmation interval | 80 ms |
| Movement gate | 900 ms, then continuously monitored clearance wait |
| Status request throttle | 80 ms |
| Bridge control queue | 8 frames; overflow clears uncertain backlog and asserts STOP |
| MCU ACK timeout / backlog cap | 3000 ms / 8 ordinary commands |
| Manual-start readiness/state age / outgoing queue age / ACK deadline | 7.5 s / 2 s / 5 s |

Individual watchdogs are not a proof of total stopping time. Sensor age,
transport delay, receive scheduling, braking, wheel slip, and load must be
included in physical measurements.

## LiDAR

Software angles are FRONT | 0°, FRONT_LEFT 45°, LEFT 90°, BACK_LEFT 135°,
BACK 180°, BACK_RIGHT 225°, RIGHT 270°, FRONT_RIGHT 315°.
`GARBY_LIDAR_YAW_OFFSET_DEG` corrects the physical mount; default 0 is
hardware-unverified. Verify it before floor operation.

For the owner's reported reversed lateral readings, the Pi defaults
`GARBY_LIDAR_REFLECT_LEFT_RIGHT=1`. Raw rays are first rotated by the yaw
offset, then reflected: `robot_angle = -(raw_angle + yaw_offset)`.
The same transform supplies safety sectors, wall estimates, nearest CL/CR and
the circular map. Reflection preserves forward distance and exchanges left/right,
including diagonals; it does not change motor commands or gyro signs. Set the
variable to `0` only for an installation verified to need the earlier orientation.
The ROS driver settings and generic scan-partition coordinate convention remain.

Collision processing covers the full circle with nearest 45° sectors, using the
closest valid return. Steering uses median estimates in centered 22° slices,
falling back to the wider sector when the slice is sparse. Positive infinity
means no return within `range_max`; NaN/negative infinity are invalid.
At least three samples must exist in each front, rear, and side sector.
An invalid side sector blocks both directions. Invalid angular/range metadata
cannot crash the ROS callback or become CLEAR.

The monitor also receives an optional immutable `LidarMapSnapshot` from
`RasPi/lidar_map.py`, containing at most 360 measured endpoints in robot-relative
forward/left coordinates. It retains the nearest finite hit in each 1° bin at
that hit's original angle; infinity and invalid ranges produce no plotted wall.
It uses the same mounting yaw offset and acquisition-age deadline as the scan
processor. This current circular scan has no localization or accumulated map.
The eight safety sectors and all BLE/UART path fields remain unchanged.
Since 2026-11-12 this map is the monitor's only LiDAR view: a radar scope with the
eight sector distances written on their own sides, stop distances drawn as arcs,
and, without a live scan, a radar sweep while the LiDAR is found and started or the
supervisor's reported reason ("LiDAR not available"). The supervisor's phase reaches
the monitor through `read_health()["lidar_driver"]`; it is presentation only and
never path evidence (`docs/lidar-map-2026-11-12.md`).
Map construction runs after urgent path dispatch, outside the history lock, and
is skipped in headless operation. A bounded angle cache and reused Canvas items
limit work and memory; hidden views defer drawing. Legacy five/six-field display
frames remain accepted. Production supplies the optional map as field six and
the acquisition time independently as field seven, so a geometry failure cannot
renew the summaries' age at dequeue. Explicit invalid source times stay unavailable.
Summaries omit sectors without usable current coverage while retaining the
underlying smoothing history. Processing errors reuse stale-scan invalidation:
health and old evidence are cleared and the watchdog queues STOP once per
unavailable interval. A new valid acquisition restores normal processing.

| Envelope | Threshold |
| --- | ---: |
| Front sector | 95 cm |
| Front-left and front-right sectors | 60 cm |
| Rear sectors | 35 cm |
| Side hard envelope | 22 cm, blocks both travel directions |

Chassis zones (`RasPi/hallway_geometry.py`, 2026-11-11) are evaluated on the same
scan and only add evidence. Frame: x forward from the LiDAR, y left from the
centreline; outline from `GARBY_CHASSIS_*` (default 60 x 60 cm, LiDAR centred,
unmeasured). Returns inside the outline are treated as the robot itself.

| Zone | Region | Effect |
| --- | --- | --- |
| Front box | ahead of the front edge to 95 cm, body width + 10 cm each side | `F=O` (front-group bypass applies) |
| Slow zone | same corridor, 95-160 cm | `F=W` -> `GO:SLOW` (caution speed) |
| Side band | beside the body, within 2 cm of its sides | both directions blocked, never bypassed |
| Spin sweep | within half diagonal + 8 cm of the spin centre | `T=O`; halves follow the front/back bypass |

Wall line fits (`WL`, `WR`, `H` in the S frame) use returns 32-300 cm to each side
within 80 cm behind to 160 cm ahead; a fit needs 15 inliers within 3 cm over 60 cm,
rms <= 2.5 cm, no bow over 1.5 cm and an angle within 20 degrees. Walls that disagree
by more than 3 degrees keep only the stronger one. Changes in the active zones are
logged and shown in the monitor footer; the eight sector readings do not show them.
A zone that appears is logged at once; one that clears is logged after 2 s without
another line, so an object on a zone boundary cannot print a line per scan.

The front diagonals use a shorter limit because they also see hallway walls: a
wall W cm to the side reads about 1.08 W. At 60 cm the robot stops when a wall is
closer than about 55 cm, which passes a 164 cm gap up to about 26 cm off center.
Nothing inside the 22 cm side envelope loses the 95 cm limit, because 60 cm at the
22.5 degree sector edge is already 23 cm to the side. Under 60 cm the bridge
withdraws steering permission, so do not lower this limit without reviewing both.

One sensor-data subscription with depth 1 avoids a backlog of old scans.
Scan history and path snapshots share a lock. Periodic and urgent path messages
share one mailbox; an older clear packet cannot overwrite a newer STOP, including
across uint32 sequence wrap. No unbounded urgent-write futures are spawned.

### Moving-object prediction (2026-11-12)

`RasPi/motion_tracker.py` runs on every acquisition after the sectors and zones (≈2.3 ms on
the Pi 5). It segments the scan (adaptive breakpoint), ignores segments over 90 cm
(structure), Kalman-tracks the rest with **ground** velocity (the executor's `[DRIVE:..]`
lines remove the robot's own motion; without them the robot speed is taken as anywhere in
0-75 cm/s and only objects closing faster than 1 m/s count), and calls a track moving only after 3 measured updates (2 above 1.5 m/s) with a
significant speed and ≥20 cm of real travel. Time to contact with the footprint grown by
20/15/10 cm (front/sides/rear) plus the object's radius:

| Prediction | Path code | Bypassed by the read-only front group |
| --- | --- | --- |
| contact ≤2.0 s | `F=O` (`BLOCKED{MOVING}`) | yes |
| contact ≤4.0 s, or a mover closing within 2.5 m ahead | `F=W` | yes |
| a mover reaching the spin circle ≤2.0 s | `T=O` | yes |

It only adds evidence, held 0.6 s (STOP) / 1.0 s (slow) after the last positive scan; a fault
in it resets it and leaves the sectors and zones in charge. Things catching up from behind and
things already abeam or moving off are not contacts. It reports moving objects, not people:
the LiDAR on top of the bin sees torsos, nothing below its plane. Logged as
`Moving-object prediction: ...`; zones `MOVING_STOP`, `MOVING_SLOW`, `MOVING_TURN` in the footer.

## BLE and UART protocol

BLE name: `GarbyESP32`.

Discovery diagnostics observe the existing active scan, without adding a scan or
relaxing identity/pinned-address matching. On timeout they distinguish no callback
events, unmatched events and GARBY candidates rejected by the strict pin, with
adapter/pin selection and bounded evidence. No matching event alone does not prove
the S3 is unpowered or not advertising. Standalone-supply faults remain separate
from GATT/readiness failures; see `docs/ble-establish-2026-11-09.md`.
Service: `4fafc201-1fb5-459e-8fcc-c5c9c331914b`.
Write: `beb5483e-36e1-4688-b7f5-ea07361b26a8`.
Notify: `beb5483e-36e1-4688-b7f5-ea07361b26a9`.
Requested connection interval: 30–50 ms, supervision timeout 4 s. The bridge
requests it only after Pi readiness and at least 1 s after connection.
Advertising interval: 100–150 ms; scan response enabled. Advertising restarts
automatically on disconnect, with a 2 s retry if it remains inactive.

The Pi scans actively by configured address (strict match) or service UUID/exact
name and passes the scanned BLEDevice to the client. `GARBY_BLE_ADAPTER` selects
the Linux adapter; an unset address permits discovery. UUID overrides are
validated. It explicitly requests BlueZ StartNotify, including compatibility
with Bleak 3.0.0/3.0.1. A session becomes ready only after subscription and an
acknowledged `[RASPI READY]` write. Failures clean up the candidate and retry
with backoff; callbacks belong to their session. Shutdown cancels setup and
waits for bounded disconnect cleanup. Disconnect/write failure publishes
UNKNOWN readiness. The 45 s setup grace cannot release path STOP.
See `BLE_CONNECTION_GUIDE.md` and `RasPi/check_ble.py` for isolated diagnosis.

```text
Pi -> bridge:
P:<uint32-seq>|F=<C|W|O|H|S>|B=<C|O|H|S>|T=<C|O|S>
   (W: clear to the stop, caution zone ahead; T: spin-sweep clearance, gated only in turns)
S:<same-seq>|L=..|R=..|F=..|B=..|FL=..|FR=..|BL=..|BR=..|CF=..|CL=..|CR=..|A=<uint32>[|WL=..][|WR=..][|H=..]|G=<ms>|T=..
   (WL/WR: wall distances from the centreline, cm; H: heading vs walls, deg, + = left;
    G: scan age refreshed immediately before the Pi's GATT write, ms; T: legacy tilt, ignored)
S:<same-seq>|STABLE
SENSOR:US=..|MQ4=..|MQ137=..|MQ135=..
[HEALTH:1:UUUUU]
[RASPI READY]
[RESET:<requestedAt-ms>]
[START:<request-id-ms>]
[START STATUS:<request-id-ms>]
[OP:<id>:ESTOP|RELEASE|SONAR_OFF|SONAR_ON|REBOOT]
[OP STATUS]

Bridge -> main, newline-delimited UART at 115200 baud:
STOP[:reason]            (reasons include MPU, TURN, TURN_UNKNOWN, STALE, LINK, OPERATOR, ...)
GO | GO:SLOW
M,<seq>,<heading cdeg>,<rate cdeg/s>,<epoch>,<allow bits>*<xor hex>   (50 Hz, no ACK)
MPU:FAULT                                                            (no valid heading)
W,<acq>,<age ms>,<flags>,<left mm>,<right mm>,<H cdeg>*<xor hex>      (per LiDAR scan, no ACK)
SENSOR:...
[HEALTH:1:UUUUU]
[BLE CONNECTION ESTABLISHED]
[ROBOT STATUS]
[RESET:<requestedAt-ms>]
[START:<request-id-ms>]
[START STATUS:<request-id-ms>]
[OP:<id>:ESTOP|RELEASE|SONAR_OFF|SONAR_ON|REBOOT]
[OP STATUS]

Main -> bridge only (local UART, not relayed to the Pi):
[HEADING:START:<route error cdeg>] / [HEADING:END]   (straight segment; logged, never re-zeroes)
[TURN:START] / [TURN:END]   (bridge requires T=C and fresh confirmations in between)
[MCU ALIVE]   (recovery ping during a route; changes nothing. [MCU READY] = restart)
State lines [IDLE]/[OUTBOUND COMPLETE]/[ROUTE FAULT] vs [RUNNING]/[RETURNING] also
gate the bridge's gyro-bias refresh; they are still relayed to the Pi.

Main -> Pi through bridge notifications:
[MCU READY], [RUNNING], [OUTBOUND COMPLETE], [RETURNING], [ROUTE FAULT], [IDLE]
[BRIDGE BUILD:<id>], [BRIDGE BOOT:<reset>], [BRIDGE DIAG fw=... reset=... up_s=... drops=... last_drop=0x... ...]
[RESET ACK:<id>], [RESET DONE:<id>], [RESET FAILED:<id>]
[SYSTEM READY]
[START ACK:<id>], [START FAILED:<id>:<reason>]
[OP ACK:<id>:<action>], [OP FAILED:<id>:<reason>]
[OP STATUS:1:ESTOP=<0|1>:SONAR=<0|1>]
[MOTION:<CLEAR|WAITING_FOR_PATH|CONFIRMING_CLEARANCE|LOCAL_OBSTACLE|MOTORS_UNAVAILABLE|ROUTE_FAULT|STOP[:reason]>]
[BRIDGE MPU VALID|INVALID bias=.. tilt=.. refresh=.. epoch=.. walls=..]
[HEADING TURN ...], [HEADING STRAIGHT ...], [HEADING FUSE ...], [HEADING LOST ...], [HEADING ROUTE:...]
LOAD_CELL:<kg>
[REQUEST-STATUS]

Notification wire framing (each ATT payload <=20 bytes):
~<first part>
+<continuation part>...\n
```

`C` = clear, `O` = obstacle, `H` = legacy human-tagged obstacle, `S` = stale/unknown.
`H` is a backward-compatible stop code only; the production LiDAR does not
classify people.
Strict path fields reject duplicates, unknown fields, uint32 overflow, and
trailing delimiters. Old or duplicate path sequences assert `STOP:SEQUENCE`.
Legacy combined `PATH:` input remains isolated for old bench tools; production
and the sequenced simulator use `P:`/`S:` only.

Each Pi-to-bridge GATT payload is a single printable line, maximum 244 bytes. Embedded NUL,
newline, or control bytes are rejected, so SENSOR cannot inject UART commands.
All Pi writes share one asyncio lock; path, readiness, manual start, and correlated return
writes are acknowledged. Steering/telemetry exceeding the characteristic's
negotiated write-without-response limit also use acknowledged writes; this
handles a 20-byte limit during MTU negotiation or on older BlueZ. Failed safety
writes request a fresh snapshot.
The P and S mailbox entries retain the same acquisition deadline. After waiting
for the writer lock, an expired P becomes STOP and an expired S becomes STABLE;
otherwise S.G is recomputed from that deadline. Age-field growth beyond the
244-byte limit also produces STABLE. The bridge adds its actual mailbox wait and
the 30 ms transit estimate. Steering permission expires at 500 ms acquisition age,
with none for absent age; walls retain their separate 900 ms forwarding limit.
Duplicate acquisitions refresh neither product. BlueZ/GATT delays after the
write begins are estimated, not measured end to end; P still has no
controller-checked source acquisition timestamp.
Bridge BLE callbacks validate framing and copy/post bounded data. A valid
operator ESTOP also asserts a protected stop latch immediately in the callback;
UART, notifications, navigation resets and connection-parameter work run in `loop()`.

Operator actions use canonical decimal IDs (1–20 digits, no leading zero).
The Pi reserves ID batches durably beside its stop-state file so a process
restart or wall-clock rollback cannot reuse an already transmitted ID. The
executor remembers the latest ID/action/result, rejects stale/reused IDs, and
never automatically repeats a release, sonar change, or reboot. A stale ESTOP
still asserts STOP conservatively. The Pi's dedicated operator mailbox takes
priority over ordinary scan/telemetry traffic; ESTOP is repeated until confirmed.
The bridge has a separate emergency mailbox and immediately rejects GO,
GO:SLOW, starts and legacy reset/return while the operator latch is set.

ESTOP force-stops both motor outputs, centers the servo, invalidates GO evidence,
and unwinds the route to stationary IDLE. RELEASE is accepted only when idle,
stopped and ready; the bridge unlocks only on the matched executor ACK with no
newer stop generation. Release requires a new manual start and never resumes
the aborted route or an automatic sensor trigger. Sonar changes also require
stationary IDLE and readiness. SONAR_OFF cancels pings, bypasses only the local
obstacle-sonar gate and locks the servo at DEFAULT_VIEW (70 degrees). SONAR_ON
requires fresh clear confirmations; it is the default on executor boot. Bin
level sensing is separate. Confirmed SONAR_OFF does not itself block motion or
manual start. LiDAR front/rear read-only controls independently bypass their
selected distance thresholds while retaining readings. Side, stale-data, MPU,
nearest steering-clearance, emergency-stop and transport gates remain active.

REBOOT requires confirmed operator STOP. The executor commits a 750 ms delayed
software restart and acknowledges; the bridge commits a 1000 ms delayed restart
only after that matching ACK. The Pi waits two seconds after the matching
attempted command's ACK and requests `systemctl --no-ask-password reboot` in a
worker thread. No forced OS reboot, shell command or privilege change is used.
The bridge retires missing reboot correlation after five seconds while keeping
STOP. Pi action deadlines are two seconds to send and five seconds for ACK;
expired/disconnected actions cannot be written later. A lost ACK may result in
only the controllers restarting; the Pi never assumes an unconfirmed reboot.
Pi OS permission failures remain visible with STOP latched. Both MCUs retain
validated no-init RTC stop markers across software restart; the Pi stop file
persists across OS/process restart. Cold power loss follows ordinary fail-closed
startup, and a retained Pi stop is reapplied. The bin/gas board has no supplied
reset endpoint and is excluded. This protocol requires both firmware updates
and the complete updated Pi directory; old firmware shows unknown/unavailable
operator status. The GUI is a software command path, not an independent physical
emergency-stop circuit.

Bridge-to-Pi notifications use bounded default-MTU-safe chunks: `~` starts a
message, `+` continues it, and newline ends it. Pi reassembly is capped at 256
bytes; a new start discards an incomplete older message. Legacy unframed
notifications remain readable. New bridge framing requires the complete new Pi
directory, including `ble_support.py`. The bridge queries MCU state every 5 s
after Pi/MCU readiness; correlated return retries recover lost completion.
The bridge announces the connection to the executor only after `[RASPI READY]`.
The updated executor reports `[SYSTEM READY]` alongside its state on handshake
and each status query, proving the round trip independently of scale/bin data.
The Pi requires both this heartbeat and IDLE to be at most 7.5 seconds old to
enable manual start. MCU READY or BLE loss invalidates pending starts. Starts
share the ordered control queue and existing session/ingress age checks; the Pi
also checks its 2-second queue deadline after acquiring the GATT write lock.
No layer automatically retries a start. After one attempted GATT write, the Pi
queries `[START STATUS:<same-id>]` at most once per second within the existing
5-second ACK deadline. The query is checked again at the GATT write boundary,
belongs to the same pending request/session, and cannot consume a request ID,
refresh path permission or start a route. An executor returns its remembered
outcome or `NOT_RECEIVED`, followed by current state and motion diagnostics.
Query transport failure is nonterminal: the bridge must not invent a rejection
for an original start it might already have delivered. A missing application
ACK expires at 5 seconds, after which new state evidence is required before
another request.
The executor remembers the latest outcome across `fullReset()` and rejects
older request numbers, preventing a duplicate/reordered start from launching
another cycle. IDs are positive, strictly increasing decimal values of at most
16 digits. Failures identify NOT_READY, NOT_IDLE, ROUTE_FAULT, STALE_REQUEST,
MOTORS_UNAVAILABLE, UART_BUSY or NOT_RECEIVED. New starts rejected by bridge UART
backpressure receive an explicit failure rather than being silently dropped.
The executor reports its motion hold on changes and status queries; this is
diagnostic only, never drive permission. The Pi displays holds after start
acceptance and logs executor state and correlated outcomes.
Executor setup does not wait for BLE: the normal loop services the existing
handshake retry with motors held fail-closed. The Pi paints the monitor before
workers start, begins BLE discovery before ROS setup, and reschedules GUI
refreshes after a snapshot/drawing exception.
Use the updated Pi and both firmware projects together for manual start.

### Confirmed manual disposal (2026-11-13)

The Pi monitor and Android dashboard label the threshold-bypassing action **Manual disposal**
and require confirmation. It dispatches the existing outbound route from IDLE, independently
of bin/gas/load readings. Automatic triggers and all motion gates retain their behavior.
The executor emits `[DISPOSAL READY:1]` with status only when its connection is ready, it is
IDLE, motor channels exist, and no stop/fault/reset/destination state prevents a new run.
The bridge suppresses that eligibility on an unready UART/BLE session or operator stop.
The Pi additionally requires fresh SYSTEM READY and IDLE (7.5 s) and no local operator stop.
It publishes `status/manualDisposalReady` and `manualDisposalUpdatedAt` on controller reports.
These are eligibility hints, never clearance; Android expires them after 15 s with 5 s future
tolerance and still requires a live Pi, cloud, phone internet, BLE and explicit IDLE.

`commands/dispose` is separate from return `commands/reset`. Android submits one version-1
pending command with a random 32-hex `requestId`, server `requestedAt`, and operator UID.
Its compare-and-set transaction refuses concurrent replacements or unresolved commands.
The Pi polls every 2 s in its own worker, rejects pending requests older than 30 s or over
5 s in the future, then transactionally records `dispatching`, `controllerRequestId` and
`claimedAt` **before** enqueueing one `[START:<id>]`. Claim I/O uses the same 2-second queue
budget; the GATT writer rechecks session, state and emergency stop. A restart or uncertain
claim can only recover with `[START STATUS:<id>]`, never resend START. After at most 15 s
without a matching result, the cloud result is `unconfirmed`. A matching START ACK alone
produces terminal `accepted`; rejection produces `failed`. Accepted means the executor
accepted the outbound route, not arrival or wheel movement. The ordinary robot state shows
arrival; return remains a separate explicit request. Identity-matched cloud transactions
preserve terminal results and newer requests. See `docs/manual-disposal-2026-11-13.md`.

The bench producer uses the same discovery/GATT/framing helpers and sequenced
mailbox, but its synthetic distances must not
replace the production LiDAR runtime.

Sensor-board UART into the Pi is **9600 baud**, separate from the 115200 bridge
UART. Its firmware is not provided here. `US=999` and gas `-1` mean UNAVAILABLE,
not full/danger/clearance. Trash-level ultrasonic telemetry is not path sonar.

## Pi process and serial ownership

Both Python install profiles require NumPy for chassis zones, walls and moving
object prediction. Runtime preflight reports its absence as a failed prerequisite.

Production, the bench simulator, and `check_ble.py --connect` share an OS lock
through `runtime_support.py`. Its default is `/tmp/garby-bridge.lock` on the Pi;
`GARBY_RUNTIME_LOCK_FILE` overrides it. Use the same path for all three programs.
A competing instance exits before connecting or launching a driver. Scanning
alone does not take this lock. Closing/exiting releases the lock; the file remains
and must not be deleted to bypass an active owner.

GARBY launches its LiDAR wrapper in a separate POSIX process group and terminates
only that owned group, including children after the wrapper exits. Failed or
cancelled startup cleans up its child. Startup, restart, and shutdown no longer
kill unrelated LiDAR/GPIO processes, stop the global ROS daemon, or erase DDS
shared-memory files. The LiDAR and sensor UART must resolve to distinct devices.

`port_detection.py` assigns both devices by their data: chained YDLIDAR packets at 115200 and
`LABEL:number` lines at 9600. It listens only, holds each role's port so the other search cannot
open it, and identifies a device by its USB device number, so re-plugging triggers a new search.
The driver takes no OS lock on its port, so this reservation is what keeps a probe off it. An
explicit `GARBY_LIDAR_PORT` or `GARBY_SENSOR_SERIAL_PORT` disables detection for that device.

## Load-cell availability

The lowest-priority HX711 worker owns all converter reads. It collects five zero samples within
each two-second tare attempt and retries in the background until successful.
Boot with the bin empty; an unavailable tare never supplies usable weight.
After taring, the worker samples roughly every 100 ms. Cached readings must be
finite, within the 20 kg plausibility limit, and no older than one second.
Invalid/stale data resets the IDLE filter and confirmation streak; two distinct
valid samples are still required for a load trigger. Gas triggering is independent.

MCU telemetry sends `LOAD_CELL:-1.00` when weight is unavailable. The Pi publishes
that sentinel with `updatedAt=0` for modern and legacy fields; Android displays
unavailable weight rather than a negative kilogram value. A restored converter
can finish taring and resume valid samples without holding up motion safety.
HX711 calibration and the 20 kg limit require verification on the actual scale.

## MPU heading, turns and hallway centring (2026-11-11)

Full design, calibration and acceptance: `docs/navigation-2026-11-11.md`.

| Parameter | Value |
| --- | ---: |
| Reader | MPU6050 6-axis, I2C 100 kHz, 25 ms bus timeout; yaw = gyro vector . measured up |
| Sample / maximum allowed gap | 10 ms / 100 ms |
| Range / low-pass filter | ±250°/s / 20 Hz |
| Scale | `GYRO_SCALE` (1.0 until calibrated) |
| Bias | boot (2 s still); refreshed while held: ≤0.2°/s per 1.5 s window, ≤1°/s total |
| Heading stream | M line every 20 ms; one continuous reference, new epoch only on re-reference |
| Turn | MPU degrees, tolerance 1°, ≤5 passes, wrong-way swap once, step limit 1.5x |
| Turn wall alignment | 2 scans agreeing within 1.5°, within 8° of command, tolerance 0.75°, ≤2 passes |
| Hallway axis filter | alpha 0.15 / beta 0.01 per scan, gate 4°, ≤15° from the route frame |
| Lateral | gain 0.5, gate 15 cm, dead reckoning ≤8 s between wall measurements |
| Centring | deadband 2 cm, engage 5 / release 2 cm, -atan(excess / 120 cm), ≤5°, 4°/s slew |
| Inner heading hold | 2.5/s on error + 0.28 s rate lead, ≤30% one-wheel cut, 1.5%/20 ms |
| Steering permission | none if CF/F ≤60 cm or a wall median ≤22 cm; per side ≤40 cm (CL/L/FL/BL) |

The executor anchors a route frame at route start (it refuses to start without a
confirmed heading stream). Each turn adds its angle, is driven to it by the gyro,
then checked against the walls; the measured hallway direction becomes the frame.
On centring-enabled straights (`isNudgeEnabled`) each W line gives
`axis = gyro heading at the scan - H` and `lateral = (R - L)/2` (or one wall plus
the learned width). The steering target is `axis + centring`; between scans the
gyro carries the axis and step travel the lateral estimate. Without walls the
target is the route frame with step-based cross-track return. At the end of a
centred straight the measured axis becomes the route frame.

Permission bits from the bridge only veto steering toward an unsafe side; with
none, both wheels run at base speed. A repeated acquisition A refreshes nothing.
Every motion start/resume requires heading readiness. Missing heading brakes
straights, plain/reverse moves and turns. An outage during an existing STOP/sonar
hold enters the same five-minute recovery deadline (low power after 10 s).
After at least 10 s with a new S3 reference, two independent fresh wall scans
agreeing within 1.5 degrees must confirm the route direction within 8 degrees of
its right-angle grid; only that residual is corrected. Missing/disagreeing walls
keep it stopped (`verify=WAIT_WALLS`) within that same budget, then the segment
fails. An off-grid measurement fails immediately. A pair spanning another heading
reference is discarded. Fresh GO and sonar clearance are still needed afterward.
Ordinary obstacle holds with continuous valid heading remain indefinite;
ordinary wall gaps do not trigger recovery verification. Turns never fall back
to step counting.

Pre-run centring (2026-11-12): before each long hallway run (`runLongHallway`) the
robot stands still; two wall scans agreeing within 3 cm / 1.5° give its offset and the
hallway axis. Within 3 cm it only faces the axis; otherwise it sidesteps (spin 35° toward
the centre, drive `|offset| / sin 35°`, spin back and wall-align), at most 3 times, learning
the real wheel distance from the first (`learn k=`), stopping on a wrong-way result. The
sidesteps' travel along the hallway is taken off the run. No walls skips it. `[MCU ALIVE]` changes nothing and
`[MCU READY]` restarts only the executor's frame. Ultrasonic side steering stays
disabled (`ENABLE_ULTRASONIC_SIDE_NUDGE=0`). Model checks:
`tools/firmware_tests/test_navigation.cpp` (kinematic, not physical).

## Local sonar and recovery

Updated 2026-11-12 ([report](docs/obstacle-sweep-hold-2026-11-12.md)). Servo commands: right 0,
forward 70, left 140. The servo **sweeps** (`SonarSweep.h`); before 2026-11-12 it was only ever
commanded to the forward view. The servo settles before each ping; echo capture uses an
interrupt; a trigger waits while ECHO is still high; safety loops never call blocking scans.

| Robot state | Sweep |
| --- | --- |
| Driving straight | forward, +25, forward, -25 (forward every ~0.1 s in the host model) |
| Stopped inside a route (holds, gates, before turns) | forward between each of ±25, ±50, ±70 (forward every ~0.2 s) |
| Spinning, idle, parked | forward only (250 ms pings idle/parked) |
| Low-power hold | forward at 4 Hz, one full survey every 4 s |

| Parameter | Value |
| --- | ---: |
| Echo listening window | 18000 µs, roughly 3.1 m at the nominal sound speed |
| Ping period (forward only) | 40 ms |
| Fresh forward sample age | 600 ms (was 180 ms, forward only) |
| Slowdown / full-speed resume | 90 / 105 cm (forward, or ±25 bearing inside the corridor) |
| Immediate close echo STOP | ≤60 cm forward, raw echo |
| Persistent obstacle confirmation | 2 blocked echoes (forward, or per side bearing) |
| Immediate emergency latch | ≤25 cm |
| Clear threshold / confirmation | ≥72 cm / 3 forward samples; 2 per side bearing |
| Spin circle (inside a turn) | 50 cm radius from the spin centre, surveyed before each spin |

An echo is placed at the beam edge nearest the path (±12°); side bearings count only if they
can be inside the 60 cm body + 8 cm corridor. A timeout/no echo means nothing within ~3 m and
is a **far clear reading** (until 2026-11-12 it was UNKNOWN, which could never release a hold
in an open hallway). Three complete surveys without an echo at any bearing report
`[SONAR:SILENT]` once (driving time never counts); a
stuck-high ECHO line reports `[SONAR:FAULT]`; neither holds the robot.

A blocked wait continues UART, watchdogs, sonar sampling and status requests, counts every new
sonar sample as clearance evidence and needs fresh LiDAR GO as well. It never ends the route:
`waitUntilMotionPermitted()` re-holds on any STOP. After 10 s without clearance evidence it
enters the low-power hold (CPU 80 MHz, 2.5 Hz polls, `[POWER:LOW]`); the first clear evidence
restores normal power. Five-second diagnostics show path state, sonar validity, distance,
confirmations and power.

## Motors and routes

Main step/direction pins: wheel 1 = 21/22; wheel 2 = 13/12.
Main UART: RX18/TX19. ESP32-S3 bridge UART: RX18/TX17, crossed TX-to-RX
with common ground. Bridge MPU: SDA8/SCL9, 3.3 V, +Z up; GPIO19/20 remain
available for native USB. Build the bridge for `esp32:esp32:esp32s3` and the
motor controller for `esp32:esp32:esp32`.
Servo23, sonar trig5/echo4, HX711 data33/clock32, buzzer25,
Air780 RX16/TX17/PWRKEY26. Verify electrical levels and boot-strapping pins.

A rising step count means forward everywhere: routes, spins and steering. Two constants
at the top of `MAIN_EXECUTOR_CODE.h`, `MOTOR1_DIR_HIGH_COUNTS_UP` (false) and
`MOTOR2_DIR_HIGH_COUNTS_UP` (true), alone connect that to the wheels. The owner's
November 9 report says the November 7 `true/false` build still drove backward;
the new pair follows that observation. Boot diagnostics identify the build
(`NAV-20261111`) and both DIR values. Which motor is the left wheel is a guess
checked by the first MPU turn (swapped once if the spin starts the wrong way);
steering uses the proven side. See `docs/motor-direction-2026-11-09.md` and
`docs/navigation-2026-11-11.md`.

| Parameter | Value |
| --- | ---: |
| Normal / caution / turn speed | 5600 / 3900 / 3000 steps/s |
| Normal acceleration | 4800 steps/s² |
| Gentle / safety stop deceleration | 6500 / 14000 steps/s² |
| Wheel steps/rev / diameter / wheelbase | 1600 / 65 mm / 150 mm (unverified; 4900 steps per 90° implies otherwise, see navigation report) |
| Motor trim | 0% each, pending chassis calibration |

Both `moveTo()` results are checked. One rejected wheel command stops both
wheels; success is not guessed from immediate `isRunning()` state. Target moves
and turns poll safety continuously. Safety stops use controlled deceleration,
with hard force stop as a timeout fallback; no reverse braking pulse is used.

`pointsRun.ino` contains physically calibrated turn/distance sequences, including
long positive moves in outbound and return. Signs and numbers were preserved,
including through the November 9 polarity correction. The physical route direction is hardware-unverified
and must not be inferred from function names. Wheel pulse counts cannot detect
lost steps or wheel slip.

## Firebase, app, and return identity

Firebase writes for telemetry are background, coalesced multi-location updates.
Sensor values publish roughly every 5 s; steady weight refreshes every 30 s;
the Pi heartbeat is every 30 s. The modem initializes and submits SMS from one
persistent background owner. Parser, motion and GUI paths only enqueue bounded
notices. No modem UART debug passthrough competes with its transactions.

### Health notices and cellular SMS

The Pi's separate daemon checks HTTPS reachability independently of Firebase
every five seconds when its probes finish within that interval. Two-second
socket timeouts and bounded response reads do not block ROS, BLE or Tk; a
stalled/DNS-delayed probe causes expired status to become unknown. Recent
completed cloud operations can prove connectivity if public probe hosts are
filtered, but stale cloud success cannot hide a new outage. Cloud credential or
permission failure with working internet is reported as cloud failure, not as
internet loss. When internet is down, cloud state is unknown to avoid redundant
cloud-down notices. Optional Firebase SDK absence is intentionally disabled.

`[HEALTH:1:UUUUU]` is exactly 16 printable bytes, with Internet, cloud, LiDAR,
bin-sensor data and thermal/power states in that order. Each state is U/D/X for
healthy/fault/unknown. Pi snapshots expire after 15 s and are rechecked after
acquiring the GATT write lock. The Pi queue coalesces health packets. The bridge
validates the exact version/length/enums at ingress and dispatch, holds the latest
snapshot in its own one-slot mailbox, drains it after safety/steering/sensors,
and rejects stale or old-session frames. The executor validates again and ACKs
without `noteBridgeRx()`, path freshness, GO evidence, readiness or motion changes.
Unknown or missing snapshots reset continuous health confirmation; silence is
not evidence of internet loss or recovery.

The executor queues sustained faults after 30 s internet, 60 s cloud, 15 s LiDAR,
30 s bin sensors, 10 s thermal/power, or 10 s established controller-link loss.
Recovery needs continuous healthy evidence for 10 s (thermal/power 30 s), and is
paired with a fault submitted to the cellular network. Local emergency-stop,
route fault and actual route events also produce notices. Synthetic ESTOP route
unwind is not a separate route fault. A reset alone cannot claim return-to-base.
Obstacle holds have ten-minute reminders, and clearing an in-flight local fault
retains a guarded recovery only if the original fault submission succeeds.

The fixed ten-slot queue coalesces categories and applies priority at both
admission and dispatch: emergency first, then route faults, then other faults,
then routine/recovery. Notices use at most 140 single-septet GSM-default
characters. Attempts are at least 20 s apart with a rolling six-attempt cap per
ten minutes; failures/retries count too. Queued notices expire at ten minutes.
An explicit pre-body failure may retry once after 30 s. Failed/uncertain
post-Ctrl-Z results and exhausted retry budgets are terminal for that episode,
including across queue replacement or X/stale health gaps. A confirmed opposite
health interval can rearm a new episode. Exact `+CMGS` reference plus final `OK`
means network submission, never confirmed handset delivery. The worker retries
initialization/registration with backoff up to five minutes.

Cellular registration, SIM SMS service, antenna and power are still required.
Complete cellular/power loss cannot produce an immediate SMS. RAM queues do not
survive a controller restart and no reboot waits for SMS. See
[SMS operation and acceptance](docs/sms-notices-2026-10-30.md) for details.

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
confirmed destination readiness (`isReadyToReturn`, and `robotState` AT_DESTINATION
when published) to enable return, and re-reads the server's status and command
immediately before writing: the Pi relays any fresh request, and one reaching the
executor mid-outbound latches a route fault. It shows the Pi's ACK, DONE and FAILED
with the failure reason. Cached sensor/status data becomes stale after 90 s, with
30 s future-clock tolerance.

Manual `[RESET]` is reserved for local supervised recovery: after a route fault,
physically reposition the robot to its known base, then issue it to clear state.
It must not be used to blindly return from an incomplete route. Correlated app
retries cannot enter this recovery branch. Deploy Pi and both ESP32s together;
old firmware cannot acknowledge the new correlated return workflow.

FCM receiving exists in the app (registered by Firebase Installation ID, published
at `devices/<id>/fcmInstallationId`), but this repository does not include a deployed
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
