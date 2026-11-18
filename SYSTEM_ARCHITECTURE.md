# GARBY system architecture

Continue run update (2026-11-14, build `CONTINUE-20261114`,
`docs/continue-run-sonar-range-2026-11-14.md`): after a confirmed emergency stop the operator
may Release (route ends at IDLE, as before) or Continue: the same confirmed release, into the
interrupted route from the executor's RAM checkpoint (`RouteResume.h`; routes are step lists in
`pointsRun.ino`). The continued route passes every motion gate again; a new S3 heading
reference since the stop needs two agreeing wall scans on the route's right-angle grid. The
obstacle sonar reads only to 110 cm, and a forward stop beyond 25 cm needs two consecutive
echoes inside 60 cm.

Turn recovery update (2026-11-14, build `RECONNECT-20261114`): BLE-session resets invalidate
path/steering freshness while retaining the executor's turn scope. After bridge boot, a
running executor's `[MCU ALIVE]` does not prove straight mode. GO is withheld until an
authoritative `[TURN:START]` or `[TURN:END]` arrives. The executor replays its complete turn
scope, including stopped holds, before status readiness; repeated START is idempotent.
An explicit controller `[MCU READY]` or confirmed operator RELEASE establishes idle mode.
Occupied, stale or absent `T=` continues to stop an interrupted turn after reconnection.

Reverse parking (2026-11-16, build `PARK-20261116`, all three parts): each route's last
step backs the robot into its parking spot (`RouteOp::BACKING`) instead of driving at it and
spinning 180 degrees beside it. Between `[BACK:START]` and `[BACK:END]` the bridge grants
`GO:BACK` only on the Pi's rear-travel code `R=C` and `B=C` (the front group is not
consulted); the executor backs only on `GO:BACK`, so an older bridge holds it. The scope is
replayed in every status report like the turn scope and is unknown after a bridge reboot
until replayed. See `docs/reverse-parking-2026-11-16.md`.

Release: 2026-10-04. Software builds and behavioral tests are documented in
`VALIDATION_RESULTS.md`; physical acceptance remains hardware-unverified.
This file describes the active source. Older audit reports are historical.

## Components and ownership

| Component | Active source | Responsibility |
| --- | --- | --- |
| Raspberry Pi | `RasPi/final_w_serial.py`, `bridge_core.py`, `ble_support.py`, `runtime_support.py` | ROS 2 LiDAR safety, wall geometry, BLE central, exclusive runtime ownership, sensor serial input, asynchronous Firebase telemetry and correlated return relay |
| BLE ESP32-S3 | `BLE_Receiver-Final/BLE_Receiver-Final.ino`, `MpuHeading.h`, `SteeringAssist.h` | BLE GATT server, path validation and STOP latch, MPU/rate feedback, confirmed LiDAR centering, controller UART transport |
| Main ESP32 | `MAIN_EXECUTOR_CODE.ino/.cpp/.h`, `pointsRun.ino` | Motor targets, local sonar protection, load/gas triggers, route state, modem/SMS worker |
| Android | [separate Android repository](https://github.com/YAKITORIHOSHI/Garby-Mobile-App) | Authenticated Firebase dashboard, confirmed manual-disposal intent, explicit return request and matching completion |

The separately maintained Android project contains no Pi code (its old compatibility launcher was
removed on 2026-11-14; run `RasPi/final_w_serial.py`). Since 2026-11-16 the deployable folders
hold only runtime files; the Pi simulator (`tests/pi/final_w_serial-simulator.py`, a manual test
producer), the tests and the bench sketches (`tests/sketches/Test/Test.ino`, serial-only) are in `tests/`.

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
7. A completed return calls `fullReset(true)`, emits `[IDLE]` then `[RETURN COMPLETE]`,
   and reports a matching DONE. Ordinary `fullReset()` does not report station arrival.
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
- Sides read-only (2026-11-16, owner's request, Pi `SIDES-20261116`): a third toggle
  (`side_disabled`, monitor "Sides", headless key `l`) makes `_side_status()` clear and
  drops the bands beside the rear half from `R=`, so something right beside the body no
  longer holds F, B or R. Zones and readings stay listed and drawn; the spin sweep `T=`
  still follows the front/back groups; missing or stale scans are still `S`. Front/back
  toggles still never bypass a side. Off when the Pi process starts. Not on the phone.
- The operator decides when GARBY moves (2026-11-16, owner; Pi `DECIDE-20261116`):
  `safety_checks_changed()` logs every read-only change, and a change while the robot is in a
  route state latches the operator stop (`pause_route_for_operator`). The first known state
  of a Pi process that is a route state does the same, so a route left running by an earlier
  monitor never drives on by itself when the LiDAR clears it; a reconnect within one process
  does not. The reason is shown above the monitor's tabs and as the phone's operator message;
  Continue run, Reverse to start and Release stop are unchanged.
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
| Rear sectors | 35 cm, shown only: forward segments never hold on them (2026-11-14) |
| Rear box (reverse parking only, `R=`) | 35 cm behind the rear edge, corridor width (2026-11-16) |
| Side hard envelope | 22 cm, blocks both travel directions |

Chassis zones (`RasPi/hallway_geometry.py`, 2026-11-11) are evaluated on the same
scan and only add evidence. Frame: x forward from the LiDAR, y left from the
centreline; outline from `GARBY_CHASSIS_*` (default 60 x 60 cm, LiDAR centred,
unmeasured). Returns inside the outline are treated as the robot itself.

| Zone | Region | Effect |
| --- | --- | --- |
| Front box | ahead of the front edge to 95 cm, body width + 10 cm each side | `F=O` (front-group bypass applies) |
| Slow zone | same corridor, 95-160 cm | `F=W` -> `GO:SLOW` (caution speed) |
| Side band | beside the body from the spin centre to the front edge, within 2 cm of its sides | both directions blocked, never bypassed; the rear half is left behind by a forward move (2026-11-14) |
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
things already abeam or moving off are not contacts. The LiDAR on top of the bin sees
torsos, nothing below its plane. Logged as
`Moving-object prediction: ...`; zones `MOVING_STOP`, `MOVING_SLOW`, `MOVING_TURN` in the footer.

Since 2026-11-14 (`docs/objects-station-reverse-2026-11-14.md`):

- The scan is converted to robot-frame points once (`RasPi/scan_frame.py`) for the zones and
  walls, the tracker and the hallway scene. The eight safety sectors keep their own pass.
- The robot's own **rotation** is removed: the wall heading `H` of each scan turns the tracks
  back about the spin centre, trail positions are kept in a ground frame, and sideways motion
  that a rotation rate of 1.5 deg/s (walls in view) or 8 deg/s (no walls, or `H` jumped more
  than 3 degrees between scans) could explain is not counted as movement. Radial motion is
  never explained by rotation.
- A track is a **likely person** when it is moving, 15-75 cm across, not flat (a panel 35 cm
  or wider within 4 m lying within 1.2 cm of a line in 60 % of its sightings is flat), at
  0.25-4.5 m/s, with 40 cm of net ground travel over at least 0.9 s and five sightings. The
  label is latched for the track's life and is display/report only.

### Hallway objects (2026-11-14)

`RasPi/scene_objects.py` classes every return of a scan against that scan's own wall fits:
wall (within 6 cm of a wall line), beyond the wall, or interior. Interior returns are grouped
in ray order into objects with distance ahead, lateral offset, width and free width to each
wall; a group within 25 cm of a wall reaching in at most 45 cm is a fixture, one spanning 70 %
of the hallway width its end wall. Objects are reported only in a real hallway (both walls
fitted, 120-450 cm apart). `SceneMonitor` asks for `F=W` (zone `OBJECT_AHEAD`) when an object
overlaps the swept corridor between the slow zone (160 cm) and 400 cm for three consecutive
scans, held 1 s; the read-only front bypass removes it. `scene_markers()` joins objects and
tracks into the map markers (`person`, `moving`, `object`) carried on the `LidarMapSnapshot`.

### Station steps (2026-11-14)

`RasPi/station_mode.py` holds the executor's latest `[LEG:..]` notice. While it names a step
flagged `STEP_STATION`, is under 3.5 s old, the link is up and the robot state is a route, the
scan callback takes the station's wall returns out before any consumer runs: those rays read
`+inf` (a valid clear ray for the sectors) and leave the converted frame. A wall return is a
point of a straight piece at least 60 cm long (split at corners over 5 cm, line RMS at most
2.5 cm), of a straight piece at least 10 cm long joined to such a wall within 150 cm (no
return more than 0.8 cm off its line, up to three rounded-corner returns at each end), or of
a leftover of at most 8 cm between two wall pieces. The map still draws every return.
`GARBY_STATION_WALL_MIN_GAP_CM` (default 0) keeps a wall that close ahead of the front edge.

### Reverse parking (2026-11-16)

Both routes end by backing into their parking spot (`RouteOp::BACKING`, op `K` in the leg
notice): the last turn faces away from the spot at the same place, and the robot reverses
the travel it used to drive forward (outbound 11100 steps, return 7500), at
`BACKING_SPEED` (2000 steps/s, ~25 cm/s), both wheels the same remaining steps. Travel
counts remain calibrated; parking orientation comes from the rear wall as follows.

The Pi fits the original rear returns before station masking (`RasPi/rear_wall.py`),
using the existing mount rotation/reflection. A rear plane must span at least 80 cm
(or chassis width + 20 cm), cross the centreline with support on both rear halves,
and have continuous central support. Side-wall `H` and its learned mounting correction
do not enter this fit. `RD`/`RH` in S become checksummed `D` lines carrying the same
acquisition and total source age. Unavailable fits explicitly withdraw old rear evidence;
duplicates cannot refresh it. A new Pi session resets rear scan numbering in the executor
without resetting the continuous heading.

Before any backing travel, two stationary scans must agree within 1 degree and 3 cm;
alignment must be within 1 degree. A correction is at most 8 degrees and uses full
turn clearance/sonar plus MPU supervision, with another fresh pair AFTER the turn gate
and after the correction. At most two completed corrections are tried; an unconfirmed
angle holds in low power rather than repeatedly spinning. The final approach turn
skips side-wall alignment on both routes. During backing, missing/stale rear fits or
more than 2 degrees of rear-relative drift brakes the robot and rechecks it outside
the backing scope, preserving remaining mean travel. There is no final gyro-only spin.
The gyro timestamps/supervises movement; the rear plane chooses the parking direction.

The Pi computes the rear-travel code `R=` for every scan, whatever the robot is doing:
`S` without a current scan; `O` for side clearance, a return within 2 cm beside the rear
half (rear edge to spin centre, never bypassed), a return in the corridor-wide box up to
`backing_stop_cm` (35 cm) behind the rear edge, or a mover predicted to reach the body
(while backing, a mover catching up from behind counts); `C` otherwise. A read-only back
group bypasses the box and the mover. Returns inside the body outline are the robot itself.
The back sectors' 35 cm limit is not part of `R=`. Station mode applies on the return's
backing step (it is a station step), so the station wall behind the robot does not hold it;
the calibrated step count decides where it stops, as the forward approach did.

The bridge holds the backing scope like the turn scope: `R=C` and `B=C` give `GO:BACK`;
`F` is not consulted; a packet without `R` is `STOP:BACK_UNKNOWN`. The executor counts only
`GO:BACK` while backing, and never counts `GO:BACK` outside it. While backing the sonar
does not hold or slow the move (it faces away); nothing low behind the robot is seen.

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
P:<uint32-seq>|F=<C|W|O|H|S>|B=<C|O|H|S>|T=<C|O|S>|R=<C|O|S>
   (W: clear to the stop, caution zone ahead; T: spin-sweep clearance, gated only in turns;
    R: rear-travel clearance, gated only while backing: rear box, bands beside the rear
    half, side clearance, a mover predicted to reach the body; 2026-11-16)
S:<same-seq>|L=..|R=..|F=..|B=..|FL=..|FR=..|BL=..|BR=..|CF=..|CL=..|CR=..|A=<uint32>[|WL=..][|WR=..][|H=..][|RD=..|RH=..]|G=<ms>|T=..
   (WL/WR: wall distances from the centreline, cm; H: heading vs walls, deg, + = left;
    G: scan age refreshed immediately before the Pi's GATT write, ms; T: legacy tilt, ignored)
S:<same-seq>|STABLE
SENSOR:US=..|MQ4=..|MQ137=..|MQ135=..
[HEALTH:1:UUUUU]
[RASPI READY]
[RESET:<requestedAt-ms>]
[START:<request-id-ms>]
[START STATUS:<request-id-ms>]
[OP:<id>:ESTOP|RELEASE|CONTINUE|REVERSE|SONAR_OFF|SONAR_ON|REBOOT]
[OP STATUS]

Bridge -> main, newline-delimited UART at 115200 baud:
STOP[:reason]            (reasons include MPU, TURN, TURN_UNKNOWN, BACK, BACK_UNKNOWN, STALE, LINK, OPERATOR, ...)
GO | GO:SLOW | GO:BACK   (GO:BACK only between [BACK:START]/[BACK:END]; the only GO a backing move counts)
M,<seq>,<heading cdeg>,<rate cdeg/s>,<epoch>,<allow bits>*<xor hex>   (50 Hz, no ACK)
MPU:FAULT                                                            (no valid heading)
W,<acq>,<age ms>,<flags>,<left mm>,<right mm>,<H cdeg>*<xor hex>      (per LiDAR scan, no ACK)
D,<acq>,<age ms>,<valid 0|1>,<rear mm>,<rear H cdeg>*<xor hex>       (rear alignment only, no ACK; never GO)
SENSOR:...
[HEALTH:1:UUUUU]
[BLE CONNECTION ESTABLISHED]
[ROBOT STATUS]
[RESET:<requestedAt-ms>]
[START:<request-id-ms>]
[START STATUS:<request-id-ms>]
[OP:<id>:ESTOP|RELEASE|CONTINUE|REVERSE|SONAR_OFF|SONAR_ON|REBOOT]
[OP STATUS]

Main -> bridge only (local UART, not relayed to the Pi):
[HEADING:START:<route error cdeg>] / [HEADING:END]   (straight segment; logged, never re-zeroes)
[TURN:START] / [TURN:END]   (bridge requires T=C and fresh confirmations in between)
[BACK:START] / [BACK:END]   (reverse parking: bridge requires R=C and B=C, ignores F, sends GO:BACK)
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
[OP STATUS:1:ESTOP=<0|1>:SONAR=<0|1>:RESUME=<0|1>:REV=<0|1>]   (RESUME and REV since 2026-11-14; the Pi also accepts them absent)
[ROUTE CONTINUE:<step>/<steps>]   (a continued route's first step)
[ROUTE REVERSE:<step>/<steps>]    (Reverse to start: the step the way back begins in)
[REVERSING] / [REVERSE COMPLETE]  (executor state while driving back; its end)
[LEG:<O|R><F|B>:<step>/<steps>:<B|S|H|L|R|K>:<flags>] / [LEG:0]
                                  (route step in progress, every 2 s; K = backing into the parking
                                   spot; flags bit0 = station step, bit1/bit2 = station wall ahead
                                   forward / on the way back)
[HEADING BACK steps=<n> err=<deg> mpu=<0|1>]   (after a backing move; a spin squares it up
                                   if the error is over 1 degree)
[MOTION:<CLEAR|WAITING_FOR_PATH|CONFIRMING_CLEARANCE|LOCAL_OBSTACLE|MOTORS_UNAVAILABLE|ROUTE_FAULT|STOP[:reason]>]
[BRIDGE MPU VALID|INVALID bias=.. tilt=.. refresh=.. epoch=.. walls=..]
[HEADING TURN ...], [HEADING STRAIGHT ...], [HEADING FUSE ...], [HEADING LOST ...], [HEADING ROUTE:...],
[HEADING CONTINUE verify=...]
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
selected distance thresholds while retaining readings; since 2026-11-16 a sides
read-only does the same for the side checks. Stale-data, MPU,
nearest steering-clearance, emergency-stop and transport gates remain active, and side
checks stay on unless the sides group itself is read only.

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

## Finite developer tests (2026-11-15)

The owner explicitly requested sensor isolation and short motor tests. The Pi monitor's
Developer tests window (a tab until 2026-11-16; its own window since, opened from the main
header, with its own emergency stop; closing it during a test cancels the test, whose lease
then lapses within 800 ms) sends one confirmed test, separate from a route. No persistent bypass
setting is introduced. Current source identifies as `TURNSCALE-20261116` on Pi and both
controllers, including the 90-degree test extension below. The base panel needs fresh test
readiness; older controllers reject the new 90-degree amount and need the matching update.

```text
Pi -> bridge -> executor:
[TEST:<id>:<I|F|B|1|2|L|R|W|S|A|D>:<amount>:<mask>]
[TEST KEEP:<same-id>]
[TEST END:<same-id>]                 (remote control: the arrow was let go)

Executor -> Pi via bridge:
[TEST READY:1]
[TESTING]
[TEST ACK:<id>]
[TEST DATA:M=<0|1>:Y=<heading-cdeg>:W=<0|1>:H=<wall-cdeg>]
[TEST RESULT:<id>:D1=<steps>:D2=<steps>:M=<0|1>:Y=<change-cdeg>]
[TEST DONE:<id>] | [TEST FAILED:<id>:<reason>]
```

Mask bits select LiDAR clearance (1), MPU supervision (2) and obstacle sonar (4), initially
all selected (7). `I` inspects sensors without movement (amount 0). `F` requests at most
400 forward steps; `1` and `2` at most 100 steps on that motor; `L` and `R` accept 1..15
MPU-measured degrees or exactly 90 and require MPU selected. The monitor offers 400/100/10,
plus separate **Turn left 90°** / **Turn right 90°** buttons (owner, 2026-11-16).
The finite-test speed setting is 2351 steps/s, nominally 30 cm/s from 1600 steps/revolution
and 65 mm wheels, rounded up. Ramps, first-turn motor-side checks and endpoint braking
can be slower; short moves may not reach that speed. Ordinary finite tests have an absolute
eight seconds, including waits; the explicit 90-degree tests retain 30 seconds for readiness
and settling. The Pi, bridge and executor keep that same
scope; every finite test still needs its 800 ms KEEP lease. Limits never extend routes.
Measured-turn confirmations require GARBY on the floor with both wheels down. If the new
90-degree test's fresh MPU angle changes less than 0.5 degree while wheel counts exceed
three expected degrees plus 300 steps, it ends as `NO_ROTATION`. It cannot prove why the
chassis did not turn; the monitor explains lifted wheels and also checks on-floor grip/MPU.
Missing/lost MPU keeps its own hold/failure and connection advice. See
`docs/developer-90-turns-2026-11-16.md`.
Step geometry and physical travel remain unverified. These short tests use the production
motor targets and MPU turn code without hallway wall alignment. Raw wall-angle telemetry
does not require a valid gyro, allowing the two measurements to be examined independently.

`B` (2026-11-16, `BACKTEST-20261116`) requests at most 400 backward steps on both wheels, the
same bound as `F`. The executor runs it inside the reverse-parking scope (`[BACK:START]` ...
`[BACK:END]`), so with LiDAR selected only `GO:BACK` (the Pi's `R=C` with `B=C`) moves it and
a plain GO holds it; a hold names `HELD_REAR` when the rear check caused it. The forward sonar
faces away and does not hold it, as in parking. It does no rear-wall alignment (spins need
ground contact). The monitor's Live line also shows the Pi's latest rear-wall fit (`RD/RH`,
display only, at most 1 s old). With LiDAR deselected, a GO of the wrong kind is ignored like a
STOP; only the lease counts. See `docs/backward-test-2026-11-16.md`.

Remote control (2026-11-16, `REMOTE-20261116`, owner's choices): `W` forward, `S` backward,
`A` spin left, `D` spin right, amount = speed 2351..4702 steps/s (the monitor sends 2351 or 4702,
nominally 30 or 60 cm/s wheel speed),
mask must be 0. The drive runs while held: no step value, no time limit, no LiDAR, MPU or sonar
gate. The Pi renews it every 120 ms only while the window reports the arrow still pressed
(every 100 ms; 0.5 s without that stops the renewals); the executor's lease for these is
500 ms. Letting go sends `[TEST END:<id>]`: the bridge relays it for the last held drive's ID
whatever its own record, lease, age or a latched stop, and the executor brakes
(`activeBrakeStopMotors`) and reports DONE. Left/Right use the current left-wheel guess
(the first MPU-measured turn confirms or swaps it). See `docs/remote-control-2026-11-16.md`.

RunStart (2026-11-16, `RUNSTART-20261116B`, owner's request): the Developer window's own list
of steps, driven in order with no LiDAR, MPU or sonar gate and no time limit.

```text
Pi -> bridge -> executor:
[TEST:<id>:P:<speed 2351..8000>:<accel 100..20000>:<op><steps>,...]   op F|B|L|R|C, 1..100000, 1..20 steps
[TEST:<id>:C:<saved-id>]     Continue run: from where the saved list stands to its end
[TEST:<id>:U:<saved-id>]     Reverse to start: back to where the list began
[TEST KEEP:<same-id>]        every 200 ms; executor lease 800 ms, no time limit
[RUN CLEAR:<saved-id>]       Discard: forget the saved list (never moves anything)

Executor -> Pi via bridge (besides TEST ACK/RESULT/DONE/FAILED):
[RUN AT:<id>:<step>:<steps done in it>]                    every 250 ms and at each step
[RUN SAVED:<id>:<step>:<done>:<speed>:<accel>:<program>] | [RUN SAVED:0]   in every status
```

`F`/`B` move both wheels, `L`/`R` spin in place (left wheel back and right wheel forward for
`L`, by the current left-wheel guess), `C` goes forward in 100-step bumps with 300 ms pauses;
250 ms between steps. Every step's wheel targets are fixed when the list starts, relative to
the wheel positions then, so wherever a stop leaves it the executor knows the exact place:
`C` moves on to the end, `U` undoes the current step and then each earlier one in reverse
order (the same wheel steps the other way; bumps are undone in one move). The list stays saved
after it finishes or stops (an emergency stop, its lease, a link/protocol STOP, a reset) until
a new list, any other motion (a bench test, the remote, any route) or `RUN CLEAR` replaces it;
an executor restart loses it. `C`/`U` must name the saved list and, like every test, need
IDLE without a latched stop: the monitor's Continue/Reverse buttons release the emergency stop
first through the operator gate (the same confirmed release), then send the request. The
bridge relays the operator stop as `STOP:OPERATOR` just before `[OP:..:ESTOP]`; since this
build a test ended by it reports `EMERGENCY_STOP` (was `LINK_OR_STOP`). The bridge treats a
list as a developer test with mask 0 and no eight-second limit (KEEP age, session and stop
generation still bound it; no gyro-bias refresh meanwhile) and relays `RUN CLEAR` on its
control lane. See `docs/runstart-2026-11-16.md`.

The executor accepts a test only while stationary IDLE, ready, without stop, route fault,
pending reset or parked route. It reports TESTING so ordinary starts/configuration cannot
overlap it. The selected overrides apply only inside that test's stack; omitted LiDAR
clearance includes sides and stale/missing scans for this test. Operator emergency stop,
link/protocol failures, motor-command rejection, the eight-second limit and a live test
lease remain authoritative. MPU faults still hold a test that selected MPU feedback.
Successful completion restores normal path evidence and re-arms bypassed sonar; another
deliberate start is required. No route checkpoint or automatic continuation is created.

The Pi reserves durable test IDs separately from operator IDs, checks a two-second send
deadline again at the serialized GATT writer, attempts TEST once, and never retries an
uncertain movement request. KEEP is sent every 200 ms after that attempt; only a matching
currently pending/executing test can consume it. A missing ACK expires at five seconds.
Disconnect/restart/emergency stop ends the Pi lease. Late ACK, KEEP and duplicate/reordered
TEST cannot revive or replay travel. The executor requires KEEP within 800 ms. The bridge
also rejects KEEP delayed over 300 ms at ingress, scopes its optional MPU veto bypass to
the acknowledged matching test and fresh lease, and clears that scope on terminal/session/
stop changes. None of these messages creates ordinary route GO evidence.

The bridge inhibits stationary gyro-bias refresh throughout a pending/active test: an ignored
LiDAR STOP cannot prove its wheels are held. Normal readiness, path/turn-sweep confirmation,
nearest steering clearance and continuous heading behavior remain unchanged for routes.
Independent readings and valid result telemetry are diagnostics; software step counters
cannot prove that a motor physically moved. The normal motion-hold banner is reserved for
normal routes; the developer panel reports the test's own acknowledgement/result.

The latest hardware identification from the owner is ESP32-S3 N16R8 (16 MB flash, 8 MB octal
PSRAM) for the bridge; earlier 8 MB/no-PSRAM build examples are superseded for this board.
Procedure and evidence: `docs/developer-tests-start-hold-2026-11-15.md`.

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
| Turn | MPU target with immediate wheel-queue abort, tolerance 1°, ≤5 passes, wrong-way swap once, step limit 1.5x |
| Turn wall alignment | 2 scans agreeing within 1.5°, within 8° of command, tolerance 0.75°, ≤2 passes |
| Hallway axis filter | alpha 0.15 / beta 0.01 per scan, gate 4°, ≤15° from the route frame |
| Lateral | gain 0.5, gate 15 cm, dead reckoning ≤8 s between wall measurements |
| Centring | deadband 2 cm, engage 5 / release 2 cm, -atan(excess / 120 cm), ≤5°, 4°/s slew |
| Inner heading hold | 2.5/s on error + 0.28 s rate lead, ≤30% one-wheel cut, 1.5%/20 ms |
| Steering permission | none if CF/F ≤60 cm or a wall median ≤22 cm; per side ≤40 cm (CL/L/FL/BL) |

The executor anchors a route frame at route start (it refuses to start without a
confirmed heading stream). Each turn adds its angle, is driven to it by the gyro,
then checked against the walls; the measured hallway direction becomes the frame.
At the target, `spinToward()` immediately aborts both wheel queues using
`forceStopAndNewPosition(currentPosition)` through `emergencyStopMotors()`
(`TURNSTOP-20261116`, 2026-11-16). The endpoint uses the existing estimated heading
(MPU sample plus intervening wheel travel and sample-latency compensation), with
no early 0.3-degree stop allowance. The approach still slows before the target;
the 250 ms settled heading check and wall confirmation run with the wheels stopped.
Successful completion does not latch an operator stop. This shared endpoint also
serves measured correction spins; step-count RunStart turns use their own moves.
It removes commanded deceleration run-out, not MPU error, slip or physical inertia.
`TURNSCALE-20261116` applies the owner's approximate 100° physical / 90° reported
observation as `GYRO_SCALE = 100/90` in the bridge's MPU rate and integration. Targets
remain 90°; the same correction reaches every consumer through M lines and must not
be applied a second time in the executor. It is provisional pending measured floor
acceptance, and affects route heading as well as developer turns. Stationary drift
refresh still has its 0.2°/s per-window and 1°/s total bounds in corrected units.
Developer measured turns additionally require at least 200 ms of fresh samples with
rate at most 1°/s and heading change at most 0.2°, after a minimum 250 ms settling
period; missing data or continued rotation fails the test. Final error must be within
1°, including on the last allowed pass. Their approach profile updates at most 50 Hz;
stop, heading and lease checks continue every loop. Route settling/hold rules remain
as above. See `docs/developer-turn-speed-2026-11-17.md`.
The final parking approach is the exception: rear-wall confirmation owns its
alignment, as described in Reverse parking above.
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
`tests/firmware/test_navigation.cpp` (kinematic, not physical).

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
| Sonar range / echo listening window | 110 cm / 6613 µs (range + 200 µs; was 18000 µs, roughly 3.1 m); a further echo is clear |
| Ping period (forward only) | 40 ms |
| Fresh forward sample age | 600 ms (was 180 ms, forward only) |
| Slowdown / full-speed resume | 90 / 105 cm (forward, or ±25 bearing inside the corridor) |
| Forward close echo STOP | 2 consecutive ≤60 cm forward echoes (one raw echo until 2026-11-14) |
| Persistent obstacle confirmation | 2 blocked echoes (forward, or per side bearing) |
| Immediate emergency latch | ≤25 cm |
| Clear threshold / confirmation | ≥72 cm / 3 forward samples; 2 per side bearing |
| Spin circle (inside a turn) | 50 cm radius from the spin centre, surveyed before each spin |

An echo is placed at the beam edge nearest the path (±12°); side bearings count only if they
can be inside the 60 cm body + 8 cm corridor. A timeout/no echo means nothing within 110 cm (~3 m until 2026-11-14) and
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
(`TURNTEST-20261116` for the executor) and both DIR values. Which motor is the left wheel is a guess
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
The reached heading target of a measured turn uses an immediate force stop, as
described above; obstacle holds and heading-loss/wrong-way/step-limit exits keep
their existing stop paths.

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

### Completed-route location telemetry (2026-11-17)

Executor/Pi build `ARRIVAL-20261117` records route endpoints in
`devices/<id>/status`. `[OUTBOUND COMPLETE]` sets `location = POINT_B` (disposal area),
and `[RETURN COMPLETE]` received at IDLE sets `location = POINT_A` and `isAtStation = true`.
Boot, ordinary IDLE, stop/release, RESET ACK/DONE, faults and developer-test completion
cannot establish arrival at the station. The executor's RAM flag retains return completion
for `[ROBOT STATUS]` and reconnect replay, and is cleared before a new route/test/reverse
motion, by an ordinary reset or by controller boot. Continued returns use the same success
path and report arrival only after the remaining steps finish.

`locationUpdatedAt` is the Pi's confirmation time in epoch milliseconds. `lastArrival` is
`{point, name, confirmedAt, source}`, where source is `OUTBOUND_COMPLETE` or `RETURN_COMPLETE`.
Repeated unchanged status does not refresh its confirmation time. Motion states set current
location to `IN_TRANSIT`; faults/link uncertainty set it to `UNKNOWN`; IDLE preserves an
already confirmed stationary endpoint. `lastArrival` remains historical telemetry across
later motion, faults and process restart. All changes use the existing asynchronous coalesced
root updater and retries, never the command or GO paths. Neither wheel steps nor this record
prove physical position, slip-free travel or parking accuracy.

The bridge's generic UART-to-BLE forwarding already carries `[RETURN COMPLETE]`; no bridge
source/firmware change is required for this telemetry addition. See
`docs/route-arrival-2026-11-17.md` for deployment, software evidence and physical limits.

### Phone control and the map (2026-11-15)

`RasPi/cloud_control.py` and `firebase_control_loop` (one thread, `firebase-phone-control`).
Once a second it reads `devices/<id>/commands` and hands `commands/operator` to
`OperatorRelay`: validate (version 1, 32-hex request ID, uid, server timestamp, a known
action), apply an emergency stop at once, claim by transaction, check the age again with the
time the claim took (a monotonic clock, so a slow transaction cannot act on an expired
request), then call `cloud_request_operator_action`, which is `operator_control.request` with
the monitor's own gates. The executor's `[OP ACK]`/`[OP FAILED]` for that request ID
(`OperatorControl.outcome`) finishes the cloud request as `accepted` or `failed`; 8 s without
one is `unconfirmed`, and so is a request found `claimed` by a process that no longer runs.
Front/back/side read-only requests set the same events as the monitor's toggles, only when
`ALLOW_RUNTIME_SAFETY_DISABLE` is true. Developer requests (`DEV_*`, 2026-11-18) go to
`cloud_developer_action`, which calls the Developer window's own functions and finishes the
request `accepted` when the Pi took it, or `failed` with the window's "Not available" reason.
Bench tests carry the request's `sensors` mask; phone arrows are Pi-timed nudges
(`PHONE_NUDGE_S`); `DEV_CANCEL` is `DeveloperTestControl.stop()`; RunStart Continue/Reverse
release a latched stop first, as the window's buttons do; `DEV_RUN_START` carries the phone's
list (`program` "F4000,L980", `speed`, `accel`), read by `parse_run_program`, checked by
`run_program` and refused while a list stands part-way, then sent by `request_run`. How each one went follows in the
mirror's `developerMessage`/`developerResult`/`developerRun`
(`docs/phone-controls-rundown-2026-11-18.md`).

`build_operator_mirror()` writes `devices/<id>/operator` on change and every 10 s, and the
same document to the status file `run_headless.sh status` reads. While
`commands/map.leaseAt` is under 45 s old, `LidarDistanceReader.cloud_map_frame()` encodes the
current scan (before any station wall is taken out) and its markers to
`devices/<id>/map/frame`, at most every 0.5 s (1 s on a hot Pi), never from a scan older than
0.8 s. `RasPi/map_frame_golden.json` is also the Android decoder's test input. Details, rules
and first-use steps: `docs/phone-control-headless-2026-11-15.md`.

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
