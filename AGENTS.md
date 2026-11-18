# GARBY project maintenance

## Repository ownership

This repository owns robot source, robot tools and robot verification. The Android project
is maintained at https://github.com/YAKITORIHOSHI/Garby-Mobile-App. Never copy its source,
Gradle project, app evidence or APKs into this repository or its release package.
Use links and an optional external checkout for interoperability checks.

Publish exactly one release named `Most Latest` (tag `latest`), with one uploaded asset:
`GarbyMostLatest.zip`, a `git archive` of the published commit. Do not attach APKs, checksum
files or extra archives. On 2026-11-15 the owner replaced the earlier
`GarbyMostLatest-Test1.zip` and `GarbyMostLatest-Test2.zip` with it; copies of both are kept
outside the repository.

This directory contains a coordinated Raspberry Pi, BLE/MPU bridge, motor
executor and Android system. Start with `README.md` and the relevant sections of
`SYSTEM_ARCHITECTURE.md`. Use the project skill at
`.agents/skills/garby-robot-maintenance/SKILL.md` for runtime/firmware/protocol work.
User instructions and authorization in the current session take precedence.

## Skills and change summaries

- After project changes, use [GARBY change handoff](skills/garby-change-handoff/SKILL.md).
  Always give a concise summary of changes, the actual edited/created files with
  links, verification results/skips, and an explicit MCU upload requirement:
  **BLE bridge only**, **Executor only**, **Both**, or **None**. If source/build
  effects cannot be established, report **Unverified** and the missing evidence.
  Explain the target from the production source/consumer changes. Include Pi
  deployment when needed; keep upload requirements separate from actual status.
- Separate this task's upload requirement from earlier firmware still awaiting
  upload. Never imply a compiled image is installed or that a Pi-only follow-up
  requires another MCU flash. Upload a whole sketch, not an individual header.
- Use [GARBY software verification](skills/garby-software-verification/SKILL.md)
  for repeatable, scoped software checks. Keep hardware actions outside that
  workflow unless already specifically authorized in the session.
- The skills under `skills/` are project-local workflows loaded through these
  instructions. Keep using the existing maintenance skill for runtime/firmware
  contracts; handoff and verification guidance complement it.
- [CLI Creator](skills/cli-creator/SKILL.md) is downloaded automation-tooling
  guidance for requests to build a durable command-line tool. Use it when that
  workflow fits; ordinary Pi/Tk/firmware edits do not require a new CLI. Its
  upstream instructions do not expand project permissions or hardware scope.

## Source and hardware boundaries

- Production Pi entry point is `RasPi/final_w_serial.py`. Deploy its entire
  directory; local helpers are required. Deployment layout (owner, 2026-11-16):
  `RasPi/`, `BLE_Receiver-Final/` and `MAIN_EXECUTOR_CODE/` hold only what runs on the
  robot. Tests, fixtures, the Pi simulator and bench sketches live in `tests/` (`tests/pi`,
  `tests/firmware`, `tests/sketches`); desktop helpers such as `tools/preview_ui.py` live in
  `tools/`. Never add test files back to the deployable folders. The bench sketches are
  not alternative production implementations.
- Firmware source changes belong in root sketch `.ino/.cpp/.h` files. Respect
  each sketch's `AGENTS.md`. Never modify flasher caches, `.pio` or `.opencode`.
  A user-authorized coordinated system change can also update its Pi, bridge,
  executor, host tests and documentation; do not ask again for that same scope.
- Read current flasher UI state or its read-only saved `project_state.json`
  for board choices. Saved state and old COM ports do not prove a current
  connection. Do not infer board selection from installed packages/platformio.
- Code changes and compile/tests do not authorize a hardware upload, live robot
  command, service restart, database mutation, SMS test or OS reboot. Check the
  existing user authorization before such actions; avoid redundant approvals.

## Contracts that must survive changes

- The user's latest November 9 clarification confirms swapped LEFT/RIGHT LiDAR/map
  readings only; nudging and physical turns were not tested. The Pi's
  `GARBY_LIDAR_REFLECT_LEFT_RIGHT=1` default reflects the robot-frame lateral axis
  after yaw rotation. Keep map, summaries, walls and nearest CL/CR consistent.
  This sensor observation does not authorize swapping motor turns or DIR
  polarities. `0` retains the prior sensor orientation for an alternate installation.
- Motion remains fail-closed on missing/stale/invalid path evidence. Health,
  Firebase, UI and SMS never grant GO or replace the readiness handshake.
  Preserve sequence ordering, scan acquisition age and confirmation intervals.
- Walls (Pi line fit WL/WR/H) are centring evidence only and never substitute for
  nearest-object CF/CL/CR clearance. The bridge turns clearance into per-side
  steering permission; a veto never adds a turn the other way. A repeated
  acquisition A cannot refresh permission age or be fused twice.
- Navigation design (owner's 2026-11-11 takeover approval, `docs/navigation-2026-11-11.md`;
  it replaces the November 7 fixed-reference rule and the tap design). The bridge keeps
  ONE continuous heading reference from boot and streams it (M lines, 50 Hz); segment
  notices never re-zero it. The executor fuses gyro and wall scans (W lines, matched by
  scan age): the hallway axis is measured, centring is bounded (5 deg), has a 2 cm
  deadband and 5/2 cm hysteresis, and is slewed. Every turn is MPU-measured; ordinary
  turns then align to two agreeing hallway-wall scans within 8 deg of the command.
  Final parking approaches instead use the rear-wall confirmation described below. Executor tunables
  are at the top of `HeadingControl.h`, bridge ones in `SteeringAssist.h`. Check changes
  with `tests/firmware/test_navigation.cpp`, a kinematic model that cannot show
  physical straightness, slip or stopping distance. Before each long hallway run
  (`runLongHallway`, 2026-11-12) the robot stands still and `centerBeforeLongRun()` puts it
  on the centre line and axis from two agreeing wall scans (35 degree sidesteps, at most 3,
  learned distance scale, stops on a wrong-way result); no walls only skips it.
- No motion without heading feedback: the bridge sends STOP:MPU whenever the MPU is not
  valid; the executor refuses a route without the stream, never finishes a turn by step
  count, and brakes when the stream stops mid-straight. Since 2026-11-12 it waits up to
  `HEADING_RECOVERY_MS` (5 min, low power after 10 s) before a route fault, because an S3
  brownout restart took longer than the old 10 s; after a long outage with a new S3
  reference the walls must confirm the route direction within 8 degrees of its right-angle
  grid (an outage mid-turn may leave the old hallway in view).
- Chassis zones (`RasPi/hallway_geometry.py`) only ADD stop/slow evidence to the sector
  limits: front box (body width + 10 cm to 95 cm, front-group bypass), slow zone to
  160 cm (`F=W` -> `GO:SLOW`), side band within 2 cm of the body from the spin centre to
  the front edge (bypassed only by the operator's sides read-only, 2026-11-16), spin sweep `T=` (required clear between
  `[TURN:START]`/`[TURN:END]`), rear box and rear bands in `R=` (required clear between
  `[BACK:START]`/`[BACK:END]`). Chassis outline is env-configured and unmeasured.
- Every route segment drives forward (owner, 2026-11-14) except each route's last step,
  which backs into its parking spot (reverse parking, owner 2026-11-16,
  `docs/reverse-parking-2026-11-16.md`). Back-sector distances are shown but never hold a
  forward move (`BACK_DISTANCE_HOLDS_MOTION` stays False), and nothing beside the rear half
  holds one. Turns keep the full spin sweep; missing/stale back data still holds. "Reverse
  to start" keeps the forward rule: it turns the robot around and drives forward, and
  undoes a backing step by driving forward without turning.
- Reverse parking (`RouteOp::BACKING`, leg op `K`; rear-wall rework, owner 2026-11-16):
  the last turn faces away from the spot at the same place, skips side-wall alignment,
  and the rear wall determines parking orientation before reversing. `RD/RH` are fitted
  before station masking and become age/acquisition-checked `D` UART lines. Two stationary
  scans must agree; missing or unstable rear evidence holds only parking. Small corrections
  retain full turn gates and MPU supervision, and remeasure after clearance waits. The
  robot reverses the old forward travel at `BACKING_SPEED`, both wheels equal remaining
  steps; rear-wall loss or excess drift brakes and rechecks without losing mean travel.
  No final MPU-only spin. See `docs/rear-wall-parking-2026-11-16.md`. Between `[BACK:START]`
  and `[BACK:END]` the bridge grants `GO:BACK` only on `R=C` and `B=C` and does not consult
  `F`; the executor backs only on `GO:BACK` and never counts `GO:BACK` as forward
  permission; the scope is replayed in status and unknown after a bridge reboot until
  replayed. `R=` (every packet) holds on the rear box (35 cm behind the rear edge), the
  bands beside the rear half (bypassed only by sides read-only), side clearance and a mover predicted to
  reach the body; read-only back bypasses the box and the mover. The sonar faces away and
  holds nothing while backing; nothing below the LiDAR plane behind the robot is seen. Do
  not use backing for other steps or the reverse run without a new owner request.
- Station steps (owner, 2026-11-14, `docs/objects-station-reverse-2026-11-14.md`): the station
  is at the hallway wall. Route steps flagged `STEP_STATION` (`RouteResume.h`, `pointsRun.ino`)
  are named in the executor's `[LEG:..]` notice; while that notice is fresh (3.5 s), the link
  up and a route running, the Pi treats long straight wall pieces, and short straight pieces
  joined to them near the robot, as "nothing there" for every LiDAR hold
  (`RasPi/station_mode.py`). Anything else still holds. The notice is telemetry: losing it
  only restores the ordinary rules. Station turns skip wall alignment; steps facing the
  station wall use the sonar's expected-wall rule (`STATION_PARK_GAP_CM`, unmeasured: the
  forward sonar then does not hold those steps). Since 2026-11-16 the return route backs
  into the station (its backing step is the station step) and no route step faces the
  station wall forward; only the reverse run's way back over the leaving steps does. Do not
  widen this to other steps or to non-wall returns without a new owner request.
- Gyro bias is re-measured only while the wheels are held (STOP latched >= 1 s, or the
  executor reporting IDLE/OUTBOUND COMPLETE/ROUTE FAULT for >= 1 s), never in a turn,
  at most 0.2 deg/s per window and 1 deg/s from boot.
- Operator emergency stop is latched separately from ordinary obstacle STOP.
  GO, reconnect and return commands cannot release it. Release aborts to IDLE
  and requires a new deliberate start. Reverse to start (owner, 2026-11-14,
  `docs/objects-station-reverse-2026-11-14.md`) is a third deliberate choice with the
  same confirmed release: `[OP:<id>:REVERSE]`, offered only while the executor reports
  `REV=1`, drives the interrupted route back to its start behind every motion gate and
  ends needing a new deliberate start. Continue run (owner, 2026-11-14,
  `docs/continue-run-sonar-range-2026-11-14.md`) is the operator's other deliberate
  choice: the same confirmed release, into the route the stop interrupted, resumed from
  the executor's RAM checkpoint (`RouteResume.h`) behind fresh GO and every motion gate;
  a new S3 heading reference since the stop needs wall confirmation. Preserve Pi stop/request-ID files and
  both controllers' software-reboot stop markers.
- `[RESET:<id>]` requests robot return/recovery. `[OP:<id>:REBOOT]` is the
  separate coordinated restart requiring confirmed STOP. Do not confuse them.
- Obstacle sonar belongs to the executor servo; bin-level ultrasonic belongs
  to the sensor link. Disabling obstacle sonar keeps the servo at DEFAULT_VIEW,
  while re-enabling requires fresh clear confirmations. Report the actual scope.
  Since 2026-11-12 the servo sweeps (`SonarSweep.h`: forward/+-25 fan driving, the whole
  span stopped inside a route, forward-only spinning/idle). A ping with no echo inside the
  timeout is a far clear reading (the old "unknown" could never release a hold in an open
  hallway); releases still need the confirmation counts. Since 2026-11-14 the sonar reads
  only to 110 cm (`ULTRASONIC_MAX_RANGE_CM`; a further echo is clear) and a forward stop
  beyond 25 cm needs two consecutive echoes inside 60 cm, as the side bearings do. Inside a turn the sonar evidence
  is only the spin circle at floor level, surveyed before every spin.
- Obstacle and path holds never end a route (owner, 2026-11-12): `waitUntilMotionPermitted()`
  holds until fresh GO, no STOP and no sonar obstacle, and a STOP at any moment holds again.
  Only an explicit reset, the operator stop, a latched fault, a rejected wheel command, a
  wrong-way/step-limit turn or the 5-minute heading limit fail a segment. A hold with no
  clearance evidence for 10 s enters low power (CPU 80 MHz, slower polls/pings); the first
  clear evidence restores normal power before motion. A persistent buzzer task beeps while
  a route drives, chirps while held, and is silent idle/parked/faulted/emergency-stopped.
- Moving-object prediction (`RasPi/motion_tracker.py`, 2026-11-12) only ADDS evidence:
  predicted contact within 2 s is generic `F=O`, a closing mover `F=W`, a mover reaching the
  spin circle `T=O`; the read-only front bypass bypasses it. It needs the executor's
  `[DRIVE:..]` telemetry for ego motion (telemetry only, grants nothing) and degrades to
  "robot speed unknown" without it. Since 2026-11-14 the wall heading of each scan removes
  the robot's own rotation from the tracks, and a person-sized, non-flat track proven to
  move over the ground is labelled a **likely person** (red dot on the map; owner's request).
  Since 2026-11-16 the label also needs motion relative to the other tracked objects when
  most of them drift together along the travel axis by no more than the wheel speed (wheels
  lifted, slip, a lagging report: still objects had been labelled). That label is display
  and report only: every stop/slow rule treats all movers alike, and none uses the drift.
- Hallway objects (`RasPi/scene_objects.py`, 2026-11-14) only ADD evidence: with both walls
  fitted 120-450 cm apart, returns between them are grouped into objects (wall fixtures and
  the end wall excluded) and marked on the map; one standing in the robot's path beyond the
  slow zone and within 4 m asks for `F=W` after three scans. The read-only front bypass
  bypasses it. No walls, no scene.
- Wall heading (2026-11-14): `H` may come from the long-baseline fit when it agrees with the
  near fit; WL/WR never do. The Pi learns the LiDAR mounting error from lateral drift
  (`RasPi/wall_heading_offset.py`, at most 0.1 deg per 3 m stretch, +-3 deg) and takes it out
  of `H` only. It is heading evidence, never clearance.
- Phone control (owner, 2026-11-14, `RasPi/cloud_control.py`,
  `docs/phone-control-headless-2026-11-15.md`): an operator request in
  `devices/<id>/commands/operator` is claimed in a transaction, passed through the SAME
  `operator_control.request` gate as the monitor's buttons, and answered with the executor's
  own acknowledgement. At most once: a request claimed by a process that restarted ends
  `unconfirmed`, never replayed. A stop is applied up to 120 s old; any other action older
  than 20 s is refused. The status mirror and the leased map frames are telemetry. None of it
  is path evidence or grants GO, and the phone's stop is a convenience, not a safety device.
- `[HEALTH:1:UUUUU]` reports internet/cloud/LiDAR/bin sensors/thermal U/D/X.
  It is low-priority, coalesced and age/session checked, never readiness or
  path evidence. Firebase failure alone is not proof of internet loss.
- The persistent modem worker owns its UART. Motion/parser/Tk callbacks only
  enqueue bounded notices. Preserve category priority, transition pairing,
  deduplication and attempt budgets even when a queue entry is replaced or
  telemetry becomes unknown. An uncertain post-Ctrl-Z result must not replay.
  `+CMGS` with final `OK` confirms network submission, not handset delivery.
- `[MCU READY]` means a controller restart: the bridge ends its segment/turn notices
  and the executor restarts its own route frame; the bridge heading continues.
  `[MCU ALIVE]` is the recovery ping during a route and changes nothing. A BLE outage
  must not end the heading stream. Missing walls remove centring, not MPU heading hold.
- The LiDAR and sensor board are found by their data (`RasPi/port_detection.py`), never by port
  number or USB name. Probing only listens, and must never open a port the other role holds.
- Motor movement is open loop. Step-counter progress cannot prove wheel motion
  or detect slip. Do not change calibrated routes/wiring/orientation by guess.
- A rising step count means forward. The user's November 9 report says the November 7
  `true/false` DIR pair still drove backward; the current pair is `false/true` in
  `MAIN_EXECUTOR_CODE.h`. Preserve route signs and turn angles. Which motor is the left
  wheel is not measured: the first MPU turn checks the guess and swaps it, and steering
  uses the proven side. Do not flip the DIR pair again without new evidence. `tests/firmware/test_motor_direction.cpp` models the reported forward
  levels and keeps motor-side assumptions explicit. Historical November 7 reports
  remain dated evidence and are superseded for the current direction settings.
- The user's October 31 correction supersedes the previous inspection-only
  LiDAR interlock. Front/back read-only controls bypass only the selected group's
  distance thresholds while readings continue. They must not globally block
  motion or manual start, or cancel pending starts. Missing/stale scan evidence,
  side clearance, emergency stop and MPU/nearest steering-clearance gates remain.
  Owner, 2026-11-16 ("let me decide when will it move", `docs/operator-decides-2026-11-16.md`):
  changing any read-only group while a route runs latches the operator stop, and so does a
  Pi process that finds a route already running at its first state report (not a Bluetooth
  reconnect); the operator then chooses Continue run, Reverse to start or Release stop.
  Sides read-only (owner, 2026-11-16: "add read only for sides too, so that the motor can
  be tested and run", `docs/side-read-only-2026-11-16.md`) is the third group: it bypasses
  the side sector limits and the side bands beside both halves of the body for F/B/R, off
  at every start; front/back read-only still never bypass a side, the spin sweep stays with
  the front/back groups, and missing/stale scans, readings, zones and steering vetoes stay.
  Confirmed SONAR_OFF likewise permits operation with its local gate bypassed
  and servo parked at DEFAULT_VIEW. Do not reinstate an extra motion interlock
  without a new user request.
- Developer tests (owner, 2026-11-15, `docs/developer-tests-start-hold-2026-11-15.md`) are
  a deliberate exception to the side-band and heading-feedback rules above: a single
  confirmed `[TEST:..]` from stationary IDLE may leave out LiDAR clearance (all zones and
  scan freshness), MPU supervision and/or obstacle sonar for that test only. It is capped at
  400 steps forward or backward, 100 on one motor or a 1..15 degree / exactly 90 degree
  MPU-measured turn. Owner's latest speed request (2026-11-16/17,
  `docs/developer-turn-speed-2026-11-17.md`) sets the finite-test speed to 2351 steps/s,
  nominally 30 cm/s; ramps, short moves, motor-side checks and turn braking can be slower.
  Small tests have 8 s, exactly 90 degrees has 30 s, and both need a live 800 ms KEEP
  lease. Measured developer turns require fresh stationary MPU confirmation and a final
  1 degree tolerance. MPU scale is provisionally 100/90 from the owner's approximate
  physical 100-degree observation for a reported 90, pending measured floor acceptance;
  do not apply that correction twice or change the degree targets. Backward (2026-11-16,
  `docs/backward-test-2026-11-16.md`) runs inside the reverse-parking rear scope: with LiDAR
  selected only `GO:BACK` moves it. Remote control (owner, 2026-11-16,
  `docs/remote-control-2026-11-16.md`): the Developer window's held arrows (`W/S/A/D`, mask 0,
  2351 or 4702 steps/s, nominally 30/60 cm/s) have no step value, no time limit and no sensor gates by the owner's
  choice; they run only while the Pi keeps a 500 ms lease alive from a window that still
  shows the arrow pressed, and `[TEST END]` (let go) stops them at once. The phone cannot keep
  that lease, so its arrows are nudges (owner, 2026-11-18, `docs/phone-controls-rundown-2026-11-18.md`):
  the Pi lets go by itself `PHONE_NUDGE_S` (0.6 s) after sending the start; never lengthen it
  from the phone's side or make a phone arrow latch. RunStart (owner,
  2026-11-16, `docs/runstart-2026-11-16.md`): the Developer window's own list of up to 20 steps
  (`P`: F/B/L/R/C, 1..100000 steps each, max speed 2351..8000 steps/s, acceleration
  100..20000 steps/s²) runs with no sensor gates and no time limit by the owner's choice, under
  the 800 ms KEEP lease. The executor keeps the list and its exact wheel place after it ends or
  stops; `C` continues it to its end and `U` undoes it back to its start, each a new confirmed
  request from IDLE with no latched stop (the monitor releases the stop first through the
  operator gate). The phone may build and start a list too (`DEV_RUN_START`, owner 2026-11-18):
  the same `request_run`, ranges and part-way lock, never another path. Any other motion, a new list or `[RUN CLEAR]` forgets it. The emergency stop
  and link/protocol stops still end every test. Once its wheels have moved, any hold ends the test with the cause named; it never
  restarts them (2026-11-16: lifted turns stop-started loudly). It never creates GO evidence,
  never starts or resumes a route, discards the route
  checkpoint and ends at IDLE needing a new deliberate start. Do not let these overrides reach
  routes or persist after the test.
- The current 2D LiDAR reports generic returns in its scan plane; it cannot
  recognise people. The "likely person" marker (owner, 2026-11-14) is a size-and-motion
  guess for the map and the log, and must be described as that. Do not label legacy `H`
  packets as human recognition or claim injury prevention without a separately validated
  people sensor and physical stopping-distance evidence.
- Pi PATH snapshots retain their LiDAR acquisition deadline through the bounded
  BLE queue and are revalidated before GATT write. This does not replace an
  end-to-end timestamp/age check in the controllers; never describe it as one.

## Verification and handoff

Run relevant tests from this directory. Standard software commands are:

```bash
python3 -B -m unittest discover -s tests/pi -p 'test_*.py'
python3 -B tools/run_firmware_tests.py
python3 -B tools/source_sanity.py .
```

Tk tests need a working display; explicitly report skips. Firmware host tests
execute production algorithms with substituted peripherals. Their task stub does
not run `xTaskCreate` workers automatically: exercise the actual worker service
and UART response parser for modem changes. Host tests include source into one
translation unit and can miss private-symbol/export mistakes; compile each
affected actual ESP32 target using the verified configuration and an external
build path. Never upload as part of a verification command.

Record exact checks, source/binary hashes and remaining physical limits in a
dated artifact directory. Preserve historical reports as dated evidence. Update
architecture/release/deployment notes for changed behavior. README.md only says what
GARBY is for and how to use it (owner, 2026-11-12): change it when usage changes, never
with change logs; history belongs in RELEASE_NOTES.md and dated docs/. Distinguish
source implemented, software tested, firmware uploaded and physically accepted;
never treat one as proof of the next.
