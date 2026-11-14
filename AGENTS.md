# GARBY project maintenance

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
  directory; local helpers are required. The serial test sketches are not
  alternative production implementations.
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
  deadband and 5/2 cm hysteresis, and is slewed. Every turn is MPU-measured, then aligned
  to two agreeing wall scans within 8 deg of the commanded direction. Executor tunables
  are at the top of `HeadingControl.h`, bridge ones in `SteeringAssist.h`. Check changes
  with `tools/firmware_tests/test_navigation.cpp`, a kinematic model that cannot show
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
  160 cm (`F=W` -> `GO:SLOW`), side band within 2 cm of the body (never bypassed), spin
  sweep `T=` (required clear between `[TURN:START]`/`[TURN:END]`). Chassis outline is
  env-configured and unmeasured.
- Gyro bias is re-measured only while the wheels are held (STOP latched >= 1 s, or the
  executor reporting IDLE/OUTBOUND COMPLETE/ROUTE FAULT for >= 1 s), never in a turn,
  at most 0.2 deg/s per window and 1 deg/s from boot.
- Operator emergency stop is latched separately from ordinary obstacle STOP.
  GO, reconnect and return commands cannot release it. Release aborts to IDLE
  and requires a new deliberate start. Preserve Pi stop/request-ID files and
  both controllers' software-reboot stop markers.
- `[RESET:<id>]` requests robot return/recovery. `[OP:<id>:REBOOT]` is the
  separate coordinated restart requiring confirmed STOP. Do not confuse them.
- Obstacle sonar belongs to the executor servo; bin-level ultrasonic belongs
  to the sensor link. Disabling obstacle sonar keeps the servo at DEFAULT_VIEW,
  while re-enabling requires fresh clear confirmations. Report the actual scope.
  Since 2026-11-12 the servo sweeps (`SonarSweep.h`: forward/+-25 fan driving, the whole
  span stopped inside a route, forward-only spinning/idle). A ping with no echo inside the
  timeout is a far clear reading (the old "unknown" could never release a hold in an open
  hallway); releases still need the confirmation counts. Inside a turn the sonar evidence
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
  "robot speed unknown" without it. It reports moving objects, never people.
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
  uses the proven side. Do not flip the DIR pair again without new evidence. `tools/firmware_tests/test_motor_direction.cpp` models the reported forward
  levels and keeps motor-side assumptions explicit. Historical November 7 reports
  remain dated evidence and are superseded for the current direction settings.
- The user's October 31 correction supersedes the previous inspection-only
  LiDAR interlock. Front/back read-only controls bypass only the selected group's
  distance thresholds while readings continue. They must not globally block
  motion or manual start, or cancel pending starts. Missing/stale scan evidence,
  side clearance, emergency stop and MPU/nearest steering-clearance gates remain.
  Confirmed SONAR_OFF likewise permits operation with its local gate bypassed
  and servo parked at DEFAULT_VIEW. Do not reinstate an extra motion interlock
  without a new user request.
- The current 2D LiDAR reports generic returns in its scan plane; it does not
  classify people. Do not label legacy `H` packets as human recognition or claim
  injury prevention without a separately validated people sensor and physical
  stopping-distance evidence.
- Pi PATH snapshots retain their LiDAR acquisition deadline through the bounded
  BLE queue and are revalidated before GATT write. This does not replace an
  end-to-end timestamp/age check in the controllers; never describe it as one.

## Verification and handoff

Run relevant tests from this directory. Standard software commands are:

```bash
python3 -B -m unittest discover -s RasPi -p 'test_*.py'
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
