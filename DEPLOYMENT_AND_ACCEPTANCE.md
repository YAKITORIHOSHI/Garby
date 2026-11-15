# GARBY deployment and acceptance

## Phone control and headless — 2026-11-15

Pi only (`PHONE-20261114`): deploy the entire `RasPi/` directory. Before the app's controls
work, merge the rules example from the app repository into the database rules and add each
operator's uid under `operators/`. Then `bash RasPi/run_headless.sh doctor` and `start`.
`sudo python3 RasPi/install_startup.py` installs the boot service, which now runs
`run_headless.sh run` (saved log, status file). First-use steps are in
`docs/phone-control-headless-2026-11-15.md`; none has been run yet.

## Station steps, reverse run, map markers — 2026-11-14

Flash both complete sketches with build ID `REVERSE-20261114` and deploy the entire matching
`RasPi/` directory at the same time. This build replaces `CONTINUE-20261114` (it contains it):
if that build was never flashed, flash only this one. The executor's operator status line
gains `:REV=`, which an older Pi does not parse (its operator controls would read as
unsupported; emergency stop still works). The new Pi accepts the older lines.

New optional Pi settings, all in `RasPi/garby-bridge.env.example`: `GARBY_WALL_FAR_FIT`,
`GARBY_WALL_HEADING_LEARN`, `GARBY_WALL_HEADING_OFFSET_FILE`, `GARBY_STATION_WALL_MIN_GAP_CM`.
Two values wait for a tape measure: the chassis outline (`GARBY_CHASSIS_*`) and the sonar's
distance to the station wall when parked (`STATION_PARK_GAP_CM` in
`MAIN_EXECUTOR_CODE/SonarSweep.h`, an executor rebuild).

Acceptance steps (wheels lifted first, then supervised on the floor) and the log lines to send
back are in `docs/objects-station-reverse-2026-11-14.md`; none has been run yet.

## Continue run — 2026-11-14

Flash both complete sketches with build ID `CONTINUE-20261114` and deploy the entire matching
`RasPi/` directory at the same time: the executor's operator status line gains `:RESUME=`,
which an older Pi does not parse (its operator controls would read as unsupported; emergency
stop still works). The new Pi also accepts the older line. Keep the Pi stop-state and
request-ID files. Acceptance steps (wheels lifted first, then supervised on the floor) are in
`docs/continue-run-sonar-range-2026-11-14.md`; none has been run yet.

## Test2 turn recovery — 2026-11-14

Use matching complete bridge and executor sketches with build ID `RECONNECT-20261114`.
The bridge preserves turn-sweep protection across BLE reconnect and requires the executor's
turn-state replay after bridge restart. Both firmware images are required for that recovery
contract. A Pi test-only repair adds no new Pi runtime deployment; the earlier disposal
feature still requires the entire matching `RasPi/` directory and reviewed cloud rules.

Under supervised acceptance with the robot secured, interrupt BLE during a held turn.
Occupied, stale and missing turn-sweep evidence must keep it stopped after reconnect;
fresh clear sweeps must resume the interrupted turn. Repeat with a bridge-only restart:
no GO is permitted before executor turn-state replay. Confirm ordinary straight travel,
emergency stop and controller restart retain their existing behavior. These physical checks
were not performed in this software review. Installed firmware identity remains unknown.

Software checks do not establish physical safety. Historical serial inventories
and saved flasher selections do not prove that a board is currently connected.
Read current hardware state before uploading. Physical route direction, servo
position, LiDAR mounting and stop behavior require secured hardware acceptance.

## Manual disposal — 2026-11-13

Upload requirement for this change: **Both**, complete `BLE_Receiver-Final` and
`MAIN_EXECUTOR_CODE` sketches, build ID `DISPOSAL-20261113`. Deploy the entire `RasPi/`
directory and build/install the updated Android app (versionCode 6) on Windows. Review and
merge the supplied disposal-only Firebase rule example into the actual rules before
using app control. No rules, software, firmware or phone installation were deployed by
this task. Installed versions and earlier pending uploads remain unknown. The new images
include the earlier local source changes; no separate additional flash of those sources is
needed when these complete images are installed.

Follow [`docs/manual-disposal-2026-11-13.md`](docs/manual-disposal-2026-11-13.md) for Windows
build commands and a supervised acceptance sequence. Confirm cancel sends nothing, an
idle robot accepts a below-threshold request once, obstacles/heading loss still hold, and
emergency stop, busy/fault state, stale eligibility, duplicate requests, offline delivery and
reconnect cannot start a later unintended run. Arrival and the existing explicit return
are separate checks. Physical acceptance and low-end phone performance remain unverified.

## Local testing revision — 2026-11-13, heading recovery and scan age

The owner requested that these changes stay local for comparison with the remote
repository. No remote update is part of this task.

Both controller builds are `REVIEW-20261113`. This change requires the complete
`BLE_Receiver-Final` and `MAIN_EXECUTOR_CODE` sketches and the complete `RasPi/`
directory, with its required NumPy dependency. The executor fixes recovery through
STOP/sonar holds and requires walls after a long outage with a new reference.
The bridge expires steering permission from acquisition age; the Pi refreshes
that age after queue/writer-lock waits. Wire grammar is unchanged. Earlier
pending images and installed revisions remain unknown.

Evidence: `docs/review-fixes-2026-11-13.md` and
`artifacts/review-fixes-2026-11-13/`. No upload, Pi deployment, service restart or
live robot action was performed here.

For secured acceptance, record boot build IDs. Confirm STOP:MPU during a straight
or turn brakes and requires the recovered heading and wall pair, followed by
fresh GO/sonar clearance. Missing required walls must keep it stopped and fail
the segment at the original five-minute deadline. An ordinary obstacle with
valid heading must remain recoverable beyond five minutes. Capture Pi/executor
logs; software models do not establish physical alignment, slip or stopping
distance. Retain previous matching images and the whole Pi directory for rollback.

## Previous revision — 2026-11-12, holds that resume, sonar sweep, prediction

Executor build `NAV-20261112` plus the complete `RasPi/` directory; the bridge stays
`NAV-20261111` (unchanged source; flash it too if the S3 still runs an older build, since the
executor needs its M/W lines). Image hashes: `artifacts/obstacle-sweep-hold-2026-11-12/`.
Nothing was uploaded by this work. A Pi without this revision still works with the new
executor (the `[DRIVE]` lines are ignored), and an older executor with this Pi only loses the
ego-motion input (the prediction then assumes any robot speed from 0 to 75 cm/s).

Acceptance: bench steps B1-B4 and floor steps F1-F2 in
`docs/obstacle-sweep-hold-2026-11-12.md` (servo sweep and bearings, hold/low-power/resume
with a box and with a hand, a person walking toward the LiDAR, the pre-run centring lines,
people in the hallway), with a saved Pi log and the executor USB log. Rollback: reflash the
previous executor image and restore the previous `RasPi/` directory.

## Earlier revision — 2026-11-11, hallway navigation redesign

Build `NAV-20261111` on both boards plus the complete `RasPi/` directory, deployed
together: the Pi adds `T=`/`F=W` and wall fields, and the bridge<->executor UART lines
changed (an old executor rejects the new heading lines; an old Pi gives no turn
sweep, so turns hold `STOP:TURN_UNKNOWN`). Images and hashes:
`artifacts/navigation-2026-11-11/`. Nothing was uploaded by this work.

Acceptance follows `docs/navigation-2026-11-11.md`: calibration steps 1-8, bench
steps A1-A7 (wheels lifted; A4 checks that the new side band and spin sweep do not
see the robot's own structure), then floor steps F1-F2 with a saved log
(`bash RasPi/run_bridge.sh 2>&1 | tee ~/garby-$(date +%F-%H%M).log`). In the general
checklist below, step 10's tap/FRONT items and the nudge items of step 5 are
superseded: verify the yaw sign (A2), MPU turns with wall alignment and the
continuous steering instead. Rollback: reflash the previous images of both boards
and restore the previous `RasPi/` together.

## Earlier revision — 2026-11-09, front direction and circular scan

The [BLE investigation](docs/ble-establish-2026-11-09.md) records owner-reported
USB success and battery failure with the S3 fed through the Executor. Verify and
repair standalone power/startup before accepting battery operation. The Pi-only
diagnostic update needs complete `RasPi/` deployment and no MCU upload; it does
not repair supply voltage. Follow the report's single-input power checks and
require successful battery-only discovery/readiness plus stable rails/reset logs.

The [LiDAR left/right correction](docs/lidar-left-right-2026-11-09.md) is Pi-only.
Deploy the complete current `RasPi/` directory. The reported swapped lateral
readings use `GARBY_LIDAR_REFLECT_LEFT_RIGHT=1` by default, with the existing yaw
offset retained. While parked, verify objects at real left/right/front/rear in
both map and summaries. Motor turns/nudges were not confirmed reversed and are
unchanged. This correction needs no BLE or Executor firmware upload; earlier
unuploaded direction firmware remains a separate deployment item.

The [follow-up integration fixes](docs/front-direction-circular-review-2026-11-09.md)
change only the Pi runtime and monitor: acquisition age survives missing plot
geometry, missing sectors remain unavailable, and processing errors immediately
invalidate health and queue STOP. Deploy the complete updated `RasPi/` directory.
The executor source and `FRONT-20261109` image below are unchanged by the follow-up.

The owner confirms the November 7 build still ran backward. The
[current direction correction](docs/motor-direction-2026-11-09.md) sets the two
DIR polarities to `false/true`, with boot ID `FRONT-20261109`. Calibrated route
values and turn/nudge algorithms are preserved. Compile-only software checks
do not deploy this change.

For deployment, use the new executor image and the entire `RasPi/` directory,
including `lidar_map.py`. The S3 firmware is unchanged. The
[circular LiDAR view](docs/circular-lidar-2026-11-09.md) shows the current local
scan with up to 360 angular bins; the button returns to the eight summaries.
It does not locate the robot or build a persistent floor map.

Before floor acceptance, confirm the executor boot ID and DIR values, then
verify physical forward travel, motor sides and both named turns/nudges with
wheels lifted. While parked, put an object at the real front and real left;
check that the circular plot and eight summaries agree. Verify the mounting
offset from observation rather than changing it to match motor polarity.
Check stale scans disappear, the selected front/rear read-only bypasses retain
readings and Start behavior, and emergency stop remains latched. Follow the
current direction report for the remaining supervised route checks.

## Earlier revision — 2026-11-07, motor direction

The [direction change](docs/motor-direction-2026-11-07.md) reverses forward, backward, left, right and nudges through the
two DIR polarities in `MAIN_EXECUTOR_CODE.h`. Flash only `MAIN_EXECUTOR_CODE`, then power-cycle the robot. The S3 sketch and
the Pi are unchanged. Software tested only. Before any floor run, follow that report's checks in order. Its first
concern is which way the LiDAR faces, since the robot now travels the opposite way.

## Earlier revision — 2026-11-07, build B

The [centering revision](docs/centering-spec-2026-11-07.md) changes only the `BLE_Receiver-Final` sketch, including
`SteeringAssist.h`. Its image and hashes are in `artifacts/centering-spec-2026-11-07/`. Confirm
`[BRIDGE BUILD:CENTER-20261107B]` in the Pi log. Executor sources are byte-identical and `RasPi/` needs no change. Follow
that report's physical checks, starting with `ENABLE_LIDAR_CENTERING = false`. No upload was performed for this work. It
supersedes the [first centering revision](docs/lidar-mpu-centering-2026-11-07.md) of the same day.

## Coordinated revision — 2026-10-30

The latest [steering stability revision](docs/nudging-2026-10-30.md) requires the
complete updated `RasPi/` directory and `BLE_Receiver-Final` sketch including
`MpuHeading.h` and `SteeringAssist.h`. Its S3 build/evidence directory is
`artifacts/nudging-2026-10-30/`, superseding the earlier S3 SMS image. The current
executor source is unchanged: retain its matching operator/SMS/heading firmware.
Old numeric Pi steering packets lack required nearest-clearance/acquisition
evidence and cannot nudge with the new S3. Follow the revision's measured
corridor, isolated-object and loaded-run acceptance steps before claiming
improved straightness. No upload or runtime restart was performed for this work.

Use the complete updated `RasPi/` directory and both updated firmware sketches
for the [tabbed monitor/operator controls](docs/gui-operator-controls-2026-10-30.md)
and [important SMS notices](docs/sms-notices-2026-10-30.md). The latest matching
executor build and earlier paired source/binary hashes and software checks are in
`artifacts/sms-notices-2026-10-30/`; these images include the earlier GUI/operator
changes and supersede its operator-only binaries. Existing Android source does
not need changing for these local features. A matching compiled image is not
evidence it has been uploaded.

Preserve `~/.config/garby/operator-stop` and its adjacent `.request-id` reservation
file when deploying. Desktop/service processes must use the same configured
`GARBY_OPERATOR_STOP_FILE` and runtime-lock identity. The reboot button requires
confirmed STOP and existing OS permission for orderly `systemctl` reboot; it
does not install boot startup or grant that permission. Reboot leaves STOP
latched and requires explicit release and a new manual start.

After software review, hardware acceptance must be explicitly authorized and
performed with secured/lifted wheels and an accessible physical stop:

1. Verify actual servo default view (70 degrees), disable obstacle sonar and
   confirm ping cessation/position hold. Re-enable and confirm fresh-clear
   gating. Bin-level sensing must remain independent.
2. Verify emergency stop during movement, queued GO/start/return resistance,
   link loss and reconnection, explicit release to stationary IDLE, and no
   automatic route resumption. Measure real stop latency and braking distance.
3. Verify confirmed-stop coordinated restart, denied Pi reboot permission,
   missing acknowledgement, startup after OS reboot and retained stop state.
   The separate bin/gas board is excluded from the reset path.
4. Verify the preserved `CONTACT_NUMBER`, SIM/antenna/service and modem
   registration. With separate authorization for an actual text, confirm a
   message at the recipient handset. `+CMGS`/`OK` alone proves submission only.
5. Disconnect only Pi internet while keeping cellular service and controller
   links working; verify sustained internet loss and confirmed recovery. Verify
   a Firebase-only failure is described separately, and unknown/stale health
   data cannot report a false recovery.
6. Exercise sensor/link/thermal and route notices through controlled failure
   injection, then verify priority, coalescing, spacing and recovery pairing.
   Do not create a real overheating, unsafe motion or electrical fault for a test.

The modem now has one persistent UART owner; raw debug-console passthrough is
disabled. SMS cannot provide immediate notice during complete cellular/power
loss, and RAM notices can be lost across restart. Queued texts never delay STOP
or the motion safety loop. No live SMS, hardware upload or reboot was performed
by the recorded software verification.

## Build and prepare

Run from the repository root:

```powershell
python -B tools/verify_project.py
```

Verified target baseline: ESP32 core 3.3.11, NimBLE-Arduino 2.5.1,
FastAccelStepper 1.2.7, ESP32Servo 3.2.1, HX711 0.6.4;
The updated BLE bridge uses FQBN `esp32:esp32:esp32s3`; the main controller
retains `esp32:esp32:esp32`. See [S3/MPU wiring](BLE_Receiver-Final/NOTE.txt)
for GPIO8/9 I2C and RX18/TX17 bridge UART. Both sketches need updating for the
heading-segment notices. Android uses the checked-in Gradle wrapper (9.9.1), Java21,
and Android SDK 37.2 (target 37). On a network-share checkout, set an absolute
`garby.buildRoot` in `Garby_MobileApp/local.properties` to build on a local disk.
Host behavioral tests require MSVC C++ Build Tools on Windows
or g++/clang++ on Unix. Full validation exports each sketch's flashing binaries
under `artifacts/firmware/<sketch>/` and builds debug plus unsigned release APKs.

Flash the main controller, then BLE bridge, with wheels lifted and an accessible
hardwired emergency stop. Deploy the entire `RasPi/` directory; a single Python
file is insufficient. Install the new Android APK after updating all nodes.
The correlated return protocol is a coordinated release, not compatible with
old controller return acknowledgements.

New BLE notification framing also requires the complete Pi directory including
`ble_support.py`. After flashing, isolate the connection with the
[BLE connection guide](BLE_CONNECTION_GUIDE.md) before starting ROS/Firebase.

## Raspberry Pi configuration

Install ROS2/rclpy, sensor_msgs, and a built YDLIDAR ROS2 driver workspace.
Install the full dependency list into the Python used by the service for
Android/cloud telemetry and remote return requests:

```bash
python3 -m pip install -r RasPi/requirements-firebase.txt
```

Confirm that this Python can import ROS packages; an unrelated virtualenv can
hide them. For a lean local installation, use `RasPi/requirements.txt` instead
and omit `--require-firebase` from preflight. Without the optional Firebase SDK,
cloud updates and remote return requests are unavailable; local sensing and
BLE safety still operate. Installing the lean list does not uninstall Firebase.

For the desktop monitor, install system Tk support with
`sudo apt install python3-tk` on Raspberry Pi OS/Debian/Ubuntu and run the launcher
without `--headless` in a desktop session. The UI uses standard Tkinter widgets
and system fonts, with no third-party UI package. The headless service does not
need Tk or a desktop. `python3 RasPi/preview_ui.py` previews simulated readings
using only Python and Tk, without ROS or robot hardware.

Copy `RasPi/garby-bridge.env.example` to `/etc/default/garby-bridge` and edit every
deployment-specific path. The LiDAR and sensor board are detected from their data
on any port; `bash RasPi/run_bridge.sh --check-runtime` reports what was found. A
sensor board on GPIO14/15 first needs `dtparam=uart0=on`. Sensor UART = 9600;
bridge-to-main UART = 115200. Set the Firebase service-account path privately;
do not put it into the Android app or source control.

```bash
set -a
source /etc/default/garby-bridge
set +a
source "$GARBY_ROS_SETUP"
source "$GARBY_LIDAR_WORKSPACE_SETUP"
bash RasPi/run_bridge.sh --check-runtime --require-firebase
bash RasPi/run_bridge.sh --headless
```

The launcher sources both ROS and the workspace overlay. Its defaults are
`/opt/ros/humble/setup.bash` and `/home/garby/ros2_ws/install/setup.bash`; override
them for the actual installation. Merely sourcing the ROS distribution does
not expose a workspace-built YDLIDAR package.

Optional service:

```bash
sudo python3 RasPi/install_startup.py --user garby
sudo systemctl start garby-bridge
journalctl -u garby-bridge -f
```

Configure /etc/default/garby-bridge first; the installer sets the current
checkout path and selected service user. See RasPi/README.md for setup. The service starts headless and restarts on process failure.
No echo and missing scans remain fail closed; restarting a process is not proof
that sensors or communication are healthy.

Keep `GARBY_RUNTIME_LOCK_FILE` identical for production, the simulator, and the
BLE connection probe; the Pi default is `/tmp/garby-bridge.lock`. A second owner
exits with a lock error. Stop the owning service/process before switching tools.
The lock releases on exit, and its file remains; do not delete it while a program
owns it. GARBY only terminates its own LiDAR process group. If an older external
driver is running, stop its specific service/process before starting GARBY.

## Wheels-lifted acceptance

Record board IDs, firmware versions, configuration, payload, and serial logs.

1. Verify hardwired emergency-stop action, motor-driver reset/brownout behavior,
   regulated supplies, common ground, UART crossing, and safe sonar echo voltage.
2. Boot MCU/bridge with Pi absent: no wheel motion. Start Pi without scans:
   `STOP:WAITING_DATA` or `STOP:STALE`, never GO.
3. Verify `[MCU READY]`, BLE connection, and repeated-clear startup sequence.
   Restart each node separately and confirm recovery starts stationary.
4. Check LiDAR physical FRONT/LEFT/BACK/RIGHT with a target at known positions.
   Set `GARBY_LIDAR_YAW_OFFSET_DEG` from observation, not assumptions. FRONT must be
   the way the robot travels: back readings never hold a forward move.
5. Confirm both motors accept short target moves. A motor command rejection
   must stop both and report a route fault. Verify physical left/right wheel
   assignment against the nudge commands. Start must roll both wheels forward,
   then turn left; if it does not, use the table in
   `docs/motor-direction-2026-11-07.md` before changing anything.
6. Measure sonar at 20, 45, 60, 72, 120, 150, and 250 cm. Check the new 18000 µs
   echo window and center-servo settling. Disconnect echo: UNKNOWN must not
   release a local obstacle latch.
7. Trigger outbound. Introduce an obstacle while moving; verify braking and
   ongoing five-second HALT diagnostics. Remove it to a position producing
   **valid clear echoes** and clear LiDAR sectors. Confirm automatic resumption
   toward the remaining original target. Repeat at least ten times.
8. Block LiDAR alone, sonar alone, and both. Add a sonar obstacle while already
   waiting on LiDAR. Clearance must satisfy both when local sonar was involved.
   No manual reset should be needed for a normal cleared obstacle.
9. Stop `/scan`, unplug LiDAR, disconnect BLE, reboot the bridge, and restore
   each. Confirm STOP during loss and automatic segment recovery only after
   fresh confirmations. Record disconnect reasons and supply measurements.
10. (Superseded 2026-11-11 by bench steps A2/F1 of `docs/navigation-2026-11-11.md`;
    kept for history.) Mount the MPU level with +Z up. Check that a left heading deviation requests
    a right correction and vice versa; verify actual wheel response. A side
    obstacle must veto a correction toward it without inventing another turn.
    Verify 1°/0.5° hysteresis, bounded taps, cooldown, new FRONT after each planned
    turn, reference preservation across an obstacle pause, and tap cancellation
    on an MPU read fault or gap. Measure stationary gyro drift and the longest
    intended run; relative heading alone cannot establish lateral position.
11. Verify every outbound and return turn/distance with wheels lifted before
    floor tests. The step signs/numbers in `pointsRun.ino` were preserved.
12. At a completed destination, request return from Android. Confirm matching
    ACK, RETURNING, and DONE after arrival. Duplicate the same request ID during
    return: no extra STOP, restart, or second route. A generic `[IDLE]` must
    not report completion of an unrelated request.
13. Request return during incomplete outbound: stationary rejection/fault.
    Force a return segment failure: stationary fault and matching FAILED.
    Retries must not clear a route fault or replay the route.
14. Boot with the bin empty and HX711 absent. MCU readiness and UART/sonar polling
    must continue; weight must show unavailable. Restore the empty-bin converter,
    verify background tare, then add a known load. Two new valid samples must
    trigger. Disconnect it again: old weight must not trigger another cycle.
    Check calibration and the 20 kg plausibility limit against the installed scale.
15. Attempt a simulator or `check_ble.py --connect` while production owns the
    lock: it must exit without taking the connection. Stop production, then
    verify the same tool can acquire the lock. Repeat after a process crash.

For a route fault, physically return the robot to its known base, then use the
local supervised `[RESET]` recovery token. Do not use the app to guess its location.

## Reduced-speed floor acceptance

Perform these with supervision and a clear test boundary:

- Measure stopping distance and payload stability for empty/max load, lowest
  expected battery, and lowest-traction floor, including communication loss.
- Confirm the 95 cm front, 60 cm front diagonal and 22 cm side
  envelopes cover the whole chassis and worst measured stop distance with a
  margin. The diagonal limit assumes the chassis stays within 22 cm of the LiDAR
  to each side; a wider chassis needs this limit reviewed.
- Drive the hallway past its narrowest post, centered and about 20 cm off center,
  and confirm no `F=O` stop. Then place an object in the robot's path and one
  beside it to confirm each stops the robot at the expected distance.
- Confirm motor trim and nudge signs, then evaluate correction strength. Source
  numbers do not prove real steering performance.
- With the robot secured and a physical stop available, verify front and rear
  read-only controls bypass only their selected distance thresholds, keep live
  readings and preserve Start readiness. Confirm side obstacles, missing/stale
  scans and emergency stop still block motion. Verify confirmed SONAR_OFF parks
  the servo at DEFAULT_VIEW and does not itself block a deliberately requested
  move; re-enabling sonar must require fresh clear confirmations.
- Using secured, soft surrogates with nobody in the robot's path, verify coverage
  for open doors, wall protrusions, extinguishers, trash bins, and objects at
  different heights and side angles. The 2D LiDAR does not classify people and
  can miss a person outside its scan plane; do not use a person as a test target.
- Validate the stop envelope with an independent physical stop available. The
  software thresholds and host tests do not establish an injury-prevention
  guarantee or a human-detection capability.
- Complete several full load-trigger and gas-trigger cycles including monitored
  obstacle clearance and operator return.
- Verify live Firebase authorization/authentication, cached-value expiry,
  return readiness, return failure display, and phone reconnection. Confirm the
  app's robot state follows each executor state, that the return screen shows the
  matching ACK, DONE and FAILED reason, and that the RTDB rules allow the app's
  writes to `commands/reset` and `fcmInstallationId` only.
- Verify SIM registration/SMS and any externally configured FCM sender separately.

## Diagnosing a robot that stays stopped

| Diagnostic | Check |
| --- | --- |
| `path=STALE` | `/scan` publication, ROS overlay, LiDAR serial port, Pi/BLE writes, UART wiring |
| `path=BLOCKED` | Actual front/rear/side LiDAR target and sector orientation |
| `sonar=UNKNOWN` | Echo voltage/wiring, servo settling, sonar supply, target angle/range; no echo cannot prove clear |
| Valid clear path and sonar, confirmation below 3 | Look for intermittent bad echoes or STOP packets resetting the evidence |
| `[MOTOR] Wheel target rejected` | Stepper connection, speed/acceleration setup, library/board selection, driver wiring |
| `[ROUTE FAULT]` | Recover the physical position; do not replay the full route |
| BLE disconnect | Capture reason and supply/radio data; proper nominal power alone does not establish radio or scheduling health |
| Weight unavailable | HX711 wiring/power, empty-bin tare, calibration, converter samples, one-second freshness |
| Runtime lock unavailable | Stop the existing GARBY service/simulator/probe; check the shared lock path and access |

Sign off with recorded evidence. Until then the software is built and host-tested,
and physical operation remains unaccepted.
