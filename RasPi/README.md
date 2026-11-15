# GARBY Raspberry Pi Runtime

Production entry point: `final_w_serial.py`
Deploy the entire `RasPi/` directory, including its local runtime and UI helpers.

## Python packages

For normal operation with Android/cloud telemetry and remote return requests,
run from the repository root using the Python that can import ROS packages:

```bash
python3 -m pip install -r RasPi/requirements-firebase.txt
```

For a lean local installation, use `RasPi/requirements.txt` instead. It installs
`bleak`, `pyserial` and `numpy`. NumPy is required for chassis zones, hallway walls
and moving-object prediction; runtime preflight fails when it is absent.
Without the optional Firebase SDK, cloud updates and
remote return requests are unavailable; local sensing, BLE, and the safety bridge
continue to operate. Installing the lean list does not remove an existing SDK.

The GUI uses Python's standard Tkinter widgets and system fonts, with no
third-party UI package, image library, or downloaded fonts. On Raspberry Pi
OS/Debian/Ubuntu, install Tk support for the GUI with `sudo apt install python3-tk`.
Headless operation does not require Tk or a desktop session.

Keep the Firebase Admin service-account JSON in `RasPi/Firebase/`. The runtime,
simulator, preflight, and Firebase probe discover it relative to their source
directory, so moving the project or renaming the key does not require editing
code. Web configuration and Android `google-services.json` files are ignored.
Keep one service-account key in that folder, or select a file explicitly with
`GARBY_FIREBASE_CREDENTIALS`. Relative overrides are resolved from `RasPi/`,
independently of the terminal's working directory.

ROS 2 (`rclpy`, `sensor_msgs`) and the YDLidar ROS 2 driver remain system/ROS
dependencies and are not installed by either requirements file.

## Start

Source both ROS and the built YDLIDAR workspace overlay, and configure the
variables from `garby-bridge.env.example`. Run from the repository root:

```bash
bash RasPi/run_bridge.sh --check-runtime --require-firebase
bash RasPi/run_bridge.sh --headless
```

The launcher uses `GARBY_ROS_SETUP`, `GARBY_LIDAR_WORKSPACE_SETUP`, and
`GARBY_PYTHON_BIN`. See [deployment and acceptance](../DEPLOYMENT_AND_ACCEPTANCE.md)
for full setup and systemd installation. Direct execution is also possible after
sourcing the same environments: `python3 RasPi/final_w_serial.py --headless`.

Omit `--headless` to open the monitor in a desktop session. If `DISPLAY` is
missing, Tk is unavailable, or the display cannot be opened, the bridge logs
the reason and runs headlessly. Display selection happens before ROS and
background workers start. For a lean local installation, run preflight without
`--require-firebase`.

The complete monitor is painted before peripheral workers start, and BLE
discovery starts before ROS initialization. There is no fixed startup sleep.
An exception in a monitor refresh is logged and retried; it does not permanently
stop the GUI's update timer.

On this Pi, start the monitor with `bash run_bridge.sh` from `RasPi/`.
The launcher selects installed ROS Jazzy (Humble fallback), sources
`~/ros2_ws/install/setup.bash`, and includes the user-local Python dependencies
without activating a virtual environment. The YDLIDAR SDK is installed under
`~/.local`, and the ROS driver is built in `~/ros2_ws`.

The LiDAR and sensor board are found by what they send, on any USB port, in either
order, and after being moved or re-plugged. No port setting is needed:

- **LiDAR:** the port streaming chained YDLIDAR packets at `GARBY_LIDAR_BAUDRATE` (115200).
- **Sensor board:** the port sending `ULTRASONIC:`, `MQ4:`, `MQ137:` or `MQ135:` lines at
  `GARBY_SENSOR_SERIAL_BAUD` (9600).

Probing only listens. It never transmits or asserts DTR/RTS, and it never opens a port the
other device holds. A port with no matching data is never selected, so an unknown board
cannot be mistaken for the LiDAR, and a missing LiDAR still means STOP. While a device is
unplugged the bridge keeps retrying, and readings resume once fresh data arrives. A silent port
is re-checked every 30 s, so a board that boots late is picked up.

Setting `GARBY_LIDAR_PORT` or `GARBY_SENSOR_SERIAL_PORT` turns detection off for that device.
`~/.config/garby/lidar-port` is no longer read. A sensor board on GPIO14/15 can only be detected
once that UART exists: add `dtparam=uart0=on` to `/boot/firmware/config.txt` and reboot.
`bash run_bridge.sh --check-runtime` reports what was found.

Fresh partial scans display their available direction readings with a
"Scan incomplete" status. Missing directions continue to require STOP; coverage
quality is distinct from stream freshness. The supervisor restarts a driver for
a sustained loss of usable fresh scans, not merely because a direction has no
returns. Stale or invalid scans still hide readings and revoke clearance.

Preview the interface with simulated readings, without ROS, Bluetooth, serial
hardware, or Firebase:

```bash
python3 RasPi/preview_ui.py
```

The preview requires only Python and Tk support and does not send robot commands.
`--scenario` selects `normal`, `obstacle`, `loading` (the radar sweep), `unavailable`
(a reported driver failure) or `stale` (one scan, then the stream stops).

## LiDAR map

The **LiDAR readings** tab is one robot-centred map, front up, drawn as a dark radar
scope (2026-11-12; it replaces the eight number cards and the circular-scan toggle).
The eight sectors (front, front left, left, rear left, rear, rear right, right, front
right) are written around the scope, each on its own side, in whole centimetres: the
X3 datasheet gives 2 cm systematic error within 1 m, so tenths would overstate it.
A sector inside its stop distance reads "close" in red and its dashed stop arc turns
solid red. A read-only (bypassed) group reads "read only" in amber and its stop arcs
are hidden, because they are not enforced. **Range** cycles 2, 4 and 8 m; an arc
under 6 px at the chosen range is hidden. Old points are never kept: this is the
current scan only, with no localization.

Without a live scan the map does not show stale numbers:

- **Loading** — a green radar sweep, titled with what the bridge is doing
  (searching for the LiDAR, starting it, waiting for its first scan, reconnecting)
  and the seconds waited.
- **LiDAR not available** — the reason the driver supervisor reported (for example
  no serial port streaming LiDAR data, or the driver exiting), or no scan for 45 s.
  It stays up while the bridge retries, until a live scan replaces it.
- **Scan needs attention** — a fresh scan that the bridge flagged unusable (too few
  valid returns); motion is held.

The map is display only; scans and their age alone decide STOP and GO.

## Moving-object prediction (2026-11-12)

`motion_tracker.py` tracks small LiDAR segments (people's torsos at the LiDAR's chest-height
plane, carts, doors opening) with ground velocity and predicts contact with the robot. It only
adds evidence to the path packet: contact within 2 s is `F=O`, within 4 s or a mover closing
within 2.5 m ahead is `F=W` (caution speed), a mover reaching the spin circle is `T=O`. The
read-only front bypass bypasses it. The executor's `[DRIVE:S|F:<cm/s>|T:<deg/s>]` lines (kept
out of the GUI log) tell it how the robot itself moves; without them only objects closing
faster than 1 m/s count. The log shows
`Moving-object prediction: STOP (moving object 3.2 m away, closing 2.0 m/s, contact in 1.6 s;
robot 71 cm/s)` and `... clear`; the footer zones show `moving stop/slow/turn`. Executor
`[POWER:LOW]`/`[POWER:NORMAL]` (low-power hold) and `[SONAR:SILENT]`/`[SONAR:FAULT]` are
logged too. It cannot tell people from objects and sees nothing below its plane. Details and
model results: `docs/obstacle-sweep-hold-2026-11-12.md`.

## Operator controls

The monitor has three tabs: **LiDAR readings**, **BLE status & messages**, and
**Controls**. The emergency-stop button stays visible on every tab. Controls
contains manual start, LiDAR read-only threshold bypasses, obstacle-sonar enable/disable,
stop release, and coordinated restart. Pending actions and unknown controller
states are shown explicitly; a BLE connection alone does not confirm a setting.
Tab headers have no focus outline and are skipped during keyboard Tab traversal.
Buttons and the BLE text area also omit visual focus outlines; buttons retain
keyboard activation. Panel borders and obstacle colors remain visible and do
not change when focused. Mouse tab selection still works.
The BLE tab's **Clear logs** button clears displayed messages. Incoming messages
continue to appear, and the saved runtime log remains available for diagnosis.
The display keeps the newest 250 lines, dropping older lines as new messages arrive.

LiDAR read-only switches keep measurements visible and bypass only the selected
front or rear distance thresholds. Either or both may be selected without
globally blocking motion, cancelling a pending Start, or disabling a new Start.
The normal readiness, emergency-stop, side-clearance and missing/stale-data
gates still apply. The Pi turns a queued path into STOP if its source scan
deadline passes before the BLE write. The current 2D LiDAR detects generic
returns in its scan plane; it does not recognize people or establish stopping
distance. See the [bypass correction](../docs/bypass-performance-2026-10-31.md).

**EMERGENCY STOP** latches a stop locally and in both supported controllers.
It cancels queued starts/returns and aborts the current route. Clear LiDAR scans,
BLE reconnection and ordinary return/reset commands cannot release this latch.
The Pi remembers it in `~/.config/garby/operator-stop` (override with
`GARBY_OPERATOR_STOP_FILE`). The controllers retain their stop marker across
software restarts. **Release stop** requires confirmed stationary IDLE and
controller acknowledgements. Release grants no motion: a new manual start is
required. **Continue run** (2026-11-14) has the same preconditions plus an executor
report of a stopped route (`RESUME=1`), asks for confirmation, and releases the stop
into that route; it moves only after fresh path clearance. Keep the stop-state file when deploying or restarting the Pi runtime.
The adjacent `.request-id` file reserves monotonic request IDs in batches to
survive process restarts and Pi clock rollback; keep that file too.

**Ultrasonic** controls the motor executor's servo-mounted obstacle sonar. A
confirmed disable cancels active pings, bypasses that sonar's obstacle gate and
holds its servo at `DEFAULT_VIEW` (the existing firmware angle). Confirmed sonar
disable does not itself block motion or manual start. LiDAR, link and MPU gates
still apply. The separate bin-level ultrasonic sensor keeps operating.
Changes require a ready, stationary IDLE executor. Re-enabling requires fresh
clear samples before sonar can grant clearance. Sonar returns to enabled after
an executor restart. A rejected, expired or uncertain action is never displayed
as a confirmed change.

**Restart Pi + MCUs** requires a confirmed emergency stop and a confirmation
dialog. It restarts the motor executor and BLE bridge, then asks the Pi OS for
an orderly reboot only after the matching controller acknowledgement. It does
not reboot the separate bin/gas board, whose reset-capable firmware is absent.
It does not use a forced OS reboot or change permissions. The desktop/service
user must already have permission to run `systemctl --no-ask-password reboot`;
if permission is denied, the monitor reports the failure and keeps STOP latched.
A lost acknowledgement may leave only the controllers restarted; no Pi reboot
is attempted without confirmation. After restart, release STOP explicitly and
start a new route. Ensure the Pi runtime is configured to launch after boot.

These software controls depend on working BLE/UART communication. An offline
stop is saved locally and sent on reconnection, and existing link watchdogs
remain active; the screen must not be interpreted as proof of a physical stop
until controller confirmation arrives. Use the hardware power/stop mechanism
when communication or the controller itself is unavailable.

The Controls tab shows **All-front** and **All-back** toggle buttons. Each starts
with **checks on**. Click to switch that group to **read only**, and click again
to restore its thresholds:

- All-front covers front, front-left, and front-right.
- All-back covers back, back-left, and back-right.
- Distances keep updating in read-only mode. That group's close readings no
  longer report a LiDAR obstacle in the path packets sent to the controller.
- Side obstacles, missing/stale scans, connection watchdogs, and the executor's
  enabled local obstacle sonar still stop motion. These toggles affect the selected
  LiDAR distance thresholds only. Both groups return to checks on when the Pi
  process restarts.

**Manual disposal** asks **Start manual disposal?** before starting the normal outbound route from
IDLE. It bypasses the weight and bin/gas trigger thresholds; it does not need a
valid scale reading, sensor-board data, or Firebase. Automatic triggering still
works as before. The route continues to use its normal motion and obstacle gates.

The button enables only after the Pi's BLE readiness write, the bridge/executor
handshake, and fresh executor readiness plus IDLE reports have completed. A BLE
connection alone is insufficient. Readiness expires after 7.5 seconds without
confirmation; the bridge queries executor status every 5 seconds. Pending
starts disable the button, and the monitor shows acceptance or rejection.
Unsent starts expire after 2 seconds; missing acknowledgements time out after
5 seconds and require a fresh state report. Starts are never retried
automatically or carried across a BLE disconnect/controller restart. Request
IDs prevent duplicates from starting another cycle after a return.

While a written start is pending, the Pi asks `[START STATUS:<id>]` once per
second, within that same five-second deadline. This only queries the result:
an accepted start returns its remembered ACK, while an undelivered start returns
`NOT_RECEIVED`. The START action is never resent. Temporary query transport
failures leave the result pending. A busy bridge UART rejects a new start with
`UART_BUSY`; an executor with an uninitialized motor channel rejects it with
`MOTORS_UNAVAILABLE`.

After acceptance, the monitor shows **Motors held: ...** if the executor is
waiting for clearance, including stale LiDAR, the MPU or local obstacle sonar.
These reasons and start transmission/acceptance/rejection also appear in the
terminal log. A missing `/dev/ttyAMA0` is the separate bin/gas sensor link and
does not prevent this manual start. MPU/path/local obstacle stops still apply.
The executor enters its main safety/recovery loop immediately after hardware
setup instead of waiting up to eight seconds for BLE.

This feature requires the updated firmware in both
[`BLE_Receiver-Final`](../BLE_Receiver-Final/BLE_Receiver-Final.ino) and
[`MAIN_EXECUTOR_CODE`](../MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.cpp), together with
the complete updated `RasPi/` directory. Rebuild/upload both firmware projects
and restart the Pi runtime. Older firmware cannot supply the required readiness
and start acknowledgements.

In headless mode with an interactive terminal, use `f` / `b` for the groups,
`s` then `y` within 10 seconds for confirmed manual disposal, and `q` to quit.
Any other confirmation key cancels. A system service has no interactive keys.
The UI preview demonstrates the controls using sample data only.

The Android app can request the same route through `devices/garby-bin-01/commands/dispose`.
`disposal_support.py` polls in a dedicated worker, validates a 30-second request deadline,
and durably claims each request before queuing one START. Recovery only queries the recorded
controller ID. The executor's `[DISPOSAL READY:1]` capability and fresh IDLE/readiness are
required for cloud requests. Cloud writes never supply GO. Deploy the whole `RasPi/`
directory, including this helper. Protocol, Firebase setup and acceptance steps:
[`docs/manual-disposal-2026-11-13.md`](../docs/manual-disposal-2026-11-13.md).

## Important SMS notices

The Pi now reports internet, Firebase, LiDAR, bin-sensor and thermal/power health
to the executor's cellular modem. Sustained faults and confirmed recoveries
produce bounded SMS notices alongside emergency-stop, route-fault and controller
link events. Internet probing runs in its own daemon thread; Firebase failure
alone is not labeled internet loss. Unknown/stale data cannot report recovery.

SMS uses the existing recipient in `MAIN_EXECUTOR_CODE.h`. The modem owns its
UART in one background worker, with a ten-slot queue, at least 20 seconds
between attempts and six attempts per ten minutes. No automatic resend follows
an uncertain post-submission result. SMS does not need Pi internet, but it still
needs cellular registration, SIM service and power. Full cellular/power loss
cannot send an immediate notice. See [SMS behavior and validation](../docs/sms-notices-2026-10-30.md)
for timing, delivery limits and acceptance checks. Deploy both updated MCU
sketches and the complete Pi directory together.

## Useful environment variables

- `GARBY_SENSOR_SERIAL_PORT` — sensor UART device; detected from its data when unset
- `GARBY_LIDAR_PORT` — LiDAR device; detected from its data when unset
- `GARBY_SENSOR_SERIAL_BAUD` — default `9600`
- `GARBY_BLE_ADDRESS` — optional fixed BLE MAC/address
- `GARBY_BLE_DEVICE_NAME` — default `GarbyESP32`
- `GARBY_LIDAR_YAW_OFFSET_DEG` — physical LiDAR yaw correction; default `0`, must be hardware-verified
- `GARBY_CHASSIS_WIDTH_CM` (60), `GARBY_CHASSIS_FRONT_CM` (30), `GARBY_CHASSIS_REAR_CM` (30), `GARBY_LIDAR_Y_OFFSET_CM` (0), `GARBY_ROTATION_X_CM` (0) — chassis outline relative to the LiDAR for the chassis zones and wall fits (`hallway_geometry.py`); unmeasured defaults except the width, see `docs/navigation-2026-11-11.md`
- `GARBY_LIDAR_DTR` — YDLIDAR motor-DTR support; default `false` for the X3
- `GARBY_LIDAR_FIXED_RESOLUTION` — default `0`. The YDLIDAR SDK's fixed resolution sizes
  every scan from a startup mean rounded to tens and drops the rest of each revolution
  (2026-11-12: 660 bins against 666-667 points, logged as `Real points 666 > fixed points 660`).
  Off, each point keeps its own angle and the scan length follows motor speed
- `GARBY_FIREBASE_CREDENTIALS` — optional Firebase service-account JSON path;
  otherwise discover the key in `RasPi/Firebase/`
- `GARBY_FIREBASE_DATABASE_URL` — override RTDB URL

Do not change UUIDs, path packet format, stale timeout, or LiDAR orientation without updating `../SYSTEM_ARCHITECTURE.md` and re-running the coordinated audit.

## LiDAR data and logging

`/scan` must provide a nonzero acquisition timestamp in the same ROS clock as
the bridge. Acquisition age counts toward the existing 0.8 second safety
timeout; delivery of an old scan cannot refresh permission to move. Missing,
malformed, duplicate, backward, or more than 0.1 second future timestamps fail
closed. A paused ROS clock cannot refresh repeated acquisitions. Ordinary
driver restarts recover on a fresh advancing acquisition timestamp. After a
ROS clock rollback, hold the robot stopped until time catches up or restart
the bridge to establish a new clock epoch. Scan quality
and obstacles are checked together in periodic and urgent path packets.

Lateral `L`/`R` wall estimates require at least three finite returns in the
centered 22 degree cone. They do not borrow wider sector rays or treat no-return
infinity as a wall. Other sectors retain no-return clearance at `range_max`.
Currently missing steering fields are omitted immediately; recovered estimates
do not mix measurements from before a dropout or stream outage. The mounting
yaw and the chassis left/right convention still require physical verification.

Production steering also sends CF/CL/CR nearest safety distances for front, left
and right groups, independent of wall medians. A group is omitted when any
constituent sector lacks current coverage. The S3 requires these fields and the
uint32 acquisition ID A before nudging; repeated polls retain A. Its centering
and correction confirmation cannot count the same scan several times. Deploy
the Pi and S3 changes together. See [steering stability](../docs/nudging-2026-10-30.md)
for the contract, gyro-rate early release and physical acceptance limits.

The production scan and BLE send paths do not print each sample or packet.
Driver console output is suppressed unless `GARBY_LIDAR_DRIVER_LOGS=1` is set.
Routine status polls are excluded from the GUI log, whose scrollback is capped
at the newest 250 lines. Fault and connection state messages remain available.

## Boot startup installation

The system service runs headless without desktop login, sources ROS and the
LiDAR overlay, checks dependencies/devices before launching, and retries every
10 seconds if preflight or the process fails. Stopping the service stops its
child driver processes too. See the [systemd service reference](https://www.freedesktop.org/software/systemd/man/latest/systemd.service.html).

From the repository root:

```bash
sudo install -m 600 RasPi/garby-bridge.env.example /etc/default/garby-bridge
sudoedit /etc/default/garby-bridge
sudo python3 RasPi/install_startup.py --user garby
sudo systemctl start garby-bridge
systemctl status garby-bridge --no-pager
journalctl -u garby-bridge -b -f
```

Do not overwrite an existing configured environment file. Set the actual ROS
setup (this machine has `/opt/ros/jazzy/setup.bash`), built YDLIDAR overlay,
Python interpreter with the requirements installed, and distinct accessible
LiDAR/sensor serial ports. If the driver is installed in the ROS underlay,
set `GARBY_LIDAR_WORKSPACE_SETUP=` explicitly. This does not install the driver.
A virtual environment used for ROS needs access to the ROS Python packages.
Use the private Firebase credential path if cloud operation is required.

The installer uses this checkout's absolute path, validates the generated unit,
and enables boot startup. It does not start or restart an existing process;
the explicit start command above does that. Reinstall after moving the checkout.
For review without installation:

```bash
python3 RasPi/install_startup.py --output /tmp/garby-bridge.service
systemd-analyze verify /tmp/garby-bridge.service
```

Disable startup with `sudo systemctl disable --now garby-bridge`.
Preflight can be run manually with the configured environment exported:
`bash RasPi/run_bridge.sh --check-runtime`.

On 2026-10-29, this machine's preflight found missing `bleak`, no discoverable
`ydlidar_ros2_driver`, and no configured accessible LiDAR/sensor serial devices.
ROS Jazzy and a powered Bluetooth adapter were available. Startup was **not**
installed/enabled: system installation needs sudo authentication, and runtime
setup remains incomplete. These observations are not a physical robot test.

## Testing without the LiDAR or robot

From the repository root, run `python3 -B RasPi/check_lidar_offline.py`.
This exercises production scan handling with synthetic raw scans and substituted
peripherals. It requires no ROS or BLE connection and sends no robot commands.
It checks chair-like obstacles, missing/invalid scans and clearance recovery.

Run `python3 -B RasPi/check_firebase.py` for an authenticated Firebase read using
the discovered/configured key and a Python with firebase-admin installed.
Use `--credentials /absolute/path/service-account.json` to select another key,
or `--anonymous` for an endpoint-only probe. With no key configured or discovered,
the default probe also falls back to the anonymous endpoint. Add `--write-test` for an isolated
diagnostic write/read/delete round-trip. No command or live telemetry paths are
written. Exit 2 means authenticated verification is incomplete.

See [whole-project functionality results](../FUNCTIONALITY_TEST_RESULTS.md)
for all commands, scope, recorded results and the remaining hardware checks.

## Runtime optimization update

Headless mode skips display frames while retaining all scan safety decisions.
Firebase return polling uses 10 seconds idle and 2 seconds at the destination
or during a request. The cloud badge reflects actual completed operations and
pending writes. Writer shutdown stops retries at its flush deadline.
See [optimization results](../OPTIMIZATION_RESULTS.md) for validation and limits.

## Field diagnostics (2026-11-04)

`[BRIDGE ...]` lines, `BLE link lost: ...` and `Pi health changed: ...` always reach the log,
with or without Firebase. Health is sampled every 5 s. Without `/dev/vcio` access (`video` group)
only undervoltage is visible, which also feeds the `[HEALTH]` thermal state and SMS power alarm;
the service example adds that group. Status snapshots requested by the controller are capped at
one per 150 ms, and urgent STOP packets bypass the cap. `bash run_bridge.sh --check-ble` scans for
the bridge; add `--connect` only with the wheels lifted.
