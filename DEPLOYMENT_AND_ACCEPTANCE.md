# GARBY deployment and acceptance — 2026-10-02

Software checks do not establish physical safety. The local serial inventory
showed COM1 only; neither ESP32 was available for flashing in this session.
The physical route direction and LiDAR mount are hardware-unverified.

## Build and prepare

Run from the repository root:

```powershell
python -B tools/verify_project.py
```

Verified target baseline: ESP32 core 3.3.11, NimBLE-Arduino 2.5.1,
FastAccelStepper 1.2.7, ESP32Servo 3.2.1, HX711 0.6.4;
FQBN `esp32:esp32:esp32`. Android uses the checked-in Gradle wrapper, Java21,
and Android SDK36. Host behavioral tests require MSVC C++ Build Tools on Windows
or g++/clang++ on Unix. Full validation exports each sketch's flashing binaries
under `artifacts/firmware/<sketch>/` and builds debug plus unsigned release APKs.

Flash the main controller, then BLE bridge, with wheels lifted and an accessible
hardwired emergency stop. Deploy the entire `RasPi/` directory; a single Python
file is insufficient. Install the new Android APK after updating all nodes.
The correlated return protocol is a coordinated release, not compatible with
old controller return acknowledgements.

## Raspberry Pi configuration

Install ROS2/rclpy, sensor_msgs, and a built YDLIDAR ROS2 driver workspace.
Install `RasPi/requirements.txt` into the Python used by the service. Confirm
that this Python can import ROS packages; an unrelated virtualenv can hide them.

Copy `RasPi/garby-bridge.env.example` to `/etc/default/garby-bridge` and edit every
deployment-specific path. Choose stable serial device paths and verify that
sensor board and LiDAR resolve to **different** devices. Sensor UART = 9600;
bridge-to-main UART = 115200. Set the Firebase service-account path privately;
do not put it into the Android app or source control.

```bash
set -a
source /etc/default/garby-bridge
set +a
source "$GARBY_ROS_SETUP"
source "$GARBY_LIDAR_WORKSPACE_SETUP"
python3 RasPi/check_runtime.py --require-firebase
bash RasPi/run_bridge.sh --headless
```

The launcher sources both ROS and the workspace overlay. Its defaults are
`/opt/ros/humble/setup.bash` and `/home/garby/ros2_ws/install/setup.bash`; override
them for the actual installation. Merely sourcing the ROS distribution does
not expose a workspace-built YDLIDAR package.

Optional service:

```bash
sudo cp RasPi/garby-bridge.service.example /etc/systemd/system/garby-bridge.service
sudo systemctl daemon-reload
sudo systemctl enable --now garby-bridge
journalctl -u garby-bridge -f
```

Adjust User, WorkingDirectory, and ExecStart before installing if your account
or checkout differs. The service starts headless and restarts on process failure.
No echo and missing scans remain fail closed; restarting a process is not proof
that sensors or communication are healthy.

## Wheels-lifted acceptance

Record board IDs, firmware versions, configuration, payload, and serial logs.

1. Verify hardwired emergency-stop action, motor-driver reset/brownout behavior,
   regulated supplies, common ground, UART crossing, and safe sonar echo voltage.
2. Boot MCU/bridge with Pi absent: no wheel motion. Start Pi without scans:
   `STOP:WAITING_DATA` or `STOP:STALE`, never GO.
3. Verify `[MCU READY]`, BLE connection, and repeated-clear startup sequence.
   Restart each node separately and confirm recovery starts stationary.
4. Check LiDAR physical FRONT/LEFT/BACK/RIGHT with a target at known positions.
   Set `GARBY_LIDAR_YAW_OFFSET_DEG` from observation, not assumptions.
5. Confirm both motors accept short target moves. A motor command rejection
   must stop both and report a route fault. Verify physical left/right wheel
   assignment against the nudge commands.
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
10. Check nudges at steady offset and near a side object. A left-side threat
    must request right correction even after leftward EMA history. Confirm
    tap duration, both-wheel restoration, cooldown, and reversal hysteresis.
11. Verify every outbound and return turn/distance with wheels lifted before
    floor tests. The step signs/numbers in `pointsRun.ino` were preserved.
12. At a completed destination, request return from Android. Confirm matching
    ACK, RETURNING, and DONE after arrival. Duplicate the same request ID during
    return: no extra STOP, restart, or second route. A generic `[IDLE]` must
    not report completion of an unrelated request.
13. Request return during incomplete outbound: stationary rejection/fault.
    Force a return segment failure: stationary fault and matching FAILED.
    Retries must not clear a route fault or replay the route.

For a route fault, physically return the robot to its known base, then use the
local supervised `[RESET]` recovery token. Do not use the app to guess its location.

## Reduced-speed floor acceptance

Perform these with supervision and a clear test boundary:

- Measure stopping distance and payload stability for empty/max load, lowest
  expected battery, and lowest-traction floor, including communication loss.
- Confirm the 95 cm front, 35 cm rear, and 22 cm side envelopes cover the whole
  chassis and worst measured stop distance with a margin.
- Confirm motor trim and nudge signs, then evaluate correction strength. Source
  numbers do not prove real steering performance.
- Exercise open doors, wall protrusions, extinguishers, trash bins, narrow/wide
  corridors, and a person passing at the side.
- Complete several full load-trigger and gas-trigger cycles including monitored
  obstacle clearance and operator return.
- Verify live Firebase authorization/authentication, cached-value expiry,
  return readiness, return failure display, and phone reconnection.
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

Sign off with recorded evidence. Until then the software is built and host-tested,
and physical operation remains unaccepted.
