# GARBY — autonomous waste-collection robot

GARBY combines a Raspberry Pi LiDAR supervisor, an ESP32 BLE bridge, an ESP32
motor controller, and an Android Firebase dashboard. It follows calibrated
outbound and return routes for an indoor waste-collection thesis project.

The 2026-10-02 repair addresses obstacle recovery, nudging, motor command
acceptance, BLE transport, and trustworthy return status across all four nodes.
Software checks are recorded in [Validation results](VALIDATION_RESULTS.md).
Physical operation still needs the [deployment and acceptance procedure](DEPLOYMENT_AND_ACCEPTANCE.md).

## Start here

- [Full project rundown](PROJECT_RUNDOWN.md): intended cycle, confirmed defects,
  repairs, deployment boundaries, and remaining work.
- [Architecture](SYSTEM_ARCHITECTURE.md): active source, protocols, timing,
  sensors, motor parameters, and state ownership.
- [Release notes](RELEASE_NOTES.md): compatibility and behavior changes.
- [Deployment and acceptance](DEPLOYMENT_AND_ACCEPTANCE.md): Pi setup, flashing
  order, lifted-wheel tests, and supervised route checks.
- [Validation results](VALIDATION_RESULTS.md): commands and measured software results.

## Active deployment set

| Node | Source | Role |
| --- | --- | --- |
| Pi | `RasPi/final_w_serial.py`, `bridge_core.py` | ROS 2 scans, path permission, geometry, BLE transport, telemetry, return relay |
| BLE ESP32 | `BLE_Receiver-Final/` | Strict path parser, STOP latch, centering decisions, controller UART |
| Main ESP32 | `NAPHTALI_CODE_V2/` | Motor targets, local sonar, load/gas triggers, route state, background SMS |
| Android | `Garby_MobileApp/app/` | Authenticated dashboard and correlated operator return requests |

Deploy the entire Pi directory. `Garby_MobileApp/final_w_serial.py` is only a
compatibility launcher. `RasPi/final_w_serial-simulator.py` is a manual bench
producer with simplified sensing; `Test/Test.ino` is a serial test sketch.

Motor execution is open loop: there are no wheel encoders in this source.
LiDAR supplies the outer centering correction. An ordinary obstacle wait keeps
monitoring and resumes the remaining target after fresh clearance. Route faults
require supervised recovery from a known physical position.

## Verify the software

Run from the repository root:

```powershell
python -B tools/verify_project.py
```

This runs Python runtime/helper tests, executable tests of the actual firmware
source, structural/audit checks, both ESP32 target builds, Android unit tests,
debug/release assembly, and lint. ESP32 binaries go to `artifacts/firmware/`.
It does not flash connected boards. See the validation report for tool versions.

For a machine without ESP32/Android toolchains:

```powershell
python -B tools/verify_project.py --host-only
```

Skipped checks are explicitly reported as unverified.

## Run on the Pi

Install ROS 2 and the YDLIDAR workspace, then install `RasPi/requirements.txt`
into the Python that can import ROS packages. Configure the environment from
`RasPi/garby-bridge.env.example`; source both ROS and the workspace overlay.

```bash
python3 RasPi/check_runtime.py --require-firebase
bash RasPi/run_bridge.sh --headless
```

Follow the deployment guide for service installation and device paths. Verify
LiDAR orientation, distinct LiDAR/sensor serial devices, actual wheel directions,
and echo measurements before floor operation.

Missing or stale path data stops movement. The app and telemetry cannot grant
motion permission. Hardware acceptance includes a hardwired emergency stop and
measured braking under the intended payload.
