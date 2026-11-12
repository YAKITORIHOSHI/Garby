# Whole-project functionality check — 2026-10-29

## Latest scoped update — 2026-10-30, steering stability

The MPU/LiDAR steering revision passes **256 Pi tests** with no Tk skips and
**407 firmware host checks**. Its actual S3 target compiles with matching source
hashes (586872 bytes flash, 37908 bytes globals). The unchanged executor source
matches its earlier verified target build. See
[current validation](artifacts/nudging-2026-10-30/validation.md) and
[behavior and physical acceptance](docs/nudging-2026-10-30.md). No firmware
upload, runtime restart, motion or physical accuracy acceptance was performed.

## Earlier scoped update — 2026-10-30, GUI/operator/SMS

The GUI/operator controls and expanded SMS revision passed **241 Pi tests**,
**316 firmware host checks**, Python syntax and structural checks. Both actual
ESP32 firmware targets have successful builds matching current source hashes.
See [current software/build evidence](artifacts/sms-notices-2026-10-30/validation.md),
[operator controls](docs/gui-operator-controls-2026-10-30.md), and
[SMS operation](docs/sms-notices-2026-10-30.md). The Pi suite includes Tk desktop
tests without skips. This update did not change or revalidate Android builds.
No firmware upload, live SMS, robot operation or physical acceptance was performed.

The remainder is the dated 2026-10-29 whole-project report, whose counts and
machine observations describe that earlier session rather than current state.

Scope: the whole supplied project, including the Pi runtime, main executor,
BLE/MPU bridge, Firebase data contracts, Android source/configuration and startup
launcher. No ESP32 target build, firmware flashing, Android build or physical
movement was performed. Host C++ tests execute the real firmware source against
simulated peripherals; they are not ESP32 firmware builds.

## Results

| Area | Result | Evidence / boundary |
| --- | --- | --- |
| Python runtime and helpers | 145 tests passed | Production parsing, queues, watchdogs, transport recovery, UI, launcher and Firebase contracts |
| Firmware behavior | 138 checks passed | Actual executor/bridge/MPU source with host peripherals; includes complete outbound/return route |
| Source integrity | Passed | Required renamed files, C/C++ structure and JSON parsing |
| Android source/configuration | Static check passed | 22 Kotlin files; resources/configuration and control invariants; Android execution/unit tests not run |
| Pi/Android Firebase contract | Passed | Same database URL and device ID; telemetry mirrors, timestamps and command identity checked |
| LiDAR without hardware | 8 scenario tests passed | Production scan callback receives synthetic 360-ray scans; no robot connections |
| Firebase logic offline | 7 contract tests passed | Real manager mapping, asynchronous writer retry and transaction callbacks; network boundary substituted |
| Firebase probe handling | 3 tests passed | Denied anonymous reads, missing credentials, cleanup after diagnostic read-back failure |
| Firebase live endpoint | Reachable | HTTPS GET of `/RASPI/STATES/lastSeen.json` succeeded without authentication; returned value omitted |
| Firebase authenticated operations | Unverified | Service-account file unavailable and firebase-admin absent in system Python; no live writes performed |
| Startup service | Unit/shell validation passed | Generated unit validated with systemd-analyze; not installed or enabled |
| Machine runtime readiness | Not ready | Missing bleak, YDLIDAR package/overlay, configured accessible LiDAR/sensor serial devices and Firebase setup |

The 8 LiDAR, 7 Firebase contract and 3 probe tests are included in the 145 Python
count, not additional tests. The runtime tests intentionally inject failures;
expected ERROR log messages from those injections are not failing assertions.
The legacy audit script is absent from this workspace and remains unverified.

## Test without the LiDAR

From the repository root:

```bash
python3 -B RasPi/check_lidar_offline.py
```

This runs without ROS, bleak, Firebase, a LiDAR, or connected boards. It reuses
the production runtime through the existing peripheral-substitution fixture.
Scenarios cover absent scans, clear scans, single chair-leg returns at multiple
front angles, rear obstacles, close side obstacles, invalid returns, stale scans,
and clearance recovery. It cannot measure real LiDAR orientation, sensing height,
chair visibility, body clearance, or braking. It does not publish ROS scans or
send GO commands to a robot. Production continues to stop when scans are absent.

The older `final_w_serial-simulator.py` is a separate interactive bench program
that can connect to BLE/Firebase; it is not the offline regression command above.

Full repeatable host validation:

```bash
python3 -B tools/verify_project.py --host-only
```

## Firebase checks

Endpoint-only probe (no credentials, no writes):

```bash
python3 -B RasPi/check_firebase.py
```

Exit 2 explicitly means authenticated testing is incomplete, even if HTTPS is
reachable. The successful anonymous GET establishes access only to the tested
heartbeat path; it does not establish other read/write permissions or sign-in.

When the service-account file and firebase-admin are available in the selected
Python, run:

```bash
python3 -B RasPi/check_firebase.py --credentials /absolute/path/service-account.json
python3 -B RasPi/check_firebase.py --credentials /absolute/path/service-account.json --write-test
```

The second command writes, reads back and deletes a unique record under
`/_diagnostics/garby-check/<uuid>`, then verifies its absence. It never writes
robot telemetry or return commands. A cleanup failure reports the exact scratch
path. The authenticated path was checked with substitutes, not a live credential.
An Admin SDK success would not establish Android user authentication or client
security-rule behavior. See the [Firebase Admin Database documentation](https://firebase.google.com/docs/database/admin/start).

## Evidence and remaining acceptance

Logs: [whole project](artifacts/functionality-2026-10-29/whole-project.log),
[offline LiDAR](artifacts/functionality-2026-10-29/lidar-offline.log),
[live Firebase probe](artifacts/functionality-2026-10-29/firebase-live.log),
[machine preflight](artifacts/functionality-2026-10-29/runtime-preflight.log).

Physical UART/BLE timing, wheel motion, servo travel, MPU installation, sonar
coverage and sensor power/wiring still need hardware acceptance. Automatic chair
bypass remains unimplemented; current behavior stops and waits for clearance.
Android sign-in, live dashboard rendering, client permissions and live return
requests require a configured device/test account. No return request was sent.
