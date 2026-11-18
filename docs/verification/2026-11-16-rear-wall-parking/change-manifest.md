# Exact change manifest — rear-wall parking, 2026-11-16

Changes are measured against the saved pre-task hashes, because this workspace has no Git metadata.
All production targets identify as `REARPARK-20261116`. MCU upload requirement: **Both**; Pi deployment: **entire RasPi directory**.
No upload, service restart, live robot command or physical acceptance was performed. Installed image identity is unknown.

## Edited existing files

- [RasPi/final_w_serial.py](../../../RasPi/final_w_serial.py)
- [MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.h](../../../MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.h)
- [MAIN_EXECUTOR_CODE/HeadingControl.h](../../../MAIN_EXECUTOR_CODE/HeadingControl.h)
- [MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.cpp](../../../MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.cpp)
- [MAIN_EXECUTOR_CODE/pointsRun.ino](../../../MAIN_EXECUTOR_CODE/pointsRun.ino)
- [BLE_Receiver-Final/SteeringAssist.h](../../../BLE_Receiver-Final/SteeringAssist.h)
- [BLE_Receiver-Final/BLE_Receiver-Final.ino](../../../BLE_Receiver-Final/BLE_Receiver-Final.ino)
- [tests/pi/test_reverse_parking.py](../../../tests/pi/test_reverse_parking.py)
- [tests/firmware/test_bridge.cpp](../../../tests/firmware/test_bridge.cpp)
- [tests/firmware/hallway_sim.h](../../../tests/firmware/hallway_sim.h)
- [tests/firmware/test_route_reverse.cpp](../../../tests/firmware/test_route_reverse.cpp)
- [tests/firmware/test_system.cpp](../../../tests/firmware/test_system.cpp)
- [README.md](../../../README.md)
- [SYSTEM_ARCHITECTURE.md](../../../SYSTEM_ARCHITECTURE.md)
- [RELEASE_NOTES.md](../../../RELEASE_NOTES.md)
- [DEPLOYMENT_AND_ACCEPTANCE.md](../../../DEPLOYMENT_AND_ACCEPTANCE.md)
- [AGENTS.md](../../../AGENTS.md)

## Created source and design files

- [RasPi/rear_wall.py](../../../RasPi/rear_wall.py)
- [tests/firmware/test_rear_parking.cpp](../../../tests/firmware/test_rear_parking.cpp)
- [docs/rear-wall-parking-2026-11-16.md](../../rear-wall-parking-2026-11-16.md)

## Created verification artifacts

Earlier failed checks are retained as regression evidence; `checks.json` identifies the final passing runs.

- [baseline.json](baseline.json)
- [binary-hashes.json](binary-hashes.json)
- [bridge-target-final.log](bridge-target-final.log)
- [bridge-target.log](bridge-target.log)
- [checks.json](checks.json)
- [executor-target-final.log](executor-target-final.log)
- [executor-target.log](executor-target.log)
- [firmware-before.log](firmware-before.log)
- [firmware-final.log](firmware-final.log)
- [firmware-focused.log](firmware-focused.log)
- [firmware-suite-final.log](firmware-suite-final.log)
- [firmware-suite.log](firmware-suite.log)
- [nudge-model.log](nudge-model.log)
- [pi-before.log](pi-before.log)
- [pi-final.log](pi-final.log)
- [pi-focused.log](pi-focused.log)
- [pi-suite.log](pi-suite.log)
- [review-final-holds-session.log](review-final-holds-session.log)
- [review-final-stale-gate.log](review-final-stale-gate.log)
- [review-stale-gate-before.cpp](review-stale-gate-before.cpp)
- [review-stale-gate-before.log](review-stale-gate-before.log)
- [source-hashes.json](source-hashes.json)
- [source-sanity.log](source-sanity.log)
- [source.diff](source.diff)
- [system-focused.log](system-focused.log)
- [unchanged-behavior.json](unchanged-behavior.json)

- [handoff.json](handoff.json)

## Validation

- Pi: 634 tests, 41 skips (Tk display and optional separate Android interoperability), exit 0.
- Firmware: 12 actual-source host executables and 825 passing assertions, exit 0.
- Source sanity and both documented actual ESP32 targets: exit 0.
- Independent nudge measurements and parking stale-gate/session/low-power review: PASS.
- Production operator/developer functions, forward drive, plain drive and common motion gates are byte-identical to baseline; see unchanged-behavior.json.

## Compiled artifacts

Binary and ELF files are stored outside the repository at `~/.local/share/garby-builds/rear-wall-parking-2026-11-16/`; hashes and exact paths are in binary-hashes.json.
Verified production board configurations come from README.md and DEPLOYMENT_AND_ACCEPTANCE.md. Saved flasher state had no selection/connection; no port was used.

The unmeasured mount yaw/body outline/rear gap, low objects below the rear scan plane, wheel slip, actual stopping distance and battery savings remain physical limits.
