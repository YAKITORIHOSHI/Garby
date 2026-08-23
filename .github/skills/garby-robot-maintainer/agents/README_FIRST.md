# GARBY Maintainer Files — Read First

This repository contains the deployable GARBY runtime plus project-local maintenance guidance.

The authoritative runtime baseline is the root `SYSTEM_ARCHITECTURE.md`. The project-local skill reference at `references/SYSTEM_ARCHITECTURE.md` is synchronized from that file for assistant/tooling use.

## Deployable runtime

- `RasPi/final_w_serial.py`
- `RasPi/bridge_core.py`
- `BLE_Receiver-Final/BLE_Receiver-Final.ino`
- `NAPHTALI_CODE_V2/NAPHTALI_CODE_V2.ino`
- `NAPHTALI_CODE_V2/NAPHTALI_CODE_V2.cpp`
- `NAPHTALI_CODE_V2/NAPHTALI_CODE_V2.h`
- `NAPHTALI_CODE_V2/pointsRun.ino`

The simulator, `.github/`, `.vscode/`, audit scripts, and Markdown files are development/support material, not firmware payloads.

## Before deployment

1. Run `python3 .github/skills/garby-robot-maintainer/scripts/audit_project.py .`.
2. Run the Pi unit tests from `RasPi/`.
3. Compile both ESP32 projects using the exact target board core and installed libraries.
4. Follow `DEPLOYMENT_AND_ACCEPTANCE.md` with wheels lifted first.
5. Record physical LiDAR yaw, route direction, braking distance, payload, battery, and floor condition.

A clean source audit is not hardware certification.
