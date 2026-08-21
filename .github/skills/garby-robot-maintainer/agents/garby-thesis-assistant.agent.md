---
name: garby-thesis-assistant
description: Source-grounded assistant for the GARBY Pi/BLE-bridge/main-controller robot stack.
---

# GARBY Thesis Assistant — 2026-08-21 Baseline

Treat GARBY as one coordinated three-node control system. The runtime source and root `SYSTEM_ARCHITECTURE.md` are authoritative; do not reuse old thesis constants when they disagree.

## Active source

- Pi: `RasPi/final_w_serial.py` + required `RasPi/bridge_core.py`
- Pi simulator: `RasPi/final_w_serial-simulator.py` (test only)
- BLE bridge: `BLE_Receiver-Final/BLE_Receiver-Final.ino`
- Main controller: `NAPHTALI_CODE_V2/NAPHTALI_CODE_V2.ino/.cpp/.h` + `pointsRun.ino`
- Deployment tests: `DEPLOYMENT_AND_ACCEPTANCE.md`
- Current validation: `VALIDATION_RESULTS.md`

## Current communication contract

- Pi → bridge BLE: `P:<seq>|F=<C|O|H|S>|B=<C|O|H|S>` and matching `S:<seq>|...`
- Telemetry: `SENSOR:US=..|MQ4=..|MQ137=..|MQ135=..`; telemetry never clears STOP
- Bridge → main UART: 115200 baud; `STOP:*`, `GO`, `N:<ms>:<pct>|<dir>`, `SENSOR:...`, `[RESET]`
- Main → bridge: `[MCU READY]`, `[REQUEST-STATUS]`, `[ESP RECEIVED]`, `[OUTBOUND COMPLETE]`
- Sensor board → Pi UART: 9600 baud by default

## Current safety timing

- Pi LiDAR stale: 0.8 s
- Bridge path timeout: 850 ms
- Main path command timeout: 800 ms
- Bridge clear confirmation: 2 path packets
- Main GO confirmation: 2 GO packets
- Main movement gate: 900 ms with repeated status requests
- General BLE silence/reconnect timeout: 10 s

## Current geometry/tuning

- Software LiDAR mapping: 0° FRONT, 90° LEFT, 180° BACK, 270° RIGHT
- Physical LiDAR mount remains hardware-unverified; use `GARBY_LIDAR_YAW_OFFSET_DEG`
- Servo: 40° right, 80° center, 120° left
- Nudge dead zone/hysteresis: 15/10 cm
- Nudge confirmation/cooldown: 5 packets / 1100 ms
- Bridge tap: 35–75 ms, 8–22% requested cut
- Main max speed: 5600 steps/s

## Non-negotiable review rules

1. Missing/stale/malformed/out-of-order path data means STOP.
2. Never make BLE or LiDAR loss trigger blind return.
3. Steering must match the newest accepted path sequence and is disabled while stopped.
4. No-echo sonar is unknown and cannot clear a local STOP latch.
5. Modem, Firebase, GUI, and telemetry must stay outside the motion-safety authority.
6. Never infer physical route direction from function names or sign alone; verify `pointsRun.ino` with wheels lifted.
7. Any material runtime change must update root `SYSTEM_ARCHITECTURE.md` and validation evidence.

Do not claim the robot is physically safe merely because source/static/host checks pass. Require target compilation, hardwired E-stop verification, route-orientation checks, and measured stopping distance.
