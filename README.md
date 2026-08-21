# GARBY — Autonomous Waste-Collection Robot

[![Safety Standard: Fail-Closed](https://img.shields.io/badge/Safety-Fail--Closed-red.svg)](SYSTEM_ARCHITECTURE.md)
[![Architecture: Distributed 3-Node](https://img.shields.io/badge/Architecture-3--Node%20Distributed-blue.svg)](SYSTEM_ARCHITECTURE.md)
[![Platform: ROS 2 + ESP32 + Android](https://img.shields.io/badge/Platform-ROS%202%20%7C%20ESP32%20%7C%20Android-green.svg)](DEPLOYMENT_AND_ACCEPTANCE.md)

**GARBY** is a three-node distributed embedded robotic system designed for autonomous indoor waste collection in corridor environments. It is developed as an engineering thesis research platform featuring autonomous navigation, obstacle classification, multi-gas air quality monitoring, bin capacity measurement, remote telemetry, and emergency fail-safe state machines.

---

## Table of Contents

- [System Overview](#system-overview)
- [Safety & Invariants](#safety--invariants)
- [Hardware Architecture](#hardware-architecture)
- [Repository Structure & Descriptions](#repository-structure--descriptions)
- [Subsystem Breakdown](#subsystem-breakdown)
  - [1. Main ESP32 Motor Controller (`NAPHTALI_CODE_V2`)](#1-main-esp32-motor-controller-naphtali_code_v2)
  - [2. ESP32 BLE Bridge (`BLE_Receiver-Final`)](#2-esp32-ble-bridge-ble_receiver-final)
  - [3. Raspberry Pi Supervisor (`RasPi`)](#3-raspberry-pi-supervisor-raspi)
  - [4. Android Companion App (`Garby_MobileApp`)](#4-android-companion-app-garby_mobileapp)
  - [5. Validation & Audit Tools (`tools`)](#5-validation--audit-tools-tools)
- [Communication & Protocols](#communication--protocols)
  - [BLE Packet Specification (Pi -> Bridge)](#ble-packet-specification-pi---bridge)
  - [UART Packet Specification (Bridge -> MCU)](#uart-packet-specification-bridge---mcu)
- [Getting Started & Deployment](#getting-started--deployment)
- [Technical Documentation](#technical-documentation)
- [License & Thesis Notice](#license--thesis-notice)

---

## System Overview

GARBY operates across three distinct processing nodes working in tight coordination:

```
+-------------------+       Firebase RTDB       +-------------------+
|    Android App    | <=======================> |   Raspberry Pi    |
| (Telemetry & UI)  |                           | (LiDAR, ROS 2,    |
+-------------------+                           |  Safety & Sync)   |
                                                +---------+---------+
                                                          |
                                                          | BLE (GATT)
                                                          v
                                                +---------+---------+
                                                | ESP32 BLE Bridge  |
                                                | (Path & Nudge)    |
                                                +---------+---------+
                                                          |
                                                          | UART (115200)
                                                          v
                                                +---------+---------+
                                                |  Main ESP32 MCU   |
                                                | (Motors, Sonar,   |
                                                |  HX711, Air780E)  |
                                                +-------------------+
```

---

## Safety & Invariants

> **Safety is strictly fail-closed.**  
> Any missing, stale, malformed, out-of-order, or disconnected path communication immediately halts the robot. Motion authorization requires independent, fresh, repeated confirmations across all nodes.

1. **Dual Freshness Gates**: The BLE bridge latches `STOP` if path packets are interrupted for >400 ms. The Main MCU requires fresh `GO` UART heartbeats (<600 ms).
2. **LiDAR Hallway Centering**: Path clearance and lane-centering are governed by LiDAR 360-degree sector analysis. Ultrasonic sonar provides near-field emergency stopping.
3. **No Autonomous Reset on Link Loss**: If communication drops, the robot safely stops in place and awaits connection recovery or authorized human intervention.
4. **App Never Directly Commands Motion**: The Android companion app is strictly a telemetry viewer and reset requester; it cannot drive or bypass hardware interlocks.

---

## Hardware Architecture

| Component | Specifications / Model | Function |
|---|---|---|
| **Supervisor Computer** | Raspberry Pi 4 Model B (4GB / 8GB) | ROS 2 navigation node, YDLidar parser, Firebase sync, BLE client |
| **LiDAR Sensor** | YDLidar (X4 / TG30 / equivalent 360° LiDAR) | Real-time hallway clearance and obstacle detection |
| **BLE Bridge MCU** | ESP32 DevKit v1 (WROOM-32) | Dedicated NimBLE server, nudge calculation, UART protocol gateway |
| **Main Motor MCU** | ESP32 DevKit v1 (WROOM-32) | Stepper pulse timing, state machine, safety sonars, load cell |
| **Motor Drivers** | TB6600 / TMC2209 Stepper Drivers | Microstepping control for differential drive motors |
| **Front Sonar Array** | HC-SR04 on SG90 / MG90S Servo | Forward sweep obstacle detection (near-field safety) |
| **Weight & Capacity** | HX711 24-bit ADC + 4x Load Cells | Real-time trash bin load measurement and capacity thresholding |
| **Cellular Gateway** | Air780E LTE Cat-1 Modem | Autonomous SMS notification alerts on bin overflow / full state |
| **Environmental Gas** | MQ-4 (Methane), MQ-135 (Air Quality), MQ-137 (Ammonia) | Multi-gas real-time hazard monitoring |

---

## Repository Structure & Descriptions

```text
GARBY/
├── BLE_Receiver-Final/        # ESP32 BLE bridge firmware (NimBLE server & nudge logic)
├── Garby_MobileApp/           # Modern Android companion app (Kotlin, Compose, Firebase)
├── NAPHTALI_CODE_V2/          # Main ESP32 MCU motor controller & hardware drivers
├── RasPi/                     # Raspberry Pi ROS 2 LiDAR hub, supervisor, and bridge core
├── tools/                     # Codebase sanity checks and structural verification scripts
├── .github/                   # Repository automation, maintainer skills, and agent configs
├── SYSTEM_ARCHITECTURE.md     # Authoritative system architecture & interface contracts
├── DEPLOYMENT_AND_ACCEPTANCE.md # Hardware wiring, flashing, and acceptance test plan
├── VALIDATION_RESULTS.md      # Test records, communication latencies, and unit tests
├── AUDIT_REPORT.md            # Comprehensive stability, protocol, and code audit log
└── README.md                  # Project overview and technical documentation
```

---

## Subsystem Breakdown

### 1. Main ESP32 Motor Controller (`NAPHTALI_CODE_V2`)
- **Location**: [`NAPHTALI_CODE_V2/`](NAPHTALI_CODE_V2/)
- **Core Files**: `NAPHTALI_CODE_V2.ino`, `NAPHTALI_CODE_V2.cpp`, `NAPHTALI_CODE_V2.h`, `pointsRun.ino`
- **Responsibilities**:
  - Precision stepper pulse generation using hardware timer interrupts.
  - State machine management: `IDLE`, `RUNNING`, `PAUSED`, `RETURNING`, `OBSTACLE_STOP`, `EMERGENCY_STOP`.
  - Continuous forward sweep via servo-mounted ultrasonic sensor.
  - HX711 weight acquisition and calibration.
  - Asynchronous AT-command handling for Air780E 4G LTE SMS dispatches.
  - UART command reception and fail-safe motion watchdog.

### 2. ESP32 BLE Bridge (`BLE_Receiver-Final`)
- **Location**: [`BLE_Receiver-Final/`](BLE_Receiver-Final/)
- **Core File**: `BLE_Receiver-Final.ino`
- **Responsibilities**:
  - High-throughput NimBLE peripheral server handling GATT characteristics.
  - Parsing and sequence validation of `P:` (Path) and `S:` (Steering) packets.
  - Real-time anti-zigzag corridor centering and nudge impulse generation.
  - Latched `STOP` enforcement when data is stale (>400 ms) or corrupted.
  - Forwarding verified commands (`GO`, `STOP`, `N:...`, `SENSOR:...`) to Main MCU via UART at 115200 baud.

### 3. Raspberry Pi Supervisor (`RasPi`)
- **Location**: [`RasPi/`](RasPi/)
- **Core Files**: `final_w_serial.py`, `bridge_core.py`, `test_bridge_core.py`, `garby-bridge.service.example`
- **Responsibilities**:
  - ROS 2 LiDAR node subscription (`/scan`) for 360-degree point-cloud evaluation.
  - Sector decomposition: Front (`F`), Back (`B`), Left (`L`), Right (`R`), Front-Left (`FL`), Front-Right (`FR`).
  - Classification of sectors into `C` (Clear), `O` (Obstacle), `H` (Human), or `S` (Stale).
  - Serial sensor aggregation (gas concentrations, temperatures, ultrasonic telemetry).
  - Robust Bleak BLE client management with automatic reconnection and sequence numbering.
  - Firebase Realtime Database telemetry synchronization and heartbeat reporting.

### 4. Android Companion App (`Garby_MobileApp`)
- **Location**: [`Garby_MobileApp/`](Garby_MobileApp/)
- **Technologies**: Android 14+ (SDK 34), Jetpack Compose, Kotlin Coroutines, Firebase RTDB.
- **Features**:
  - Real-time visual dashboard for bin weight percentage, gas PPM levels, and battery/health stats.
  - Visual robot state representation (`IDLE`, `COLLECTING`, `RETURNING`, `STOPPED`).
  - Secure, structured return/reset intent dispatches with atomic status handshakes.
  - Push notifications for critical threshold breaches and maintenance alerts.

### 5. Validation & Audit Tools (`tools`)
- **Location**: [`tools/`](tools/)
- **Core File**: `source_sanity.py`
- **Responsibilities**:
  - Host-side syntax, delimiter balance, and preprocessor block verification for C++/Arduino code.
  - JSON schema and configuration validation across the entire repository tree.

---

## Communication & Protocols

### BLE Packet Specification (Pi -> Bridge)
- **Service UUID**: `4fafc201-1fb5-459e-8fcc-c5c9c331914b`
- **Path Characteristic**: `beb5483e-36e1-4688-b7f5-ea07361b26a8`

```text
Path Safety Packet (Every 100-250 ms):
P:<seq>|F=<C|O|H|S>|B=<C|O|H|S>
Example: P:1042|F=C|B=C

Steering Geometry Packet:
S:<seq>|L=<dist>|R=<dist>|F=<dist>|B=<dist>|FL=<dist>|FR=<dist>|BL=<dist>|BR=<dist>|T=<tilt>
Example: S:1042|L=1.20|R=1.25|F=3.50|B=4.10|FL=1.40|FR=1.45|BL=1.35|BR=1.38|T=0.00
```

### UART Packet Specification (Bridge -> MCU)
- **Baud Rate**: 115200 baud, 8N1

```text
Motion Control:
- GO                           -> Fresh path clear confirmation
- STOP                         -> Path blocked or general stop
- STOP:HUMAN                   -> Human detected in path
- STOP:STALE                   -> Path data expired (>400 ms)
- STOP:LINK                    -> BLE connection lost

Steering Taps:
- N:<duration_ms>:<intensity>|<NUDGE_LEFT|NUDGE_RIGHT|STABLE>
Example: N:80:40|NUDGE_RIGHT

Telemetry Relay:
- SENSOR:US=<val>|MQ4=<val>|MQ137=<val>|MQ135=<val>
```

---

## Getting Started & Deployment

### 1. Raspberry Pi Setup
```bash
# Clone the repository
git clone https://github.com/YAKITORIHOSHI/Garby.git
cd Garby/RasPi

# Install Python requirements
python3 -m pip install -r requirements.txt

# Run the safety bridge supervisor
python3 final_w_serial.py --headless
```

### 2. Microcontroller Flashing
- Open `BLE_Receiver-Final/BLE_Receiver-Final.ino` in Arduino IDE or PlatformIO and flash to the BLE Bridge ESP32.
- Open `NAPHTALI_CODE_V2/NAPHTALI_CODE_V2.ino` and flash to the Main Motor ESP32.

### 3. Android Companion App
- Open `Garby_MobileApp/` in Android Studio (Giraffe or newer).
- Place your `google-services.json` in `Garby_MobileApp/app/`.
- Build and run on an Android device running Android 8.0+ (API 26+).

---

## Technical Documentation

Detailed engineering references and test reports are maintained in the repository:

- [**System Architecture (`SYSTEM_ARCHITECTURE.md`)**](SYSTEM_ARCHITECTURE.md) — Comprehensive technical specification, watchdog details, and safety state machines.
- [**Deployment & Acceptance Guide (`DEPLOYMENT_AND_ACCEPTANCE.md`)**](DEPLOYMENT_AND_ACCEPTANCE.md) — Pinout tables, wiring diagrams, calibration procedures, and acceptance criteria.
- [**Validation Results (`VALIDATION_RESULTS.md`)**](VALIDATION_RESULTS.md) — Empirical unit test records, timing logs, and sensor benchmark results.
- [**Audit Report (`AUDIT_REPORT.md`)**](AUDIT_REPORT.md) — Full investigation and defect remediation report.

---

## License & Thesis Notice

This project is part of an academic engineering thesis research study on autonomous mobile robotics for environmental sanitation. All rights reserved. See individual component directories for third-party library licenses.