# GARBY — Autonomous Waste-Collection Robot

GARBY is a three-node distributed embedded robotic system for autonomous waste collection in indoor corridor environments. Developed as an engineering thesis project, it integrates a main ESP32 motor controller, an ESP32 BLE bridge, a Raspberry Pi 4 supervisor/sensor hub, and an Android companion app.

> **Safety Invariant:** The system is strictly **fail-closed**. Missing, stale, malformed, out-of-order, or disconnected path data immediately halts motion. The Android app monitors telemetry and sends reset/return intents; it never directly authorizes movement.

---

## System Architecture

```text
Android Companion App (Kotlin / Jetpack Compose)
       │
       │ Firebase Realtime Database
       ▼
Raspberry Pi 4 Supervisor
  ├── ROS 2 YDLIDAR (/scan processing & safety gating)
  ├── Serial Sensor Board (MQ-4, MQ-135, MQ-137 gas + US)
  └── BLE Central Client
       │
       │ BLE GATT (P: path packets, S: steering, SENSOR: telemetry)
       ▼
ESP32 BLE Bridge
  ├── NimBLE Server & strict packet parser
  ├── Hallway-centering & anti-zigzag nudge algorithm
  └── Fail-closed STOP latch
       │
       │ UART @ 115200 baud (STOP/GO, N:<ms>:<pct>, SENSOR)
       ▼
Main ESP32 Motor Controller
  ├── FastAccelStepper closed-loop stepper control
  ├── Centered Servo + Ultrasonic obstacle detection
  ├── HX711 Load Cell (bin capacity)
  ├── Air780E LTE Modem (asynchronous SMS alerts)
  └── IDLE / RUNNING / RETURNING route state machine
```

---

## Repository Layout

```text
GARBY/
├── BLE_Receiver-Final/        # ESP32 BLE bridge firmware (NimBLE, nudge calc, STOP latch)
├── NAPHTALI_CODE_V2/          # Main ESP32 MCU firmware (steppers, sonar, HX711, Air780E)
├── RasPi/                     # Raspberry Pi ROS 2 LiDAR supervisor & sensor bridge
│   ├── bridge_core.py         # Core LiDAR parsing, coalescing mailbox & health helpers
│   ├── final_w_serial.py      # Main production ROS 2 node & BLE client
│   ├── requirements.txt       # Python dependencies
│   └── test_bridge_core.py    # Host unit test suite (32 test cases)
├── Garby_MobileApp/           # Android companion app (Kotlin, Firebase RTDB)
├── tools/                     # Host sanity and verification scripts
│   └── source_sanity.py       # Structural syntax and JSON integrity validator
├── SYSTEM_ARCHITECTURE.md     # Authoritative system design & protocol specification
├── DEPLOYMENT_AND_ACCEPTANCE.md# Hardware checklist & supervised deployment runbook
├── VALIDATION_RESULTS.md      # Static, unit test, and compilation verification record
├── AUDIT_REPORT.md            # Coordinated multi-node audit & defect repair log
├── RELEASE_NOTES.md           # Multi-node throttling resilience & safety release notes
└── .github/                   # Copilot/agent definitions & maintainer skills
```

---

## Node Summary

| Node | Primary Files | Role |
| :--- | :--- | :--- |
| **Main ESP32 MCU** | `NAPHTALI_CODE_V2/` | Stepper execution (`FastAccelStepper`), centered ultrasonic interlock, load cell (`HX711`), background cellular modem (`Air780E`), route state machine (`IDLE` &rarr; `RUNNING` &rarr; `RETURNING`). |
| **ESP32 BLE Bridge** | `BLE_Receiver-Final/` | NimBLE GATT server, uint32 sequence validation, hallway-centering nudge calculation, UART relay at 115200 baud to main MCU. |
| **Raspberry Pi 4** | `RasPi/final_w_serial.py`<br>`RasPi/bridge_core.py` | ROS 2 YDLIDAR reader, sensor UART bridge (9600 baud), Firebase sync, BLE client, reset command bridge. |
| **Android App** | `Garby_MobileApp/` | Real-time telemetry dashboard, sensor visualization, explicit reset and route return trigger. |

---

## Key Protocols

### 1. BLE Packet Protocol (Raspberry Pi &rarr; BLE Bridge)
```text
P:<seq>|F=<C|O|H|S>|B=<C|O|H|S>     # Path safety packet (C=Clear, O=Obstacle, H=Human, S=Stale)
S:<seq>|L=..|R=..|F=..|B=..|...     # Steering geometry (bound to matching path seq)
SENSOR:US=..|MQ4=..|MQ137=..|MQ135=..# Environmental telemetry (US=999, gas=-1: unavailable)
[RASPI READY]                       # Pi bridge liveness beacon
[RESET]                             # Explicit operator reset intent
```

### 2. UART Command Protocol (BLE Bridge &rarr; Main MCU @ 115200 baud)
```text
STOP, STOP:HUMAN, STOP:STALE, STOP:LINK, STOP:WAITING_DATA, STOP:PROTOCOL
GO                                  # Released only after 2+ consecutive clear packets
N:<duration_ms>:<pct_cut>|<DIR>     # Steering correction tap (e.g., N:45:12|NUDGE_LEFT)
SENSOR:...                          # Relayed sensor telemetry
[MCU READY]                         # Main MCU boot epoch notification
[REQUEST-STATUS]                    # Main MCU status request probe
```

---

## Quick Start & Verification

### 1. Raspberry Pi Runtime
```bash
cd RasPi
python3 -m pip install -r requirements.txt
python3 final_w_serial.py --headless
```

### 2. Host Validation & Static Audit
```bash
# Run unit tests and syntax checks
cd RasPi
python3 -m unittest -v test_bridge_core.py
python3 -m py_compile bridge_core.py final_w_serial.py final_w_serial-simulator.py test_bridge_core.py
cd ..

# Run structural sanity and coordinated multi-node audit
python3 tools/source_sanity.py .
python3 .github/skills/garby-robot-maintainer/scripts/audit_project.py .
```

### 3. Android Companion App
```powershell
cd Garby_MobileApp
.\gradlew.bat testDebugUnitTest assembleDebug assembleRelease
```

---

## Documentation

- [Release Notes](RELEASE_NOTES.md) — Multi-node throttling resilience, return-route fault latching, and verification results.
- [System Architecture](SYSTEM_ARCHITECTURE.md) — Comprehensive technical architecture, watchdogs, timing parameters, and fail-closed state machines.
- [Deployment & Acceptance Runbook](DEPLOYMENT_AND_ACCEPTANCE.md) — Step-by-step flashing order, wheels-lifted verification, and physical acceptance checklist.
- [Validation Results](VALIDATION_RESULTS.md) — Host test results, static audit outputs, and target compilation details.
- [Audit & Repair Report](AUDIT_REPORT.md) — In-depth investigation of communication, watchdog, and parser repairs.
- [Raspberry Pi Node Guide](RasPi/README.md) — Pi environment setup, ROS 2 configuration, and systemd service installation.

---

## Hardware Safety Notice

> [!WARNING]
> Software verification and unit tests cannot certify real-world physical safety. Hardware safety depends on measured stopping distances under load, verified LiDAR orientation, confirmed route directions on lifted wheels, electrical/power integrity, and a verified hardwired emergency stop. Always conduct initial testing with wheels lifted before floor operations.
