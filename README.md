# GARBY — Autonomous Waste-Collection Robot

GARBY is a three-node distributed embedded robotic system for autonomous
waste collection in corridor environments. It is the subject of an engineering
thesis project and consists of an ESP32 motor controller, an ESP32 BLE bridge,
a Raspberry Pi supervisor/sensor hub, and an Android companion app.

> **Safety is fail-closed.** Missing, stale, malformed, out-of-order, or
> disconnected path data always stops motion. The Android app never authorizes
> motion — it only monitors and sends a reset/return intent that the firmware
> and Pi must still execute safely.

## Repository Layout

```text
GARBY/
├─ .github/                    # Agent definitions & skills for this repo
│  ├─ agents/
│  └─ skills/garby-robot-maintainer/   # SKILL.md, agents, references, scripts
├─ Garby_MobileApp/            # Android companion app (Kotlin, Firebase RTDB)
├─ SINGLE-TASK-DEFAULT/        # Active single-task (main loop) firmware variant
│  ├─ BLE_Receiver-Final/      # ESP32 BLE bridge sketch + docs
│  ├─ NAPHTALI_CODE_V2/        # Main ESP32 MCU motor controller (active source)
│  ├─ RasPi/                   # Raspberry Pi ROS 2 LiDAR + serial sensor node
│  ├─ SYSTEM_ARCHITECTURE.md   # Full system architecture
│  ├─ DEPLOYMENT_AND_ACCEPTANCE.md
│  └─ VALIDATION_RESULTS.md
├─ GARBY-NOTIF-TRIG-HOST (EXPERIMENT)/  # Experimental Firebase Functions host
└─ .vscode/                    # Editor configuration
```

## Architecture

```
Android App  ──Firebase RTDB──▶  Raspberry Pi
                                  │ ROS 2 YDLidar
                                  │ Serial sensor board
                                  │ Firebase sync
                                  ↓ BLE
                        ESP32 BLE Bridge
                        (path parser + nudge calc)
                                  ↓ UART
                        Main ESP32 MCU
                        (motor execution, servo, HX711, Air780E SMS)
```

### Node Breakdown

| Node | Source | Role |
|---|---|---|
| **Main ESP32 MCU** | `SINGLE-TASK-DEFAULT/NAPHTALI_CODE_V2/` | Stepper motor execution, servo+ultrasonic obstacle check, HX711 load cell, Air780E SMS, state machine (`IDLE`/`RUNNING`/`RETURNING`) |
| **ESP32 BLE Bridge** | `SINGLE-TASK-DEFAULT/BLE_Receiver-Final/` | NimBLE server, path packet parser, lane-centering/anti-zigzag nudge algorithm, UART relay to MCU |
| **Raspberry Pi** | `SINGLE-TASK-DEFAULT/RasPi/final_w_serial.py` | ROS 2 LiDAR reader, Pi serial sensor reader, Firebase heartbeat/sync, BLE client, reset command bridge |
| **Android App** | `Garby_MobileApp/` | Telemetry dashboard, sensor monitoring, explicit reset intent |

## Documentation

- [**System Architecture**](SINGLE-TASK-DEFAULT/SYSTEM_ARCHITECTURE.md) — detailed design, watchdogs, BLE packet protocol, Firebase schema
- [Deployment & Acceptance](SINGLE-TASK-DEFAULT/DEPLOYMENT_AND_ACCEPTANCE.md)
- [Validation Results](SINGLE-TASK-DEFAULT/VALIDATION_RESULTS.md)
- [RasPi README](SINGLE-TASK-DEFAULT/RasPi/README.md)

## Key Protocols

**MCU commands (from BLE bridge over UART):**
```
STOP, STOP:HUMAN, STOP:STALE, STOP:LINK, STOP:WAITING_DATA, STOP:PROTOCOL, GO
N:<ms>:<intensity>|<NUDGE_LEFT|NUDGE_RIGHT|STABLE>
SENSOR:US=..|MQ4=..|MQ137=..|MQ135=..
[RESET]
[IDLE]
```

**BLE packet protocol (Pi → bridge):**
```
P:<seq>|F=<C|O|H|S>|B=<C|O|H|S>
S:<seq>|L=..|R=..|F=..|B=..|FL=..|FR=..|BL=..|BR=..|T=..
SENSOR:US=..|MQ4=..|MQ137=..|MQ135=..
[RESET]
[RASPI READY]
```

## Development

- **MCU firmware**: Arduino IDE / PlatformIO targeting ESP32
- **RasPi supervisor**: `pip install -r requirements.txt` (ROS 2, Bleak, Firebase)
- **Android app**: Android Studio / Gradle, target SDK 33+

## License

This is a thesis research project. See individual components for their own
licensing notes.
