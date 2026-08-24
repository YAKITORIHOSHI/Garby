# GARBY Validation Results — 2026-08-24 Return-Route Safety Follow-up

This report covers the active root deployment set. Generated build/cache
directories are not production source. Static and target builds do not replace
physical safety acceptance.

## Host validation

Raspberry Pi syntax and tests:

```bash
python3 -m py_compile bridge_core.py final_w_serial.py final_w_serial-simulator.py test_bridge_core.py
python3 -m unittest -v test_bridge_core.py
```

Result: **18/18 PASS**. Coverage includes sensor unavailable/recovery states,
backoff/coalescing, LiDAR sector assignment, closest-return safety, median wall
steering, heading clamping, infinity handling, sparse steering fallback,
telemetry/path isolation, and the direction-independent side-obstacle envelope.

Coordinated static audit:

```bash
python3 .github/skills/garby-robot-maintainer/scripts/audit_project.py .
```

Result: **71 PASS / 2 WARN / 0 FAIL**. The warnings are expected: physical
route direction remains hardware-unverified, and generated Python bytecode is
present from validation runs.

C/C++ and JSON structural sanity:

```bash
python3 tools/source_sanity.py .
```

Result: **PASS** for the active root sources and checked JSON. The checker also
sees the ignored MCU Flasher cache mirror; that mirror is not a deployment
source.

## Nudge and obstacle follow-up

- Bridge nudge tuning: 18 cm dead zone, 12 cm reversal hysteresis, 0.10 EMA,
  35–60 ms taps, 8–18% requested cut, and 1200 ms cooldown.
- Side LiDAR returns at ≤40 cm redirect away from the occupied side; ≤22 cm
  reports a direction-independent STOP for people, bins, and wall-mounted
  extinguishers.
- Centered Servo+Ultrasonic stops on any fresh ≤60 cm echo before a nudge is
  consumed. No echo remains UNKNOWN and cannot clear a local latch.
- The simulator is non-production, but now models the same side hard-stop
  envelope for scenario testing.

## Target compilation

The active main controller compiled in a temporary PlatformIO fixture for
`esp32dev` using ESP32Servo 3.2.1, FastAccelStepper 1.2.7, and HX711 0.6.4.
The BLE bridge compiled for `esp32-s3-devkitm-1` using NimBLE-Arduino 2.5.1.
The main controller used 8.5% RAM and 26.1% flash; the BLE bridge used 10.0%
RAM and 15.5% flash. Strict `-Wall -Wextra` builds produced only third-party
library warnings and no project-source warnings.
No hardware was flashed.

## Return-route safety follow-up

- An incomplete `returnToPointB()` result now latches a stationary route fault.
- The RETURNING state no longer replays the full route from an unknown chassis position.
- Communication and sensor servicing continue while motion remains fail-closed.
- The coordinated audit now fails if the RETURNING route call is no longer protected by the fault latch.

## Android validation

The existing Android validation remains successful: debug unit tests, debug
assembly, release assembly, and the static quality checks passed. Android was
not changed during this follow-up.

## Remaining physical acceptance

Still required before floor operation:

- verify LiDAR yaw and sector orientation;
- verify outbound and return route direction with wheels lifted;
- calibrate motor trim and nudge sign on the real chassis;
- measure stopping distance with payload and low traction;
- verify Servo+Ultrasonic distances, echo voltage, servo endpoints, and power;
- test a person passing on either side, a hallway trash bin, and a wall-mounted
  fire extinguisher;
- verify the hardwired emergency stop and brownout behavior.

Use `DEPLOYMENT_AND_ACCEPTANCE.md` for the supervised acceptance procedure.
