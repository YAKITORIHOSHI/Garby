# GARBY Validation Results — 2026-09-26 Throttling Resilience & Safety Validation

This report covers the active root deployment set. Generated build/cache
directories are not production source. Static and target builds do not replace
physical safety acceptance.

## Host validation

Raspberry Pi syntax and tests:

```bash
python -B -m unittest discover -s RasPi -p "test_*.py" -v
```

Result: **32/32 PASS**. Coverage includes sensor unavailable/recovery states,
backoff/coalescing under CPU contention, LiDAR sector assignment, closest-return safety,
median wall steering, heading clamping, infinity handling, sparse steering fallback,
telemetry/path isolation, direction-independent side-obstacle envelope,
adaptive watchdog tolerance under CPU/connection throttling (jittered cadence,
bursty executor deliveries, boundary conditions, step-convergence), connection
retry backoff pacing, telemetry cadence stretching, and thermal/voltage throttling flags.

Coordinated static audit:

```bash
python -B .github/skills/garby-robot-maintainer/scripts/audit_project.py .
```

Result: **72 PASS / 1 WARN / 0 FAIL**. The single remaining warning is expected: physical
route direction remains hardware-unverified (`pointsRun.ino`), documented in Section 13.
Package hygiene passed with zero stale compiled Python cache files.

C/C++ and JSON structural sanity:

```bash
python -B tools/source_sanity.py .
```

Result: **PASS** (6 C/C++ files, 18 JSON files).

## CPU and Connection Throttling Resilience

- **Pi LiDAR Node:** Adaptive staleness tolerance (0.8 s floor, 2.0 s cap) with EWMA cadence
  estimation. Outage recovery resets tolerance to the conservative 0.8 s floor.
- **BLE Service:** Connection interval updated to 30–50 ms (24–40), reducing radio wakeups;
  write retries pace exponentially (50 ms → 800 ms, 5 strikes) before session reset;
  initial reconnect starts after 1.0 s.
- **ESP32 BLE Bridge:** Adaptive ingress freshness window (650 ms floor, 1550 ms cap) drains
  valid packets during CPU stalls; bounded inter-arrival period prevents outage inflation;
  stale watchdog resets window to floor.
- **ESP32 MCU Controller:** Adaptive path watchdog (800 ms floor, 1200 ms cap) prevents spurious
  emergency stops on throttled bridge loops; UART RX budget enlarged to 512 bytes matching
  the hardware buffer to absorb processing bursts without data loss.
- **MCU Return-Route Fault Latch:** Incomplete return route segments latch stationary and fail-closed;
  cleared explicitly by `fullReset()` on completed return or operator `[RESET]`.

## Target compilation (arduino-cli)

Both sketches compiled for `esp32:esp32:esp32` (ESP32 Dev Module, Core v3.3.11):
- `BLE_Receiver-Final`: 610,180 bytes (46%) flash, 39,428 bytes (12%) RAM. 0 errors.
- `NAPHTALI_CODE_V2`: 378,438 bytes (28%) flash, 26,216 bytes (8%) RAM. 0 errors.

## Android validation (Garby_MobileApp)

Android unit tests executed via Gradle:
```bash
./gradlew test --rerun-tasks
```
Result: **13/13 unit tests passed** (ExampleUnitTest, SensorStatusTest, DatabaseSchemaTest, ResetStatusTest, SensorFreshnessTest). Build successful.

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
