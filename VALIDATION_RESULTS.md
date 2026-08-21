# GARBY Validation Results — 2026-08-21 Coordinated Repair

This file reports only validation actually performed on the current repaired source. Older claims of successful Arduino/PlatformIO builds applied to a previous revision and are **not** carried forward as proof for this release.

## 1. Host validation completed

### Raspberry Pi syntax

Command:

```bash
python3 -m py_compile bridge_core.py final_w_serial.py final_w_serial-simulator.py test_bridge_core.py
```

Result: **PASS**.

### Raspberry Pi helper tests

Command:

```bash
python3 -m unittest -v test_bridge_core.py
```

Result: **17/17 PASS**.

Covered behaviors include sensor unavailable/recovery transitions, bounded exponential backoff, Firebase update coalescing/retry behavior, gap-free LiDAR sector assignment, closest-return safety representation, wall-distance median, heading clamp, positive-infinity LaserScan handling, sparse steering fallback, and telemetry/path isolation helpers.

### Coordinated static audit

Run with:

```bash
python3 .github/skills/garby-robot-maintainer/scripts/audit_project.py .
```

Final result: **71 PASS / 1 WARN / 0 FAIL**, recorded in `AUDIT_RESULTS.json`. The single warning is intentional: physical wheel/chassis route direction in `pointsRun.ino` cannot be verified from source alone.

### C/C++ and JSON structural sanity

`tools/source_sanity.py` reports **PASS** for all five Arduino/C++ source files and all checked JSON files; the output is stored in `SOURCE_SANITY_RESULTS.txt`. This check validates delimiter/preprocessor structure and JSON parseability, not Arduino libraries or target-board semantics.

## 2. Major repaired defects verified in source

- **Hard Pi startup blocker fixed:** `bridge_core.py` is now real source; production no longer depends on cached `.pyc` from another Python version.
- **Blind startup CLEAR fixed:** before the first valid LiDAR scan, path is stale/unknown (`S`).
- **Incomplete LiDAR scan fail-closed fixed:** missing required front/back sectors become `S`.
- **Duplicate `/scan` subscriptions removed:** one sensor-data subscription remains.
- **LaserScan `+inf` semantics fixed:** valid no-return samples map to `range_max` rather than disappearing as missing data.
- **Collision representative made conservative:** closest valid return is used for safety; wall steering remains median-based.
- **BLE connection task recovery fixed:** unexpected connection exceptions cannot leave `_connecting` permanently stuck.
- **Failed path write recovery fixed:** a fresh sequenced path state is requested instead of replaying an old packet.
- **Main startup modem gate removed:** cellular initialization occurs in a background task after `[MCU READY]`.
- **Bridge boot-epoch behavior fixed:** main-controller readiness defaults false; a new `[MCU READY]` resets old navigation permission.
- **Movement-gate timing fixed:** 900 ms gate with repeated status requests supports bridge + MCU repeated-clear confirmation.
- **Sonar timeout false-clear fixed:** no echo is UNKNOWN and cannot clear an obstacle latch.
- **Strict main nudge parsing added:** malformed numeric fields are rejected.
- **Strict bridge path parsing added:** exact `F=`/`B=` fields, duplicate/unknown-field rejection, uint32 sequence overflow rejection.
- **Legacy path parsing hardened:** non-`CLEAR`/non-`BLOCKED...` legacy values fail closed.
- **Telemetry unavailable semantics fixed:** `US=999` and gas `-1` map to unavailable state instead of false full/danger classification.
- **MCU ACK accounting fixed:** one generic ACK consumes one outstanding command rather than clearing the whole backlog.

### Follow-up route/nudge verification

- Main motion and route APIs now propagate completion status through every outbound and return segment.
- An incomplete or unaccepted motor command cannot set `outboundComplete` or emit `[OUTBOUND COMPLETE]`.
- Confirmed nudge direction reversals are executed after restoring both wheel speeds.
- Removed the NimBLE-Arduino 2.5.x deprecation warning for `NimBLEService::start()`.
- Active controller sources passed the structural sanity check after these changes.

## 3. Static review findings

No source evidence was found of:

- BLE-disconnect-triggered automatic return;
- telemetry clearing STOP;
- side-sonar steering being enabled;
- reverse torque pulse in the safety-braking routine;
- multi-second modem delay in the primary motion-safety loop.

A bounded `pulseIn()` remains only in stopped/manual compatibility scan code. The live motion safety path uses the interrupt/state-machine sonar service.

## 4. Not validated in this environment

The current environment does **not** contain `arduino-cli` or PlatformIO and does not have the project’s exact ESP32 board cores/libraries. Therefore the modified BLE bridge and main-controller sketches have **not been compiled here**. Source-level checks are not a substitute for target compilation.

Hardware behavior is also unverified here, including:

- physical LiDAR yaw/mount orientation;
- actual outbound/return route direction;
- motor polarity and wheel trim;
- stopping distance;
- sonar accuracy and echo voltage level;
- servo power integrity/endpoints;
- BLE range/interference;
- Air780E wiring/network behavior;
- brownout/thermal behavior;
- hardwired emergency-stop function.

Use `DEPLOYMENT_AND_ACCEPTANCE.md` before floor operation.
