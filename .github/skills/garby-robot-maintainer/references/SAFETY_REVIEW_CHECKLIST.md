# GARBY Safety Review Checklist

Use this checklist for full audits, release reviews, protocol changes, and any issue involving STOP, steering, braking, or sensor timing.

## Table of contents

1. [Project and version control](#1-project-and-version-control)
2. [Safety ownership](#2-safety-ownership)
3. [STOP and GO protocol](#3-stop-and-go-protocol)
4. [Watchdogs and freshness](#4-watchdogs-and-freshness)
5. [BLE and UART transport](#5-ble-and-uart-transport)
6. [LiDAR obstacle safety](#6-lidar-obstacle-safety)
7. [Hallway centering and anti-zigzag](#7-hallway-centering-and-anti-zigzag)
8. [Servo and ultrasonic](#8-servo-and-ultrasonic)
9. [Motor movement and braking](#9-motor-movement-and-braking)
10. [State machine and route](#10-state-machine-and-route)
11. [Performance and blocking work](#11-performance-and-blocking-work)
12. [Software validation](#12-software-validation)
13. [Hardware validation](#13-hardware-validation)
14. [Documentation and release](#14-documentation-and-release)

## 1. Project and version control

- [ ] Identify the exact Pi, bridge, controller, route, and architecture files intended for deployment.
- [ ] Remove or clearly separate obsolete version-suffixed copies.
- [ ] Confirm all nodes use the same packet protocol revision.
- [ ] Confirm `pointsRun.ino` is included in the controller build.
- [ ] Record board core, library versions, and build environment.
- [ ] Compare source constants against `SYSTEM_ARCHITECTURE.md`.

## 2. Safety ownership

- [ ] Raspberry Pi produces path and geometry state; it does not command motors directly.
- [ ] BLE bridge is the sole raw navigation parser and nudge decision maker.
- [ ] Main controller executes only guarded STOP, GO, nudge, sensor, and reset commands.
- [ ] No second component independently makes hallway-centering decisions.
- [ ] GUI, Firebase, telemetry, and SMS are not motion authorities.

## 3. STOP and GO protocol

- [ ] Controller initializes with STOP active.
- [ ] Bridge initializes with STOP latched.
- [ ] Only a valid path packet can produce GO.
- [ ] `SENSOR:` processing returns before path logic.
- [ ] Unknown packets are ignored or fail closed.
- [ ] Malformed path packets produce STOP, not GO.
- [ ] Old path packets cannot overwrite a newer STOP.
- [ ] Steering packets must match the newest accepted path sequence.
- [ ] Bridge requires multiple clear packets.
- [ ] Controller requires multiple GO packets.
- [ ] STOP clears pending nudge state.
- [ ] Nudge commands are rejected during STOP or stale path state.
- [ ] BLE disconnect produces STOP and never automatic return.
- [ ] Explicit reset is preceded by STOP.

## 4. Watchdogs and freshness

- [ ] Pi converts stale LiDAR into a stale path packet.
- [ ] Pi stale timeout is appropriate for the actual LiDAR frequency.
- [ ] Bridge path timeout is approximately motion-scale, not tens of seconds.
- [ ] Controller independently checks path-command freshness.
- [ ] General link timeout is documented as connection recovery only.
- [ ] Timeout arithmetic uses unsigned elapsed-time subtraction.
- [ ] Reconnection begins fail-closed and requires fresh confirmations.

## 5. BLE and UART transport

- [ ] Safety packets are queued before steering and telemetry.
- [ ] Latest path state replaces older queued path state.
- [ ] A newer STOP cannot remain behind an older queued clear packet.
- [ ] GATT writes are serialized.
- [ ] Safety/control writes use acknowledgment where supported.
- [ ] Queue lengths are bounded.
- [ ] UART baud rates match at both ends.
- [ ] UART line buffers are bounded and reset on overflow.
- [ ] High-rate request and ACK logging is suppressed in normal operation.
- [ ] Legacy parsing is isolated from new protocol dispatch.

## 6. LiDAR obstacle safety

- [ ] Front and back cone definitions match physical LiDAR orientation.
- [ ] Front threshold is supported by measured stopping distance.
- [ ] Close front obstacles bypass slow outlier confirmation.
- [ ] Missing required obstacle data fails closed.
- [ ] Runtime safety-disable controls cannot bypass active-operation protection.
- [ ] LiDAR history access is thread-safe.

## 7. Hallway centering and anti-zigzag

- [ ] Left/right wall estimates reject isolated close points.
- [ ] Centered dead zone prevents constant corrections.
- [ ] Direction-reversal hysteresis prevents hunting.
- [ ] Several consecutive packets confirm a correction direction.
- [ ] Cooldown and settle intervals prevent overlapping taps.
- [ ] Nudge duration and intensity are proportional and bounded.
- [ ] Front-obstacle proximity suppresses steering.
- [ ] Side-wall proximity prevents steering into a nearby wall.
- [ ] Doorway and protrusion handling are present.
- [ ] Left/right execution is symmetric unless measured calibration is documented.
- [ ] Logs include error, direction, duration, intensity, and sequence during tuning.

## 8. Servo and ultrasonic

- [ ] Servo endpoints avoid mechanical binding.
- [ ] Servo uses command -> settle -> ping order.
- [ ] Settling time accounts for angular travel.
- [ ] Echo timeout is bounded and shorter than the intended schedule.
- [ ] Center samples receive stop priority.
- [ ] Very-close obstacles stop immediately.
- [ ] Normal obstacles require confirmation without excessive delay.
- [ ] Resume requires repeated local-clear samples and fresh LiDAR clearance.
- [ ] Long nested rescan delays are absent.
- [ ] Automatic reverse retreat is absent unless separately justified and tested.
- [ ] Ultrasonic side nudging is disabled by default.
- [ ] Servo uses a dedicated regulated supply with common ground and local capacitance.

## 9. Motor movement and braking

- [ ] Every movement and turn passes through a fresh-path motion gate.
- [ ] Moving loops continue polling UART and enforcing the watchdog.
- [ ] Safety stop uses controlled deceleration.
- [ ] Normal endpoint stop is gentler than safety stop.
- [ ] Hard force stop is only a timeout fallback.
- [ ] No reverse braking pulse is present without measured justification.
- [ ] STOP clears active nudge state.
- [ ] Theoretical speed and braking calculations are labeled estimates.
- [ ] Actual stopping distance is measured with maximum payload and low traction.

## 10. State machine and route

- [ ] IDLE requests status often enough to keep telemetry and link state current.
- [ ] Load and gas triggers require confirmation.
- [ ] `runStart()` executes once per outbound cycle.
- [ ] Destination behavior remains stationary while waiting for explicit return.
- [ ] Link loss does not transition to RETURNING.
- [ ] RETURNING still requires live motion permission.
- [ ] Outbound and return directions are verified with wheels lifted.
- [ ] Rear protection matches the actual return direction.

## 11. Performance and blocking work

- [ ] LiDAR callback avoids network, modem, sleep, and explicit garbage-collection work.
- [ ] Firebase updates are throttled and exception-safe.
- [ ] GUI refresh is decoupled from LiDAR rate.
- [ ] SMS work runs outside the motion-safety loop.
- [ ] `pulseIn()` timeouts do not exceed the scheduler budget.
- [ ] No multi-second `delay()` exists in a moving or halted safety polling loop.
- [ ] Shared state uses locks, events, or message passing.
- [ ] Repeated dynamic string growth is bounded on ESP32.

## 12. Software validation

- [ ] Run `scripts/audit_project.py`.
- [ ] Run Python bytecode compilation.
- [ ] Compile both ESP32 projects in the actual toolchain.
- [ ] Test telemetry during STOP.
- [ ] Test malformed and unknown messages.
- [ ] Test stale and out-of-order path packets.
- [ ] Test stale and mismatched steering packets.
- [ ] Test all clear-confirmation layers.
- [ ] Test nudge rejection while STOP is active.
- [ ] Test Pi queue coalescing.
- [ ] Test BLE disconnect and reconnect.
- [ ] Test LiDAR topic loss.
- [ ] Test explicit reset ordering.

## 13. Hardware validation

- [ ] Verify a latching hardwired emergency-stop.
- [ ] Verify motor drivers default disabled on reset and brownout.
- [ ] Perform all first tests with wheels lifted.
- [ ] Verify servo endpoints and power integrity.
- [ ] Verify sonar against known target distances.
- [ ] Measure braking with empty and maximum payload.
- [ ] Measure low-battery and lowest-traction conditions.
- [ ] Test open doors, pillars, wall fixtures, people at the side, narrow corridors, and wide corridors.
- [ ] Record worst-case stopping distance and apply a safety margin.
- [ ] Preserve logs and final tuned constants with the deployed firmware version.

## 14. Documentation and release

- [ ] Update `SYSTEM_ARCHITECTURE.md` for every material behavior change.
- [ ] Update packet tables and constants.
- [ ] Explain compatibility and deployment order.
- [ ] Include patch notes and validation results.
- [ ] Distinguish host-tested behavior from hardware-tested behavior.
- [ ] State unresolved physical assumptions explicitly.
