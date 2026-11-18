# Completed route endpoint reporting — 2026-11-17

Owner request: GARBY records that a completed outbound run reached point B (the disposal
area), and that a completed return reached point A (its station). This document covers the
robot part; mobile source remains in its own existing app checkout.

## Source behavior

The executor already emits `[OUTBOUND COMPLETE]` only after the outbound route succeeds.
The Pi now includes its endpoint in the existing asynchronous Firebase status update.

The executor adds a retained `returnComplete` flag, initially false. The main loop calls
`fullReset(true)` only after `returnToPointB()` returns success, including the same route
resumed with Continue. A fault, emergency stop or pending abort vetoes that success report.
It emits `[IDLE]` followed by `[RETURN COMPLETE]`, and repeats this pair in status responses
while the flag remains set. Every new outbound, continued/reverse route and developer motion
clears it before movement; ordinary `fullReset()` and MCU boot clear it as well. The bridge's
existing generic `handleMcuLine()` → `sendNotification()` path forwards the new line without
modifying either safety gates or bridge source.

The Pi accepts return arrival only with an IDLE state. A generic IDLE, boot, operator stop or
release, RESET ACK/DONE, command timeout, route fault or developer completion is insufficient.
The correlated return command retains its existing independent request-ID transaction and
ACK/DONE/FAILED contract. Arrival telemetry never acknowledges or replays a command.

## Firebase fields

All fields are under `devices/garby-bin-01/status/`, written by the Pi:

| Field | Values/meaning |
| --- | --- |
| `location` | `POINT_A`, `POINT_B`, `IN_TRANSIT` or `UNKNOWN` |
| `isAtStation` | True only when the current reported endpoint is `POINT_A` |
| `locationUpdatedAt` | Pi epoch milliseconds when the current location value changed |
| `lastArrival` | `{point, name, confirmedAt, source}` for the most recent confirmed route endpoint |

Point A is **Station** and point B is **Disposal area**. `source` is `RETURN_COMPLETE` or
`OUTBOUND_COMPLETE`. `confirmedAt` is the Pi's observation time, not a timestamp generated
by the MCU at the physical endpoint. Repeated unchanged reports do not refresh it. A
disconnect/restart may need a new executor status confirmation and therefore a new time.

`OUTBOUND`, `RETURNING`, `REVERSING` and `TESTING` clear the current endpoint to `IN_TRANSIT`.
Faults, controller reboot and link uncertainty set it to `UNKNOWN`. IDLE preserves a known
stationary endpoint and otherwise stays unknown. `lastArrival` remains the last historical
arrival when current location changes and is not erased on Pi process startup.

These are route execution reports. Open-loop wheel steps cannot demonstrate wheel motion,
absence of slip, physical straightness, stopping distance or the exact parking position.

## Verification and deployment

The original four Pi contract tests failed on missing location fields before the production
change. Exact commands, results, source/binary hashes, patch and manifest are saved in
`artifacts/mobile-route-2026-11-17/`. The final verification summary is `robot-checks.json`.
The tests exercise production notification parsing, the coalesced Firebase writer, initial
and repeated completion, stale completion during motion, reset/boot/fault refusal, continued
return success, clearing before new route/developer motion and real forwarding through both
controller sources with substituted peripherals.

The actual executor target compiles as the saved flasher board selection: ESP32 Dev Module,
4 MB flash, PSRAM disabled (`esp32:esp32:esp32:FlashSize=4M,PSRAM=disabled`). Build path is
outside the sketch at `/tmp/garby-arrival-20261117-executor`; persistent binary hashes and
the application image are saved in the artifact directory. Saved state reports no connected
MCU/port; it is not a live USB observation. No compile command contains an upload option.

**MCU upload requirement: Executor only**, complete `MAIN_EXECUTOR_CODE` sketch, build
`ARRIVAL-20261117`. The preceding MPU/direction/navigation source is included in the current
sketch; do not upload an individual header or intermediate image. Earlier bridge firmware
may independently remain pending. **Pi deployment:** deploy the entire `RasPi/` directory,
also `ARRIVAL-20261117`. No hardware was uploaded or operated and no live database, rules,
service or robot command was changed in this work. Installed versions and physical acceptance
remain unknown.
