# GARBY project rundown — 2026-10-02

## Assessment

The codebase could build before this repair, but successful compilation did not
establish correct motion or recovery. The review found interacting defects in
steering, motor-command acceptance, sonar recovery, BLE scheduling, path queueing,
and app return reporting. These have been repaired across the active nodes and
covered by tests that execute the production Python and firmware source.

The user's reported symptoms were unreliable motors, BLE disconnects, critical
nudging problems, and permanent stopping after an obstacle. Source findings
explain several ways those symptoms can arise. Without robot logs, connected
boards, or physical measurements, they are not a diagnosis of every observed
hardware failure. The current result is a coordinated, software-verified release
with an explicit physical acceptance procedure.

## What GARBY is intended to do

GARBY is a fixed-route robot, not a general mapping/path-planning system. The
main ESP32 executes the wheel distances and turns in `pointsRun.ino`; the Pi
classifies LiDAR clearance and supplies corridor geometry. The BLE ESP32
validates that information and decides whether to STOP, GO, or apply a short
centering correction. The local centered servo/ultrasonic is an independent
near-field stop sensor. The Android app monitors Firebase and requests return.

The full cycle is:

1. Boot stationary and establish controller/bridge/Pi readiness.
2. Obtain repeated fresh LiDAR clearance before permitting motion.
3. Trigger outbound on confirmed load or dangerous gas readings.
4. Execute the calibrated route, braking for obstacles or lost communication.
5. Continue sensing while stopped. After the relevant LiDAR and valid local
   sonar clearance confirmations, resume the remaining original segment.
6. Finish outbound and publish destination readiness.
7. Accept an explicit, correlated operator return request.
8. Return under the same safety gates; report completion only after the route
   succeeds, then become IDLE for the next cycle.

Motion permission does not depend on Firebase, the GUI, Android, or modem/SMS
availability. Missing safety information remains STOP. A normal cleared
obstacle is recoverable; a failed motor command or incomplete route is a
stationary fault requiring supervised recovery from a known physical position.

## Component inventory

| Component | Source and interfaces | Role and boundary |
| --- | --- | --- |
| Pi | `RasPi/final_w_serial.py`, `bridge_core.py`; ROS 2 `/scan`, sensor serial 9600 baud, BLE, Firebase | Fresh clearance, geometry, transport, readiness, telemetry; no direct motor output |
| BLE ESP32 | `BLE_Receiver-Final/BLE_Receiver-Final.ino`; NimBLE GATT, UART 115200 baud | Strict framing/sequence checks, independent STOP latch, nudging, controller transport |
| Main ESP32 | `NAPHTALI_CODE_V2.ino/.cpp/.h`, `pointsRun.ino`; FastAccelStepper, sonar/servo, HX711, Air780 | Motion targets, local obstacle protection, route state, load/gas triggers, SMS worker |
| Android | `Garby_MobileApp/app/`; Compose, Firebase Auth/RTDB, FCM receiver | Dashboard, freshness, readiness, correlated operator return intent |
| External sensor board | Firmware not supplied; sensor UART into Pi | Bin-fill ultrasonic and MQ readings; these are telemetry, not path sonar |
| External services | Firebase project/configuration, deployed rules, SIM/network, optional push sender | Must be verified against the actual deployment |

`Garby_MobileApp/final_w_serial.py` now delegates to the authoritative Pi runtime
instead of maintaining another copy. The Pi simulator is a simplified manual
bench producer. It does not validate the production raw-scan geometry.
`Test/Test.ino` is a serial-only test sketch.

## Findings and repairs

### Obstacle stops and permanent waits

In `NAPHTALI_CODE_V2.cpp`, `haltAndWait()` used a fixed decision about whether
local sonar clearance was required and counted cached readings repeatedly.
Bad echoes did not reliably break consecutive-clear evidence. The 8000 µs echo
window also made targets beyond about 1.4 m unavailable, so removing an obstacle
could produce UNKNOWN rather than measurable clearance.

The wait now continues UART, sonar, watchdogs, and status requests, counts
distinct valid samples, and enables the local requirement if a sonar obstacle
appears during a LiDAR wait. No echo preserves an existing obstacle latch.
The listening window is 18000 µs, and raw close echoes stop immediately instead
of waiting for the median to catch up. Five-second diagnostics show path state,
sonar validity/distance, and clear confirmations. Recovered moves retain the
original absolute targets; turns and straight segments use that same recovery.

This does not convert no echo into CLEAR. If the installed sensor cannot obtain
valid clear echoes after removal, the diagnostics and acceptance measurements
must resolve that sensor condition before changing the clearance policy.
Automatic sonar sampling remains centered. LiDAR monitors the wider area;
side-servo nudging remains disabled to preserve one centering authority.

### Nudging

`computeNudgeFromLidar()` selected an avoidance direction, but the transmitted
command could still follow the smoothed centering error's opposite sign.
Duration scaling could underflow when side avoidance overrode a small error.
Cooldown/confirmation returned STABLE, which could cancel an active tap before
its requested duration. The bridge requested up to 320 ms while the controller
capped a tap at 260 ms.

The selected direction now controls the output, scaling is clamped, and waiting
for confirmation/cooldown emits HOLD internally without sending a cancellation.
Unsafe, invalid, or centered geometry still cancels a correction. Confirmation
is bounded and cleared on STOP. Bridge tap duration now agrees with the MCU's
cap. Controller expiry restores both wheel speeds. Actual wheel assignment,
correction sign, strength, and loaded stability need chassis measurements.

### Motors and route state

Target commands were not checked as a pair, and a start could be judged failed
from an immediately sampled `isRunning()` rather than the API's command result.
A rejected command could leave one wheel active. STOP followed by queued GO in
one receive batch could erase the stop condition before the moving loop braked.
A GO after a stale gap could reuse an old confirmation streak.

Both `moveTo()` results now determine acceptance; a rejection stops both.
Target start no longer depends on immediate running-state propagation. STOP
ends its receive batch, and freshness expiration resets GO confirmation.
Outbound completion is explicit and globally tracked. A failed route cannot be
cleared by GO or an app retry, replayed, or treated as a known return position.

Calibrated route distances/signs and motor speeds/trim were preserved because
their physical meaning cannot be inferred from function names. FastAccelStepper
controls pulses without encoders: lost steps, slip, torque, driver setup, and
mechanical direction cannot be proved by these software tests.

### BLE stability and path integrity

BLE callbacks previously performed UART/navigation/notification work. The Pi
could schedule direct urgent writes independently of the mailbox, allow an old
clear snapshot to replace a newer STOP, and process an old client's disconnect
after a replacement session existed. Steering frames could exceed the allowed
write-without-response size even while short path frames succeeded.

Callbacks now post bounded state/data and loop context performs the work.
All production Pi writes share one lock and mailbox; path replacement is
sequence-aware across wrap. Temporary failed clients are cleaned up and stale
disconnect callbacks ignored. Safety/control writes are acknowledged; larger
steering/telemetry frames use acknowledgement when necessary for the negotiated
size. Bleak documents both the no-response size limit and the older-BlueZ
20-byte limitation. [Bleak characteristic API](https://bleak.readthedocs.io/en/latest/api/index.html),
[Bleak write API](https://bleak.readthedocs.io/en/latest/api/client.html).

The bridge rejects control characters, newline injection, trailing fields,
invalid/old/duplicate path sequences, and ingress overflow. These assert STOP.
Telemetry remains unable to authorize motion. Freshness limits no longer grow
with throttling; reconnect retries remain tolerant while the robot is stationary.
Real disconnect causes still require reason codes, radio observations, and
supply measurements during servo/motor loads.

### Pi sensing and deployment

The ROS subscription now holds only the latest scan. Scan processing and status
snapshots use a shared lock. Invalid angular/range metadata fails closed rather
than killing the callback or producing clearance; missing side-sector evidence
blocks both directions. Invalid yaw configuration can log safely during startup.
An explicitly missing LiDAR device cannot silently select the sensor board.
Production GUI safety bypasses remain disabled even in IDLE.

The new read-only preflight checks ROS/Python dependencies, the YDLIDAR overlay,
distinct accessible serial devices, and optional Firebase configuration.
The launcher/service source both ROS and the workspace overlay. Those checks
must run on the Pi; the local Windows host does not establish Pi runtime readiness.

### Return requests and Android truthfulness

The old workflow could ACK a queued request before MCU acceptance, treat generic
IDLE as completion, interrupt an accepted return with retries, resurrect a
terminal request through a legacy boolean, or expire a long return at the
short acceptance/app timeout. Cached readiness could enable return while the
robot or connection was unavailable.

Return now carries the server request timestamp through Firebase, Pi, bridge,
and MCU. ACK/DONE/FAILED must match that ID. Duplicate IDs return the remembered
outcome without replaying motion. Periodic queries of the same ID recover a
completion notification lost at BLE disconnect/reconnect. New commands require
completed outbound; incomplete outbound is stopped/rejected. Firebase transitions are conditional
transactions, and the app tracks the exact request even for the same operator.
Terminal commands cannot be resurrected by the old APP flag.

Accepted returns have a separate 15-minute completion window. The app requires
fresh heartbeat, live Firebase, BLE/LiDAR health, and actual destination
readiness. Cached values expire after 90 seconds. Auth waiting is bounded, and
best-effort prewarming handles timeout without discarding an actual cancellation.
The exact deployed Firebase rules/auth and network behavior remain unverified.

## What verification establishes

See [Validation results](VALIDATION_RESULTS.md) for the final counts, target
sizes, Android results, and tool versions. The verification command executes:

- Existing Pi helper tests and new tests of the production runtime with external
  ROS/BLE/Firebase adapters replaced.
- Native C++ harnesses that include the actual firmware source, including a
  connected bridge/controller UART flow. These check obstacle recovery and BLE
  reconnect recovery, not merely matching constants or copied algorithms.
- Structural/audit checks, both ESP32 builds with installed board libraries,
  Android unit tests, debug/release builds, and lint.

Peripheral substitutes cannot reproduce electrical timing, motor physics,
wireless scheduling, echo waveforms, or deployed backend policy. Previous
reports are labeled historical; their conclusions do not replace this release's
evidence. The initial suspicion of missing STOP ACKs was not confirmed: the
complete source trace showed those ACKs already existed. Tests now exercise the
complete ACK flow to distinguish that from actual latch/queue problems.

## Remaining acceptance and deployment

No ESP32 was connected in the local serial inventory; only COM1 appeared.
Boards were not flashed, and the Pi/robot were not operated from this computer.
The following remain necessary in the deployed environment:

1. Configure/source ROS and the driver overlay; verify serial paths and Pi preflight.
2. Flash main ESP32, BLE ESP32, deploy the complete Pi directory, then install
   Android. The new return protocol requires the coordinated versions.
3. With wheels lifted, verify motor assignment, LiDAR orientation, echo readings,
   STOP/recovery, reconnect, nudges, every route segment, and correlated return.
4. Measure braking and steering with the intended payload and floor before
   supervised corridor operation. Verify a hardwired emergency stop.
5. Verify live Firebase authorization/auth, anonymous-auth configuration,
   readiness and command permissions. Previous Android audit notes about exposed
   historical credentials require account-side remediation if not already done.
6. Verify SIM/SMS and externally configured push delivery. This repository has
   an FCM receiver but no deployed cloud notification sender or RTDB rules.

The detailed tests, pass conditions, and stationary diagnostics are in
[Deployment and acceptance](DEPLOYMENT_AND_ACCEPTANCE.md). The repair is ready
for that procedure; physical behavior remains unknown until its results are
recorded. There is no software-only basis to claim flawless hallway operation.
