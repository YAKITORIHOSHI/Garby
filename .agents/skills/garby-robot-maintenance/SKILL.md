---
name: garby-robot-maintenance
description: Use when changing, validating or diagnosing GARBY's Raspberry Pi runtime, paired BLE and motor-executor firmware, sensing, operator controls, or SMS protocols, including nudging or steering, BLE disconnects, Pi throttling or undervoltage, or a robot that stopped and will not resume, even when no file is named. Excludes mobile-only work and unrelated plugin setup.
---

# GARBY robot maintenance

Preserve the deployed system's motion gates while making the requested change.
Read [project guidance](../../../AGENTS.md) and the affected sketch's guidance;
consult [architecture](../../../SYSTEM_ARCHITECTURE.md) for the relevant wire
contract and ownership. The user's session instructions determine authorized
scope; this skill does not add permission to operate hardware or send messages.

Trace the real path before editing: Pi producer, coalescing mailbox, BLE bridge,
UART parser, executor and acknowledgement. Keep fixes in active production
sources and coordinate both ends of a protocol change. A transport write or
queued request does not prove the executor accepted an action.

Prove a defect with a failing test on the unchanged code before fixing it. When
behaviour depends on a library, read its real source (Bleak in
`~/.local/share/garby/python-packages/`, NimBLE in `~/Arduino/libraries/`, the
driver in `~/ros2_ws/src/ydlidar_ros2_driver/`): a host stub that disagrees with
the library hides bugs, as the old notification stub did.

For field symptoms (BLE drops, throttling, absent nudging, a robot held in STOP),
read [field triage](references/field-triage.md) and the Pi log's `[BRIDGE DIAG ...]`
line before theorising.

For motion/operator work, check acquisition freshness and sequence gates, missing
data, disconnect/reconnect, duplicate IDs and delayed acknowledgements. Emergency
stop must dominate normal GO/start/return. Release/reboot and sonar mode changes
need their controller-confirmed preconditions; unknown/stale status cannot show
confirmed success. Keep obstacle sonar separate from bin-level sensing.

For Pi-to-controller path transport, test coalescing replacements as well as
first admission: blocked, malformed and stale PATH packets must retain STOP
priority, while a newer all-clear must not inherit a stale packet's priority.
Carry the LiDAR acquisition deadline through the Pi queue and revalidate before
the GATT write; clearly state that this is not end-to-end source-age validation
unless the controller protocol carries and checks acquisition age too. Preserve
the user's selected bypass scope: front/rear read-only bypasses only that group's
distance thresholds while retaining readings and Start behavior; confirmed
SONAR_OFF bypasses the local sonar gate and parks the servo at DEFAULT_VIEW.
Do not add a global motion interlock merely because a bypass is enabled. Missing
or stale path evidence, side clearance, emergency stop and MPU/nearest steering
clearance remain authoritative. Describe current 2D LiDAR as generic obstacle
sensing, never human classification, unless a separately validated people-sensing
system exists.

For heading/centering changes, keep wall estimates separate from nearest-object
clearance. Exercise independent acquisition A, repeated/backward scans, wall gaps,
door leaves, single walls and hidden thin objects through the real consumer. The
bridge streams one continuous heading (M lines) and forwards wall fits with their
age (W lines); the executor fuses them (`HeadingControl.h`). Permission bits only
veto steering toward a side, never add a turn the other way; missing walls remove
centring, not MPU heading hold; malformed values and missing CF/CL/CR withdraw all
permission. One bad MPU read is skipped, and the 100 ms gap budget stays the hard
limit. `[MCU ALIVE]` is a recovery ping that changes nothing; `[MCU READY]` means a
controller restart; neither re-zeroes the bridge heading. No motion without heading
feedback. Use `tools/firmware_tests/test_navigation.cpp` (kinematic hallway) for
tuning, and do not infer physical settling, slip or straightness from it. Check the
production Pi obstacle envelope (sectors plus chassis zones) when choosing corridor
test geometry; direct bridge fixtures can be unreachable behind PATH STOP.

For health/SMS work, measure internet independently of cloud authentication.
Preserve unknown states, fault dwell, recovery evidence and low-priority health
transport. Read the current constants and APIs rather than copying old timing
values from a report. The modem's single worker owns AT transactions; safety
callbacks enqueue only. Model a notice's whole transition, including an in-flight
send and replacement/cancellation, not only its queue slot. Indeterminate
post-body outcomes and exhausted retry budgets must survive repeated health
polls and unknown gaps. Final OK and +CMGS mean network submission only.

Use deterministic tests of the real consumer and boundary, not duplicate
algorithms. Firmware task creation is stubbed in host tests, so modem changes
need `serviceSmsQueueOnce` with scripted UART responses. Include malformed,
duplicate/stale and burst cases that prove health/SMS cannot refresh motion
permission. Test critical queue admission as well as dispatch priority.

Run the applicable project checks in `AGENTS.md`, then real ESP32 target builds
for affected sketches. A host-only include can conceal missing public header
declarations across actual compilation units. Resolve target identity from
current/saved state as the sketch guidance requires; old ports and Bluetooth
availability are not evidence that a USB upload target is connected.

For USB-versus-standalone startup faults, trace both supply and ground from the
actual source to the board, including intermediate boards/converters. Record
reported ratings separately from measured rails. Verify physical board identity,
EN/boot conditions and the installed image's USB options before attributing a
fault to Serial or assigning an RGB GPIO; RGB pins vary between board revisions.
Preserve brownout protection. An existing ELF can establish its own Serial/CDC
configuration, but it does not prove that image is flashed on the board.

To run the real bridge on the bench, isolate it first, because synthetic scans read
as a clear path and could release a real robot, and this tree holds a real Firebase
key and operator latch. Stop it with the PID from `$!`, not `pgrep -f`, which can
match your own shell:

```bash
ROS_DOMAIN_ID=77 GARBY_BLE_SERVICE_UUID=11111111-2222-3333-4444-555555555555 \
GARBY_BLE_DEVICE_NAME=GarbyNoMatchBench GARBY_FIREBASE_CREDENTIALS=/nonexistent \
GARBY_SENSOR_SERIAL_PORT=/nonexistent GARBY_LIDAR_PORT=/nonexistent \
GARBY_OPERATOR_STOP_FILE=/tmp/bench-operator-stop GARBY_RUNTIME_LOCK_FILE=/tmp/bench.lock \
bash RasPi/run_bridge.sh --headless
```

Keep physical actions outside software validation unless already authorized.
Provide reviewable source changes and evidence before any final deployment
approval is needed. Update operating descriptions and dated validation notes
with tested limits; do not claim live SMS delivery, physical braking or an
uploaded firmware revision from mocked tests or compiled binaries.
