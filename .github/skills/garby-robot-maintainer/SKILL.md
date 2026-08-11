---
name: garby-robot-maintainer
description: Safety-first maintenance, debugging, optimization, and documentation workflow for the GARBY autonomous waste-collection robot. Use when ChatGPT receives GARBY files such as SYSTEM_ARCHITECTURE.md, final_w_serial.py, BLE_Receiver-Final.ino, NAPHTALI_CODE_V2.ino/.cpp/.h, pointsRun.ino, logs, or test results and must analyze or modify LiDAR hallway centering, anti-zigzag nudging, STOP/GO behavior, BLE/UART parsing, watchdogs, servo-ultrasonic sweeping, motor braking, state transitions, performance, deployment, or architecture documentation. Preserve three-node control ownership, prioritize fail-closed human safety, coordinate protocol changes across every affected node, validate the result, and update SYSTEM_ARCHITECTURE.md whenever runtime behavior changes.
---

# GARBY Robot Maintainer

## Purpose

Maintain GARBY as one coordinated safety-critical system rather than as independent sketches. Treat the Raspberry Pi, BLE bridge ESP32, and main-controller ESP32 as one distributed control loop whose interfaces, timing, and failure behavior must remain consistent.

Read `references/SYSTEM_ARCHITECTURE.md` before changing code. Use the supplied project files as the source of truth. When code and documentation disagree, identify the mismatch, determine the deployed behavior from code, and update both the implementation and architecture instead of silently choosing one.

## Required inputs

Prefer the complete project set:

- `SYSTEM_ARCHITECTURE.md`
- `final_w_serial.py`
- `BLE_Receiver-Final.ino`
- `NAPHTALI_CODE_V2.ino`
- `NAPHTALI_CODE_V2.cpp`
- `NAPHTALI_CODE_V2.h`
- `pointsRun.ino`
- Relevant serial logs, ROS 2 logs, protocol captures, and hardware-test notes

Proceed with a partial set only when the requested task can be bounded safely. State which conclusions remain unverified because a component, log, hardware detail, or toolchain is missing.

## Non-negotiable safety invariants

Preserve these rules in every review and patch:

1. Treat unknown, malformed, missing, stale, disconnected, or out-of-order path data as STOP.
2. Never allow `SENSOR:` telemetry, GUI state, logging, Firebase traffic, or an unrelated control message to produce GO or clear STOP.
3. Latch STOP. Require repeated fresh clearance evidence before movement resumes.
4. Reject old path packets and bind steering data to the currently accepted path sequence.
5. Disable nudging whenever STOP is latched, path data is stale, or motion permission is not confirmed.
6. Keep LiDAR as the only hallway-centering authority by default. Use the local servo ultrasonic sensor for near-field stopping unless a separately tested mode explicitly enables side steering.
7. On BLE or LiDAR loss, remain stationary. Do not begin an automatic blind return.
8. Separate normal endpoint deceleration, deliberate gentle stopping, safety stopping, and last-resort hard stopping.
9. Do not add a reverse torque pulse to safety braking without measured evidence that it improves stability for the real payload.
10. Keep safety polling responsive. Do not place long modem waits, multi-second delays, blocking rescans, unbounded queues, or verbose high-rate logging in the motion-safety path.
11. Require a hardwired emergency-stop and measured stopping-distance validation before claiming the robot is safe around people.
12. Never infer the physical return direction from function names alone. Verify `pointsRun.ino` behavior with the wheels lifted and document the result.

## Workflow

### 1. Inventory and establish the baseline

1. List all provided files and identify duplicate or version-suffixed copies.
2. Identify the intended deployment set before editing.
3. Read the architecture, packet definitions, timing constants, pin map, state machine, and route functions.
4. Run `scripts/audit_project.py <project-root>` when a local project tree is available.
5. Record mismatches between the architecture and source before proposing changes.

Do not mix code from different revisions unless the protocol and shared constants are reconciled explicitly.

### 2. Trace the complete behavior

For each reported symptom, trace the full path from sensing to actuation:

```text
LiDAR or local sensor
  -> Raspberry Pi processing
  -> BLE queue and GATT write
  -> BLE bridge dispatch and safety latch
  -> UART command
  -> main-controller parser and watchdog
  -> motor, servo, buzzer, state machine, or modem action
```

Identify every producer and consumer of the affected state. Check both the normal path and failure paths such as stale data, disconnect, malformed input, queue backlog, reset, boot, and resume.

### 3. Triage by risk

Use this priority order:

1. Human contact or failure to stop
2. Motion on stale/disconnected data
3. Unintended restart after STOP
4. Braking instability or payload spill
5. Conflicting steering authorities or zigzag
6. Blocking servo, ultrasonic, modem, or parsing work
7. Throughput, CPU, logging, GUI, and maintainability issues

Patch the safety path first even when the user primarily reports performance or steering symptoms.

### 4. Analyze the requested subsystem

#### STOP, GO, and communication

- Verify strict packet-type dispatch on the bridge.
- Verify only a valid path packet can affect STOP/GO.
- Verify sequence freshness and wrap-safe comparison.
- Verify path and link watchdogs are motion-scale, not tens of seconds.
- Verify the controller starts fail-closed and independently checks path freshness.
- Verify one clear packet cannot release a previous STOP.
- Verify BLE loss emits a STOP reason and does not queue automatic return.
- Verify safety messages have priority over steering and telemetry.
- Verify BLE writes are serialized; use acknowledged writes for safety/control packets.
- Verify UART parsing is bounded, newline-delimited, and non-blocking.

#### LiDAR centering and anti-zigzag

- Preserve one steering authority.
- Use robust side-wall estimates rather than a single closest side point.
- Apply smoothing, a centered dead zone, direction-reversal hysteresis, multi-packet confirmation, cooldown, proportional short taps, and a post-tap settle interval.
- Suppress steering near a front obstacle and near the target side wall.
- Reject doorway, wall-protrusion, passing-leg, and partial-cone artifacts.
- Keep left and right execution symmetric unless measured calibration supports a documented bias.
- Change one tuning group at a time and retain logs that show error, direction, duration, intensity, and path sequence.

#### Servo and ultrasonic

- Use a command -> settle -> ping state machine.
- Limit servo angles away from mechanical end stops.
- Use a bounded echo timeout shorter than the scheduling interval.
- Prioritize the center path and require sample confirmation, with an immediate very-close threshold.
- Require repeated local-clear samples plus fresh LiDAR clearance before resuming after a local obstacle.
- Avoid long nested left/front/right scan loops.
- Check servo power separately from software: regulated 5 V supply, common ground, stall-current capacity, and local bulk capacitance.

#### Braking and movement

- Use controlled deceleration for a safety stop.
- Reserve force stop for a failed or timed-out deceleration ramp.
- Keep nudges cleared during every stop.
- Calculate theoretical speed and braking distance, but label them as estimates.
- Base acceptance on measured worst-case stopping distance with the heaviest payload, lowest expected battery, and lowest-traction floor.

#### Performance and real-time behavior

- Bound or coalesce queues that carry latest-value state.
- Ensure a newer STOP can replace an older queued clear packet.
- Keep telemetry slower than the path-safety stream.
- Keep Firebase, GUI refresh, garbage collection, SMS, and console logging outside the LiDAR or motor safety hot path.
- Use locks or message passing for shared Pi state.
- Avoid dynamic work in high-frequency loops when a fixed buffer or cached value is sufficient.
- Treat repeated `String` growth on ESP32 as a review concern; bound buffers and reset on overflow.

### 5. Patch the smallest coordinated set

Patch all components affected by an interface or timing change. A protocol change normally requires coordinated edits to:

- Raspberry Pi producer
- BLE bridge parser and safety latch
- Main-controller parser or execution guard
- Tests or audit expectations
- `SYSTEM_ARCHITECTURE.md`

Do not leave mixed old/new packet formats without an explicit compatibility boundary. Keep legacy parsing isolated so unrelated packets cannot fall through to movement logic.

### 6. Update the architecture

Update `SYSTEM_ARCHITECTURE.md` in the same task whenever any of the following changes:

- Node responsibility or control ownership
- Packet format, baud rate, UUID, sequence handling, or acknowledgment behavior
- STOP/GO latch, clearance, watchdog, disconnect, reset, or resume behavior
- LiDAR cones, thresholds, filters, centering, or nudge tuning
- Servo scan pattern, timing, ultrasonic thresholds, or steering authority
- Motor speed, acceleration, deceleration, braking, or route behavior
- State-machine transitions, deployment order, or validation requirements

Use `references/SYSTEM_ARCHITECTURE.md` as the baseline structure. Keep implementation constants and documentation synchronized.

### 7. Validate before delivery

Perform every available check and report unavailable checks honestly.

Minimum software checks:

1. Run the project audit script.
2. Run `python3 -m py_compile` on the Raspberry Pi program.
3. Compile both ESP32 projects with the actual board core and installed libraries when available.
4. Test packet parsing and dispatch, including telemetry during STOP.
5. Test stale and out-of-order packet rejection.
6. Test bridge and controller clearance-confirmation counts.
7. Test that nudges are ignored while stopped or stale.
8. Test disconnect and LiDAR-stale behavior.
9. Test queue coalescing so a newer STOP replaces an older clear state.
10. Search for blocking delays and long `pulseIn()` timeouts in safety loops.

Required physical checks before hallway use:

- Wheels-lifted boot, disconnect, stale-LiDAR, STOP-latch, and route-direction tests
- Servo endpoint, power, sweep, and sonar-distance tests
- Empty and maximum-payload braking tests at multiple speeds
- Corridor tests for open doors, protrusions, people at the side, narrow/wide sections, and low battery
- Hardwired emergency-stop verification

Never describe host-side syntax tests as hardware validation.

## Output contract

For a review, provide:

1. System understanding and affected data flow
2. Findings ordered by safety severity
3. Root cause with file/function evidence
4. Coordinated recommended changes
5. Risks, assumptions, and missing evidence
6. Validation and hardware-test plan

For an implementation, deliver:

1. Complete modified files, not isolated snippets when full files are practical
2. Updated `SYSTEM_ARCHITECTURE.md`
3. Patch notes that explain behavior changes and compatibility impact
4. Validation results with commands and outcomes
5. Hardware test checklist for any motion-related change
6. A deployment order when more than one node changed

Use explicit labels such as `verified in source`, `host-tested`, `requires hardware test`, and `physical behavior unknown`.

## Review checklist

Read `references/SAFETY_REVIEW_CHECKLIST.md` for the detailed subsystem checklist. Use it when conducting a full audit, preparing a release, or reviewing a safety-related patch.

## Companion agent

Use `agents/garby-thesis-assistant.agent.md` when the task needs a user-invocable, thesis-focused agent that explains source evidence, traces behavior end-to-end, edits architecture, and preserves this skill's safety invariants. The agent is intentionally read/search/edit focused and must not claim compilation or hardware testing unless another tool reports it.

## Bundled resources

- `agents/garby-thesis-assistant.agent.md`: Companion GARBY safety and thesis agent.
- `references/SYSTEM_ARCHITECTURE.md`: Current safety-first GARBY architecture and protocol baseline.
- `references/SAFETY_REVIEW_CHECKLIST.md`: Detailed audit and release checklist.
- `scripts/audit_project.py`: Static project audit for expected files, fail-closed constants, protocol separation, watchdogs, braking, and servo safeguards.
