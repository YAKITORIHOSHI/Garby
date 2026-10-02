# GARBY release notes — 2026-10-02

This is a coordinated reliability repair for the Pi, BLE ESP32, main ESP32,
and Android app. See `PROJECT_RUNDOWN.md` for findings and
`VALIDATION_RESULTS.md` for verified results.

## Motion and recovery

- Normal obstacles keep the controller in a monitoring wait, servicing UART,
  sonar, watchdogs, and status requests. Recovery resumes the original wheel
  targets after fresh LiDAR permission and, when locally blocked, three distinct
  valid clear echoes.
- Sonar detects a close raw echo immediately, preserves the obstacle latch
  through missing echoes, and uses an 18000 µs listening window. Periodic HALT
  diagnostics identify which clearance condition is missing.
- Both wheel-target command results are checked; one rejection stops both.
  Start acceptance no longer depends on an immediately updated `isRunning()`.
- STOP cannot be erased within the same UART batch. A GO after a stale gap
  restarts confirmation. Timing tolerance cannot authorize motion on old scans.
- Selected avoidance direction determines the nudge command; duration math
  cannot underflow. Confirmation/cooldown waits preserve the active tap.
  Requested taps now match the controller's 260 ms cap.

## BLE, Pi, and parsing

- BLE callbacks post bounded state/data. Loop context owns navigation reset,
  UART, notifications, and connection-parameter changes.
- Overflow, malformed framing, duplicate/old path sequences, and trailing
  delimiters assert STOP. Telemetry cannot inject newline-separated commands.
- One serialized Pi mailbox handles urgent and periodic paths. Older clear
  snapshots cannot replace newer STOP across sequence wrap.
- GATT writes use acknowledgement for path/control and whenever a steering or
  telemetry frame exceeds the negotiated no-response size.
- Old-client disconnect callbacks cannot tear down a replacement connection.
  Failed temporary connections are cleaned up. Scan subscription depth is one.
- Invalid configuration/scan metadata fails closed; an explicitly missing
  LiDAR port no longer falls back to the unrelated sensor device.

## App and return protocol

- Return uses `[RESET:<server-request-timestamp>]` with matching MCU
  ACK/DONE/FAILED. Duplicate IDs do not restart or brake an accepted return.
  Periodic queries of the same ID recover completion notifications lost during
  BLE reconnection.
- An interrupted outbound route cannot trigger a blind return. Route faults
  stay stationary and require local supervised recovery after repositioning.
- The Pi uses conditional Firebase transactions. The app tracks the exact
  request ID, including multiple requests from the same operator.
- Generic IDLE and an obsolete APP boolean cannot produce false completion.
  Accepted returns use a 15-minute completion window; new requests have a
  separate 120-second acceptance window.
- Return requires destination readiness, live Firebase, BLE/LiDAR health, and
  a fresh heartbeat. Cached app data expires after 90 seconds.
- Anonymous-auth waiting is bounded, and startup cancellation/timeout handling
  distinguishes best-effort prewarming from an actual cancelled operation.

## Deployment and compatibility

Update the main ESP32, BLE ESP32, complete Pi directory, then Android with wheels
lifted. Mixed old/new return implementations cannot complete the new request
workflow. UUIDs, UART pins/baud, calibrated route distances/signs, and motor
speed/trim settings are retained.

`Garby_MobileApp/final_w_serial.py` delegates to the authoritative Pi runtime.
`tools/verify_project.py` exports firmware and repeats the software checks.
`RasPi/check_runtime.py`, the launcher, and service/environment examples make
ROS overlay and serial prerequisites explicit.

Physical radio behavior, steering calibration, stopping distances, route
directions, sonar clearance, and deployed Firebase/SIM/FCM services remain
acceptance items. Compilation and host simulation do not establish those results.
