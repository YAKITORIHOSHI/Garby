# ESP32-S3 MPU steering implementation plan

> **For agentic workers:** Use superpowers:executing-plans to implement and verify these tasks in the current workspace. Preserve the user's existing changes.

**Goal:** Move the BLE bridge to the ESP32-S3 44-header-pin board and use the supplied MPU-6050 heading algorithm for straight-segment nudges.

**Architecture:** Sample gyro Z at 100 Hz using explicit I2C pins, the test sketch's register configuration, trapezoidal integration, and 2°/1° direction hysteresis. The controller announces straight-segment boundaries so the bridge resets the relative heading after intentional turns. MPU deviation chooses correction direction; fresh lidar clearance can veto it and the existing path STOP protocol retains priority.

**Tech stack:** Arduino ESP32-S3, Wire, NimBLE-Arduino, existing controller UART protocol and C++ host firmware tests.

**Spec:** User request and `Test/MPU_ESP32-S3-DEV.ino`. The reference sketch uses raw gyro data without bias calibration; retain that behavior and document drift and the lack of position sensing.

## Global constraints

- BLE bridge target: `esp32:esp32:esp32s3`; motor controller retains its existing ESP32 target.
- Approved wiring: MPU SDA GPIO8, SCL GPIO9; bridge UART RX GPIO18, TX GPIO17. Use GPIO labels, not header positions; leave GPIO19/20 for native USB.
- Preserve BLE name/UUIDs, path freshness, STOP latch, ACK watchdog and controller nudge limits.
- No lidar-only fallback when MPU heading is invalid; lidar remains responsible for obstacle/path clearance.
- Preserve the original reference sketch and all unrelated working-tree edits. Do not flash hardware.

## Review focus

- Positive gyro Z is a left deviation with +Z up, so forward correction must be right; test both signs and hysteresis.
- An intentional turn must not be corrected as drift; reset heading at each new straight segment and disable it at every segment exit.
- I2C faults, saturated readings and gaps over 100 ms invalidate heading and cancel an active correction; a new reference is required.
- A stop during a straight segment preserves its heading reference; a controller reboot invalidates it.
- Invalid or stale lidar/path data cannot authorize nudging, and side objects cannot redirect MPU correction into an unsafe side.

## Tasks

- [x] Add meaningful failing firmware checks for MPU ownership of steering and actual gyro sampling/direction/failure behavior.
- [x] Add a root-level `MpuHeading.h` reader adapted from the test; use it from the BLE loop and replace lidar centering with MPU-derived nudges gated by lidar clearance.
- [x] Add paired straight-segment UART notices to the controller, including early exits; test notices and stop/resume behavior through the actual bridge/controller source.
- [x] Update the bridge pin map, firmware build target and wiring/operation notes.
- [x] Run firmware behavior tests and repository host verification; compile both real targets. Results: 89 Pi tests, 108 firmware checks, both target builds passed. See `docs/s3-mpu-validation-2026-10-24.md` for evidence and hardware limits.
