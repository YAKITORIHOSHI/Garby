# Garby investigation and repairs — 2026-10-29

This review repaired reproducible software defects in motion, MPU feedback,
LiDAR clearance, and logging. It does **not** establish that the reported loud
buzzing with no rotation is physically repaired. No board was flashed and no
robot, motor driver, LiDAR or MPU was physically exercised.

## How this project runs

The Pi processes ROS LiDAR scans and sends sequenced clearance/geometry over BLE.
The ESP32-S3 bridge combines that geometry with its MPU6050 and sends GO, STOP
and bounded steering commands over UART. The main ESP32 executes the fixed route
in `MAIN_EXECUTOR_CODE/pointsRun.ino`, with local ultrasonic stopping, load/gas
triggers and optional modem messages. The Android/Firebase layer monitors and
requests return; it does not grant collision clearance. This is an open-loop
wheel system: emitted step counts are not measured wheel movement.

## Confirmed defects and repairs

| Defect | Resulting behavior after repair |
| --- | --- |
| MPU integrated raw Z rate without subtracting stationary bias | Boot measures 200 fresh samples while level/still, subtracts bias, rejects unstable/invalid calibration |
| Repeated readable gyro registers counted as fresh | DATA_RDY and sensor bytes are read together; missing data expires heading |
| LiDAR could veto yaw but could not recenter an off-center, parallel robot | Consistent opposing walls supply a bounded target heading offset; MPU controls yaw toward it |
| Invalid/missing MPU only disabled nudging while a route could continue | Failed calibration holds STOP; a lost straight-run reference cannot be cleared by fresh LiDAR alone |
| Wheel catch-up to unchanged endpoints undid a speed steering tap | Correction-induced wheel differential adjusts the endpoints while their sum remains fixed |
| Steering could arrive too near endpoint braking | New taps require an acceleration/queue-derived remaining-distance margin |
| A rejected continuous command could leave the other wheel running | Paired command acceptance stops both wheels on rejection |
| Urgent LiDAR updates could mark a blocked or unavailable opposite direction clear | Urgent and periodic messages use one complete, locked clearance snapshot |
| A queued old scan became fresh when received | Acquisition age consumes the same 0.8-second freshness budget; missing/invalid/stale timestamps stop |
| Invalid current side data reused historical wall geometry | Current invalid fields are omitted immediately; real outages clear history |
| Infinite returns or sparse broad fallback could look like walls | L/R require at least three finite narrow-cone returns |
| Zero-range data could break scan processing | Invalid zero returns are handled without granting clearance |
| Routine diagnostics/status polls consumed UART, GUI memory and formatting work | Debug dumps default off; unchanged STABLE commands are suppressed; status polls do not fill GUI logs; scrollback is capped at 200 lines |

## Hallway tracking behavior and limits

Forward nudge-enabled segments acquire relative FRONT through the existing
START/END UART notices. Turns and reverse travel do not use forward steering.
MPU yaw has 2-degree entry / 1-degree release hysteresis. At boot, keep the
chassis stationary and level for about 2?3 seconds; steady rotation can resemble
gyro bias and cannot be ruled out by software alone.

When L/R and all four diagonals agree with opposing walls, the bridge requests
a small heading toward the corridor center. The target is limited to ±4 degrees,
with an 8 cm lateral deadband, an 80–400 cm combined width and at most 250 cm to
either wall. Inconsistent doorways/open space do not create a wall target.
Front/side clearance can always veto the correction. These conservative tuning
values need validation on the real chassis; they are not a claim of measured
tracking performance. Existing confirmation, cooldown and bounded tap limits
remain. Endpoint compensation preserves mean commanded route travel, not exact
physical distance or pose when wheels slip or miss steps.

**Narrow-hallway constraint:** the existing 95 cm radial front/diagonal obstacle
envelope may stop a centered robot in a hallway roughly narrower than 176 cm.
That threshold was preserved because chassis clearance and braking measurements
are unavailable. Sidewall centering does not override a collision STOP. Verify
the actual hallway width and scan orientation before reducing any margins.

Pi `/scan` now requires a valid acquisition timestamp in the same ROS clock
domain as the node. Configure the actual LiDAR yaw offset and verify each sector
physically. A sensor outage requires new evidence; buffered old scans cannot
restore motion. A lost MPU reference during a straight run requires supervised
alignment and an explicit new reference; see [bridge operation](BLE_Receiver-Final/NOTE.txt).

## No-motion / intense-sound diagnosis

The source does not identify the installed motor/driver models, enable wiring,
current limit, supply behavior or microstep switches. Healthy UART and software
step counts cannot distinguish rotation from a mechanical stall. The motor pin
map, calibrated route step counts and existing normal speed/acceleration profiles
were preserved rather than replacing electrical settings with guesses.

Use the separate FastAccel project's `MotorsConfig` one-wheel, finite 400 Hz test
with wheels lifted. If STEP pulses exist but the motor only buzzes, verify coil
pairs, driver enable, current limit, supply and input voltage/timing against that
driver's manual. If no pulses exist, trace initialization, calibration, two-way
UART, fault state and STEP wiring. Do not connect or disconnect motor windings
while the driver is powered. Then verify motor assignment and yaw correction
sign, followed by short supervised loaded runs, measured braking, obstacle
recovery and the longest actual hallway segment.

## Verification and deployment

Representative target builds passed using Arduino-ESP32 3.3.10,
FastAccelStepper 1.2.7 and NimBLE-Arduino 2.5.0:

| Sketch | Target | Program size | Static RAM |
| --- | --- | ---: | ---: |
| Main motor controller | esp32:esp32:esp32 | 396,210 B | 26,208 B |
| BLE/MPU bridge | esp32:esp32:esp32s3 | 596,056 B | 36,844 B |

The source targets are compile checks, not identification of a connected board.
No live board selection was available. The bridge's saved GUI state reports
no selected board or connected MCU; it is historical, not a live connection check.
Dated images, source/image SHA-256 hashes and logs are in
`artifacts/verification-2026-10-29/`. The main compile's successful exit/size was
captured in the tool result; its stdout was not separately saved as a logfile.
Full Android Gradle builds, live Pi/ROS/BLE operation and Firebase deployment
were not validated in this investigation.

Final host validation passed **106 Python tests and 131 firmware behavioral checks**,
including the complete outbound/return cycle. The separate FastAccel project's
**52 host scenarios** also passed independently. C++/JSON structural checks and
Android static quality checks passed. `verify_project.py --host-only` now supports
portable native compilers through `CXX` and explicitly reports that the old
`.github/skills/garby-robot-maintainer/scripts/audit_project.py` is absent from
this supplied workspace; that legacy audit is unavailable, not passed.

Executable regressions include the production sketch sources with host peripheral
substitutes, not copies of the control algorithms. These establish state/data
handling and command behavior, not motor torque, pulse waveforms, wireless timing,
physical heading accuracy or stopping distance. Current verification results are
recorded with logs under `artifacts/verification-2026-10-29/`.

Deploy the updated Pi directory, motor sketch and ESP32-S3 bridge together.
Existing firmware binaries/build reports elsewhere in the tree predate these
changes and must not be mistaken for newly compiled release images. Android
source and Firebase deployment were outside the repaired control path and were
not changed. No existing internal flasher caches were edited.

Primary references: [MPU6050 specification](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Datasheet.pdf),
[register map](https://cdn.sparkfun.com/datasheets/Sensors/Accelerometers/RM-MPU-6000A.pdf),
[FastAccelStepper](https://github.com/gin66/FastAccelStepper).
