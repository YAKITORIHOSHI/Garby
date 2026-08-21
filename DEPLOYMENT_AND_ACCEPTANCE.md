# GARBY Deployment, Calibration, and Acceptance

**Updated:** 2026-08-21  
**Applies to:** the coordinated release described in `SYSTEM_ARCHITECTURE.md`

Do not perform floor testing until wheels-lifted communication and STOP tests pass. Use a reachable hardwired latching emergency stop and a human spotter for first motion tests.

## 1. Build and deployment order

1. **Main ESP32:** compile the sketch directory `NAPHTALI_CODE_V2/` so Arduino includes `NAPHTALI_CODE_V2.ino`, `pointsRun.ino`, `NAPHTALI_CODE_V2.cpp`, and `NAPHTALI_CODE_V2.h` in one build. Flash it first.
2. **BLE bridge ESP32:** compile and flash `BLE_Receiver-Final/BLE_Receiver-Final.ino` with the installed NimBLE-Arduino/ESP32 core used on the target board.
3. **Raspberry Pi:** deploy the entire `RasPi/` folder. Do not copy only `final_w_serial.py`; `bridge_core.py` is required source.
4. Install Pi Python packages in `RasPi/requirements.txt`. ROS 2/rclpy, `sensor_msgs`, and the YDLidar ROS 2 driver must be installed through the Pi/ROS environment, not from that requirements file.
5. Start from `RasPi/` with `python3 final_w_serial.py --headless`.
6. Firebase is optional for motion safety. Set `GARBY_FIREBASE_CREDENTIALS` if the credential JSON is not at the legacy default location.
7. Set `GARBY_LIDAR_YAW_OFFSET_DEG` only after physically checking LiDAR orientation. Do not guess.

A systemd example is supplied as `RasPi/garby-bridge.service.example`; update its working path before enabling it.

## 2. Expected healthy startup trace

With the wheels lifted, the expected control sequence is:

```text
Main MCU -> bridge: [MCU READY]
Bridge -> main: [BLE CONNECTION ESTABLISHED]
Bridge -> main: STOP:WAITING_DATA
Main -> bridge/Pi: [REQUEST-STATUS]
Pi -> bridge: P:<seq>|F=...|B=...
Pi -> bridge: S:<same-seq>|...
Bridge -> main: STOP:CLEARING       (first clear while latched)
Bridge -> main: GO                  (second clear path)
Bridge -> main: GO                  (next fresh clear path)
Main: Fresh path confirmed -- motion enabled
```

Until a valid first LiDAR scan exists, the Pi must send `F=S|B=S`; seeing CLEAR before the first scan is a release failure.

## 3. Communication checks — wheels lifted

Record PASS/FAIL for every item.

| Test | Required result |
| --- | --- |
| Pi program starts | No `bridge_core` import error; process remains running |
| Firebase SDK/credential/network unavailable | BLE/LiDAR safety process remains running; Firebase is disabled/retrying only |
| Main boots while modem/network is absent | `[MCU READY]` is sent without waiting for cellular registration |
| Bridge boots before main | Bridge waits fail-closed for `[MCU READY]` |
| Main reboots while BLE stays connected | New `[MCU READY]` creates a fresh boot epoch; previous path permission is discarded |
| BLE disconnect | `STOP:LINK`; no automatic `[RESET]` or return |
| LiDAR has not produced first scan | `P:<seq>|F=S|B=S` |
| LiDAR stream stops | Pi stale at 0.8 s; bridge path watchdog at 850 ms; controller independent freshness watchdog at 800 ms |
| Fresh scan missing required front/back sectors | affected path side becomes `S`, never `C` |
| Old/out-of-order `P:` | rejected; cannot clear a newer STOP |
| `S:` sequence differs from newest `P:` | ignored/stable; no steering action |
| malformed `P:` (duplicate field/unknown field/overflow sequence) | `STOP:PROTOCOL` |
| `SENSOR:` during STOP | telemetry is accepted/forwarded but STOP remains latched |
| malformed `N:` numeric field | rejected by controller |
| MCU UART ACKs stop | bridge latches `STOP:MCU_LINK` |
| communication recovers | STOP remains latched until fresh repeated path/GO confirmations complete |

## 4. Local sonar tests — wheels lifted

1. Verify servo center is physically forward at 80° and endpoints 40°/120° do not bind.
2. Check known target distances with a ruler/tape.
3. At ≤25 cm, one valid center reading must latch the emergency obstacle state.
4. At ≤60 cm, two valid center samples must latch STOP.
5. Remove the obstacle: three **valid** samples ≥72 cm are required before the local latch clears.
6. While latched, disconnect the echo lead or force sonar timeout. The latch **must remain set**; timeout/no echo cannot count as clear.
7. Verify no side-sonar steering occurs (`ENABLE_ULTRASONIC_SIDE_NUDGE=0`).

Electrical requirements: regulated servo 5 V supply sized for stall current, common ground, local bulk capacitance, and a 3.3 V-safe ultrasonic echo input.

## 5. LiDAR orientation and sector test

The software default is 0° FRONT, 90° LEFT, 180° BACK, 270° RIGHT. Physical orientation has not been proven from source.

1. Keep wheels lifted and place one object about 0.5–1.0 m directly in front.
2. Confirm the GUI/log reports FRONT, not BACK/LEFT/RIGHT.
3. Repeat at LEFT, BACK, RIGHT.
4. If the entire mapping is rotated, set `GARBY_LIDAR_YAW_OFFSET_DEG` (for example 180 for a reversed sensor) and restart the Pi.
5. Record the accepted offset in the test log and deployed environment.

Do not compensate a mount-rotation error by swapping motor commands or rewriting nudge directions.

## 6. Route direction test — wheels lifted

`pointsRun.ino` cannot prove real-world route direction from sign alone.

1. Mark both wheels and identify chassis FRONT.
2. Trigger a supervised outbound sequence at reduced speed or step counts.
3. Record every straight and turn direction in order.
4. Verify the return sequence physically retraces/implements the intended route.
5. Only after observation may route signs/turn steps be changed; if changed, update `SYSTEM_ARCHITECTURE.md` and retest all watchdogs.

## 7. Straight-line calibration

The wheel trim defaults are `MOTOR1_TRIM_PCT=0.0` and `MOTOR2_TRIM_PCT=0.0`.

- Use a flat measured lane.
- Disable/avoid active nudge influence during mechanical calibration.
- Repeat at least three runs with the same payload.
- Adjust trim in small increments only after identifying repeatable mechanical drift.
- Do not hide tire, alignment, bearing, motor-current, or power faults with large software trim.

## 8. Hallway-centering acceptance

After wheel trim and LiDAR orientation are correct:

- verify 15 cm dead zone prevents constant hunting;
- verify direction needs five fresh packets;
- verify 35–75 ms taps and 8–22% bridge-requested cut remain gentle;
- verify 1100 ms cooldown and 220 ms controller settle prevent rapid reversals;
- verify steering is fully suppressed at ≤60 cm front distance and attenuated through 60–120 cm;
- test open doors, pillars, protrusions, narrow/wide corridor sections, and a person walking near a side wall;
- record sequence, left/right distances, combined error, direction, duration, and intensity when tuning.

Change one tuning group at a time.

## 9. Braking acceptance

Software constants are not a stopping-distance certificate.

Measure worst-case total stopping distance using:

- empty and maximum expected payload;
- highest allowed speed and caution speed;
- full and low expected battery;
- normal and lowest-traction floor;
- repeated front and diagonal obstacle entries.

The 95 cm front LiDAR threshold is acceptable only if the measured worst-case stop plus safety margin remains inside it. Otherwise reduce speed and/or increase stop distance before operation around people.

## 10. Power and thermal endurance

Run at least 30–60 minutes in the final enclosure while monitoring:

- Pi undervoltage/throttling/restarts;
- ESP32 brownouts/resets;
- motor-driver and motor temperature;
- servo supply dip/noise;
- BLE reconnect count;
- LiDAR health/stale events.

A software reconnect loop cannot fix inadequate power distribution.

## 11. Final release criteria

Do not approve unattended corridor use until all are true:

- hardwired emergency stop verified;
- both ESP32 sketches compile in the actual target toolchain with the installed library versions;
- Pi host tests and static audit pass;
- complete wheels-lifted startup/disconnect/stale/malformed tests pass repeatedly;
- LiDAR physical orientation is recorded;
- route direction is physically verified;
- sonar timeout cannot release a STOP;
- maximum-payload braking distance is measured and safe;
- corridor/open-door/person-side tests pass;
- final constants, library/core versions, battery, payload, floor, and test results are recorded.
