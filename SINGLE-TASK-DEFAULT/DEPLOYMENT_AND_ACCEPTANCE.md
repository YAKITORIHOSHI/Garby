# GARBY Deployment, Calibration, and Acceptance

Last updated: 2026-08-11

This is the hardware handoff for the optimized GARBY stack. Complete the stages
in order. Keep the drive wheels lifted until the reset, stop, and link-loss tests
pass. A reachable physical emergency stop and a spotter are required for every
first floor run.

## 1. Deployment Order

1. Flash the main motor ESP32 from the root files in `NAPHTALI_CODE_V2/`.
2. Flash the BLE receiver ESP32 from
   `BLE_Receiver-Final/BLE_Receiver-Final.ino`.
3. Copy `RasPi/final_w_serial.py` and `RasPi/bridge_core.py` to the Pi, install
   the existing ROS 2, YDLidar, Firebase, BLE, and serial dependencies, and set
   the environment values documented in `RasPi/README.md`.
4. Start the bridge with `python3 final_w_serial.py --headless`.
   For normal deployment, adapt and enable `RasPi/garby-bridge.service.example`
   so the headless bridge starts after boot and recovers from a process exit.
5. Confirm Firebase telemetry and health before installing the Android build.
6. Install the debug APK for supervised tests. Sign the release APK before any
   normal deployment.

Do not flash files from `NAPHTALI_CODE_V2/src/`; the active executor sketch is
the set of root `.ino`, `.h`, and `.cpp` files. Do not deploy anything from the
experimental `GARBY-NOTIF-TRIG-HOST` project.

## 2. Power-Off Inspection

- Confirm a shared ground between both ESP32s, motor drivers, sensor board, and
  Pi where the design requires it.
- Confirm the ultrasonic echo presented to the ESP32 is no higher than 3.3 V.
- Confirm the servo center really points down the chassis centerline at 80
  degrees; verify its stopped/manual positions at 40, 80, and 120 degrees.
- Confirm LiDAR `FRONT` and `BACK` match the installed sensor orientation.
- Check wheel diameter, wheelbase, steps per revolution, motor directions, and
  left/right connector assignment against `NAPHTALI_CODE_V2.h`.
- Use a stable supply with motor-current margin. Pi undervoltage and motor
  transients cannot be fixed in software.
- Fit and test the Pi heatsink/fan in the closed, final enclosure.

## 3. Wheels-Lifted Safety Tests

Run each item from a fresh boot and record the result.

| Test | Required result |
| --- | --- |
| Pi not started | Wheels remain stopped |
| BLE disconnected during motion request | Receiver sends `STOP:LINK`; motion stops |
| LiDAR scan stopped | Stale path fails closed; motion stops |
| Path packets stop | Receiver stops by 650 ms; executor has an 800 ms final watchdog |
| Malformed or old sequence packet | Packet is rejected; it cannot authorize movement |
| Malformed/non-finite side distance | Receiver emits stable, resets direction confirmation, and never nudges |
| Front sonar object | Slowdown begins by 90 cm; two samples at or below 60 cm stop; one at or below 25 cm emergency-stops |
| Object removed | Three samples at or above 72 cm plus fresh LiDAR `GO` are needed to resume |
| App reset | `pending -> ack -> returning -> [IDLE] -> done` |
| Repeated reset while returning | Return continues; no early `[IDLE]` or false `done` |
| Old Firebase reset | Request is rejected and cannot move the robot |

Do not continue if any safety test fails intermittently.

## 4. Straight-Line Wheel Calibration

Mechanical wheel mismatch should be corrected before tuning LiDAR nudges.

1. Use a flat 3-5 m lane, mark the centerline, and begin at the current slower
   `MAX_SPEED = 5600` configuration.
2. Temporarily prevent remote nudge commands during the measurement run, or use
   a test in which both walls are outside the nudge decision range.
3. Run straight three times and measure lateral drift at the same distance.
4. Adjust only `MOTOR1_TRIM_PCT` or `MOTOR2_TRIM_PCT` in
   `NAPHTALI_CODE_V2.h`. Positive speeds that motor up; negative slows it down.
5. Change in small 0.25 percentage-point steps, reflash, and repeat three runs.
6. Stop when the repeatable mechanical drift is small. Do not hide a large
   mounting, tire, bearing, or current-limit fault with software trim.

Both trims intentionally default to `0.0f`; a guessed bias would make a
different floor or payload worse.

## 5. LiDAR and Proportional Nudge Calibration

The receiver now calculates both nudge duration and intensity from the measured
lane error. It confirms direction for five packets, uses a 15 cm dead zone and
10 cm reversal hysteresis, fires at most once per 1100 ms, and emits only 35-75
ms taps at an 8-22 percent speed cut. The executor applies tighter final caps
and rejects commands older than 250 ms.

Use this order after wheel trim:

1. In a straight corridor, log `LEFT`, `RIGHT`, diagonals, and transmitted tilt
   while the stationary chassis is physically centered.
2. Correct LiDAR mounting orientation or offsets before changing thresholds.
   The production mapping is 0 degrees back, 90 right, 180 front, and 270 left.
   Safety sectors cover the complete circle; the narrower centered slices are
   used only for steering geometry.
3. Walk the robot slowly for 3-5 m. Confirm a small error produces a small tap
   and a large error produces a stronger, still-bounded tap.
4. Confirm the first eight accepted side samples cause no nudge and the next
   eight samples ramp smoothly to full authority.
5. Test doorways and missing-wall openings. Heading tilt is clamped to plus or
   minus 20 cm and steering uses median wall geometry, so far returns must not
   cause a hard correction.
6. If it oscillates, first increase `NUDGE_DEAD_ZONE_CM` slightly or increase
   `NUDGE_COOLDOWN_MS`. If it corrects too slowly after wheel calibration,
   reduce the dead zone slightly before increasing maximum intensity.
7. Change one constant at a time and keep a before/after corridor trace.

Do not increase stop distances and nudge strength in the same test run; their
effects will be difficult to separate.

## 6. Sensor and Firebase Outage Tests

| Test | Required database/app behavior |
| --- | --- |
| Ultrasonic becomes unavailable | Write `999` once for that outage; display stale/offline |
| Any gas sensor becomes unavailable | Write `-1` once for that sensor's outage; display stale/offline |
| Repeated unavailable frame | No repeated sentinel database writes |
| One sensor recovers | Its first valid value publishes immediately; other unavailable sensors remain independent |
| Sensor UART unplug/replug | Bounded reconnect; no Pi CPU hot-loop; recovery value publishes |
| Firebase network outage | Motion safety remains active; writes coalesce and retry with bounded backoff |
| Missing both legacy and device nodes | Android reports offline after initial reads, not an endless loading state |
| FCM token refresh | Compatibility token paths update atomically; token value is absent from logs |

Use the supplied structure as the source of truth. Live values remain available
on the legacy `/RASPI/VALUES/...` paths and are mirrored under
`/devices/garby-bin-01/...` for the Android app.

The checked-in database template deliberately contains empty FCM token values;
the app writes the current token at runtime. Do not restore or commit an old
device token from a database export.

The Pi UART ultrasonic value is trash-bin level only. It must not confirm or
classify a front corridor obstacle; that role belongs to LiDAR plus the
executor's separate servo-mounted front sonar.

## 7. Thermal Endurance Test

Run for 30-60 minutes in the closed final enclosure, first with wheels lifted
and then on the supervised route. Watch:

- `/devices/garby-bin-01/status/cpuTemperatureC`;
- `thermalWarning` and `throttledFlags`;
- `bleConnected`, `lidarHealthy`, and `sensorSerialConnected`;
- Pi undervoltage/restart evidence, driver temperature, motor temperature, and
  BLE reconnect count.

The bridge now uses blocking serial reads, bounded reconnect backoff, latest-only
LiDAR work, coalesced Firebase writes, a 30-second health cadence, and headless
operation to lower load. These changes reduce overheating risk but cannot
guarantee safe temperature with inadequate cooling or power. Do not approve an
unattended run if `thermalWarning` appears, throttling flags are nonzero, the Pi
restarts, or link health flaps.

## 8. Floor Acceptance

Approve the system only after all of these pass repeatedly:

- five centered 5 m runs without accumulating wall drift;
- doorway/opening passage without an aggressive nudge;
- a person or object entering from front-left, front, and front-right at the
  worst expected payload and speed, with measured clearance before contact;
- obstacle removal without premature resume;
- BLE loss, Pi process stop, LiDAR loss, and sensor UART loss;
- reset during outbound travel, including Pi reset retries;
- Firebase outage and recovery;
- 30-60 minute thermal run with no restart, undervoltage, or throttle flag.

Record the final wheel trims, stopping distances, LiDAR orientation, nudge
constants, payload, floor surface, battery voltage, and thermal result. Those
measurements are the configuration baseline for future changes.
