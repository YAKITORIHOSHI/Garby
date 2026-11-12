# Hallway centering to the requested rules (2026-11-07, build B)

**Status: source implemented and software tested. Not flashed. Not physically accepted.** Bridge ID
`CENTER-20261107B`. Only the S3 sketch changed. The executor sources are byte-identical and the Pi needs no change.
Evidence is in [`artifacts/centering-spec-2026-11-07/`](../artifacts/centering-spec-2026-11-07/). This replaces the
behaviour in the [first centering report](lidar-mpu-centering-2026-11-07.md), which stays as history.

Every figure below comes from a model of the hallway (`tools/firmware_tests/hallway_sim.h`) wrapped around the real
bridge and executor firmware. It has no slip, backlash, floor or payload effects. **It does not show that the robot
drives straight.** No record of the floor behaviour you described exists: the bridge ran on this Pi at 01:49 and 02:13,
but its console was not saved. The causes below are what the model shows, not what was seen on your floor.

## The cost of a reference that never moves

The reference is now a constant, as requested. That removes the last build's reference trim, which is also what let
it survive a start that is not parallel to the walls. Sidesteps now have to undo that drift forever, so in the model:

| Start, 190 cm hallway, 90 s | Old: settled offset | New: settled offset | New: ends at the wall stop |
| --- | --- | --- | --- |
| Parallel | 0 cm | 0 cm | 0 of 24 runs |
| 1 degree misaligned | 15 cm | 24 cm | 0 of 24 |
| 1.5 degrees | 15 cm | 32 cm | 0 of 24 |
| 2 degrees | 16 cm | 40 cm | 3 of 24 |
| 2.5 degrees | 14 cm | at the stop | **24 of 24** |
| 3 degrees | 16 cm | at the stop | **24 of 24** |
| Gyro drift 0.02 deg/s | 15 cm | 31 cm | 0 of 24 |
| Gyro drift 0.05 deg/s | 18 cm | at the stop | **24 of 24** |

The wall stop is safe: it halts the robot, and an operator has to restart it. It is not what you want on a route, and
FRONT after an open-loop turn may well be 2 or 3 degrees off. **Measure this on the floor before relying on it** (step 4
below). If it reaches the wall stop with `BIASED REFERENCE?` lines, the choice is to align starts better, or to allow the
reference to change between corrections. That second option is a new decision for you, so nothing here does it.

## What was wrong

Measured on the unchanged `CENTER-20261107` firmware:

1. **Reference trim moved the target on a centered robot.** Noise on a short baseline read as drift. At 5 cm of wall
   noise, a robot 6 cm off center took 2.2 corrections, 12.8 taps and moved up to 20 cm. In the deadband it should do nothing.
2. **No hysteresis or confirmation.** One scan just over 8 cm started a correction.
3. **Taps stopped 0.5 degrees short of the target, which was the main zigzag.** After every sidestep the leftover heading
   sat on the side the robot had just come from, so it slid on at 0.6 cm/s, past the deadband and into a correction back.
   With every other rule in place: 359 swings at 0.5 degrees, 182 at 0.25, 17 at 0.1, **0 when the tap aims at the target**.
4. **Ending on the measured error overshoots.** At this scan rate the median is up to 2 s old, and unwinding 5 degrees takes
   2 to 5 s more, with the robot still moving sideways. Hence the exit prediction below: 244 swings without it, 0 with it.
5. **Nothing stopped a correction that was not working.** A time limit with no improvement simply ran again.

## Results, old against new

Worst or mean of 960 runs each (40 scenarios, 24 seeds, 90 s). Full tables: `matrix-old-vs-new.txt`.

| Metric | Old | New |
| --- | --- | --- |
| Full swings across +/-12 cm, all runs | 316 | **0** |
| Taps per run | 20.6 | **12.7** |
| Centered, 5 to 8 cm of wall noise: taps per run | 3 to 4 | **0** |
| 6 cm off with noise / 9.5 cm off: taps per run | 12.8 / 20.3 | **0 / 0** |
| 25 cm off: corrections, taps, RMS offset | 3.8, 23.4, 5.9 cm | **1.0, 7.3, 3.3 cm** |
| 50% over speed: corrections, swings | 5.8, 56 | **1.1, 0** |
| Taps 5x stronger: corrections, swings | 3.5, 9 | **1.0, 0** |
| Reversed LiDAR warned, normal hallway | 1 of 72 | **72 of 72** |
| Wall stops, leaving out reversed LiDAR | 1 | **123** (see above) |
| Heading swings across +/-2 degrees | 121 | 264, of which 262 from misaligned, drifting or 1 deg/s wheel runs |
| Open doorways every 5 m: corrections per run | 3.6 | 0.1: it holds heading and stays 15 cm off |

## Your rules, and where each one is

| Rule | Status | Where |
| --- | --- | --- |
| \|error\| at or under `CENTER_TOLERANCE_CM` (8): nothing happens, timer restarts | Done | `decide()` |
| error = (L - R) / 2 from the median of `LIDAR_SAMPLE_COUNT` readings | Done (`CENTER_BIAS_CM` stays 0) | `errorOf()`, `statsOf()` |
| Start only above tolerance + `HYSTERESIS_CM` (3) | Done | `decide()` |
| 2 or 3 consecutive off-center scans | Done: 2 (`OFF_CENTER_SCANS_REQUIRED`), same side | `decide()` |
| Units | **Consistent, no fault found.** LaserScan metres, `* 100` in `robust_wall_distance_cm()`, `L=`/`R=` in cm, no conversion in the bridge | Pi: `test_thin_side_object...`. Bridge: `L=50 R=110` gives `err=-30.0` |
| Reference saved at startup, never overwritten | Done, stronger than asked: `REFERENCE_HEADING_DEG` is a constant. `MpuHeading` zeroes yaw at `[HEADING:START]`. Only MPU recovery or an operator `R` re-zero it, both while held stopped | Top of `SteeringAssist.h` |
| Target = reference +/- `MAX_CORRECTION_ANGLE_DEG` (5) | Done, for every error size | `beginCorrection()` |
| Hold until in tolerance or `MAX_CORRECTION_TIME_MS`, then back to the reference | Done, with a prediction: see below | `recheck()`, `leaveCorrect()` |
| Time limit with no improvement: do not repeat, warn | Done, latched until an S3 restart | `correctionTimedOut()` |
| L > R steers left | Correct in software, end to end: target up, `NUDGE_LEFT`, slows motor 1. **Motor 1 being the physical left wheel is not verified** | Step 2 below |
| DRIVE_STRAIGHT -> SCAN -> CORRECT or CONTINUE -> DRIVE_STRAIGHT | Done | `update()`, `onScan()` |
| `CHECK_INTERVAL_MS` (3000), then rescan before any decision | Done, the first check included. A scan window takes 1.5 s, so scans are 4.5 s apart | `update()` |
| Scan only within a few degrees of the reference | Done: `SCAN_HEADING_TOLERANCE_DEG` (3). A reading outside skips the cycle | `onScan()` |
| Invalid or very large reading: skip that cycle | Done. One such reading ends the scan | `onScan()` |
| Front blocked or wall under `MIN_WALL_DISTANCE_CM`: obstacle avoidance first | Already held. Removing either one fails the tests | `computeNudgeCommand()` |
| Non-blocking, no `delay()` | Done. `loop()` keeps its existing 1 ms yield, for the idle task; a test checks nothing else waits | `test_bridge.cpp` |

## Where this differs from your rules

| Rule | What it does | Why | For the literal rule |
| --- | --- | --- | --- |
| Hold until the error is back within tolerance | Until the error *will land* within it: measured error minus yaw x speed x 1.75 s | 244 swings against 0 in 480 runs | `CORRECT_EXIT_LEAD_S = 0` |
| Warn on the time limit | Also when the next clean scan, or a STOP during the correction, finds it a full tolerance worse on the same side | With the prediction, a reversed sensor rarely reaches the time limit: at most 3 of 6 runs in a normal hallway | No constant |

Choices you left open, each from `sensitivity-one-constant.txt` (480 runs each):

| Constant | Value | Alternatives |
| --- | --- | --- |
| `OFF_CENTER_SCANS_REQUIRED` | 2 | 3: wall stops 88 to 96, and a correction starts 4.5 s later |
| `MAX_CORRECTION_TIME_MS` | 6000 | At 4000: 56 false sign faults, against 10 at 6000. At 8000: none, but a reversed robot strays 60 cm instead of 47 |
| `SCAN_HEADING_TOLERANCE_DEG` | 3 | "A few degrees" |
| `CORRECT_EXIT_LEAD_S` | 1.75 | 1.5: 1 swing. 2.0: 0 swings, but stops further off center |
| `HEADING_RELEASE_DEG` | 0 | Was 0.5: see cause 3. You did not ask for this one, and the zigzag does not go away without it |

Your 5 degrees also stands against the model's preference: 6 degrees gives fewer wall stops (75 against 88).

**Removed:** reference trim and drift measurement, `LOOKAHEAD_CM` and `MIN_CORRECTION_ANGLE_DEG`, the separate 4 cm exit
tolerance, the immediate first check, and the older two-strike sign checks.

## How to read a run

The console is not saved by itself, so capture it: `bash RasPi/run_bridge.sh 2>&1 | tee ~/garby-floor.log`

From the model, starting 25 cm off center:

```text
[BRIDGE CENTER 11.0s WIDTH 193cm L=71 R=122 captured]
[BRIDGE CENTER 11.0s SCAN L=71 R=122 err=-25.8 -> OFF CENTER 1/2, rescanning taps=0]
[BRIDGE CENTER 15.2s SCAN L=71 R=121 err=-25.0 -> CORRECTING right 5.0deg same=1]
[BRIDGE CENTER 19.1s CORRECT L=77 R=113 err=-17.8 -> DRIVE_STRAIGHT]
[BRIDGE CENTER 23.3s SCAN L=93 R=98 err=-2.8 -> CENTERED taps=6]
[BRIDGE CENTER 27.8s SCAN L=92 R=99 err=-3.2 -> CENTERED taps=1]
[BRIDGE CENTER 32.1s SCAN L=93 R=98 err=-2.8 -> CENTERED taps=0]
```

The sidestep ends at `err=-17.8` because the median lags and the heading has yet to unwind. The robot lands near -3.

`taps=` counts every tap since the last scan result, and the first line after a correction includes that correction's own
taps. **This is how to judge "a centered robot must never nudge".** The LiDAR now never asks for one in the deadband.
Heading-hold still taps when yaw leaves the 1 degree band, because taps are the only way to hold a heading: `taps=0` on a
run of `CENTERED` lines means no nudges at all, and a steady count means yaw is moving by itself. In the model, unequal
wheels at 0.5 deg/s take 67 taps per run even near the center. That is what `MOTOR1_TRIM_PCT` and `MOTOR2_TRIM_PCT` are
for, and no setting here removes it.

## Limits

1. **Nothing here has moved a robot.** Chassis response, tap strength, gyro noise under stepper vibration and LiDAR
   accuracy are unmeasured. The prediction assumes 71 cm/s, which is calculated from the header. The model tolerated 0.7x to
   1.5x speed and 0.5x to 5x tap strength.
2. **A misaligned start or gyro drift reaches the wall stop**, as in the first table.
3. **Doorways cost corrections.** One bad reading skips a cycle, as requested, and consecutive scans must be clean. With
   open doorways every 5 m the model almost never corrects, and just holds heading. Closed doors (8 cm recess) and posts are fine.
4. **A sign fault can be a false alarm** on a robot already heading for the wall stop: 2 to 7 of 24 runs at 3 degrees or
   0.05 to 0.1 deg/s. None on a robot within the first table's safe rows. It holds heading until an S3 restart.
5. **Eight sector summaries cannot tell a long flat object from a wall.** Above 250 cm to a wall, or with a hallway over
   400 cm wide, there is no centering at all, only heading-hold.
6. **The earlier supply problem is still open.** The Nov 4 review recorded S3 brownouts on the daisy-chained supply.

## Deploy

Flash the `BLE_Receiver-Final` sketch, including `SteeringAssist.h`. An image and hashes are in the artifact directory.
Confirm `[BRIDGE BUILD:CENTER-20261107B]` in the Pi log. No Pi or executor change is needed for this build.

## Physical checks, in order

Wheels lifted, hardware stop within reach:

1. Fix the S3 supply first ([Nov 4 review](ble-nudge-review-2026-11-04.md)).
2. Deflect the robot left: the correction must slow the RIGHT wheel. Confirm the reverse too.

On the floor, supervised, with `ENABLE_LIDAR_CENTERING = false`:

3. **Heading-hold only, 10 m.** Pass: heading within about 2 degrees. If taps fire constantly one way, set the motor trim first.
4. **Measure what the model only assumes:** real speed (`CRUISE_SPEED_CM_S`), how far FRONT is from parallel after a real
   turn, and yaw drift over 2 minutes at a standstill (`yaw=` on USB). Compare with the first table.

With centering on:

5. Tape-measure the robot to the center, parallel. `err=` should be near 0, or set `CENTER_BIAS_CM`. Expect `CENTERED taps=0`.
6. Start 7 cm off: `CENTERED`, and no taps. Start 10 cm off: `HOLD`, and no taps.
7. Start 25 cm off. Pass: `OFF CENTER 1/2`, one `CORRECTING`, no later correction back.
8. Start 2 to 3 degrees misaligned. Expect repeated corrections, `BIASED REFERENCE?`, and the wall stop from about 2.5 degrees.
9. Pass doors and posts. Expect `SKIP` lines, not swerves. Soft test objects: a veto must abort the correction, and the front stop must still work.

## Tuning

- **Still zigzags:** raise `CORRECT_EXIT_LEAD_S` (1.75 to 2.5). If taps overshoot, check that `HEADING_RELEASE_DEG` is 0.
- **Stops well short of center, or takes two corrections from one side:** lower `CORRECT_EXIT_LEAD_S` (1.75 to 1.25).
- **Nudges while `CENTERED`, `taps=` above 0:** wheel trim and gyro drift. Raising `HEADING_ENTER_DEG` only delays them, and it
  doubles or triples the heading swings.
- **`CHECK STEERING SIGN`:** check tap direction on lifted wheels and `GARBY_LIDAR_YAW_OFFSET_DEG` before any tuning.

If taps themselves are the limit, the agreed next step remains a continuous trim command in the executor. That changes
motor-driving code and needs its own review.
