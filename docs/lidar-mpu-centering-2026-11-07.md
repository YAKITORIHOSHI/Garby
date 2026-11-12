# LiDAR and MPU hallway centering (2026-11-07)

**Status: source implemented and software tested. Not flashed. Not physically accepted.** Bridge ID
`CENTER-20261107`. Only the S3 sketch changed. The executor sources are byte-identical and the Pi needs no change.
Evidence is in [`artifacts/lidar-mpu-centering-2026-11-07/`](../artifacts/lidar-mpu-centering-2026-11-07/).

Every figure below comes from a model of the hallway (`tools/firmware_tests/hallway_sim.h`) wrapped around the real
bridge and executor firmware. It has no slip, backlash, floor or payload effects, so it compares control designs and
catches regressions. **It does not show that the robot drives straight.**

## How it works

Two loops on different time scales, with the executor's bounded taps as the only actuator (option A):

```text
Inner, every MPU sample:   yaw vs target -> tap, released early from the gyro rate
Outer, every 5 s:          DRIVE_STRAIGHT -> SCAN -> CENTERED or CORRECT -> DRIVE_STRAIGHT
                           LiDAR only moves the target. It never drives a motor.
Above both, always:        STOP, nearest-object CF/CL/CR vetoes, MIN_WALL_DISTANCE_CM
```

The state machine is `HallwayCentering` in `BLE_Receiver-Final/SteeringAssist.h`, and every tunable is at the top of
that file. Each decision prints one line on USB and reaches the Pi log as `[BRIDGE CENTER ...]`:

```text
[12.0s] SCAN L=71 R=122 err=-25.9 -> CORRECTING right 4.9deg same=1
[BRIDGE CENTER 22.7s SCAN L=101 R=89 err=+6.1 -> CENTERED]
```

## What was actually wrong

A plain "check every 5 s and correct" scheme was built first and measured. It weaved **worse** than the old firmware,
with 5 to 7 swings in 90 s. Four defects were found, none of them in the state machine's shape:

1. **The heading band was too loose for the LiDAR's schedule.** A 2 degree band lets the robot hold 1.9 degrees
   indefinitely. At 71 cm/s, 1 degree is 1.2 cm/s, which is 6 cm per check. One trace slid from +6 to -17 cm at a
   constant -1 degree with no tap firing.
2. **Taps stopped after about a quarter of the error.** Early release assumed a fixed 0.25 s of coasting, but coast
   grows with the square of the yaw rate. Returning to the reference took 4 or 5 taps at 1.2 s each, while the robot
   kept sliding sideways.
3. **The lookahead was shorter than the loop's delay.** 120 cm at 71 cm/s is a 1.7 s time constant against about 2 s
   of delay (LiDAR median lag plus tap confirmation). That oscillates however good the logic is.
4. **The reference is rarely parallel to the walls.** FRONT is wherever the robot pointed after an open-loop turn, and
   the gyro drifts. At 3 degrees the robot drifts 24 cm per check cycle and the corrections cannot keep up. On the old
   firmware it parks 32 cm off center, because the 2 degree band uses up most of the 4 degree target cap.

## How LiDAR and MPU now work together

Each sensor covers what the other cannot see:

| Question | Answered by |
| --- | --- |
| Where am I across the hallway? | LiDAR only |
| Which way am I pointing, right now? | MPU only |
| Is my reference truly parallel to the walls? | **Both:** LiDAR measures real lateral speed while the gyro reports the heading held. Their difference is the error in the reference, so heading left inside the tap band is not mistaken for a bad reference |
| When should a correction end? | **Both:** the LiDAR error, minus how far the current yaw will still carry the robot (`CORRECT_EXIT_LEAD_S`) |
| Is the steering sign wrong? | **Both:** yaw has clearly turned the commanded way, yet the LiDAR error keeps growing |

## Results, old against new

Worst or mean of 144 runs (24 scenarios, 6 seeds, 90 s each). Full table: `matrix-old-vs-new.txt`.

| Metric | Old firmware | New firmware |
| --- | --- | --- |
| Runs ending at the wall stop | 18 | **0** |
| Worst settled offset | 40 cm | **27 cm** (all biases stacked; typically 13 to 17) |
| Mean RMS offset | 16.2 cm | **7.1 cm** |
| Full swings across +/-12 cm, all runs | 133 | **51** |
| 3 degree misaligned start, settled offset | 32 cm, parked | **16 cm** |
| 0.1 deg/s gyro drift, settled offset | 40 cm, stopped | **21 cm** |
| 25 cm off center, mean RMS | 8.5 cm | **5.6 cm** |
| Heading RMS / swings across +/-2 degrees | 0.49 / 3 | 0.73 / 57 |
| Taps per 90 s run | 22.7 | 29.2 |

**The last two rows are the cost.** Heading moves more because corrections now take effect, and the old firmware
often could not steer at all (its low figures include runs parked off center). 57 swings over 144 runs is about one
per 230 s, which is a slow correction rather than a zigzag, but it is not zero. Some weaving also remains, and most
when the robot is faster than assumed: 3 swings at 27% over speed.

## Override report

Against the requested specification. Restore a value by setting the named constant.

| Item | Requested | Now | Why |
| --- | --- | --- | --- |
| `LOOKAHEAD_CM` | 120 | 300 | Delay comparable to the time constant oscillates: heading swings 313 at 120, 57 at 300 |
| `ENABLE_REFERENCE_TRIM` | false | true | With it off, 3 degrees of misalignment walks into the wall stop (15,478 blocked packets against 0) |
| Trim calculation | Fraction of the last correction, after 3 same-direction | Fraction of the *measured* misalignment, at any out-of-tolerance check | Waiting for 3 corrections takes about 40 s; the wall stop arrives in about 10 s |
| `REF_TRIM_FRACTION` | 0.25 | 0.75 | It now scales a measurement, not a guess. 0.25 gives 7,128 blocked packets |
| `REF_TRIM_MAX_DEG` | 3 | 8 | Misaligned start plus drift over 100 s exceeds 3. At 3: 1,232 blocked packets |
| Trim timing | Only in DRIVE_STRAIGHT | At the end of SCAN, before CORRECT starts | Your rule that the reference never moves during CORRECT is kept. Waiting for DRIVE_STRAIGHT costs about 7 s |
| CENTERED | Keep the reference | Trims if the measured drift would leave tolerance before the next check | A robot crossing the center at 5 cm/s reads as centered, then hits the wall stop. It is not a correction. `ENABLE_REFERENCE_TRIM=false` removes it |
| `LIDAR_MAX_AGE_MS` | 300 | 650 | While the executor polls, S packets arrive every 300 ms, so 300 would skip almost every scan |
| `MAX_CORRECTION_TIME_MS` | L / v | L / v + `TAP_RESPONSE_MS` (1500) | A tap takes about 1 s to confirm and act. 0 restores the plain formula; the worst offset rises to 40 cm |
| CORRECT exit | Measured error at or under 4 cm | Measured error minus what the current yaw still adds | LiDAR lag overshoots otherwise. `CORRECT_EXIT_LEAD_S=0` restores it |
| Sign check | Error larger, twice | Larger by over `SIGN_CHECK_MARGIN_CM` (8), plus a new in-correction check | Plain "larger" falsely blames bias: 7 false faults in 144 runs at 0 |
| First check | After 5 s | Right after the width capture | 49 to 36 wall stops in the harsh misalignment sweep. Later checks use the interval. No constant |
| Width capture | At start | First clean 5-scan window | One scan cannot be trusted, and the robot is still in startup grace |
| Invalid scan in SCAN | SKIP | Not counted; `SCAN_TIMEOUT_MS` bounds the wait | A door crossing would otherwise waste the check. During CORRECT it still ends the correction at once |
| Inner loop | PD/PID differential trim at 50 Hz+ | Bounded taps (option A). Release at loop rate; new taps start only on a fresh scan | The executor has no trim input, and every tap needs fresh CF/CL/CR. So `HEADING_KP/KI/KD` and `MAX_TRIM` do not exist: `NUDGE_ERROR_SCALE_DEG` acts as P, gyro-rate release as D, and there is no I |
| LiDAR-lost fallback | MPU runs alone | No new taps without a fresh scan | Tap clearance needs fresh evidence, and the Pi path stops the robot within 850 ms anyway |
| `LR,<seq>,...` message | Added | Not added | `S:` packets already carry L, R, clearance and an acquisition ID. **No Pi change** |
| Constants location | Top of the sketch | Top of `SteeringAssist.h`; `MAX_YAW_JUMP_DEG` in `MpuHeading.h` | Host tests compile the header on its own |

Existing values also changed: heading band 2/1 to **1/0.5 degrees** (`HEADING_ENTER_DEG`, `HEADING_RELEASE_DEG`), and
early release from a fixed 0.25 s to a physical coast estimate (`TAP_COAST_DECEL_DPS2`, `TAP_RELEASE_LATENCY_S`).
`MPU_REVERSE_Z` is now derived from `YAW_POSITIVE_IS_LEFT`. Kept as requested: `CHECK_INTERVAL_MS`,
`CENTER_TOLERANCE_CM`, `CORRECT_EXIT_TOLERANCE_CM`, `CORRECT_RECHECK_MS`, `LIDAR_SAMPLE_COUNT`, `SCAN_TIMEOUT_MS`,
`MAX_SAMPLE_SPREAD_CM`, `WIDTH_TOLERANCE_CM`, `MIN_WALL_DISTANCE_CM`, `MAX_VALID_DISTANCE_CM`, `TARGET_SLEW_DEG_PER_S`,
both correction angle limits and `MAX_YAW_JUMP_DEG`. **GPIO is unchanged.** One-at-a-time results are in
`sensitivity-one-constant.txt`.

Safety behaviour kept: fail-closed path evidence, confirmation over 3 independent scans and 400 ms, the 1 s cooldown,
startup grace and ramp, nearest-object vetoes, and the operator stop. Repeated or backward acquisitions cannot fill a
window. New: a yaw step over 20 degrees between samples faults as `JUMP` and holds `STOP:MPU`; STOP goes on the wire
before anything is logged about it.

## Limits

1. **Nothing here has moved a robot.** Chassis response, tap strength, gyro noise under stepper vibration and LiDAR
   accuracy are all unmeasured.
2. **Speed is calculated, not measured.** 71 cm/s comes from 5600 steps/s, 1600 steps/rev and a 65 mm wheel. Trim and
   exit prediction use it, and they tolerated 30% slower and 27% faster in the model. Caution-zone speed (50 cm/s) is
   not reported to the bridge.
3. **Tap strength comes from the route calibration:** 90 degrees per 9800 differential steps. `WHEELBASE_MM 150` in the
   executor header is unused and would imply yaw five times stronger. A spin may scrub differently from a straight
   run, so the model was also run at half and double strength.
4. **A large misaligned start reaches the wall stop**, which halts the robot and needs an operator. In a 190 cm hallway
   the model tolerates about 3 degrees with up to 15 cm of offset toward that wall, and about 4 degrees from center.
   About 5 degrees from center reaches it. See `misalignment-envelope.txt`. Startup grace (2.4 s) is the largest delay
   before the first action, and it was deliberately left alone.
5. **A reversed LiDAR usually hits the wall stop first.** That is safe. `CHECK STEERING SIGN` appears reliably only in a
   wide hallway, so a robot that steers away from center and stops is the sign to look for in a normal one.
6. **Unequal wheels leave a steady offset.** 0.5 to 1 deg/s of mechanical yaw settles around 10 to 17 cm with many
   one-direction taps. There is no integrator. `MOTOR1_TRIM_PCT` and `MOTOR2_TRIM_PCT` are the real fix once measured.
7. **Eight sector summaries cannot tell a long flat object from a wall.** Above 250 cm to a wall, or with a hallway
   over 400 cm wide, there is no centering at all, only heading-hold.
8. **The earlier supply problem is still open.** The Nov 4 review recorded S3 brownouts on the daisy-chained supply. A
   brownout mid-segment ends heading for that segment.

## Deploy

Flash the `BLE_Receiver-Final` sketch, including `SteeringAssist.h` and `MpuHeading.h`. An image and hashes are in the
artifact directory. Confirm `[BRIDGE BUILD:CENTER-20261107]` in the Pi log. If the executor on the robot predates the
Nov 4 `[MCU ALIVE]` change, flash that sketch too. No Pi deployment is needed for this change.

## Physical checks, in order

Wheels lifted, hardware stop within reach:

1. Fix the S3 supply first ([Nov 4 review](ble-nudge-review-2026-11-04.md)).
2. Deflect the robot left: the correction must slow the RIGHT wheel. Confirm the reverse too.

Then on the floor, supervised, with `ENABLE_LIDAR_CENTERING = false`:

3. **Heading-hold only, 10 m.** Pass: heading within about 2 degrees. Watch the USB telemetry `yaw=`. If taps fire
   constantly in one direction, measure and set the motor trim before going on.
4. Measure speed over a known distance and set `CRUISE_SPEED_CM_S`.

Then with centering on:

5. Start centered and parallel. Expect `CENTERED` lines and almost no taps.
6. Start 25 cm off center. Pass: one correction per interval, and no reversal unless `err=` changed sign.
7. Start 2 to 3 degrees misaligned. Expect `reference trim` lines, then settling. Repeated `BIASED REFERENCE?` means the
   trim is not keeping up.
8. Pass doors and posts. Expect `SKIP` lines, not swerves. Place soft test objects: a veto must abort the correction,
   and the front stop must still work.

## Tuning

- **Still zigzags:** raise `LOOKAHEAD_CM` first (300 to 400), then widen `HEADING_ENTER_DEG` and `HEADING_RELEASE_DEG` if taps chatter.
- **Never centers:** lower `LOOKAHEAD_CM` (300 to 200), raise `TAP_RESPONSE_MS`, and look for `BIASED REFERENCE?` or `SKIP` reasons in the log.
- **Measure first:** `CRUISE_SPEED_CM_S` and the wheel trim, because both shift everything else.

## If option A is not enough

Taps limit the inner loop to about one correction per 1.2 s. If floor runs still weave after tuning, the next step is
option B: a continuous trim command in the executor. That changes motor-driving code and needs its own review.
