# Motor direction reversed, 2026-11-07

The owner reported that movement and nudges ran reversed: forward went backward, backward went forward, left went right
and right went left. This change corrects all of them. Source implemented and software tested. **Not flashed, not
physically tested.**

## The change

Two values in `MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.h`, applied by `setup()`:

| Constant | Was | Now |
| --- | --- | --- |
| `MOTOR1_DIR_HIGH_COUNTS_UP` | `false` | `true` |
| `MOTOR2_DIR_HIGH_COUNTS_UP` | `true` | `false` |

No GPIO changed. Route numbers, turn code, nudge code, the S3 bridge and the Pi are untouched. Flash the executor only.

## Why one change fixes everything

Routes, turns and nudges all treat a rising step count as forward. These two values are the only link between that count
and the real wheels, so swapping them reverses every wheel in every command:

| Command | Wheel steps | Before | Now |
| --- | --- | --- | --- |
| Route leg | Both up | Backward | **Forward** |
| `turnLeft` | Motor 1 down, motor 2 up | Right | **Left** |
| `turnRight` | Motor 1 up, motor 2 down | Left | **Right** |
| `NUDGE_LEFT` | Motor 1 slows | Turned right | **Turns left** |
| `NUDGE_RIGHT` | Motor 2 slows | Turned left | **Turns right** |

**Do not also swap the turn or nudge code.** The report fits only one wiring: motor 1 is the left wheel, and both wheels
ran backward on a rising count. So slowing motor 1 is already the right way to steer left once the robot rolls forward.
Swapping the nudge as well would reverse it a second time, and heading-hold would run away again. Negating the route legs
instead would disable steering altogether, because `moveToTarget()` only steers on rising counts.

## Why the old nudge looked reversed

While the robot rolled backward, slowing the left wheel turned it right. So every correction pushed the heading further
the wrong way, and the next correction added to that.

The floor run saved at 03:44 fits this, and the real firmware does the same in the hallway model with the old polarity:

| | First scan | Second scan, 3 s later | Then |
| --- | --- | --- | --- |
| Floor run | `L=81 R=112 err=-15.2`, heading off reference | `L=66 R=130 err=-32.0`, walls lost | `LiDAR obstacle` |
| Model, old polarity | `L=82 R=113 err=-15.2`, heading off reference | `L=66 R=131 err=-32.6`, heading off reference | Wall stop |

The model's start offset (10 cm) and wheel mismatch (-0.3 deg/s) were chosen by hand to resemble the run, so this shows the
behaviour is consistent with the report. It does not prove the cause on its own. Given the same start, the new polarity
holds its heading to 1.2 degrees and stays clear of the wall stop. With a 1% wheel mismatch, heading-hold alone stays
within 1.6 degrees on the new polarity and runs away to 16.5 degrees on the old one.

## Check the LiDAR before driving

**The firmware cannot tell which way the LiDAR faces, and this change makes that matter.** The robot now travels the
opposite way, and the Pi's limits are not symmetric: 95 cm in FRONT, 60 cm on the front diagonals and 35 cm at the BACK. It
assumes the robot is travelling toward FRONT. `GARBY_LIDAR_YAW_OFFSET_DEG` is still its unverified default of 0, and
nothing saved shows which way FRONT points: the floor log would look the same either way.

If FRONT does not point the way the robot now travels, then:

- The robot approaches obstacles with only the 35 cm rear limit.
- LiDAR left and right are exchanged, so centering steers toward the nearer wall until it latches `CHECK STEERING SIGN`.

The 03:43 log also shows `SONAR_OFF` requested, which would leave the LiDAR as the only stop.

Set it from observation. If FRONT and BACK are exchanged, start the bridge with the offset. That also corrects left and right:

```bash
GARBY_LIDAR_YAW_OFFSET_DEG=180 bash RasPi/run_bridge.sh 2>&1 | tee ~/garby-floor.log
```

## Physical checks, in order

Nothing below has been done. Keep the hardware stop within reach.

1. **Flash `MAIN_EXECUTOR_CODE`, then power-cycle the whole robot.** A `CHECK STEERING SIGN` latched by a reversed run lasts
   until the S3 restarts.
2. **LiDAR, parked.** In the monitor window (it draws "Front at top"), hold a box about 50 cm from the robot's real front:
   `Front` must react. Repeat on its left: `Left` must react.
3. **Wheels lifted, press Start.** First both wheels roll forward for about 4 s, starting with five short bumps, with the top
   of each wheel moving toward the robot's front. Then a left turn for about 2 s: left wheel backward, right wheel forward.
4. **Wheels still lifted, on the long hallway leg.** Rotate the robot to its left: the RIGHT wheel must briefly slow.
   Rotate it to its right: the LEFT wheel must briefly slow.
5. **Before the floor run, check the route.** From the same parked spot the robot now sets off the opposite way, and the
   whole route is rotated 180 degrees about the start compared with before. Park it facing the way the route should begin.
6. **Floor, supervised, console saved:** `bash RasPi/run_bridge.sh 2>&1 | tee ~/garby-floor.log`. Expect the first scan to read
   `CENTERED`, `HOLD` or `OFF CENTER`, not `SKIP: heading off reference`. Then follow the floor checks in the
   [centering report](centering-spec-2026-11-07.md).

| What you see at step 3 | What it means | Next |
| --- | --- | --- |
| Forward, then a left turn | As intended | Continue |
| Backward, then a right turn | The old firmware is still on the board | Flash again. Do not change the code |
| Forward, then a right turn | Motor 1 is the right wheel, which the report ruled out. Polarity cannot fix this | Stop. Do not flip the values again |
| Backward, then a left turn | The same crossed wheels, with the polarity also reversed | Stop, as above |
| Wheels roll opposite ways when going forward | Only one wheel's direction is right | Stop. Check that wheel's driver and DIR wiring |

## Limits

- **The wiring is inferred, not measured.** The tests model the owner's report, and step 3 is what confirms it.
- **The route was calibrated without this polarity.** Step counts are unchanged, and slip, load and turn accuracy going
  forward are unmeasured.
- **The hallway model has no slip, backlash, vibration or floor effects.** It can compare designs. It cannot show that the
  robot drives straight.
- **A wheel mismatch or a misaligned start still costs what the [centering report](centering-spec-2026-11-07.md) says.** With
  the steering sign right, those limits apply for the first time on a real floor.
- **The S3 supply problem from the [Nov 4 review](ble-nudge-review-2026-11-04.md) is not addressed here.**

Evidence: `artifacts/motor-direction-2026-11-07/`, starting with `validation.md`.
