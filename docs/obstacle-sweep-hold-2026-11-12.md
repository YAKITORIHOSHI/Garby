# Holds that resume, sonar sweep, moving-object prediction, pre-run centring, running beep — 2026-11-12

Status: **source implemented, software tested, ESP32 target compiled. Not uploaded, not floor tested.**
Executor build `NAV-20261112`. The BLE bridge sources did not change (still `NAV-20261111`).
Deploy the complete `RasPi/` directory. Evidence: `artifacts/obstacle-sweep-hold-2026-11-12/`.

**MCU upload for this change: Executor only** (`MAIN_EXECUTOR_CODE`). It needs the bridge's
`NAV-20261111` M/W-line protocol; as of 2026-11-11 neither board had that build, so if the S3 is
still older it must be flashed with the unchanged `BLE_Receiver-Final` sketch too (earlier pending
image, not a requirement of this change).

## 1. What the owner asked for, and what was found

| Request | Found in the code | Done |
|---|---|---|
| Never stay "stuck/blocked forever"; low power while blocked; resume when clear | Three permanent-stop paths, all reproduced by tests on the unchanged firmware (`logs/test_hold_resume-on-unchanged-firmware.log`): (1) a ping with **no echo** was "unknown" and reset the clear streak, so once something had been seen ahead, an open hallway (nothing within ~3 m, the normal case) could **never** release the sonar hold; re-enabling the sonar latched the same way. (2) A STOP that arrived while the start gate waited for a sonar sample (or just after a hold) made the segment return false, which **latched a route fault**. (3) A heading stream missing **10 s** latched a route fault; an S3 brownout restart (logged 2026-11-04, up to 2 min without advertising) does exactly that | No echo = far clear reading (same 3-sample confirmation). Every segment re-holds instead of failing (`waitUntilMotionPermitted`). Heading hold 5 min. Low-power hold after 10 s |
| Ultrasonic + servo 180° sweep; servo "not moving at all" in this codebase | The servo was only ever commanded to the forward view (70). There is no hardware conflict: the steppers use MCPWM/PCNT, the servo LEDC (confirmed in the ESP32 build) | `SonarSweep.h`: sweeps while driving and while stopped (section 3) |
| Detect people/objects walking or running toward GARBY and stop before an accident | Only static distance limits (95 cm box): a person walking at 1.4 m/s toward the robot at 0.71 m/s closes at 2.1 m/s, leaving 0.3 s | `RasPi/motion_tracker.py`: tracks, ground velocity, time to contact (section 4) |
| After the turn into the long hallway, centre with the LiDAR before the long run | Turns were MPU + wall *heading* aligned, but a lateral offset (first leg ending off centre, imperfect turn) was left for the in-run centring to steer out over metres | `centerBeforeLongRun()` before both long runs (section 2) |
| Continuous beeping so people notice GARBY | Beeps only every 5 s while held | Persistent buzzer task (section 5) |

## 2. Pre-run centring (the owner's idea)

`runLongHallway(steps)` in `pointsRun.ino` replaces the two long `safeMoveDistance` calls
(595500 / 595700 steps; all step counts and turn angles unchanged):

1. Stand still (`TURN_SETTLE_MS`), then take **two independent wall scans** with both walls,
   a plausible width (120-450 cm) and the hallway heading, agreeing within 3 cm and 1.5°.
   `lateral = (WR - WL) / 2` (+ = left of centre), `axis = gyro heading at the scan - H`.
2. Reject walls more than 8° off the route (not this hallway) or an offset over 60 cm.
3. Adopt the measured axis as the route frame. Within **3 cm** (inside the in-run centring's
   5 cm engage threshold): face the axis if more than 0.75° off, done.
4. Otherwise **sidestep**: spin 35° toward the centre (MPU), drive the diagonal
   `|lateral| / sin 35°` (a plain move with every obstacle hold), spin back to the axis and
   wall-align. Each spin is a normal turn segment: the bridge needs the LiDAR spin sweep
   clear, and the sonar checks the spin circle at floor level.
5. Measure again (up to 3 sidesteps). The first sidestep teaches the real wheel distance
   (`[HEADING CENTER learn k=..]`, k = actual/assumed travel: a free STEPS_PER_CM calibration,
   the open 1600 vs 3200 steps/rev question). A sidestep the walls show going the wrong way
   (a LiDAR/MPU left-right sign problem) stops the centring: `stop=WRONG_WAY`.
6. The sidesteps' travel along the hallway (`steps × cos 35°`) is taken off the long straight.

No walls, walls off the route or a failed measurement **skip** it (the in-run centring is
unchanged and still works). Model results (`tools/firmware_tests/test_navigation.cpp`):

| Case | Result |
|---|---|
| 22 cm off centre and 3° off the axis after a turn | +0.1 cm, -0.36°, one sidestep |
| Wheel travel half the firmware's assumption | learns k = 0.5, centred on the second sidestep |
| Already centred | no motion |
| LiDAR left/right reversed against the MPU | one sidestep, then `stop=WRONG_WAY` |
| Whole outbound route, each turn leaving the robot 6 cm off | largest centring command on the long run **0.00°** (1.67° before) |

Log lines (Pi log, `[HEADING ...]`): `[HEADING CENTER pass=1 y=+12.3 hdg=+0.40 w=190]`,
`move=R 21.4cm at 35 deg`, `learn k=..`, `done y=+0.8 sidesteps=1`, `skip=NO_WALLS`.

## 3. Sonar sweep and holds that always resume (executor)

**Sweep** (`SonarSweep.h`, bearings: 0 = ahead, + = left):

| Robot state | Pattern | Purpose |
|---|---|---|
| Driving straight | forward, +25, forward, -25 | forward re-read every ~0.1 s (host model, echoes add ≤18 ms); ±25 covers the body's corners |
| Stopped inside a route (holds, start gates, before turns) | forward between each of ±25, ±50, ±70 | the whole servo span 0-140 (with the beam width about 165° in front), forward re-read every ~0.2 s (worst 0.29 s) |
| Spinning in place; idle; parked | forward only (slow pings idle/parked) | the rotation sweeps it; no servo wear at base |
| Low-power hold | forward, slow pings, one full survey every 4 s | |

Each echo is placed conservatively (beam edge nearest the path): an echo at ±50/±70 counts
only if it can be inside the 60 cm-wide corridor (+8 cm); the hallway walls (95 cm) and 82 cm
pillars never hold or slow the robot (tested). Inside a turn the only sonar question is the
**spin circle** (50 cm radius): a wall 50 cm ahead of the sonar no longer stops a spin (it
could have held a turn near a pillar forever), a box beside a front corner does. Before every
spin the robot waits up to 2.5 s for a full survey. 2 consecutive near echoes at one bearing
block; 2 clear ones release it; forward keeps its 3-sample latch.

**No echo** (nothing within ~3 m) is a far, clear reading. A sonar that hears nothing at any
bearing in three complete surveys is reported once (`[SONAR:SILENT]`, Pi warning; driving
down an open hallway never counts toward it): it cannot
hold the robot, like the operator's SONAR_OFF. An ECHO line stuck high counts as no echo and
is reported (`[SONAR:FAULT]`).

**Holds**: `waitUntilMotionPermitted()` is the only way into motion: fresh GO, no STOP, no
sonar obstacle. A STOP at any moment just holds again. A segment now fails only for an
explicit reset, the operator emergency stop, a latched fault, a rejected wheel command, a turn
that goes the wrong way twice / hits its step limit, or **5 minutes** without the MPU stream.
After a heading outage over 10 s with a new S3 reference, two wall scans must confirm the
route direction within 8° of its right-angle grid (an outage in the middle of a turn may leave
the old hallway's walls in view) before the robot moves on, or the route faults (someone may
have turned it). Only that small residual is corrected.

**Low-power hold** (after 10 s with no clearance evidence): CPU 240 → 80 MHz (the 80 MHz
peripheral clock is unchanged, so UART, step timing and servo PWM are unaffected), status
polls 10 → 2.5 Hz, forward-only sonar at 4 Hz with a survey every 4 s, sparse chirp, `[POWER:LOW]`.
The **first** clear evidence restores normal power (`[POWER:NORMAL]`), before the confirmations
finish and the wheels move (test: low power +10.0 s → normal +15.2 s → released +15.4 s). The
stepper drivers' holding current is the largest load and needs their ENABLE wired:
`MOTOR_ENABLE_PIN` (now -1, not wired) disables them in low power once set.

## 4. Moving-object prediction (Pi)

`RasPi/motion_tracker.py`, run on every scan inside the LiDAR callback (2.3 ms median on this Pi 5):

1. Segment the scan (adaptive breakpoint); segments longer than 90 cm are structure (walls,
   pillar faces, door leaves) and are not tracked. The visible half of a body is shifted back by
   (π/4)·r, the bias of LiDAR samples on a round body.
2. Kalman-track each object with **ground** velocity: the executor's new `[DRIVE:F:<cm/s>]` /
   `[DRIVE:T:..]` / `[DRIVE:S]` lines remove the robot's own motion (a report that arrives up to
   0.5 s after a scan's acquisition still applies to it); without them (an older executor, the
   first 2 s after a reconnect) the robot speed is only known to be 0-75 cm/s, the
   classification allows for all of it, and only objects closing faster than 1 m/s count.
3. Moving = for 3 consecutive *measured* updates (2 above 1.5 m/s): speed ≥ 35 cm/s beyond
   every still-object explanation and ≥ 2σ, plus ≥ 20 cm real ground travel over ≥ 0.45 s
   (0.3 s above 1.5 m/s). A coasting track, a segment that just split/merged, or a track found
   again after missed scans (it may be another object) cannot be moving.
4. Time to contact with the footprint grown by 20 cm front, 15 cm sides, 10 cm rear and the
   object's radius (slab method). Sideways motion is extrapolated only when it is real (≥ 25
   cm/s and ≥ 2σ). **≤ 2.0 s: STOP** (`F=O`, generic obstacle code), **≤ 4.0 s, or a mover
   closing within 2.5 m ahead: caution speed** (`F=W`), a mover reaching the spin circle within
   2 s: **turn held** (`T=O`). Something catching up from behind is ignored (the 35 cm rear sector
   still stops for it); something already within the margin (or the spin circle) but abeam or
   moving off is passing.
5. STOP/SLOW are held 0.6/1.0 s after the last positive prediction.

It **only adds** STOP/SLOW evidence; the bridge, sectors, zones and every executor gate are
unchanged; the read-only front bypass bypasses it too. No BLE protocol change. It reports
"moving object", never "human": the LiDAR at the top of the bin sees torsos at chest height
and nothing below its plane (children, chairs, bins: the sonar's job).

| Synthetic hallway (190 cm, pillars, open door, swung leaf; 1-2 cm noise, dropped rays) | Result |
|---|---|
| Walker 1.4 m/s head-on, robot at 0.71 m/s | STOP 1.04 s after appearing, **4.8 m** apart (static box: 0.95 m) |
| Runner 4.5 m/s head-on | STOP after 0.52 s, **4.9 m** apart |
| Crossing into the path / crossing well ahead | STOP / no stop |
| Walking away ahead; catching up from behind; walking against the wall | no stop |
| Passing 19 cm from the robot's side | caution speed, no stop |
| Walking toward a stopped robot | stays stopped |
| Robot driving 14 s / standing 40 s / ramping with late `[DRIVE]` reports | no stop, no slow |

**Real LiDAR, isolated bench runs** (no-match BLE identity, no Firebase, `ROS_DOMAIN_ID=77`,
scratch stop/lock files; the owner's cluttered desk, nobody walking on purpose): the production
bridge started the X3 and ran the prediction live. The first version flagged clutter (4-5 brief
STOPs, 40-52 scans with a "mover": coasting tracks, segments splitting/merging); after the
coasting/shape/displacement gates and the review's re-acquisition rule, the final run (440
scans, saved as `bench/scans-bench-static.npz`, now a unit test) gives **0 stops, 0 slows,
0 turn holds, 0 movers** live and in replay, at 2.5 ms per scan.

## 5. Buzzer

One persistent task plays a pattern (`MAIN_EXECUTOR_CODE.h`): **driving a route** 90 ms beep
every 1.2 s; **held** two 60 ms chirps every 2 s; **low power** 50 ms every 5 s; silent idle,
parked at the destination, route-faulted or emergency-stopped. The first-link "ready" beep
stays. Active buzzer on GPIO25 assumed (as before: the old code drove it HIGH/LOW).

## 6. Telemetry lines added (executor → bridge → Pi, forwarded unchanged by the bridge)

`[DRIVE:S]`, `[DRIVE:F:<cm/s>]`, `[DRIVE:T:<deg/s>]` (on change, every 2 s), `[POWER:LOW]`,
`[POWER:NORMAL]`, `[SONAR:SILENT]`, `[SONAR:FAULT]`, `[HEADING CENTER ...]`,
`[HEADING LOST verify=OK|FAIL|NO_WALLS ...]`, `[HEADING TURN ... align=SKIP ...]`. Telemetry
only: none grants motion. USB console: `[SONAR SWEEP -70:101 -50:--- ... +70:96]` every 5 s
while surveying (`---` no echo, `!` blocking).

## 7. Verification

| Check | Result |
|---|---|
| New executor tests on the **unchanged** firmware | 5 of 5 FAIL (the stuck paths and the idle servo) |
| Self-review of the diffs (the independent reviewer hit the usage limit) | 5 more defects, each shown failing without its fix: false `SONAR:SILENT` after a long drive, too-broad side emergency, false route fault after a long outage mid-turn, drive reports read as "from the future", a lost track re-matched to another object (real recording) |
| Firmware host suite | 556 PASS (522 before); `test_hold_resume.cpp` new; navigation model extended |
| ESP32 executor build (esp32:esp32:esp32, 4 MB) | OK, 415,822 bytes (31 %); only warning is inside FastAccelStepper |
| Pi suite | 442 OK on a private Xvfb `:77` (none skipped), 0 deprecation warnings |
| `source_sanity.py`, `verify_project.py --host-only` | PASS |
| Bridge sources | unchanged (hashes in `sha256-after.txt` vs `before/sha256.txt`) |

## 8. Bench and floor steps (send back the saved Pi log and the executor USB log)

`bash RasPi/run_bridge.sh 2>&1 | tee ~/garby-$(date +%F-%H%M).log`

* **B1 boot**: executor USB shows `[EXECUTOR BUILD:NAV-20261112 ...]` and
  `[SERVO] Ready: forward 70, sweep 0..140`.
* **B2 servo and sonar** (wheels lifted, start a route so it is "in a route"): the servo
  sweeps while held; the USB log prints `[SONAR SWEEP ...]`. Check each bearing against a
  tape: a box 50 cm straight ahead reads ~50 at `+0`; the hallway walls read ~95-100 at ±70.
  If the servo buzzes at an end, lower `SCAN_LEFT`. For a true ±90° sweep the horn must be
  re-mounted so straight ahead is command 90 (then `DEFAULT_VIEW 90`, `SCAN_RIGHT 0`,
  `SCAN_LEFT 180`). Confirm the sensor sits below the LiDAR plane (A7 of the Nov 11 guide).
* **B3 hold and resume**: wheels lifted, during a straight put a box 40 cm in front of the
  sonar: wheels stop, double chirp; after 10 s `[POWER:LOW]` and the sparse chirp. Remove it:
  `[POWER:NORMAL]`, wheels resume within ~1-2 s. Repeat with a hand in front of the LiDAR.
* **B4 moving objects** (robot stationary, wheels lifted): walk toward the LiDAR from 6 m;
  the Pi log shows `Moving-object prediction: STOP (moving object x.x m away ...)` at about 3-5 m
  and `clear` after you pass. Walk past along the wall: `caution speed` or nothing, never STOP.
  Stand still in front: no moving-object line (the static limits still apply).
* **F1 floor, supervised**: run to the first long hallway; send the `[HEADING CENTER ...]`
  lines and the first minute of `[HEADING FUSE ...]` (expect `ctr=+0.00`). Note k if logged.
* **F2 floor people**: someone walks toward the robot in the long hallway; another person
  passes along the wall. Note where it stopped (tape) and how long it took to resume.

## 9. Assumptions and limits

* Nothing here is physically proven: the servo's span and direction, the sonar's mounting
  height and beam, the buzzer's polarity, CPU switching on this board, the pre-run centring
  on the floor and the predictor on real people are all bench/floor items above.
* Sonar geometry defaults: sonar 30 cm ahead of the spin centre on the centre line, 60 cm base,
  ±12° bearing uncertainty, spin radius 50 cm. Unmeasured, constants in `SonarSweep.h`.
* Treating no echo as clear means a soft/angled object that stops echoing reads as gone. The
  LiDAR must also be clear before any hold ends, and the side bearings keep looking.
* The predictor needs a target in the LiDAR plane (chest height): it does not see children,
  pets, chairs or bins; it cannot tell people from objects; dark clothing may return few points.
* Holding instead of faulting means a robot that is never cleared beeps and waits (low power)
  indefinitely; the existing 10-minute "still blocked" SMS still goes out.
* Stopping distance is still estimated, not measured (Nov 11 guide, F2).
