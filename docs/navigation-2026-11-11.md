# Hallway navigation: root cause, redesign, calibration and acceptance — 2026-11-11

Status: **source implemented and software tested. Not uploaded, not floor tested.**
Bridge build `NAV-20261111`, executor build `NAV-20261111`. Both boards must be
flashed together (the UART protocol between them changed). Deploy the whole
`RasPi/` directory with them.

The owner handed over the work on 2026-11-11 ("Please takeover and decide what do
you think will work, I will approve every proposal you have"). That approval covers
the design decisions below, including replacing the November 7 rule that the
heading reference never moves. It does not cover uploading firmware or moving the
robot; nothing in this work did either.

Evidence: `artifacts/navigation-2026-11-11/validation.md`.

## 1. Root cause of the off-centre runs

It is not a units or sign slip. The audit found the arithmetic consistent (cm on
the wire, degrees in firmware, ROS metres converted once on the Pi, left-positive
yaw everywhere). The cause is structural:

1. **The heading the robot held had no absolute reference.** Yaw is integrated
   gyro only. The deployed design zeroed it at each segment start, right after an
   open-loop turn, and then held that zero rigidly. Any error in that zero (turn
   error, placement, start-up bias) is held exactly, and the robot walks sideways
   by `distance x sin(error)`: 1 degree is 1.75 cm per metre, 1.3 m over the 76 m
   leg if the header geometry is right. A residual gyro bias of 0.05 deg/s adds
   about 3.5 m over that 106 s leg.
2. **LiDAR could not correct it.** Walls were used only as a thresholded left/right
   trigger (8 cm + 3 cm hysteresis, two scans 3 s apart) for a bounded sidestep —
   about 2 degrees of average authority. Any bias larger than that ends at the wall
   stop. The heading error itself was never measured.
3. The interim design found in the tree this morning made it worse: one gyro frame
   for the whole route with LiDAR capped at 3 degrees.

The closed-loop model reproduces it: from a 3 degree start error with 0.05 deg/s
drift and a 0.4% wheel mismatch, gyro-only heading hold ends **64.4 cm** off centre
after 15 m (`test_navigation.cpp`, "ROOT CAUSE reproduced").

Other audit findings:

| Item | Finding | Action |
|---|---|---|
| Wheel geometry | 4900 steps per 90 degrees with 1600 steps/rev and 65 mm wheels implies an 80 cm track; the robot is 60 cm wide. One of these is wrong (likely 3200 microsteps/rev) | Steps per degree are learned from MPU turns; steps per cm is measured in calibration step 4 |
| Old side stop | 22 cm radial from the LiDAR is inside a 60 cm body | Chassis side band added (section 4) |
| Gyro tilt | Only raw Z was integrated; calibration accepts up to 37 degrees of tilt | Yaw is now the gyro vector projected on measured gravity |
| Gyro scale | MPU6050 sensitivity is +/-3% (about 2.7 degrees per 90) and invisible to the gyro | `GYRO_SCALE` constant plus wall alignment after every turn |
| Gyro bias | Measured once at boot | Re-measured while held stopped or parked |
| MPU loss | Turns fell back to step counting; straights ran blind | Robot now stops; route fails after 10 s without heading |
| Timing | Scans 5.9 Hz, about 0.18 s old on arrival | Each wall scan carries its age and is matched to the gyro heading of that moment |
| LiDAR angles | Driver runs `inverted:=false` (SDK native, clockwise). The Nov 9 reflection is consistent with that | Unchanged. Mount yaw (`GARBY_LIDAR_YAW_OFFSET_DEG=0`) still unverified |
| Indexing | No off-by-one found | — |

## 2. New design

```
Pi (RasPi/)                 BLE            S3 bridge (BLE_Receiver-Final/)     UART          Executor (MAIN_EXECUTOR_CODE/)
LaserScan -> sectors (as before) ------ P:  F/B/T codes -> GO / GO:SLOW / STOP ---------> path gate (2 GO)
          -> chassis zones (new)         S:  CF/CL/CR -> steering permission bits   M 50 Hz: gyro yaw, rate, epoch, permission
          -> wall line fits (new)        S:  WL/WR/H/G -> W line per scan          -> W: wall fit + age -> fusion
                                         MPU6050 (tilt-compensated, bias refresh)
```

* **Pi** (`hallway_geometry.py`, new): per scan, a deterministic Hough + least-
  squares line fit for one wall on each side gives its perpendicular distance from
  the chassis centreline and the robot's heading relative to it. Doorways, posts,
  open doors, door leaves and curved surfaces are outliers, not walls (tested with
  the owner's hallway figures). It also evaluates chassis-sized zones (section 4).
  The existing 8-sector safety logic is unchanged; zones only add STOP/SLOW.
  This departs from the earlier plan to drop the sectors: they are the tested,
  fail-closed core, and the boxes cover what they missed.
* **Bridge**: one continuous heading reference from boot (no re-zero per segment),
  streamed at 50 Hz. Per new LiDAR acquisition it computes steering permission from
  nearest-object clearance and forwards the wall fit with its age. It no longer
  decides centering and never sends taps.
* **Executor** (`HeadingControl.h`): fuses gyro and walls.
  `axis = gyro heading at the scan - H` (an alpha-beta filter that also learns
  residual gyro drift), `lateral = (right - left) / 2` (single wall: with the
  learned width), carried between scans by step travel. Steering target =
  `axis + centring term`, centring = `-atan(excess / 120 cm)` beyond a 2 cm
  deadband, engaging above 5 cm and releasing below 2 cm, capped at 5 degrees,
  slewed at 4 deg/s. The inner loop is the existing continuous wheel-speed heading
  hold. Through wall gaps the gyro carries the hallway direction. At the end of a
  centred straight the measured hallway axis becomes the route frame, so the next
  turn starts from the real hallway.
* **Turns**: closed loop on MPU degrees (90/90/90/90/180 as before), tolerance 1
  degree, then a **wall alignment**: two fresh scans after the spin; if they agree
  within 1.5 degrees and are within 8 degrees of the commanded direction, the robot
  spins to the measured hallway direction (tolerance 0.75 degrees) and the route
  frame is set to it. Every turn logs commanded vs achieved (section 6).

Model results (`tools/firmware_tests/test_navigation.cpp`; a model, not the floor):

| Case | Result |
|---|---|
| 3 degree start error, 0.05 deg/s drift, 0.4% wheel mismatch, 15 m | gyro-only +64.4 cm; fused +2.5 cm, max 3.5 cm, heading -0.7 degrees |
| 30 cm off centre | back within 5 cm, no overshoot, heading command within 5 degrees |
| centred (|y| 1.5 cm) | no centring command at all |
| 4 m with no walls either side | max 15 cm, re-centred after |
| one wall missing for 10 m | centred within 6 cm on the other wall and learned width |
| wall heading 9 degrees wrong for 0.7 s (door leaf) | gated out |
| 3% gyro scale error, 90 degree turn | gyro alone ends at 92.8; aligned to within 1.2 |
| whole outbound route, 2% scale, 0.03 deg/s drift, unequal wheels | completes, final heading error +0.49 degrees |

## 3. Protocol changes (why, and every file affected)

Needed because the LiDAR must measure heading and the executor must fuse it; the
tap interface could carry neither. The BLE framing, sequencing, ACK handshake,
operator, health, start and reset messages are unchanged.

| Link | Change |
|---|---|
| Pi -> bridge `P:` | `F` may be `W` (clear, caution zone ahead). New optional `T=<C|O|S>` spin-sweep clearance. An old bridge treats both as malformed -> STOP (fail safe) |
| Pi -> bridge `S:` | Adds `WL=`, `WR=` (wall distances cm), `H=` (heading vs walls, deg, + left), `G=` (scan age ms). Old fields kept |
| bridge -> executor | `M,<seq>,<hdg cdeg>,<rate cdps>,<epoch>,<allow>*XX` (offset field removed), new `W,<acq>,<age ms>,<flags>,<L mm>,<R mm>,<H cdeg>*XX`, `GO:SLOW`, `STOP:TURN`, `STOP:TURN_UNKNOWN`. `N:` taps removed |
| executor -> bridge | `[HEADING:START:<err cdeg>]`, `[TURN:START]`/`[TURN:END]` (bridge gates T), state lines also drive bias refresh; reports `[HEADING TURN ...]`, `[HEADING STRAIGHT ...]`, `[HEADING FUSE ...]`, `[HEADING LOST ...]` |

Files: `RasPi/final_w_serial.py`, `RasPi/hallway_geometry.py` (new),
`RasPi/bridge_core.py` (mailbox all-clear), `RasPi/monitor_ui.py` (footer),
`RasPi/preview_ui.py`, `BLE_Receiver-Final/BLE_Receiver-Final.ino`,
`BLE_Receiver-Final/SteeringAssist.h`, `BLE_Receiver-Final/MpuHeading.h`,
`MAIN_EXECUTOR_CODE/HeadingControl.h`, `MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.cpp`,
`MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.h`, `MAIN_EXECUTOR_CODE/pointsRun.ino`
(route start refuses without MPU; distances and turn angles unchanged), tests.

## 4. Obstacles: behaviour, distances, priority

Frame: x forward from the LiDAR, y left from the chassis centreline. Chassis
defaults (env-configurable, **unmeasured** except the width): 60 cm wide, LiDAR at
the centre of a 60 x 60 cm body.

| Priority | Condition | Behaviour |
|---|---|---|
| 1 | Operator emergency stop | Latched stop on both boards; release needs a new start (unchanged) |
| 2 | Missing/stale/invalid LiDAR, MPU or UART evidence | STOP and wait for fresh evidence. If the executor loses the heading stream while path permission continues, it brakes; 10 s without it ends the route as a fault |
| 3 | Anything in the path: front sector <= 95 cm, diagonals <= 60 cm, **or the swept corridor (body width + 10 cm each side) to 95 cm** | STOP, wait, resume after fresh confirmations. Never steer around |
| 4 | Anything beside the body (within 2 cm of its sides) or the old 22 cm side limit | STOP in both directions; never bypassed |
| 5 | During a spin: anything inside the sweep circle (half diagonal + 8 cm = 50 cm) | STOP:TURN until clear (front/back halves follow their bypass) |
| 6 | Swept corridor 95-160 cm | Caution speed (3900 instead of 5600 steps/s); full speed after 3 clear updates |
| 7 | Rear <= 35 cm | STOP (unchanged) |
| 8 | Nearest object <= 40 cm on a side, <= 60 cm ahead | No steering toward it (never an added turn the other way) |
| 9 | Walls | Centring only; walls are never obstacle clearance |

**People take priority by design**: a 2D LiDAR cannot tell a person from a bin, so
every return in the path is treated as one — stop and wait, never steer around.

Per obstacle type (owner's list):

* **People**: any leg/body in the corridor stops at <= 95 cm from the LiDAR (about
  65 cm in front of the body), slows from 160 cm. The robot waits; motion resumes
  only after fresh clear scans on both boards.
* **Open door (room beyond, 340-360 cm)**: not an obstacle. The fit treats the gap
  as outliers; with the other wall the robot stays centred.
* **Door leaf swung into the hallway (110/100 cm diagonal readings)**: a leaf that
  intrudes into the corridor stops the robot (wait, re-check). A leaf outside it is
  passed; nearest clearance vetoes steering toward it; the wall fit rejects it.
* **Chairs beside rooms**: legs in the corridor stop the robot. A low seat or
  table edge below or above the scan plane is **not seen by the LiDAR**.
* **Trash bins**: as chairs. A bin lower than the scan plane is not seen by the LiDAR.

What covers the 2D-plane misses: only the executor's centred front ultrasonic
(slows at 90 cm, stops at 60 cm, emergency at 25 cm, needs 3 clear echoes). It
covers a narrow forward cone only, nothing at the sides, and nothing while the
operator has turned it off. Its mounting height is unknown — check that it is below
the LiDAR plane (acceptance step A7).

## 5. Calibration procedure

Run the Pi with a saved log every time:
`bash RasPi/run_bridge.sh 2>&1 | tee ~/garby-$(date +%F-%H%M).log`

1. **Gyro bias** (automatic). Power on with the chassis still for 3 s. The Pi log
   shows `[BRIDGE MPU VALID bias=+0.xxxdps tilt=x.xdeg refresh=N epoch=E]` about 2 s
   after the DIAG line. Expect |bias| < 3 deg/s and tilt < 5 degrees (more tilt is
   compensated but suggests a loose mount). Leave it parked 2 minutes: `refresh`
   must increase and bias change by < 0.05 deg/s. Send these lines.
2. **Gyro scale** (`GYRO_SCALE` in `MpuHeading.h`). In a hallway with both walls,
   run a few turns (both directions). Each prints
   `[HEADING TURN cmd=+90.0 gyro=+G err=... align=CORRECTED wall_before=+H ...]`.
   Average `H / G` over at least 6 turns. If left and right turns agree in sign and
   |mean| > 0.005: `GYRO_SCALE_new = GYRO_SCALE_old x (1 + mean)`. If they disagree,
   it is not scale — do not change it; send the lines. Assumes hallways meet at
   right angles.
3. **Steps per degree** (`DEFAULT_STEPS_PER_DEG` in `HeadingControl.h`). Learned on
   every clean turn and printed as `spd=`. After 6 turns copy the settled value in.
4. **Steps per cm** (`STEPS_PER_REV`, `WHEEL_DIAMETER_MM` in `MAIN_EXECUTOR_CODE.h`).
   Measure the distance of the first 8000-step leg with a tape:
   `steps_per_cm = 8000 / cm`. Fix `STEPS_PER_REV` (likely 1600 vs 3200) or the
   wheel diameter so `STEPS_PER_REV / (pi x diameter_cm)` matches. Used only for
   lateral dead reckoning between scans and through gaps.
5. **LiDAR mount yaw** (`GARBY_LIDAR_YAW_OFFSET_DEG`). Put the chassis parallel to
   a straight wall (tape: front and rear corners equal distance). The GUI footer
   shows `heading +x.x deg`. Change the offset by 1 degree, restart, see which way
   the reading moves, then set it so the heading reads 0.0 +/- 0.3.
6. **LiDAR lateral offset** (`GARBY_LIDAR_Y_OFFSET_CM`, + = LiDAR left of centre).
   Centre the chassis between two parallel walls with a tape. Set the variable to
   the footer's `offset` reading (from 0), restart, confirm it reads 0 +/- 1.
7. **Chassis outline** (`GARBY_CHASSIS_WIDTH_CM`, `_FRONT_CM`, `_REAR_CM`,
   `GARBY_ROTATION_X_CM` = axle midpoint ahead of the LiDAR). Measure from the
   LiDAR centre and set in `garby-bridge.env`.
8. **Hallway table check** with the robot centred by tape, in the GUI footer:
   walls 95/95 (offset 0); beside posts still 95/95 (posts are not walls); at an
   open door the far wall is still 95; a door leaf in the corridor shows
   `Zones: front box`. Send a photo or the numbers.

## 6. Bench and floor procedure (send back the saved log)

Wheels lifted, normal stop controls at hand, `tee` running.

* **A1 boot**: USB console of each board shows `[EXECUTOR BUILD:NAV-20261111 ...
  TURNS=MPU]` and `[BRIDGE] Firmware NAV-20261111`; Pi log shows
  `[BRIDGE BUILD:NAV-20261111]`, DIAG, then `[BRIDGE MPU VALID ...]`.
* **A2 yaw sign**: on the bridge's USB console (115200) type `H`, rotate the robot
  by hand about 90 degrees to the LEFT, type `H` again: `heading=` must have risen by
  about 90 (left = +). If it fell, stop: set `YAW_POSITIVE_IS_LEFT=false` in
  `SteeringAssist.h`. Send both lines.
* **A3 LiDAR orientation**: an object at real front/left/right/rear appears in the
  matching place in the GUI map and summaries; wall offset sign: robot moved to the
  left reads offset +.
* **A4 zones in an open hallway**: footer shows no zones. If `left band`,
  `right band` or `turn` zones persist with nothing there, they are the robot's
  own structure: set the chassis variables (calibration 7). A live bench probe on
  2026-11-11 saw returns 20-36 cm in every direction, so check this first.
* **A5 heading loss**: during a lifted-wheel straight, unplug the MPU's SDA wire:
  the wheels stop at once and the Pi log shows `Executor motion: MPU not ready`;
  the robot waits stopped. Reconnect: `[BRIDGE MPU VALID ...]` with a new epoch and
  the straight resumes after fresh confirmations.
* **A6 first leg**: Start with the wheels lifted: calibration bumps and the first
  8000-step leg must turn both wheels FORWARD. The first turn then cannot complete:
  a lifted chassis does not rotate, so the gyro sees nothing and the turn stops with
  `TURN step limit` and a route fault. That is the correct protection. Turns,
  wall alignment and the motor-side self-check need the floor (F1).
* **A7 sonar height**: confirm the front ultrasonic sits below the LiDAR plane and
  sees a bin-height object at 50 cm.
* **F1 floor, supervised, short**: one straight of the hallway, then the full route.
  If the first spin starts the wrong way it backs off about 1 degree, swaps and
  reports `first spin went the wrong way; set MOTOR1_IS_LEFT_GUESS = ...` on USB.
  Send the whole log; the useful lines are `[HEADING FUSE ...]` every 3 s
  (lidar=1, y = cm left of centre, hdg = deg vs hallway, ctr = centring command,
  w = width, drift), `[HEADING STRAIGHT ...]` and `[HEADING TURN ...]`.
* **F2 obstacles**: a person stepping into the corridor at 1.5 m (slow, then stop at
  about 95 cm), a chair beside the path (passes, no steering into it), a bin in the
  path (stops), an open door and a swung door (as in section 4).

Acceptance targets (proposed; the "To fill in" values were not supplied):
turns within +/-1 degree by gyro and +/-1.5 degrees by walls after alignment;
straights within +/-10 cm of centre where walls are visible; no contact; every stop
recovers only after fresh clearance.

## 7. Assumptions

1. Hallway table figures are distances from the chassis centreline when centred and
   parallel; "door swung open" readings are the 45-degree diagonals with a leaf
   protruding (about 17 cm at 110 cm).
2. Chassis 60 cm wide (owner); LiDAR at the centre of a 60 x 60 cm body; rotation
   centre under the LiDAR; LiDAR on the centreline. All configurable, all unmeasured.
3. MPU mounted +Z up (calibration still rejects otherwise); positive yaw = left.
4. Hallways meet at right angles (used by the alignment check and calibration 2).
5. The owner's Nov 11 decisions stand: speeds 5600/4800, `pointsRun.ino`
   distances, MPU turn angles 90/90/90/90/180.
6. Acceptance targets above, since "To fill in" was missing.
7. Wheel side: a guess (motor 1 = left) checked and corrected by the first MPU turn.

## 8. Remaining risks

* Nothing here is physically proven: gyro sign, LiDAR mount yaw/offset, motor side
  and the real hallway are untested with this firmware. Bench steps A1-A7 first.
* Self-returns from the robot's own structure would hold it stopped through the new
  side band or sweep (A4).
* 2D LiDAR misses objects above/below its plane; dark or absorbing surfaces may give
  no return; the sonar covers only a narrow forward cone.
* Stopping distance is estimated, not measured: about 0.4-0.5 s detection latency
  plus braking at up to 71 cm/s gives 35-50 cm; the 95 cm limit leaves about 15 cm
  margin in front of a 30 cm nose. Measure it (F2).
* Steps are not encoders: slip corrupts lateral dead reckoning through long gaps
  (bounded to 8 s) and the step-limit check in turns.
* Wall fits assume straight walls and right-angle junctions; a long cart or a
  diagonal wall could be fitted as a wall (heading gate 4 degrees, route gate 15).
* Scan age is measured from the scan stamp (start of rotation); up to about 90 ms
  of skew, < 0.2 degrees at straight-line rates.
* The interim executor sources from earlier on 2026-11-11 and the pre-change bridge
  sketch were not snapshotted (only hashes in
  `artifacts/mpu-map-integration-2026-11-11/before/sha256.txt`).
* A route refused for no MPU latches a route fault (cleared by `[RESET]`/IDLE),
  which also queues the route-fault SMS.
