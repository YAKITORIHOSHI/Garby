# Likely people, hallway objects, station steps, reverse run, straighter runs — 2026-11-14

The owner's requests 1, 2, 3, 4 and 8 of 2026-11-14 (the full list and the remaining work are in
[`work-plan-2026-11-14.md`](work-plan-2026-11-14.md)):

> (1) ... a human detection that predicts if it was a moving object (don't ... detect a fire
> extinguisher a human just because it was moving on the map while you were running ...). Mark
> them RED dot if you think they were humans.
> (2) Don't be bothered with walls during the first and parking, garby's station can be found at
> the side of hallways which was at the walls.
> (3) Add a reverse run ... let garby go back to point A on itself to start over.
> (4) ... can we use the data for front and rear reading to help the system to walk straight?
> (8) When garby is centered ... could you automatically detect if there was an object from
> there? or even walking person and such?

Build ID for both controllers: `REVERSE-20261114`. **Source implemented, software tested and
both targets compiled here. Not uploaded, not physically accepted.** It includes
`CONTINUE-20261114`, which is itself not yet uploaded.

Deploy together: both complete sketches and the entire `RasPi/` directory. The operator status
line gains `:REV=`, which an older Pi does not parse (its controls would read as unsupported;
emergency stop still works). The new Pi accepts the older lines.

Contents: [red dots and objects](#1-and-8--likely-people-and-objects-on-the-map) ·
[station steps](#2--station-steps) · [reverse to start](#3--reverse-to-start) ·
[straighter runs](#4--straighter-runs) · [what changed where](#what-changed-where) ·
[tests](#tests-run-here) · [bench and floor steps](#bench-and-floor-steps) ·
[limits](#limits-and-assumptions)

## 1 and 8 — likely people and objects on the map

The radar map of the monitor now marks:

| Marker | Meaning |
| --- | --- |
| **red dot** with a line | likely person: person-sized (15–75 cm across), not a flat panel, and proven to move over the ground at 0.25–4.5 m/s for at least 40 cm over 0.9 s. The line shows where it will be in one second |
| amber dot with a line | moving object that is not person-shaped (a cart, a carried board, a door leaf) |
| orange square | object standing between the two fitted walls of the hallway, moving or not |

The line under the map counts them ("1 likely person · 2 objects"), and the log gets one line
per change (`Hallway view: 1 likely person(s), 1 object(s) between the walls; ...`).

**Why a still object no longer looks like a mover while GARBY drives.** The tracker already
removed GARBY's forward speed. It did not remove GARBY's own rotation: a heading correction of
4 deg/s slides a still object 5 m away sideways at 35 cm/s, the tracker's own threshold. In
the hallway model that produced 6 false movers in a minute at 4 deg/s and 90 at 8 deg/s
(`artifacts/work-2026-11-14/proof_rotation.txt`). Now the wall fit's heading of every scan
turns the tracks back by GARBY's rotation, positions are kept in a ground frame, and without
walls any sideways motion that up to 8 deg/s of rotation could explain is not counted. Motion
along the line of sight is never explained by rotation, so someone walking at GARBY is found as
fast as before (the model: stop 1.0 s after first sight, 4.8 m away, unchanged).

**A red dot stays red** while that object stays in view, also when the person stops walking.
A person who has not moved since coming into view is an orange square: a 2D scan at chest
height cannot tell a standing person from a post.

**Objects between the walls (request 8).** With both walls fitted 120–450 cm apart, every
return is wall (within 6 cm of a wall line), beyond the wall (through a door) or inside the
hallway. Inside returns are grouped into objects with distance ahead, width and free width on
each side. A thing touching the wall and reaching in no more than 45 cm is a wall fixture
(post, extinguisher) and is not marked; one spanning 70 % of the hallway is its end wall.

**Motion.** Likely-person is a label for the map and the log. Every stop and slow rule still
treats all objects alike. One rule is added: an object standing in GARBY's own path between
the walls, beyond the 160 cm slow zone and within 4 m, asks for caution speed (`F=W`,
zone `OBJECT_AHEAD`) once three scans in a row show it. It never stops the robot, and the
read-only front bypass removes it like the slow zone.

## 2 — station steps

The station is at the hallway wall. The 2026-11-14 live run was held there by the wall behind
the robot and a wall step beside it. On the way home, the last straight drives at that wall:
the front box and the sonar hold it long before it is parked.

- The route table marks the steps driven at the station: the outbound route's bumps and first
  straight (leaving), and the return route's last bumps, straight and 180 degree turn (parking).
  The outbound route's first turn is at the hallway centre and stays an ordinary turn.
- The executor names the step in progress: `[LEG:<O|R><F|B>:<step>/<count>:<op>:<flags>]`, sent
  when the step begins and every 2 s, `[LEG:0]` when the route ends. The Pi shows it under the
  state ("Outbound step 2 of 15: straight · station step, station walls ignored") and logs
  each step once (`Route step: ...`), so a floor log now shows where the robot was.
- **LiDAR.** While a station step's notice is fresh (3.5 s), the link is up and a route is
  running, returns that belong to a straight piece at least 60 cm long are the station's walls
  and are treated as "nothing there" by every LiDAR hold. So are short, truly straight pieces
  joined to such a wall near the robot (the recorded station has a 13 cm step and a 35 cm
  return right beside the body) and the few returns of the corner between two wall pieces.
  A person, a bin or a box standing free still holds the robot. A person against the wall
  still holds it: a body's outline is curved.
- **Sonar.** On the steps that face the station wall its echo is expected. With the parked
  distance measured (`STATION_PARK_GAP_CM` in `SonarSweep.h`), only an echo more than 15 cm
  nearer than the wall can be holds the robot. It is **not measured**, so today the forward
  sonar does not hold those steps, and the turn beside the wall skips its sonar survey.
- **Turn.** The parking turn skips the wall alignment (`align=SKIP`): beside the station no wall
  is parallel to the robot.
- When the notice stops arriving (end of the step, controller restart, link loss) the ordinary
  rules are back within 3.5 s.

On the recorded station scans (`artifacts/manual-disposal-hold-2026-11-14/scans-base.npz`) all
15 are held by a chassis zone under the ordinary rules and none under the station rules.

## 3 — Reverse to start

For calibration runs that stop short. After an emergency stop, the *Controls* tab has a third
choice beside **Release stop** and **Continue run**: **Reverse to start**, with a confirmation.

GARBY turns around on the spot, drives **forward** back along exactly the wheel travel it
covered, mirrors each turn, and at the start turns to face the way the route began. It ends at
IDLE (after an outbound route) or parked at the destination (after a return route), and the
next run needs a deliberate start, as after Release.

- Every motion gate of a route applies, and it drives forward, so the sonar, the front box and
  the prediction face the way it goes. "Every route segment drives forward" still holds.
- Headings are absolute: each leg is driven at its outbound heading plus 180 degrees. A stop
  in the middle of a turn needs no special case.
- An emergency stop on the way back can be continued or reversed again (both carry on back to
  the start), or released.
- It is offered only while the stop is latched and confirmed, the executor is at IDLE and
  reports `REV=1`. The record is in RAM: a controller restart removes the option.

Host model (wheels never slip): back at the start within 0.3 cm and 0.4 degrees from the first
straight, 0.8 cm from 3.9 m down the hallway, and 2.3 cm after the whole outbound route.
**A real robot repeats its slip, so its miss must be measured on the floor.**

## 4 — straighter runs

"Centred but tilted" means the heading GARBY holds is not the hallway's, so it drifts until the
5 cm centring threshold pulls it back, again and again. Wall data already reaches the executor
with the front and back checks on read-only; that was not the gap.

- **Far wall fit.** A second line fit over the wall points 3 m behind to 5 m ahead supplies the
  heading when it sees at least 3.5 m of clean wall that agrees with the near fit (1.5 degrees,
  4 cm). In the hallway model its heading noise is a fifth of the near fit's. The wall
  *distances* still come from the near window. `GARBY_WALL_FAR_FIT=0` turns it off.
- **LiDAR mounting angle, learned from drift.** If the LiDAR sits a degree or two rotated on
  the chassis (it was never measured), every heading carries that error, and the robot drives
  the sawtooth described above. While GARBY drives, its sideways drift per distance is the sine
  of its *true* heading, so `mean measured heading − true heading` is the mounting error. The
  Pi learns it over 3 m stretches with both walls in view, at most 0.1 degree per stretch and
  ±3 degrees in all, takes it out of the `H` it sends, saves it
  (`~/.local/state/garby/wall-heading-offset.json`) and logs
  `Wall heading offset learned: +1.3 deg ... Equivalent setting: GARBY_LIDAR_YAW_OFFSET_DEG=...`.
  `GARBY_WALL_HEADING_LEARN=0` turns it off.

In the model the near fit's noise is already small (0.04–0.12 degrees), so noise is probably
not the cause of the tilt and the mounting angle is the likelier one. No executor change was
made. A floor log decides (step 6 below).

## What changed where

| Area | Files |
| --- | --- |
| Pi, new | `RasPi/scan_frame.py`, `scene_objects.py`, `station_mode.py`, `wall_heading_offset.py` and their `test_*.py` |
| Pi, changed | `final_w_serial.py`, `motion_tracker.py`, `hallway_geometry.py`, `lidar_map.py`, `monitor_ui.py`, `operator_support.py`, `garby-bridge.env.example` and their tests |
| Executor | `MAIN_EXECUTOR_CODE/RouteResume.h`, `pointsRun.ino`, `MAIN_EXECUTOR_CODE.cpp`, `.h`, `.ino`, `SonarSweep.h` |
| Bridge | `BLE_Receiver-Final/BLE_Receiver-Final.ino` (accepts and correlates `REVERSE`, knows the `REVERSING` state) |
| Firmware host tests | `tools/firmware_tests/test_route_reverse.cpp` (new), `test_bridge.cpp`, `test_continue_run.cpp` |

Wire additions: `[OP:<id>:REVERSE]`; `:REV=<0|1>` on `[OP STATUS:1:...]`; executor notices
`[REVERSING]`, `[REVERSE COMPLETE]`, `[ROUTE REVERSE:<step>/<count>]`, `[LEG:...]`. The `S:`
and `P:` packets are unchanged in form; `H` is now the corrected heading.

The calibrated route numbers are unchanged: the same steps and angles in the same order, with
a flag added to five steps.

## Tests run here

| Check | Result |
| --- | --- |
| `python3 -B -m unittest discover -s RasPi -p 'test_*.py'` (no display) | see `artifacts/work-2026-11-14/pi-suite-phase3.txt` |
| `test_monitor_ui` on a private Xvfb display | see `artifacts/work-2026-11-14/pi-monitor-xvfb.txt` |
| `python3 -B tools/run_firmware_tests.py` | see `artifacts/work-2026-11-14/firmware-tests.txt` |
| `arduino-cli compile` executor (ESP32, 4M) | exit 0, 422,002 bytes (`compile-executor.txt`) |
| `arduino-cli compile` bridge (ESP32-S3, 8M) | exit 0, 590,108 bytes (`compile-bridge.txt`) |

Proofs that a test fails on the old code: the rotation test on the pre-change tracker (6 and
90 false movers); the recorded bench clutter before the two-wall rule (202 of 440 scans asked
for caution); the recorded station scans under the ordinary rules (15 of 15 held).

## Bench and floor steps

Always save the log: `bash RasPi/run_bridge.sh 2>&1 | tee ~/garby-floor.log`.

1. **Wheels lifted, after flashing both boards.** The Pi log shows `[BRIDGE BUILD:REVERSE-20261114]`.
   Start a route. Expect `Route step: Outbound step 1 of 15: calibration bumps · station step,
   station walls ignored` and `Station step: the station's walls are left out of the LiDAR
   holds`. Send back those lines.
2. **Wheels lifted, reverse.** Press EMERGENCY STOP during the first straight. **Reverse to
   start** becomes available. Press it and confirm. Expect `[ROUTE REVERSE:1/15]`, a 180 degree
   turn, the wheels running forward for about the same time as before the stop, a second
   180 degree turn, `[REVERSE COMPLETE]`, state Idle. Send back the lines between
   `Operator action requested: REVERSE` and `Executor state: IDLE`.
3. **Floor, leaving and parking with the checks ON.** Front and back checks enabled, sonar on.
   Start from the station; request the return; let it park. Send back every
   `Executor motion:` line and the `Route step:` line before it. If it holds on a station
   step, the `LiDAR chassis zones:` line says what held it.
4. **Floor, reverse.** Mark the wheel positions at the station with tape. Run to mid-hallway,
   stop, Reverse to start. Measure the miss at the tape in centimetres and the angle. That
   number is the answer to "how exact is it".
5. **Floor, people.** One helper walks toward GARBY, across the hallway ahead of it, away from
   it, then stands still. Watch the map: red dot within about a second of walking, stays red
   standing. Then run past the fire extinguisher and other still things with nobody moving:
   **no red or amber dot may appear**. Send back the `Hallway view:` and
   `Moving-object prediction:` lines, and a phone video of the map if possible.
6. **Floor, straightness.** One long hallway run. Send back the `[HEADING STRAIGHT ...]` lines,
   any `Wall heading offset learned:` lines and whether the tilt is visible. To compare, run
   once with `GARBY_WALL_HEADING_LEARN=0 GARBY_WALL_FAR_FIT=0` in the environment.

## Limits and assumptions

- **People.** "Likely person" is a guess from size and movement. A child below the scan plane
  is not seen by the LiDAR at all; a person who stands still from the start is an object, not
  a red dot; two people side by side wider than 75 cm are a moving object. No claim is made
  about preventing injury.
- **Station steps** ignore long flat surfaces at the station, whatever they are: a door leaf, a
  board, the side of a cart, a flat box pushed against the wall. With the parked sonar distance
  unmeasured, the forward sonar does not cover the last 96 cm of parking either. On those steps
  the robot relies on its calibrated step counts, as it does today with the checks on read-only.
  Measure and set `STATION_PARK_GAP_CM` to get the sonar back there.
- **Chassis outline** is still the unmeasured 60 x 60 cm default (`GARBY_CHASSIS_*`). The
  recorded station has a wall step 0.2 cm from that outline's side, so the real body is
  narrower or the LiDAR is off-centre. `GARBY_STATION_WALL_MIN_GAP_CM` stays 0 (off) until the
  outline is measured.
- **Reverse** does not undo the forward travel of pre-run centring sidesteps when the stop came
  during the centring itself (some tens of centimetres at most), and it returns only as exactly
  as the wheels repeat their steps.
- **Mounting-angle learning** assumes the two wall distances are measured where the robot is
  and that the reported wheel speed is roughly right. It is bounded and slow by design; its
  value is in the log and can be checked against a tape measurement.
- **Models.** The hallway and kinematic models show the bookkeeping and the gates. They cannot
  show real people, real walls, slip, stopping distance or straightness.
