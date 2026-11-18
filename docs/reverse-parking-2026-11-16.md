# Reverse parking, likely-person dots while moving, turn noise — 2026-11-16

The owner's requests of 2026-11-16:

> Add backwards movement for parking purposes
> (asked where: "Back into station + Reverse parking for both 'Point A to Point B' and
> 'Point B to Point A'")
>
> The human detection were working as intended yet needs optimization ... It detects the
> persons around yet it detects objects sometime as humans sometimes during the run, maybe
> because garby itself were running?
>
> Turn Left/Right still produces weird noice but it do the work just fine.

Build ID for all three parts: `PARK-20261116` (executor, bridge and Pi software).
**Source implemented, software tested and both targets compiled here. Not uploaded, not
physically accepted.** Upload requirement: **Both** MCUs, plus the whole `RasPi/` folder.
Deploy all three together; a mixed set holds the robot at the parking step (section 1.4).

## 1 — Reverse parking

### 1.1 What changed

Each route used to end by turning toward its parking spot, driving at it and spinning
180 degrees beside it. Now its last turn goes the other way, with the robot's back to the
spot, at the same place, and it reverses in by the same travel. Right 90 then left 180 is
left 90, so it ends in the same place facing the same way, and there is no spin beside the
wall.

| Route end | Before | Now |
| --- | --- | --- |
| Outbound (destination) | right 90, 500 + 10600 forward, left 180 | left 90, **11100 back** |
| Return (station) | right 90, 500 + 7000 forward (station steps), left 180 beside the wall | left 90, **7500 back** (station step) |

The calibrated step counts are unchanged: the backing step is the old approach's bumps plus
straight. Both routes now have 13 steps instead of 15; the backing step is step 13 of 13.
The final turn at the hallway centre is an ordinary turn and is aligned to the walls.

The backing move (`backIntoPlace`, `MAIN_EXECUTOR_CODE.cpp`): both wheels the same steps
backwards at `BACKING_SPEED` = 2000 steps/s (about 25 cm/s with the calculated 78.4 steps/cm),
one plain ramp, no steering. Every STOP holds it, a lost MPU stream brakes it, the operator
stop ends it, and after a hold it resumes toward the same wheel targets. Afterwards
`[HEADING BACK steps=.. err=.. mpu=..]` reports the gyro error, and an MPU-measured spin
squares the robot up if it is more than 1 degree off (no wall alignment there).

### 1.2 What watches behind the robot

Only the LiDAR. The sonar faces forward, away from the travel, so it neither holds nor
slows a backing move and its servo stays forward. **Nothing below the LiDAR's plane behind
the robot is seen** (a bag on the floor, a low box, a small child): keep both parking spots
clear of low objects.

The Pi sends a rear-travel code with every path packet, `R=` (`RasPi/final_w_serial.py`,
`_rear_travel_status`), whatever the robot is doing:

| `R=` | when |
| --- | --- |
| `S` | no current scan or no back data |
| `O` | side clearance blocked; a return within 2 cm beside the rear half (rear edge to spin centre, never bypassed); a return in the corridor-wide box up to 35 cm behind the rear edge (`backing_stop_cm`); a moving object predicted to reach the body (while backing, one catching up from behind counts) |
| `C` | otherwise |

A read-only back group bypasses the box and the mover, as for every other back-group
limit; the rear bands and side clearance stay. Returns inside the body outline are the robot
itself. The 35 cm box is sized for 25 cm/s and about 0.6 s of scan, link and braking delay,
with a factor of two; the chassis outline (`GARBY_CHASSIS_*`) is still unmeasured.

The return route's backing step is a station step, so the station's walls (the wall behind
the robot, its steps and corners) are taken out of the scan as before, and the calibrated step
count decides where it stops, as the forward approach did. A person, bin or box standing
free behind it still holds it. The destination has no station flag: its ordinary rules apply,
including the 35 cm box.

### 1.3 Bridge and executor

Between `[BACK:START]` and `[BACK:END]` (executor, `BackingSegmentNotice`) the bridge
grants motion only on `R=C` with `B=C`, and sends it as `GO:BACK`. `F` is not consulted
(it faces away from the travel, as the back group is not consulted for a forward move).
`R=O` gives `STOP:BACK`; a missing or unknown `R` gives `STOP:BACK_UNKNOWN`. The executor
backs only on `GO:BACK` and never takes `GO:BACK` as forward permission. The scope is
replayed in every status report, like the turn scope, and is unknown after a bridge reboot
until the executor replays it. No steering permission is sent while backing.

### 1.4 Mixed versions hold, never back blind

| Executor | Bridge | Pi | Result at the parking step |
| --- | --- | --- | --- |
| PARK | PARK | PARK | backs in under `R=` |
| PARK | older | any | holds: "Bridge has not confirmed rear clearance" (no `GO:BACK`) |
| older | PARK | any | no GO at all: the bridge starts with the backing scope unknown and the older executor never reports it |
| PARK | PARK | older | holds: "Rear clearance unknown" (no `R=`) |

The Pi simulator in `tests/pi` sends neither `T=` nor `R=`, so turns and backing hold with it.

### 1.5 Continue run and Reverse to start

Continue after an emergency stop while backing resumes the backing step and backs only what
is left (the leg counts falling step counts; model: stopped 4002 steps in, parked exactly
11100 steps from where backing began). Reverse to start still only drives forward: it undoes
a backing step by driving forward, facing the way it faced (no turn round).

## 2 — Likely-person dots on still objects while GARBY moves

**Cause.** Ground velocity is the scan motion minus the wheel speed the executor reports. When
the wheels turn but the robot does not cover the ground at that speed (wheels lifted on the
bench, slip, a lagging report), every still object drifts along the travel axis by the
difference, and person-sized round posts and bins passed the person test. Your last session's
status file had both Front and Back read-only on, which fits a bench run with lifted wheels.

Ray-cast hallway model, the real tracker, 4 runs each, still objects only:

| Ground speed / reported speed | 0 (lifted) | 0.5 | 0.7–1.25 (slip, scale error), braking/start lag, heading swings |
| --- | ---: | ---: | ---: |
| False likely persons | 2 | 0 | 0 |

With the robot standing still but reporting 71 cm/s next to the model's posts and bins,
2–4 were labelled within about 1 s in all 15 positions tried.

**Fix** (`RasPi/motion_tracker.py`, `_scene_drift`). When at least three confirmed, measured
tracks mostly agree on one velocity along the travel axis that a wheel-speed error can
explain (between "not moving at all" and "as fast as reported"), the label also needs motion
relative to them (walking speed and 40 cm of travel). The last drift stands in for up to
1.2 s when a scan has too few tracks or noisy ones. Results: 0 false labels in all 15 lifted
positions and all 36 comparison runs (walkers head-on, overtaking and crossing; lifted, driving
and stopped); in every driving and stopped run the walker's label time is the same as before.

**Limits.** Display and report only: moving, stop and slow decisions do not use the drift.
With the wheels lifted, a walker coming head-on may be labelled later or not at all (the wrong
ground speed hides half its speed even from the moving rule). Two or more people walking
together at about GARBY's own speed and direction may lose their dots. The LiDAR still cannot
recognise people; the dot is a size-and-motion guess.

## 3 — Turn noise

The fix for the loud stop-start developer turns is executor `DEVTEST-20261116`. The MCU
Flasher log shows two **Build Failed** results for `MAIN_EXECUTOR_CODE` on 2026-11-15 at
23:51 and 23:52 EST (the flasher prints 12:51/12:52 in its own clock), and no upload after
them. The robot is therefore most likely still on `DEVTEST-20261115`, which has the noise.
The same source compiles here without warnings, and `PARK-20261116` includes that fix. If the
flasher fails again, send its full error text. If the noise remains on `PARK-20261116`, send
a log of one Developer-tests turn (`[TEST RESULT..]` and the `Executor motion:` lines).

## 4 — Steps on your side

1. **Flash both boards** with `PARK-20261116`: the bridge (ESP32-S3 N16R8,
   `FlashSize=16M,PSRAM=opi`) and the executor (ESP32, 4 MB). Whole sketch folders.
2. **Deploy the whole `RasPi/` folder** and restart the bridge program. The monitor's title
   and log show `PARK-20261116`.
3. **Bench, wheels lifted.** Run a Developer-tests *Forward ~5 cm* and *Turn left 10°*: the
   turn should be quiet now. Routes cannot be bench-tested past their first turn with the
   wheels lifted (the gyro sees no rotation), so the backing step is first seen on the floor.
4. **Bench, red dots.** Lifted wheels, front/back read-only as before, start a route: still
   objects should stay amber/grey, not red. Walk past slowly: you should get a red dot.
5. **Floor, supervised.** Keep both parking spots clear of low objects. Run the full route
   with the checks on, stand behind the robot's path near the destination, and watch the
   last turn (it turns the other way now) and the slow reverse in. Step into the path behind
   it once: it must stop and continue when you leave. Request the return and watch it back
   into the station.
6. **Send back** the whole log (`bash RasPi/run_bridge.sh 2>&1 | tee ~/garby-$(date +%F-%H%M).log`),
   especially the `Route step: ... backing into place` lines, `Executor motion:` holds,
   `[HEADING BACK ...]`, `[HEADING TURN ...]` and where it stopped against each spot (cm from
   the wall), so the parked gap and `STATION_PARK_GAP_CM` can be measured.

## 5 — Assumptions and limits

- Backing has no steering. Equal wheel steps backwards drive as straight as forwards; drift
  over 1.4 m is squared up afterwards by the gyro, but the lateral offset is not corrected.
- The calculated 78.4 steps/cm is not measured. Speeds and the 35 cm box depend on it.
- Stopping distance while backing is not measured. The model shows the gates hold; it cannot
  show braking, slip or what the LiDAR misses behind the robot.
- The chassis outline is the unmeasured 60 x 60 cm default. A real body longer than 30 cm
  behind the LiDAR would put its own frame in the rear box and hold every backing move
  (named "Rear travel clearance blocked (REAR_BOX)"); measure `GARBY_CHASSIS_REAR_CM` then.

## Files

Executor: `MAIN_EXECUTOR_CODE/RouteResume.h`, `pointsRun.ino`, `MAIN_EXECUTOR_CODE.cpp`,
`MAIN_EXECUTOR_CODE.h`, `NOTE.txt`. Bridge: `BLE_Receiver-Final/BLE_Receiver-Final.ino`,
`NOTE.txt`. Pi: `RasPi/final_w_serial.py`, `hallway_geometry.py`, `station_mode.py`,
`motion_tracker.py`, `bridge_core.py`. Tests: `tests/pi/test_reverse_parking.py` (new),
`tests/pi/test_motion_tracker.py`, `test_runtime.py`, `test_forward_only_motion.py`,
`test_lidar_offline.py`, `tests/firmware/test_bridge.cpp`, `test_system.cpp`,
`test_continue_run.cpp`, `test_route_reverse.cpp`, `hallway_sim.h` and the sim wiring in
`test_navigation.cpp`, `test_motor_direction.cpp`. Verification:
`artifacts/reverse-parking-2026-11-16/VERIFICATION.md`.
