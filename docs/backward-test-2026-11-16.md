# Backward on the bench, review of the rear-wall parking — 2026-11-16

## 1 — What was reported and what the evidence shows

The owner reported "it seems that backward was not working at all" and said the test was on
the **bench with the wheels lifted**. No log of that session was saved (the bridge ran by hand,
07:02-07:03 EST, and before that 04:16-04:19 EST; only `~/.ros/log` lines remain).

The flasher's own notifications (stamped in the owner's UTC+8 clock) show both boards were
built and uploaded from the current source just before those runs:

| Board | Build | Upload (EST) |
|---|---|---|
| Bridge, `BLE_Receiver-Final` | succeeded, 556 s | 03:54 |
| Executor, `MAIN_EXECUTOR_CODE` | succeeded, 362 s | 04:12 |

The source then was Codex's `REARPARK-20261116` (the tree still matched its recorded hashes),
so both boards most likely run `REARPARK-20261116`. A saved build is not proof of what is
installed, but nothing points to another image.

**Cause.** On the bench, no control could turn the wheels backward:

- The Developer tests had Forward, Motor 1, Motor 2 and the two turns, but nothing backward.
- The only backward travel in the firmware is each route's last step (`RouteOp::BACKING`). A
  route with the wheels lifted ends at its first turn (the gyro sees no rotation:
  `NO_ROTATION`), many steps before parking. "Reverse to start" turns round and drives forward
  by design.

So the backing code was never reached. This is not a fault in the rear-clearance gate: in the
host models the parking step backs when `R=C`, and holds when `R=O` or when `R` is missing.

## 2 — Change: a Backward developer test

| Button | Moves | Limit |
|---|---|---|
| Backward ~5 cm | both wheels backward | 400 steps, 400 steps/s, 8 s, live KEEP lease (as Forward) |

- The executor runs it inside the same rear scope as the parking step (`[BACK:START]` ...
  `[BACK:END]`). With **LiDAR ticked** the bridge sends `GO:BACK` only on the Pi's rear-travel
  clearance (`R=C`, with `B=C`). A plain forward GO never moves it. If something behind holds it,
  the result says so (`HELD_REAR`: "held by the LiDAR rear check: something is behind GARBY").
- The forward sonar faces away from this travel and does not hold it, as in parking.
- With **LiDAR unticked** only the lease counts (as for every test). A GO of the wrong kind is
  now ignored like a STOP, so it can no longer turn into a hold.
- With the **gyro ticked** the move is under live MPU supervision. Backing needs no rotation,
  so it works with the wheels lifted.
- It does **no rear-wall alignment**: that needs spins, which the gyro cannot see with the
  wheels up. That part can only be checked on the floor.
- The Live line on the Developer tab now also shows the Pi's rear-wall fit, the one parking
  squares up to: `rear wall 152 cm, angle -0.84°`, or `no rear wall seen`. Angle 0 means
  square to it. It is display only and at most 1 s old.

Wire format: `[TEST:<id>:B:<1..400>:<mask>]`; see `SYSTEM_ARCHITECTURE.md` "Finite developer
tests". Older boards drop a `B` request without an answer. The Pi then says the robot did not
confirm it and names the `BACKTEST-20261116` firmware.

## 3 — Bench steps (wheels lifted)

1. Upload both sketches and deploy `RasPi/` (all `BACKTEST-20261116`). Start the monitor and
   check that the Developer tab shows **Ready**.
2. **Backward ~5 cm with LiDAR unticked.** Both wheels should turn about a quarter turn the
   other way from Forward. The result should read `Motor 1 -400 steps (≈-5.1 cm), motor 2
   -400 steps`. If a wheel turns forward, stop and report it. Do not flip the DIR settings.
3. **Backward with LiDAR ticked**, nothing within about 35 cm behind the robot's rear edge.
   It should move the same way. With a hand or a box held behind the robot, it should not move.
   The message should say the rear check held it.
4. **Rear wall reading.** Put a flat wall or board (at least 80 cm wide) 0.5-2.5 m behind the
   robot. The Live line should show its distance and an angle near 0 when the robot is square
   to it. Turning the robot by hand a few degrees should change the angle by about as much,
   with the sign matching the turn (+ = robot rotated left).

Then, on the floor, the route's parking step runs the full rear-wall alignment (section 4).

## 4 — Review of Codex's rear-wall parking (REARPARK-20261116)

Reviewed: `RasPi/rear_wall.py`, the Pi's `RD/RH` fields, the bridge's D line, and the
executor's `measureRearPair`, `alignRearForParking` and `backIntoPlace`, against
`docs/verification/2026-11-16-rear-wall-parking/source.diff`. The source matches Codex's
recorded hashes. The protocol is consistent end to end:

- The Pi fits the rear wall before the station mask.
- The bridge forwards each acquisition as a D line, including an explicit "no wall" line.
- The executor drops repeated acquisitions and old scans (900 ms), and moves the measured
  normal into the gyro frame at scan time. Sign conventions match the `H` wall heading.

The sides packet with `RD/RH` stays at about 160 bytes, well under the 244-byte limit, so the
new fields cannot push steering into `STABLE`. Controls, Forward, turns and nudging are
untouched by Codex's diff.

Nothing found would stop backing on the bench. These points can hold the parking step **on the
floor** and should be checked there; they were not changed without floor evidence:

1. **No usable rear wall means no parking.** It needs a straight surface at least 80 cm wide,
   crossing the robot's rear centre line, within 3 m. Until two scans agree (within 1° and
   3 cm), it holds with "Parking: waiting for two fresh rear-wall scans", indefinitely. Use the
   new Live reading at each parking spot first.
2. **Large or unresolved misalignment holds.** If the robot is more than 8° off the rear wall,
   or still more than 1° off after two small corrections, it holds with "Parking: rear
   alignment needs checking" and does not retry by itself.
3. **The outbound spot still has the 35 cm rear box** (Codex's doc): with the assumed outline,
   a wall closer than about 65 cm behind the LiDAR at the end of the backing holds the last
   part of it. The return's station wall is exempt only through its station step.
4. **One failed rear fit while backing brakes at once.** A single missing or disagreeing scan
   stops the wheels, re-measures two scans standing and continues. Near clutter this can make
   stop-and-go.

## 5 — Limits

Host models only: no robot was moved, nothing was uploaded. Steps per cm (78.4) is calculated
and not measured, and so is the distance of 400 steps. The test cannot show braking distance,
wheel slip, or anything below the LiDAR plane behind the robot. The chassis outline (and with
it the rear box) is still the unmeasured default.

**MCU upload requirement: Both** complete sketches (`BLE_Receiver-Final`, `MAIN_EXECUTOR_CODE`)
plus the whole `RasPi/` folder, all `BACKTEST-20261116`. The bridge must accept the new `B`
request and the executor runs it. This build includes Codex's `REARPARK-20261116` unchanged.

Files: `MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.cpp`, `MAIN_EXECUTOR_CODE.h`;
`BLE_Receiver-Final/BLE_Receiver-Final.ino`; `RasPi/developer_support.py`, `monitor_ui.py`,
`final_w_serial.py`. Tests: `tests/firmware/test_developer.cpp`, `test_bridge.cpp`,
`test_system.cpp`; `tests/pi/test_developer_support.py`, `test_monitor_ui.py`,
`test_reverse_parking.py`. Evidence: `artifacts/backward-test-2026-11-16/`.
