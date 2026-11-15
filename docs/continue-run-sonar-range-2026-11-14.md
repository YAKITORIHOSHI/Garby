# Continue run, sonar range and stop-and-go — 2026-11-14

The owner asked for three things:

> - Continue Run on control if stopped.
> - Adjust the reading of sonar, it should not read too far since the first run only goes center.
> - The motors sometimes stopping and go, but it still works as intended.

Asked to choose, the owner picked: **Continue after an emergency stop** (Release stays as it
was), **a shorter sonar distance**, and **record the stop-and-go and reduce it**.

Build ID for both controllers: `CONTINUE-20261114`. Source implemented and software tested
here; **not uploaded, not physically accepted**.

## Continue run

The Pi monitor's *Controls* tab has a **Continue run** button next to **Release stop**. It is
enabled only while the emergency stop is latched and confirmed, the executor is stationary at
IDLE, and the executor reports a stopped route to continue. Pressing it asks **Continue run?**
("Only continue if GARBY has not been moved by hand"); Cancel sends nothing, and the
controller state is checked again after the dialog closes.

- **Release stop** is unchanged: the route ends at IDLE and a new route needs a fresh start.
- **Continue run** releases the stop and finishes the same route from where it stopped. The
  route still moves only after fresh GO, no STOP, sonar clearance, MPU heading and (for a
  turn) the spin-sweep check, the same gates as any route.

What the executor saves at the emergency stop (`MAIN_EXECUTOR_CODE/RouteResume.h`):

| Interrupted step | Continued as |
| --- | --- |
| Calibration bumps (500 steps in moves of 100) | the bump steps still left |
| Straight | the steps still left (mean of both wheels, as the straight measures it) |
| Long hallway | pre-run centring again, then the steps still left (centring sidesteps are not route travel); a tail under 50,000 steps (~6.4 m) skips pre-run centring and keeps the straight's own wall centring, since a sidestep pass could travel further forward than the tail |
| Turn, before its angle entered the route frame | the whole turn |
| Turn, after that | a spin to the same route target (the rest of the turn), then the usual wall alignment |
| Parked at the destination | parked again (`[OUTBOUND COMPLETE]`), no motion |
| Before the route heading was anchored | the route from its start |

To make that possible the routes in `pointsRun.ino` are step lists. The step counts, turn
angles and order are the calibrated values, unchanged; the 100 ms pauses between steps are
kept (only the return route's pause after its final turn, with nothing after it, is gone).
The host navigation model gives identical results before and after, whole outbound route
included (end heading error +0.51 deg, last hallway y +5.8 cm).

The heading frame from before the stop is kept. If the S3 bridge restarted while the robot was
stopped (a new heading reference), two agreeing wall scans must put the route direction within
8 degrees of its right-angle grid before anything moves, the same rule as a long heading
outage. Without confirmable walls it stays still and the route faults at the 5-minute heading
limit. In the host model that happens when the bridge restarts 40 degrees into a turn (no wall
line is fitted at that angle), while 12 degrees into a turn the old hallway's walls confirm it.

Limits:

- The saved point is in RAM: an executor restart or reboot loses it and Continue is unavailable.
- The executor cannot tell whether the robot was pushed or carried while stopped. The dialog
  says so; walls are re-checked only after a heading-reference change, and before a long
  hallway by its pre-run centring.
- Emergency stop at the destination followed by **Release** still leaves the executor at IDLE
  as if at base (behaviour from before this change); **Continue** keeps it parked there.
- The Android app has no Continue button (separate repository, not changed). The app's existing
  controls are unaffected.

Protocol (Pi → bridge → executor, same correlation and gates as RELEASE):

```text
[OP:<id>:CONTINUE]                       Pi -> bridge -> executor
[OP ACK:<id>:CONTINUE]                   executor: released into the route
[OP FAILED:<id>:NO_ROUTE]                executor: nothing to continue (also NOT_STOPPED, NOT_IDLE, NOT_READY)
[OP STATUS:1:ESTOP=<0|1>:SONAR=<0|1>:RESUME=<0|1>]   RESUME=1: a stopped route can continue
[RUNNING] / [RETURNING]                  after the ACK, the route state it continues in
[ROUTE CONTINUE:<step>/<steps>]          the step it continues at
[HEADING ROUTE:CONTINUE]                 heading frame kept (same S3 reference)
[HEADING CONTINUE verify=OK|WAIT_WALLS|FAIL|TIMEOUT ...]  after a new S3 reference
```

The bridge requires its acknowledged emergency stop before relaying CONTINUE (as for RELEASE),
releases its own latch only on the matching `[OP ACK:<id>:CONTINUE]` of the same stop
generation, and then holds `STOP:WAITING_DATA` until fresh path confirmations. The Pi accepts
the older status line without `RESUME` (Continue then stays disabled), so updating the Pi first
is safe. **An old Pi does not understand the new status line**: with new firmware and an old
Pi, operator status reads as unsupported (Release/Restart unavailable; emergency stop still
works). Deploy all three together.

## Sonar range

The obstacle sonar now reads only to `ULTRASONIC_MAX_RANGE_CM` = 110 cm
(`MAIN_EXECUTOR_CODE.h`). A ping gives up after that echo time plus 200 µs (6.6 ms; it was
18 ms, about 3 m), and an echo from beyond 110 cm counts as *nothing in range*, a clear
reading. Every decision distance is inside the range: stop 60, clear 72, slow 90, full-speed
resume 105 cm, emergency 25 cm, and the spin circle (about 80 cm from the sonar). A
`static_assert` keeps the range above the full-speed resume distance.

What it changes: far walls, doors and people beyond 1.1 m are no longer sonar evidence at all.
What it does not change: echoes beyond 90 cm never slowed or held the robot before either, so
this alone is not expected to remove stops. `[SONAR:SILENT]` (three full surveys without any
echo) becomes more likely where every wall is over 1.1 m away; it never holds the robot. In the
190 cm hallway the ±70 survey bearings still hear the walls at about 96 cm.

## Stop-and-go (recorded, and one cause reduced)

Owner's observation, 2026-11-14: the motors sometimes stop and go, and the run still works as
intended. No log of those runs was available, so the field cause is **not established**. What
the code does that looks like this:

1. **One forward sonar echo inside 60 cm braked the robot** (no confirmation), which then had to
   re-confirm GO before driving on. A stray echo the next ping does not repeat gave a short
   stop-and-go. **Changed:** beyond 25 cm it now takes two consecutive forward echoes inside
   60 cm, as the side bearings already required; within 25 cm one echo still latches at once.
   Cost: in the host model the forward bearing is re-read every ~102 ms while driving, about
   7 cm at the calculated 71 cm/s (5600 steps/s at 78.4 steps/cm, unmeasured) before braking
   begins; the calculated braking distance is about 14 cm.
2. **Calibration bumps**: five 100-step moves with a stop between each, before every leg. This
   is a stop-and-go by design and is unchanged (calibrated route).
3. **LiDAR caution zone and holds**: `GO:SLOW` drops to 3900 steps/s, and LiDAR STOP holds until
   confirmed clear. Unchanged.

To tell which one a run shows, keep the executor USB log (`[HALT] ...` lines: `SONIC:BLOCKED`
is the sonar, otherwise the LiDAR/path) and the Pi log (`Executor motion: ...`). The Pi's floor
log is saved only if it is teed to a file.

## Verification

- Firmware host tests: all 630 checks pass, including the new `tools/firmware_tests/test_continue_run.cpp`
  (real main loop, kinematic hallway: emergency stop mid-straight, mid-turn, parked, on the
  return route; a new S3 reference with and without confirmable walls; Release and refused
  Continue) and new checks in `test_controller.cpp` (sonar range, single/two-echo stop) and
  `test_bridge.cpp` (CONTINUE gate and correlation).
- The new tests were run against the code from before the change: the Continue-run tests,
  three sonar checks, the first bridge CONTINUE check and three Pi tests fail there. The
  reconstructed pre-change `pointsRun.ino` matches its SHA-256 in
  `artifacts/review-2026-11-14/source-hashes.csv`.
- Two problems were found and fixed while developing, each by a new test: after a route
  finished, a Release left the parked route as "live", so a later emergency stop at IDLE
  offered Continue; and a continued long-hallway tail shorter than a centring sidestep pass
  did not end where the uninterrupted run did (the old "ignore centring progress over a
  quarter of the straight" rule, which full runs never reach, is now "at most the straight").
- Long hallway in the model: stopped 200,000 steps in and continued, or stopped 1,500 steps
  before its end and pushed 20 cm sideways, it ends at 7606.0 cm, the same as without a stop.
- Pi: full suite 476 tests OK with 33 Tk tests skipped (no display); `test_monitor_ui` 62 tests
  OK on a private temporary Xvfb display. Source sanity and `tools/verify_project.py --host-only`
  pass.
- ESP32 target builds (external build paths): executor `esp32:esp32:esp32` 418,662 bytes
  flash, bridge `esp32:esp32:esp32s3` 590,052 bytes, no warnings.

Evidence, hashes and logs: `artifacts/continue-run-2026-11-14/`.

## Bench steps (owner-run)

1. Flash both complete sketches; deploy the whole `RasPi/` directory. Check
   `[EXECUTOR BUILD:CONTINUE-20261114 ...]` on the executor USB log and
   `[BRIDGE BUILD:CONTINUE-20261114]` in the Pi log.
2. Wheels lifted: start a manual disposal; during the first straight press **EMERGENCY STOP**.
   The *Controls* tab must show the stop latched and **Continue run** enabled. Press it,
   confirm; the wheels may turn only after the path is clear again, and the log shows
   `[ROUTE CONTINUE:1/15]`. Stop again and press **Release stop**: IDLE, Continue disabled.
3. On the floor, supervised: stop during the first turn, continue, and check that the turn ends
   facing the hallway (it must not turn a further 90 degrees). Note `[HEADING TURN ...]`.
4. Sonar: an object at about 100 cm straight ahead must be read; at 120 cm the forward slot in
   `[SONAR SWEEP ...]` must read `---` (no echo).
5. During a normal run, keep both logs and note when the motors stop and go.

These steps are not yet run. Software tests do not prove stopping distance, wheel slip or that
a continued route ends at its physical destination.
