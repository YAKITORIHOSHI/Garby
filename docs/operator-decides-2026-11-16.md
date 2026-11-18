# You decide when GARBY moves — 2026-11-16

## Report

> "Why does it start when I turn my lidar read-only, let me decide when will it move such as
> release stop and dispose manually."

## What the first saved log shows (`~/garby-logs/bridge.log`)

Both boards ran `REMOTE-20261116` (flashed 08:57 and 09:00).

| Time | What happened |
|---|---|
| 09:02:56 | Manual disposal accepted, route **OUTBOUND**. On the bench the LiDAR held it (low power). |
| 09:03:09 | The monitor was closed. The executor kept the route, held, because a lost Pi only pauses a route. |
| 09:17:59 | A new monitor connected; the executor still reported OUTBOUND, step 1. |
| 09:18:14-15 | The LiDAR came up. Step 1 is a station step, whose rule leaves the station's walls out, so the path cleared: "clearance confirmed". **It drove step 2 (8000 steps) and began the left turn**, with no button pressed. |
| 09:18:27 | Monitor reopened, route still OUTBOUND, step 3, held by `Side clearance blocked (RIGHT)`. |
| 09:18:49 | The hold cleared and the turn went on, consistent with read-only being switched on. Toggle clicks were not logged then. |

So read-only didn't start a new route. A route that was still running and held simply carried
on as soon as anything cleared its path. That is by design for obstacles: holds resume when
clear (owner, 2026-11-12). But the owner had not pressed anything in this session.

## Change (Pi only, `DECIDE-20261116`)

- **A route already running when the monitor starts** is stopped. On the Pi process's first
  known state report, a route state (OUTBOUND, RETURNING, REVERSING) latches the operator stop.
  A Bluetooth reconnect within the same run is not a new monitor: a route survives link drops as
  before.
- **A read-only change during a route** (Front, Back or Sides; from the monitor, the phone or a
  headless key) latches the operator stop too. While idle, a change is just a setting. Once
  stopped, further changes are just settings until you choose. Every change is now logged
  (`LiDAR read-only changed: …`).
- **You choose next**, using the existing buttons:
  - **Continue run** finishes the route from where it stopped, with the new read-only setting.
  - **Reverse to start** drives the route back to where it started.
  - **Release stop** ends it at idle; then **Manual disposal** starts a fresh route.

  The executor keeps the route's checkpoint after this stop (firmware tests
  `test_continue_run.cpp`). The `[ROUTE FAULT]` line printed then is only the route unwinding.
- **The reason is shown** above the monitor's tabs ("Stopped for you to decide: …") and as the
  phone's operator message.
- The LiDAR driver's own `[INFO]` lines are now logged as INFO, not as orange warnings.

Unchanged: obstacle holds during a route you started still resume on their own when the path
clears (a person walking past). Nothing here grants motion; it only adds stops.

## Checks

- `tests/pi/test_operator_decides.py` (new, 5 tests). They failed on the previous code and
  cover:
  - a route found at start is stopped,
  - a route started in this session, and a reconnect, are not,
  - a change during a route stops it; idle or already-stopped changes don't,
  - the phone toggle follows the same rule,
  - the reason reaches the monitor and the phone message.
- `test_monitor_ui.py` (hold line), `test_log_support.py` (driver line levels).
- Full suite in `artifacts/operator-decides-2026-11-16/VERIFICATION.md`.

**MCU upload requirement: None.** Restart the monitor from this folder. Both boards stay at
`REMOTE-20261116`.
