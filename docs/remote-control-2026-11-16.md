# Remote control: drive while an arrow is held — 2026-11-16

## 1 — Request and the owner's choices

> "Can we add a remote control section with arrows? Up (Forward), Down (Backward), Left (Turn
> Left), and Right (Turn Right). No specific step value, only runs when held."

When asked, the owner chose:

- **Where:** the Developer tests window.
- **Sensor checks:** none. "Because user can release the hold if it was needed to stop"; it
  runs only while held, never for a set number of steps.
- **Speed:** a Slow/Fast switch.
- **Time limit:** none.

## 2 — What it does

The Developer tests window now has two tabs, *Bench tests* and **Remote control**. The remote
has four arrows (▲ Forward, ▼ Backward, ◀ Left, Right ▶), a **Slow (about 5 cm/s, 400
steps/s) / Fast (about 25 cm/s, 2000 steps/s)** switch, and a status line.

- **Pressing** an arrow starts one drive: forward, backward, or a spin in place to the left or
  right. **Letting go** stops it: the Pi sends `[TEST END]`, and the executor brakes with its
  quick controlled stop. Over this short ramp GARBY travels about 2 cm when Fast, and well
  under 1 cm when Slow.
- **No step value, no time limit, no LiDAR, gyro or sonar check**, as the owner chose.
- **Starting.** It starts only when the robot is idle and ready, the same as every developer
  test. A press that can't start says why on the status line and moves nothing. A tap released
  before the start reached the robot is never sent.
- **What still stops it**, beyond letting go:
  - **Window closed or minimised:** the window lets go.
  - **Frozen window or dropped `ssh -Y` session:** the window stops saying "held". The Pi
    renews the drive every 120 ms only while the window reports the arrow pressed, at least
    every 0.5 s. Without renewal, the executor's 500 ms lease runs out.
  - **Link or controller loss:** the same lease.
  - **EMERGENCY STOP:** on both windows.
  - **Monitor refresh failure:** the window lets go.
- **Left and Right** use the executor's current guess of which motor is the left wheel. The
  first MPU-measured turn of a route confirms or swaps it. If Left spins right, report it; don't
  rewire anything.

A drive ends at IDLE like every developer test, needing a new deliberate start for any route.
A new press works once the robot has reported idle again, a fraction of a second after the
last release. Until then the status line says the last drive is still finishing.

Wire format (`SYSTEM_ARCHITECTURE.md`, "Finite developer tests"):

- Start: `[TEST:<id>:W|S|A|D:<speed>:0]`
- Renew: `[TEST KEEP:<id>]` every 120 ms
- Let go: `[TEST END:<id>]`, which the bridge relays whatever its own record, lease, age or a
  latched stop

## 3 — Upload

**MCU upload requirement: Both.** Upload the complete `BLE_Receiver-Final` and
`MAIN_EXECUTOR_CODE` sketches, and deploy the whole `RasPi/` folder; all are
`REMOTE-20261116`. The bridge must accept `W/S/A/D` and relay `[TEST END]`; the executor runs
the drive. This build contains everything still waiting:

- Executor: `BACKTEST-20261116` (Backward test) and Codex's `REARPARK-20261116`.
- Pi: `CONTROLS-20261116` (two windows, Log tab).

The bridge's `BACKTEST-20261116`, flashed at 07:55, is replaced by this one. With an older
board the arrows do nothing: the press is refused or never answered, and the status line says
so.

## 4 — First check (wheels lifted, then floor)

1. Developer tests… → Remote control. Slow. Hold ▲ for two seconds, then let go: both wheels
   turn forward and stop as you let go. Then ▼: both turn backward.
2. Hold ◀ and Right ▶: the wheels turn opposite ways. With the wheels lifted the gyro sees
   nothing, which is fine here: no rotation is measured.
3. Hold ▲, then close the Developer window while still holding: it stops.
4. On the floor, Slow first, with space around GARBY. **Nothing watches for obstacles while you
   drive.** Then Fast.

## 5 — Checks and limits

- **Host tests:** executor `tests/firmware/test_developer.cpp` (10 new checks), bridge
  `test_bridge.cpp`, end to end through both boards `test_system.cpp`; Pi
  `tests/pi/test_developer_support.py`, `test_monitor_ui.py` and `test_runtime.py`.
- **Target builds:** both ESP32 targets compile.
- **Evidence:** `artifacts/remote-control-2026-11-16/` (screenshot, logs).
- **Not measured:** stopping distance, wheel slip, and how the dialogs and buttons respond on
  the owner's own desktop over `ssh -Y`.
- **No obstacle checks** apply while driving by hand, by the owner's choice. The robot does not
  stop for anything except letting go, the window, the link and the emergency stop.
