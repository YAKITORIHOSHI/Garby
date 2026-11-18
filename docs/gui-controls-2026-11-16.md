# Manual disposal "not working", two windows, logs in the GUI — 2026-11-16

## 1 — What was reported

The owner, who always runs the monitor over `ssh -Y garby@garby-desktop.local`, said:

- Keep the logs in the GUI instead of printing them in the terminal.
- Manual disposal "does not work": the Yes/Cancel prompt was missing and nothing moved.
  Developer tests worked fine; everything else felt wrong.
- Asked how to separate the Developer tests from the actual controls, the owner chose
  **two windows**.

## 2 — What the evidence shows

No log of the session was saved; the terminal was the only log. What remained:

- **Flasher log.** The bridge was flashed with `BACKTEST-20261116` at 07:55 EST. The executor
  was **not** reflashed: it still runs `REARPARK-20261116` (04:12). So the Backward test cannot
  run until the executor is flashed too.
- **Status file** (`~/.local/state/garby/bridge-status.json`, monitor 07:56-08:01):
  - Robot state **OUTBOUND**: a route had been accepted.
  - Hold `Side clearance blocked (RIGHT_BAND)`; zones `FRONT_BOX, SLOW, RIGHT_BAND, TURN_FRONT`.
  - Sonar off.
  - "Pi reboot failed: Call to Reboot failed: Interactive authentication required."
- **Live LiDAR capture**, 08:08, robot where the owner left it on the bench: 12 s, isolated ROS
  domain, listen only, no bridge; `artifacts/gui-controls-2026-11-16/scans-bench-0808.npz`.
  Replayed through the production zone code:
  - On the right, a straight line of returns 19.9-21.9 cm from the LiDAR, against a 22 cm
    side limit.
  - In front, returns at 36 cm inside the front box.
  - Picture: `scene-bench-0808.png`. On Nov 15 the blocking returns were on the left and front,
    at other places. They move with the bench layout, so they are bench objects, most likely
    not part of GARBY. The outline is still unmeasured.

**What happened.** A Manual disposal started a route (state OUTBOUND). The never-bypassed side
check held it at once, because something on the bench was inside the side limit. That is the
same story as Nov 14 at the base (`docs/developer-tests-start-hold-2026-11-15.md`). While a
route runs, the Manual disposal button is greyed out, and Tk ignores clicks on a greyed-out
button, so a second click showed no prompt and no reason. The Developer test moved the wheels
because LiDAR was unticked there; route motion can never untick the side band. The controls
themselves behaved as designed. What failed was the GUI saying so.

**Restart.** `systemctl --no-ask-password reboot` is allowed without a password only for the
user at the Pi's own screen. polkit 124 on Ubuntu 24.04 has `implicit active: yes` but
`implicit any: auth_admin_keep`. An ssh session is "any", so the Pi refused the reboot.

## 3 — Changes (Pi only, `CONTROLS-20261116`)

- **Developer tests in their own window.** The **Developer tests…** button in the main header
  opens it. It has its own EMERGENCY STOP, and its Yes/No dialogs open over it. Closing it
  while a test runs cancels the test: the Pi stops renewing the lease and the executor brakes
  within 800 ms. The main window keeps only the real controls.
- **Greyed-out controls answer a click.** Manual disposal, Release stop, Continue run, Reverse
  to start, the sonar switch and Restart each explain why they are unavailable. If a route is
  running, the message names it, gives its hold in plain words, and says how to end the route
  (EMERGENCY STOP, then Release stop). An available button still asks Yes/Cancel first, as
  before.
- **Holds in plain words**, kept on one line. For example: `Side clearance blocked
  (RIGHT_BAND): something is right beside GARBY's right side`.
- **Logs in the window.** A new *Log* tab shows the Pi's log: warnings orange, errors red,
  800 lines, a Clear button. Each run is also saved in `~/garby-logs/bridge.log`, one file per
  day with 14 kept; this is the same folder `run_headless.sh` uses. The terminal gets one line
  saying where the log is, and one when the monitor stops. These now go to that log instead of
  the terminal:
  - the ROS node's own messages,
  - the LiDAR driver's error output (a repeated line is counted, not repeated),
  - monitor callback errors, thread errors and Python warnings.
  Headless mode keeps its console, which `run_headless.sh` already writes to its file.
- **Restart over ssh.** `RasPi/49-garby-reboot.rules` lets the account `garby`, and only it,
  reboot. A refusal now says so in the Controls status and names the one-time fix. Install once:

  ```bash
  sudo install -m 644 RasPi/49-garby-reboot.rules /etc/polkit-1/rules.d/
  ```

  To remove it: `sudo rm /etc/polkit-1/rules.d/49-garby-reboot.rules`.

Nothing in the motion gates changed. The side band is still never bypassed in a route.

## 4 — What to do

1. Deploy the whole `RasPi/` folder (`CONTROLS-20261116`). **MCU upload requirement for this
   change: None.** Still pending from before: the **executor** sketch `BACKTEST-20261116`.
   The bridge already has it; without the executor, the Backward test cannot run.
2. Optional: install the reboot rule above so Restart works over ssh.
3. On the bench, use the Developer tests window (untick LiDAR if the bench is cluttered). Real
   controls such as Manual disposal will hold while anything is inside the side limit. Test
   them on the floor, or clear about 25 cm around the robot.
4. After any test, the log is in `~/garby-logs/bridge.log`; send that with a report.

## 5 — Limits

Host tests and Xvfb screenshots only (`artifacts/gui-controls-2026-11-16/*.png`). Not verified:
the dialogs on the owner's own desktop over `ssh -Y`; the polkit rule on the Pi (it needs sudo,
which was not used). The chassis outline is still unmeasured, so whether the returns that sit
inside the assumed 60 × 60 cm outline belong to GARBY or to the bench is not proven.
