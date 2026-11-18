@AGENTS.md

# Claude Code notes

Active sources: `RasPi/final_w_serial.py` and its helpers, `BLE_Receiver-Final/` (S3 bridge),
`MAIN_EXECUTOR_CODE/` (motor controller), `Garby_MobileApp/app/` (Android).
`RasPi/`, `BLE_Receiver-Final/` and `MAIN_EXECUTOR_CODE/` hold only deployable files; tests,
the Pi simulator and bench sketches are in `tests/` (`tests/pi`, `tests/firmware`, `tests/sketches`).
`WebConnect-GarbyController` is not production.
Newest findings: `docs/phone-controls-rundown-2026-11-18.md` (owner: every phone control
checked; the Pi's phone Developer handler called missing functions; phone arrows are Pi-timed
0.6 s nudges; Pi `PHONEDEV-20261118`), then `docs/runstart-2026-11-16.md` (owner: a RunStart tab in the Developer
window, a list of Forward/Backward/Turn left/Turn right/Bump calibrate steps with "+" and a bin,
locked once started, max speed and acceleration, no LiDAR/sonar/MPU checks; emergency stop,
Continue run and Reverse to start from the executor's saved exact place; `[TEST:id:P|C|U:..]`,
`[RUN AT/SAVED/CLEAR]`; lists up to 8000 steps/s since build `RUNSTART-20261116B` on all three
parts, `RUNSTART-20261116` (5600 cap) is on both boards. Before it the owner's
"weird noise, one motor" report: route motor code unchanged since Nov 15, the remote drove both
wheels fine, bench tests were held by the LiDAR on the desk), then
`docs/operator-decides-2026-11-16.md` (first saved log: a held Manual
disposal route survived closing the monitor and drove on by itself when the LiDAR cleared
or read-only was switched on; now both latch the operator stop; Pi `DECIDE-20261116`), then
`docs/side-read-only-2026-11-16.md` (owner: sides read-only, the third
LiDAR group; `side_disabled` makes `_side_status()` clear and drops the rear-half bands from
`R=`; spin sweep stays with front/back; Pi `SIDES-20261116`), then `docs/remote-control-2026-11-16.md` (owner: hold-to-drive arrows in the
Developer window, `W/S/A/D` mask 0, Slow 400 / Fast 2000 steps/s, no step value, no time limit,
no sensor gates by the owner's choice; `[TEST END]` on release, 500 ms lease, window must keep
saying "held"; build `REMOTE-20261116` on all three parts), then `docs/gui-controls-2026-11-16.md` (owner runs the monitor over `ssh -Y`; a
"Manual disposal that did nothing" was a running route held by bench objects inside the side
limit while the greyed button ignored clicks; Developer tests now in their own window, greyed
controls explain themselves, Log tab + `~/garby-logs/bridge.log` instead of the terminal,
`RasPi/49-garby-reboot.rules` for Restart over ssh; Pi `CONTROLS-20261116`; executor still on
`REARPARK`, `BACKTEST` executor not flashed), then `docs/backward-test-2026-11-16.md` (owner: "backward was not working at all",
wheels lifted: nothing could reverse on the bench, a lifted route faults at its first turn; new
Developer test `B` inside the parking rear scope, live rear-wall reading; build `BACKTEST-20261116`
on all three parts; the flasher installed Codex's `REARPARK-20261116` on both boards that morning),
then `docs/rear-wall-parking-2026-11-16.md` (Codex: parking squares the rear to its LiDAR wall,
D lines), then `docs/reverse-parking-2026-11-16.md` (both routes end by backing into their
parking spot: `RouteOp::BACKING`, Pi `R=`, bridge `[BACK:START]`/`GO:BACK`; likely-person label
needs motion relative to the drifting still scene; build `PARK-20261116` on all three parts;
the owner's flasher failed to build `DEVTEST-20261116`, so that was never installed), then
`docs/developer-tests-start-hold-2026-11-15.md` (a start that never moved was a
LiDAR side-band hold from returns at the body's front edge; Developer tests tab; stricter person
marker; executor `DEVTEST-20261116` with quiet test turns, bridge `DEVTEST-20261115`; tests moved
to `tests/`), then `docs/phone-control-headless-2026-11-15.md` (phone control and the map over
Firebase, `run_headless.sh`, thermal profile) and `docs/objects-station-reverse-2026-11-14.md`
(likely-person red dots, hallway objects, station steps, Reverse to start, far wall fit and LiDAR
mounting-angle learning, bench steps); progress is in `docs/work-plan-2026-11-14.md`. Before it: `docs/continue-run-sonar-range-2026-11-14.md` (Continue
run after an emergency stop, route step lists, 110 cm sonar range, two-echo forward stop) and
`docs/obstacle-sweep-hold-2026-11-12.md` (stuck-forever root causes, holds that
resume, low power, sonar sweep, moving-object prediction, pre-run centring, buzzer, bench steps), `docs/lidar-map-2026-11-12.md` (X3 scan truncation, radar map) and
`docs/navigation-2026-11-11.md` (navigation design, calibration).

```bash
python3 -B tools/verify_project.py --host-only 2>&1 | grep -vE '^PASS'
bash RasPi/run_bridge.sh --check-runtime
bash RasPi/run_bridge.sh --check-ble     # scan only; --connect needs wheels lifted
arduino-cli compile --fqbn esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi --build-path <scratch> BLE_Receiver-Final   # S3 N16R8 (owner, 2026-11-15)
arduino-cli compile --fqbn esp32:esp32:esp32:FlashSize=4M,PSRAM=disabled --build-path <scratch> MAIN_EXECUTOR_CODE
```

Load `garby-robot-maintenance` for protocol, firmware or field-triage work. It has the bench isolation
needed before running the bridge: this tree holds a real Firebase key and the operator stop latch.
Android cannot build here (no JDK).

## Keeping token use down

- `final_w_serial.py`, `BLE_Receiver-Final.ino` and `MAIN_EXECUTOR_CODE.cpp` run to 2,000-2,800 lines:
  `grep -n` first, then read a range.
- Dated reports at the root, in `docs/` and in `artifacts/` are history. Read the newest one only.
- Filter test output with `grep -vE '^PASS'`. The full Pi suite (`python3 -B -m unittest discover -s tests/pi`) takes about 1-2 minutes; run
single modules from `tests/pi` while iterating (they import `raspi_path`, which puts `RasPi/` on the path).
