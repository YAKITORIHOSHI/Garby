# Start hold, developer tests and person markers — 2026-11-15

Source build `DEVTEST-20261115` on the Pi, the bridge and the executor. Source implemented and
software tested; **not uploaded and not physically accepted**. Evidence lives in
`artifacts/start-investigation-2026-11-15/` (not published with the repository).

## Update 2026-11-16: turn noise, clearer tab, quiet shutdown, deployment layout

After the owner's first bench run (both boards on `DEVTEST-20261115`, wheels lifted, LiDAR,
MPU and sonar all ticked): Forward was fine, but Turn left/right made a random, intense noise;
the tab's greyed-out buttons gave no reason; and every shutdown logged
`[ERROR] LiDAR stream stale for > 0.8 s; issuing fail-closed STOP`.

- **Turn noise (executor, `DEVTEST-20261116`).** A turn waits for LiDAR turning-circle
  clearance and the sonar spin check. On the bench those flickered, and each hold braked the
  spin and started it again as soon as clearance returned. Every new spin also reset the
  turn's step limit, so the wheels stop-started until the 8 s limit. In the host model
  (clearance blocked 150 ms every 1.5 s) the old code re-spun 5 times. Now a developer test
  never restarts its wheels after a hold. It ends at once and names the cause: `HELD_LIDAR`,
  `HELD_SONAR`, `HELD_MPU`, `MPU_DATA_LOST`, or `NO_ROTATION` for a lifted turn (the gyro sees
  nothing). A test that never moved reports what held it instead of a bare timeout. The
  turn also no longer re-sends an unchanged speed to both drivers (125 ramp re-plans per
  turn before, 16 after). Routes keep their hold-and-resume rule; nothing changed for them.
- **Developer tab (Pi).** A banner on top says Ready, "Not available:" plus the reason and
  what to do (emergency stop on, route running, not connected, waiting for the robot…), or
  the running test. The tick boxes say what they make GARBY do. The last result is shown in
  plain words with steps and an estimated distance. Readiness now survives one late status
  reply (12 s instead of 7.5 s; the bridge asks every 5 s), and a later "ready" no longer
  wipes the last outcome.
- **Shutdown (Pi).** Shutdown stops the LiDAR driver first, so the watchdog saw a stale stream
  and logged an ERROR. It now logs `LiDAR driver stopped for shutdown; holding fail-closed
  STOP` at INFO; the STOP is still sent.
- **Deployment layout (owner).** `RasPi/`, `BLE_Receiver-Final/` and `MAIN_EXECUTOR_CODE/` now
  hold only what runs on the robot. The Pi tests and fixtures, the Pi simulator, the firmware
  host tests and the bench sketches moved to `tests/pi`, `tests/firmware` and `tests/sketches`;
  the monitor preview moved to `tools/preview_ui.py`. Moves are listed in
  `artifacts/deployment-layout-2026-11-16/moves.json`.

**Upload: Executor only** (`MAIN_EXECUTOR_CODE`, now `DEVTEST-20261116`). The bridge source is
byte-identical to `DEVTEST-20261115`, which is already installed. Deploy the whole `RasPi/`
folder again.

## 1. Why GARBY "started" but the wheels never moved

Report: the Pi, the bridge and the executor were all connected and the log said the system was
running, but the motors made no sound at all.

Finding: the start had worked. The executor accepted the outbound route (`[RUNNING]`), then
held at its first step because the LiDAR said something was too close. "Running" means a route
is active; it does not mean the wheels are turning. The hold reason was in the status but not
shown prominently, so it looked like nothing happened.

Evidence (live monitor and 47 recorded scans, robot stationary, `live-scan-analysis.txt/.png`):

- Every scan (47 of 47) held the **front box** (nearest return 30.0 cm ahead of the LiDAR) and
  the **left side band** (128 returns 23.0–26.5 cm ahead and 31.1–32.0 cm left of the LiDAR).
  Station-wall filtering did not change the side-band result.
- The returns form a straight line across the full assumed body width, exactly at the front
  edge of the default 60 × 60 cm outline. In an open area, that pattern most likely comes from
  **part of GARBY itself** in the LiDAR scan plane (a front panel, bin lip, frame bar or cable)
  rather than an obstacle. This is an inference from the scan shape; it has not been confirmed
  by looking at the robot.
- Front/rear read-only cannot release this: the side band is never bypassed, by design.
- BLE, MPU (`mpu=OK`) and LiDAR were healthy; there were no link drops. This is not a link,
  firmware or motor fault. The 2026-11-14 base-parking hold
  (`artifacts/manual-disposal-hold-2026-11-14/`) had the same shape.

The proper fix is geometry, not a bypass. `RasPi/hallway_geometry.py` ignores any return inside
the configured body outline ("can only be the robot itself"). The outline defaults are
unmeasured. Measure them at the height of the LiDAR's scan plane and set:

| Variable | Default | Measure |
|---|---|---|
| `GARBY_CHASSIS_FRONT_CM` | 30 | LiDAR centre to the front-most part of GARBY |
| `GARBY_CHASSIS_REAR_CM` | 30 | LiDAR centre to the rear-most part |
| `GARBY_CHASSIS_WIDTH_CM` | 60 | full body width |
| `GARBY_LIDAR_Y_OFFSET_CM` | 0 | LiDAR centre left (+) or right (−) of the body centre line |

Set them in the shell before `bash RasPi/run_bridge.sh` (or in the headless launcher's
environment). If something sticks out in front of the body only at LiDAR height (a cable, a
bracket), move it out of the scan plane instead of enlarging the outline.

## 2. What changed

### Motion hold shown on every tab (Pi only)

While a route is active and the executor reports a hold, the monitor shows `Motors held: …`
above every tab. A generic LiDAR hold is expanded to the gate that is actually holding, for
example `Side clearance blocked (LEFT_BAND)`. Bypassed groups are left out, and rear distances
are never named because they do not hold a forward move. The phone's status mirror receives
the same text. It is telemetry only.

### Developer tests (Pi, bridge and executor)

At the owner's request, the monitor has a **Developer tests** tab for checking each part
separately. It has three checkboxes, all ticked by default, that apply to **one test only**:
LiDAR clearance (all zones, including the side bands and scan freshness), MPU heading/turn
feedback, and obstacle sonar.

| Button | Moves | Limit |
|---|---|---|
| Check sensors (no movement) | nothing | shows the gyro heading and the LiDAR wall angle as two separate readings |
| Forward ~5 cm | both wheels forward | 400 steps (about 5 cm with the nominal 65 mm wheel, 1600 steps/rev) |
| Motor 1 only / Motor 2 only | one wheel | 100 steps (about 1/16 of a turn) |
| Turn left / right 10° | spin in place | MPU-measured; needs MPU ticked; no wall alignment |

Every movement needs a Yes in a confirmation dialog. Limits that cannot be switched off:
400 steps/s (about 7 % of route speed), an absolute 8 seconds including waits, and a live lease
(the Pi sends KEEP every 200 ms; the executor stops if none arrives for 800 ms). The emergency
stop, a link/protocol/sequence stop and a rejected motor command still end the test at once.
Tests run only from IDLE with the stop released. When a test ends, the normal gates come back,
a bypassed sonar must confirm clear again, any Continue/Reverse checkpoint is discarded, and a
route needs a new deliberate start. A test can never start or resume a route.

The wire protocol and the bridge/executor rules are in `SYSTEM_ARCHITECTURE.md`, section
"Finite developer tests (2026-11-15)". The panel stays disabled until both controllers run
`DEVTEST-20261115`. Older firmware never sends the `[TEST READY:1]` capability.

### Fewer false "likely person" markers (Pi only)

Before this change, any person-sized, non-flat mover could be labelled. A box corner is
non-flat, so moving boxes and fragments were marked red. A track now also needs repeated
rounded, torso-sized arc profiles (8 to 40 cm radius, at least three agreeing sightings). It
loses the label when it re-associates with a different object or keeps showing flat panels.
The label is still a size-and-motion guess shown on the map and in reports, not person
recognition. No stop or slow rule reads it. In replay, generic moving-object decisions were
identical before and after across 1,236 cases (`person-agent/README.md`).

## 3. Owner steps

1. **Deploy the whole `RasPi/` directory** to the Pi (it includes the new
   `developer_support.py`).
2. **Upload the sketches** (whole sketch folders). On 2026-11-16 only the executor changed;
   the bridge `DEVTEST-20261115` already installed is current:
   - Bridge `BLE_Receiver-Final`: ESP32-S3 N16R8,
     `esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi`. The bridge pins (I2C 8/9, UART 17/18) do
     not use the octal-PSRAM pins 35–37.
   - Executor `MAIN_EXECUTOR_CODE`: ESP32 Dev Module, `FlashSize=4M,PSRAM=disabled`.
   - Check the USB consoles at boot: executor `DEVTEST-20261116`, bridge `DEVTEST-20261115`.
3. **Wheels lifted**, run the monitor and capture the log:
   `bash RasPi/run_bridge.sh 2>&1 | tee ~/garby-devtest.log`. Open **Developer tests** and wait
   for the green "Ready" banner (otherwise the banner says why not).
4. **Check sensors.** Note the gyro heading and the wall angle.
5. **Forward ~5 cm with all three ticked.** If the robot is parked as before, expect no
   movement and "held by the LiDAR" after about 8 s, while the LiDAR tab footer ("Zones: front box,
   left band") and the log (`LiDAR chassis zones:`) name the zones. That confirms the LiDAR
   hold. (The `Motors held` banner
   is for routes; it is hidden during a test.)
6. **Untick LiDAR clearance, Forward ~5 cm.** Both wheels should turn about a quarter turn.
   The result line shows both step counters.
   - Counters about 400 but the wheels did not turn: motor power, driver enable or wiring.
   - Counters 0: a firmware gate is still holding. Send the log.
7. **Motor 1** and **Motor 2** (LiDAR unticked). Note which physical wheel each one turns.
8. **MPU turn left/right** (LiDAR unticked, MPU ticked). Lifted, the gyro sees no rotation.
   The wheels spin in opposite directions for about 3 s, then the turn stops on its step
   limit and the test reports that the gyro saw no rotation (`NO_ROTATION`). That shows each wheel's turn direction.
   With no measured rotation, no wheel swap and no turn scale are learned, and no route
   fault is latched. Do the real 10° check on the floor, with all three ticked once the
   outline is fixed (step 10).
9. Press **EMERGENCY STOP** during a test once to confirm it ends the test, then release.
10. **Measure the outline** (section 1) and set the variables. Run **Read sensors** and check
    that the `LEFT_BAND`/`FRONT_BOX` hold is gone with every check ticked, before any route.

Send back `~/garby-devtest.log` and the results of steps 5 to 8.

## 4. Verification

Re-run on 2026-11-15 22:57–23:05 after Codex's last edits, against the files as they now stand:

| Check | Result |
|---|---|
| `python3 -B -m unittest discover -s RasPi -p 'test_*.py'` (no display; now `-s tests/pi`) | 604 tests OK, 40 skipped (Tk and recordings) |
| `test_monitor_ui` + `test_runtime` under a private `xvfb-run` display | 181 tests OK, none skipped |
| `python3 -B tools/run_firmware_tests.py` (includes the new `test_developer.cpp`) | 767 PASS, 0 FAIL |
| `python3 -B tools/source_sanity.py .` | PASS |
| Executor compile, `esp32:esp32:esp32:FlashSize=4M,PSRAM=disabled` | OK, 426,778 bytes, 0 warnings |
| Bridge compile, `esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi` | OK, 599,170 bytes, 0 warnings |

Firmware sources match the hashes recorded by the agents' builds
(`bridge-agent/verification.json`, `executor-agent/results.json`). The final hashes and binary
hashes are in `artifacts/start-investigation-2026-11-15/FINAL-VERIFICATION.md`.

Re-run on 2026-11-16 after the turn, tab, shutdown and layout changes (`DEVTEST-20261116`):

| Check | Result |
|---|---|
| `python3 -B -m unittest discover -s tests/pi -p 'test_*.py'` under a private `xvfb-run` display | 608 tests OK, 1 skipped (optional phone-app cross-check) |
| Same, no display | 608 OK, 41 skipped (Tk) |
| `python3 -B tools/run_firmware_tests.py` (from `tests/firmware`) | 770 PASS, 0 FAIL |
| `python3 -B tools/source_sanity.py .` | PASS |
| Executor compile, `esp32:esp32:esp32:FlashSize=4M,PSRAM=disabled` | OK, 427,602 bytes, 0 warnings |
| Bridge compile, `esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi` | OK, 599,170 bytes (unchanged source), 0 warnings |

New executor checks were first shown to fail on the old behaviour: with the hold rule
disabled, the flickering-clearance turn re-spun 5 times and ran to the 8 s limit; without the
speed de-duplication, a floor test turn re-planned the ramps 125 times.

## 5. Limits

- Nothing here was uploaded or run on the robot by Claude. The owner's 2026-11-15 bench run is the
  only physical evidence (forward fine, turn noisy); the 2026-11-16 fixes are not yet bench-tested.
- Step counts do not prove wheel travel. 400 steps ≈ 5 cm assumes the nominal wheel geometry.
- The near-front returns are inferred, not seen, to be GARBY's own body. The outline is still
  unmeasured.
- With LiDAR unticked, nothing on the Pi side stops a test for an obstacle. Only the sonar (if
  ticked), the emergency stop, the lease and the travel/time limits remain. Keep tests
  supervised.
- The "likely person" marker is a guess from a 2D scan and can still mark a rounded mover that
  is not a person.
