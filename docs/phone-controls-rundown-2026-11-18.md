# Phone controls, every control checked — 2026-11-18

Owner: "just have full maintenance and rundown on controls to make sure it will work as
intended. EACH OF EVERY CONTROL". This covers every control on the Android app's Controls tab
(the **Controls** and **Developer** sections), traced from the phone's request through
`RasPi/cloud_control.py` to the Pi function the monitor's own button calls. The Pi monitor's
Controls tab and Developer window (bench tests, remote control, RunStart) are the reference.

Build `PHONEDEV-20261118` (Pi). No firmware changed.

## What did not work

1. **Five phone Developer actions crashed the Pi relay.** `cloud_developer_action` called
   `request_developer_run_resume`, `request_developer_run_clear`, `release_developer_hold` and
   `request_developer_hold`, which do not exist. The `NameError` came after the claim, so the
   poll raised ("Phone control bridge unavailable; retrying") and the next poll ended the
   claimed request `unconfirmed`. RunStart Continue/Reverse/Discard, all four arrows and the
   arrows' Stop never reached the robot.
2. **The arrows could not work like the window's even with the names fixed.** A held drive runs
   only while the window says "still held" every 100 ms; the Pi stops renewing it 0.5 s after
   that stops. The phone talks to the Pi through Firebase, polled once a second, so a tap would
   drive about half a second and end as `LEASE_EXPIRED` ("lost contact with the Pi"). The owner
   chose **Tap = short nudge** (below).
3. **Every Controls-tab request carried `sensors` and `fast`.** The app repository's example
   database rules refuse unknown fields (`$other: false`). Under those rules Release, Continue,
   Reverse, sonar, read-only and restart would all be refused, shown as "not on GARBY's
   operator list". The rules also allowed only eleven actions: sides read-only and every
   Developer action were refused.
4. **The phone's Developer gating did not follow the Pi.** Tests and arrows were offered while
   a route ran, while not idle and before the robot reported test support. Stop test and the
   arrows' Stop were disabled exactly while a test or drive was running. RunStart Continue and
   Reverse were disabled after an emergency stop (the window releases the stop first; that is
   their main use) and enabled with no saved list. Turns were offered with the gyro unticked.
5. **Refusals had the wrong reason or none.** A refused test reported the last test's *result*
   ("refused (Motor 1 +400 steps …)") or nothing. RunStart requests the Pi refused ("Nothing
   to continue.") were reported `accepted`.
6. **The phone could not see why.** The mirror lacked the Developer window's message and
   "Not available" reason and the RunStart state, so a test held by the LiDAR or stopped by the
   lease was never explained, and the RunStart buttons could not know the list's place.
7. Developer outcomes read "confirmed by the controllers" when only the Pi had taken the
   request; the phone's cancel was logged as "the Developer tests window was closed".

## Every control

"Offered" is when the phone enables the button; in every row it is now the Pi's own rule.

### Controls section

| Control | Request | Pi function | Offered | Result |
| --- | --- | --- | --- | --- |
| Continue run | `CONTINUE` | `operator_control.request` | `canContinue` | Worked; confirmation text now the window's |
| Reverse to start | `REVERSE` | same | `canReverse` | Worked; adds "obstacle detection … remain active" |
| Release stop | `RELEASE` | same | `canRelease` | Worked |
| Obstacle sonar | `SONAR_ON` / `SONAR_OFF` | same | `canConfigure` (idle) | Worked |
| Front / Side / Rear LiDAR checks | `*_READONLY_ON/OFF` | `cloud_set_read_only` | `readOnlyAllowed` | Worked; side needed the rules; now says a change mid-route stops GARBY (operator decides) |
| Restart Pi and controllers | `REBOOT` | `operator_control.request` | `canReset` | Worked |
| Emergency stop (under every tab) | `ESTOP` | same, applied before the claim | always (link up) | Unchanged |

All of these failed under the example rules because of the extra fields (item 3); they now
send only the six fields the rules list.

### Developer section

| Control | Request | Pi function | Offered | Result |
| --- | --- | --- | --- | --- |
| Sensor chips (LiDAR 1, MPU 2, sonar 4) | `sensors` on `DEV_TEST_*` only | the test's mask | locked while a test runs, as the window's checkboxes | Fixed |
| Forward/Backward ~5 cm, Motor 1/2, Turn 10°/90° | `DEV_TEST_F/B/M1/M2/L/R/L90/R90` | `request_developer_test` | `developerEnabled`; turns need MPU ticked | Fixed gating and reasons |
| Check sensors | `DEV_TEST_I` | same, no confirmation (as the window) | `developerEnabled` | Fixed gating |
| Stop test | `DEV_CANCEL` | `DeveloperTestControl.stop("from the phone")` | while a test or list runs | Fixed (was disabled while running) |
| Arrows | `DEV_HOLD_W/S/A/D` + `fast` | `request_hold(..., pulse_s=PHONE_NUDGE_S)` | `developerEnabled` | Fixed: nudge |
| Stop (arrows) | `DEV_HOLD_STOP` | `release_hold` → `[TEST END]` | always | Fixed (`NameError`) |
| RunStart Continue run | `DEV_RUN_CONTINUE` | `request_run_resume("C")` | saved list at start/part-way; ready, or stopped and releasable | Fixed (`NameError`, gating) |
| RunStart Reverse to start | `DEV_RUN_UNDO` | `request_run_resume("U")` | part-way/end; same | Fixed |
| RunStart Discard saved | `DEV_RUN_CLEAR` | `request_run_clear` | a saved list not moving (also while stopped) | Fixed |
| RunStart rows, Max speed, Acceleration, Start run | `DEV_RUN_START` + `program`, `speed`, `accel` | `request_run` | ready; rows locked while a list moves or stands part-way | New (owner's follow-up) |

Bench-test confirmations now say what the window says: the sensors obeyed and ignored, and for
turns that GARBY must be on the floor. RunStart confirmations give the step and say when the
emergency stop will be released first. The phone shows the saved list (steps, done/current
marks, place, speed and acceleration).

### RunStart list editor (owner's follow-up, 2026-11-18)

"Something missing, the RunStart Textfield and MaxSpd/Accel - please do it on the mobile app
too." The phone now has the RunStart tab's editor: numbered rows, each with what it does
(Forward, Backward, Turn left, Turn right, Bump calibrate) and a steps field, a bin per row,
Add step (up to 20), Max speed and Acceleration fields (defaults 2351 and 4800) and Start run,
plus Copy saved list to edit the robot's last list. The phone checks the rows with the
window's own rules and words (`_run_collect`, `run_program`) and Start run stays disabled with
that reason until they pass; the confirmation lists the steps, speed and acceleration as the
window's does. The rows lock while a list moves or stands part-way, as the window's do.

The request is `DEV_RUN_START` with `program` ("F4000,L980"), `speed` and `accel`. The Pi reads
the list with `parse_run_program` (anything malformed: "The step list is not valid."), checks it
again with `DeveloperTestControl.run_program`, refuses it while a list stands part-way or
anything runs, then calls the window's `request_run`: the same `[TEST:id:P:speed:accel:list]`,
KEEP lease, no sensor gates and no time limit.

`DeveloperTestControl.stop()` ends whatever runs the fastest way available: a held drive or
nudge gets `[TEST END]` at once; a bench test or list loses its KEEP lease, so the executor
brakes within 800 ms (`[TEST END]` ends only held drives); a Continue/Reverse waiting for the
release is dropped. A list stopped this way keeps its place, as after closing the window.

## Remote nudge (owner, 2026-11-18)

Each phone arrow is one held drive the Pi lets go of by itself `PHONE_NUDGE_S` (0.6 s) after
sending the start: the same `[TEST:id:W|S|A|D:speed:0]`, KEEPs every 0.12 s and `[TEST END]` the
window uses, so no firmware change. Nominal travel is 2351 or 4702 steps/s × 0.6 s ÷ 78.4
steps/cm ≈ 18 or 36 cm, less with the acceleration ramp, plus braking; turns spin in place. The
time is measured on the Pi from the send, so a slow phone or internet link delays a nudge but
can never lengthen it; if the Pi stops, the executor's 500 ms lease ends it. `request_hold`
refuses a pulse outside (0, 1.2 s]. No sensor gates, as for the window's arrows (owner,
2026-11-16). Stop and EMERGENCY STOP end a nudge early, and Stop may overtake an unclaimed
nudge, which then never runs.

## Mirror additions (telemetry only)

`devices/<id>/operator` gains `developerMessage`, `developerBlocked` and `developerRun`
`{known, place, running, waitingRelease, mode, message, steps: "F4000,L980", step, done,
speed, accel, runningStep}`. While a list moves only `runningStep` changes (`[RUN AT]` arrives
four times a second and would otherwise rewrite the mirror as often). None of it grants motion.

## Database rules

`Garby_MobileApp/firebase/manual-disposal.rules.example.json` now lists all 32 actions and the
optional `sensors` (0..7, `DEV_TEST_*` only), `fast` (boolean, nudges only) and `program`,
`speed`, `accel` (Start run only, in the window's ranges). The rules were
checked against the app enum and `cloud_control.py` by script; they were **not published or
tested against the live database**. Publishing them is the owner's console action. Until then,
if the earlier example rules are live, side read-only and Developer requests are refused and
the ordinary controls work again; if no field-checking rules are live, everything works.

## Checks and limits

See [`artifacts/phone-controls-rundown-2026-11-18/VERIFICATION.md`](../artifacts/phone-controls-rundown-2026-11-18/VERIFICATION.md).
Software only: nothing was run on the robot or a phone, no database was changed, and no
firmware was built or uploaded. Nudge distances are nominal from step counts, which cannot show
slip or real travel.

## Deployment

- **MCU upload requirement for this change: None.** Nudges and RunStart need boards already on
  `TURNSCALE-20261116` or newer (remote control and 2351+ steps/s); earlier firmware still
  awaiting upload is separate (see `RELEASE_NOTES.md`).
- Pi: the entire `RasPi/` directory, `PHONEDEV-20261118`.
- App: a new APK from the app repository. Older Pi software shows "update GARBY's robot
  software" on the RunStart group instead of guessing.
- Firebase: publish the updated example rules, if the rules are used.
