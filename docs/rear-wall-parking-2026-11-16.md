# Rear-wall reverse parking — 2026-11-16

The owner requested alignment from the LiDAR wall behind GARBY before reversing,
for both outbound parking and return parking. The owner clarified that the return
still drives forward through the hallway and reverses only into the station.
Controls and developer tests were reported working and were left unchanged.
Nudge optimization was optional; the independent model review supports keeping
the current tuning.

This supersedes the alignment portion of `reverse-parking-2026-11-16.md`. Its
rear-clearance scopes, route travel counts, station masking and physical limits
continue to apply.

## Resulting parking behavior

1. The final approach turn faces away from the parking spot at the same place as
   before. On both routes it now skips side-wall alignment: the parking wall is
   behind the robot, and a hallway side fit must not choose this direction.
2. Before any backward travel, GARBY stands still and requires two independent
   rear-wall acquisitions, no older than 900 ms, acquired after it stopped.
   Their normals must agree within 1 degree and distances within 3 cm. Repeated
   acquisition IDs cannot provide a second confirmation or renew freshness.
3. The measured rear-wall normal determines alignment. If its error exceeds
   1 degree but is no more than 8 degrees, GARBY makes a slow correction under
   full spin-sweep, floor-sonar, emergency-stop, path and MPU supervision.
   It measures another rear pair **after** any turn-clearance wait and after
   the correction. At most two completed corrections are attempted; a larger
   or still-unconfirmed angle holds stationary and enters low power after
   10 seconds instead of repeatedly spinning.
4. Fresh `GO:BACK` remains mandatory. After its clearance wait another rear pair
   is required before wheel commands. The final legs retain 11100 outbound
   steps and 7500 return steps at `BACKING_SPEED` (2000 steps/s). Both wheels
   receive the same remaining backward travel; in-place corrections do not
   count toward mean route distance.
5. Rear-fit loss, stale evidence or more than 2 degrees of rear-relative drift
   during the approach brakes the robot. It leaves the backing scope before
   any correction, checks the rear again, then resumes only the remaining mean
   travel under fresh rear permission. The existing route checkpoint handles
   an operator stop and continued parking.
6. There is no MPU-only square-up after the final parking distance. The MPU
   still supervises every motion and measures corrections. Its old route
   reference and side-wall heading do not choose the parking direction.

No 180-degree spin beside either parking spot was added. No calibrated route
counts, motor DIR levels, wheel-side assumptions, general clearance thresholds,
operator commands or developer-test overrides were changed.

## Rear measurement and transport

`RasPi/rear_wall.py` fits the original robot-frame scan **before** station walls
are removed. It reuses the existing robust line fit with rotated coordinate axes:
rear-facing distance, a minimum 80 cm span (or chassis width + 20 cm), support
across the rear centreline and both halves, and no central gap over 20 cm.
A short panel, one-sided surface, front wall or ordinary side wall cannot supply
the required rear-plane evidence. A station corner step may hide an outer end
without invalidating a long visible line crossing the centreline. Orientation
evidence does not claim that the corners or path are clear.

Mount yaw and left/right reflection are shared with the map and clearance
processing. The side-wall mounting-angle learner is not applied to rear fits.
The new fields are `RD` (perpendicular centimetres) and `RH` (degrees, chassis
left positive). Both are omitted without a usable fit.

The bridge forwards one unacknowledged measurement for each accepted independent
S acquisition:

```text
D,<acquisition>,<age-ms>,<valid-0-or-1>,<rear-mm>,<rear-heading-cdeg>*<xor>
```

An unavailable fit explicitly withdraws the previous one. Source age is refreshed
at the Pi GATT write, then includes bridge queue time and the existing 30 ms transit
estimate; the executor matches it against gyro history. This is an age estimate,
not a measured end-to-end transport deadline. D lines never grant GO, renew path
permission, satisfy readiness, or consume the bridge's command-ACK budget. A
confirmed new Pi/BLE session clears only rear acquisition state, allowing a
restarted Pi to begin at A=1 without resetting continuous MPU heading.

`PARK:WAIT_REAR_WALL`, `PARK:WALL_UNSTABLE`, `PARK:ALIGNING` and
`PARK:ALIGNMENT_UNCONFIRMED` explain parking waits in the existing motion status.
Parking reports are also retained in headless logs.

## Review and software verification

Before implementation, new tests failed against the unchanged source: backward
travel did not require a rear-wall fit, repeated rear acquisitions did not hold it,
and the Pi had no rear alignment fields. Logs are in
`verification/2026-11-16-rear-wall-parking/`.

Independent review found and verified fixes for restarted acquisition numbering,
a stale correction target after a long clearance wait, and low-power behavior for
an unconfirmed angle. In the delayed-clearance model, a 120-second hold with
0.1 degree/s gyro bias previously produced a 15.78-degree physical correction;
after remeasurement it corrected by 3.71 degrees and left 0.29 degree rear error.
These are simulated values, not floor measurements.

Final checks, commands, timestamps and exit codes are recorded in
[`checks.json`](verification/2026-11-16-rear-wall-parking/checks.json):

| Check | Result |
| --- | --- |
| Pi unittest discovery | 634 tests, 41 skips, exit 0 |
| Actual firmware host tests | 12 executables, 825 passing assertions, exit 0 |
| Source sanity | PASS, exit 0 |
| ESP32-S3 bridge target | PASS, 600006 bytes program, 38664 bytes globals |
| ESP32 executor target | PASS, 431854 bytes program, 29712 bytes globals |

Tk display tests were skipped: Tkinter imports, but `localhost:11.0` is not a
working display; Xvfb is unavailable. The optional separate Android checkout
interoperability check was not enabled. No Android source or APK was copied.

The ESP32 core was 3.3.11. Builds used external build/cache directories and the
documented production configurations: bridge
`esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi`; executor
`esp32:esp32:esp32:FlashSize=4M,PSRAM=disabled`. Saved flasher state selected no
board and reported no connection; that state was not treated as current USB
evidence or used to invent a port.

Tests cover the actual Pi producer, bridge parser and UART consumer; malformed,
missing, duplicate, stale and disagreeing rear evidence; rear loss during motion;
unchanged GO dominance; Pi-session restart; low-power holds; continued parking;
both complete routes; the ordinary return and Reverse-to-start behavior. All
15 recorded station scans retain a stable rear fit even after station masking.
Original and final source hashes, exact edits, binary hashes and review evidence
are linked in the [change manifest](verification/2026-11-16-rear-wall-parking/change-manifest.md).

## Optional nudging review

The independent review exercised current production algorithms in the existing
hallway model, with no tuning changes. A centered 15 m run had a maximum 2.16 cm
deviation and no centring commands. A 3-degree initial heading error, 0.05 degree/s
gyro bias and 0.4% wheel-scale mismatch finished 2.54 cm off centre; a 4 m wall
gap finished 0.81 cm off centre. Pre-run centring reduced a 22 cm offset to
0.1 cm. Existing permission-veto, wall-loss, slow-zone and wrong-way checks passed.
These results support retaining the intentional 5 cm engage / 2 cm release
behavior; reducing it would introduce steering activity without physical evidence.
See [`nudge-model.log`](verification/2026-11-16-rear-wall-parking/nudge-model.log).

## Deployment and remaining physical checks

**MCU upload requirement for this change: Both.** Upload the complete
`BLE_Receiver-Final` and `MAIN_EXECUTOR_CODE` sketches and deploy the entire
matching `RasPi/` directory. All identify as **REARPARK-20261116**. Both firmware
targets changed because the bridge produces D evidence and the executor consumes
and enforces it; Pi deployment supplies its rear fit. An older Pi or bridge
cannot provide the new parking evidence. Ordinary forward travel has no new
rear-wall requirement.

Earlier source images such as `PARK-20261116` and its pending developer-turn
fixes are included in this build; they do not need separate older uploads.
Installed firmware identity is unknown. This session performed no upload,
robot command, service restart, cloud mutation or physical acceptance.

Compiled binary/ELF evidence is kept outside the repository at
`~/.local/share/garby-builds/rear-wall-parking-2026-11-16/`, with exact hashes in
[`binary-hashes.json`](verification/2026-11-16-rear-wall-parking/binary-hashes.json).
The complete sketch, rather than an individual header, is the upload unit.

Before floor use:

1. Verify actual mounting yaw/reflection with the chassis physically square to
   a known flat rear wall. A rear line alone cannot distinguish mounting error
   from chassis error. Neither its physical alignment nor its installation was
   established by the recorded-data/model checks.
2. Measure body outline, LiDAR offset and the available rear gap. **Outbound
   parking still obeys the existing rear box**: with the assumed 30 cm rear edge
   and 35 cm stop, the wall must remain at least 65 cm behind the LiDAR. A 11100
   step (~142 cm) approach therefore needs an initial distance around 207 cm or
   more, plus real stopping margin. If the configured gap is insufficient, it
   holds rather than silently ignoring that wall. The return's station wall is
   omitted only under its existing fresh station-step scope. No such omission
   was added at the outbound destination. The model's rear plane is orientation
   evidence only; its R/C path fixture is not physical clearance validation.
3. With supervision, check small correction direction and full spin clearance,
   rear wall loss/recovery, an obstacle hold, emergency stop/continued parking,
   and both final approaches before a complete route. Wheel steps remain open
   loop and cannot detect slip. If drift occurs close to the wall without a
   clear spin sweep, the robot holds.
4. Keep low objects out of both parking approaches. The forward sonar faces
   away while reversing; nothing below the rear LiDAR plane is observed. Actual
   stopping distance, final gap, physical centring and battery savings remain
   unmeasured.
