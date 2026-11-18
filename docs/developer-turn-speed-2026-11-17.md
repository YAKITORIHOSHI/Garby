# Developer turn correction, speed and map layout — 2026-11-17

The owner reported that the developer 90° turn went too far, then supplied an approximate
100° physical angle for the MPU's reported 90°. The same request sets the developer minimum
speed to 30 cm/s, moves map notices/metadata to the bin-sensor sidebar, and renames the
main window's button **Developer**. Work began November 16 and completed November 17.

## MPU correction and turn completion

The saved Pi log's last two right-turn tests reported MPU changes of -90.13° and -90.16°
(`TURNTEST-20261116`). Those are sensor reports, not independent physical measurements.
Using the owner's estimate, the correction is:

```text
new scale = old scale × physical angle / MPU angle
          = 1 × 100 / 90 = 1.111111…
```

`BLE_Receiver-Final/MpuHeading.h` multiplies both yaw rate and its integrated heading by
that factor. A desired 90° turn therefore ends near the old raw 81° reading. The executor
still asks for 90° and consumes the corrected MPU M stream; it applies no second factor.
This corrects the bridge's continuous heading for routes and tests alike. Motor DIR levels,
route travel/angle commands and the initial left-wheel guess are retained. Steps per degree
continue to be learned from clean MPU turns; step counts are not angle evidence.

The scale is **provisional**, based on an estimate. It cannot distinguish sensor scale
error from coasting, wheel slip or mounting error, and needs measured floor acceptance in
both directions. No software test establishes an exact physical 90°. A later measured
calibration uses the same formula with the current scale; do not replace degree targets
or apply this factor in both controllers. Runtime bias-change limits retain their corrected
0.2°/s per-window and 1°/s-from-boot budgets.

A regression reproduced another completion defect: the old fixed 250 ms pause could report
DONE while fresh MPU samples still showed rotation. Developer measured turns now keep
observing after the immediate wheel-queue abort. They require at least 200 ms of fresh
samples with rate at most 1°/s and heading change at most 0.2°, with a minimum 250 ms overall
pause. Missing MPU, a new heading reference, a sensor/protocol hold, lost lease or emergency
stop cannot pass this check. Rotation that does not settle within 1.5 s reports
`TURN_UNSETTLED`; final error outside 1° after the allowed correction passes reports
`TURN_ACCURACY`. Both messages are shown plainly on the Pi.

The approach profile still brakes before the endpoint. Its developer updates are limited
to 50 Hz so the higher setting does not rebuild both stepper ramps every millisecond;
heading, endpoint, stop and lease checks continue each loop. Profile quantization preserves
the exact non-round 2351 Hz peak instead of rounding it below the requested speed. Route
hold/settling behavior retains its existing scope.

## Developer speed settings

Nominal geometry is 1600 steps per revolution and 65 mm wheel diameter:

```text
steps/cm = 1600 / (π × 6.5) = 78.3532…
30 cm/s  = 2350.596… steps/s → round up to 2351 steps/s
```

- Finite forward, backward and individual-motor tests use the 2351 steps/s setting.
  MPU turns approach at that setting and brake toward their endpoint.
- Remote Slow is 2351 and Fast 4702 steps/s, nominally 30/60 cm/s wheel speed. Both bridge
  and executor reject held-drive settings outside 2351..4702.
- RunStart accepts 2351..8000 steps/s and defaults to 2351. Pi, bridge and executor enforce
  the lower bound; acceleration stays 100..20000 steps/s².
- These are speed **settings**. Acceleration, braking, short moves and the initial
  motor-side confirmation may be slower; wheel geometry and physical speed are unmeasured.
  RunStart turns remain step-count turns, not MPU angles.
- Ordinary finite tests retain eight seconds; exactly 90° retains thirty. The 800 ms
  finite/RunStart and 500 ms held-drive leases, selected gates, request identity/session
  handling and operator stop retain their scope. A short move can finish within its
  initial lease without an additional KEEP; ongoing movement still requires renewal.

## Map and sidebar

The map canvas occupies the surroundings panel. Its former top toolbar, lower caption,
hit/nearest/mount text, threshold status and the bottom geometry/zone line move into the
right sidebar with the bin sensors. Robot state and motion notices also appear there on
the map tab. The sidebar scrolls at small screen sizes and has a fixed width, so long text
does not shrink the map or enlarge the window. The eight sector labels remain on the map.

The top button reads **Developer**. The separate testing window retains its descriptive
title and emergency stop. Other main-window tabs keep state/hold notices in their header,
where the map sidebar is hidden. Emergency stop remains visible throughout.

## Build and deployment

Current build: **TURNSCALE-20261116**.

**MCU upload requirement: Both** — upload complete `BLE_Receiver-Final` and
`MAIN_EXECUTOR_CODE` sketches. The bridge owns MPU scale and speed request validation;
the executor owns movement speed, approach updates and settled-turn completion. Deploy the
**entire `RasPi/` directory** for matching settings, messages and layout. This incorporates
earlier source changes; no intermediate image needs a separate upload. A previously
pending image is not evidence that any current image is installed.

No hardware upload, robot command, Pi deployment/service restart, database write or SMS
was performed in this task. Installed firmware and physical acceptance remain unverified.
Saved GUI board selections were read without opening hardware. Builds use external
temporary paths with no upload/port flags. Exact checks, limits, board configuration and
source/binary hashes are recorded in
[`VERIFICATION.md`](../artifacts/developer-turn-speed-2026-11-16/VERIFICATION.md).
