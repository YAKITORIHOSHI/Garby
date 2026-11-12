# Applied direction and GUI integration fixes — 2026-11-09 follow-up

All changes below are saved in the production workspace.

## Change summary

- **Forward direction already implemented and independently verified:**
  `MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.h:96-98` selects motor 1 DIR LOW,
  motor 2 DIR HIGH for increasing counts, with build ID `FRONT-20261109`.
  Production `setup()` applies both constants. The review preserves this pair.
- **Circular view already implemented and independently reviewed:**
  `RasPi/monitor_ui.py` draws up to 360 measured scan bins from `lidar_map.py`,
  with range rings and the eight-summary toggle. The review preserves its layout.
- **Newly applied freshness fix:** `RasPi/final_w_serial.py` supplies acquisition
  time independently of optional drawing geometry; `monitor_ui.py` retains that
  time even if the plot builder fails. Explicit malformed timestamps stay unavailable.
- **Newly applied missing-reading fix:** the runtime omits a summary value when
  its current sector lacks enough valid rays. Historical smoothing remains intact,
  but an old distance cannot acquire a new scan's timestamp.
- **Newly applied scan-error fix:** processing errors clear health, current-sector
  validity and old evidence, then queue STOP through the existing watchdog.
  A new valid scan restores processing; the same unavailable interval does not
  generate repeated watchdog STOP packets.

Production edits in this follow-up are limited to `final_w_serial.py` and
`monitor_ui.py`, with corresponding regression tests. Firmware, the map helper
and the preview are unchanged by the review.

## Optimizations and workarounds preserved

The GUI retains its 150 ms poll, bounded queue drain, cached widget setters,
fixed Canvas item pool, hidden-view no-op, snapshot/render cache, tabs, controls,
summary toggle and layout. No new display dependency or background worker was added.

The motor route implementation and calibrated route file remain byte-identical.
Turn targets, nudge-wheel selection, centering reference/timing, readiness,
emergency-stop latch, front/rear threshold bypasses, sonar-off operation, nearest
clearance and BLE/UART protocols are preserved. A map-generation error remains
display-only; a scan-processing error invalidates motion evidence.

The internal display frame now has an independent seventh acquisition-time field.
Existing five/six-field callers remain accepted. This is a Pi-local interface;
there is no firmware protocol change. The production launcher resolves its own
`RasPi/final_w_serial.py`, and the mobile compatibility launcher delegates there.

## Verification summary

- Full Pi suite: **359 cases, 331 passed, 28 skipped** because no Tk display is accessible.
- Firmware behavioral suite: **826 PASS assertions**, no failed assertions.
- Source sanity: **passed** for 49 C/C++ files and 144 JSON files at execution time.
- Fresh independent motor test: **26 checks passed**. The archived pre-correction
  source still fails under the same updated forward-wiring fixture.
- Four scan regressions fail against the exact prior runtime source, verified by
  its prior recorded hash. GUI regressions also fail before the age fix and pass after it.
- Current executor source/dependency and binary hashes match its successful
  compile from the initial implementation; no repeated firmware build was needed.

The detailed commands, failure logs, applied patches and hashes are in the
[follow-up validation report](../artifacts/front-direction-circular-review-2026-11-09/validation.md).
The earlier [direction report](motor-direction-2026-11-09.md) and
[circular-view report](circular-lidar-2026-11-09.md) remain historical implementation evidence.

No firmware was uploaded, robot operated, service restarted or database/SMS
action performed. Real Tk layout and physical wheel, LiDAR/gyro orientation,
turn/nudge direction, straightness and braking remain hardware acceptance items.
