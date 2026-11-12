# BLE discovery and S3 standalone supply — 2026-11-09

## Findings

The owner's 04:11–04:12 Pi log fails during discovery, before any BLE connection,
GATT subscription or readiness write. LiDAR starts normally. The owner then
reports an unexpected S3 NeoPixel, a battery/buck feed through the Executor,
and successful BLE operation with the S3 on desktop USB alone.

That comparison narrows the failure to standalone power/startup conditions; it
does not measure a voltage fault or identify a defective component. USB also
changes grounding/reset conditions. The same daisy-chained power setup had
recorded S3 brownout resets on November 4; those remain historical evidence,
not a measured diagnosis of this run. The current bridge has no application
NeoPixel control, so the pixel is not a GARBY status indicator.

The Pi discovery predicate and installed Bleak 3.0.2 APIs agree with the S3's
advertised UUID/name. Unchanged production S3 advertising recovery passed fault
injection with repeated start failures and missing MPU. No firmware change is
justified by the available evidence. Live radio access from the coding session
is blocked by D-Bus permissions; the USB comparison is owner-reported.

## Proposed physical repair and acceptance

With power off and the robot secured, verify the buck's input rating against
the battery's maximum charged voltage, then verify its regulated output and
the exact S3 board's documented input. After those checks, give the S3 its own
short supply/ground branch from the buck, bypassing the intermediate Executor
power connection. Keep a common ground for the controller UART.

Use the documented 5 V/VIN input only if specified for the actual board; never
apply 5 V to 3V3. Disconnect USB for the battery test and use one board power
input at a time. Measure voltage at the S3 input and its 3.3 V rail during startup,
and capture reset/advertising messages. Check EN/reset timing if the rails are
stable but cold startup still fails. Do not choose RGB GPIOs, disable brownout,
or alter boot circuitry from the LED symptom alone.

Relevant primary guidance: [board power options](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html#power-supply-options),
[ESP32-S3 supply and reset requirements](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32s3/schematic-checklist.html).
These references do not establish the identity or wiring of the owner's board.

Acceptance is successful GARBY discovery/readiness on battery alone, stable
rails/reset logs, and normal fail-closed STOP behavior. Neither the proposed
wiring change nor its acceptance was performed by this software investigation.

## Applied software changes

The previous discovery timeout incorrectly asserted that the robot was not
advertising. It now reports what the existing scan actually observed:

- No advertisement callbacks received.
- Advertisement callbacks received, but no configured identity matched.
- GARBY-identifying candidates rejected by a strict pinned address.

Diagnostics include adapter selection and pinned address, a saturated event
count and at most three bounded GARBY candidate addresses. They retain no
unrelated device identities. This adds no scan and changes no matching, timing,
retry, queue, readiness or motion gates. The update improves diagnosis; it is
not a software remedy for unstable board power.

Edited production: `RasPi/ble_support.py`, `final_w_serial.py`, `check_ble.py`.
Corresponding BLE/probe tests are updated. Exact files, patches, source hashes
and checks are in `artifacts/ble-establish-2026-11-09/validation.md`.

**MCU upload: None for this change.** Deploy the complete `RasPi/` directory for
the improved diagnostics. No S3/Executor firmware, LED pins or power protection
was changed; no firmware upload, robot command or service restart occurred.
