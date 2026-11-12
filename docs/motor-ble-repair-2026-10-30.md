# Motor reference comparison and BLE repair — 2026-10-30

The supplied `WebConnect-GarbyController` reference was inspected without
modification. Current motor STEP/DIR pins, polarity, 5600 Hz straight speed and
4800 steps/s² acceleration already matched it. Two turn functions were reversed
relative to that working reference; they now match its left/right wheel signs.
Route distances and step counts were preserved. Setup reports actual per-motor
initialization success or failure.

## Why the scale trigger can leave the wheels stopped

The production log records the executor entering `OUTBOUND`, then reporting a
LiDAR obstacle hold. This shows a route request reaching the executor; it does
not show an electrical motor failure. A read-only live `/scan` capture found:

| Sector | Nearest reading | Configured limit |
| --- | --- | --- |
| Front | 62.6 cm | 95 cm |
| Front left | 32.1 cm | 95 cm |
| Front right | 55.1 cm | 95 cm |
| Rear left | 33.2 cm | 35 cm |

The capture used the running yaw setting and the production partition/nearest
helpers, with fresh source timestamps. The live GUI bypass settings were not
inspected, so these are readings before any operator bypass. Left/right were
outside their 22 cm limit. Clear these returns or verify physical LiDAR
orientation before expecting the automatic route to move.

The web controller directly commands motors without the autonomous controller's
LiDAR/sonar/MPU gates. Its ability to move proves useful motor setup but does not
establish that autonomous sensing grants clearance. The new integration test
feeds two fresh load samples through the actual executor loop, verifies no wheel
progress under a blocked path, then completes the full outbound route after
fresh path and sonar clearance. The existing 4 kg trigger uses the corrected
load reading; the existing 0.010 kg correction and calibration were preserved.

## BLE corrections

- Fragmented notifications used NimBLE 2.5.1's deferred characteristic-read
  overload. Immediate `setValue()` calls could replace earlier fragments before
  the host sent them, losing readiness/status/START/return outcomes. Explicit
  payload notifications now copy each fragment immediately and check actual
  enqueue rejection. A corrected deferred-read host model reproduced the old
  failure, then passed with the fix.
- TX power incorrectly passed an ESP-IDF enum with numeric value 12 to NimBLE's
  dBm API. It now passes the intended explicit 9 dBm.
- The Pi now caps long steering packets at the documented 250 ms cadence during
  bursts of MCU polls. Immediate path replies and urgent STOP handling remain;
  each emitted steering packet keeps its paired path sequence.
- Pi write errors now include exception class and packet kind, so empty
  `TimeoutError` messages are identifiable.
- Bridge diagnostics identify firmware `MOTORBLE-20261030`, reset classification,
  heap, connection timing, disconnect reason/duration and notification failures.
  The build/reset notices reach the Pi after readiness. Console work remains
  outside NimBLE callbacks.

No obstacle thresholds, freshness requirements or safety watchdogs were relaxed.
The historical log contains 18 disconnects but no Pi five-strike deliberate
disconnect. A passive HCI-event monitor observed 55 seconds without a drop, so
no HCI reason was captured. Power, radio and software reset causes remain
unproven; the new diagnostics distinguish them after the bridge is uploaded.

## Verification and deployment state

All **200 Python tests** and **191 firmware behavior checks** passed, including
real Tk tests through XLaunch. Structural and Android static checks passed.
Independent review found no important correctness issue. Target builds passed
for ESP32 Dev Module (4 MB, no PSRAM) and ESP32-S3 Dev Module (8 MB, no PSRAM),
using the previously documented pinned core/library versions. Android Gradle
and the absent legacy audit remain unverified.

Evidence and fresh binaries are in
[`artifacts/motor-ble-repair-2026-10-30/`](../artifacts/motor-ble-repair-2026-10-30/).
`firmware-builds.json` records exact commands, dependencies and source/binary
hashes. `verification.json` summarizes the final checks. The turn, notification
and poll-burst failures were reproduced before their corrections.

On the owner's request to rerun, the old Pi process was shut down and the updated
Pi application launched in XLaunch (`localhost:10.0`), process **17657**. The
monitor window was verified; the log confirms BLE readiness, Firebase
initialization and LiDAR scans. It is left running for the owner. No manual
Start was clicked by the agent.

**The new motor and BLE firmware has not been uploaded by this agent.** Changes
to sketches on the shared drive cannot alter firmware already running on an
ESP32. Upload the current `MAIN_EXECUTOR_CODE` and `BLE_Receiver-Final` projects
from the connected Windows flasher or connect their programming ports to the
Pi. Current saved Windows state lists COM22 and COM18 respectively; those are
saved selections, not verified Pi ports. The Pi's only USB serial device is the
LiDAR adapter and must not be flashed.

After uploading both, restart the bridge and verify `BRIDGE BUILD:MOTORBLE-20261030`
in its messages. Confirm corrected turn direction and complete the existing
wheels-lifted acceptance procedure before a floor route. Physical motor motion
and wireless reliability of the new firmware remain unverified until deployment.
The separate sensor-board UART is still unavailable on this Pi.
