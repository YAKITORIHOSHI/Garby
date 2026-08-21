# GARBY Full Project Investigation and Repair Report

**Audit/rework date:** 2026-08-21  
**Input:** 30 files from the supplied `GARBY.zip`  
**Release principle:** treat Raspberry Pi + BLE bridge + main ESP32 as one coordinated control system; fail closed on uncertain motion permission.

## Executive findings

The startup/communication problem was caused by several interacting defects rather than one broken wire/protocol statement:

1. **Production Pi could fail before starting:** `final_w_serial.py` imported `bridge_core`, but the ZIP did not contain `bridge_core.py`; only Python 3.12/3.14 cache files existed.
2. **Main MCU handshake was coupled to slow cellular startup:** modem/network work could delay control readiness by many seconds before `[MCU READY]`.
3. **Clearance timing was internally inconsistent:** bridge and main MCU require repeated fresh confirmations, but the old motion gate waited only 400 ms. Normal 250 ms Pi status cadence could therefore time out before enough confirmations arrived.
4. **Pi could temporarily advertise CLEAR before its first LiDAR scan:** startup grace was incorrectly applied to safety state rather than only driver supervision.
5. **Three ROS subscriptions processed the same LiDAR stream:** duplicated callbacks distorted histories/confirmation behavior and wasted CPU.
6. **Fresh but unusable LiDAR data could be interpreted as clear:** missing safety-sector evidence was not explicitly represented as stale/unavailable.
7. **Sonar no-echo could clear a prior local STOP:** timeout was recorded as `999` and then counted as a clear sample.
8. **BLE recovery had a stuck-task failure mode:** an unexpected connect coroutine exception could leave connection state wedged.
9. **Bridge/main parsers were more permissive than appropriate for a motion protocol:** partial numeric conversion and substring path extraction accepted malformed values too easily.
10. **Documentation described a different system:** old watchdogs, servo angles, nudge constants, LiDAR orientation, paths, and validation claims did not match the actual code.

All software-fixable items above were repaired in the coordinated release. Physical route direction, LiDAR mounting orientation, stopping distance, power integrity, and target-board compilation remain hardware/toolchain validation items rather than safe guesses.

## File-by-file investigation of the original 30 files

| # | Original file | Finding | Release action |
| ---: | --- | --- | --- |
| 1 | `.github/agents/garby-thesis-assistant.agent.md` | Stale protocol/tuning constants and old architecture claims. | **Rewritten** to current 2026-08-21 protocol/constants and hardware-verification limits. |
| 2 | `.github/skills/garby-robot-maintainer/SKILL.md` | Maintenance workflow is structurally useful and safety-oriented; no runtime payload. | **Kept/reviewed**; its architecture reference is now synchronized to root docs. |
| 3 | `.github/skills/garby-robot-maintainer/agents/README_FIRST.md` | Referenced nonexistent `RaspberryPi/`, `docs/`, patch-note, and hardware-test paths. | **Rewritten** with actual repository layout and deployment steps. |
| 4 | `.github/skills/garby-robot-maintainer/agents/garby-thesis-assistant.agent.md` | Duplicate stale agent content. | **Rewritten/synchronized** with root agent. |
| 5 | `.github/skills/garby-robot-maintainer/agents/openai.yaml` | Metadata only; no contradictory runtime constants. | **Reviewed/kept**. |
| 6 | `.github/skills/garby-robot-maintainer/references/SAFETY_REVIEW_CHECKLIST.md` | General checklist useful but missing newly identified startup/sonar/import regressions. | **Updated** with release-specific checks. |
| 7 | `.github/skills/garby-robot-maintainer/references/SYSTEM_ARCHITECTURE.md` | Major mismatch: old 1200/1500 ms watchdogs, old 30°/130° servo endpoints, old nudge values, old paths/orientation. | **Replaced** with synchronized copy of root architecture. |
| 8 | `.github/skills/garby-robot-maintainer/scripts/audit_project.py` | Original file selection could choose `final_w_serial-simulator.py` instead of production, producing misleading Pi audit results. | **Reworked** to prefer exact production paths and expanded with current communication/safety checks. |
| 9 | `.vscode/c_cpp_properties.json` | Editor-only; generic include configuration, no runtime effect. | **Reviewed/kept**. |
| 10 | `.vscode/launch.json` | Contained hardcoded stale Windows thesis-directory/program paths. | **Neutralized** to portable empty debug configurations. |
| 11 | `.vscode/settings.json` | Editor-only compiler-runner settings; no runtime effect. | **Reviewed/kept**. |
| 12 | `BLE_Receiver-Final/BLE_Receiver-Final.ino` | Core transport needed stronger boot epoch, ACK accounting, exact path parsing, overflow rejection, legacy validation. | **Modified extensively**; remains sole navigation parser/STOP latch/nudge authority. |
| 13 | `DEPLOYMENT_AND_ACCEPTANCE.md` | Described old timings/orientation and nonexistent files/apps. | **Completely rewritten** for this release. |
| 14 | `FIREBASE-DATABASE/LATEST_FIREBASE_REAL-TIME_DATABASE.json` | Valid JSON; sentinel structure (`999`, `-1`) consistent with current telemetry model. No motion authority. | **Reviewed/kept**. |
| 15 | `NAPHTALI_CODE_V2/NAPHTALI_CODE_V2.cpp` | Motion-gate timing mismatch, permissive nudge parse, sonar timeout false-clear, telemetry sentinel semantics, blocking modem architecture in prior behavior. | **Modified extensively**: strict parser, valid-sonar flag, 900 ms repeated-request gate, asynchronous modem ownership, controlled safety behavior. |
| 16 | `NAPHTALI_CODE_V2/NAPHTALI_CODE_V2.h` | Constants/types needed to match repaired controller behavior. | **Updated** with motion-gate timing and unavailable sensor enum states; existing safety caps reviewed. |
| 17 | `NAPHTALI_CODE_V2/NAPHTALI_CODE_V2.ino` | Control readiness was previously delayed by modem initialization. | **Updated** so `[MCU READY]` precedes background modem initialization; startup remains fail-closed if BLE is absent. |
| 18 | `NAPHTALI_CODE_V2/pointsRun.ino` | Route is syntactically coherent and gated, but physical direction cannot be proven from signs/function names. | **Reviewed/kept**; mandatory wheels-lifted route verification documented. |
| 19 | `RasPi/__pycache__/bridge_core.cpython-312.pyc` | Generated cache, not portable source; dangerous because actual source was missing. | **Removed from release**. |
| 20 | `RasPi/__pycache__/bridge_core.cpython-314.pyc` | Same issue. | **Removed from release**. |
| 21 | `RasPi/__pycache__/final_w_serial-simulator.cpython-312.pyc` | Generated cache. | **Removed from release**. |
| 22 | `RasPi/__pycache__/final_w_serial-simulator.cpython-314.pyc` | Generated cache. | **Removed from release**. |
| 23 | `RasPi/__pycache__/final_w_serial.cpython-312.pyc` | Generated cache. | **Removed from release**. |
| 24 | `RasPi/__pycache__/final_w_serial.cpython-314.pyc` | Generated cache. | **Removed from release**. |
| 25 | `RasPi/__pycache__/test_bridge_core.cpython-312.pyc` | Generated cache; test source was absent. | **Removed from release**. |
| 26 | `RasPi/__pycache__/test_bridge_core.cpython-314.pyc` | Generated cache. | **Removed from release**. |
| 27 | `RasPi/final_w_serial-simulator.py` | Legacy transport producer was inconsistent with sequenced production `P:`/`S:` protocol. | **Updated** to sequenced protocol and serialized writes; remains non-production. |
| 28 | `RasPi/final_w_serial.py` | Missing helper dependency, unsafe first-scan grace, duplicate subscriptions, incomplete-scan ambiguity, BLE recovery weaknesses, Firebase import coupling. | **Modified extensively**; see detailed Pi fixes below. |
| 29 | `SYSTEM_ARCHITECTURE.md` | Did not reflect current source. | **Completely rewritten** as authoritative release architecture. |
| 30 | `VALIDATION_RESULTS.md` | Contained old date and build claims that do not validate the current modified source. | **Completely rewritten** to report only current checks and explicit limitations. |

## New files added to make the release self-contained

| File | Purpose |
| --- | --- |
| `RasPi/bridge_core.py` | Reconstructed required production helper source: sensor state, backoff/coalescing, LiDAR partition/representation, health helpers. |
| `RasPi/test_bridge_core.py` | Host regression tests for helper behavior. |
| `RasPi/README.md` | Actual Pi deployment/dependency/environment instructions. |
| `RasPi/requirements.txt` | Pip-installable Pi dependencies (ROS/YDLidar remain system/ROS dependencies). |
| `RasPi/garby-bridge.service.example` | Optional headless systemd service template. |
| `tools/source_sanity.py` | Host structural/JSON sanity checker. |
| `SOURCE_SANITY_RESULTS.txt` | Latest structural check output. |
| `AUDIT_RESULTS.json` | Latest machine-readable coordinated audit. |
| `AUDIT_REPORT.md` | This exhaustive investigation report. |

## Detailed coordinated repairs

### Raspberry Pi

- Restored missing `bridge_core.py` source so the program is no longer dependent on cache bytecode from a particular Python version.
- Made first-scan state fail closed: no first scan = stale/unknown.
- Reduced production ROS subscription to exactly one `/scan` sensor-data subscription.
- Added front/back scan-quality validity gates; incomplete safety sectors transmit `S`.
- Treat ROS LaserScan positive infinity as a valid no-return sample at `range_max`.
- Use nearest valid LiDAR return for collision safety and median wall geometry for steering.
- Added configurable `GARBY_LIDAR_YAW_OFFSET_DEG` without guessing physical mounting.
- Added guarded BLE connect task and safe transient-write recovery.
- Preserved queue coalescing and acknowledged safety/control writes.
- Made Firebase SDK absence non-fatal to the motion-safety runtime.

### BLE bridge

- Starts with `mcuReady=false`.
- Repeated `[MCU READY]` is a new MCU boot epoch and discards old path permission.
- Keeps path timeout at 850 ms and BLE silence/reconnect at 10 s.
- Validates decimal uint32 sequence without silent overflow.
- Parses path body as exact `F=`/`B=` tokens with no duplicate/unknown fields.
- Hardened legacy `PATH:` compatibility so arbitrary text cannot become CLEAR.
- Requires steering sequence to equal newest path sequence.
- Tracks one MCU ACK per queued command rather than letting one ACK erase the whole outstanding count.
- Continues STOP on disconnect; never starts automatic return.

### Main ESP32

- Sends `[MCU READY]` before Air780E initialization; cellular startup is a background FreeRTOS task.
- Motion gate increased to 900 ms and actively re-requests status through the existing 80 ms throttle.
- Turns now also require fresh path state at launch.
- Strict digit/overflow parsing for `N:` duration/intensity.
- No-echo sonar samples are explicitly invalid and cannot increment clear counters or release an obstacle latch.
- `US=999` and gas `-1` are represented as UNAVAILABLE telemetry states.
- Modem/SMS ownership prevents the main loop from stealing AT responses while the background worker is active.
- Existing controlled-deceleration safety stop remains; hard force-stop remains a fallback only.

## Follow-up repairs in this workspace

- Main route primitives now return success/failure through `safeMoveDistance()`, `safeTurnLeft()`, and `safeTurnRight()`.
- `runStart()` and `returnToPointB()` stop at the first incomplete segment, so a load trigger cannot be followed by a false `[OUTBOUND COMPLETE]` when no motor command was accepted.
- The main state machine keeps RUNNING/RETURNING active until the route reports success; explicit reset behavior remains unchanged.
- A confirmed nudge reversal restores the base wheel speeds and applies the opposite tap immediately instead of discarding it.
- C++ pointer/task null uses were modernized to `nullptr` in the active controller source.
- Removed the deprecated `NimBLEService::start()` no-op; NimBLE-Arduino 2.5.x starts services as part of advertising/server startup.

## Remaining hardware-only uncertainties

These were intentionally **not guessed** in code:

- physical LiDAR yaw relative to chassis FRONT;
- physical forward/reverse meaning of the full `pointsRun.ino` route sequence;
- motor polarity and wheel trim;
- actual stop distance with maximum payload/low battery/low traction;
- servo mechanical limits/power integrity on the installed hardware;
- ultrasonic voltage level/accuracy;
- BLE range/interference and UART electrical integrity;
- hardwired emergency-stop behavior.

Follow `DEPLOYMENT_AND_ACCEPTANCE.md`. The software release is designed to fail closed while these are being verified, but source review alone cannot certify human-safe operation.
