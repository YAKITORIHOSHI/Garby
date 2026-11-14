# Confirmed manual disposal — 2026-11-13

Manual disposal sends GARBY from idle at base along its existing outbound route to the
collection point, for example after school when the bin should be emptied before tomorrow.
The operator confirms **Start manual disposal?** on the Pi monitor or **Yes, start disposal**
in the Android dashboard dialog. Cancel sends nothing. Bin level, gas and load readings may
be below thresholds or unavailable. At the collection point the operator empties the bin,
then requests return using the existing app action. No tipping/unloading actuator is added.

Automatic triggers retain their existing behavior (weight and gas; bin distance is telemetry).
Manual disposal changes route intent only. Live path, heading, nearest/side clearance, sonar
as configured, operator emergency stop, faults and readiness still gate actual movement.
No route calibration, DIR polarity, motor side, LiDAR orientation or threshold is changed.

## Source and protocol

The existing `[START:<decimal-id>]` action still starts the normal outbound route. The executor
reports `[DISPOSAL READY:1]` only during eligible idle status. The bridge filters that report
on BLE/Pi/MCU/UART readiness and operator stop, and rejects noncanonical START IDs before
UART relay. Pi eligibility requires that capability, fresh SYSTEM READY and IDLE (7.5 s),
current BLE and no operator stop. Android receives `manualDisposalReady` with
`manualDisposalUpdatedAt`, expiring after 15 s (5 s future-clock tolerance). Missing capability
on older firmware keeps the mobile button unavailable. The local monitor retains its
existing handshake gate, adds confirmation, and rechecks readiness after the dialog closes.
Interactive headless operation uses `s`, then `y` within 10 s; other confirmation keys cancel.

Android submits `devices/garby-bin-01/commands/dispose` with exactly:

```json
{
  "version": 1,
  "requestId": "0123456789abcdef0123456789abcdef",
  "requestedAt": "server timestamp (numeric on the server)",
  "requestedBy": "authenticated operator UID",
  "status": "pending"
}
```

The example timestamp above describes the value; the app writes `ServerValue.TIMESTAMP`,
not a string. The separate `commands/reset` return contract is unchanged.

| Stage | Owner and meaning |
| --- | --- |
| pending | App's server-confirmed intent; no controller acceptance yet |
| dispatching | Pi transaction claimed the exact request and recorded controllerRequestId/claimedAt before enqueueing one START |
| accepted | Matching executor START ACK; the route may still be held by safety checks |
| failed | Invalid/expired/unready request or matching controller rejection |
| unconfirmed | Controller result unknown; check GARBY before another deliberate request |

Android rechecks the gate after confirmation and against a fresh status read, then uses a
compare-and-set transaction that refuses a changed ID or unresolved command. Transaction
callbacks have no motion side effects and speculative local events are disabled. Firebase's
[transaction handler contract](https://firebase.google.com/docs/reference/android/com/google/firebase/database/Transaction.Handler)
and [runTransaction options](https://firebase.google.com/docs/reference/android/com/google/firebase/database/DatabaseReference#runTransaction(com.google.firebase.database.Transaction.Handler,boolean))
were consulted for this implementation. Timeout/cancellation purges queued client writes.

The dedicated Pi cloud worker polls every 2 s, accepts pending requests at most 30 s old and
no more than 5 s in the future, and durably claims before queueing. Slow claim I/O consumes
the existing 2-second queue deadline. The actual GATT boundary checks the current session,
state, queue age and emergency stop again. A claim timeout, process restart or BLE recovery
only permits `[START STATUS:<recorded-id>]`; START is never replayed. Missing results become
unconfirmed after 15 s from claim. Results match request ID, timestamp and requester, so a
late response cannot overwrite another command. Terminal records are immutable to the Pi
transition helper. An accepted cloud result is a dispatch receipt, never a completion receipt;
ordinary executor state reports arrival and any subsequent fault/emergency stop.

## Windows and Firebase preparation

Clone https://github.com/YAKITORIHOSHI/Garby-Mobile-App separately on Windows. Keep machine-specific SDK paths in its
own `local.properties`; do not reuse another computer's SDK path. Use the project's existing
JDK/SDK/Gradle versions from its README. From `Garby_MobileApp/` in PowerShell:

```powershell
python -B tools/static_quality_check.py
.\gradlew.bat :app:testDebugUnitTest :app:lintDebug :app:assembleBenchmark --console=plain
```

The new `DisposalLogicTest` covers unavailable/stale eligibility, offline and non-idle states,
missing bin readings, repeated submission, distinct acceptance/arrival and concurrent command
replacement. These Kotlin tests and the Android compile/lint were **not run here**, per the
requested Windows handoff. Output defaults to `app/build/outputs/apk/benchmark/`; a configured
`garby.buildRoot` changes the output root. The benchmark build is minified/non-debuggable and
development-signed for device checks. Use `:app:assembleRelease` with your normal release
signing process for distribution. VersionCode is 6; old APK artifacts are not this feature.

For a network-share checkout, an example machine-local setting is
`garby.buildRoot=C:/GarbyBuild/manual-disposal` in `local.properties`.

Merge the disposal-only example at
[`Garby_MobileApp/firebase/manual-disposal.rules.example.json`](https://github.com/YAKITORIHOSHI/Garby-Mobile-App/blob/main/firebase/manual-disposal.rules.example.json)
into the actual rules, preserving existing status/sensor/return access. Test it first in the
Firebase Emulator or Rules Playground as described in
[`FIREBASE_CONFIGURATION.md`](https://github.com/YAKITORIHOSHI/Garby-Mobile-App/blob/main/FIREBASE_CONFIGURATION.md).
The example is JSON-checked; live rule semantics/permissions have not been tested or deployed.
No live database command or rules update was sent during development.

The phone addition uses the existing static glass components, adaptive lazy grid, sampled
artwork cache, reduced-effects profile and shared bounded background dispatcher. One
conflated disposal listener lives only while the dashboard is visible; the existing freshness
tick is reused. There is no new dependency, permission, bitmap, timer or thread pool.
Low-end physical-device performance must still be measured on a Windows-built optimized APK.

## Deployment and acceptance

MCU upload requirement for this task: **Both** complete sketches, `BLE_Receiver-Final` and
`MAIN_EXECUTOR_CODE`, build `DISPOSAL-20261113`. Deploy the **entire RasPi/** directory,
including `disposal_support.py`, plus the Windows-built app and reviewed Firebase rules.
The source includes earlier local pending changes; installed revisions/earlier upload status
remain unknown. No upload, Pi deployment/restart, app installation, live command, SMS test or
rule deployment was performed during this task. Saved flasher state selects ESP32-S3 (8 MB,
no PSRAM) and ESP32 (4 MB, no PSRAM) and reports no connected MCU; saved state is not a live
connection check.

Under the existing supervised hardware-acceptance process:

1. Confirm build IDs and ordinary readiness. With secured wheels and all bin triggers below
   threshold or unavailable, confirm cancel sends no START, then confirm one disposal sends
   one ID and shows matching acceptance. Repeated taps/another phone cannot replace it.
2. Verify stop/blocked/stale LiDAR or missing heading holds the wheels. Emergency stop still
   latches, and a busy/faulted robot rejects a new disposal. Confirmed obstacle bypass modes
   keep their existing scope. No manual request creates GO or releases emergency stop.
3. Interrupt phone internet/cloud, BLE and Pi process at controlled points. Pending commands
   expire, claimed commands only query, lost ACKs never replay START, and unconfirmed results
   never claim arrival. Test two phones and the actual Firebase rules separately.
4. After existing floor acceptance, supervise the unchanged outbound route, actual arrival,
   disposal and explicit return. Measure stopping distance under the real payload separately.
5. On a low-end phone, use the optimized benchmark APK; check first-open/scroll/dialog
   responsiveness, rotation, large fonts, small/landscape screens and background/resume.
   Check background listener detachment and that offline/old status disables confirmation.

Exact software commands, outcomes, hashes and remaining skips are in
[`artifacts/verification/2026-11-13-manual-disposal/VERIFICATION.md`](../artifacts/verification/2026-11-13-manual-disposal/VERIFICATION.md).
