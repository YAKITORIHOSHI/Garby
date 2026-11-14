# GARBY Android integration — 2026-11-13

In the coordinated workspace, the robot-wide architecture is authoritative for Pi/MCU behavior.
This Android repository describes its Firebase integration here and in
[FIREBASE_CONFIGURATION.md](FIREBASE_CONFIGURATION.md). Live Firebase and hardware remain
external acceptance items.

## Responsibilities

Android authenticates through Firebase, shows what GARBY reports, marks old data
stale, and asks GARBY to start a confirmed manual disposal or return to base. It cannot grant GO, clear an obstacle,
steer a wheel, release an emergency stop, or substitute cached values for sensor
clearance.

| Area | Source |
| --- | --- |
| Firebase gateway (listeners, disposal and return writes) | `realtime/GarbyRealtimeDb.kt` |
| Pure rules, unit tested | `realtime/ReturnLogic.kt`, `Thresholds.kt`, `PiHealth.kt`, `SensorFreshness.kt`, `Models.kt` |
| Dashboard state | `realtime/DeviceViewModel.kt` (screen-scoped, lifecycle-aware) |
| Return workflow | `ReturnViewModel.kt` (activity-scoped, `SavedStateHandle`) |
| Screens | `StartScreen.kt`, `MainDashboard.kt`, `ReturnScreen.kt` |
| Glass design system | `ui/glass/GlassKit.kt`, `ui/glass/GlassVisuals.kt`, `ui/theme/` |

## What the dashboard shows

- **Robot state** from `devices/<id>/status/robotState`, as the Pi maps the executor's lines:
  IDLE (at base), OUTBOUND (driving to the collection point), AT_DESTINATION (waiting there),
  RETURNING, FAULT (`[ROUTE FAULT]` or `[RETURN REJECTED]`), UNKNOWN (controllers restarting or
  BLE down). A route track shows base, collection point and the robot's position by state.
- **Presence** from the Pi heartbeat (`lastSeen`, every 30 s): live up to 90 s, then the state is
  shown as "Last known". This is re-evaluated every 5 s, so a silent Pi cannot stay "online".
- **Bin and air**, with the executor's own thresholds (`MAIN_EXECUTOR_CODE`): load trigger 4.0 kg
  (two filtered readings), 20 kg plausibility; MQ4/MQ137 normal < 400 ≤ warning < 700 ≤ danger;
  MQ135 clean < 300 ≤ moderate < 500 ≤ poor < 700 ≤ very poor; the danger rule (any top band, or
  all three second bands, three packets in a row). Weight is reported only while idle at base.
  Bin distance is the sensor board's raw ultrasonic reading; its fill direction is not assumed.
- **System health** reported by the Pi: heartbeat, BLE link, LiDAR, sensor board, CPU
  temperature (warning at 80 °C like the Pi) and decoded `get_throttled` flags (undervoltage,
  throttling, now or since boot), plus the phone's internet and the cloud connection. Values the
  Pi reported before its heartbeat went stale are labelled "Last known".

## Return workflow

1. The return screen lists every condition and enables **Send back to base** only when all hold:
   the Pi's `isReadyToReturn` is true **and**, when published, `robotState` is AT_DESTINATION; the
   heartbeat is fresh; BLE and LiDAR are healthy; Firebase is connected; the phone has validated
   internet; no earlier request is still waiting or returning. The Pi relays any fresh request,
   and a request that reaches the executor mid-outbound latches a route fault, so this gate is
   strict.
2. After the operator confirms, `requestReturn()` waits for the Firebase connection, purges stale
   writes, then **re-reads the server's status and the current command** and re-applies the same
   gate. A request that fails here is reported as "Nothing was sent".
3. It writes `devices/garby-bin-01/commands/reset` with server `requestedAt`, the anonymous
   operator UID as `requestedBy`, `pending`, and cleared prior outcome fields, then reads back the
   server timestamp. That identity is kept in `SavedStateHandle`, so leaving the screen,
   rotating or backgrounding the app does not lose the request. A write that cannot be confirmed
   is reported as "Delivery not confirmed".
4. Progress comes only from the Pi's transitions of that exact request: **pending** (the Pi
   accepts requests up to 120 s old; the screen counts this down), **ack** (the executor accepted
   and is returning), **done** (matching completion), **failed** with the Pi's reason in words
   (`operator_emergency_stop`, `return_already_in_progress`,
   `controller_rejected_or_route_incomplete`, `completion_unverified`,
   `stale_or_future_reset_request`, `missing_reset_timestamp`). Unknown reasons are shown as-is.
5. Pending beyond 135 s reads "Not accepted"; accepted beyond 15 min 15 s reads "No arrival
   report". These change wording only; ages use Firebase's server-time offset because
   `requestedAt` is server time. A newer request from anyone marks the tracked one as replaced.

The obsolete `APP/isReadyToReset` flag is no longer read. A route fault or interrupted outbound
requires supervised physical recovery, never another app request.

## Freshness and startup

### Manual disposal

The dashboard adds a static **Manual disposal** card and scrollable confirmation dialog.
`DisposalLogic.kt` requires internet/cloud, fresh Pi and 15-second executor eligibility,
explicit IDLE, BLE and a loaded command record. Bin level, gas, load and obstacle readings
do not authorize or prevent the request: the controllers retain all movement gates. A
pending/dispatching/unknown command and a submission already in progress disable the action.

`DeviceViewModel` collects one new conflated `commands/dispose` listener inside the existing
visible-screen coroutine scope. It detaches on backgrounding; no timer or thread pool is
added. Submission and snapshot decoding use `GarbyDispatchers.background`, with Firebase
Tasks/callbacks suspending. Existing bitmap limits, lazy grid, reduced effects and no-blur
rendering remain intact; no dependency or Android permission is added.

After confirmation, the ViewModel rechecks readiness, then the gateway re-reads device status
and the command before an atomic compare-and-set transaction. A random request ID and
server timestamp identify the request. Competing or unresolved requests abort replacement;
speculative local transaction events are disabled. Timeout/cancellation purges pending
writes and reports uncertainty when transmission may have happened. A committed write is
displayed as pending until the Pi records the matching executor ACK (`accepted`), rejection
(`failed`) or missing result (`unconfirmed`). Acceptance does not claim arrival. Server
command state remains available after rotation/process death; there is no automatic resend.

The Pi claims each request durably before one START; recovery only queries the recorded
controller ID. `commands/reset` and its existing return rules do not change. Rules preparation
is in `FIREBASE_CONFIGURATION.md`; `DisposalLogicTest.kt` is prepared for the Windows test run.

Dashboard values expire after 90 s, with 30 s future-clock tolerance. Unknown values remain
unavailable rather than synthetic zeros; the Pi's 999 cm and -1 sentinels show as unavailable.
Firebase fields are read with type-safe getters, so one malformed field cannot fail a stream.
Listeners attach only while a screen is visible (`repeatOnLifecycle`, `WhileSubscribed`).

## Glass design

The activity and confirmation dialogs use immersive fullscreen: Android status/navigation
bars are hidden using the shared `ui/Fullscreen.kt` window policy. An edge swipe reveals
transient system controls; focus restoration reapplies the policy. Compose dialogs own a
separate window and a disposable focus listener. Insets still protect display cutouts;
hidden bars no longer reserve the normal status/navigation space.

Launcher foreground is a vector traced from the retained official GARBY wordmark/cart,
centered inside the 66 dp adaptive safe circle. The print-only ring and tagline are omitted
for small-size legibility. The themed foreground uses the same silhouette. Regenerate the
vector from `design/garby_logo_source_2000x1467.webp` with
`design/generate_launcher_mark.py` (Python + Pillow); the original artwork is unchanged.

Every screen uses cached marble artwork, smoked fills, a subtle sheen and glass borders. Live
backdrop blur and continuous decorative animations are removed on every device, including
mid-range phones; no off-screen Haze source or blur layers are allocated. Dialogs use thick
smoked glass over the system dim. Status colours always come with an icon or a word.

Category dividers combine a stable icon/accent, a concise description and an independent
status summary. `ui/DashboardIndicators.kt` derives summaries from fresh, valid readings and
the same levels used by the system tiles. Positive sensor counts require cloud and phone
connectivity plus valid timestamps; current gas warnings remain visible if another sensor
is missing. These summaries only describe telemetry and never affect robot control or the
return gate. Headings merge their accessible descriptions, and status capsules wrap below
the title when needed. Metric-card status has its own row, avoiding competition with the icon
on narrow cards. All new decoration is static.

The adaptive metric grid scales its minimum card width with the user's font scale. Larger
text therefore selects fewer columns instead of squeezing the readings and units. Cards
share a 264 dp minimum height at the default font size; explanatory update text can wrap.

Large bitmaps live in `res/drawable-nodpi` so they are not density-scaled at decode (the old
`drawable/` placement turned the 2000 px logo into an ~80 MB bitmap on a 420 dpi phone). The
launcher icon is adaptive with a themed (monochrome) layer; the original 2000 px logo is kept as
source art in `design/`.

### Low-end rendering and background work (2.0.1)

`performance/DevicePerformance.kt` selects a stable process-wide profile from Android's low-RAM
flag, total RAM (up to 6 GiB), heap class (up to 192 MiB), or CPU count (up to four). No brand/model
conditions are used. Both profiles avoid blur and continuous decorative effects. The reduced
profile also uses immediate navigation/gauge updates and omits decorative glows/large shadows.
Capable devices use short, finite transitions. Loading spinners still indicate active work.

All app-owned background CPU work uses one shared `Dispatchers.Default.limitedParallelism`
view: one worker at a time on reduced-effects devices, two otherwise. This does not resize the
global coroutine pool or Firebase's own network threads. Firebase Tasks suspend instead of
blocking workers. Callback channels conflate before snapshot decoding as well as at the output,
so bursts replace queued obsolete telemetry; unchanged decoded states are suppressed. The
dashboard and return workflow share connection/status listeners. The last subscriber detaches
them; replay expires on stop, and errors still reach each consumer. Existing screen lifecycle
and return-workflow subscription timeouts remain in force.

Bundled artwork is decoded on that background dispatcher with power-of-two sampling. Reduced
backgrounds have a maximum 1024 px dimension; their logo uses 512 px (1024 px on capable devices).
The shared decoded cache is
byte-bounded (4 MiB reduced, 16 MiB otherwise), serialized against duplicate decodes, and cleared
on memory pressure/background trim. Active images are never forcibly recycled. With the current
artwork the sampled images use approximately one quarter of the original decoded pixel memory.
The control write, server readiness re-read and freshness deadlines are unchanged.

The return ViewModel also moves its clock-driven readiness/progress combination upstream of
`flowOn(GarbyDispatchers.background)`. Return submission performs server queries/response
decoding on that same bounded dispatcher, then updates `SavedStateHandle` and visible
submission state on the main dispatcher. All Firebase Tasks still suspend; no worker blocks
on network I/O and no new thread pool or timing gate is added.

Device observations and verification are described in `docs/2026-11-14-low-end-performance.md`.
Raw evidence remains in the coordinated workspace's `artifacts/android-low-end-2026-11-14/`.
Before/after phone acceptance is a separate step from source changes and successful builds.

## Notifications

FCM registers app instances by Firebase Installation ID (`firebase_messaging_installation_id_enabled`);
the deprecated registration-token API is not used. After sign-in the app publishes its ID at
`devices/<id>/fcmInstallationId`. This repository supplies no FCM sender; the receiver accepts the
existing `isRunningToPointB` / `isReadyToReturn` events. The small icon is the monochrome cart glyph.

## Build

AGP 9.5.1, Gradle 9.9.1 (wrapper pinned by SHA-256), Kotlin 2.4.21, Compose BOM 2026.09.00,
Firebase BoM 35.2.0, compileSdk 37.2, targetSdk 37, minSdk 26, Java 21. Android 17 target-only
changes (local-network permission, certificate transparency, background audio, widgets) do not
affect this Firebase-only app.

This checkout lives on a network share. `settings.gradle.kts` lets a machine keep build outputs
on a local disk by setting an absolute `garby.buildRoot` in its own `local.properties`; without
it outputs stay in `app/build`. Daemon memory is 3 GB heap / 1 GB metaspace so lint, R8 and tests
fit in one run.

The coordinated robot workspace's `tools/verify_project.py` runs Android unit tests,
debug/release assembly and lint with its robot checks. In this Android repository,
`python -B tools/static_quality_check.py` checks release invariants without an Android SDK.
Release assembly is unsigned.

`:app:assembleBenchmark` creates an installable, non-debuggable, R8-minified build signed with
the local development key, for performance checks on the phone. It uses the same application
ID and data as the app; obtain approval before replacing the phone installation. This is a local
verification artifact, not production signing. Use release-equivalent builds for performance
comparisons because Compose debug instrumentation adds substantial overhead.

## Manual-disposal outcome reconciliation

Local uncertain-delivery notices carry their generated request ID. A known server stage for
that exact ID clears the local warning and displays the authoritative request status. This
also handles the listener arriving before the write callback. A report for another ID or an
unknown stage cannot resolve the uncertainty, and a write receipt cannot replace a newer
request or downgrade an outcome already delivered by the listener. This adds no listener,
timer or network retry. Acceptance still describes route intent, separately from arrival.
