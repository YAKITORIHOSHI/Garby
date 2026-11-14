# GARBY Mobile App - Patch Notes

## GarbyMostLatest-Test2 — 2026-11-14

- Integrates the confirmed manual-disposal feature below. A late matching cloud report now
  replaces a local timeout notice; a callback arriving before the timeout result cannot have
  its accepted/rejected outcome hidden. Another request cannot resolve this request's warning.
- Android: 64 passing unit tests, lint with no issues, optimized development-signed APK and
  unsigned release APK built on Windows. Physical phone performance was not retested.
- Coordinated robot review reproduced and repaired loss of turn-sweep protection across BLE
  reconnect. The bridge preserves turn state and holds GO while unknown after restart; the
  executor replays that state before readiness. Matching complete firmware sketches are
  required. Test1 retains the previous mobile revision for separate comparison.
- [Review, exact changes and verification limits](docs/2026-11-14-integration-review.md).

## Most Latest · Manual Disposal (versionCode 6)

2026-11-13 — Confirmed manual disposal for end-of-school collection.

- Dashboard button and confirmation send an idle GARBY to the collection point before
  bin/gas/load thresholds are met. Obstacles, heading and emergency stop still gate motion.
- Transactional, expiring requests cannot replace unresolved requests; sent and controller
  acceptance are separate. Unconfirmed delivery never shows as success or auto-resends.
- Reuses existing reduced effects, lazy dashboard, bounded background dispatcher and
  lifecycle-owned listeners. No new dependency, image, animation, phone polling or thread pool.
- Source static checks run in the coordinated workspace. Android tests, lint and assembly now
  pass on Windows; low-end phone performance remains unmeasured for this feature. Matching Pi/MCU deployment
  and an explicit disposal Firebase rule are required.

## Most Latest

2026-11-14 — Dashboard categories, indicators and fullscreen experience.

- Android's status and navigation bars are hidden on all app screens and confirmation
  dialogs. Swipe from an edge to reveal system controls temporarily; immersive mode is
  restored when the app or dialog regains focus. Cutout padding remains respected.
- Replaced the padded bitmap launcher foreground with a centered vector traced from the
  official GARBY wordmark and cart. Round, rounded-square and themed icons share the same
  recognizable mark inside Android's safe area, without the print-only ring and tiny tagline.
- Bin, Air and System now have inset category dividers with distinct icons, subtle accent
  rules, concise descriptions and summaries of the readings below them.
- Live counts, gas bands, dispatch weight and system issues use words and icons as well as
  colour. Missing, invalid, stale or disconnected telemetry cannot produce an all-clear.
- Metric cards give their status a separate row so longer labels fit narrow cards. Stale
  sensors use neutral icon accents and a clock indicator. System tiles use status symbols.
- Larger system text selects fewer grid columns, metric cards share a consistent minimum
  height, and explanatory update text wraps instead of being cut off.
- Status capsules use quieter borders, tinted text and static dot halos. Category titles
  remain accessible headings, and their status wraps when space or font size requires it.
- The app's displayed version name and this release title are **Most Latest**. Android keeps
  a numeric build code internally for upgrade ordering.
- Audited Firebase subscriptions and background work. Return-state calculations and server
  response handling now use the existing bounded background dispatcher; saved request state
  stays on the main thread. Database paths, readiness gates and robot commands are unchanged.
- Verification: 53 unit tests, lint with no issues, static checks and debug/release/optimized
  builds. Headless Compose layouts cover 320–400 dp widths and text scales up to 200%.
- Installed and checked the exact optimized APK on the authorized test phone. In a fixed
  scroll comparison against its debug installation, janky frames fell from 57.1% to 20.0%,
  P90 frame time from 81 to 36 ms, and PSS from 153.3 to 61.1 MiB. This includes the build-type
  change; some jank remains and only one phone was measured. Fullscreen/dialog/resume and
  blocked return-screen navigation passed without sending a robot command.
- **MCU upload: None; Pi deployment: None.** Only the Android app changes.

Fullscreen, icon and runtime implementation/verification are recorded in
[the fullscreen report](docs/2026-11-14-fullscreen.md).

## 2026-11-14 Device-wide rendering and memory performance (2.0.1)

- Removed live blur/Haze layers and continuous decorative animations on every device, including
  mid-range phones. Smoked glass, readable status words and all controls remain. Short finite
  transitions remain on capable devices.
- Capacity-based profiles use RAM, heap class and CPU count only, with smaller images, immediate
  navigation/gauges and fewer glows/shadows on constrained devices. No brand/model special cases.
- Reuses one bounded coroutine dispatcher view (one background CPU task at a time on reduced
  devices, two otherwise), without adding custom executors or changing Firebase's worker pools.
- Snapshot bursts conflate before decoding; unchanged telemetry is suppressed. Connection and
  robot-status listeners are shared between the dashboard and return workflow, with replay
  cleared after the last observer leaves and errors delivered to every observer.
- Background image decoding, sampled artwork and a byte-limited cache reduce main-thread work
  and bitmap memory. Memory-trim callbacks drop cached off-screen artwork safely.
- Added a development-key-signed `benchmark` build with release optimizations for phone checks.
  Production release signing stays external. Readiness, freshness and robot protocols are unchanged.
- After installation on the connected test phone, sampled median scrolling frame time changed
  from 300 ms to 26 ms, and process PSS from 200.6 MiB to 58.5 MiB. This includes a debug-to-optimized
  build change; some jank remains and other physical devices were not measured. All 43 unit tests,
  lint, static checks and APK builds passed. Details:
  [performance report](docs/2026-11-14-low-end-performance.md).
  **MCU upload: None; Pi deployment: None.**

## 2026-11-13 Glass redesign, current robot protocol, toolchain update (2.0.0)

Current behaviour is described in [SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md).

- Redesigned in glassmorphism (Haze 2.1 backdrop blur on Android 12+, opaque smoked glass
  below): start, dashboard and return screens; floating glass top bar and dock.
- Shows the robot state the Pi publishes (idle, outbound, at the collection point, returning,
  route fault, controllers unknown) with a route track, and heartbeat-based presence that turns
  "Last known" after 90 s. Previously the online label could stay ONLINE after the Pi went silent.
- Bin and gas cards use the executor's thresholds (4 kg load trigger; MQ135 four bands; the gas
  danger rule). The old 0.7/1.0 kg and single 400/700 ppm bands were out of date.
- Return: readiness checklist; the server status and command are re-read immediately before the
  write; the Pi's ACK, completion and failure reason are shown; the request survives leaving the
  screen or the app. The screen no longer locks the operator in for 15 minutes.
- Fixed: bin distance above 300 cm displayed as 300; malformed boolean/string fields failed a
  whole stream; launch FCM write before sign-in; white window flash; navigation bar force-hidden;
  notification small icon rendered as a white disc; logo/backgrounds density-scaled to ~80 MB;
  device-to-device transfer of the anonymous session on Android 12+.
- FCM moved from deprecated registration tokens to Firebase Installation IDs
  (`devices/<id>/fcmInstallationId`).
- Toolchain: AGP 9.5.1, Gradle 9.9.1 (checksum-pinned), Kotlin 2.4.21, Compose BOM 2026.09.00,
  Firebase BoM 35.2.0, navigation 2.10.2, compileSdk 37.2, targetSdk 37. Unused WorkManager
  removed; adaptive launcher icon added.
- Cleanup (2026-11-14): new [README.md](README.md); Firebase note rewritten with the app's
  read/write paths; launcher label "GARBY". Removed the Pi compatibility launcher
  (`final_w_serial.py`; run `RasPi/final_w_serial.py`), a May Gradle debug log, template-only
  `proguard-rules.pro` and example tests, and stale `.kotlin/`, `build/` and Gradle 9.6.1
  caches. Older audit, validation and startup-plan reports moved to `docs/history/`.

## 2026-10-31 Firebase client configuration review

- Synced `app/google-services.json` from `RasPi/Firebase/google-services.json`.
  Both files currently match byte-for-byte and register `com.thesis.garby`.
- Kept the existing anonymous-authentication flow. The provided Admin SDK
  private key and plaintext account password were not copied into Android.
- See [Firebase configuration](FIREBASE_CONFIGURATION.md) for the project and
  database consistency checks and credential handling.

## 2026-10-02 coordinated repair

See [release notes](../RELEASE_NOTES.md) and [current architecture](../SYSTEM_ARCHITECTURE.md).
Return uses exact server-issued IDs and actual MCU ACK/DONE/FAILED, fresh live
readiness gates, and a 15-minute completion window. The obsolete APP flag is no
longer written with commands. Historical sections below describe earlier releases.

## 2026-08-11 deployed-database integration

- Mapped each dashboard metric to the exact legacy path in the supplied
  `GARBY-DATABASE.json`, while retaining the newer `/devices/{id}/sensors/...`
  path as the preferred rolling-upgrade source.
- Replaced five whole-database sensor subscriptions with narrow listeners for
  only the relevant device and legacy sensor nodes.
- Narrowed robot-status, reset-status, and return-readiness listeners as well,
  so sensor writes no longer resend/reparse the entire RTDB tree in unrelated
  app streams.
- Removed ambiguous numeric-child guessing that could mistake `updatedAt` for
  a sensor value; deployed value-field names are now explicit and tested.
- Made the structured reset command and `/APP/isReadyToReset` compatibility
  flag one atomic multi-location write.
- Fixed a reset race where `APP/isReadyToReset = false` could incorrectly turn
  a fresh structured `pending`/`ack` command into a displayed completion.
- Batched the three FCM-token compatibility writes and stopped logging the
  registration token value.
- Added an optional system-health line for Pi temperature/throttling and
  BLE/LiDAR/sensor-board connectivity; older database snapshots remain fully
  compatible because absent fields are hidden.
- Added schema and reset-compatibility unit tests.

## Security

- Removed embedded Firebase login credential.
- Removed legacy Basic-Auth direct-Pi client and trust-all TLS/hostname bypass.
- Disabled app backup and cleartext traffic.
- Removed unnecessary network-state mutation permission.
- Authentication uses Firebase anonymous auth with no embedded password or
  service credential; Firebase SDK owns token refresh. Anonymous auth must be
  enabled for this Firebase project, and RTDB rules must still restrict writes
  to authenticated clients and the intended reset paths.

## Reset/control safety

- Added explicit confirmation dialog.
- Added `ResetViewModel` so reset state survives recomposition/configuration.
- Requires authenticated UID, validated Android internet, and Firebase connected state before send.
- Disabled Firebase disk persistence to avoid persisted control-write replay.
- Purges outstanding writes around failed reset transmission.
- Correlates completion with a newer request marker and requester identity.
- Unknown/timeout status is presented as unknown; the app never claims completion without matching `done`.

## Monitoring correctness

- Missing sensor values no longer default to zero.
- Added finite-value and timestamp validation.
- Added periodic stale detection.
- Added explicit robot online/offline/stale status separate from Firebase cloud connection.
- Battery `null` is no longer displayed as `0%`.

## Reliability/performance

- Replaced singleton Firebase listener lifecycle with collector-owned callback Flows.
- Retry only transient RTDB failures with bounded exponential backoff/jitter.
- Conflated latest-value monitoring streams.
- Removed leaked process coroutine scope from network monitor.
- Removed dead unbounded thread pools and double gauge animation.
- Pruned unused libraries/plugins.
- Enabled release minification/resource shrinking.
- Reduced font and drawable payload substantially.
- Debug unit tests, Android lint, debug assembly, and R8 release assembly all
  pass in the current workspace.

## Compatibility note

Reset field names/status strings are unchanged. `requestedBy` now contains the authenticated Firebase UID instead of the previous constant Android label. Verify the Raspberry Pi does not require an exact literal value.

## Deployment order

1. Rotate any credential that was exposed by an older application build.
2. Enable Firebase anonymous auth and verify RTDB rules.
3. Confirm Pi compatibility with UID-valued `requestedBy`.
4. Build/test the Android app with the real Android SDK/Gradle environment.
5. Run wheels-lifted reset/STOP integration tests.
6. Sign and deploy the mobile app only after those checks pass.
