# GARBY release notes

## Test2 integration review — 2026-11-14

- Android manual-disposal outcomes reconcile late reports by request identity in either
  callback order. A timeout notice cannot hide a matching server acceptance/rejection.
  Windows verification passed 64 unit tests, lint, optimized and unsigned release assembly.
- Bridge BLE-session reset now preserves the executor's turn scope. After bridge restart,
  GO stays blocked until that scope is known; executor status replays TURN:START/END before
  readiness, and repeated turn reports do not restart clearance confirmations. Original
  production code failed reconnect and missing-replay regressions before repair.
- **MCU upload requirement: Both**, complete bridge/executor sketches, build
  `RECONNECT-20261114`. No upload was performed here; installed identity remains unknown.
  This follow-up changes only a Pi test; prior manual-disposal Pi deployment remains required.
- Mobile Test1/Test2 assets preserve the previous app and the new candidate separately.
  Full review, Windows Pi-suite limitations and exact sibling patch are in
  `Garby_MobileApp/docs/2026-11-14-integration-review.md` and its verification directory.

## Confirmed manual disposal — 2026-11-13

- Pi monitor and Android dashboard offer **Manual disposal**, with an explicit confirmation,
  for a school-day-end outbound run before bin/gas/load thresholds are reached. The normal
  path, heading, obstacle and emergency-stop gates still control every movement.
- The executor advertises fresh idle eligibility; the bridge filters it against readiness
  and operator stop. The Pi atomically claims short-lived cloud requests before sending one
  START, matches the executor result and only queries after uncertainty or restart. The app
  separates sent, accepted, failed and unconfirmed; arrival uses ordinary robot state.
- The app reuses static glass controls, its bounded background dispatcher and lifecycle-owned
  conflated listeners. No new library, bitmap, thread pool, animation or polling timer is
  introduced on the phone. Android tests/build/lint and device performance are prepared for
  Windows and not executed in this task.
- **MCU upload requirement: Both** complete sketches, ID `DISPOSAL-20261113`.
  **Pi deployment: entire RasPi/**. Android versionCode 6 and a disposal-only rules example
  are prepared. No hardware or cloud deployment was performed; installed revisions and
  earlier pending firmware remain unknown. Design and acceptance:
  `docs/manual-disposal-2026-11-13.md`; exact checks and hashes:
  `artifacts/verification/2026-11-13-manual-disposal/`.

## Local testing copy — 2026-11-13, findings review repairs

- All motion resumes require heading readiness, including STOP:MPU and an outage
  during an existing obstacle/sonar hold. Plain/reverse moves also brake on lost
  heading. A long outage with a new reference needs two fresh agreeing wall scans;
  missing walls hold within the original five-minute recovery budget. Ordinary
  obstacle holds and continuous-heading wall gaps retain their existing behavior.
- Pi P/S snapshots retain acquisition deadlines through coalescing. S.G is
  recomputed after the BLE writer lock; expired/oversized steering becomes STABLE.
  Bridge permission expires at 500 ms acquisition age including its mailbox wait
  and the existing 30 ms transit estimate. Wall age limits remain separate. This
  is not end-to-end source-age validation for PATH.
- NumPy is required by both install profiles and preflight. The unresolved map
  test was reproduced with a frozen host clock and given explicit independent
  source stamps; duplicate-source rejection remains required. Details:
  `docs/review-fixes-2026-11-13.md`.
- **MCU upload requirement: Both**, complete bridge/executor sketches, build
  `REVIEW-20261113`. **Pi deployment: entire RasPi/** plus requirements.
  No upload/deployment was performed. Installed revisions and earlier pending
  firmware remain unknown. Changes remain local for owner comparison; no remote
  repository update is included.

## 2026-11-14 — Android 2.0.1: device-wide rendering and memory performance

- No live blur source layers or continuous decorative animations on any device; the glass styling
  and operator controls remain. Smaller images/concurrency limits use RAM, heap class and CPU
  capacity, with no brand/model special cases.
- One shared background coroutine dispatcher view (one/two concurrent CPU tasks by profile),
  latest-snapshot decoding queues, shared connection/status listeners, and sampled background
  bitmap decoding with a bounded cache. Firebase's internal thread pools are unchanged.
- Added a signed local `benchmark` variant with release optimizations. The exact final APK was
  installed and measured: median scrolling frame time 300 → 26 ms, process PSS 200.6 → 58.5 MiB,
  including debug-to-optimized build differences. Some jank remains; other physical devices were
  not measured. All 43 tests, lint, static checks and APK builds passed.
  See `Garby_MobileApp/docs/2026-11-14-low-end-performance.md` and
  `artifacts/android-low-end-2026-11-14/` for evidence, checks and the exact file manifest.
- **MCU upload: None. Pi deployment: None.** Android-only changes; return gates and wire contracts
  are unchanged. This creates no additional firmware upload requirement.

## 2026-11-13 — Android app 2.0.0: glass redesign, robot state, return progress (Android only)

- New glass UI (start, dashboard, return) over the marble backdrop; Haze blur on Android 12+.
- Shows the robot state the Pi already publishes (`status/robotState`) with a route track, and
  heartbeat-based presence; bin and gas cards use the executor's thresholds (4 kg trigger, MQ135
  four bands, gas danger rule) instead of stale ones.
- Return to base: readiness checklist, a server re-read of status and command immediately before
  the write (a request reaching the executor mid-outbound latches a route fault), the Pi's
  ACK/DONE/FAILED with the failure reason in words, and tracking that survives leaving the app.
- FCM uses Firebase Installation IDs (`devices/<id>/fcmInstallationId`); registration tokens are
  deprecated. Toolchain: AGP 9.5.1, Gradle 9.9.1, Kotlin 2.4.21, Compose BOM 2026.09.00, Firebase
  BoM 35.2.0, compileSdk 37.2, targetSdk 37.
- 2026-11-14 cleanup: `Garby_MobileApp/README.md` added; the Android-folder Pi launcher
  `Garby_MobileApp/final_w_serial.py` removed (`tools/verify_project.py` no longer compiles it);
  old app reports moved to `Garby_MobileApp/docs/history/`.
- Firebase paths, statuses and the return protocol are unchanged. **MCU upload: none**; no Pi
  change. Install the new APK. Details: `Garby_MobileApp/PATCH_NOTES.md`; evidence:
  `artifacts/android-glass-2026-11-13/`. Not tested on a phone or against live Firebase.

## 2026-11-12 (B) — holds that resume, sonar sweep, moving-object prediction (NAV-20261112 executor + Pi)

- Never stuck forever: a sonar ping with no echo is a far clear reading (an open hallway could
  never release a hold before); every segment re-holds on a STOP instead of latching a route
  fault; the heading limit is 5 min (an S3 restart outlasted the old 10 s), with a wall check
  of the route direction after a long outage. Holds drop to low power after 10 s (CPU 80 MHz,
  slower polls/pings) and restore normal power on the first clear evidence.
- Obstacle sonar servo sweeps (`SonarSweep.h`): ±25 fan while driving, ±70 survey while stopped,
  spin-circle check at floor level before every spin. It was never moved before.
- Pi `motion_tracker.py`: moving-object tracking with time to contact; STOP at ≤2 s (a walker
  4.8 m away, a runner 4.9 m in the model), caution speed at ≤4 s or a mover closing within
  2.5 m, turn held for a mover reaching the spin circle. Adds evidence only; generic codes; the
  read-only front bypass bypasses it. Real-LiDAR bench recording: no false holds.
- Pre-run centring before both long hallway runs (`runLongHallway`), with a learned wheel
  distance scale and a wrong-way stop. Model: long-run centring command 0.00° (was 1.67°).
- Buzzer task: beep every 1.2 s while driving a route, double chirp while held, sparse in
  low power. New telemetry: `[DRIVE:..]`, `[POWER:..]`, `[SONAR:..]`, `[HEADING CENTER ..]`.
- Upload: **Executor only** (`MAIN_EXECUTOR_CODE`, NAV-20261112); bridge source unchanged
  (NAV-20261111, still pending upload if the S3 is older). Deploy the complete `RasPi/`.
  Report: `docs/obstacle-sweep-hold-2026-11-12.md`; evidence:
  `artifacts/obstacle-sweep-hold-2026-11-12/`. Not uploaded or floor tested.

## 2026-11-12 — LiDAR radar map, full X3 scans, quieter startup log (Pi only)

- LiDAR: the YDLIDAR SDK's fixed resolution cut 4-11 points off every X3 scan
  (`Real points 694..701 > fixed points 690`, hidden by the suppressed driver output).
  The driver now runs with `fixed_resolution:=false`; valid returns per scan rose from
  557 to 578 on the bench. `GARBY_LIDAR_FIXED_RESOLUTION=1` restores the old setting.
- Monitor: one radar-style map replaces the eight number cards. All eight sectors,
  including front left/right and rear left/right, are written around the scope in whole
  centimetres. Stop distances are arcs, and a close sector reads "close" in red. Without a
  scan: a green radar sweep while the LiDAR is found and started, or "LiDAR not available"
  with the supervisor's reason.
- Log: no startup ERROR before the first scan (INFO "Waiting for the first LiDAR scan";
  STOP unchanged); chassis-zone lines no longer repeat every scan on a zone boundary.
- Hardware findings (not fixed by software): the S3 was not advertising; the sensor board
  was not on any port (GPIO UART off). See [the report](docs/lidar-map-2026-11-12.md).
- Software tested (Pi 416 OK with Tk on Xvfb; firmware host and sanity checks pass).
  **No MCU upload.** Deploy the complete `RasPi/` directory.

## 2026-11-11 — hallway navigation redesign (NAV-20261111, both boards + Pi)

- Root cause of off-centre driving: the held heading was gyro-only and re-zeroed per
  segment, so its start error and gyro drift became lateral error that the tap-based
  LiDAR check could not remove. See [the report](docs/navigation-2026-11-11.md).
- Pi: wall line fits (`WL`, `WR`, `H`, scan age `G`) per scan; chassis-sized zones add a
  front box, a caution zone (`F=W`), a side band and a spin sweep (`T=`). The existing
  sector limits are unchanged. Zone changes are logged and shown in the monitor footer.
- Bridge: one continuous tilt-compensated heading streamed at 50 Hz; bias re-measured while
  held; per-side steering permission; wall fits forwarded to the executor; `GO:SLOW`,
  `STOP:TURN`; `STOP:MPU` whenever the heading is invalid; `H` console command. Taps removed.
- Executor: gyro/LiDAR fusion steers to the measured hallway direction and centre
  (bounded, deadband and hysteresis); MPU turns checked and corrected against the walls
  with commanded-vs-achieved logs; no motion without heading.
- Software tested (Pi 396 OK + 53 Tk on Xvfb; firmware host 522 checks) and compiled for
  both targets. Not uploaded, not floor tested. Flash both boards and deploy `RasPi/` together.

## 2026-11-09 — accurate BLE discovery diagnostics

Pi-only: distinguish no scan events, unmatched advertisements and GARBY candidates
excluded by a pinned address. Keep strict identity matching, scan deadlines,
backoff, readiness and motion gates unchanged. Deploy complete `RasPi/`; no MCU
upload. The owner's USB-success/battery-failure comparison points to standalone
power/startup conditions; no physical repair or reconnection is claimed. See
[the investigation](docs/ble-establish-2026-11-09.md).

## 2026-11-09 — correct reported LiDAR left/right readings

- Reflect the robot-frame lateral scan coordinate after the existing mounting yaw.
  Map, summaries, wall estimates and nearest-object CL/CR share the correction;
  front/back distances retain their meaning. Default `GARBY_LIDAR_REFLECT_LEFT_RIGHT=1`;
  `0` retains the earlier orientation for an alternate verified installation.
- Preserve GUI layout/caches, acquisition freshness, thresholds, bypasses and
  all motor/turn/nudge algorithms. The user clarified nudging was not tested.
- Pi-only change: deploy the complete `RasPi/` directory, with no new MCU upload.
  See [the report](docs/lidar-left-right-2026-11-09.md).

## 2026-11-09 follow-up — apply integration fixes while preserving GUI optimization

Pi source changes in `final_w_serial.py` and `monitor_ui.py`; deploy the complete
`RasPi/` directory. The motor `false/true` correction and compiled executor image
are unchanged. See the [summarized review](docs/front-direction-circular-review-2026-11-09.md).

- Preserve acquisition time independently of optional circular geometry. A failed
  plot build or malformed source timestamp cannot make delayed summaries look fresh.
- Show unavailable for sectors without current coverage instead of reusing an old
  distance under a new scan timestamp. Valid-sector smoothing history is preserved.
- Scan-processing exceptions immediately clear health/evidence and queue STOP,
  using the existing watchdog and normal fresh-scan recovery.
- Preserve the GUI layout, 150 ms polling, Canvas reuse, hidden-view optimization,
  cached setters, route calibration, nudging, selected bypasses and operator controls.

## 2026-11-09 — front direction and circular LiDAR readings

Deploy the complete `RasPi/` directory and compile/flash the executor separately
when hardware deployment is authorized. The S3 source is unchanged. See the
[direction report](docs/motor-direction-2026-11-09.md),
[circular scan report](docs/circular-lidar-2026-11-09.md) and dated validation artifacts.

- The owner confirms that the November 7 build still drove backward. DIR polarity
  is now motor 1 `false`, motor 2 `true`, with build and polarity boot diagnostics.
  Route values, turn targets and nudge-wheel selection are preserved. The latest
  report establishes forward reversal only; named turns/nudges need physical checking.
- The LiDAR monitor can show up to 360 measured angular bins on a circular view,
  with range rings, front-up orientation and mounting offset. The existing eight
  distance summaries remain accessible. This view does not track robot position.
- Rendering and snapshot sizes are bounded, immutable geometry is cached, headless
  operation skips scan-display work, and hidden views defer Canvas updates.
- Obstacle thresholds, read-only bypass scope, sonar-off behavior, emergency stop,
  path freshness, centering reference and BLE/UART contracts are preserved.

## 2026-11-07 (C) — motor direction reversed

Executor sketch only: flash `MAIN_EXECUTOR_CODE`. The S3 sketch stays `CENTER-20261107B`, and the Pi needs no change. Source
implemented and software tested. **Not flashed, not physically tested.** See the [report](docs/motor-direction-2026-11-07.md)
and `artifacts/motor-direction-2026-11-07/`.

- Forward, backward, left, right and both nudges were all reversed on the floor. They are corrected by swapping the two
  DIR polarities in `MAIN_EXECUTOR_CODE.h`. No GPIO changed, and route, turn and nudge code stay as they were.
- The old nudge looked reversed because slowing a wheel steers the opposite way while rolling backward. In the hallway
  model, the old polarity behaves like the 03:44 floor run: every tap adds to the deviation, and the robot ends at the wall stop.
- **Before a floor run, check which way the LiDAR faces.** The robot now travels the opposite way, and the Pi's limit is
  95 cm in front but only 35 cm behind. The route is also rotated 180 degrees about its start.

## 2026-11-07 (B) — centering to the requested rules

Bridge ID `CENTER-20261107B`. S3 sketch only: flash `BLE_Receiver-Final`. Executor sources are unchanged and the Pi needs
no change. Source implemented and software tested against a hallway model. **Not flashed, not floor tested.** See the
[report](docs/centering-spec-2026-11-07.md) and `artifacts/centering-spec-2026-11-07/`.

- A centered robot gets no LiDAR action at all: 8 cm deadband, 3 cm hysteresis, two off-center scans in a row to start.
- Each correction is one sidestep of +/-5 degrees from a reference that never moves, ended on where the robot will land.
- The tap now aims at the target instead of stopping 0.5 degrees short. That leftover heading was the main cause of the zigzag.
- A correction that does not improve the error is never repeated, and the log says `CHECK STEERING SIGN`.
- A rescan every 3 s, only near the reference. One invalid or very large reading skips that cycle.
- **Cost:** reference trim is gone, so in the model a start misaligned by about 2.5 degrees or more, or gyro drift of about
  0.05 deg/s or more, ends at the wall stop. Read the report's first section before a floor run.

## 2026-11-07 — LiDAR and MPU hallway centering

Bridge ID `CENTER-20261107`. S3 sketch only: flash `BLE_Receiver-Final`. Executor sources are unchanged and the Pi needs
no change. Source implemented and software tested against a hallway model. **Not flashed, not floor tested.** See the
[report](docs/lidar-mpu-centering-2026-11-07.md) and `artifacts/lidar-mpu-centering-2026-11-07/`.

- LiDAR no longer steers on every scan. Every 5 s it checks a 5-scan median and may move a heading target, which the
  MPU holds with the existing bounded taps.
- Heading band is now 1/0.5 degrees, and taps release from a physical coast estimate, so one tap covers most of an error.
- LiDAR drift and held yaw together correct a reference that is not parallel to the walls, within +/-8 degrees.
- Corrections end on where the robot will be, using its yaw, instead of on a lagging median.
- New: a yaw step over 20 degrees faults as `JUMP`. `CHECK STEERING SIGN` and `BIASED REFERENCE?` report in the log.
- Decisions reach the Pi log as `[BRIDGE CENTER ...]`. Debug prints skip rather than block the gyro loop.

## 2026-11-05 — stop reading hallway walls as obstacles

Pi only: restart the bridge. No firmware flash. Evidence is in
`artifacts/hallway-lidar-stop-2026-11-05/`.

- The front-left and front-right stop limit is now 60 cm instead of 95 cm. The
  diagonals also see hallway walls, so 95 cm stopped the robot beside a post even
  when centered (82/82 cm), and left only 7 cm of room in the 95/95 cm hallway.
  The robot now stops when a wall is closer than about 55 cm.
- The straight-ahead limit stays 95 cm, as do the rear 35 cm and side 22 cm limits.
- The monitor shows the new limit and no longer marks a diagonal reading between
  60 and 95 cm as "Close obstacle".
- Not yet measured on the robot: stopping distance, chassis width against the
  22 cm envelope, and an object entering from the side, which is now seen later
  on a diagonal.

## 2026-11-04 — keep steering and motion through link and sensor glitches

Bridge ID `LINKFIX-20261104C`. Flash both sketches and deploy `RasPi/` together: an older bridge
ignores the new `[MCU ALIVE]` ping. See the [review](docs/ble-nudge-review-2026-11-04.md).

- An MPU fault or failed boot calibration no longer needs a USB console or a reset. The robot holds
  `STOP:MPU`, recalibrates while still with unchanged limits, and resumes. `mpu=` in `[BRIDGE DIAG]` names the cause.
- The Pi closes a dropped BLE session before reconnecting. Each reconnect used to add a duplicate of
  every notification and corrupt lines over two frames.
- `[BRIDGE DIAG]` waits 2 s after readiness: inside the connect burst it could make NimBLE reject a Pi write.
- A BLE outage over 3 s no longer ends MPU steering for the rest of the segment.
- A side with no wall return (doorway, open space) removes centering, not heading correction.
- One bad gyro read is skipped; 100 ms without good data still faults.
- Lost UART lines can no longer accumulate into a permanent `STOP:MCU_LINK`.
- Status replies are capped at one per 150 ms, and parked polling is 2 Hz. Urgent STOP is unchanged.
- Undervoltage is now visible without the `video` group, and health is logged without Firebase.
- `[BRIDGE DIAG]` and link-loss lines record the reset cause and drop reason in the Pi log.
- Status and scan threads survive unexpected exceptions and fail closed.
- The LiDAR and sensor board are detected from their data on any port, in either order, and after
  re-plugging. No port setting is needed; `~/.config/garby/lidar-port` is no longer read.

## 2026-10-31 — restore requested obstacle bypass behavior

- Removed the extra global motion/Start interlock added on October 30. Front
  and rear LiDAR read-only controls again bypass only their selected distance
  thresholds, retain live readings and preserve pending Start/status queries.
- Confirmed obstacle-sonar disable remains independent: operation can continue
  with that gate bypassed and the servo parked at DEFAULT_VIEW. Changing sonar
  mode still requires stationary IDLE; re-enabling needs fresh clear samples.
- Preserved urgent STOP ordering, Pi path expiry, side-clearance checks,
  missing/stale-data stops, MPU/nearest nudge-clearance gates and latched ESTOP.
- Corrected the operating docs and maintenance guidance to preserve the user's
  requested bypass scope. See the [bypass and workload report](docs/bypass-performance-2026-10-31.md)
  for verification, measurements and deployment status.

## 2026-10-30 — obstacle STOP priority and LiDAR read-only interlock

The read-only motion interlock below is historical and was removed by the
October 31 correction above; queue priority and path-expiry changes remain.

- Fixed a Pi BLE-mailbox defect where a newer blocked PATH snapshot could be
  appended behind sensor/health traffic after replacing an urgent STOP. Blocked,
  stale and malformed path states now retain urgent priority through coalescing;
  queued operator ESTOP remains first and cannot be evicted by PATH traffic.
- Carries each fresh path's monotonic LiDAR evidence deadline into the writer.
  If it expires while queued or waiting for the writer lock, the Pi sends a
  fail-closed STOP instead of the old CLEAR.
- LiDAR read-only controls now preserve live measurements but force STOP and
  block manual start until both threshold checks are re-enabled.
- Recorded the current sensing limit in the UI and docs: generic LiDAR returns
  are not human classification, and software tests do not prove coverage or
  stopping distance. No hardware motion or upload was part of this change.

See [obstacle STOP review and evidence](docs/obstacle-stop-priority-2026-10-30.md).

## 2026-10-30 — BLE Clear logs and live inspection

- Added Clear logs to the BLE/messages tab; displayed history clears while
  incoming messages, connection state and saved runtime logs remain available.
- Retains the newest 250 displayed BLE lines and removes component focus
  outlines while retaining keyboard activation and colored status borders.
- Passed 32 focused Tk tests with no skips. Applied the change to the running
  monitor while preserving its stop latch. See
  [live observations and evidence](docs/live-check-and-ble-clear-2026-10-30.md).

## 2026-10-30 — MPU/LiDAR steering stability

- Added gyro-rate early release to reduce correction overshoot. Prediction can
  cancel/suppress a tap; actual heading retains direction ownership.
- Confirmed corridor targets require independent scans and change gradually.
  Repeated scans cannot earn confidence or advance startup grace/ramp;
  reversing corrections retain confirmation and cooldown.
- Added separate CF/CL/CR nearest-object steering vetoes alongside wall medians,
  with strict acquisition identifier A. Missing fields suppress nudging. PATH
  stops, emergency latch, sonar and executor route/endpoint calibration remain.
- Updated descriptions and project maintenance guidance. See the
  [design, evidence and physical acceptance procedure](docs/nudging-2026-10-30.md).

Deploy the complete updated Pi directory with the S3 sketch and both headers.
Bridge ID is `STRAIGHT-20261030`; dated builds/evidence are in
`artifacts/nudging-2026-10-30/`. Keep the current matching executor firmware;
its source is unchanged by this revision. Software validation does not imply
firmware upload, runtime restart or physical route acceptance.

## 2026-10-30 — operator controls and important SMS notices

- Split the Pi monitor into LiDAR, BLE/messages and Controls tabs. Emergency stop
  remains accessible on every tab; controls use acknowledged state rather than
  treating queued requests as success.
- Added latched emergency stop/release, obstacle-sonar enable/disable with servo
  default-view lock, and confirmed-stop reboot of the Pi, BLE bridge and executor.
  Release does not resume a route; a new manual start is required. The separate
  bin/gas board has no reset endpoint and is excluded from coordinated reboot.
- Added distinct internet/cloud, LiDAR/bin-sensor, thermal/power and established
  controller-link fault/recovery SMS, plus accurate emergency and route notices.
  Health telemetry is coalesced, strict, low-priority and cannot refresh motion
  clearance. Unknown data cannot manufacture a fault recovery.
- Replaced drop-if-busy SMS sending with a ten-slot priority queue and a single
  modem owner. Added spacing, rolling attempt cap, expiry, bounded pre-body
  retries and terminal uncertain-send handling. Recovery pairing covers local
  faults that clear while their text is already in flight. Removed raw modem
  console passthrough to prevent AT command collisions.
- Kept the configured SMS recipient. Texts require cellular registration and
  SIM service; modem submission confirmation is not proof of handset delivery.
- Updated README, architecture, deployment notes and project-local agent/skill
  guidance. See [operator-control results](docs/gui-operator-controls-2026-10-30.md)
  and [SMS notices and current evidence](docs/sms-notices-2026-10-30.md).

Deploy the complete updated `RasPi/` directory and both matching firmware
sketches. SMS build artifacts supersede earlier operator-only images. No new
Android APK is required for these local GUI/firmware changes. Compilation and
host tests do not establish physical stop behavior or live SMS delivery; no
firmware upload, live text or robot reboot was part of this validation.

## Historical coordinated repair — 2026-10-04

This is a coordinated reliability repair for the Pi, BLE ESP32, main ESP32,
and Android app. See `PROJECT_RUNDOWN.md` for findings and
`VALIDATION_RESULTS.md` for verified results.

## Continued full review

- MCU GO confirmations now need 80 ms between them, preventing queued bursts
  from releasing STOP. Corrupt/oversized UART frames invalidate permission.
- HX711 taring and reads run in one background worker. Weight stays unavailable
  until taring succeeds; invalid or older-than-one-second readings reset load
  confirmation. Restored valid samples recover without blocking control.
- Unavailable weight uses `LOAD_CELL:-1.00`, zero Firebase freshness, and an
  unavailable dashboard message.
- Production, the simulator, and the BLE connection probe share an OS lock.
  LiDAR cleanup only terminates GARBY's owned process group, including orphaned
  children. Host-wide process killing and ROS/DDS cleanup were removed.
- LiDAR and sensor UART cannot resolve to the same serial device.

## BLE connection follow-up

The second review addresses the reported Pi/ESP32 connection failure. Discovery
uses the scanned BLEDevice, exact service/name matching, optional strict address,
and explicit Linux adapter selection. Deadlines are 12 s discovery, 30 s GATT
connection, 8 s subscription, and 2 s writes. Subscription and acknowledged
readiness must succeed before reporting connected. Failed candidates and shutdown
are cleaned up; stale callbacks cannot change a replacement session.

StartNotify is explicit for BlueZ/Bleak compatibility. The bridge has 45 s for
initial setup before its first valid write, retains the 10 s established-link
silence limit, and continues the independent 850 ms path STOP watchdog. Advertising
uses explicit scan response, 100–150 ms intervals, automatic disconnect restart,
and a 2 s inactive-advertising retry. Parameter tuning waits for Pi readiness.

Notifications now use default-MTU-safe 20-byte chunks with bounded Pi reassembly,
preserving long return IDs. MCU state queries every 5 s recover missing readiness
notifications. The manual bench producer uses the same discovery/GATT/framing
helpers and shared sequenced mailbox. Pi preflight checks UUID configuration and
BlueZ adapter power; the service starts after `bluetooth.service`.

`RasPi/check_ble.py` isolates discovery, GATT, subscription, readiness write, and
notification receipt without ROS/Firebase. Follow `BLE_CONNECTION_GUIDE.md`.
Deploy **both** bridge firmware and the complete Pi directory, including
`ble_support.py`; an old Pi receiver cannot read the new notification framing.

The review also preserves invalid LiDAR range metadata for fail-closed validation
and removes a remaining redundant adjustment acceptance check. New executable
firmware tests run the entire outbound/return route and the next load/gas cycle
with simulated peripherals. Live connection and physical motion remain unverified.

## Motion and recovery

- Normal obstacles keep the controller in a monitoring wait, servicing UART,
  sonar, watchdogs, and status requests. Recovery resumes the original wheel
  targets after fresh LiDAR permission and, when locally blocked, three distinct
  valid clear echoes.
- Sonar detects a close raw echo immediately, preserves the obstacle latch
  through missing echoes, and uses an 18000 µs listening window. Periodic HALT
  diagnostics identify which clearance condition is missing.
- Both wheel-target command results are checked; one rejection stops both.
  Start acceptance no longer depends on an immediately updated `isRunning()`.
- STOP cannot be erased within the same UART batch. A GO after a stale gap
  restarts confirmation. Timing tolerance cannot authorize motion on old scans.
- Selected avoidance direction determines the nudge command; duration math
  cannot underflow. Confirmation/cooldown waits preserve the active tap.
  Requested taps now match the controller's 260 ms cap.

## BLE, Pi, and parsing

- BLE callbacks post bounded state/data. Loop context owns navigation reset,
  UART, notifications, and connection-parameter changes.
- Overflow, malformed framing, duplicate/old path sequences, and trailing
  delimiters assert STOP. Telemetry cannot inject newline-separated commands.
- One serialized Pi mailbox handles urgent and periodic paths. Older clear
  snapshots cannot replace newer STOP across sequence wrap.
- GATT writes use acknowledgement for path/control and whenever a steering or
  telemetry frame exceeds the negotiated no-response size.
- Old-client disconnect callbacks cannot tear down a replacement connection.
  Failed temporary connections are cleaned up. Scan subscription depth is one.
- Invalid configuration/scan metadata fails closed; an explicitly missing
  LiDAR port no longer falls back to the unrelated sensor device.

## App and return protocol

- Return uses `[RESET:<server-request-timestamp>]` with matching MCU
  ACK/DONE/FAILED. Duplicate IDs do not restart or brake an accepted return.
  Periodic queries of the same ID recover completion notifications lost during
  BLE reconnection.
- An interrupted outbound route cannot trigger a blind return. Route faults
  stay stationary and require local supervised recovery after repositioning.
- The Pi uses conditional Firebase transactions. The app tracks the exact
  request ID, including multiple requests from the same operator.
- Generic IDLE and an obsolete APP boolean cannot produce false completion.
  Accepted returns use a 15-minute completion window; new requests have a
  separate 120-second acceptance window.
- Return requires destination readiness, live Firebase, BLE/LiDAR health, and
  a fresh heartbeat. Cached app data expires after 90 seconds.
- Anonymous-auth waiting is bounded, and startup cancellation/timeout handling
  distinguishes best-effort prewarming from an actual cancelled operation.

## Deployment and compatibility

Update the main ESP32, BLE ESP32, complete Pi directory, then Android with wheels
lifted. Mixed old/new return implementations cannot complete the new request
workflow. UUIDs, UART pins/baud, calibrated route distances/signs, and motor
speed/trim settings are retained.

`Garby_MobileApp/final_w_serial.py` delegates to the authoritative Pi runtime.
`tools/verify_project.py` exports firmware and repeats the software checks.
`RasPi/check_runtime.py`, the launcher, and service/environment examples make
ROS overlay and serial prerequisites explicit.

Physical radio behavior, steering calibration, stopping distances, route
directions, sonar clearance, and deployed Firebase/SIM/FCM services remain
acceptance items. Compilation and host simulation do not establish those results.
