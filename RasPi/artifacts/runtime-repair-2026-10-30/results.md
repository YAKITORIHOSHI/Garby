# Runtime and Firebase repair verification — 2026-10-30

Implemented and verified on the Raspberry Pi:

- Installed firebase-admin 7.7.0 and dependencies in the user-local package directory loaded by run_bridge.sh.
- Added firebase_support.py. The runtime, simulator, preflight, and Firebase probe locate the sole service-account JSON in Firebase/ beside the source files. Android/web configurations are ignored. Explicit GARBY_FIREBASE_CREDENTIALS overrides remain supported; relative paths use the runtime directory. Missing or ambiguous keys report actionable errors.
- Verified discovery after relocating the runtime directory and renaming the credential file.
- Display selection now happens before background workers start. Missing DISPLAY, unavailable Tk, and an unusable display select headless operation.
- Replaced the launcher overlay's fixed home directory with the current user's HOME.
- Fixed an existing 800x480 sidebar overflow by reclaiming 16 pixels of static padding. The previously failing test_small_display_keeps_values_and_sensors_visible now passes.

Firebase and mobile integration:

- Authenticated Admin SDK heartbeat read passed.
- Isolated diagnostic write/read/delete passed; deletion was verified.
- Android's supplied and existing google-services.json files match. Project, database URL, registered package, device ID, five sensor mappings, freshness/sentinels, health fields, and structured return-command contract match the production Pi bridge.
- Found the mobile app's signInAnonymously flow rejected with ADMIN_ONLY_OPERATION. Firebase Anonymous sign-in was disabled. Enabled only signIn.anonymous.enabled and verified the configuration readback.
- Retested anonymous sign-in using the Android client configuration. All ten expected live database reads returned HTTP 200. The temporary authentication account was removed.
- No mobile source or database-schema rewrite was needed. Existing robot/return command records were not used to trigger a test return.

Validation:

- Full Python suite with a real local Tk display: 190 tests passed, zero skipped.
- Headless suite: 190 tests, 17 desktop-dependent tests skipped; all remaining tests passed.
- Android static quality check: PASS (22 Kotlin files, 3 font files).
- Launcher shell syntax: PASS. Modified/new Python source syntax: PASS.
- Real launcher invoked from /tmp with DISPLAY unset: automatic headless selection, Firebase initialization, ROS LiDAR startup, real LiDAR scans, and clean shutdown/exit 0 all passed.
- Real GUI launcher startup also passed with a local desktop display: Firebase initialized, real LiDAR scans arrived, no traceback occurred, and shutdown exited cleanly with code 0.
- Both launcher smoke tests used an unavailable BLE test adapter to prevent a robot connection. Normal hci0 remains configured; its powered-state preflight passes. No smoke-test adapter setting was persisted.
- Read-only code review found no critical or important regression or production Android/Pi schema mismatch.

Remaining physical acceptance:

- The sensor board is unplugged, as confirmed by the user. Preflight passes ROS, LiDAR, Bluetooth, Python dependencies, and Firebase; its only failure is the missing GARBY_SENSOR_SERIAL_PORT. Attach the board and configure its real port if it is not an auto-detected Pi UART. LiDAR was never assigned as the sensor-board port.
- Phone UI/APK and real return/motion testing were not performed. This host does not have an Android/Java build toolchain; no Android source change required rebuilding for this repair.

Start from RasPi/ with: bash run_bridge.sh
Use --headless explicitly when desired; a desktop/forwarded DISPLAY enables the GUI.
