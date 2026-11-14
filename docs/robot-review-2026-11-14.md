# Robot review and repository ownership — 2026-11-14

This repository owns the Pi runtime, BLE bridge, motor executor, robot tools and robot
verification. The Android source and APK are maintained exclusively in
[Garby-Mobile-App](https://github.com/YAKITORIHOSHI/Garby-Mobile-App).
The embedded app copy and APK were removed from current robot source and Test2 packaging.
Test1 is preserved byte-for-byte as a historical combined-workspace archive.

The review reproduced and repaired loss of turn-sweep protection across BLE reconnect.
The bridge preserves turn scope and withholds GO while mode is unknown after bridge boot;
the executor replays its whole turn scope before readiness. Repeated notices are idempotent.
The original source failed reconnect and missing-replay regressions; the final host suite
passed. The Pi durable-claim/query-only disposal path matched the intended wire contract.
A Tk test fixture now mocks its confirmation rather than waiting on a modal dialog.

Retained verification: eight host executables / 596 passing assertions; both actual ESP32
targets compiled, build RECONNECT-20261114. Source and binary hashes plus logs are in
[robot evidence](verification/2026-11-14-robot-review/VERIFICATION.md).
The full Windows Pi suite completed 464 tests with 3 failures and 6 errors, including Linux
platform expectations and one unresolved scan fixture. Physical acceptance, installed image
identity and live cloud rules remain unverified. No upload, robot command, SMS, cloud write
or Pi restart was performed.

Repository separation does not modify motion/runtime production source. Robot verification
runs independently. `tools/verify_project.py --mobile-repo <external-checkout>` optionally
checks a separately cloned app; the cross-repository Firebase test is otherwise explicitly
skipped. Neither path requires app source to be embedded here.

MCU upload requirement for the earlier turn repair: Both complete sketches. Repository
separation requires no additional flash or Pi runtime deployment; the earlier disposal
feature still requires the matching entire RasPi directory and reviewed cloud rules.
