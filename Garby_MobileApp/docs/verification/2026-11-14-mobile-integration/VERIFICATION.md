# Verification evidence — 2026-11-14

[`verification-summary.json`](verification-summary.json) records the commands, outcomes,
target configurations, toolchain versions and remaining limits. Source hashes are in
[`source-after.json`](source-after.json); exact task edits are listed in
[`CHANGES.json`](CHANGES.json). Before-hash records preserve the original working state,
including pending mobile changes that existed when this task started.

Android: **64 passing tests**, zero failures/errors/skips; lint clean; benchmark and unsigned
release APK assembly successful. See `android-final.log`, `unit-tests.xml`,
`lint-results-debug.txt`, `apk-metadata.json` and the two APK signature reports. Both signed
test APKs use the same development certificate. No private signing key is included.

Firmware: **8 host executables, 596 passing assertions**. Both actual ESP32-S3 bridge and
ESP32 executor targets compiled successfully, build `RECONNECT-20261114`. The target logs
and `binary-artifacts.json` identify the locally retained outputs; nothing was uploaded.

The initial production code failed the reconnect sweep regression and both executor
turn-replay assertions; see `firmware-regression-before.log` and
`executor-regression-before.log`. The final host suite passes those cases. The firmware
patch reverse-checks against the edited coordinated workspace without modifying it.

Full Pi discovery completed 464 tests on Windows with **3 failures and 6 errors**. See
`pi-final.log` for the full records. Linux adapter/bash/POSIX/path assumptions account for
several cases; one LiDAR orientation fixture returned no frame and remains unresolved here.
Do not interpret this run as a green Raspberry Pi target suite. An earlier run blocked on
an unmocked Tk confirmation; that fixture is repaired and its test subsequently passed.

Static source structure/JSON and Android quality checks passed. Live Firebase rules,
multi-phone operation, physical phone performance for this feature and robot floor behavior
were not tested. No live database command/rule write, robot command, SMS, firmware upload,
Pi restart or phone installation was performed.

`robot-integration.patch` preserves the eleven sibling source/test/documentation edits,
because those directories do not have Git metadata. It excludes credentials and flasher
caches. MCU requirement: **Both** complete sketches. This follow-up changes only a Pi test;
the previously pending manual-disposal Pi/runtime/rules deployment remains a separate need.

`release-before.json` preserves the original release asset identities and hashes. Test1
retains the original mobile commit and APK; Test2 is the reviewed manual-disposal candidate.
The Windows kernel crash and subsequent Studio freeze are described in the
[review report](../../2026-11-14-integration-review.md); their cause is not established.
