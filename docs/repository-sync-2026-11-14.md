# Coordinated repository synchronization — 2026-11-14

This publishes the maintained testing workspace to `YAKITORIHOSHI/Garby` and embeds the
current `YAKITORIHOSHI/Garby-Mobile-App` snapshot at commit
`1304f9803b8f347c0347fc7397e8e13d396ef609`. The source directories on the shared drive had
no parent Git metadata, so synchronization used a separate local checkout. Credentials,
Firebase Admin files, runtime stop/request state, flasher caches, local SDK paths and build
outputs are excluded. The standalone mobile repository is also committed and pushed.

The published source includes the pending confirmed manual-disposal feature, prior heading
recovery/acquisition-age repairs, the reviewed BLE turn-state recovery repair, and the
current optimized/fullscreen mobile UI. Obsolete embedded mobile files were removed to
match the standalone app; dated reports were retained under its history directory.

Every flat production Python/C++ file in the reviewed Pi, bridge, executor and tools folders
matches the maintained workspace. The embedded app is an exact tracked export of the
mobile commit. Native builds used matching local source snapshots; synchronization changed
no production algorithm after those builds. Changed/copied files are recorded in
`repository-sync-2026-11-14.json`.

Verification: 64 Android unit tests passed, lint reported no issues, both APK variants built;
eight firmware host programs passed 596 assertions; actual ESP32-S3 and ESP32 targets compiled
with one worker. The final source export also passed structural/JSON checks and Python syntax
checks. Detailed evidence is in
`Garby_MobileApp/docs/verification/2026-11-14-mobile-integration/`.

The full Pi suite on Windows completed 464 tests with 3 failures and 6 errors, including
Linux adapter/bash/POSIX/path expectations and one unresolved scan fixture. Linux target,
live Firebase permissions/multi-phone behavior, this feature's phone performance and physical
robot acceptance remain unverified. No live database/rules change, robot command, firmware
upload, Pi restart, SMS or app installation was performed.

The existing system ZIP is preserved byte-for-byte as `GarbyMostLatest-Test1.zip`, SHA-256
`c207aba6f55daa513b060611796ea19c9e135d0954d907b96810f7b5ef77552d`. Test2 packages this
repository revision with the matching optimized installable APK. Both remain on the same
GitHub release for separate testing; Test1 is historical and does not include the repairs.

MCU upload requirement for Test2: **Both** complete sketches, build `RECONNECT-20261114`.
Deploy the entire matching `RasPi/` directory and its requirements for the pending disposal
feature, and review/deploy the disposal Firebase rules separately. Installed firmware and
prior deployment identities remain unknown. Use the existing supervised acceptance guide.
