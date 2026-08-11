# GARBY Mobile App - Patch Notes

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
