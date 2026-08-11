# GARBY Mobile App Full Audit Report

## Scope

Audited the Android/Kotlin project from application architecture through Firebase control flow, lifecycle/state handling, security, networking, UI data integrity, performance, build configuration, and package hygiene.

This workspace now contains the GARBY Raspberry Pi bridge, BLE bridge, main-controller firmware, Android app, and Firebase RTDB shape. Physical test evidence and Firebase RTDB security rules are still external validation items.

## Findings and disposition

### Critical - fixed in source

1. **Embedded Firebase account credential**
   - Old source embedded a Firebase email/password and auto-signed in.
   - Fix: removed all embedded user credentials; the prototype now uses Firebase anonymous Auth and Firebase SDK token management.
   - External action still required: rotate the previously exposed Firebase credential and invalidate/disable any account no longer needed.

2. **Embedded direct-Pi/API credential + disabled TLS verification**
   - Legacy HTTP code contained another plaintext credential and trusted every certificate/hostname.
   - The active app did not use that client.
   - Fix: removed the legacy client and its Apache HttpClient/Gson dependencies; Firebase RTDB is now the only app data/control channel.
   - External action still required: rotate the previously exposed API/Pi credential if it is valid anywhere.

3. **Safety-relevant reset write could be persisted/replayed offline**
   - RTDB disk persistence was globally enabled while the same database sends reset commands.
   - A timed-out queued write could later synchronize after connectivity recovery.
   - Fix: disk persistence disabled; reset requires Android validated internet + Firebase connected state; outstanding writes are purged before a new reset and on failed/timed-out transmission.
   - Robot-side follow-up: the Raspberry Pi reset bridge now rejects stale, future, or timestamp-less pending reset commands before relaying BLE reset.

### High - fixed in source

4. **Unavailable weight/gas silently became zero and therefore safe/low**
   - Fix: loading/error/stale/missing states render explicitly; threshold algorithms run only on a fresh finite reading.

5. **Freshness constant existed but was never enforced**
   - Fix: timestamps are checked on receipt and re-evaluated every 5 seconds; sensor values older than `STALE_AFTER_MS` (10 minutes) are stale.

6. **Reset workflow was owned by a Composable coroutine**
   - Navigation/configuration could cancel monitoring of a safety-relevant command.
   - Fix: dedicated `ResetViewModel`; system back is blocked while a reset exchange is active.

7. **Reset could match a stale previous terminal state**
   - Fix: pre-request timestamp marker + authenticated requester correlation. Only a newer matching `done`/`failed` completes the new request.

8. **Unknown reset protocol values were treated as Pending**
   - Fix: explicit `ResetStatus.Unknown`.

9. **Firebase retry loop retried permanent errors up to 50 times**
   - Fix: retries reduced/bounded and restricted to transient RTDB error codes.

10. **Connection listener was detached on UI-hidden memory callback and could stay stale**
    - Fix: `.info/connected` is now a collector-owned `callbackFlow`; no global listener cleanup race.

11. **Manual auth token-refresh loop duplicated Firebase SDK behavior and could create contradictory auth state**
    - Fix: removed manual refresh jobs. Firebase Auth owns token lifecycle.

12. **Android network `onAvailable` was treated as internet-ready**
    - Fix: every state refresh verifies both INTERNET and VALIDATED capabilities.

### Medium / optimization - fixed

13. Unbounded/dead custom executor pools removed.
14. Double gauge animation removed and zone calculation made synchronous.
15. Unused runtime dependencies/plugins removed.
16. Release R8 minification and resource shrinking enabled.
17. Backup disabled; cleartext traffic disabled; unnecessary `CHANGE_NETWORK_STATE` permission removed.
18. Font payload reduced from about 7.2 MB to about 1.0 MB by keeping only used Montserrat weights.
19. Three static PNG drawables (~4.9 MB) converted to WebP (~0.6 MB total) with unchanged resource names.
20. IDE/build/cache/debug artifacts are excluded from the clean deliverable.

## Deliberately unchanged

- Weight and gas thresholds were aligned closer to current firmware behavior, but still need physical calibration against the final bin load cell and MQ sensors.
- Existing Firebase path names and reset status strings were preserved to avoid an uncoordinated protocol break.
- Legacy Firebase paths were preserved while adding app-friendly `/devices/{deviceId}` mirrors.

## Remaining release blockers / external actions

1. Rotate both previously embedded credentials. Treat them as compromised even if this revised source no longer contains them.
2. Verify and harden Firebase RTDB security rules. No rules file was supplied; client-side authentication cannot compensate for permissive backend rules.
3. Confirm Firebase RTDB security rules only allow authorized users/devices to read/write the expected paths.
4. Run the wheels-lifted and physical safety tests in `SYSTEM_ARCHITECTURE.md` before floor deployment.

## Validation performed in this environment

- Security source scan: no embedded password/username constants, trust-all TLS manager, or hostname-verifier bypass remain in `app/src/main/java`.
- Legacy dependency/reference scan: no direct-Pi HTTP helper or custom executor references remain.
- Android validation: `python tools\static_quality_check.py`, `.\gradlew.bat testDebugUnitTest`, `.\gradlew.bat lintDebug`, and `.\gradlew.bat assembleDebug` passed.
- Raspberry Pi validation is recorded in the parent `VALIDATION_RESULTS.md`:
  15/15 reliability tests pass and the production bridge, core, and simulator
  compile. The Android-root `final_w_serial.py` is now only a compatibility
  launcher to that single reviewed production implementation.
- Firmware validation: both active ESP32 root sketches target-compile with zero
  GARBY source warnings; exact memory results are in the parent validation file.
- Asset conversion: resource IDs retained; lossless WebP used for the app icon and WebP used for static backgrounds.
- Remaining firmware risk: the repo contains a managed/stale `NAPHTALI_CODE_V2/src` mirror that Arduino CLI will compile if the full folder is used directly. The active root sketch files are the verified source per local `AGENTS.md`.

### Current build status

No Android build limitation remains in this workspace. Thirteen debug unit
tests, lint, debug APK assembly, and the minified/R8 unsigned release APK build
all pass.
