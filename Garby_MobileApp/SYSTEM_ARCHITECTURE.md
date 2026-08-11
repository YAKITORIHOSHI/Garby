# GARBY Android Mobile App - Integration and Safety Architecture

Document status: revised Android-client baseline. This repository contains the mobile app only. Raspberry Pi, BLE-bridge ESP32, main-controller ESP32, robot logs, Firebase security rules, and hardware-test evidence are not present here and therefore are **not verified by this document**.

## 1. Safety boundary

The Android app is a monitoring client and an explicit reset-intent producer. It is **not** a motion authority. It must never infer GO, clear a robot STOP latch, steer motors, or substitute cached/unknown telemetry for current clearance evidence.

Required system-level behavior outside this repository:

1. Missing, stale, malformed, disconnected, or out-of-order robot path data means STOP.
2. A mobile reset request must not itself authorize motion.
3. The Raspberry Pi / BLE bridge / main controller must enforce the robot's normal STOP-before-reset/return behavior and live path freshness independently of the Android app.
4. BLE or LiDAR loss must keep the robot stationary; it must not trigger a blind return.
5. Reset/return commands must be freshness-checked by the robot-side consumer before relay. The Raspberry Pi bridge currently rejects stale, future, or timestamp-less pending reset commands.
6. Human-safe operation still requires a hardwired emergency stop and measured worst-case stopping distance.

## 2. Android runtime ownership

```text
Operator
  -> Firebase anonymous Auth
  -> MainDashboard / Reset screen
      -> DeviceViewModel (monitoring state + freshness)
      -> ResetViewModel (one lifecycle-stable command workflow)
          -> GarbyRealtimeDb
              -> Firebase RTDB
                  -> Raspberry Pi / robot integration (not included here)

Android ConnectivityManager ----> validated internet gate
Firebase .info/connected --------> RTDB connection gate
```

Responsibilities:

- `AuthViewModel`: Firebase anonymous sign-in and sign-out. No embedded user/service passwords. Firebase SDK owns token refresh.
- `NetworkConnectivityManager`: process-wide Android validated-internet signal.
- `GarbyRealtimeDb`: the only app data/control gateway. Attaches RTDB listeners per Flow collector and removes them with collector lifecycle.
- `DeviceViewModel`: aggregates sensor/device streams and re-evaluates timestamp freshness every 5 seconds.
- `ResetViewModel`: owns reset confirmation/send/correlation/timeout state across recomposition and configuration changes.
- Compose screens: presentation and explicit user intent only.

## 3. Firebase interfaces

Database URL and default device ID are centralized in `realtime/Constants.kt`.

Monitoring paths:

```text
/devices/{deviceId}/sensors/level
/devices/{deviceId}/sensors/weight
/devices/{deviceId}/sensors/mq135
/devices/{deviceId}/sensors/mq137
/devices/{deviceId}/sensors/mq4
/devices/{deviceId}/status
/.info/connected
```

Expected sensor shape:

```text
value: numeric finite value
unit: string (optional; falls back to catalog unit)
sensorType: string (optional)
updatedAt: positive epoch-millisecond timestamp
```

Expected device-status shape:

```text
online: boolean
lastSeen: positive epoch-millisecond timestamp
batteryPercent: optional number, clamped to 0..100 for display
wifiRssi: optional number
cpuTemperatureC: optional number
thermalWarning: optional boolean
throttledFlags: optional integer bitmask
bleConnected: optional boolean
lidarHealthy: optional boolean
sensorSerialConnected: optional boolean
```

The supplied deployed database export keeps sensor values under
`/RASPI/VALUES/...` and may not yet contain `/devices/{deviceId}/sensors` or
health fields. The app subscribes narrowly to both exact schemas, selects the
newest valid copy, and treats missing timestamps as stale. The Raspberry Pi
adds device sensor/status nodes during normal operation; this is an additive
rolling migration rather than a destructive database rewrite.

Reset command path:

```text
/devices/{deviceId}/commands/reset
  requestedAt: Firebase server timestamp
  requestedBy: authenticated Firebase UID
  status: "pending" | "ack" | "done" | "failed"
```

Unknown reset status values map to `Unknown`; they do not silently become `Pending`.

## 4. Monitoring integrity and freshness

`STALE_AFTER_MS = 600_000` and `FUTURE_TIMESTAMP_TOLERANCE_MS = 300_000`.

A sensor/device value is current only when its timestamp is positive, no more than 10 minutes old, and no more than 5 minutes into the future. Freshness is recalculated every 5 seconds even if Firebase sends no new callback. This prevents a once-valid reading from remaining visually current forever.
A value is also marked stale immediately whenever the Firebase connection is not `Connected`; timestamp freshness alone is not proof that the monitoring channel is live.

Malformed or non-finite sensor data emits an explicit error state while the Firebase listener remains attached so a later valid update can recover automatically. Missing timestamps and offline sentinels (`999` ultrasonic, `-1` gas) are stale/offline. The dashboard never substitutes `0` for missing weight or gas data. Therefore unavailable data cannot be classified by the normal weight/gas threshold algorithm as a safe low/normal value.

Firebase disk persistence is intentionally disabled. The same RTDB instance carries safety-relevant reset commands; persisting its offline write queue could replay a timed-out command after connectivity returns. Monitoring therefore prefers explicit unavailable/stale state over offline cache convenience.

## 5. Network and RTDB connection behavior

Android network availability requires both:

- `NET_CAPABILITY_INTERNET`
- `NET_CAPABILITY_VALIDATED`

A reset additionally requires Firebase `.info/connected == true` within 5 seconds. `onAvailable()` alone is never treated as proof of internet access.

RTDB listeners retry only Firebase failures classified as transient (`DISCONNECTED`, `NETWORK_ERROR`, `UNAVAILABLE`). Permission/auth/schema errors are terminal and surface to the UI instead of retrying for minutes.

Latest-value sensor/device streams are conflated to avoid UI backlog.

## 6. Reset command algorithm

A reset is deliberately harder to trigger than ordinary navigation:

1. Operator opens Reset screen.
2. App requires an explicit confirmation dialog explaining that the command can initiate the robot return workflow.
3. `ResetViewModel` requires a current Firebase-authenticated UID.
4. Android must report validated internet.
5. `GarbyRealtimeDb` waits for Firebase `.info/connected == true`.
6. Any older outstanding Android RTDB writes are purged.
7. The app reads the previous reset `requestedAt` value.
8. The app atomically writes `requestedAt = ServerValue.TIMESTAMP`, authenticated `requestedBy`, and `status = pending`.
9. The UI waits only for a `done` or `failed` record whose `requestedBy` matches and whose `requestedAt` is newer than the pre-request marker. An old terminal state cannot satisfy the new request.
10. If the write times out/fails, outstanding writes are purged to reduce delayed replay risk.
11. If robot completion is not reported within 60 seconds, the UI reports **state unknown** and instructs physical verification rather than claiming success/failure.

The system back action is blocked while reset transmission/correlation is active so UI lifecycle cancellation cannot silently abandon the command workflow.

### Robot-side consumer requirement

App-side purge/correlation cannot prove that a server-committed command was not already consumed. The Raspberry Pi reset bridge therefore rejects reset commands older than 120 seconds, more than 30 seconds in the future, or missing a timestamp unless the live `/APP/isReadyToReset` compatibility flag is present.

## 7. Authentication and secrets

No Firebase user password or Raspberry-Pi/API Basic Auth credential is embedded in source.

The previous legacy direct HTTP sensor client was removed because it was unused by the active architecture and disabled TLS certificate/hostname verification. Firebase RTDB is the sole Android data/control channel.

Any credentials previously embedded in source or released APKs must be rotated externally; deleting them from the new source does not revoke an already exposed credential.

Firebase client configuration (`google-services.json`) is not a substitute for server-side authorization. Production Firebase RTDB rules must require authenticated reads and must restrict reset writes to authorized operators/devices. Those rules are not included in this archive and remain a release prerequisite.

Android application backup is disabled and cleartext network traffic is disabled in the manifest.

## 8. UI and performance architecture

- The old unbounded custom executor pools were removed; Compose/coroutines/Firebase own their intended execution models.
- The gauge performs one bounded animation instead of nested/double animations.
- Firebase listeners are lifecycle-bound callback Flows rather than singleton listeners removed from memory callbacks.
- No periodic manual Firebase token-refresh loop exists.
- Unused HTTP, Gson, Kotlin serialization, Analytics, Crashlytics-buildtools, DataStore, Window, AppCompat, ConstraintLayout, and Material-view dependencies were removed from the app module.
- Release builds enable R8 minification and resource shrinking.
- Montserrat is reduced to the three weights actually used by production UI.
- Large static PNGs were replaced by much smaller WebP resources with unchanged Android resource IDs.
- The connection banner shows optional Pi thermal, BLE, LiDAR, and sensor-board
  health when the updated bridge publishes those fields. Missing health fields
  remain hidden for compatibility with older database snapshots.

## 9. Compatibility

The RTDB reset object keeps the existing field names and status strings. `requestedBy` is the authenticated Firebase UID for auditability and correlation. The Pi consumer treats it as an opaque string rather than requiring one literal value.

Monitoring now treats `/devices/{deviceId}/sensors/level` as ultrasonic distance in centimeters, not percentage trash-bin fill. Weight and gas thresholds match the firmware's 1.0 kg load trigger and MQ warning/danger ranges more closely.

## 10. Required release validation

Software before signing/release:

- Build `testDebugUnitTest`, `lintDebug`, `assembleDebug`, and `assembleRelease` using the project's real Android SDK/Gradle environment.
- Verify Firebase security rules with an authorized and unauthorized test account.
- Test malformed/missing sensor values, stale timestamps, future timestamps, Android offline, Firebase disconnected, auth expiry/sign-out, and app background/foreground.
- Test reset confirmation/cancel, no-internet reset, Firebase-disconnected reset, success, robot-reported failure, terminal timeout, stale previous `done`, and repeated requests.
- Verify R8 release output and signing configuration.

Robot integration before floor use:

- With wheels lifted, verify an Android reset cannot bypass STOP and that a reset/return still requires fresh motion permission.
- Verify BLE disconnect and LiDAR stale behavior remain stationary.
- Verify stale/replayed reset handling on the Pi/firmware with old `pending`, missing timestamp, and future timestamp commands.
- Verify physical return direction independently.
- Perform hardwired E-stop and worst-case stopping-distance tests before operation around people.
