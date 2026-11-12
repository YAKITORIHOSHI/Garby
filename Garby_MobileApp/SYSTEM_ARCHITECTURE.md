# GARBY Android integration — 2026-10-04

The complete Pi, bridge, controller, and Android source are present in this
workspace. [Root architecture](../SYSTEM_ARCHITECTURE.md) is authoritative for
the coordinated system. [Validation](../VALIDATION_RESULTS.md) records software
checks; live Firebase and hardware remain external acceptance items.

## Responsibilities

Android authenticates through Firebase, displays telemetry, marks cached data
stale, and requests return. It cannot grant GO, clear an obstacle, steer a wheel,
or substitute cached values for sensor clearance.

Compose screens use lifecycle-aware ViewModels. `GarbyRealtimeDb` owns narrow
Firebase listeners and command writes; `SensorFreshness` and the realtime
models hold the pure freshness/command rules exercised by unit tests. Firebase
callbacks and flows publish state; motion remains on the robot.

## Return workflow

1. Enable return only with destination readiness, actual Firebase connectivity,
   a fresh heartbeat, and BLE/LiDAR health. This is checked again at submission.
2. Write `devices/garby-bin-01/commands/reset` with server `requestedAt`,
   operator `requestedBy`, `pending` status, and cleared prior outcome fields.
3. Read the server-issued timestamp and track that exact request ID. A newer
   request by the same operator cannot complete the current workflow.
4. Display ACK only from the Pi's matching MCU acceptance. DONE/FAILED require
   matching controller outcomes; generic IDLE or a legacy boolean cannot finish
   the request.
5. Allow 15 minutes for accepted return completion. Timeout means the outcome is
   unverified. New-request acceptance on the Pi is separately limited to 120 s.

The MCU accepts a new return only after completed outbound. Duplicate IDs are
idempotent. A route fault or interrupted outbound requires supervised physical
recovery from a known position, not another app retry. Both ESP32s and the Pi
must be upgraded with the app for this protocol.

## Freshness and startup

Dashboard values expire after 90 s, with 30 s future-clock tolerance. Unknown
values remain unavailable rather than synthetic zeros. Negative weight/gas
sentinels use zero freshness and an unavailable dashboard message. Connectivity and
readiness errors disable return. Auth initialization has a bounded wait;
best-effort prewarming handles timeout separately from actual cancellation.

The current Android Firebase client file and credential boundaries are recorded
in [Firebase configuration](FIREBASE_CONFIGURATION.md). The Android client
configuration is synced from `RasPi/Firebase/google-services.json`; service
account keys and account passwords stay out of the APK.

The app has no embedded service-account credential, trust-all TLS client,
direct motor API, or Firebase disk persistence in the active implementation.
Live anonymous-auth configuration, device/operator authorization, and deployed
RTDB rules must be checked in the Firebase project.

## Build and acceptance

From the repository root, run `python -B tools/verify_project.py`. It runs
Android unit tests, debug/release assembly, lint, and all robot software checks.
Release assembly is unsigned; production signing/distribution is external.

FCM receiving code exists, but this repository does not supply a deployed cloud
notification sender. Phone/backend acceptance includes authentication, offline
cache expiry, readiness, exact return outcomes, reconnect, and notification
delivery. Follow [deployment and acceptance](../DEPLOYMENT_AND_ACCEPTANCE.md).

Historical Android audit/patch reports are retained for context and point to
the current coordinated release.
