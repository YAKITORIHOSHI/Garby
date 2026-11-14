# GARBY — Android operator app

The phone app for GARBY, the autonomous waste-collection robot. It shows what
the robot is doing and lets an operator request manual disposal or send it back to base from the collection point. It
talks only to Firebase; it never drives the robot directly, and GARBY's own sensors and the
Pi monitor's emergency stop always decide whether it moves.

## What it does

- **Connect:** signs in anonymously with Firebase. No password or service key is in the app.
- **Dashboard:**
  - **Robot:** where GARBY is (idle at base, driving to the collection point, waiting there,
    returning, route fault), shown on a base-to-collection-point track, and whether the Pi is
    still reporting.
  - **Bin:** weight against the 4 kg dispatch threshold, and the bin sensor's distance reading.
  - **Air:** MQ135, MQ137 and MQ4 readings in the robot's own bands, and whether the gas danger
    rule is met.
  - **System:** Pi heartbeat, cloud, phone internet, Bluetooth link to the robot, LiDAR, sensor
    board, Pi temperature and power.
  - Anything older than 90 s is marked stale; nothing missing is shown as zero.
- **Return to base:** a checklist of what must be true before sending (GARBY waiting at the
  collection point, reporting, linked and scanning; cloud and phone online; no return already
  running), a confirmation with the safety points, and live progress: sent, accepted by GARBY,
  back at base, or failed with the reason in plain words.
- **Manual disposal:** on the dashboard, press **Manual disposal**, then confirm **Yes, start
  disposal** to send an idle GARBY to the collection point before the bin, gas or weight
  thresholds are reached. Cancel sends nothing. GARBY's own obstacle, heading and emergency
  stop checks still control motion. The latest request shows sent, accepted, rejected or
  delivery not confirmed; accepted does not mean arrival. Follow robot status, empty the bin
  at the collection point, then use the existing return action.
- **Notifications:** receives GARBY push messages when a sender is configured (none is
  deployed yet).

The app opens fullscreen, with Android's status and navigation bars hidden. Swipe inward
from a screen edge to reveal the system controls temporarily.

## Requirements

- Android 8.0 (API 26) or newer. All devices use smoked glass without live blur or continuous
  decorative animations. Memory and CPU capacity determine image sizes and background concurrency.
- Android Studio 2026.1 or newer with JDK 21, and Android SDK platform 37.2 (Android Studio or
  the build downloads it).
- `app/google-services.json` for the `garby-thesis` Firebase project (checked in; a client
  configuration, not a secret). Anonymous sign-in must be enabled in that project.

## Build and run

Open this folder in Android Studio and run the `app` configuration, or from a terminal:

```bash
./gradlew :app:assembleDebug           # debug APK
./gradlew :app:testDebugUnitTest       # unit tests
./gradlew :app:lintDebug               # lint
./gradlew :app:assembleRelease         # minified, unsigned release APK
./gradlew :app:assembleBenchmark       # minified, signed local performance-test APK
```

Outputs are in `app/build/outputs/apk/`. If this checkout is on a network share, add an absolute
`garby.buildRoot=<local folder>` to `local.properties` and outputs go there instead (see
`settings.gradle.kts`). Release signing and distribution are done outside this project.
The `benchmark` APK uses the local development signing key, runs without debug instrumentation,
and is intended for phone performance checks. Installing it replaces the same app and preserves
its data when the existing signing key matches; it is not a production-signed release.

For normal use and performance measurements, install the optimized **Most Latest** APK or
select the `benchmark` build variant in Android Studio. The `debug` variant includes Compose
instrumentation and can scroll substantially more slowly; pressing Run with that variant
replaces the optimized installation with a debug build.

`python -B tools/static_quality_check.py` checks the app's security and safety rules without an
Android SDK. In the coordinated robot workspace, its root `tools/verify_project.py` also runs
the Android checks alongside the robot's checks.

To verify a manual-disposal build from this folder in PowerShell:

```powershell
python -B tools/static_quality_check.py
.\gradlew.bat :app:testDebugUnitTest :app:lintDebug :app:assembleBenchmark --console=plain
```

Use the optimized `benchmark` variant for a low-end phone check. The new versionCode is 6.
Manual disposal also needs the matching Pi/MCU update and permission for `commands/dispose`
in the actual Firebase rules. See [FIREBASE_CONFIGURATION.md](FIREBASE_CONFIGURATION.md)
and the coordinated workspace's [deployment notes](../docs/manual-disposal-2026-11-13.md).

The release provides `GarbyMostLatest-Test1.zip` (the previous app revision) and
`GarbyMostLatest-Test2.zip` (manual disposal). Extract each ZIP to a local disk before
opening it in Android Studio. Each contains its matching APK for installation without a build.
Both use the same Android package; compare them on separate devices or installations.

## Project layout

| Path | Contents |
| --- | --- |
| `app/src/main/java/com/thesis/garby/` | Screens (`StartScreen`, `MainDashboard`, `ReturnScreen`), `MainActivity`, `ReturnViewModel` |
| `.../realtime/` | Firebase gateway and the pure, unit-tested rules: return gate and progress, executor thresholds, Pi health, freshness |
| `.../ui/glass/`, `.../ui/theme/` | Glass design system (frosted panels, bars, gauges, route track) and the dark crimson theme |
| `.../firestore/`, `.../network/`, `.../notifications/` | Anonymous sign-in, validated-internet monitor, FCM receiver |
| `app/src/test/` | Unit tests |
| `design/` | Source artwork (the full-size logo); not packaged |
| `tools/static_quality_check.py` | Source-only release checks |
| `docs/` | Performance investigation and verification summaries |

## Documents

- [SYSTEM_ARCHITECTURE.md](SYSTEM_ARCHITECTURE.md): how the app works, what it reads and writes,
  the return workflow, freshness rules and build settings.
- [FIREBASE_CONFIGURATION.md](FIREBASE_CONFIGURATION.md): Firebase project, data paths and the
  security boundaries the database rules must enforce.
- [PATCH_NOTES.md](PATCH_NOTES.md): what changed in each version.
- Robot firmware, Pi deployment and physical acceptance are maintained in the coordinated GARBY
  workspace, separately from this Android repository.
