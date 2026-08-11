# GARBY Mobile App - Validation Results

Validation date: 2026-08-11

## Passed host/source checks

### Static release-invariant checker

Command:

```bash
python3 tools/static_quality_check.py
```

Result:

```text
STATIC_QUALITY_CHECK: PASS
Kotlin files checked: 22
Bundled font files: 3
```

The checker verifies high-value regressions including embedded credential constants, trust-all TLS code, RTDB disk persistence, legacy direct-HTTP code, synthetic zero telemetry fallback, reset gating/correlation primitives, manifest backup/cleartext settings, removed dependencies, and local resource references.

### Pure Kotlin realtime/control logic

Compiled and executed `Constants.kt`, `Models.kt`, and `SensorFreshness.kt` with host `kotlinc` plus a small validation harness.

Result:

```text
PURE_LOGIC_CHECK: PASS
```

Covered:

- current timestamp accepted;
- stale timestamp rejected;
- excessive future timestamp rejected;
- reset status parsing recognizes case-insensitive known values;
- malformed reset status maps to `Unknown`;
- reset marker rejects an old terminal command and accepts a newer matching request.

### Sensor threshold boundaries

`SensorStatus.kt` was host-compiled with a minimal Compose `Color` stub solely to execute its pure threshold functions.

Result:

```text
SENSOR_STATUS_CHECK: PASS
```

Weight status boundaries are 0.70 kg and 1.00 kg, with 1.00 kg matching the
executor dispatch threshold. Gas boundary tests retain the reviewed MCU-aligned
ranges.

### Resource/config validation

- `gradle/libs.versions.toml` parsed successfully.
- Android manifest and all remaining XML resources parsed successfully.
- Referenced local drawable/font resources resolve.
- No duplicate resource stem was introduced.

Result:

```text
RESOURCE_CONFIG_CHECK: PASS
```

### Kotlin parser probe

All production Kotlin files were passed to the host Kotlin compiler without Android/Compose/Firebase classpaths. Type resolution therefore fails as expected, but the diagnostics were scanned for Kotlin parser/syntax failures.

Result:

```text
KOTLIN_PARSER_PROBE: PASS
(no syntax-like diagnostics; unresolved external Android symbols are expected)
```

## Android Gradle build

Executed with the checked-in Gradle wrapper, JDK/SDK configuration, and current
sources:

```powershell
.\gradlew.bat --no-daemon testDebugUnitTest
.\gradlew.bat --no-daemon lintDebug
.\gradlew.bat --no-daemon assembleDebug assembleRelease
```

Result:

- `testDebugUnitTest`: **PASS** (13 tests, 0 failures/errors/skips)
- `lintDebug`: **PASS**
- `assembleDebug`: **PASS**
- `assembleRelease` including R8/lint-vital: **PASS**

Generated artifacts:

- `app/build/outputs/apk/debug/app-debug.apk`
- `app/build/outputs/apk/release/app-release-unsigned.apk`

The release APK is unsigned and still requires the project's production signing
process before distribution.

## Integrated robot-side validation

The obsolete archive-only audit has been superseded by validation from the
complete parent workspace. The final main ESP32 executor and BLE receiver both
target-compile with zero GARBY source warnings. Raspberry Pi reliability tests
pass 15/15, its production Python compiles, the supplied Firebase leaf paths
are all preserved, and cross-protocol/static safety invariants pass.

See the parent `VALIDATION_RESULTS.md` for exact build memory, commands, and
coverage. Physical stopping distance, wheel trim, LiDAR orientation, radio
range, power stability, and closed-enclosure thermal performance still require
the documented robot hardware tests.

## Required validation on the real Android/robot environment

1. Verify the unsigned R8 release build after applying the production signing
   configuration, including Firebase Auth, RTDB monitoring, and reset.
2. Verify Firebase RTDB rules reject unauthenticated/unauthorized reset writes.
3. Test offline/reconnect, stale sensor timestamps, malformed sensor records,
   missing device status, sign-out/re-authentication, and app background/foreground.
4. Test reset cancel/confirm, Firebase disconnect, successful `done`,
   robot-reported `failed`, stale previous `done`, and 60-second no-response timeout.
5. With wheels lifted, verify mobile reset cannot bypass firmware STOP and
   return still requires fresh path permission.
6. Complete the robot hardware checks described in `SYSTEM_ARCHITECTURE.md`
   before floor deployment.
