# GARBY Integrated Validation Results

Validation date: 2026-08-11

## Main ESP32 executor

An external, root-sketch-only PlatformIO CI build targeted `esp32dev` with
`-Wall -Wextra` and completed successfully.

- RAM: 27,948 bytes / 327,680 bytes (8.5%)
- Flash: 333,749 bytes / 1,310,720 bytes (25.5%)
- GARBY source warnings: 0
- Remaining warnings: unchanged third-party ESP32Servo/HX711 warnings

The final build includes reset retry idempotence and confirmed 60 cm sonar
latching in straight, turn, and adjustment motion contexts.

## ESP32 BLE receiver

An external PlatformIO build used `esp32dev`, Arduino framework
3.20017.241212, NimBLE-Arduino 2.5.1, and `-Wall -Wextra`.

- RAM: 39,356 bytes / 327,680 bytes (12.0%)
- Flash: 613,057 bytes / 1,310,720 bytes (46.8%)
- GARBY source warnings: 0
- Remaining warnings: six identical third-party NimBLE macro warnings

The build copy was byte-identical to the production root sketch. Structural,
packet grammar, stale-data, `C/O/H/S`, startup ramp, and strict numeric
fail-stable checks passed. No managed project config, backup `src`, or build
directory was used or modified.

## Raspberry Pi bridge

Commands:

```bash
python -m unittest -v test_bridge_core.py
python -m py_compile bridge_core.py final_w_serial.py final_w_serial-simulator.py
```

Result: 15 tests passed; all three Python files compiled.

Coverage includes:

- one-shot unavailable sentinels and recovery rearming;
- partial sensor failure and bounded live publish cadence;
- bounded reconnect/retry behavior and Firebase coalescing;
- retry backoff that cannot be bypassed by new telemetry;
- robust LiDAR safety/wall representatives and tilt clamping;
- gap-free nearest-sector boundaries across a 10,800-angle sweep;
- former between-cone blind-angle obstacle retention;
- trash-level ultrasonic isolation from corridor path classification;
- thermal unknown/normal/warning behavior.

## Android and Firebase

Commands:

```powershell
python tools\static_quality_check.py
.\gradlew.bat --no-daemon testDebugUnitTest lintDebug assembleDebug assembleRelease
```

Results:

- static quality checker: pass (22 Kotlin files, 3 fonts)
- unit tests: 13 passed, 0 failed/errors/skipped
- Android lint: pass
- debug APK: pass
- minified/R8 release APK: pass (unsigned)

Artifacts:

- `Garby_MobileApp/app/build/outputs/apk/debug/app-debug.apk`
- `Garby_MobileApp/app/build/outputs/apk/release/app-release-unsigned.apk`

The supplied database export contains 16 leaf paths. The sanitized project
template preserves all 16 and adds the device sensor/status mirrors. Runtime
FCM values are intentionally empty. Cross-component checks confirmed the
`P:`, `S:`, `SENSOR:`, reset, ready, nudge, STOP, GO, and IDLE tokens plus the
executor return-reset guard and sonar motion-context guards.

## Not certifiable without hardware

Automated success does not certify stopping distance, wheel trim, LiDAR mount
orientation, radio range, power integrity, or enclosure temperature. Complete
every test in `DEPLOYMENT_AND_ACCEPTANCE.md`, especially the wheels-lifted
fail-safe tests, measured obstacle clearance, five repeated corridor runs, and
30-60 minute closed-enclosure thermal run before unattended use.
