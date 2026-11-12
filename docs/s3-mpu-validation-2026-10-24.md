# ESP32-S3 and MPU heading integration validation — 2026-10-24

The BLE bridge now targets ESP32-S3 Dev Module. Its MPU6050 supplies relative
heading for straight-segment nudges. Lidar path permission and obstacle clearance
remain authoritative for collision prevention; lidar geometry no longer chooses
steering direction. The original `Test/MPU_ESP32-S3-DEV.ino` is unchanged.

## Results

| Check | Result |
| --- | --- |
| `python3 -B tools/verify_project.py --host-only` | PASS, exit 0 |
| Pi unit tests | 89 passed |
| Actual-source firmware host checks | 108 passed, across bridge, controller, MPU reader and connected-system executables |
| Source audit | 75 passed, 1 existing physical-route-direction warning, 0 failed |
| Source structural check and Android static quality check | PASS |
| ESP32-S3 bridge target compile | PASS, 577640 bytes flash (44%), 37028 bytes globals (11%) |
| ESP32 controller target compile | PASS, 380994 bytes flash (29%), 26256 bytes globals (8%) |
| Whitespace check for changed tracked files | PASS |
| Physical MPU, radio, wheel directions, braking and route runs | Not tested; no board flashed |
| Android Gradle build/unit tests/lint | Not rerun for this firmware change |

Target builds used Espressif Arduino core 3.3.11, NimBLE-Arduino 2.5.1,
FastAccelStepper 1.2.7, ESP32Servo 3.2.1 and HX711 0.6.4. A temporary Arduino CLI
toolchain was used outside the repository. The full vendor install exceeded the
disk quota while extracting an unrelated RISC-V compiler; the successful builds
used the official, checksum-verified core and only the tools/libraries needed
for ESP32 and ESP32-S3. Existing flasher caches were not changed.

The commands below use the current `MAIN_EXECUTOR_CODE` name after the
2026-10-29 rename. Historical binaries/logs retain their original names.

```bash
python3 -B tools/verify_project.py --host-only
arduino-cli compile --fqbn esp32:esp32:esp32s3 \
  --output-dir artifacts/firmware/BLE_Receiver-Final BLE_Receiver-Final
arduino-cli compile --fqbn esp32:esp32:esp32 \
  --output-dir artifacts/firmware/MAIN_EXECUTOR_CODE MAIN_EXECUTOR_CODE
```

The firmware binaries and full logs are in `artifacts/firmware/` and
`artifacts/verification-2026-10-24-s3-mpu/`. Host tests substitute peripherals
and time; they do not establish electrical or mechanical behavior.

The repository already had unrelated working-tree changes. They were preserved.
A full `git diff --check` reports existing CRLF whitespace issues in unrelated
files; the check limited to this task's changed tracked files passes.

## Behaviors exercised

- Positive and negative signed gyro samples, trapezoidal heading integration,
  2°/1° hysteresis, persistent deviation after rotation stops, and micros rollover.
- Explicit I2C pins/rate/timeout, register setup, probing 0x68/0x69, wrong identity,
  partial reads, read failure, saturation and gaps over 100 ms.
- Left heading deviation requests right correction and vice versa, independent
  of lidar TILT. Lidar front/side clearance may veto a tap, without reversing it.
- No MPU heading, missing front clearance and legacy direction-only data cannot
  generate a tap. Cooldown does not prematurely cancel a running correction.
- Segment START/END through actual controller/bridge UART, a new reference
  after a turn, cleanup after a rejected motor command, and cached-command reset.
- A sensor fault cancels an active correction from the bridge loop. An obstacle
  STOP preserves the current segment reference, and a controller reboot clears it.
- Simulated raw MPU samples traverse the production bridge and controller code
  and produce the expected motor speed correction. Existing full outbound/return,
  BLE reconnect, stale-data STOP and obstacle stop/resume checks still pass.

## Deployment notes

Flash both updated sketches. Use bridge MPU SDA=GPIO8/SCL=GPIO9 and bridge
UART RX=GPIO18/TX=GPIO17. Cross bridge TX17 to controller RX18, and controller
TX19 to bridge RX18; share ground. Full power, mounting and console instructions
are in [`BLE_Receiver-Final/NOTE.txt`](../BLE_Receiver-Final/NOTE.txt).

The supplied algorithm integrates raw gyro Z without bias calibration. It can
drift and cannot measure sideways displacement. Verify stationary drift, heading
sign, wheel response and the longest intended straight run on the actual chassis.
