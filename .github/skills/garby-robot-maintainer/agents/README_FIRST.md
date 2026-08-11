# GARBY Safety, Stability, and Maintenance Release

This package contains coordinated changes for all three GARBY processing nodes plus a reusable ChatGPT skill for future reviews and maintenance.

Deploy the Raspberry Pi, BLE bridge, and main-controller changes together because the safety and steering protocol is sequenced and shared across all three nodes.

## Folder layout

- `RaspberryPi/final_w_serial.py` - ROS 2 LiDAR, serial sensors, BLE client, Firebase, and GUI.
- `BLE_Receiver-Final/BLE_Receiver-Final.ino` - ESP32 BLE bridge, sole navigation parser, STOP latch, and nudge controller.
- `NAPHTALI_CODE_V2/NAPHTALI_CODE_V2.ino` - main-controller setup and state machine.
- `NAPHTALI_CODE_V2/NAPHTALI_CODE_V2.cpp` - parser, safety gates, motors, braking, servo/ultrasonic, SMS, and helpers.
- `NAPHTALI_CODE_V2/NAPHTALI_CODE_V2.h` - pins, constants, types, and interfaces.
- `NAPHTALI_CODE_V2/pointsRun.ino` - outbound and return route functions.
- `docs/SYSTEM_ARCHITECTURE.md` - complete safety-first system architecture and protocol baseline.
- `docs/PATCH_NOTES.md` - root causes and implemented changes.
- `docs/HARDWARE_TEST_CHECKLIST.md` - required bench and floor tests.
- `docs/VALIDATION_RESULTS.md` - software checks and limitations.
- `skills/garby-robot-maintainer/` - reusable skill source, companion thesis agent, static audit script, and architecture reference.
- `garby-thesis-assistant.agent.md` - standalone user-invocable GARBY safety and thesis agent.

## Deployment order

1. Back up the currently deployed programs and record board/library versions.
2. Compile and flash `BLE_Receiver-Final` to the BLE bridge ESP32.
3. Compile and flash the complete `NAPHTALI_CODE_V2` folder, including `pointsRun.ino`, to the main-controller ESP32.
4. Replace the Raspberry Pi program with `RaspberryPi/final_w_serial.py`.
5. Confirm bridge and controller UART are both 115200 baud.
6. Perform every wheels-lifted test in `docs/HARDWARE_TEST_CHECKLIST.md`.
7. Verify the physical direction of both `runStart()` and `returnToPointB()` before any floor test.
8. Perform reduced-speed empty tests before maximum-payload tests.

## Skill usage

The skill package is also provided separately as `skill.zip`. Its source is included under `skills/garby-robot-maintainer/`. The companion user-invocable agent is `skills/garby-robot-maintainer/agents/garby-thesis-assistant.agent.md`.

Typical prompts after installing it:

- Review these GARBY files for STOP-safety regressions.
- Fix the hallway zigzag without weakening obstacle stopping.
- Analyze BLE/UART timing and update SYSTEM_ARCHITECTURE.md.
- Audit the servo-ultrasonic scan and braking behavior.

The bundled audit can be run directly:

```bash
python skills/garby-robot-maintainer/scripts/audit_project.py .
```

## Important safety statement

The software is fail-closed and has passed host-side syntax, protocol, skill-validation, and static invariant checks. It has not been flashed to or driven on the physical robot in this environment.

This release is not a substitute for:

- a latching hardwired emergency-stop;
- a motor-driver enable or power interlock;
- a front bumper/contact switch;
- measured stopping-distance tests with the heaviest expected payload;
- verification of servo power integrity and route direction.
