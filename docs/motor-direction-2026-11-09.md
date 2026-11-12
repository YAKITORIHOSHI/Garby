# Forward direction correction, November 9, 2026

The owner confirmed that a forward/front route run still travelled backward with the
November 7 corrected executor firmware. This newer observation supersedes the forward
wiring conclusion in `motor-direction-2026-11-07.md`. The two executor DIR polarities now
change from `true/false` to `false/true`. Source implemented, host motor tests passed, and the actual ESP32 target compiled successfully. No upload or
physical acceptance was performed in this work.

The observation establishes the required reversal of the reported forward wheel motion.
It does **not** establish which controller is the physical left wheel, whether the names
of the turns/nudges match their physical direction, or why the November 7 assumptions did
not match the robot. There is no encoder or measured DIR-voltage capture in this evidence.

## Production path and scope

`runStart()` executes five positive 100-step calibration bumps, then a positive 8000-step
leg. `safeMoveDistance()` retains the motion gate, and `moveDistance()` adds its signed
step count to each current wheel target. `moveToTarget()` commands FastAccelStepper and
permits straight-segment steering only when both targets rise. `setup()` applies the
named polarity constants to motor 1 DIR GPIO22 and motor 2 DIR GPIO12.

| Setting | November 7 | This correction |
| --- | --- | --- |
| `MOTOR1_DIR_HIGH_COUNTS_UP` | `true` | `false` |
| `MOTOR2_DIR_HIGH_COUNTS_UP` | `false` | `true` |
| Positive-count DIR levels | motor 1 HIGH, motor 2 LOW | motor 1 LOW, motor 2 HIGH |

The real installed FastAccelStepper 1.2.7 source was inspected. `setDirectionPin()` saves
`dirHighCountsUp`, initializes the pin to that level, and passes the same setting to the
pulse queue; increasing/decreasing targets retain their counter meaning. The correction
therefore reverses each wheel's physical direction for the same commanded count. The
host stub's polarity storage agrees with that API; the plant separately maps it to the
reported physical forward DIR levels.

`MAIN_EXECUTOR_CODE.cpp` and `pointsRun.ino` remain byte-identical to the pre-change
snapshots. Route distances/signs, turn targets, nudge wheel selection, all stop/clearance
and operator gates, sonar scope, heading reference, steering timing and protocols are
preserved. Negating route signs would also disable forward-only steering and is not
used here. Changing turn/nudge selection alongside the global polarity pair would
introduce another change unsupported by the forward-only report.

USB startup now prints the fixed identifier and actual selected polarities:

```text
[EXECUTOR BUILD:FRONT-20261109 DIR1_UP=LOW DIR2_UP=HIGH]
```

This line distinguishes the compiled correction from the November 7 source. Seeing a
compiled identifier or hash does not prove that image is installed. This is a USB serial
diagnostic; the Pi's bridge build line identifies the S3 firmware separately.

## Reproduction and software checks

Before editing executor production code, the motor test's independent wiring fixture was
updated to the owner's latest observation: motor 1 rolls forward with DIR LOW and motor 2
with DIR HIGH. The unchanged November 7 production constants then failed 12 checks,
including real `setup()` direction, positive 8000-step travel, calibration bump and actual
`runStart()` forward travel. The compiler succeeded and the executable exited 1. Exact
unchanged executor source snapshots/hashes, test fixture and failure log are preserved in
`artifacts/front-direction-circular-2026-11-09/motor-before-*`.

After the production change, `test_motor_direction.cpp` passes. It exercises the real
executor setup, safe positive/negative moves, calibration bumps, start sequence, original
left/right differential targets, and actual parser-to-consumer nudge wheel-speed selection.
It also requires the new boot identifier and checks that straight travel is forward for
either possible physical assignment of motor 1 to the left or right wheel.

The shared hallway plant now labels motor 1 = left as an unverified model assumption.
Under that assumption, the correction converges in the heading-hold/offset examples while
the November 7 polarity diverges. Crossing the wheel-side assignment reverses named turn
results without changing straight forward travel; the test demonstrates that ambiguity.
These synthetic outcomes do not establish a circular path's cause, physical straightness,
correct gyro orientation, measured turn direction, or physical stopping distance.

Evidence directory: `artifacts/front-direction-circular-2026-11-09/`.

- `motor-before-failing-test.log`: actual failing run against unchanged executor source.
- `motor-after-passing-test.log`: corrected motor checks and conditional plant results.
- `motor-production-diff.patch`: exact production header/setup difference.
- `motor-before-sha256.json`, `motor-after-source-sha256.json`: source hashes; identical
  production motion implementation and route hashes establish preservation.
- `motor-library-sha256.json`, `motor-tool-versions.txt`: inspected library and tool identity.
- `motor-saved-project-state.json`: read-only saved target selection; it is dated November 7
  and does not establish a current USB connection.
- `motor-executor-build.log`, `firmware-executor/`, `motor-binary-sha256.json`: successful
  actual compile output, generated binaries and their hashes. No upload flags or port
  are used. Flash use is 393186 bytes; static RAM use is 28312 bytes.

The verified target selection is saved as ESP32 Dev Module / `esp32dev`, 4 MB flash,
PSRAM absent. The compile uses its Arduino equivalent
`esp32:esp32:esp32:FlashSize=4M,PSRAM=disabled`, with build and cache under `/tmp` and output
in the new evidence directory. Whole-project suite results are recorded by the coordinated
change's validation report.

## Physical acceptance still required

With wheels lifted and normal stop controls available, first confirm the fixed boot ID,
which motor is connected to each physical wheel, and both wheel directions for a positive
move. Then check named left/right turns and each nudge separately; the forward-only report
cannot prove those. Confirm LiDAR front, gyro yaw sign, driver input levels and common
ground before a supervised low-speed floor test. Preserve emergency stop and all normal
fresh-path/nearest-clearance gates. Open-loop step counts cannot establish actual wheel
travel, slip, braking distance or floor straightness.
