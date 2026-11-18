# Developer 90-degree floor turns — 2026-11-16

The owner added separate left/right 90-degree developer tests to the immediate turn-stop
request, with a floor-contact prompt and a notice when the MPU cannot see rotation.

The Developer tests window now offers **Turn left 90°** and **Turn right 90°** alongside
the existing 10-degree tests. Every measured-turn confirmation says GARBY must stand on
the floor with both wheels touching the ground and the motors/wheels must not be lifted.
The MPU measures rotation of the chassis: turning wheels in the air does not rotate it.
Cancel sends nothing; deselecting MPU disables all measured-turn buttons.

The Pi maps `L90` / `R90` UI actions to `[TEST:<id>:L|R:90:<mask>]`. Both controllers accept
exactly 90 as an addition to the existing 1..15-degree range; intermediate large angles
and MPU-deselected turns remain invalid. These are MPU-measured turns with no hallway-wall
alignment, using the same immediate wheel-queue abort at the estimated endpoint as routes.
They do not start or resume a route and end at IDLE with normal gates restored.

At 400 steps/s the nominal 4900-step quarter turn takes about 12 seconds. Only 90-degree
tests therefore get a 30-second absolute deadline, including readiness waits; the Pi allows
0.5 second for the final report. The bridge and executor enforce 30 seconds themselves.
Small finite tests retain eight seconds. All finite tests keep the 400 steps/s cap, the
800 ms KEEP lease, confirmed ID/session handling, emergency stop and selected LiDAR/sonar
gates. MPU supervision is compulsory for any measured turn. RunStart and held remote
controls retain their separate behavior.

A 90-degree test that receives fresh but almost unchanged MPU heading (<0.5 degree) while
wheel counts exceed three expected degrees plus 300 steps immediately ends once as
`NO_ROTATION`. This avoids running lifted wheels for the whole larger-angle step limit.
Small-turn no-rotation checks and their existing hold priority remain unchanged. Missing
or lost heading cannot be replaced by step counts and keeps its own hold/failure result.

The monitor's `NO_ROTATION`, `HELD_MPU` and `MPU_DATA_LOST` messages explain floor contact.
They do not claim lifted wheels are the only possible cause: on the floor, check wheel
grip and the MPU; an undetected or disconnected sensor also needs its connection checked.

Deployment requirement for the combined session: **Both** whole sketches,
`BLE_Receiver-Final` and `MAIN_EXECUTOR_CODE`, and the entire `RasPi/` directory, all build
`TURNTEST-20261116`. This includes the first request's `TURNSTOP-20261116` change and the
earlier `RUNSTART-20261116B` sources. Do not upload those intermediate images separately.
Installed revisions remain unverified; no hardware was operated during these checks.

The current saved bridge GUI selects ESP32-S3, 8 MB flash / no PSRAM, no connected port.
It differs from the earlier documented N16R8 16 MB / octal PSRAM configuration. The executor
GUI selects no board or port; its documented verified production target is ESP32 Dev
Module, 4 MB / PSRAM disabled. Exact configurations used for target verification and all
commands, results, source/binary hashes and changed files are recorded in
[`artifacts/developer-90-turns-2026-11-16/VERIFICATION.md`](../artifacts/developer-90-turns-2026-11-16/VERIFICATION.md).

The host kinematic model proves consumer behavior, not floor rotation, wheel grip or a
physically exact 90-degree stop. Floor acceptance still requires measured angles and
observation of abrupt stopping with the real payload. See `DEPLOYMENT_AND_ACCEPTANCE.md`.
