# Executor and BLE start repair

**Goal:** Make desktop startup prompt and recover the result of an operator start without replaying motion.

**Scope:** Correct the existing Pi GUI/start path, BLE bridge and motor executor. Preserve route calibration, collision thresholds, watchdog deadlines and the requirement for fresh motion permission. Work in this supplied directory: it has no Git metadata.

- [x] Reproduce delayed first GUI paint, a stopped GUI refresh loop, dropped start outcomes, UART backpressure and the executor's blocking boot handshake.
- [x] Paint the monitor before backend startup, remove the fixed startup sleep, and keep refresh scheduling alive after a failed refresh. Start BLE before ROS setup.
- [x] Add a bounded, session-scoped query for the pending start ID. A query only reports the remembered result and state; it cannot start motion, reset the route or refresh path permission. Never retransmit START automatically.
- [x] Report bridge UART rejection and executor motion holds, including MPU, path and local sonar, through existing notifications and the GUI. Reject a start if a motor channel did not initialize.
- [x] Replace the eight-second executor setup wait with the existing main-loop handshake/recovery service.
- [x] Run the Python suite, production-firmware host tests and project checks. Exercise the GUI on an available desktop with robot peripherals substituted. Record target-build and physical-validation limits and deployment instructions.

Review cases: an ACK lost after execution; a write that did not reach the bridge; disconnect/reboot while a query is queued; a busy UART; blocked or stale sensing; a transient GUI exception. No tests may send commands to the physical robot.

Completed with 199 Python tests and the firmware behavioral suite passing in
the final host project run. Target compilation, flashing and physical acceptance
remain deployment checks; see `docs/executor-ble-repair-2026-10-30.md`.
