# GUI tabs and acknowledged operator controls

The Pi monitor now separates LiDAR readings, BLE status/messages and operator
controls into three tabs. Emergency stop remains visible on every tab. The
Controls tab contains the existing front/rear LiDAR read-only switches, manual
start, obstacle-sonar toggle, explicit stop release and coordinated restart.

## Resulting behavior

- LiDAR read-only keeps distances updating and bypasses only the selected
  front/rear close-distance thresholds. Side and stale-scan stops remain active.
- Obstacle-sonar disable requires a ready stationary IDLE executor. It cancels
  pings, bypasses that sonar's obstacle checks and holds the servo at the existing
  `DEFAULT_VIEW` of 70 degrees. Bin-level sensing is separate. Sonar defaults to
  enabled after executor reboot; re-enabling requires fresh clear confirmations.
- Emergency stop latches at the Pi, bridge and executor, aborts the active route,
  force-stops both motors and centers the servo. GO packets, sensor thresholds,
  reconnects and ordinary return/reset requests cannot release it.
- Release requires acknowledged stationary IDLE and a matched request ID. It
  restores no motion permission; automatic triggering remains suppressed until
  a new deliberate manual start. It does not resume an interrupted route.
- Coordinated restart requires a confirmed emergency stop and a GUI confirmation
  dialog. It commits delayed executor and bridge software restarts before the Pi
  asks its OS for an orderly reboot. All three retain/reapply the stop. A new
  emergency-stop press cannot cancel an already committed coordinated reboot.
- Unknown/stale controller state is displayed as unknown. Pending actions are
  distinct from confirmed settings. Release, sonar change and reboot are never
  automatically replayed after uncertain writes or reconnection.

The Pi uses a dedicated priority mailbox for operator commands. It persists the
stop marker at `~/.config/garby/operator-stop` and reserves request IDs in batches
of 256 in the adjacent `.request-id` file. Reservations survive process restart
and wall-clock rollback without writing the ID file on every click. Both MCU
stop markers use validated no-init RTC memory across software restart. Preserve
the Pi state files and run desktop/service instances under the same configured
stop-file identity (`GARBY_OPERATOR_STOP_FILE` if overridden).

Missing reboot acknowledgements expire after five seconds without releasing
STOP. The bridge checks the deadline at the UART ACK handler as well as its
normal ingress timer. The Pi does not reboot on an expired, unmatched or
unattempted ACK. Pi reboot permission failures stay visible across automatic
stop reconfirmation. No permission grants, system-service installation, firmware
uploads or live restarts were performed during implementation.

## Scope and deployment

Deploy the complete updated `RasPi/` directory and rebuild/upload both
`MAIN_EXECUTOR_CODE` and `BLE_Receiver-Final`. Older firmware does not implement
the acknowledgement/status protocol and leaves the corresponding controls
unavailable. The updated target binaries, build commands, source hashes and
compile logs are in `artifacts/operator-controls-2026-10-30/`.

The separate bin/gas sensor board has no supplied reset endpoint or firmware
and is excluded from the restart button, as stated in its confirmation dialog.
The user confirmed rebooting the Pi and both supported controllers.

The Pi invokes `systemctl --no-ask-password reboot` in a worker thread. It uses
normal OS shutdown, without `--force`, shell execution, password prompts or
privilege changes. The runtime account needs existing OS permission for reboot,
and the runtime needs startup configuration if it should relaunch automatically.
The command's behavior is documented by
[systemd](https://www.freedesktop.org/software/systemd/man/latest/systemctl.html#reboot).

A dropped MCU/BLE acknowledgement can result in only the controllers restarting;
the Pi stays running unless it receives confirmation. Their stop latches remain
set. Distributed reboot completion cannot be proven after a link disappears.
An offline emergency stop is latched locally and sent on reconnection; hardware
watchdogs remain active. This GUI is not an independent physical emergency-stop
circuit. Physical stop/reboot behavior and electrical effects still require
bench acceptance with the robot secured against movement.

## Verification

- Full Pi suite: `python3 -B -m unittest discover -s RasPi -p 'test_*.py'` —
  223 tests passed, including Tk desktop tests on the available display.
- After the final committed-reboot persistence guard: `PYTHONPATH=RasPi python3
  -B -m unittest RasPi/test_operator_support.py` — all 18 targeted tests passed.
- `python3 -B tools/run_firmware_tests.py` — passed. Tests execute production
  firmware against deterministic host peripherals and cover real command
  round trips between both boards, stop during motion, GO/start/return override
  resistance, sonar cancellation/servo positioning/rearm, release without
  automatic motion, delayed reboot and late/lost ACK recovery.
- `python3 -B tools/source_sanity.py .` — passed structural source checks.
- Python syntax compilation — all seven changed/new Python files passed.
- Both ESP32 target builds passed using the recorded board identities and the
  installed ESP32 core 3.3.11, without uploading or opening hardware ports.
- All three tabs were captured and measured at 800×480 and 1024×600, including
  unavailable and simulated stopped/sonar-disabled/read-only states. No visible
  widgets extend outside the root or their panels.

See the [build and preview report](../artifacts/operator-controls-2026-10-30/validation.md)
for exact commands, hashes and memory usage. Test logs are stored beside it.
Remaining physical checks include stop latency under motion/link loss, actual
servo centering and sonar rearm, latch retention across controller/Pi reboot,
OS reboot permission, restart order and startup recovery. These checks were not
run against connected hardware.
