# Recovered RasPi work — 2026-10-30

Recovered the interrupted local session
`01a18c0a-4cd6-7281-9045-d0c23d7a4a04` from this device's Codex records.
Its request was to fix slow GUI startup and manual Start failing to start the
executor. The session hit its usage limit after saving its source changes and
verification notes, before delivering a final answer. The subsequent
"Continue all and everything" request failed during the account switch.

The saved repair is documented in
[executor-ble-repair-2026-10-30.md](executor-ble-repair-2026-10-30.md).
All 17 source hashes matched its saved manifest when work resumed.

## Verification after resuming

- All 199 Python tests passed with no skips on an actual Tk desktop.
- All 185 firmware host behavior checks passed.
- Structural checks and Android static checks passed.
- Three additional GUI integration tests passed through the live XLaunch
  forwarding connection. Robot peripherals were substituted in these tests.
- The desktop launcher pointed to the obsolete `GARBY-Test_CODEBASE` folder.
  It now delegates to the current project's `run_bridge.sh`. Both launchers
  are executable, and the desktop instructions were updated.
- Current preflight passes Python/ROS, powered Bluetooth adapter, LiDAR and
  Firebase configuration. The separate sensor-board UART remains unavailable;
  manual Start does not require its trigger readings.

Logs, file hashes and detailed results are in
[`artifacts/resume-2026-10-30/`](../artifacts/resume-2026-10-30/).
During diagnostic probes, no physical START or GO command was sent.
After the owner powered the standalone boards, a live BLE probe passed
discovery, GATT validation, notification subscription and acknowledged readiness.
Notifications included `SYSTEM READY`, `IDLE`, and `LOAD_CELL:0.00`. A separate
read-only `START STATUS` query got no correlated reply in eight seconds; deployed
outcome-query support remains unverified. This is not a motor test.

After the owner requested launching after fixes and taking over, the complete
production bridge was started in XLaunch at `localhost:10.0`, detached from Codex.
Process 15680 owns the runtime lock. The live startup log confirms LiDAR scans
(690 range samples initially), BLE notification/readiness handshake, executor
`IDLE`, and Firebase initialization. The actual `GARBY - Robot monitor` window
was verified on the forwarded display. The process is being left running for
the owner; no manual Start was clicked by the agent. Normal production sensing,
path feedback and cloud updates are active. The separate sensor UART remains
unavailable. See `live-bridge.json` and `live-bridge.log` in the evidence directory.

## Codex startup repairs

The `codex-security` launcher lacked Node. An official ARM64 Node runtime was
installed at its existing discovery location, with checksum verification.
The production MCP launcher now initializes successfully and lists 47 tools.

Ubuntu AppArmor denied bubblewrap's namespace setup. Ubuntu's scoped
`bwrap-userns-restrict` profile was installed and loaded after administrator
authentication. Codex sandbox creation succeeds; separate tests confirm
outside-workspace writes and network sockets are still blocked. Global
namespace restriction settings were preserved.

See [the startup repair evidence](../artifacts/resume-2026-10-30/codex-startup.md).
The active Codex daemon was preserved during builds and tests. A final delayed
restart is being prepared to refresh old startup warnings and MCP state after
all results are saved; see the restart log in the evidence directory. If that
restart has not completed, use `/daemon` in Codex to restart the background server.

The owner-requested sudo credential is stored and verified in the encrypted
GNOME Login keyring. Nonsecret retrieval metadata is in
`/home/garby/.config/garby/credential-locations.md`; no plaintext credential file
was created, and the separate default keyring was preserved.

## Deployment work

Both target firmware compilations passed using documented ESP32 core 3.3.11
and the pinned library baseline (NimBLE-Arduino 2.5.1, FastAccelStepper 1.2.7,
ESP32Servo 3.2.1, HX711 0.6.4). The compiler and build script never upload.

| Source target | Flash / PSRAM | Program bytes | Static RAM bytes |
| --- | --- | --- | --- |
| MAIN_EXECUTOR_CODE / ESP32 Dev Module | 4 MB / disabled | 383782 | 26336 |
| BLE_Receiver-Final / ESP32-S3 Dev Module | 8 MB / disabled | 579864 | 37060 |

Fresh binaries are under
[`artifacts/resume-2026-10-30/firmware/`](../artifacts/resume-2026-10-30/firmware/).
`firmware-builds.json` records exact commands, dependency versions and source and
binary SHA-256 hashes. These outputs supersede the older firmware elsewhere in
the workspace for this repair.

The Pi's USB inventory exposes the LiDAR adapter. Saved Windows flasher state
lists COM4 and COM8; that does not establish a current Pi firmware connection.
The owner confirmed both controllers run from their standalone power supply.
Flashing and physical acceptance remain pending USB access; power and BLE
connectivity alone do not provide a firmware upload transport. No wireless
firmware-update implementation was found in the bridge/executor sources.

To open the repaired monitor, run this in your active `ssh -Y` terminal with
XLaunch running:

```bash
~/Desktop/start_garby_gui.sh
```

Manual Start skips scale/bin/gas triggers. It continues to require controller
readiness and valid motion safety data. The repaired monitor reports executor
acceptance/rejection and any MPU, path or sonar hold.
