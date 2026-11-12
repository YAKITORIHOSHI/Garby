@AGENTS.md

# Claude Code notes

Active sources: `RasPi/final_w_serial.py` and its helpers, `BLE_Receiver-Final/` (S3 bridge),
`MAIN_EXECUTOR_CODE/` (motor controller), `Garby_MobileApp/app/` (Android).
`RasPi/final_w_serial-simulator.py`, `Test/` and `WebConnect-GarbyController` are not production.
Newest findings: `docs/obstacle-sweep-hold-2026-11-12.md` (stuck-forever root causes, holds that
resume, low power, sonar sweep, moving-object prediction, pre-run centring, buzzer, bench steps).
Before it: `docs/lidar-map-2026-11-12.md` (X3 scan truncation, radar map) and
`docs/navigation-2026-11-11.md` (navigation design, calibration).

```bash
python3 -B tools/verify_project.py --host-only 2>&1 | grep -vE '^PASS'
bash RasPi/run_bridge.sh --check-runtime
bash RasPi/run_bridge.sh --check-ble     # scan only; --connect needs wheels lifted
arduino-cli compile --fqbn esp32:esp32:esp32s3:FlashSize=8M,PSRAM=disabled --build-path <scratch> BLE_Receiver-Final
arduino-cli compile --fqbn esp32:esp32:esp32:FlashSize=4M,PSRAM=disabled --build-path <scratch> MAIN_EXECUTOR_CODE
```

Load `garby-robot-maintenance` for protocol, firmware or field-triage work. It has the bench isolation
needed before running the bridge: this tree holds a real Firebase key and the operator stop latch.
Android cannot build here (no JDK).

## Keeping token use down

- `final_w_serial.py`, `BLE_Receiver-Final.ino` and `MAIN_EXECUTOR_CODE.cpp` run to 2,000-2,800 lines:
  `grep -n` first, then read a range.
- Dated reports at the root, in `docs/` and in `artifacts/` are history. Read the newest one only.
- Filter test output with `grep -vE '^PASS'`. The full Pi suite takes about 3 minutes; run single modules while iterating.
