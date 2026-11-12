# Field triage

## Start with the evidence

The Pi's journal or console has what the bridge's USB console used to hide:

```
[BRIDGE BUILD:CENTER-20261107B]
[BRIDGE BOOT:POWERON]
[BRIDGE DIAG fw=CENTER-20261107B reset=POWERON up_s=812 drops=1 last_drop=0x208 last_link_s=640 itvl_ms=50 sup_ms=4000 mpu_faults=0 uart_resync=0 notify_fail=0]
BLE link lost: session 640.2 s, last write 4.01 s ago, Pi none, CPU 58.2 C
Pi health changed: undervoltage-now, CPU 61.0 C
```

An older `fw=` means the board was not reflashed. Editing a sketch changes nothing on it.

## `MOTION:STOP:MPU`, "MPU not ready"

The link can be perfect while this holds the wheels. `mpu=<now>/<last fault>` in `[BRIDGE DIAG]` names the cause:

| Word | Meaning | Next step |
| --- | --- | --- |
| `NOT_FRESH` | The MPU answers but gives no new data: it probably reset and went to sleep | Measure its 3.3 V and ground while the motors start |
| `READ` | I2C reads are failing | Check SDA/SCL wiring, pull-ups and distance from stepper cables |
| `NOT_FOUND`, `BUS` | No answer on the bus | Check MPU power and wiring |
| `MOVING` | Too much motion while calibrating | Keep the robot still; check for vibration |
| `GRAVITY`, `SATURATED`, `GYRO_OFFSET` | Readings out of range | Check mounting and the sensor itself |
| `GAP` | The S3 loop stalled over 100 ms | Firmware: find what blocks the loop |
| `JUMP` | A yaw step over 20 degrees between two samples: corrupt data | Check I2C wiring and distance from stepper cables |

Never widen these limits to clear the hold. Latch the E-stop before resetting the S3 during a route: a restarted
bridge has no heading for that segment.

## Weaving or not centering

Read the `[BRIDGE CENTER ...]` lines in the Pi log. The console is not saved by itself, so capture a floor run with
`bash RasPi/run_bridge.sh 2>&1 | tee ~/garby-floor.log`. Tuning is in `docs/centering-spec-2026-11-07.md`.

| Line | Meaning | Next step |
| --- | --- | --- |
| `CENTERED taps=N`, N above 0 | Heading-hold tapped a centered robot: yaw left the 1 degree band. The LiDAR did not ask for it | Measure wheel trim and gyro drift. Do not widen the deadband for this |
| `CORRECTING` while the robot looks centered | The LiDAR reads an offset the eye does not | Compare `err=` with a tape measure. Set `CENTER_BIAS_CM` |
| `OFF CENTER 1/2` never reaching `CORRECTING` | The reading is not lasting: features, or a robot crossing the center | Normal unless it is far off center |
| `SKIP: noisy`, `width changed`, `walls lost` repeating | Doors or clutter, or the width was captured beside one. One bad reading skips a cycle | Heading-hold is carrying the robot. Check `WIDTH` against the real hallway |
| `SKIP: heading off reference` repeating | Yaw stayed over 3 degrees from FRONT at the check | Watch `yaw=` on USB. Taps may be vetoed, or too weak for the wheel mismatch |
| `SKIP: samples missing`, `stale` | Few valid scans, or a BLE dropout | Check the LiDAR and link first |
| `BIASED REFERENCE?` repeating | Corrections all one way: misaligned start, gyro drift or unequal wheels. The reference never moves, so from about 2 degrees the robot ends at the wall stop | Align the start, then measure drift and wheel trim |
| `CHECK STEERING SIGN` | A correction did not improve the error. Latched until an S3 restart | Check `GARBY_LIDAR_YAW_OFFSET_DEG`, left/right and which wheel a tap slows. With a badly misaligned start or strong gyro drift, that is the likelier cause |
| `centering off:` at a segment start | The same latch, from an earlier segment | As above, then restart the S3 |
| No lines at all | Build older than `CENTER-20261107`, or no valid walls | Check `[BRIDGE BUILD]` |

## BLE drops

| Evidence | Meaning | Next step |
| --- | --- | --- |
| `reset=BROWNOUT`, or small `up_s` after a drop | The S3 rebooted | Measure 5 V at the S3 under motor load; try its own short branch from the buck instead of the daisy chain through the ESP32 |
| `reset=PANIC` or `WATCHDOG` | Firmware crash | USB serial capture; find the backtrace |
| `reset=SOFTWARE` | The monitor's Reboot button, the sketch's only restart call | Check who pressed it before suspecting a fault |
| `CONNECTED...` or `[IDLE]` repeated, count growing per reconnect | `RasPi/` older than Nov 4: dropped sessions were not closed | Deploy the current `RasPi/` |
| `last_drop=0x208` with a long `up_s` | Supervision timeout: 4 s of total radio silence | RF, antenna, 2.4 GHz WiFi or BT audio on the Pi, or an S3 stall |
| `last_drop=0x213` | The Pi ended it | Look for `BLE persistent write failure` in the Pi log |
| `last_drop=0x216` | The S3 ended it | Its 10 s silence watchdog: the Pi stopped writing. Check the status responder |
| `sup_ms=420` persisting | Parameter update never applied | Set `ConnectionSupervisionTimeout=400` in BlueZ `[LE]`; see BLE_CONNECTION_GUIDE.md |
| Not discovered for minutes | S3 unpowered, boot-looping, or another central connected | Serial log; `bluetoothctl devices Connected` |

The Pi shares one antenna between WiFi and Bluetooth. This Pi has paired, trusted BT audio
devices, and active audio takes most of the airtime.

## LiDAR or sensor board not found

Ports are detected from the data, so run `bash RasPi/run_bridge.sh --check-runtime`. It reports the
port found or that none is sending. No port means the device is not producing data: check power and
the cable, and for a GPIO sensor board that `dtparam=uart0=on` is set. A port variable in the
environment disables detection for that device, so check for a stale one.

`Waiting for the first LiDAR scan; holding fail-closed STOP` at start is normal: the driver is
found and launched seconds later. The monitor's map shows a radar sweep meanwhile, then either
the live scan or "LiDAR not available" with the supervisor's reason. Driver stdout is discarded
unless `GARBY_LIDAR_DRIVER_LOGS=1`; with it, `Real points N > fixed points M` means the SDK is
truncating scans, which `fixed_resolution:=false` (the default since 2026-11-12) prevents.
`BLE discovery failed: ... none matched` with other advertisers heard means the S3 itself is
not advertising; see the BLE table above.

## Throttling

The bridge is light (1.7% of one core plus 0.6% for the LiDAR driver, measured with the real LiDAR
and BLE idle), so software load is an unlikely cause. Look at power first: on a buck converter a Pi 5 cannot negotiate USB-PD, assumes a weak
supply and caps USB current, which matters because the LiDAR motor runs from USB.

- `Pi health changed:` lines report changes every 5 s, with or without Firebase.
- Full throttle flags need the `video` group. Without it only undervoltage is visible, and that
  also feeds the `[HEALTH]` thermal state and SMS power alarm.
- Do not read `soc:firmware/get_throttled`: it logs a kernel WARN. `journalctl -k | grep -i voltage` shows history.

## Robot stopped and will not resume

Check the GUI motion hold, the `[MOTION:...]` log line, or the SMS reason:

| Reason | Cause | Fix |
| --- | --- | --- |
| Emergency stop / `STOP:OPERATOR` | Operator latch, stored in `~/.config/garby/operator-stop` | Explicit release from the GUI; a reboot keeps it |
| `MPU:HEADING_LOST` / `STOP:MPU` | Over 100 ms without good gyro data in a straight run | Check I2C wiring, align the robot, send `R` on the bridge console |
| `LiDAR obstacle` (`F=O`) | Within 95 cm straight ahead, 60 cm front diagonal, 35 cm rear or 22 cm side | A wall W cm to the side reads about 1.08 W on the diagonal, so it trips when closer than about 55 cm |
| `Stale LiDAR data` (`F=S`) | Scan stale, or a sector with under 3 returns | An open-sided area with nothing within 8 m across a full 45 degree sector holds STOP |
| `BLE link unavailable`, `Controller UART unavailable` | No fresh packets, or no UART ACKs | BLE table above; then UART wiring and `uart_resync` |

## Nudging looks absent or wrong

1. Enabled only on the long hallway segments (`isNudgeEnabled=true` in `pointsRun.ino`).
2. After any BLE reconnect, startup grace and ramp need 16 fresh scans in side packets (about 5 s).
3. Nudges assume motor 1 is the physical LEFT wheel. Test with wheels lifted. If a correction
   speeds the deviation up, the sign is wrong. If forward and turns are reversed too, the cause is
   the DIR polarity pair in `MAIN_EXECUTOR_CODE.h`, not the nudge code. If forward is right but
   turns are reversed, the wheels are crossed and polarity cannot fix it. See
   `docs/motor-direction-2026-11-07.md`.
4. Gyro drift: the reference never moves, so drift becomes a slow turn the sidesteps must keep undoing. In the model
   0.02 deg/s holds and 0.05 deg/s reaches the wall stop. Measure heading drift over 2 minutes.
