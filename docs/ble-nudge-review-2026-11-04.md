# Nudging, Pi throttling and BLE review (2026-11-04)

Source implemented and software tested. The S3 ran `LINKFIX-20261104` and then `LINKFIX-20261104B` in
wheels-lifted tests (see [Live run](#live-run-wheels-lifted-2026-11-04)). **Not physically accepted:** the
standalone S3 supply is unresolved, the controller's firmware version is unverified, and nudging and floor
runs are untested.
Evidence is in [`artifacts/ble-nudge-review-2026-11-04/`](../artifacts/ble-nudge-review-2026-11-04/),
including the red test logs from the unchanged code and the pre-change source archive.

## Live run, wheels lifted (2026-11-04)

Log: `artifacts/ble-nudge-review-2026-11-04/live-wheels-lifted.log`. It runs the production bridge with
LiDAR read-only on front and rear. Lines without a `COMMAND` line before them came from the monitor window.

| Time | What the log shows |
| --- | --- |
| 04:27:38 | Link up, `LINKFIX-20261104`, `reset=POWERON`. NimBLE rejects one control write with `INSUFFICIENT_RESOURCE`; `notify_fail=12` |
| 04:28:11 to 04:29:14 | Three drops after 32.9, 29.2 and 9.4 s, each about 4.1 s after the last good write |
| 04:29:14 to 04:31:15 | **Two minutes with no advertising** |
| 04:31:17 | **`reset=BROWNOUT up_s=10`**, then a drop after 6.6 s |
| 04:31:26 | `reset=POWERON`. Link then holds for 52.6 s |
| 04:31:35 to 04:31:55 | **Start accepted:** `[START ACK]`, `[RUNNING]`, `Executor state: OUTBOUND`, `[MOTION:CLEAR]`, until E-stop from the monitor. Link stays up throughout |
| 04:32:14 | Monitor Reboot restarts the S3, which causes the 04:32:19 drop and later `reset=SOFTWARE`. The Pi refuses its own reboot: `Interactive authentication required` |
| 04:32:26 | One more drop, after 4.4 s. **Unexplained** |
| 04:32:28 to 04:32:33 | E-stop latched and acknowledged, then a clean monitor shutdown |

**1. The S3's supply is now the leading cause, from the chip's own report.** The owner reports BLE only
connects with the S3 on a laptop, the symptom recorded on
[Oct 30](s3-standalone-power-2026-10-30.md). The S3 itself reported a brownout reset. The longest and only
stable session followed the `POWERON` at 04:31:26, which fits a move to laptop power, though that timing
is inferred. It held through the 20 s route start. Limits: one brownout sample, which plugging a cable in
could also cause, and no voltage has been measured. Pins 17, 18, 8 and 9 are not strapping pins, and the
sketch does not wait for a USB host.

**2. Pi bug: every reconnect added a duplicate of each notification. Fixed.** `CONNECTED...` arrived 1, 2,
3 times per session, back to 1 after the two minutes without advertising, then 2, 3, 4. It kept growing
across an S3 power-on, so the count lives on the Pi. Bleak only closes a session's D-Bus connection in
`disconnect()`, which `_handle_disconnect` never called after a dropped link, and BlueZ re-arms a
still-open session's notifications on connect. Effects: single-frame lines were handled N times, lines over
two frames were corrupted or lost (this is why `[BRIDGE DIAG]` vanished after the first reconnect), and one
D-Bus connection leaked per drop, against a limit `system.conf` lists as 256 per user. The BlueZ mechanism is
inferred from Bleak's source and the counts. The next live run confirms it: one `CONNECTED...` per session.
This did not cause the drops.

**3. Firmware regression from this review: `[BRIDGE DIAG]` flooded the connect burst. Reduced.** One line
is about 8 notifications and was sent twice within a second, and NimBLE has 12 buffers. It now waits 2 s
after readiness and merges repeat requests. No write was rejected on `LINKFIX-20261104B`, but `notify_fail=8`
and `9` show the connect burst still loses some notifications.

The owner confirmed the wheels turned in this run, and that they only do so with the S3 on a desktop.
Nudging was not exercised.

### Second run, `LINKFIX-20261104B`

Logs: `live-LINKFIX-20261104B-first-boot.log` and the session after it.

- 04:59:11: new build confirmed, and **a second `reset=BROWNOUT`** (`up_s=8`). The link dropped after 51.4 s
  and the S3 was found again 2 s later.
- 05:00:07: `CONNECTED...` arrived once where the old code would have repeated it. Only one reconnect has
  been seen so far, and the monitor closed cleanly 1 s later, with nothing logged to say why.
- 05:04:45: `reset=POWERON`. Start was accepted at 05:05:14 and 05:06:01, with no drops up to 05:06:23.
  **The route held at `[MOTION:STOP]`, `LiDAR obstacle`**, because read-only was left off in this session.
  One scan showed real objects at 29 cm in front and 26 cm behind, against limits of 95 and 35 cm.

### Why the wheels only turn with the S3 on a desktop: the MPU

Log: `live-LINKFIX-20261104B-battery-mpu-fault.log`. With the S3 on battery (owner's report) and read-only on:

| Time | What the log shows |
| --- | --- |
| 05:10:49 | Release after a battery boot gives `[MOTION:STOP:MPU]`, "MPU not ready", in IDLE: that boot's calibration had failed |
| 05:14:41 | A later boot calibrated. Start accepted, `[MOTION:CLEAR]` |
| 05:14:49 | **`[MOTION:STOP:MPU]`, `mpu_faults=1`, 8 s after Start** |
| 05:15:42 to 05:15:50 | **Same again, 8 s after Start**, `mpu_faults=2` |
| 05:13:41 to 05:22:45 | BLE and the controller stayed up for 549 s throughout |

The 20 s desktop run at 04:31 had none of these. The link was healthy, so **the BLE was not what stopped these
runs**. The heading is only sampled inside a straight segment, so the first straight segment is where the
loss shows, whenever it really began. Two things are not known yet: which check failed, and why battery power
matters. A supply dip that resets the MPU, or I2C noise once the steppers run, would both fit.

`LINKFIX-20261104C` closes the design gap. Before it, recovery took a USB console or a board reset, and the
reason only reached the console, so one bad moment on battery looked like dead motors. Now, after any MPU fault
or failed boot calibration:

1. `STOP:MPU` holds as before, and readiness is withdrawn at once.
2. After 1.5 s to brake, the bridge reconfigures the MPU and recalibrates, without blocking its loop, retrying
   every 3 s. Boot and recovery share one calibration check, and no limit changed.
3. Only a full clean calibration restores readiness. In a straight segment it then takes a new reference, and
   motion resumes through the normal clear-path confirmation, about 4 s after the fault.
4. `[BRIDGE DIAG ... mpu=<now>/<last fault>]` names the cause in the Pi log.

**This does not remove an electrical cause.** If the MPU keeps failing while the motors run, the wheels will
stop and start, and `mpu_faults` will climb. The new reference also re-zeroes "straight" mid-segment, so check
it on the floor: a robot pushed while held would treat its new direction as straight.

## Your three questions

### Will nudging work as intended? It would not have, in common situations

1. **Any BLE outage over about 3 s silently ended nudging for the rest of the hallway.** With no UART
   traffic the executor re-announced itself with `[MCU READY]`, the line it sends at boot, and the bridge
   treated that as a restart and dropped its heading reference. Motion resumed after reconnecting with no
   steering, for a segment of up to about 100 s. Reconnecting nearly always takes longer than 3 s. During
   a route the ping is now `[MCU ALIVE]`. IDLE still sends READY because its startup handshake needs renewing.
2. **A doorway or open side switched off heading correction.** The Pi omits `L=`/`R=` when the steering
   cone has no wall return, and the bridge read that as a corrupt packet. Confirmed against real `rclpy`: a
   doorway gives `P:3|F=C|B=C` with `CL`/`CR` present and no `L=`. Now only centering is skipped. Malformed
   values, missing CF/CL/CR and known-close sides still suppress the tap.
3. **One bad I2C read could end the run.** A single failed gyro read, likely near stepper drivers,
   invalidated the heading until someone typed `R` on a serial console. A bad read is now skipped; 100 ms
   without good data still faults.

Also fixed: a path packet landing just before a new segment's first gyro sample could brake the robot as
it started the long hallway, and a reference on a frozen sensor could stay pending forever.

### Is the Pi throttling? Software load is not the cause

The real bridge on this Pi 5 with the real LiDAR at 5.9 scans/s, BLE scanning but not writing, used **1.7%
of one core plus 0.6% for the LiDAR driver**, with a 0.22 ms median scan callback measured separately. CPU
stayed at 42-46 C, with no undervoltage or thermal messages in the kernel log back to Oct 21. Live BLE
writes and the motor load are not in that figure.

The real gap was that **throttling was invisible**. `vcgencmd` needs `/dev/vcio`, and user `garby` is not
in the `video` group, so flags were always unknown. That also disabled the `[HEALTH]` power alarm and its
SMS notice. Health was only sampled when Firebase was connected and was never logged locally. The bridge now
reads the kernel's undervoltage alarm as a fallback and logs every change. If this fires on the robot, check
power first: on a buck converter a Pi 5 cannot negotiate USB-PD, so it assumes a weak supply and caps USB
current, which the LiDAR motor draws from.

### Why does BLE suddenly disconnect? Most likely the S3's supply

[Oct 30 evidence](ble-discovery-2026-10-30.md) shows one write timeout, HCI reason `0x08` (supervision
timeout) about 2 s later, then **minutes with no advertising**. That is not the Pi's retry logic, which
never reached its fifth strike. The Nov 4 live run repeated this pattern and added a reported
`reset=BROWNOUT` on the daisy-chained `52 V -> buck 5 V -> ESP32 5V -> S3 5Vin` supply.

**The voltage is still unmeasured, and software cannot measure it.** The bridge's reset cause and disconnect
reason used to stay on its unattended USB console. They now reach the Pi log. See "Reading a drop" in
[the BLE guide](../BLE_CONNECTION_GUIDE.md).

## LiDAR and sensor ports are now detected automatically

Both devices are found by what they send, on any port, in either order, and after re-plugging. Names
cannot do this: ttyUSB numbers follow plug order, and both boards can use the same CP2102 chip.

| Device | Recognised by |
| --- | --- |
| LiDAR | Chained YDLIDAR packets at 115200. Verified on the real X3, which streams on its own |
| Sensor board | A complete `ULTRASONIC:`, `MQ4:`, `MQ137:` or `MQ135:` line at 9600 |

Probing only listens, with DTR and RTS held off because they reset many boards. It never opens a port
the other device holds: the driver takes no OS lock, so a probe changing the baud could otherwise break a
live LiDAR link. A port with no matching data is never selected, so missing LiDAR data still means STOP.
An explicit `GARBY_LIDAR_PORT` or `GARBY_SENSOR_SERIAL_PORT` still wins and disables detection.
`~/.config/garby/lidar-port` is no longer read, and the file was left in place.

**Limits.** The real LiDAR was detected and ran with no port set, but moving it between physical sockets
and the sensor board itself were not exercised here: swaps and re-plugging were tested with real `pyserial`
over pseudo-terminals, and the sensor board was not connected. A board wired to GPIO14/15 cannot be detected
until that UART exists: add `dtparam=uart0=on` to `/boot/firmware/config.txt` and reboot. It is off on this Pi.

## Changes

| Defect | Effect | Fix |
| --- | --- | --- |
| Recovery ping identical to the boot announcement | BLE outage over 3 s ends nudging for the segment | `[MCU ALIVE]` during a route |
| Missing L/R treated as corrupt | Doorways and open sides disable heading correction | Missing skips centering only |
| One bad MPU read invalidates heading | Run ends until a console `R` | Skip the sample; 100 ms budget still faults |
| MPU fault or failed boot calibration has no recovery on battery | Motors stay held, with the reason only on the USB console | Recalibrate while held; report the reason to the Pi |
| UART ACK count never recovers lost lines | After 8 lost lines in a session, `STOP:MCU_LINK` until BLE reconnects | Resync once the executor has answered and gone quiet; a silent executor still faults |
| Unbounded PATH replies; parked polling at 12.5 Hz | Link near saturation: an ACKed write costs about 100 ms at a 50 ms interval | 150 ms floor on the Pi, 2 Hz when parked. Urgent STOP bypasses both |
| Throttling unreadable; health tied to Firebase | No evidence when something fails | hwmon fallback, 5 s local sampling, logged on change |
| Path packet racing a new heading reference | About 1 s stop at the start of the long hallway | Bounded settling window; no data becomes a fault |
| Status thread and ROS scan callback unguarded | One exception ends the stream: every reconnect then drops after 10 s with no visible cause | Guards, logged and failing closed |
| Bridge notices discarded in headless mode | Build and reset cause never logged | All `[BRIDGE ...]` lines logged; new `[BRIDGE DIAG ...]` |
| MPU loss reported as `PATH:BLOCKED from LiDAR` | Operator chases the wrong sensor | `MPU:HEADING_LOST` |
| Dropped BLE session never closed on the Pi | Each reconnect duplicates notifications, corrupts long lines and leaks a D-Bus connection | Close it before reconnecting |
| `[BRIDGE DIAG]` sent inside the connect burst | NimBLE runs out of buffers and rejects a Pi write | Wait 2 s, merge repeats |

Added: `run_bridge.sh --check-ble`, `SupplementaryGroups=video dialout` in the service example, preflight
advice for the BlueZ timeout and `video` group, and bridge ID `LINKFIX-20261104C`.

Already correct in this tree, so unchanged: NimBLE explicit-payload notifications, 9 dBm TX power, STOP queue
priority, PATH expiry, LiDAR supervisor gating, and turn signs matching `WebConnect-GarbyController`.

## Decisions and limits

1. **An emergency stop is latched.** `~/.config/garby/operator-stop` was rewritten at 04:32:28 on Nov 4 and loads
   on the next runtime start, so the robot will not move until it is released from the monitor.
2. **An open-sided stretch holds STOP.** The driver reports no-return as `0.0` (its `invalid_range_is_inf`
   option is ignored), so a 45 degree sector with nothing inside 8 m reads `LIDAR_UNAVAILABLE`. Verified:
   `F=S|B=S`. Treating no-return as clear is unsafe with dark objects, so this stays. If your hallway has an
   open side, it needs a deliberate design decision.
3. **Hallways under about 176 cm stop the robot** (95 cm diagonal limit), or wider ones when off-center.
   The read-only bypass skips that check but also removes the protection, so measure width and braking first.
4. **Gyro drift is unmeasured.** Bias is calibrated once at boot and centering is capped at 4 degrees.
   Warm-up can shift bias, and 0.1 deg/s over a 100 s hallway already exceeds the cap. Measure stationary drift
   for 2 minutes. If significant, re-zeroing while stopped is the next step.
5. **After a bridge reboot mid-segment** the robot continues without heading control for that segment.
   Latch the E-stop before flashing or resetting the S3 during a route. `STOP:MPU` no longer needs a console.
6. **Sensor board not found on this Pi yet.** Nothing is sending sensor lines. On USB it will be found
   automatically. On GPIO14/15 it needs `dtparam=uart0=on` first, as `/dev/ttyAMA0` does not exist.

## Deploy

Flash **both** sketches and copy the whole `RasPi/` directory: an older bridge ignores `[MCU ALIVE]`. Images
and hashes are in the artifact directory: `firmware-LINKFIX-20261104C/` for the S3, `firmware/MAIN_EXECUTOR_CODE/`
for the controller. Confirm `[BRIDGE BUILD:LINKFIX-20261104C]` in the Pi log afterwards.
Editing a sketch here does not change the boards: the live Oct 30 run reported `STRAIGHT-20261030`.

On the Pi, once, for a stabler connection start and readable throttle flags:

```bash
sudo sed -i 's/^#ConnectionSupervisionTimeout=.*/ConnectionSupervisionTimeout=400/' /etc/bluetooth/main.conf
sudo systemctl restart bluetooth
sudo usermod -aG video garby     # log out and back in
```

## Verification

| Check | Result |
| --- | --- |
| Pi unit tests | 298 pass with no display (26 Tk tests skip; was 280 in total). The 32 tests in `test_monitor_ui`, including those 26, pass on a private Xvfb in 6 s. Over forwarded X they take minutes and one Start test fails |
| Firmware behaviour checks, production sources | 490 pass (was 407), including full outbound and return |
| Target build, `esp32:esp32:esp32s3:FlashSize=8M,PSRAM=disabled` | Pass, 588,324 B flash, 38,004 B RAM |
| Target build, `esp32:esp32:esp32:FlashSize=4M,PSRAM=disabled` | Pass, 393,086 B flash, 28,312 B RAM |
| Real `rclpy` node against synthetic scans | Clear, doorway, open side, stale, recovery, narrow hallway and silence all correct |
| Real bridge and real LiDAR, no port set, isolated | Detected, 5.9 scans/s, 1.7% + 0.6% of one core, 46 C; clean SIGTERM exit |
| Preflight, nothing configured | Finds the real LiDAR; reports the sensor board missing |
| Real Bleak scan through BlueZ | Works; robot off, so `DISCOVERY FAIL` as expected |
| C++/JSON structure, Android static check | Pass |
| Android Gradle build | **Not run**: no JDK on this Pi |
| Live BLE and start, wheels lifted, both S3 builds | Start accepted; wheels turn only with the S3 on a desktop, per the owner. See [Live run](#live-run-wheels-lifted-2026-11-04) |
| `LINKFIX-20261104C`, nudging, sensor board on hardware, floor run | **Not run** |

Each new test failed on the unchanged code first. Negative controls for the thin-object veto, missing
clearance and a silent executor passed before and after, which shows they test the intended behaviour.

## Physical checks, in order

Wheels lifted, hardware stop within reach:

0. Fix the S3 supply first. Change wiring only with power off, and use one board power input at a time. Measure
   5 V at the S3's 5V pin against its own GND on battery only, then try a short separate branch from the buck.
1. Release the emergency stop deliberately, then confirm the new `[BRIDGE BUILD]` and `reset=POWERON`.
2. Turn direction: this tree's signs match the web controller but were never confirmed on the calibrated
   route. Nudge direction: deflect the robot left; the correction must slow the RIGHT wheel.
3. Pull BLE for over 5 s mid-segment. Wheels stop, reconnect, resume, and **corrections still occur**.
4. Run motors under load and watch `[BRIDGE DIAG]` and `Pi health changed` for a reset or undervoltage.
5. On the floor: stationary gyro drift, then a short supervised run past a doorway.

## Claude Code setup

- `.agents/skills/garby-robot-maintenance/` improved in place, and linked from `.claude/skills/` because
  Claude Code does not read `.agents/`. Both tools now share one copy. Field triage loads only when needed.
- `CLAUDE.md` imports `AGENTS.md` and adds commands and token guidance.
- `.ignore` cuts searches from 11,198 files to 192. `.gitignore` does nothing because this is not a git repository.
- `.claude/settings.json` blocks reads of build output and the Firebase key, caps inline command output at
  15,000 characters, and lists 13 skills irrelevant here by name only.
- Plugins (user level): `session-report` and `pyright-lsp`.
