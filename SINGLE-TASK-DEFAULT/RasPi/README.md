# GARBY Raspberry Pi bridge

`final_w_serial.py` is the production bridge. It connects the sensor UART,
YDLiDAR/ROS 2, the ESP32 BLE receiver, and Firebase RTDB without allowing a
cloud or telemetry failure to block the motion-safety path.

## Recommended launch

```bash
python3 final_w_serial.py --headless
```

Headless mode avoids Tk rendering load during long hallway runs. Optional
configuration:

```bash
export GARBY_FIREBASE_CREDENTIALS=/home/garby/Desktop/service-account.json
export GARBY_FIREBASE_DATABASE_URL=https://garby-thesis-default-rtdb.asia-southeast1.firebasedatabase.app
export GARBY_SENSOR_SERIAL_PORT=/dev/ttyAMA0
export GARBY_SENSOR_SERIAL_BAUD=9600
export GARBY_LIDAR_PORT=/dev/serial/by-id/<ydlidar-device>
```

Set `GARBY_LIDAR_DRIVER_LOGS=1` only while diagnosing the ROS driver; normal
operation suppresses its verbose subprocess output.

## Boot and crash recovery

`garby-bridge.service.example` is a systemd service template. Adjust its
`User`, `WorkingDirectory`, `ExecStart`, and ROS distro setup path to the actual
Pi deployment. Ensure that user can access the sensor UART and LiDAR device
(normally through the `dialout` group), then install and enable it:

```bash
sudo install -m 0644 garby-bridge.service.example /etc/systemd/system/garby-bridge.service
sudo systemctl daemon-reload
sudo systemctl enable --now garby-bridge.service
systemctl status garby-bridge.service
```

Put the `GARBY_*` environment values in `/etc/default/garby-bridge`. The service
runs headlessly at boot and restarts five seconds after a process exit. It does
not make a thermally unstable or undervolted Pi safe; investigate any reboot or
nonzero throttling flag before unattended use.

`garby-bridge.env.example` contains the supported keys without credentials or
device-specific identifiers. Copy it to `/etc/default/garby-bridge`, adjust the
values, and keep the resulting file and Firebase service account out of source
control.

## LiDAR geometry

The established sensor orientation is `0° BACK`, `90° RIGHT`, `180° FRONT`,
and `270° LEFT`, with diagonal sectors between them. A scan is partitioned in
one O(n) pass into two related views:

- safety uses eight nearest 45° sectors. Their half-open boundaries cover the
  complete 360° exactly once, so there are no gaps between named directions;
- steering uses only the centered 22° slice of each sector for cleaner wall
  medians and tilt. If a centered slice has fewer than three valid points, its
  containing 45° sector is the conservative median fallback.

All valid in-range returns therefore remain available to `robust_near`
obstacle detection, including returns at former between-cone angles, while
corner and diagonal returns normally stay out of lane-centering calculations.

`final_w_serial-simulator.py` is a manual visualization/protocol harness, not a
raw-scan model. Its eight direction cards are already-aggregated inputs and do
not exercise this angular partition, ROS watchdogs, or physical sensor timing;
therefore simulator success is not a collision-safety certification. It shares
the production LiDAR-only blockage formatter, so simulated trash ultrasonic
values likewise cannot affect path decisions.

## Firebase schema mapping

Each publish is one atomic root multi-location update. The bridge preserves the
legacy paths used by the supplied database export and mirrors live values under
the device node used by Android:

| Source | Legacy path | Device path |
| --- | --- | --- |
| trash ultrasonic | `/RASPI/VALUES/ULTRASONIC_SENSOR/CM_DISTANCE` | `/devices/garby-bin-01/sensors/level/value` |
| MQ135 | `/RASPI/VALUES/MQ135_SENSOR/AIR_QUALITY` | `/devices/garby-bin-01/sensors/mq135/value` |
| MQ137 | `/RASPI/VALUES/MQ137/AMMONIA` | `/devices/garby-bin-01/sensors/mq137/value` |
| MQ4 | `/RASPI/VALUES/MQ4_SENSOR/METHANE` | `/devices/garby-bin-01/sensors/mq4/value` |
| load cell | `/RASPI/VALUES/LOAD_CELL/WEIGHT_IN_KG` and `/MCU/VALUES/LOAD_CELL` | `/devices/garby-bin-01/sensors/weight/value` |

Sensor mirrors include `value`, `unit`, `sensorType`, and epoch-millisecond
`updatedAt`. Reset commands remain at
`/devices/garby-bin-01/commands/reset`, with `/APP/isReadyToReset` retained as
the compatibility flag.

Unavailable values are state transitions, not periodic telemetry:

- ultrasonic writes `999` once when it becomes unavailable;
- each gas sensor writes `-1` once when it becomes unavailable;
- repeated absent/sentinel UART frames cause no further database write;
- the first valid recovery is written immediately and rearms one future outage
  sentinel.

The UART `ULTRASONIC` value is exclusively the trash-level distance inside the
bin. It is published to Firebase and the `SENSOR:US=...` telemetry packet, but
it never participates in front obstacle detection, person classification,
`P:` path status, or steering. Corridor blockages are LiDAR-only and therefore
encode as generic obstacle `O` (or stale `S`); legacy `H` parsing remains for
protocol compatibility only.

The 30-second heartbeat retains `/RASPI/STATES/lastSeen`,
`/RASPI/STATES/recent_uptime`, and their device mirrors. These additional
device status fields are change-only:

| Field | Type | Meaning |
| --- | --- | --- |
| `cpuTemperatureC` | number | whole-degree Pi CPU temperature |
| `thermalWarning` | boolean | at least 75 °C or an active Pi throttle bit; omitted when neither thermal source is available |
| `throttledFlags` | integer | raw `vcgencmd get_throttled` bitmask, when available |
| `bleConnected` | boolean | usable BLE GATT write + notification session |
| `lidarHealthy` | boolean | recent ROS scan containing valid range points |
| `sensorSerialConnected` | boolean | recent recognized sensor UART traffic |

Link-health booleans are always published. Thermal fields remain absent/unknown
until at least one real source is readable; database template fields may remain
`null` until that first reading. Android must not interpret a missing or `null`
`thermalWarning` as `false`/THERMAL OK.

## Local verification

Hardware-independent invariants are covered by standard-library tests:

```bash
python3 -m unittest -v test_bridge_core.py
python3 -m py_compile bridge_core.py final_w_serial.py
```

Before an unattended run, bench-test the physical LiDAR orientation, front and
back stop distances, BLE reconnect, UART unplug/replug, and the Pi's cooling
under the actual enclosure. Software reports thermal throttling but cannot
replace a heatsink/fan or adequate power supply.
