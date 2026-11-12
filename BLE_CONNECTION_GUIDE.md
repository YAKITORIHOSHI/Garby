# GARBY Pi ↔ ESP32 BLE connection — 2026-10-04

Latest live investigation (2026-10-30): [discovery gaps and captured connection timeout](docs/ble-discovery-2026-10-30.md).
The Pi can discover and complete the handshake when the S3 appears, but the link
intermittently drops with HCI reason `0x08`. Power/radio/firmware-reset evidence
from the S3 is still needed to distinguish the underlying cause. The following
sections retain the earlier connection procedure and its dated validation scope.

Flash `BLE_Receiver-Final/` on the BLE ESP32 and deploy the complete `RasPi/`
directory, including `ble_support.py` and `runtime_support.py`. The motor controller is a different
board and does not advertise GARBY's service. Build outputs are under
`artifacts/firmware/`. Live Pi/ESP32 radio operation has **not** been tested here.

## Isolate the connection

Lift wheels, stop the production service, and close other BLE clients (including
the simulator and phone test apps). GARBY uses BLE GATT; desktop Bluetooth
pairing is not its connection procedure.

From the repository root on the Pi:

```bash
sudo systemctl stop garby-bridge
set -a
source /etc/default/garby-bridge
set +a
bash RasPi/run_bridge.sh --check-ble
bash RasPi/run_bridge.sh --check-ble --connect
```

Without an installed service/environment file, skip its stop/source commands
and use the same `run_bridge.sh --check-ble` commands. Install
`RasPi/requirements.txt` into the service's Python using the Pi environment's
package-installation policy. The diagnostic needs Bleak, neither ROS nor Firebase.

The default probe only scans. `--connect` checks GATT, subscribes, writes **only**
`[RASPI READY]` with acknowledgement, waits for a notification, and disconnects.
It sends neither GO nor clear path data. The independent path watchdog holds
STOP throughout the probe. Its unit tests use a BLE substitute, not a real radio.

The connection probe, production, and the bench simulator share an OS lock at
`GARBY_RUNTIME_LOCK_FILE` (default `/tmp/garby-bridge.lock`). Use the same path
for all of them. A lock error means another owner or inaccessible lock file;
stop that owner before trying again. Process exit releases the lock. The file
remains and must not be deleted to bypass an active owner. Scan-only probes do
not acquire this lock.

Expected stages:

```text
DISCOVERY PASS: GarbyESP32 [actual address]
CONNECTION/GATT PASS: expected service, write, notify
NOTIFY PASS: subscription accepted
READINESS WRITE PASS: acknowledged; no GO sent
RX CONNECTED...
BLE ROUND TRIP PASS...
```

An MCU state notification may appear instead of `CONNECTED...`.

| First failure | Next check |
| --- | --- |
| Import | Requirements installed in the same Python. |
| ADAPTER inactive/blocked/Powered=false | Bluetooth service, rfkill, adapter power. |
| DISCOVERY | Correct bridge sketch, serial advertising log, adapter/address, power, another connected client. |
| CONNECTION/GATT timeout | ESP32 disconnect reason, Pi BlueZ logs, competing clients, power under load. |
| Missing service/characteristics | Board/sketch and UUID overrides; inspect the specific BlueZ device cache if GATT changed after reflashing. |
| NOTIFY | Bleak version, BlueZ/D-Bus error, NOTIFY property, service-user access. |
| READINESS WRITE | WRITE property, matching firmware, link stability, reported BlueZ error. |
| NOTIFICATION RECEIVE | Bridge loop running and serial readiness receipt; capture both logs. |

## Adapter and firmware identity

The environment example selects `GARBY_BLE_ADAPTER=hci0`. Use the actual `hciN`
for a USB adapter. Leave `GARBY_BLE_ADDRESS` unset initially. Discovery accepts
the service UUID or exact `GarbyESP32` name, including UUID advertisements without
a name. A pinned address must match the real bridge; a wrong address deliberately
does not fall back to another robot.

| GATT item | UUID |
| --- | --- |
| Service | `4fafc201-1fb5-459e-8fcc-c5c9c331914b` |
| Write | `beb5483e-36e1-4688-b7f5-ea07361b26a8` |
| Notify | `beb5483e-36e1-4688-b7f5-ea07361b26a9` |

Read-only checks:

```bash
systemctl status bluetooth --no-pager
rfkill list bluetooth
bluetoothctl list
bluetoothctl show
journalctl -u bluetooth -n 80 --no-pager
"${GARBY_PYTHON_BIN:-/usr/bin/python3}" -m pip show bleak
```

If Bluetooth is disabled, restore it on the Pi:

```bash
sudo systemctl enable --now bluetooth
sudo rfkill unblock bluetooth
bluetoothctl power on
```

Do not erase all BlueZ state or ESP32 storage initially. Remove a specific stale
device entry only when changed GATT and the logs establish a cache issue.

## Reading a drop (2026-11-04)

The Pi log now records why a link ended:

```text
[BRIDGE DIAG fw=CENTER-20261107B reset=POWERON up_s=812 drops=1 last_drop=0x208 last_link_s=640 itvl_ms=50 sup_ms=4000 ...]
BLE link lost: session 640.2 s, last write 4.01 s ago, Pi none, CPU 58.2 C
Pi health changed: undervoltage-now, CPU 61.0 C
```

| Evidence | Points at |
| --- | --- |
| `reset=BROWNOUT`, or a small `up_s` after a drop | S3 rebooted: measure its 5 V under motor load; try a short separate branch from the buck rather than a daisy chain through the ESP32 |
| `last_drop=0x208` with a long `up_s` | Supervision timeout, 4 s of total radio silence: RF, WiFi/BT audio on the Pi's shared antenna, or an S3 stall |
| `reset=SOFTWARE` | The monitor's Reboot button: it is the sketch's only restart call |
| `last_drop=0x213` | The Pi ended the link: look for `BLE persistent write failure` |
| `last_drop=0x216` | The S3 ended the link: its 10 s silence watchdog saw no Pi writes |
| Not discovered for minutes | S3 unpowered or boot-looping, or another central connected |

The Oct 30 evidence in [BLE discovery](docs/ble-discovery-2026-10-30.md) shows this pattern: one PATH
write timeout, reason `0x08` about 2 s later, then minutes with no advertising. That fits an S3 that
went silent, not the Pi's retry logic, which never reached its fifth strike. The Nov 4 live run repeated it
and reported `reset=BROWNOUT`, so measure the S3's supply first; the voltage itself is still unmeasured.

Until the bridge asks for its 4 s timeout, about 1 s after readiness, the link uses BlueZ's default
420 ms. To close that window, once:

```bash
sudo sed -i 's/^#ConnectionSupervisionTimeout=.*/ConnectionSupervisionTimeout=400/' /etc/bluetooth/main.conf
sudo systemctl restart bluetooth
```

Units are 10 ms. This only changes when a dead link is declared; the 850 ms path watchdog still stops
the robot. Keep paired Bluetooth audio devices off while running: the Pi shares one radio between
audio, WiFi and this link.

## Connection changes

- The Pi uses the scanned BLEDevice, avoiding another implicit scan. Deadlines:
  discovery 12 s, connection/GATT 30 s, subscription 8 s, each write 2 s.
  Readiness requires subscription and an acknowledged readiness write.
- Failed candidates are cleaned up and retried with backoff. Callbacks belong
  to their session. Shutdown waits for bounded cleanup. Disconnect/write failure
  clears readiness; old callbacks cannot affect a replacement connection.
- Linux explicitly uses StartNotify for Bleak 3.0.0/3.0.1 compatibility.
  [Bleak Linux guidance](https://bleak.readthedocs.io/en/latest/backends/linux.html).
- The bridge allows 45 s before the first valid write and 10 s silence afterward.
  Its **850 ms path STOP watchdog remains active** during setup.
- Scan response is explicit, advertising interval is 100–150 ms, and automatic
  disconnect advertising has a 2 s inactive-advertising retry. Connection tuning
  waits for Pi readiness and at least 1 s after connection.
- Notifications fit the default 20-byte ATT payload: `~` starts a line, `+`
  continues it, and newline completes it. Bounded Pi reassembly preserves long
  return IDs without requiring a larger MTU. MCU queries every 5 s and idempotent
  return queries recover lost status/outcomes.
- The bench simulator shares discovery, GATT validation, framing, and the
  bounded sequenced mailbox. Its distances remain synthetic input.

Bleak explains why scanned devices are preferable to address strings.
[BleakClient guidance](https://bleak.readthedocs.io/en/latest/api/client.html).

## Resume production

After the isolated probe passes, source ROS and the YDLIDAR overlay and run
`bash RasPi/run_bridge.sh --check-runtime` following `DEPLOYMENT_AND_ACCEPTANCE.md`, then:

```bash
sudo systemctl start garby-bridge
journalctl -u garby-bridge -f
```

Expect `BLE ready: notifications subscribed and readiness write acknowledged`.
Confirm fresh scans and MCU readiness separately. With wheels lifted, test STOP
during disconnect and fresh-clearance recovery afterward. Save probe output,
Pi log, ESP32 serial log, address/adapter, firmware versions, and power readings
with the remaining route, sonar, and braking acceptance results.
