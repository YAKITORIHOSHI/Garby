# Tab focus and intermittent Pi/S3 BLE connection

## GUI change

The monitor's three tab headers now skip keyboard Tab traversal and omit the
theme's focus outline. Mouse tab selection and selected-tab styling remain.
Focus indication/traversal on action controls, including emergency stop, remains
available. Only the custom notebook style is changed, not the global Tk theme.

The GUI test fixture now loads production helpers with a temporary operator-stop
path. It must not depend on or clear the actual robot's persisted stop marker.
The full isolated GUI run passed 31 tests, and both changed tests passed on the
final source with the fixture supplying its own isolation. No skips occurred.

The production monitor was reloaded at 15:56:25 UTC (11:56:25 Pi local time),
preserving its stop file. Latest launch details are recorded in
`artifacts/live-monitor-2026-10-30/latest-launch.json`.

## Observed connection failure

The initial Pi failures were **discovery failures**, before connection, GATT,
notifications or application writes. The BlueZ adapter was powered, unblocked,
and received a live advertisement from another peripheral. An independent
unfiltered 20-second scan did not receive GARBY at its verified historical
address `84:FC:E6:58:A7:7D`, or any GARBY name/service match. The running Pi had
no `GARBY_BLE_*` overrides selecting a wrong address/name/UUID/adapter.

The bridge later appeared without a Pi code/configuration change:

| Pi local time | Observation |
| --- | --- |
| 11:46:17 through 11:49:13 | GARBY discovery failures |
| 11:49:52 | Same verified bridge MAC discovered |
| 11:49:54 | Notifications subscribed, readiness write acknowledged; executor IDLE and operator-stop motion status received |
| 11:50:15 | First PATH write timeout, retry strike 1/5 |
| 11:50:17 | BLE disconnected; GARBY discovery subsequently failed again |
| 11:54:24 | Same bridge discovered again |
| 11:54:27 | BLE readiness handshake succeeded again |
| 11:55:20 | First PATH write timeout, retry strike 1/5 |
| 11:55:22 | BLE disconnected; passive HCI event recorded reason `0x08` |
| 11:55:57 | A further disconnect matched a second passive HCI `0x08` event, following another single write-timeout strike |
| 11:56:36 | Reloaded monitor completed readiness and received executor IDLE/emergency-stop state |

The passive HCI listener recorded only Disconnect Complete events, with no
Bluetooth commands or payload data. It captured a successful-status event for
connection handle 64 at **2026-10-30T15:55:22Z**, reason **0x08**. Bluetooth
defines 0x08 as Connection Timeout, including expired link supervision timeout.
[Bluetooth Core Specification, controller error codes](https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Core-61/out/en/architecture,-change-history,-and-conventions/controller-error-codes.html).

This event occurred before the deliberate GUI reload, and with only one logged
write-retry strike. It was not a Pi five-strike intentional disconnect. Successful
live handshakes demonstrate that the existing name/UUID/subscription/readiness
path can work; they do not demonstrate a stable radio or exclude a firmware stall.
The capture also recorded a second 0x08 event at 15:55:57Z matching the second
production drop. Event-only capture does not include the peer address; timestamp
correlation with the GARBY log associates these events with its connection.
Further runtime observation after reload again showed a drop at 11:57:24,
preceded by a single write timeout. The reload applied the UI change but did not
resolve the radio/connection fault. No BLE transport settings were changed.

Historical production evidence predates these changes and shows the same
unavailable-advertising symptom: `artifacts/resume-2026-10-30/live-bridge.log`
lost the connection at 05:47:40, repeatedly found no advertisement, then recovered
the same MAC at 05:56:40 and acknowledged readiness at 05:56:42.

## What remains uncertain

The device owner reported uncertain S3 power/current firmware state. The saved
Windows flasher selection reports COM18 and ESP32S3 Dev Module; that is not a
serial port available to this Pi and does not verify the current uploaded image.
The Pi's LiDAR adapter must not be treated as an S3 programming/diagnostic port.

Radio interference/range, peripheral power/reset and a stalled S3 radio/firmware
can all produce a link timeout or gaps in advertising. The captured event alone
does not identify which caused this outage. The Pi's observed temperature was
45.2 C; direct `vcgencmd` throttling flags were unavailable because `/dev/vcio`
was absent. No device node or power configuration was changed to mask that.

The next useful physical evidence is stable S3 power and its serial boot/reset,
advertising and disconnect lines, correlated with the Pi timestamps. With the
robot secured and stopped, verify power, antenna clearance/distance and absence
of another connected BLE client. Use the actual S3 serial monitor to identify
the running build and reset reason before choosing a firmware change. Do not
blindly alter UUIDs, flash the LiDAR port, erase Bluetooth state or relax path/
supervision safety settings based on this observation alone.

## Evidence and scope

- `artifacts/ble-discovery-2026-10-30/unfiltered-scan.json`
- `artifacts/ble-discovery-2026-10-30/hci-live-events.json`
- `artifacts/live-monitor-2026-10-30/monitor-154556.log`
- GUI test output and source hashes in `artifacts/ble-discovery-2026-10-30/`

The investigation used read-only adapter/log checks, advertisement scanning and
passive HCI event capture, plus the existing application's normal reconnection.
The GUI reload used the existing preserved Pi stop latch. No firmware upload,
start-motion, stop release, controller/Pi reset, Bluetooth-state deletion or
test SMS command was issued. The missing bin/gas serial link is separate from
this BLE discovery/link-timeout diagnosis.

The owner subsequently reported PC-USB-dependent S3 operation and a 52 V battery
feeding a 5 V/10 A buck, then the ESP32 5 V connection, then S3 5Vin, with shared
grounds confirmed by the wiring description. See the
[standalone-power investigation](s3-standalone-power-2026-10-30.md) for the source/
compiled-image audit and proposed electrical checks. No measured supply or
ground evidence yet establishes the cause of the intermittent connection.
