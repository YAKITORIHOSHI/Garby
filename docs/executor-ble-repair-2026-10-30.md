# Executor, BLE and GUI repair — 2026-10-30

The supplied log confirms BLE readiness and a manual-start request, but contains
no executor acknowledgement or motion-hold diagnostics. It does not establish
the robot's physical failure cause. The following software failure modes were
reproduced and corrected in the supplied sources.

| Reproduced behavior | Correction |
| --- | --- |
| GUI was built after backend initialization and a fixed one-second sleep | Paint the full monitor first, remove the sleep, begin BLE discovery before ROS setup |
| Any exception in the GUI refresh callback permanently stopped future updates | Log the failure, temporarily disable Start and schedule the next refresh in `finally` |
| A missing START write/ACK left the operator waiting with no result recovery | Transmit START once; query its result once per second within the existing five-second deadline |
| Bridge UART backpressure silently discarded a new START | Report `UART_BUSY` explicitly |
| Temporary query transport failure could be mistaken for rejection of the original action | Only the executor supplies a terminal query outcome; failed query delivery remains pending |
| Executor accepted manual start even if a stepper channel was absent | Reject it with `MOTORS_UNAVAILABLE` |
| Successful start could be held by MPU/path/local sonar with no Pi explanation | Relay motion holds to the GUI and terminal; preserve holds throughout sonar clearance confirmation |
| Setup waited up to eight seconds for BLE | Enter the existing safety/handshake recovery loop immediately after hardware setup |

The query `[START STATUS:<id>]` returns the remembered result or `NOT_RECEIVED`
and current state. It cannot launch a route, consume a request ID or refresh
path permission. Queries are discarded after acknowledgement, timeout,
disconnect or executor restart. Route geometry, motor speed/acceleration,
collision thresholds and watchdog deadlines remain unchanged. Manual start
bypasses the scale/bin/gas trigger; it does not override motion safety holds.

Final verification used:

```bash
python3 -B tools/verify_project.py --host-only
```

All **199 Python tests** passed on a real Tk desktop with ROS/BLE/Firebase/serial
boundaries substituted. The desktop integration test clicks the production
Start button, loses the first acknowledgement, recovers it through the BLE
query path and continues using the bypass/message controls. Production firmware
host checks passed, including full outbound/return motion with simulated
peripherals, UART query identity/freshness, query congestion, sonar hysteresis
and prompt boot without a bridge. Structural Python/C++/JSON checks and Android
static quality checks also passed. A read-only independent code review found no
remaining issues after its two findings were corrected and regression tested.

The final log is [project-host.log](../artifacts/start-repair-2026-10-30/project-host.log).
The directory also contains a source manifest, source patch and expected-failure
logs proving the query-congestion and sonar-hysteresis regressions.

Limits: no target ESP32 compiler was available on PATH, no firmware was flashed,
and no real BLE, UART or motor command was sent. Target builds and Android
Gradle builds were skipped. The old legacy audit script is absent. Host tests
do not establish motor rotation, electrical timing or wireless behavior on the
robot. Initial GUI paint is improved; a stalled synchronous ROS initialization
can still delay interactive use before the Tk main loop begins.

Rebuild/upload both `MAIN_EXECUTOR_CODE` and `BLE_Receiver-Final` using the actual
selected boards in the flasher, deploy the complete updated `RasPi/` directory,
then restart from that directory with `bash run_bridge.sh`. Existing firmware
binaries elsewhere in the workspace predate this repair. Verify the new log
shows START written, executor acceptance/rejection and any motion-hold reason.
The missing `/dev/ttyAMA0` warning describes the separate bin/gas sensor UART;
configure its real port when restoring those readings. Manual start does not
require that board.

Continuation: both ESP32 target compilations subsequently passed in the resumed
session. The desktop launcher, XLaunch integration and Codex startup repairs
were also verified. See [the resumed-work report](raspi-resume-2026-10-30.md)
for fresh binaries, live BLE observations and the remaining USB deployment and
physical acceptance steps.
