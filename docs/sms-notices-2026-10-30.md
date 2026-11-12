# Important SMS notices

GARBY now queues important fault and recovery texts through the existing Air780E
modem and configured `CONTACT_NUMBER` in `MAIN_EXECUTOR_CODE.h`. The recipient
was preserved. Pi internet and Firebase are not required for the modem to submit
SMS, but power, an active SIM, cellular registration and SMS service are required.

## Notices and timing

| Notice | Confirmed fault before queueing | Recovery confirmation |
| --- | --- | --- |
| Pi internet unavailable | 30 seconds | 10 seconds healthy |
| Firebase/cloud unavailable | 60 seconds | 10 seconds healthy |
| LiDAR stream/coverage unhealthy | 15 seconds | 10 seconds healthy |
| Bin sensor link/data unavailable | 30 seconds | 10 seconds healthy |
| Pi high temperature/current throttling | 10 seconds | 30 seconds healthy |
| Pi/controller BLE or UART link unavailable | 10 seconds confirmed unhealthy, after a previously observed link | 10 seconds continuous healthy traffic |
| Emergency stop applied/released | On confirmed local transition | Local release does not resume a route |
| Route fault/recovery | On actual fault/reset transition | Recovery only after its fault notice was submitted |
| Restart, load/gas trigger, destination, return | On actual controller event | Return notice requires successful route completion |
| Obstacle hold | Initial hold | Clearance notice when paired with a submitted hold; reminders at most every 10 minutes |

These are queueing thresholds, not promised receipt times. Rate limits, modem
registration and carrier delivery add delay. Initial Firebase startup gets a
separate 60-second grace period; intentional operation without the optional
Firebase SDK is unknown/disabled and does not generate a false cloud outage.
The health worker runs on a five-second cadence when its probes complete within
that interval. Sensor-board health requires fresh, valid data for all four bin
sensors, independent of the executor's obstacle-sonar enable/disable setting.

Examples include `GARBY: Internet unavailable on the Pi. Local robot safety
remains active.` and `GARBY: Internet connection recovered.` These notices do
not grant movement, return automatically, release an emergency stop or reboot
the robot. A route fault still requires supervised recovery.

## Internet and health evidence

A dedicated Pi daemon thread tests two HTTPS destinations independently of the
Firebase SDK: Google's connectivity-check endpoint and Cloudflare's trace
endpoint. Redirects/unexpected content are rejected, reads are bounded, and
individual socket timeouts are two seconds. DNS or operating-system stalls stay
outside Tk, ROS and BLE. A completed recent cloud operation can also demonstrate
connectivity when the public probe hosts are filtered; an old cloud badge cannot
hide a new outage. Sites that block both probes and do not permit Firebase may
need their network policy adjusted; a reachability test is not proof that every
internet service works.

Firebase failure with working internet is a separate cloud notice. An internet
outage suppresses the redundant cloud-down condition until connectivity is
known again. Current Pi temperature and active throttling/power flags are used;
historical throttling bits do not create a permanent fault. A missing reading
cannot prove that the previous high-temperature or power fault has cleared.

The Pi sends `[HEALTH:1:UUUUU]` over the existing BLE/UART path. Its five states
mean internet, cloud, LiDAR, bin sensors and thermal/power, each U/D/X for
healthy/fault/unknown. The latest snapshot replaces older ones. The Pi checks
snapshot age at the serialized GATT write boundary; the bridge uses a separate
lowest-priority mailbox with session/age checks. The executor treats missing
health telemetry after 15 seconds as unknown. Unknown never means recovered.
This packet never refreshes path, GO confirmation, or controller readiness.

## Queue and submission behavior

The executor has a fixed ten-slot RAM queue. Category/state coalescing retains
the current notice; emergency and fault priorities apply to queue admission as
well as dispatch. Routine/recovery entries can be displaced by more important
faults. A single persistent background worker owns modem UART transactions;
motion, parser and UI callbacks do not wait for the modem. Raw debug-console
passthrough to the modem was removed to prevent AT transaction collisions.

Messages use at most 140 default-alphabet characters and a `GARBY:` prefix.
There is at least 20 seconds between attempts and at most six attempts per
rolling ten minutes, including failures/retries. Queued notices expire after
ten minutes. A failure before submitting the body may retry once after 30
seconds; a failed or indeterminate result after Ctrl-Z is never automatically
replayed. Transition attempt limits survive queue replacement and unknown/stale
health gaps. A new confirmed healthy interval can rearm a later distinct fault.

The modem must provide a valid `+CMGS` reference followed by final `OK` to mark
submission successful. This confirms carrier-network submission, not delivery
to the recipient's handset. Uncertain sends remain uncertain; no success is
invented. Recovery is paired with a submitted fault, including when a local
fault clears during an in-flight SMS. Initialization/registration failures
retry in the background, with backoff capped at five minutes.

RAM queues do not survive controller power loss or restart. A complete cellular
outage or loss of robot power cannot generate an immediate SMS; an already
committed controller reboot cannot guarantee its queued notice is delivered.
There is no remote SMS command interface and no recipient change in BLE input.

## Deployment and verification

Deploy the full updated `RasPi/` folder and rebuild/upload **both** root firmware
projects. The new health packet needs the paired bridge and executor firmware.
This combined firmware also contains the earlier GUI/operator-control changes;
the latest SMS build artifacts supersede the earlier operator-only images.

Software checks use fake internet/modem boundaries and production algorithms:
outage/recovery/unknown handling, probe failure and captive portals, stale writes,
current vs historical thermal flags, queue priority/coalescing, retry/cap/expiry,
uncertain submissions, in-flight transitions, strict AT terminal lines and
health traffic that cannot refresh motion evidence. Both ESP32 target builds
are checked separately from host simulation. See
[the current build/test evidence](../artifacts/sms-notices-2026-10-30/validation.md).

No live SMS, firmware upload, modem operation, robot command or reboot was used
for validation. Physical acceptance must verify real SIM/registration, a
deliberately authorized test message to the configured recipient, Pi-only
internet loss/recovery, controller-link loss, and bounded repeated faults while
the robot is secured against movement. Handset delivery and physical safety
remain unverified until those checks are performed.

Protocol references: [Air780E SMS documentation](https://docs.openluat.com/air780e/at/app/command/sms/)
and [Cloudflare trace endpoint](https://developers.cloudflare.com/fundamentals/reference/cdn-cgi-endpoint/).
