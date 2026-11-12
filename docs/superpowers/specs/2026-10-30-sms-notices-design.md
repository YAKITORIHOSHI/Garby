# GARBY important SMS notices

The user requested internet-loss notices and selected important faults and
recovery notices covering Firebase, sensors, emergency stop, route failure and
controller-link loss. Keep the existing configured recipient. No live text,
hardware upload or reboot is needed to implement or test this work.

Use the existing Air780E cellular SMS path. Cloud-only notifications cannot
report loss of the Pi's internet. Pi health packets over the existing BLE/UART
link give the executor enough information to compose fixed allowlisted notices;
free-form SMS text and recipient changes are not accepted from BLE.

The implementation choices below are engineering defaults. Internet checks run
in a dedicated Pi thread, use two HTTPS endpoints with bounded response reads,
and never run in motion, scan, BLE-write or Tk callbacks. Internet loss and
Firebase failure remain distinct. Unknown/unconfigured cloud state and missing
health packets must never be described as proven internet failure/recovery.

`[HEALTH:1:UUUUU]` is an exact 16-byte snapshot. The five values represent
internet, Firebase, LiDAR, bin-sensor data, and Pi thermal/power status.
Each is U (healthy), D (fault) or X (unknown/disabled). Send every five seconds
while connected, coalesce snapshots, discard stale/session-mismatched snapshots,
and never use this packet as path clearance or controller-readiness evidence.

The executor confirms sustained faults before enqueueing: internet30s, cloud60s,
LiDAR15s, bin sensors30s, thermal10s. Recovery needs10s of confirmed healthy
readings (thermal30s), and is sent only for a previously announced fault.
Missing snapshots after15s become unknown. These health notices do not change
the existing motion safety decisions or automatically restart/reset the robot.

Add local controller notices for emergency-stop/release transitions, route fault,
confirmed Pi/controller-link loss/recovery, and completed destination/return.
Preserve relevant load/gas notices. Report a return only when its route succeeds;
an ordinary reset is not evidence of returning to base. Slow obstacle reminders
to10 minutes. Exclude the synthetic route-fault flag used to unwind ESTOP.

One background modem worker owns the UART after startup. A bounded fixed-size
queue retains notices while starting/busy; category coalescing keeps current
state and prioritizes emergency/fault notices. Limit attempted sends to six
per10 minutes and at least20s apart. Expire queued notices after10 minutes.
Retry only explicit pre-submission failure, at most once after30s. An uncertain
result after Ctrl-Z is not automatically replayed. Require both modem +CMGS and
final OK before reporting network submission, never handset delivery. Modem
initialization/registration retries occur only in its background worker.

SMS still requires power, an active SIM, cellular registration and SMS service.
A full cellular outage/power failure cannot produce immediate SMS. Queues are
bounded RAM, not a durable delivery guarantee; software reset may discard them.

Validation uses mocked internet and modem boundaries, deterministic production
firmware tests, malformed/duplicate/stale telemetry and flood cases, internet
vs cloud distinction, queue/backoff/expiry/recovery behavior, and both target
builds. No actual SMS or cloud write is part of validation.
