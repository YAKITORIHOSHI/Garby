# Expand important SMS notices

1. Add Pi health probe/snapshot tests, observe missing-feature failures, then
   implement background reachability and measured cloud/sensor/thermal state.
   Coalesce a strict16-byte health packet into ordinary low-priority BLE traffic.
2. Add bridge relay tests and a dedicated health mailbox. Reject malformed,
   stale and old-session data; preserve emergency/path priority and watchdogs.
3. Add executor tests and a bounded, single-owner modem worker. Implement
   health fault/recovery dwell and accurate local event notices, bounded
   retries/spacing, submission status, and no motion-side modem waits.
4. Run all Pi and host firmware tests, independent correctness review, both
   ESP32 target builds, and document results/deployment and physical limits.

Work is split by source ownership between Pi, bridge and executor implementers.
The complete directory is not a Git checkout; do not create/delete existing
worktrees, modify flasher caches, flash hardware or contact SMS recipients.
The earlier GUI/operator-control implementation remains active and must pass
its existing regression checks unchanged.

Completed: all four tasks, independent implementation/guidance review, 241 Pi
tests, 316 firmware host checks and both matching actual ESP32 target builds.
README, architecture, release, deployment and project-local skill/agent guidance
are updated. Evidence is in `artifacts/sms-notices-2026-10-30/validation.md`.
Firmware upload, live SMS and physical acceptance remain separate authorized
operations and were not performed.
