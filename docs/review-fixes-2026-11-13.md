# Findings review and repairs — 2026-11-13

This follows `artifacts/review-2026-11-14/REVIEW.md`, the newest findings report by
modification time. Its November 14 label remains historical evidence; this repair
was performed on Ubuntu on November 13. Original reports/reproductions are unchanged.
The owner requested local testing before updating the remote repository. All work
remains in this testing copy; no remote repository update is included.

## Repairs

1. **STOP:MPU bypassing heading recovery.** Unchanged source reproduced a
   12-second outage that completed a straight without recovery/direction checks.
   Every motion start/resume now requires heading readiness. A clearance hold
   yields to that common gate on heading failure, so an MPU outage during an
   existing obstacle STOP cannot hide its five-minute deadline. Turns and
   pre-centring spins retain corrected route frames; plain/reverse moves brake
   on missing heading. Recovery never substitutes for GO or sonar clearance.
2. **New-reference recovery without walls.** Unchanged source returned true
   after a long outage without walls. After at least ten seconds and a new
   reference it now requires two independent fresh wall scans agreeing within
   1.5 degrees, within 8 degrees of the route's right-angle grid. Missing or
   disagreeing scans log `verify=WAIT_WALLS`. One five-minute budget covers
   heading and required wall evidence; timeout or an off-grid result fails the
   segment while stopped. Scans spanning another reference are retried. Ordinary
   wall gaps with continuous heading and indefinite obstacle holds remain valid.
3. **Fresh lifetime for old steering clearance.** Unchanged source granted
   steering with G=700 despite its 500 ms limit. The bridge now includes source
   age, actual mailbox time and its 30 ms transit estimate in granting/expiring
   permission. Missing age permits no steering. Walls retain their separate
   900 ms forwarding limit. Repeated acquisition IDs refresh neither product.

The Pi queues P/S with the same acquisition deadline, captured before packet
construction. It refreshes S.G after queue and writer-lock waits, before GATT.
Expired steering or age-field growth beyond 244 bytes produces STABLE; expired
PATH still produces STOP. Tests cover a deadline crossed during construction.
The installed Bleak BlueZ writer may retry WriteValue after InProgress; delays
after starting that call remain estimated. PATH has no end-to-end source timestamp.

## Dependency and previously unresolved test

NumPy is declared in the lean requirements, inherited by the Firebase profile,
and required by preflight for zones, walls and prediction. Tests exercise its
absence/presence without touching devices or cloud services. Fallback behavior
does not establish that geometry/prediction are available.

The map `queue.Empty` did not fail in the normal Ubuntu baseline (442 tests).
Freezing the fixture's `time_ns` reproduced it: synthetic scans reused a source
timestamp and production correctly rejected them. That map test now supplies
distinct source stamps independent of host clock resolution. The existing
repeated/frozen-source test still requires STOP. No production validity rule
was weakened.

## Evidence and remaining limits

`artifacts/review-fixes-2026-11-13/VERIFICATION.md` records final checks and outcomes.
That directory also contains full logs, exact commands/exits, before/after source
hashes, binary hashes and the change manifest. The root lacks Git, so original
source copies were captured before editing. The three original reproductions
fail before repair and pass afterward. Permanent tests use real production
algorithms with substituted peripherals.

The GUI preview was attempted. The sandbox cannot connect to the inherited
forwarded display or either local X socket (connect returns EPERM). The owner
authorized GUI testing, but desktop access remains blocked by the environment;
display tests skip and visual/layout acceptance is unverified. No GUI code changed.

Physical wheel motion, slip, chassis dimensions, orientation, stopping distance
and sonar fault coverage remain unverified. The review's existing sonar
no-echo/stuck-high behavior is unchanged; it cannot diagnose an unplugged sensor.
The Android concurrent-requester item remains an unproven risk without a live
cloud experiment. No live Firebase action, robot command, SMS test, firmware
upload, Pi deployment, service restart or OS reboot was performed.

## Deployment requirement

**MCU upload: Both**, complete `BLE_Receiver-Final` and `MAIN_EXECUTOR_CODE`
sketches, build `REVIEW-20261113`. The bridge consumes scan age for permission;
the executor owns common recovery and wall confirmation. Each requires its own
new revision. **Pi deployment: entire RasPi directory**, with its requirements,
for acquisition-age propagation and preflight. Wire grammar is compatible.

No upload or deployment occurred in this task. Installed identity and earlier
pending firmware remain unknown, separate from this task's requirement. Use
`DEPLOYMENT_AND_ACCEPTANCE.md` for secured acceptance and rollback.
