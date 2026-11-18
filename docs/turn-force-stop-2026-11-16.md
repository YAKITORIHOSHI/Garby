# Immediate stop at the turn target — 2026-11-16

The owner requested a force stop when a left/right turn reaches 90 degrees instead of
requesting a normal stop and waiting for the wheels to finish decelerating.

`spinToward()` in `MAIN_EXECUTOR_CODE.cpp` previously broke out 0.3 degrees before the
estimated target, called `stopMove()` on both steppers, and waited for them to stop. The
installed FastAccelStepper 1.2.7 source confirms that `stopMove()` only requests normal
deceleration. Its `forceStop()` still drains queued commands; its
`forceStopAndNewPosition()` stops the ramp, aborts the queue and assigns the supplied
wheel coordinate. The executor already wraps the last method for both wheels in
`emergencyStopMotors()`, which does not latch the operator emergency stop.

The reached-target branch now calls that existing immediate-stop helper. The early
allowance is zero: the stop happens on the first control iteration at/past the estimated
target heading in the requested direction. That estimate still combines a fresh MPU
sample, wheel travel since the sample, and the existing 10 ms sample-latency compensation.
The approach speed profile still slows before the endpoint. Both wheel coordinates are
retained at their sampled counts instead of being reset to zero.

The 250 ms settled measurement, turn tolerance and bounded correction passes remain.
Ordinary route turns still check two agreeing wall scans; parking orientation still uses
its separate rear-wall confirmation. The shared spin endpoint also applies to other
MPU-measured angles, hallway alignment, centring and parking correction spins. Obstacle
holds and non-target exits keep their existing stop paths. The step-count turns in a
RunStart list and held remote arrows use their existing controls.

Only executor production sources change; there is no wire-protocol change. Build ID:
`TURNSTOP-20261116`. Upload requirement for this task: **Executor only**, the complete
`MAIN_EXECUTOR_CODE` sketch. No additional bridge flash or Pi deployment is needed.
Earlier `RUNSTART-20261116B` bridge/Pi changes retain their separate deployment requirement;
their current installed state is unverified.

Software evidence is saved in
[`artifacts/turn-force-stop-2026-11-16/VERIFICATION.md`](../artifacts/turn-force-stop-2026-11-16/VERIFICATION.md),
including the failing-before regression, actual commands and exit codes, source/binary
hashes and an exact change manifest. The regression exercises the production spin with
an optional pending-pulse tail in the host stepper: a normal stop emits that tail, while a
queue abort removes it. This is a boundary test, not a physical deceleration model.

This change removes commanded run-out after the estimated heading target. It cannot
guarantee an exact physical 90-degree stop: MPU scale/bias, sample timing, wheel slip and
chassis inertia remain unmeasured. No firmware upload, robot movement, service restart or
physical acceptance is performed by software verification. Supervised acceptance is
described in `DEPLOYMENT_AND_ACCEPTANCE.md`.
