# Implemented optimizations — 2026-10-29

User selected a 4 kg load threshold and authorized the remaining review changes.
The active motor sketch is MAIN_EXECUTOR_CODE. No target build or flashing was
performed. Host tests execute the source with simulated peripherals.

## Changes

- **4 kg trigger:** two fresh qualifying samples of the corrected, filtered load
  are required. Empty/below-threshold readings cannot start the route. Existing
  10 g correction and small tare deadband remain; a raw 4.01 kg test sample is
  corrected to 4.00 kg. Negative readings outside that deadband are unavailable,
  rather than converted to positive load. Gas-trigger behavior is preserved.
- **Firebase writer shutdown:** close has a real flush deadline and reports
  whether flushing completed. Pending retries stop at expiry; no later batch is
  started. A network operation already in flight cannot be forcibly cancelled by
  the thread; its result is not guaranteed at timeout. New Admin SDK initialization
  sets a 2-second HTTP timeout; the application keeps its final shutdown watchdog.
  SDK retries/token acquisition can still extend total call duration.
- **Cloud health:** the Pi display reports Connecting, Connected, Pending, Offline
  or Stale from completed cloud operations and queued/in-flight fields. A manager
  object alone is no longer connection evidence. Success expires after 60 seconds.
  A later successful read proves reachability but does not prove queued writes
  succeeded; pending work remains visible. This is not Android permission testing.
- **Cloud polling:** reset requests are checked every 10 seconds when idle, every
  2 seconds at the destination or while a return request is pending/in flight.
  Rough ideal idle request count falls from 43,200/day to 8,640/day (80% reduction;
  latency/backoff changes actual counts). An idle-to-active transition during the
  wait may take up to the remaining 10 seconds to be observed. Command expiry,
  identity, acknowledgement and completion safeguards are unchanged.
- **Headless scans:** production headless mode supplies no display queue, so it
  skips GUI dictionary/median construction and frame replacement. Safety/steering
  computation and urgent STOP behavior remain active. Tests compare packets from
  headless and display modes for clear, blocked and unavailable scans.
- **Android dashboard:** collectors and the freshness ticker are children of a
  STARTED-lifecycle coroutine; leaving/backgrounding the dashboard cancels them
  and detaches Firebase listeners. Foregrounding resets old readiness and reloads
  state. This follows the [Android lifecycle guidance](https://developer.android.com/topic/libraries/architecture/coroutines).
- **Android sensor subscriptions:** when a device-schema record exists, only its
  listener remains attached. A missing record attaches legacy fallback; when the
  primary appears, fallback is removed. Queued callbacks from detached listeners
  are ignored. Present unavailable/malformed/stale primary data does not fall back
  to an older apparently healthy value. Five sensors normally use five listeners
  instead of ten; legacy-only deployments retain the fallback listeners.
- **Executor serial work:** modem/debug passthrough now forwards at most 128 bytes
  per direction per loop iteration. Startup modem draining is bounded too. This
  bounds byte work, not the electrical UART transmit time of each write.
- Removed the unused Task/ExecutorService implementation in the production Pi
  module. The separate interactive simulator remains separate.

Geometry caching, a buzzer-task redesign, motor speed/acceleration changes and
parser rewrites were not justified by these measurements and were left unchanged.
The existing chair response remains stop-and-wait; this is not a detour planner.

## Verification

- **153 Python tests passed**, including new shutdown, cloud health/polling,
  headless equivalence and actual Tk badge tests.
- **145 firmware behavioral checks passed**, including the 4 kg boundary,
  empty/negative readings, stale samples, bounded serial bursts and complete route.
- C/C++ structural checks, JSON parsing and Android static checks passed (22 Kotlin
  files). Android execution/unit tests and target compilation were not run.
- Eight offline raw-LiDAR scenarios passed.
- Generated startup unit passed systemd-analyze verify; shell syntax passed.
  The service was not enabled or started.
- Legacy audit script remains absent/unverified. Live authenticated Firebase,
  hardware wiring/timing, motor torque, sonar coverage and real stopping distance
  remain unverified; credentials/hardware were not available for those checks.

Repeat whole-project host checks:

```bash
python3 -B tools/verify_project.py --host-only
python3 -B RasPi/check_lidar_offline.py
```

[Host validation log](artifacts/optimization-2026-10-29/host-validation.log),
[LiDAR log](artifacts/optimization-2026-10-29/lidar-offline.log),
[headless profile and shutdown reproduction](artifacts/optimization-2026-10-29/headless-profile.txt).

Post-change headless median callback times were 0.510/0.963/1.907 ms for
360/720/1440 synthetic rays; display queue frames were zero. Prior medians were
0.519/0.981/1.898 ms. Differences are small/noisy: no substantial scan-speed gain
is claimed. The shutdown probe changed from an alive worker with 3 -> 9 retries
after close to a stopped worker with 3 -> 3 attempts.

## Remaining on-device checks

1. Calibrate/tare empty, test below 4 kg and at/above 4 kg; verify real loaded
   braking before floor operation. Significant negative readings require fixing
   wiring/calibration rather than flipping the sign in software.
2. In Android, verify primary-only, legacy-only, primary appearing/disappearing,
   unavailable sentinels, background/foreground, rotation and screen navigation.
   Verify that listeners do not accumulate and return stays disabled when stale.
3. With Firebase credentials, test outage/recovery, backlog indication and shutdown
   during a slow request. Successful Admin access does not validate client rules.
4. Install runtime dependencies and configure serial devices before enabling the
   previously supplied startup service. Missing LiDAR continues to hold motion.
