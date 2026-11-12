# Codebase optimization review — 2026-10-29

Scope: active executor, BLE/MPU bridge, Pi runtime, Firebase worker/transport,
Android dashboard subscriptions and startup integration. This is a review, not
an implementation change. Only this report and an offline measurement script/log
were added. No physical hardware, cloud writes, firmware build or flashing.

## Priority 1: fix the load trigger before tuning performance

`MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.h:24` sets LOAD_TRIGGER_KG to 0.0.
`MAIN_EXECUTOR_CODE/MAIN_EXECUTOR_CODE.ino:177` counts samples greater than or
equal to that threshold, and line 205 accepts two confirmations. The sampler
normalizes small readings to zero and uses the absolute value of other readings.
Consequently two fresh valid empty-bin readings qualify for outbound RUNNING.
Movement still requires the existing path gates, but loading is not required to
trigger the route. This conclusion comes from source tracing, not a floor test.

Change: choose a positive, calibrated collection threshold and separate explicit
bench triggering from production configuration. Add an empty-bin regression and
checks just below/at the intended threshold. Also review negative load treatment:
a large negative tare drift currently becomes a positive load through fabsf.
The intended load threshold is a product/hardware choice, not a value to guess.

## Priority 2: bound Firebase shutdown and report connection truthfully

`RasPi/bridge_core.py:301` joins the writer for the close timeout but leaves
flush/retry enabled. `_run()` requeues a failed batch and continues retrying.
The offline probe used a deliberately short backoff: close(flush=True, .05)
returned with the worker alive after 3 attempts; 120 ms later it had made 9.
An explicit close(flush=False) terminated it. Production uses slower backoff,
but the same lifecycle. Its forced process exit limits shutdown duration; it
can terminate with unflushed data instead of completing a clean worker shutdown.

Change: define a real flush deadline, abort remaining retries when it expires,
return a flush outcome, and use explicit network request timeouts that fit the
shutdown budget. A join timeout cannot cancel a network call already in progress.
Test permanent outage, delayed network calls and normal flush completion.

`RasPi/final_w_serial.py:1192` marks Firebase healthy when the manager object
exists. `firebase_connection_loop()` assigns that object before its first network
read; write failures do not invalidate it. This is initialization state, not
connection evidence. Track last successful request/write, pending data and last
failure separately. The dashboard should distinguish configured, reachable and
backlogged/offline states. No live Firebase outage was induced in this review.

## Priority 3: reduce Firebase work while idle

`RasPi/final_w_serial.py:311,599` uses a two-second reset polling interval.
At negligible request latency this approaches 43,200 reads per day even without
new commands; actual count decreases with request time/backoff. This is a request
estimate, not a billing estimate or observed network measurement.

Change: evaluate a narrow command subscription or slower/adaptive idle polling,
while retaining prompt checks during an active return. Preserve request identity,
expiry, deduplication, acknowledged retries and matching completion. Measure
command latency before selecting an idle interval. Never reduce safety path
message frequency as part of this cloud optimization.

## Priority 4: reduce mobile subscriptions and background processing

`DeviceViewModel.kt:58-62` starts five sensor collectors. Each call to
`GarbyRealtimeDb.kt:286-287` attaches both current and legacy listeners: ten
sensor subscriptions, before status/connection/return listeners. Pi telemetry
writes both schemas, so both subscriptions receive updates. This compatibility
behavior is deliberate, but has an ongoing callback/data cost.

Change: after checking deployment compatibility, prefer the device schema and
activate legacy fallback only when needed, or share a narrowly scoped sensor
snapshot. Measure payload sizes and callback counts; fewer listeners alone do
not guarantee fewer downloaded bytes.

`DeviceViewModel.kt:93,161` owns collectors and the five-second freshness ticker
in viewModelScope; `MainDashboard.kt:75` uses collectAsState. Work can continue
while the retained ViewModel is in the background. Make UI observation and
upstream subscription lifetime follow screen visibility, while keeping any
intentional background notification requirements separate. Verify that freshness
and return readiness are recalculated immediately on foregrounding. Android
runtime/battery measurements were not performed.

## Priority 5: modest Pi scan-path improvements, after the above

Measured locally using 500 synthetic clear scans per size after 50 warm-up scans:

| Rays | Median callback | p95 callback | Maximum |
| --- | ---: | ---: | ---: |
| 360 | 0.519 ms | 0.546 ms | 0.611 ms |
| 720 | 0.981 ms | 1.073 ms | 1.417 ms |
| 1440 | 1.898 ms | 2.241 ms | 2.597 ms |

These times cover the production scan callback with peripheral substitutes;
they exclude actual DDS scheduling, BLE transmission, hardware contention and
motor braking. They do not establish end-to-end stop latency.

`final_w_serial.py:1778,1841` builds display dictionaries/medians and queues them
for every scan. Headless mode has no consumer for that display queue. The probe
confirmed it stays at five frames and replaces old frames. Memory is bounded;
the waste is repeated work, not a growing queue. Skip display construction in
headless mode or publish it at a lower independent rate in GUI mode.

Profiling points to partition_lidar_samples / nearest_lidar_sector as the largest
scan CPU cost. A cached index-to-sector/cone lookup keyed by scan length, angle
metadata, yaw and cone could reduce repeated angle calculations. This is a later
optimization: the measured callback is already short. Any cache must invalidate
on geometry changes and preserve invalid-reading handling, sector boundaries,
negative angular increments and single thin-obstacle detection.

## Lower-priority firmware/maintenance candidates

- Main sketch lines 137-143 drain modem/debug serial without a byte budget,
  unlike the bounded bridge UART parser. Apply a per-iteration budget and check
  progress under serial flooding; do not assume this is the current bottleneck.
- Repeated buzzer task creation (executor cpp lines 851,1737) could be replaced
  by a persistent notification task or timed output state. Allocation/jitter
  benefits need an ESP32 heap/timing soak test before prioritizing this.
- The production Pi module contains unused Task/ExecutorService infrastructure;
  remove it only after reference checks. Avoid a broad firmware String/parser
  rewrite without heap measurements and protocol regression coverage.
- Retain current bounded BLE queues, fail-closed freshness, sonar confirmation,
  MPU fault handling and local STOP precedence. Do not widen watchdogs or raise
  motor speeds to hide pauses. Chair bypass remains a separate geometry/route
  problem, not a CPU optimization.

## Evidence and proposed order

Reproduce offline measurements:

```bash
python3 -B artifacts/optimization-review-2026-10-29/probe.py
```

[Probe source](artifacts/optimization-review-2026-10-29/probe.py) and
[recorded results](artifacts/optimization-review-2026-10-29/probe-results.txt).

Recommended first batch: correct the production load-trigger configuration,
bound writer shutdown, and expose real Firebase health. Then reduce cloud polling
and mobile background/legacy work. Skip headless display processing before
considering a scan-geometry cache. Establish before/after measurements and retain
whole-project behavioral checks for every implementation batch.
