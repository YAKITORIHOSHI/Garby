# Obstacle STOP priority and LiDAR interlock — 2026-10-30

The read-only motion interlock in this historical report was removed at the
user's request on October 31. See the [bypass correction and workload review](bypass-performance-2026-10-31.md)
for current behavior; queue priority and path-expiry protections remain.

## Changes

- The Pi BLE mailbox now preserves urgent priority when a newer blocked, stale,
  or malformed `P:` snapshot replaces an older path. The exact all-clear forms
  remain ordinary traffic. Operator ESTOP stays ahead of PATH and cannot be
  evicted by it.
- Fresh path snapshots carry a monotonic LiDAR evidence deadline through the
  queue. After waiting for the serialized writer lock, the Pi changes an
  expired snapshot to `F=S|B=S` immediately before beginning the BLE write.
- All-front/All-back read-only mode leaves distance readings visible but forces
  STOP. Manual start is disabled until both threshold checks are restored. A
  pending start is cancelled when read-only mode is selected.
- The Controls tab states that read-only mode blocks motion and that the current
  LiDAR reports generic returns rather than identifying people.

## Verification

Focused checks passed for mailbox admission/replacement, ESTOP dominance,
sequence rollover, BLE-writer expiration, read-only STOP enforcement, manual
start gating, missing/stale scans, and Controls-tab warnings. The complete Pi
suite and source sanity results, exact source hashes, and live reload status are
recorded in `artifacts/obstacle-stop-review-2026-10-30/final-verification.json`.
The full Python suite passed 277 tests with no skips; source sanity passed for
30 C/C++ files and 114 JSON files.

The test fixtures substitute BLE, ROS, sensors and motors. They do not prove
radio delivery timing, real braking, sensor coverage, or that a flashed board
matches source. No hardware upload or route movement was part of this revision.

## Detection and timing limits

The production scanner is a 2D LiDAR. A person is only represented as a generic
obstacle if a valid return intersects its scan plane and passes current range
and coverage checks. There is no production human/person classifier. `H` is a
legacy parser value that also stops; it is not classifier output. No camera
person-detection feed was present in the reviewed Pi runtime. Do not describe
this system as providing dependable human detection or injury prevention.

The new expiry check closes time spent queued on the Pi and waiting for its
writer lock. The S3 currently timestamps PATH at BLE receipt, so transport time
after the Pi begins a GATT operation is not end-to-end LiDAR source-age checked.
Closing that remaining gap needs a coordinated acquisition-age wire contract,
tests on both controllers, and actual target builds. The configured 0.8 s Pi
scan-age budget and controller watchdogs are not measured stopping-time bounds.

The existing 95 cm front, 35 cm rear and 22 cm side thresholds and controlled
braking remain unchanged. Physical speed, load-dependent stopping distance,
scan-plane coverage, and obstacle detection near the sensor's minimum range have
not been measured. Use secured soft targets and an independent physical stop for
future supervised acceptance; do not put a person in the test path.

The production Pi process is currently stopped. The persistent operator STOP
file remains at `~/.config/garby/operator-stop` and will be loaded on the next
runtime start. The operator reported Garby was at home and did not know whether
it was at its calibrated route start, so no route trial was attempted. Earlier
live GUI checks preserved that stop file. Software verification is not physical
acceptance.
