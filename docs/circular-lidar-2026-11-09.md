# Circular LiDAR readings, November 9, 2026

The production monitor now shows the measured scan around the robot, with up to
360 angular bins. Front is at the top, left is at the left, and the robot stays
at the center. Use **8 safety distances** to return to the existing summaries,
or **Circular scan** to reopen the plot. **Range** cycles through 2, 4 and 8 metres.
The plot includes range rings, the nearest measured return and the mounting yaw
offset. The offset describes the LiDAR mount, rather than a compass heading.

This is the current local scan. It does not track robot position or accumulate
a floor map. Partial scans retain their gaps. Missing, invalid and infinite
returns do not become walls, and expired scan points disappear. Changing the
display range does not change sensor range or obstacle thresholds.

## Data and workload

`RasPi/lidar_map.py` builds an immutable `LidarMapSnapshot` from one accepted
production acquisition. Each 1° bin retains its nearest finite, positive,
in-range measurement at the measurement's original angle. Degree-centered bins
avoid floating-point loss of points at exact degree boundaries. ROS angle
metadata, clockwise scans and `GARBY_LIDAR_YAW_OFFSET_DEG` are respected.

The snapshot retains the scan's age-adjusted monotonic acquisition time. Queue
delay cannot give the plot or summaries a new freshness budget. Invalid scan
metadata produces no map snapshot; scans containing only valid no-return
infinities produce an empty plot. The renderer retains no point history.

Display geometry is computed after the current urgent path dispatch, outside
the safety-history lock. Headless operation skips it. Geometry failures are
isolated from path decisions and their logs are rate-limited. The trig cache has
four entries, each with at most 4096 rays; larger inputs are processed without
retaining their geometry. Each output contains at most 360 points.

The Tk renderer creates 376 Canvas items once, including 360 point markers, and
reuses them. Hidden views defer Canvas work; unchanged frames skip redraws. The
last rendered snapshot is retained so Python object-ID reuse cannot suppress a
new frame after hidden updates. The existing 150 ms monitor poll remains.
Host microbenchmarks recorded approximately 141 microseconds per cached 360-ray
snapshot and 372 microseconds per cached 1440-ray snapshot. These are local host
measurements, without a real display or robot workload.

## Preserved behavior

The existing eight full-circle safety sectors, nearest-object clearance,
steering wall estimates, thresholds and all BLE/UART fields are preserved.
The plot supplies no motion permission. Front/rear read-only controls keep their
selected threshold bypasses and readings; side clearance and missing/stale path
evidence still apply. Sonar-off operation, latched emergency stop, readiness,
heading reference and the existing steering workarounds are preserved.

The GUI accepts the existing five-field frames and the optional sixth snapshot.
The first received circular snapshot opens the plot; a subsequent user choice
of summaries persists. The manual eight-direction simulator remains a separate
bench producer and cannot create additional measured scan detail.

## Validation and deployment

The combined software checks passed: 352 Python cases ran, with 28 Tk window
cases skipped because no display was accessible; 826 firmware assertions passed.
Display-independent tests exercise the production renderer for orientation,
nearest return, stale hiding, bounded item reuse, hidden views and queued source
age. Eight geometry tests and runtime regressions cover actual producer wiring,
invalid data, headless work avoidance and preserved path decisions.

Real Tk layout, button geometry and screenshot inspection remain unverified in
this environment. The display integration tests remain in the suite for a
machine with a working display. Preview the dashboard with synthetic data using:

```bash
python3 -B RasPi/preview_ui.py --size 800x480
```

Deploy the entire `RasPi/` directory, including the new helper. Apply the
[executor direction correction](motor-direction-2026-11-09.md) separately when
hardware deployment is authorized. The S3 source is unchanged. No firmware was
uploaded, robot operated, service restarted or cloud/SMS action performed.

While parked, compare an object at the real front and left against both views
before a supervised drive. Motor polarity does not establish LiDAR mounting
orientation. Exact checks, source/binary hashes and limits are recorded in
[the dated validation report](../artifacts/front-direction-circular-2026-11-09/validation.md).
