# LiDAR bypass correction and Pi workload review — 2026-10-31

## Restored control behavior

The October 31 correction removes the extra motion interlock that had made the
front/rear read-only controls block movement and cancel pending starts. With a
fresh, valid scan, All-front bypasses only the front group's distance threshold;
All-back does the same for the rear group. Readings remain visible. The two
controls act independently, and a pending manual start and its status query
remain active when either control changes.

The bypass does not make unknown data clear. Missing or stale LiDAR evidence
still stops the path. The direction-independent side obstacle envelope still
blocks both directions after the bypass is applied. Emergency stop stays
latched; motor readiness, sequence, scan-age, and BLE-write expiry checks
remain. Nudge `CF`/`CL`/`CR` nearest-object clearances are still sent and checked
independently from the wider steering medians. This is a threshold bypass, not
a global permission or replacement for the readiness handshake.

Confirmed obstacle-sonar `SONAR_OFF` remains a separate controller setting. It
disables only the executor's local sonar gate and parks the servo at
`DEFAULT_VIEW` (70 degrees). It does not disable LiDAR, bin-level sensing,
side/nudge clearance, or emergency stop. Changing sonar mode still requires
stationary IDLE, and re-enabling sonar requires fresh clear confirmations. No
executor or bridge firmware source was changed in this correction.

## Pi scan workload

`partition_lidar_samples` now keeps a four-entry, bounded cache of immutable
ray-angle classifications for scans up to 4096 samples. A cache entry contains
only each ray's sector and angular offset. Every scan still validates and
classifies its current ranges, invalid returns, thresholds, and steering cone;
large or non-finite geometries use the uncached path. No safety result or
distance is retained across scans.

The offline profiler called the production scan callback with synthetic scans
for 500 measured iterations after warm-up. Median callback time changed as
follows:

| Rays per scan | Before (ms) | After (ms) | Reduction |
| ---: | ---: | ---: | ---: |
| 360 | 0.509 | 0.226 | 55.6% |
| 690 | 0.938 | 0.391 | 58.3% |
| 1440 | 1.896 | 0.749 | 60.5% |

The 690-ray case reduced measured callback CPU for 510 calls from 0.479 s to
0.202 s. A deterministic differential run compared old and new partition
helpers for 192 geometries and range sets; all returned partitions and valid
point counts were exactly equal. Coverage included invalid ranges, sector
boundaries, reverse scan direction, changed geometry and cone, and the
oversized uncached path.

These are isolated synthetic callback measurements, not whole-runtime Pi CPU,
scan-drop, or thermal acceptance. The Pi 5 Model B Rev 1.1 reported 43.55 C
when sampled during final verification. `vcgencmd get_throttled` could not read
firmware flags because this environment has no accessible `/dev/vcio`; this
sample cannot prove that the live robot never throttles.

## Verification evidence

- `python3 -B -m unittest discover -s RasPi -p 'test_*.py'`: 280 passed, no
  skips, in 15.339 s. This includes read-only bypass, independent groups,
  retained measurements, side-obstacle protection, stale/missing scans,
  pending starts, ESTOP and nearest nudge clearance.
- `python3 -B tools/run_firmware_tests.py`: 407 firmware behavioral checks
  passed. The sonar-disable contract reports the servo centered at default
  view. These are host-substituted firmware checks, not target builds or board
  uploads.
- `python3 -B tools/source_sanity.py .`: passed (30 C/C++ files and 120 JSON
  files).
- LiDAR partition differential: 192/192 exact matches, recorded in
  `artifacts/bypass-performance-2026-10-31/partition-equivalence.json`.
- Before/after callback samples are in `scan-before.json` and `scan-after.json`;
  commands and test logs are preserved in the same dated artifact directory.

No production Pi service was launched or restarted, no MCU was flashed, and no
live robot command or route trial was sent. The code is implemented and
software-tested, but it is not confirmed deployed to the Pi or physically
accepted. The expected persistent operator-stop marker was not present at
`/home/garby/.config/garby/operator-stop` during final inspection, so this
verification does not claim that an emergency-stop latch is active. Before
live operation, confirm the controller and GUI stop state and keep the
physical stop available. Deploy the complete `RasPi/` directory and verify the
controls with the robot secured and the acceptance steps in
`DEPLOYMENT_AND_ACCEPTANCE.md`. Do not treat generic 2D LiDAR returns as human
recognition or as proof of injury prevention.
