# LiDAR left/right correction — 2026-11-09

## Changes

- Correct the owner's confirmed swapped LiDAR/map LEFT and RIGHT readings with
  a robot-frame lateral reflection. Both diagonals are exchanged consistently.
- Use the same angular transform for collision sectors, nearest CL/CR clearance,
  wall estimates, distance summaries and the circular map.
- Preserve the forward/back coordinate, mounting yaw value, source acquisition
  age, thresholds, bypass behavior and GUI drawing optimizations.

The owner explicitly corrected the earlier answer of “Both”: only the GUI
LiDAR/map readings are known to be reversed; nudging has not been tried. Motor
DIR polarities, physical wheel assignment, turn targets, nudge algorithms and
gyro signs are unchanged. A sensor-side observation does not establish that
motor direction or yaw feedback is reversed.

## Coordinate mapping

Software robot coordinates remain 0 degrees front, 90 left, 180 rear and 270
right. The correction defaults `GARBY_LIDAR_REFLECT_LEFT_RIGHT=1` and applies:

```text
unreflected = raw_angle + GARBY_LIDAR_YAW_OFFSET_DEG
robot_angle = -unreflected
robot_angle_increment = -raw_angle_increment
```

For every ray this retains `forward = distance * cos(unreflected)` and reverses
only `left = distance * sin(unreflected)`. Rotation is applied before reflection,
so front/back are preserved even with a nonzero mounting yaw offset. This is a
reflection, not a 180-degree rotation. The generic mathematical partition and
Canvas projection conventions remain unchanged.

Set `GARBY_LIDAR_REFLECT_LEFT_RIGHT=0` for an alternate installation only when
observation confirms that it needs the previous orientation. The correction is
based on the owner's report, not a measured mounting survey. The local ROS driver
publishes SDK angle metadata; no driver arithmetic defect is established, and
its `inverted`/`reversion` settings are unchanged.

## Verification and deployment

Regression tests execute the production scan consumer, including summaries,
wall values, nearest CL/CR and map endpoints. They cover default configuration,
opt-out, nonzero yaw, front/back preservation and acquisition timestamps.
Existing raw-convention fixtures explicitly retain their previous orientation;
the new production-default tests check the reported installation independently.
The before-fix test log records seven failing assertions on unchanged production.

Exact final checks, source hashes and unchanged firmware/GUI identities are in
`artifacts/lidar-left-right-2026-11-09/validation.md`. Historical direction and
centering reports are preserved as dated evidence.

**MCU upload for this correction: None.** Deploy the complete `RasPi/` directory
and use the configured reflection default. Earlier unuploaded Executor direction
firmware remains a separate requirement; no BLE firmware source changed here.
No robot command, firmware upload or service restart is part of this software work.

While parked, compare an object at real left, right, front and rear against both
map and summaries. The physical mounting and the effect of correctly oriented
sensor inputs on untried nudging remain hardware acceptance items. Software
checks cannot establish physical wheel travel, straightness or stopping distance.
