"""One LiDAR acquisition converted to robot-frame points once (2026-11-14).

The hallway zones and walls, the moving-object tracker and the hallway-object
scene all need the same thing from a scan: its valid returns as centimetre
coordinates in the robot frame (x forward from the LiDAR, y LEFT of the chassis
centerline), in ray order. Each of them used to convert the ~700 rays itself.

The eight safety sectors (``bridge_core.partition_lidar_samples``) keep their
own validated pass and do not use this.
"""
from dataclasses import dataclass
import math
from typing import Any

try:  # numpy ships with ROS 2 on the Pi.
    import numpy as np
except ImportError:  # pragma: no cover - stripped install
    np = None


@dataclass(frozen=True)
class ScanFrame:
    index: Any               # ray index of each valid return (ascending)
    xs: Any                  # cm, forward from the LiDAR
    ys: Any                  # cm, left of the chassis centerline
    rs: Any                  # cm, measured range
    angle_increment: float   # rad between neighbouring rays (signed)
    ray_count: int           # rays in the acquisition, valid or not

    def __len__(self):
        return int(len(self.index))


def _number(value):
    try:
        return float(value)
    except (TypeError, ValueError, OverflowError):
        return math.nan


def build_scan_frame(ranges, *, angle_min, angle_increment, range_min, range_max,
                     lidar_y_cm=0.0):
    """Valid returns of one robot-frame scan, or None.

    None means numpy is missing, the scan is empty or its metadata is not a
    finite, ordered range. The caller's sector validator already turns such an
    acquisition into STOP; nothing here can clear one.
    """
    if np is None:
        return None
    try:
        angle_min, angle_increment, range_min, range_max, lidar_y_cm = (
            float(angle_min), float(angle_increment), float(range_min), float(range_max),
            float(lidar_y_cm))
        count = len(ranges)
    except (TypeError, ValueError, OverflowError):
        return None
    if (count == 0 or not all(math.isfinite(value) for value in
                              (angle_min, angle_increment, range_min, range_max, lidar_y_cm))
            or range_min < 0.0 or range_max <= range_min):
        return None
    try:
        distance = np.asarray(ranges, dtype=float)
    except (TypeError, ValueError):
        distance = np.array([_number(value) for value in ranges], dtype=float)
    if distance.ndim != 1:
        return None
    angle = angle_min + np.arange(count, dtype=float) * angle_increment
    valid = (np.isfinite(distance) & (distance > 0.0) & (distance >= range_min)
             & (distance <= range_max) & np.isfinite(angle))
    index = np.nonzero(valid)[0]
    r_cm = distance[index] * 100.0
    a = angle[index]
    return ScanFrame(index, r_cm * np.cos(a), r_cm * np.sin(a) + lidar_y_cm, r_cm,
                     angle_increment, count)
