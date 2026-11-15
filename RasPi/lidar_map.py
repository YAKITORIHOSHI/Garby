"""Bounded, display-only geometry from one ROS LaserScan acquisition.

The map is a robot-relative scan view, not an accumulated position map. It never
fills gaps, turns infinity into a wall, or supplies motion/steering evidence.
"""
from dataclasses import dataclass
from functools import lru_cache
import math

from lidar_geometry import robot_scan_angles


MAX_MAP_POINTS = 360
_TWO_PI = 2.0 * math.pi


@dataclass(frozen=True)
class LidarMapSnapshot:
    points_m: tuple[tuple[float, float], ...]
    acquired_at: float
    yaw_offset_deg: float
    # (forward m, left m, kind, vx m/s, vy m/s) from scene_objects.scene_markers:
    # "person" (likely person), "moving" or "object". Display only.
    markers: tuple = ()


def _ray_geometry(angle):
    angle %= _TWO_PI
    # Degree-centered bins avoid splitting a cardinal ray at floating-point
    # roundoff around zero, while retaining the actual selected ray angle.
    bucket = int(angle * MAX_MAP_POINTS / _TWO_PI + 0.5) % MAX_MAP_POINTS
    return bucket, math.cos(angle), math.sin(angle)


@lru_cache(maxsize=32)
def _scan_geometry(count, angle_min, angle_increment):
    """Reuse trigonometry; the scan length varies a few points with motor speed."""
    return tuple(_ray_geometry(angle_min + index * angle_increment)
                 for index in range(count))


def build_lidar_map_snapshot(ranges, *, angle_min, angle_increment, range_min,
                             range_max, acquired_at, yaw_offset_deg=0.0,
                             reflect_left_right=False, markers=()):
    """Keep the nearest finite measured hit per degree, at its actual angle.

    Positive infinity is valid clearance evidence in the safety processor but
    has no measured endpoint to plot here. Invalid metadata yields no snapshot;
    valid scans with no finite hits yield an empty snapshot. Cache memory and
    output size are bounded independently of the number of input rays.
    """
    try:
        angle_min, angle_increment, range_min, range_max, acquired_at, yaw_offset_deg = (
            float(value) for value in (angle_min, angle_increment, range_min,
                                      range_max, acquired_at, yaw_offset_deg))
        count = len(ranges)
        if (not all(math.isfinite(value) for value in
                    (angle_min, angle_increment, range_min, range_max,
                     acquired_at, yaw_offset_deg))
                or (angle_increment == 0.0 and count > 1)
                or range_min < 0.0 or range_max <= range_min):
            return None
        angle_min, angle_increment = robot_scan_angles(
            angle_min, angle_increment, yaw_offset_deg=yaw_offset_deg,
            reflect_left_right=reflect_left_right,
        )
        if not math.isfinite(angle_min + max(0, count - 1) * angle_increment):
            return None
    except (TypeError, ValueError, OverflowError):
        return None

    geometry = (_scan_geometry(count, angle_min, angle_increment)
                if count <= 4096 else None)
    nearest = [None] * MAX_MAP_POINTS
    for index, raw_distance in enumerate(ranges):
        try:
            distance = float(raw_distance)
        except (TypeError, ValueError, OverflowError):
            continue
        if (not math.isfinite(distance) or distance <= 0.0
                or distance < range_min or distance > range_max):
            continue
        bucket, forward, left = (geometry[index] if geometry is not None else
                                 _ray_geometry(angle_min + index * angle_increment))
        previous = nearest[bucket]
        if previous is None or distance < previous[0]:
            nearest[bucket] = (distance, distance * forward, distance * left)
    return LidarMapSnapshot(
        tuple((hit[1], hit[2]) for hit in nearest if hit is not None),
        acquired_at, yaw_offset_deg, tuple(markers),
    )
