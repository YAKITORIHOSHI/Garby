"""Parking alignment from the unmasked REAR of one robot-frame LiDAR scan.

This is orientation evidence only. It never clears an obstacle or chooses a
parking distance. The configured mounting rotation/reflection has already been
applied by the scan producer; no side-wall heading or learned offset is used.
"""
from dataclasses import dataclass
import math

from hallway_geometry import fit_wall, np, WALL_INLIER_TOLERANCE_CM


@dataclass(frozen=True)
class RearWallFit:
    distance_cm: float
    heading_deg: float  # chassis relative to the wall normal, positive = left


def fit_rear_wall(frame, geometry):
    if frame is None or np is None:
        return None
    # Rotate the coordinate axes, not the sensor mounting: u goes across the
    # rear wall, v goes backwards. A rear wall is then a "left" line v=a+b*u.
    u, v = frame.ys, -frame.xs
    half_span = max(120.0, geometry.half_width_cm + 60.0)
    wall = fit_wall(u, v, "left", 10.0, behind_cm=half_span, ahead_cm=half_span,
                    min_span_cm=max(80.0, geometry.width_cm + 20.0))
    if wall is None:
        return None
    angle = math.radians(wall.angle_deg)
    residual = v * math.cos(angle) - u * math.sin(angle) - wall.distance_cm
    hits = u[(np.abs(residual) <= WALL_INLIER_TOLERANCE_CM)
             & (v > 0.0) & (np.abs(u) <= half_span)]
    # The broad surface must cross the rear centreline and cover both halves of
    # the approach. A wall step can hide one outer corner (the recorded station
    # does): orientation needs a long line, not a claim that every corner is clear.
    # Clearance is independently enforced by R/B/T and is never changed here.
    support = min(20.0, geometry.half_width_cm)
    if (int(np.sum(hits <= -support)) < 3 or int(np.sum(hits >= support)) < 3
            or int(np.sum(np.abs(hits) <= support)) < 5):
        return None
    corridor = np.sort(hits[(hits >= -support) & (hits <= support)])
    if len(corridor) < 2 or float(np.max(np.diff(corridor))) > 20.0:
        return None
    return RearWallFit(wall.distance_cm, round(-wall.angle_deg, 2))
