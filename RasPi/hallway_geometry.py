"""Chassis-sized obstacle zones and hallway wall line fits from one LiDAR scan.

Frame (centimetres): x points forward from the LiDAR, y points LEFT from the
chassis centerline. Ray angles must already be in the robot frame
(``lidar_geometry.robot_scan_angles``: mount yaw, then the left/right
reflection), so this module never re-derives the mounting convention.

Two independent products come out of one acquisition:

* Zones. Simple rectangles and a circle sized from the chassis. They only ADD
  stop/slow evidence to the existing sector thresholds in final_w_serial.py;
  they can never clear a sector STOP.
* Walls. A deterministic Hough search plus least-squares refinement for one
  straight wall on each side. A wall gives its perpendicular distance from the
  chassis centerline and its direction. The robot's heading relative to the
  hallway is minus that direction. Doorways, posts and open doors are outliers
  of the fit, not walls. A side without a clean line reports no wall: that
  removes centering evidence, never obstacle evidence.

Nothing here keeps state between scans. Each result describes exactly one
acquisition, so a repeated scan cannot earn confidence downstream.
"""
from dataclasses import dataclass
import math
import os

try:  # Required by requirements.txt and runtime preflight; fail closed if absent.
    import numpy as np
except ImportError:  # pragma: no cover - exercised only on a stripped install
    np = None

from scan_frame import build_scan_frame


# ---------------------------------------------------------------------------
# Chassis geometry
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class ChassisGeometry:
    """Chassis outline relative to the LiDAR. Defaults are assumptions.

    The owner gave a 60 cm chassis width on 2026-11-11. The LiDAR-to-edge
    distances, the LiDAR lateral offset and the rotation centre are NOT
    measured; the defaults place the LiDAR at the centre of a 60 x 60 cm body.
    """
    width_cm: float = 60.0
    front_cm: float = 30.0          # LiDAR to the front edge
    rear_cm: float = 30.0           # LiDAR to the rear edge
    lidar_y_cm: float = 0.0         # LiDAR lateral offset from the centerline, + = left
    rotation_x_cm: float = 0.0      # spin centre (wheel axle midpoint) ahead of the LiDAR
    corridor_margin_cm: float = 10.0
    side_margin_cm: float = 2.0
    turn_margin_cm: float = 8.0
    front_stop_cm: float = 95.0     # same straight-ahead limit as THRESHOLD_CM, from the LiDAR
    front_slow_cm: float = 160.0

    @property
    def half_width_cm(self):
        return self.width_cm / 2.0

    @property
    def corridor_half_width_cm(self):
        return self.half_width_cm + self.corridor_margin_cm

    @property
    def turn_radius_cm(self):
        reach_x = max(self.front_cm - self.rotation_x_cm, self.rear_cm + self.rotation_x_cm)
        return math.hypot(reach_x, self.half_width_cm) + self.turn_margin_cm


_GEOMETRY_ENV = (
    # name, attribute, minimum, maximum
    ("GARBY_CHASSIS_WIDTH_CM", "width_cm", 10.0, 200.0),
    ("GARBY_CHASSIS_FRONT_CM", "front_cm", 0.0, 150.0),
    ("GARBY_CHASSIS_REAR_CM", "rear_cm", 0.0, 150.0),
    ("GARBY_LIDAR_Y_OFFSET_CM", "lidar_y_cm", -50.0, 50.0),
    ("GARBY_ROTATION_X_CM", "rotation_x_cm", -100.0, 100.0),
)


def geometry_from_environment(environ=None, warn=None):
    """Read the chassis outline; an invalid value keeps its default and warns."""
    environ = os.environ if environ is None else environ
    values = {}
    for name, attribute, low, high in _GEOMETRY_ENV:
        text = environ.get(name)
        if text is None or not str(text).strip():
            continue
        try:
            value = float(text)
            if not math.isfinite(value) or not low <= value <= high:
                raise ValueError
        except (TypeError, ValueError):
            if warn is not None:
                warn(f"Invalid {name}={text!r}; using the default")
            continue
        values[attribute] = value
    geometry = ChassisGeometry(**values)
    if warn is not None and geometry.front_cm >= geometry.front_stop_cm:
        warn("Chassis front edge is beyond the 95 cm front stop; the front box is empty")
    return geometry


# ---------------------------------------------------------------------------
# Wall fit tunables (centimetres / degrees)
# ---------------------------------------------------------------------------
WALL_WINDOW_BEHIND_CM = 80.0     # wall points used behind the LiDAR
WALL_WINDOW_AHEAD_CM = 160.0     # and ahead of it
# A wall nearer than this is not used: anything inside the body's half width plus
# its side margin is already a side-band STOP (analyze_hallway raises the limit to
# that). The first live scan (2026-11-11, on a bench) fitted clutter 29 cm away
# as a 17 degree "wall"; the geometry limit (32 cm by default) rejects it.
WALL_MIN_LATERAL_CM = 20.0
WALL_MAX_LATERAL_CM = 300.0      # an open door's room interior (340-360 cm) is not the wall
WALL_MAX_ANGLE_DEG = 20.0        # walls are near-parallel to travel; a door leaf is not
WALL_INLIER_TOLERANCE_CM = 3.0
WALL_MIN_INLIERS = 15
WALL_MIN_SPAN_CM = 60.0
WALL_MAX_RMS_CM = 2.5
# A flat wall has no bow. About 1 m of a 2 m-radius arc (a round room, a large
# pillar, a curved desk) fits inside the inlier tolerance, but its middle third
# sits several cm off its ends. Noise on a real wall moves this by a few mm.
WALL_MAX_BOW_CM = 1.5
WALLS_AGREE_DEG = 3.0            # left and right must be parallel to give one heading
# Far fit (2026-11-14, owner: "use the data for front and rear reading to help the
# system to walk straight"). The heading error of a line fit falls with the
# length of wall it is fitted to, and the near window is only 2.4 m long. A
# second fit over the wall points far ahead and far behind gives the same
# direction with a fraction of the noise. It replaces only the HEADING, and only
# when it is a long clean wall that agrees with the near fit: a door, an
# opening, a corner or clutter fails that and the near fit stands alone. Wall
# DISTANCES always come from the near window, where the robot is.
FAR_WINDOW_BEHIND_CM = 300.0
FAR_WINDOW_AHEAD_CM = 500.0
FAR_MIN_SPAN_CM = 350.0          # of wall actually seen; less is no better than the near fit
FAR_MIN_INLIERS = 30
FAR_AGREE_DEG = 1.5              # far and near headings must agree this closely
FAR_DISTANCE_AGREE_CM = 4.0      # and be the same wall


@dataclass(frozen=True)
class WallFit:
    distance_cm: float   # perpendicular distance from the centerline origin
    angle_deg: float     # wall direction in the robot frame, + = turning CCW (left)
    inliers: int
    span_cm: float
    rms_cm: float


@dataclass(frozen=True)
class HallwayAnalysis:
    front_stop: bool
    front_slow: bool
    side_left: bool
    side_right: bool
    turn_front: bool
    turn_back: bool
    front_gap_cm: float | None        # nearest corridor return beyond the front edge
    left: WallFit | None
    right: WallFit | None
    heading_deg: float | None         # robot heading relative to the walls, + = rotated left
    heading_source: str = "near"      # "far" when the long-baseline fit supplied heading_deg
    near_heading_deg: float | None = None   # the near window's own heading, for comparison


def _robot_points(ranges, angle_min, angle_increment, range_min, range_max, lidar_y_cm):
    frame = build_scan_frame(ranges, angle_min=angle_min, angle_increment=angle_increment,
                             range_min=range_min, range_max=range_max, lidar_y_cm=lidar_y_cm)
    return None if frame is None else (frame.xs, frame.ys)


def _densest_offset(xs, ys, theta):
    """Most points within +/- tolerance of one line at direction theta."""
    offsets = np.sort(ys * math.cos(theta) - xs * math.sin(theta))
    upper = np.searchsorted(offsets, offsets + 2.0 * WALL_INLIER_TOLERANCE_CM, side="right")
    counts = upper - np.arange(offsets.size)
    best = int(np.argmax(counts))
    centre = 0.5 * (offsets[best] + offsets[upper[best] - 1])
    return int(counts[best]), centre


def fit_wall(xs, ys, side, min_lateral_cm=WALL_MIN_LATERAL_CM, *,
             behind_cm=WALL_WINDOW_BEHIND_CM, ahead_cm=WALL_WINDOW_AHEAD_CM,
             min_span_cm=WALL_MIN_SPAN_CM, min_inliers=WALL_MIN_INLIERS):
    """One straight wall on ``side`` ('left' or 'right'), or None."""
    if np is None:
        return None
    sign = 1.0 if side == "left" else -1.0
    lateral = sign * ys
    # Points a little nearer than the limit are kept for the fit, so a wall just
    # inside it is fitted whole (and then rejected) instead of by its noise tail.
    window = ((lateral >= min_lateral_cm - 2.0 * WALL_INLIER_TOLERANCE_CM)
              & (lateral <= WALL_MAX_LATERAL_CM)
              & (xs >= -behind_cm) & (xs <= ahead_cm))
    px, py = xs[window], ys[window]
    if px.size < min_inliers:
        return None

    # Coarse then fine angle search. Deterministic, so a test scan and a robot
    # scan with the same rays always produce the same wall.
    best_count, best_theta, best_offset = -1, 0.0, 0.0
    for degrees in np.arange(-WALL_MAX_ANGLE_DEG, WALL_MAX_ANGLE_DEG + 0.5, 1.0):
        count, offset = _densest_offset(px, py, math.radians(degrees))
        if count > best_count:
            best_count, best_theta, best_offset = count, math.radians(degrees), offset
    coarse = math.degrees(best_theta)
    for degrees in np.arange(coarse - 1.0, coarse + 1.0001, 0.25):
        if abs(degrees) > WALL_MAX_ANGLE_DEG:
            continue
        count, offset = _densest_offset(px, py, math.radians(degrees))
        if count > best_count:
            best_count, best_theta, best_offset = count, math.radians(degrees), offset
    if best_count < min_inliers:
        return None

    # Least-squares refinement (y = a + b x), then once more on its own inliers.
    residual = py * math.cos(best_theta) - px * math.sin(best_theta) - best_offset
    inlier = np.abs(residual) <= WALL_INLIER_TOLERANCE_CM
    for _ in range(2):
        if int(inlier.sum()) < min_inliers:
            return None
        ix, iy = px[inlier], py[inlier]
        if float(ix.max() - ix.min()) < min_span_cm:
            return None
        mean_x, mean_y = float(ix.mean()), float(iy.mean())
        spread_x = float(np.sum((ix - mean_x) ** 2))
        slope = float(np.sum((ix - mean_x) * (iy - mean_y))) / spread_x
        intercept = mean_y - slope * mean_x
        cosine = 1.0 / math.sqrt(1.0 + slope * slope)
        residual = (py - (intercept + slope * px)) * cosine
        inlier = np.abs(residual) <= WALL_INLIER_TOLERANCE_CM
    count = int(inlier.sum())
    if count < min_inliers:
        return None
    ix = px[inlier]
    span = float(ix.max() - ix.min())
    inlier_residual = residual[inlier]
    rms = float(math.sqrt(float(np.mean(inlier_residual ** 2))))
    position = (ix - ix.min()) / span
    middle = (position > 1.0 / 3.0) & (position < 2.0 / 3.0)
    if (not bool(np.any(middle)) or not bool(np.any(~middle))
            or abs(float(inlier_residual[middle].mean())
                   - float(inlier_residual[~middle].mean())) > WALL_MAX_BOW_CM):
        return None
    angle = math.degrees(math.atan(slope))
    distance = sign * intercept * cosine
    if (span < min_span_cm or rms > WALL_MAX_RMS_CM or abs(angle) > WALL_MAX_ANGLE_DEG
            or not min_lateral_cm <= distance <= WALL_MAX_LATERAL_CM):
        return None
    return WallFit(round(distance, 1), round(angle, 2), count, round(span, 1), round(rms, 2))


def combine_walls(left, right):
    """Return (left, right, heading). Disagreeing walls keep only the stronger."""
    if left is not None and right is not None:
        if abs(left.angle_deg - right.angle_deg) > WALLS_AGREE_DEG:
            if left.inliers >= right.inliers:
                right = None
            else:
                left = None
    if left is not None and right is not None:
        weight = left.inliers + right.inliers
        direction = (left.angle_deg * left.inliers + right.angle_deg * right.inliers) / weight
    elif left is not None:
        direction = left.angle_deg
    elif right is not None:
        direction = right.angle_deg
    else:
        return None, None, None
    return left, right, round(-direction, 2)


def far_heading(xs, ys, near_left, near_right, near_heading, min_lateral_cm):
    """Heading from the long-baseline fit, or None when it should not be used."""
    if near_heading is None:
        return None
    walls = []
    for side, near in (("left", near_left), ("right", near_right)):
        if near is None:
            continue
        far = fit_wall(xs, ys, side, min_lateral_cm, behind_cm=FAR_WINDOW_BEHIND_CM,
                       ahead_cm=FAR_WINDOW_AHEAD_CM, min_span_cm=FAR_MIN_SPAN_CM,
                       min_inliers=FAR_MIN_INLIERS)
        if far is not None and abs(far.distance_cm - near.distance_cm) <= FAR_DISTANCE_AGREE_CM:
            walls.append(far)
    if not walls:
        return None
    if len(walls) == 2 and abs(walls[0].angle_deg - walls[1].angle_deg) > FAR_AGREE_DEG:
        return None
    weight = sum(wall.inliers * wall.span_cm for wall in walls)
    heading = -sum(wall.angle_deg * wall.inliers * wall.span_cm for wall in walls) / weight
    if abs(heading - near_heading) > FAR_AGREE_DEG:
        return None
    return round(heading, 2)


def analyze_hallway(ranges, *, angle_min, angle_increment, range_min, range_max,
                    geometry=ChassisGeometry(), frame=None, far_fit=True):
    """Zones and walls for one robot-frame scan; None if numpy is unavailable.

    Invalid numeric metadata yields None; the caller's sector validator
    already turns that acquisition into STOP. ``frame`` is this same scan
    already converted (scan_frame.build_scan_frame with this geometry's
    lidar_y_cm); it only saves converting it again.
    """
    try:
        angle_min, angle_increment, range_min, range_max = (
            float(angle_min), float(angle_increment), float(range_min), float(range_max))
    except (TypeError, ValueError, OverflowError):
        return None
    if (np is None or not all(math.isfinite(value) for value in
                              (angle_min, angle_increment, range_min, range_max))
            or range_min < 0.0 or range_max <= range_min):
        return None
    if frame is not None:
        points = (frame.xs, frame.ys)
    else:
        points = _robot_points(ranges, angle_min, angle_increment, range_min, range_max,
                               geometry.lidar_y_cm)
    if points is None:
        return None
    xs, ys = points
    half = geometry.half_width_cm
    inside = (xs >= -geometry.rear_cm) & (xs <= geometry.front_cm) & (np.abs(ys) <= half)
    outside = ~inside  # a return inside the body outline can only be the robot itself
    corridor = np.abs(ys) <= geometry.corridor_half_width_cm
    # Ahead of the front edge only: a return beside the body is the side band's
    # business, and the robot does not sweep that spot by driving forward.
    front_stop = bool(np.any(corridor & (xs > geometry.front_cm)
                             & (xs <= geometry.front_stop_cm)))
    front_slow = bool(np.any(corridor & (xs > geometry.front_stop_cm)
                             & (xs <= geometry.front_slow_cm)))
    # Side band: from the spin centre to the front edge. Route segments only drive
    # forward, so what lies beside the rear half is left behind (owner, 2026-11-14:
    # the wall at the base spot held every start); a turn's own sweep still covers
    # it. Heading corrections (at most 5 degrees) swing the rear edge sideways by
    # up to about rear_cm * sin(5 deg), 2.6 cm for the default outline.
    alongside = ((xs >= max(geometry.rotation_x_cm, -geometry.rear_cm))
                 & (xs <= geometry.front_cm))
    side_left = bool(np.any(alongside & (ys > half) & (ys <= half + geometry.side_margin_cm)))
    side_right = bool(np.any(alongside & (ys < -half) & (ys >= -half - geometry.side_margin_cm)))
    sweep = outside & (np.hypot(xs - geometry.rotation_x_cm, ys) <= geometry.turn_radius_cm)
    turn_front = bool(np.any(sweep & (xs >= geometry.rotation_x_cm)))
    turn_back = bool(np.any(sweep & (xs < geometry.rotation_x_cm)))
    ahead = corridor & (xs > geometry.front_cm)
    front_gap = (round(float(xs[ahead].min() - geometry.front_cm), 1)
                 if bool(np.any(ahead)) else None)
    nearest_wall = max(WALL_MIN_LATERAL_CM, half + geometry.side_margin_cm)
    left, right, near_heading = combine_walls(fit_wall(xs, ys, "left", nearest_wall),
                                              fit_wall(xs, ys, "right", nearest_wall))
    heading, source = near_heading, "near"
    if far_fit:
        far = far_heading(xs, ys, left, right, near_heading, nearest_wall)
        if far is not None:
            heading, source = far, "far"
    return HallwayAnalysis(front_stop, front_slow, side_left, side_right, turn_front,
                           turn_back, front_gap, left, right, heading, source, near_heading)
