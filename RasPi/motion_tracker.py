"""Moving-object prediction from the 2D LiDAR (2026-11-12).

The owner asked for delivery-robot behaviour in a hallway too narrow to steer
around anyone: see what is walking or running toward GARBY and stop *before* it
is close, instead of only when something is already inside the 95 cm stop box.

What it is, and what it is not
------------------------------
* The LiDAR sits on top of the bin, so its plane is around chest height. It sees
  people's torsos, doors and carts, never chairs, bins, feet or small children
  (the low front sonar covers those). It returns generic points: nothing here
  knows that a cluster is a person. "Moving object" is all it can say.
* Prediction only ADDS stop/slow evidence (like the chassis zones). Missing or
  stale scans, the sectors, the zones and every other gate are unchanged, and a
  bypassed (read-only) front group bypasses this as well.

Method (per scan, robot frame: x forward from the LiDAR, y left, centimetres)
--------------------------------------------------------------------------------
1. Segment the scan in ray order with an adaptive breakpoint (Borges & Aldon):
   neighbours belong together when their gap is under
   ``r sin(k dtheta) / sin(lambda - k dtheta) + 3 sigma``.
   Adjacent small segments within MERGE_GAP_CM merge (arm and torso, two legs).
2. Long segments (walls, pillar faces, door leaves, extent > OBJECT_MAX_EXTENT_CM)
   are structure and are not tracked. Small segments are object detections.
3. Each object is tracked with a constant-velocity Kalman filter whose velocity is
   GROUND velocity, so the robot's own motion is removed: a still object predicts
   ``p' = p - v_robot dt``. The executor reports its wheel speed ([DRIVE:...]);
   without it the robot speed is only known to lie in [0, ROBOT_MAX_SPEED_CM_S]
   and the classification below allows for all of it.
4. An object is MOVING when, in the current scan, its ground speed is over
   MOVING_MIN_CM_S and statistically certain (MOVING_SIGMAS) for MOVING_CONFIRM
   updates in a row, AND it has really travelled NET_DISPLACEMENT_MIN_CM over at
   least NET_WINDOW_MIN_S (ground frame). A track coasting without a measurement,
   or one whose segment just split or merged (its size jumped), cannot be moving:
   on the owner's cluttered bench, those were the only false movers seen.
5. For each moving object the relative path ``p(t) = p0 + (u - v_robot) t`` is
   intersected with the robot's footprint grown by margins and the object's own
   radius (slab method). Time to contact <= STOP_TTC_S: STOP. <= SLOW_TTC_S, or a
   moving object approaching within CAUTION_RANGE_CM ahead: SLOW. An object that
   would reach the footprint only from behind is not a reason to stop.
6. STOP and SLOW are held a short time after the last positive prediction so a
   flickering track cannot toggle the path packet every scan.

Stopping distance check (owner's hallway): at full speed (~71 cm/s) a person
walking toward GARBY at 1.4 m/s closes at 2.1 m/s. The static 95 cm box would
leave 0.3 s. A 2.0 s time to contact stops the robot about 4 m away, after a
~0.5 s track confirmation; a runner closing at 5 m/s is stopped as soon as the
track confirms (the X3 sees about 8 m).
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Any
import math
import threading
import time

try:  # numpy ships with ROS 2 on the Pi.
    import numpy as np
except ImportError:  # pragma: no cover - stripped install: prediction disabled
    np = None

from scan_frame import build_scan_frame


# ---------------------------------------------------------------------------
# Tunables (cm, s). Physical values are estimates until a floor run measures them.
# ---------------------------------------------------------------------------
ROBOT_MAX_SPEED_CM_S = 75.0       # 5600 steps/s on 65 mm wheels at 1600 steps/rev
BREAK_LAMBDA_DEG = 10.0           # adaptive breakpoint: worst incidence still joined
RANGE_SIGMA_CM = 1.5              # X3 noise (2 cm systematic within 1 m)
BREAK_MIN_CM = 6.0
MAX_RAY_GAP = 4                   # this many missing rays always split a segment
MIN_POINTS = 3
MERGE_GAP_CM = 25.0
OBJECT_MAX_EXTENT_CM = 90.0       # a person is ~45-60 cm across; a wall run is longer
MAX_TRACK_RANGE_CM = 780.0       # X3 range 8 m; a torso there is still 5-6 points
MEASUREMENT_SIGMA_CM = 6.0        # centroid jitter of a partly seen torso
ACCEL_SIGMA_CM_S2 = 120.0         # walking people change velocity by ~1 m/s^2
# The LiDAR sees only the near half of a round body, sampled evenly in bearing, so
# the points' centroid lies (pi/4) r toward the sensor (mean of cos over uniform
# sin). Uncorrected, a person passing beside the robot drifts toward it in the
# estimate as the bearing opens up: a false "on course" reading.
VISIBLE_ARC_BIAS = math.pi / 4.0
INITIAL_SPEED_SIGMA_CM_S = 200.0  # a new track may already be running
GATE_CHI2 = 9.21                  # 2 dof, 99 %
GATE_MAX_CM = 150.0
MAX_MISSES = 3                    # ~0.5 s at 5.75 Hz
CONFIRM_HITS = 3
MAX_SCAN_GAP_S = 0.8              # a longer gap restarts tracking (same as the stale limit)
MOVING_MIN_CM_S = 35.0
MOVING_SIGMAS = 2.0
MOVING_CONFIRM = 3
FAST_MOVING_CM_S = 150.0          # running: two significant updates are enough
FAST_MOVING_CONFIRM = 2
NET_WINDOW_MIN_S = 0.45           # ground displacement measured over at least this
FAST_NET_WINDOW_MIN_S = 0.30      # a runner covers 1.5 m in that: no clutter artefact does
NET_WINDOW_MAX_S = 1.2
NET_DISPLACEMENT_MIN_CM = 20.0
SHAPE_JUMP_CM = 25.0              # a segment this much bigger/smaller split or merged
SHAPE_JUMP_SPEED_SIGMA_CM_S = 60.0
EGO_SPEED_SIGMA_CM_S = 8.0        # reported wheel speed vs ground (no encoders)
EGO_STALE_S = 3.5                 # the executor repeats [DRIVE] every 2 s
# Scans carry their acquisition time and are processed ~0.2 s later; a report that
# arrived in between is still the best figure for that scan, not one "from the future".
EGO_LEAD_S = 0.5
# Footprint margins and decisions.
FRONT_MARGIN_CM = 20.0
SIDE_MARGIN_CM = 15.0
REAR_MARGIN_CM = 10.0
OBJECT_RADIUS_MIN_CM = 10.0
OBJECT_RADIUS_MAX_CM = 30.0
HORIZON_S = 4.0
STOP_TTC_S = 2.0
SLOW_TTC_S = 4.0
TURN_TTC_S = 2.0
CAUTION_RANGE_CM = 250.0
# Sideways motion is extrapolated only when it is real. Walking people move mostly
# along the hallway, and a centroid that slides as a body separates from a wall
# reads as a slow drift toward the robot.
LATERAL_MIN_CM_S = 25.0
LATERAL_SIGMAS = 2.0
CLOSING_MIN_CM_S = 10.0
# Without [DRIVE] reports (an older executor, or the first 2 s after a reconnect) the
# robot may be spinning, which makes every still object appear to move sideways (with
# no closing speed). Then only an object closing faster than the robot could ever close
# on a still one counts. A precaution: the other gates already held in the spin model.
UNKNOWN_EGO_CLOSING_MIN_CM_S = ROBOT_MAX_SPEED_CM_S + 25.0
STOP_HOLD_S = 0.6
SLOW_HOLD_S = 1.0
# --- The robot's own rotation (2026-11-14) ---
# A heading correction turns the whole scan: a still object 5 m away slides
# sideways at 35 cm/s while the robot yaws at 4 deg/s. The wall fit gives the
# robot's heading relative to the hallway for every scan (H), so its change from
# one scan to the next is the rotation, and the tracks are turned back by it.
# What is left unexplained is allowed for as a rotation rate: sideways motion
# (about the robot) up to range x rate is not evidence of a mover. Motion along
# the line of sight is never explained by rotation, so someone walking toward the
# robot is unaffected.
YAW_STEP_MAX_DEG = 3.0            # a bigger change between scans is a different wall fit, not a rotation
YAW_STEP_SIGMA_DEG = 0.25         # noise of one scan-to-scan heading difference
YAW_KNOWN_ALLOW_DPS = 1.5         # rotation rate left unexplained with walls in view
YAW_UNKNOWN_ALLOW_DPS = 8.0       # no walls: heading hold and centring may turn this fast
# --- Likely person (2026-11-14, owner: "mark them RED if you think they were humans") ---
# A 2D scan at chest height cannot recognise a person. "Likely person" is a
# person-sized cluster that has proven to move over the ground at a walking or
# running speed and repeatedly shows a rounded profile. A corner is non-flat
# too, so that alone is not torso evidence. It is a display/report label: every
# stop and slow rule above treats all moving objects alike.
PERSON_MIN_EXTENT_CM = 15.0
PERSON_MAX_EXTENT_CM = 75.0
PERSON_MIN_SPEED_CM_S = 25.0
PERSON_MAX_SPEED_CM_S = 450.0
PERSON_MIN_TRAVEL_CM = 40.0       # net ground travel ...
PERSON_MIN_TRAVEL_S = 0.9         # ... over at least this long
PERSON_MIN_HITS = 5
PERSON_WINDOW_MAX_S = 2.2          # the trail holds about 2 s of positions
FLAT_MIN_EXTENT_CM = 35.0         # a panel this wide ...
FLAT_MAX_RMS_CM = 1.2             # ... lying this close to one straight line is not a torso
FLAT_MAX_RANGE_CM = 400.0         # beyond this the range noise hides a torso's curve
FLAT_VETO_FRACTION = 0.6          # of the track's sightings
PERSON_PROFILE_POINTS = 8        # sparse fragments cannot establish a rounded outline
PERSON_RADIUS_MIN_CM = 8.0
PERSON_RADIUS_MAX_CM = 40.0
PERSON_PROFILE_RMS_CM = 1.6
PERSON_PROFILE_IMPROVEMENT = 0.65 # a curved fit must beat a straight line clearly
PERSON_PROFILE_CONFIRM = 3
PERSON_PROFILE_FRACTION = 0.6
# --- The still scene as the speed reference for the label (2026-11-16) ---
# Owner, 2026-11-16: "it detects objects sometimes as humans during the run, maybe
# because garby itself were running?" Ground velocity rests on the wheel speed the
# executor reports. When the wheels turn but the robot does not cover the ground at
# that speed (wheels lifted on the bench, slip, a report lagging a ramp), EVERY still
# object drifts along the travel axis by the difference, and person-sized posts and
# bins earned the label (ray-cast hallway, wheels lifted at 71 cm/s: 2-4 still
# objects labelled within about 1 s in every case tried). The other tracked objects
# show that drift. When most of them agree on one velocity along the travel axis that
# a wheel-speed error can explain, the label also needs motion relative to them.
# Display and report only: the moving, stop and slow rules above never use this.
SCENE_MIN_TRACKS = 3              # confirmed, measured tracks needed for a scene drift
SCENE_AGREE_CM_S = 15.0           # a track this close to the median velocity agrees with it
SCENE_AGREE_FRACTION = 0.6        # of those tracks
SCENE_LATERAL_MAX_CM_S = 15.0     # a wheel-speed error moves the scene along x only
SCENE_MIN_EGO_CM_S = 10.0         # only while the wheels report driving
# A wheel-speed error does not vanish between scans: the last drift stands in for a
# scan with too few measured tracks (misses, a split segment) or with tracks that
# disagree (noisy velocities), for this long, while the reported speed stays within
# this of the speed it was measured at.
SCENE_HOLD_S = 1.2
SCENE_HOLD_EGO_CM_S = 8.0


# ---------------------------------------------------------------------------
# Ego motion reported by the executor
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class EgoMotion:
    """What the wheels do. mode: 'S' stopped, 'F' straight, 'T' spinning, None unknown."""
    mode: str | None = None
    speed_cm_s: float = 0.0       # straight speed, + forward
    turn_dps: float = 0.0         # spin rate, + left
    received_at: float = 0.0

    def fresh(self, now):
        return self.mode is not None and -EGO_LEAD_S <= now - self.received_at <= EGO_STALE_S


class EgoMotionState:
    """Thread-safe latest [DRIVE:...] line from the executor."""

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._lock = threading.Lock()
        self._value = EgoMotion()

    def update_from_line(self, line):
        """Parse ``[DRIVE:S]``, ``[DRIVE:F:<cm/s>]`` or ``[DRIVE:T:<deg/s>]``."""
        text = str(line).strip()
        if not (text.startswith("[DRIVE:") and text.endswith("]")):
            return False
        fields = text[7:-1].split(":")
        mode = fields[0]
        try:
            if mode == "S" and len(fields) == 1:
                value = EgoMotion("S", 0.0, 0.0, self._clock())
            elif mode == "F" and len(fields) == 2:
                speed = float(int(fields[1]))
                if abs(speed) > 300.0:
                    return False
                value = EgoMotion("F", speed, 0.0, self._clock())
            elif mode == "T" and len(fields) == 2:
                rate = float(int(fields[1]))
                if abs(rate) > 400.0:
                    return False
                value = EgoMotion("T", 0.0, rate, self._clock())
            else:
                return False
        except ValueError:
            return False
        with self._lock:
            self._value = value
        return True

    def invalidate(self):
        with self._lock:
            self._value = EgoMotion()

    def snapshot(self):
        with self._lock:
            return self._value


# ---------------------------------------------------------------------------
# Results
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class TrackView:
    track_id: int
    x_cm: float
    y_cm: float
    vx_cm_s: float               # ground velocity, robot axes
    vy_cm_s: float
    moving: bool
    ttc_s: float | None          # time to contact with the footprint, if on course
    kind: str = "still"          # "person" (likely), "moving" or "still"
    extent_cm: float = 0.0


@dataclass(frozen=True)
class MotionAssessment:
    stop: bool = False
    slow: bool = False
    turn_stop: bool = False
    reason: str = ""
    ego: str = "unknown"
    tracks: tuple = ()

    @property
    def moving_count(self):
        return sum(1 for track in self.tracks if track.moving)

    @property
    def person_count(self):
        return sum(1 for track in self.tracks if track.kind == "person")


# ---------------------------------------------------------------------------
# Segmentation
# ---------------------------------------------------------------------------
@dataclass
class Detection:
    x: float
    y: float
    extent: float
    points: int
    near: float                  # nearest point's range
    flat: bool = False           # a wide, straight panel (door leaf, cart side, board)
    rounded: bool = False        # repeated rounded profiles are needed for the person label


@dataclass(frozen=True)
class ScanSegments:
    """One scan cut into segments, shared by the tracker and the hallway scene."""
    xs: Any
    ys: Any
    rs: Any
    groups: tuple                # index arrays into xs/ys/rs, small neighbours merged


def scan_points(ranges, angle_min, angle_increment, range_min, range_max, lidar_y_cm=0.0):
    """Valid returns in ray order: (ray index, x cm, y cm, range cm) arrays."""
    frame = build_scan_frame(ranges, angle_min=angle_min, angle_increment=angle_increment,
                             range_min=range_min, range_max=range_max, lidar_y_cm=lidar_y_cm)
    if frame is None or len(frame) == 0:
        return None
    return frame.index, frame.xs, frame.ys, frame.rs


def _number(value):
    try:
        return float(value)
    except (TypeError, ValueError, OverflowError):
        return math.nan


def segment_scan(index, xs, ys, rs, angle_increment):
    """Split the ray-ordered points into segments; returns a list of index arrays."""
    count = len(index)
    if count == 0 or np is None:
        return []
    step = abs(float(angle_increment)) or math.radians(0.5)
    lam = math.radians(BREAK_LAMBDA_DEG)
    ray_gap = np.diff(index)
    k_dtheta = np.minimum(ray_gap * step, lam * 0.9)
    threshold = rs[:-1] * np.sin(k_dtheta) / np.sin(lam - k_dtheta) + 3.0 * RANGE_SIGMA_CM
    threshold = np.maximum(threshold, BREAK_MIN_CM)
    gap = np.hypot(np.diff(xs), np.diff(ys))
    breaks = np.nonzero((gap > threshold) | (ray_gap > MAX_RAY_GAP))[0] + 1
    segments = np.split(np.arange(count), breaks)
    # The scan is a circle: the last segment may continue into the first.
    if len(segments) > 1 and len(segments[0]) and len(segments[-1]):
        first, last = segments[0][0], segments[-1][-1]
        wrap_gap = math.hypot(xs[first] - xs[last], ys[first] - ys[last])
        total_rays = int(round(2.0 * math.pi / step))
        wrap_rays = (int(index[first]) + total_rays) - int(index[last])
        wrap_threshold = max(BREAK_MIN_CM, rs[last] * math.sin(min(wrap_rays * step, lam * 0.9))
                             / math.sin(lam - min(wrap_rays * step, lam * 0.9)) + 3.0 * RANGE_SIGMA_CM)
        if 0 < wrap_rays <= MAX_RAY_GAP and wrap_gap <= wrap_threshold:
            segments[0] = np.concatenate([segments[-1], segments[0]])
            segments.pop()
    return segments


def _extent(xs, ys):
    if np is None:
        return 0.0
    return float(math.hypot(float(xs.max() - xs.min()), float(ys.max() - ys.min())))


def _line_rms(xs, ys):
    """RMS distance of the points from their own best straight line."""
    if np is None or len(xs) < 3:
        return 0.0
    dx, dy = xs - xs.mean(), ys - ys.mean()
    sxx, syy, sxy = float(np.mean(dx * dx)), float(np.mean(dy * dy)), float(np.mean(dx * dy))
    # Smaller eigenvalue of the 2x2 scatter matrix = variance across the line.
    across = 0.5 * (sxx + syy) - math.sqrt(0.25 * (sxx - syy) ** 2 + sxy * sxy)
    return math.sqrt(max(0.0, across))


def build_segments(frame, body=None):
    """Segments of one converted scan, or None for an empty/unavailable frame.

    body: (front_cm, rear_cm, half_width_cm) of the chassis; returns inside it
    are the robot itself and are ignored.
    """
    if frame is None or np is None or len(frame) == 0:
        return None
    index, xs, ys, rs = frame.index, frame.xs, frame.ys, frame.rs
    if body is not None:
        front, rear, half = body
        outside = ~((xs <= front) & (xs >= -rear) & (np.abs(ys) <= half))
        index, xs, ys, rs = index[outside], xs[outside], ys[outside], rs[outside]
    segments = [s for s in segment_scan(index, xs, ys, rs, frame.angle_increment) if len(s)]
    # Merge neighbouring small segments (an arm beside a torso, two legs).
    merged = []
    for seg in segments:
        if merged:
            prev = merged[-1]
            gap = math.hypot(xs[seg[0]] - xs[prev[-1]], ys[seg[0]] - ys[prev[-1]])
            if gap <= MERGE_GAP_CM:
                joined = np.concatenate([prev, seg])
                if (_extent(xs[prev], ys[prev]) <= OBJECT_MAX_EXTENT_CM
                        and _extent(xs[seg], ys[seg]) <= OBJECT_MAX_EXTENT_CM
                        and _extent(xs[joined], ys[joined]) <= OBJECT_MAX_EXTENT_CM):
                    merged[-1] = joined
                    continue
        merged.append(seg)
    return ScanSegments(xs, ys, rs, tuple(merged))


def _rounded_profile(xs, ys):
    """Conservative torso-sized arc evidence; corners/panels are generic objects."""
    if len(xs) < PERSON_PROFILE_POINTS:
        return False
    x, y = xs - xs.mean(), ys - ys.mean()
    try:
        fit, _, rank, _ = np.linalg.lstsq(
            np.column_stack((2.0 * x, 2.0 * y, np.ones(len(x)))), x * x + y * y, rcond=None)
    except np.linalg.LinAlgError:
        return False
    if rank < 3:
        return False
    cx, cy, offset = fit
    radius = math.sqrt(max(0.0, float(offset + cx * cx + cy * cy)))
    if not PERSON_RADIUS_MIN_CM <= radius <= PERSON_RADIUS_MAX_CM:
        return False
    rms = float(np.sqrt(np.mean((np.hypot(x - cx, y - cy) - radius) ** 2)))
    return rms <= PERSON_PROFILE_RMS_CM and rms <= PERSON_PROFILE_IMPROVEMENT * _line_rms(xs, ys)


def detect_objects(ranges=None, *, angle_min=0.0, angle_increment=0.0, range_min=0.0,
                   range_max=0.0, lidar_y_cm=0.0, body=None, segments=None):
    """Small segments (object candidates) of one robot-frame scan.

    Pass ``segments`` (build_segments of this scan) to reuse a segmentation;
    otherwise the scan is converted and segmented here.
    """
    if np is None:
        return []
    if segments is None:
        frame = build_scan_frame(ranges, angle_min=angle_min, angle_increment=angle_increment,
                                 range_min=range_min, range_max=range_max,
                                 lidar_y_cm=lidar_y_cm)
        segments = build_segments(frame, body)
    if segments is None:
        return []
    xs, ys, rs = segments.xs, segments.ys, segments.rs
    detections = []
    for seg in segments.groups:
        if len(seg) < MIN_POINTS:
            continue
        sx, sy = xs[seg], ys[seg]
        extent = _extent(sx, sy)
        if extent > OBJECT_MAX_EXTENT_CM:
            continue  # wall, pillar face, door leaf: structure, not tracked
        near = float(rs[seg].min())
        if near > MAX_TRACK_RANGE_CM:
            continue
        cx, cy = float(sx.mean()), float(sy.mean())
        distance = math.hypot(cx, cy)
        flat = (extent >= FLAT_MIN_EXTENT_CM and near <= FLAT_MAX_RANGE_CM
                and _line_rms(sx, sy) <= FLAT_MAX_RMS_CM)
        if distance > 1e-6:
            shift = VISIBLE_ARC_BIAS * min(0.5 * extent, OBJECT_RADIUS_MAX_CM) / distance
            cx, cy = cx * (1.0 + shift), cy * (1.0 + shift)
        rounded = (not flat and PERSON_MIN_EXTENT_CM <= extent <= PERSON_MAX_EXTENT_CM
                   and _rounded_profile(sx, sy))
        detections.append(Detection(cx, cy, extent, len(seg), near, flat, rounded))
    return detections


# ---------------------------------------------------------------------------
# Tracking
# ---------------------------------------------------------------------------
@dataclass
class _Track:
    track_id: int
    state: list                  # [x, y, vx, vy] (ground velocity, robot axes)
    cov: list                    # 4x4 nested lists
    extent: float
    hits: int = 1
    misses: int = 0
    moving_streak: int = 0
    born: float = 0.0
    last_seen: float = 0.0
    ttc: float | None = None
    moving: bool = False
    shape_jump: bool = False
    person: bool = False         # retained while identity/shape agrees, including a person stopping
    sightings: int = 0
    flat_sightings: int = 0
    profiles: deque = field(default_factory=lambda: deque(maxlen=8))
    flat_profiles: deque = field(default_factory=lambda: deque(maxlen=8))
    # (time, x, y) of corrected positions in the ground frame: the robot's own
    # integrated travel and rotation are taken out.
    trail: deque = field(default_factory=lambda: deque(maxlen=12))

    def radius(self):
        return min(OBJECT_RADIUS_MAX_CM, max(OBJECT_RADIUS_MIN_CM, 0.5 * self.extent))


def _mat(rows):
    if np is None:
        raise RuntimeError("numpy unavailable")
    return np.array(rows, dtype=float)


class MotionTracker:
    """Tracks small LiDAR segments and predicts contact with the robot footprint."""

    def __init__(self, geometry, clock=time.monotonic):
        self.geometry = geometry
        self._clock = clock
        self._tracks: list[_Track] = []
        self._next_id = 1
        self._last_time = None
        self._stop_until = -1.0
        self._slow_until = -1.0
        self._turn_until = -1.0
        self._last_reason = ""
        # The robot's own pose in the ground frame the trails are kept in: the
        # rotation centre's position (cm) and the heading (rad, + = left).
        self._ego_x = 0.0
        self._ego_y = 0.0
        self._ego_heading = 0.0
        self._last_wall_heading = None   # H of the previous scan, deg
        self._yaw_unknown_until = -1.0
        self._drift = None               # (scene drift cm/s, measured at, reported speed)
        self.enabled = np is not None

    # -- lifecycle ---------------------------------------------------------
    def reset(self):
        self._tracks = []
        self._last_time = None
        self._ego_x = self._ego_y = self._ego_heading = 0.0
        self._last_wall_heading = None
        self._yaw_unknown_until = -1.0
        self._drift = None

    # -- one scan ----------------------------------------------------------
    def update(self, ranges=None, *, angle_min=0.0, angle_increment=0.0, range_min=0.0,
               range_max=0.0, scan_time, ego=None, heading_deg=None, segments=None):
        """Process one robot-frame scan acquired at ``scan_time`` (monotonic s).

        heading_deg: the robot's heading relative to the hallway walls in this
        scan (hallway_geometry's H, + = rotated left), or None without walls.
        segments: build_segments() of this scan, to reuse its segmentation.
        """
        if not self.enabled:
            return MotionAssessment(reason="numpy unavailable")
        ego = ego if ego is not None else EgoMotion()
        geometry = self.geometry
        try:
            detections = detect_objects(
                ranges, angle_min=angle_min, angle_increment=angle_increment,
                range_min=range_min, range_max=range_max, lidar_y_cm=geometry.lidar_y_cm,
                body=(geometry.front_cm, geometry.rear_cm, geometry.half_width_cm),
                segments=segments)
        except (TypeError, ValueError, OverflowError):
            self.reset()
            return MotionAssessment(reason="invalid scan")

        known = ego.fresh(scan_time)
        if known and ego.mode == "T":
            # Spinning in place: everything sweeps past at the spin rate. Start
            # again from the first scan after the spin.
            self.reset()
            return self._decide(scan_time, ego, known)

        dt = None if self._last_time is None else scan_time - self._last_time
        if dt is not None and (dt <= 0.0 or dt > MAX_SCAN_GAP_S):
            self.reset()
            dt = None
        self._last_time = scan_time

        v_ego, ego_lo, ego_hi = self._ego_bounds(ego, known)
        turned, yaw_known = self._ego_rotation(ego, known, heading_deg, dt is not None)
        if not yaw_known:
            self._yaw_unknown_until = scan_time + NET_WINDOW_MAX_S
        yaw_allow = math.radians(YAW_UNKNOWN_ALLOW_DPS if scan_time <= self._yaw_unknown_until
                                 else YAW_KNOWN_ALLOW_DPS)
        if dt is not None:
            self._ego_heading += turned
            self._ego_x += v_ego * dt * math.cos(self._ego_heading)
            self._ego_y += v_ego * dt * math.sin(self._ego_heading)
            yaw_sigma = (math.radians(YAW_STEP_SIGMA_DEG) if yaw_known
                         else math.radians(YAW_UNKNOWN_ALLOW_DPS) * dt)
            for track in self._tracks:
                self._predict(track, dt, v_ego, ego_hi - ego_lo)
                self._rotate(track, turned, yaw_sigma)
        self._associate(detections, scan_time)
        drift = self._scene_drift(scan_time, ego, known, v_ego)
        for track in self._tracks:
            self._classify(track, ego_lo, ego_hi, v_ego, scan_time, yaw_allow, drift)
        return self._decide(scan_time, ego, known, v_ego)

    # -- internals -----------------------------------------------------------
    @staticmethod
    def _ego_bounds(ego, known):
        """Speed used by the filter, and the interval the true speed lies in."""
        if known and ego.mode == "F":
            v = ego.speed_cm_s
            sigma = EGO_SPEED_SIGMA_CM_S + 0.1 * abs(v)
            return v, v - sigma, v + sigma
        if known and ego.mode == "S":
            return 0.0, -EGO_SPEED_SIGMA_CM_S, EGO_SPEED_SIGMA_CM_S
        # Unknown: assume stopped in the filter, but the robot may be doing anything
        # from standing to full speed forward.
        return 0.0, -EGO_SPEED_SIGMA_CM_S, ROBOT_MAX_SPEED_CM_S + EGO_SPEED_SIGMA_CM_S

    def _ego_rotation(self, ego, known, heading_deg, continuing):
        """(rotation since the last scan in rad, whether it is known)."""
        previous = self._last_wall_heading
        try:
            heading = None if heading_deg is None else float(heading_deg)
        except (TypeError, ValueError):
            heading = None
        if heading is not None and not math.isfinite(heading):
            heading = None
        self._last_wall_heading = heading
        if known and ego.mode == "S":
            return 0.0, True  # wheels stopped: no rotation, whatever the wall noise says
        if not continuing:
            return 0.0, True  # first scan of a tracking run: nothing to turn yet
        if heading is None or previous is None:
            return 0.0, False
        step = heading - previous
        if abs(step) > YAW_STEP_MAX_DEG:
            return 0.0, False  # another wall was fitted; not a rotation
        return math.radians(step), True

    def _rotate(self, track, turned, yaw_sigma):
        """Turn a track back by the robot's own rotation about its spin centre."""
        centre = self.geometry.rotation_x_cm
        x, y, vx, vy = track.state
        px, py = x - centre, y
        if turned:
            c, s = math.cos(-turned), math.sin(-turned)
            px, py = c * px - s * py, s * px + c * py
            vx, vy = c * vx - s * vy, s * vx + c * vy
            track.state = [px + centre, py, vx, vy]
        # The rotation is known only to yaw_sigma: that much position noise across
        # the line of sight from the spin centre, and none along it.
        tx, ty = -py * yaw_sigma, px * yaw_sigma
        track.cov[0][0] += tx * tx
        track.cov[0][1] += tx * ty
        track.cov[1][0] += tx * ty
        track.cov[1][1] += ty * ty

    def _ground(self, x, y):
        """Robot-frame point to the ground frame of the trails."""
        px, py = x - self.geometry.rotation_x_cm, y
        c, s = math.cos(self._ego_heading), math.sin(self._ego_heading)
        return self._ego_x + c * px - s * py, self._ego_y + s * px + c * py

    @staticmethod
    def _predict(track, dt, v_ego, ego_spread):
        if np is None:
            return
        x, y, vx, vy = track.state
        track.state = [x + (vx - v_ego) * dt, y + vy * dt, vx, vy]
        F = _mat([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]])
        q = ACCEL_SIGMA_CM_S2 ** 2
        dt2, dt3, dt4 = dt * dt, dt ** 3, dt ** 4
        Q = q * _mat([[dt4 / 4, 0, dt3 / 2, 0], [0, dt4 / 4, 0, dt3 / 2],
                      [dt3 / 2, 0, dt2, 0], [0, dt3 / 2, 0, dt2]])
        # The robot's own speed is known only to its sigma: that much position noise.
        ego_sigma = max(EGO_SPEED_SIGMA_CM_S, 0.5 * ego_spread)
        Q[0, 0] += (ego_sigma * dt) ** 2
        P = F @ _mat(track.cov) @ F.T + Q
        track.cov = P.tolist()

    def _associate(self, detections, now):
        if np is None:
            return
        R = _mat([[MEASUREMENT_SIGMA_CM ** 2, 0], [0, MEASUREMENT_SIGMA_CM ** 2]])
        pairs = []
        for ti, track in enumerate(self._tracks):
            P = _mat(track.cov)
            S = P[:2, :2] + R
            try:
                S_inv = np.linalg.inv(S)
            except np.linalg.LinAlgError:
                continue
            for di, det in enumerate(detections):
                nu = np.array([det.x - track.state[0], det.y - track.state[1]])
                if math.hypot(nu[0], nu[1]) > GATE_MAX_CM:
                    continue
                d2 = float(nu @ S_inv @ nu)
                if d2 <= GATE_CHI2:
                    pairs.append((d2, ti, di))
        pairs.sort()
        used_tracks, used_dets = set(), set()
        for d2, ti, di in pairs:
            if ti in used_tracks or di in used_dets:
                continue
            used_tracks.add(ti)
            used_dets.add(di)
            track = self._tracks[ti]
            # Found again after missed scans: the match may be a different object
            # (bench, 2026-11-12: a 5 cm leg re-matched to something 60 cm away read
            # as 1 m/s). Like a split/merge, it earns no velocity: the trail restarts.
            reacquired = track.misses > 0
            self._correct(track, detections[di], R, now, reacquired)
            if reacquired:
                track.trail.clear()
            track.trail.append((now, *self._ground(track.state[0], track.state[1])))
        survivors = []
        for ti, track in enumerate(self._tracks):
            if ti not in used_tracks:
                track.misses += 1
                track.moving_streak = 0
            x, y = track.state[0], track.state[1]
            if track.misses <= MAX_MISSES and math.hypot(x, y) <= MAX_TRACK_RANGE_CM + 100.0:
                survivors.append(track)
        self._tracks = survivors
        for di, det in enumerate(detections):
            if di in used_dets:
                continue
            v0 = INITIAL_SPEED_SIGMA_CM_S ** 2
            m0 = MEASUREMENT_SIGMA_CM ** 2
            track = _Track(
                self._next_id, [det.x, det.y, 0.0, 0.0],
                [[m0, 0, 0, 0], [0, m0, 0, 0], [0, 0, v0, 0], [0, 0, 0, v0]],
                det.extent, born=now, last_seen=now)
            track.sightings = 1
            track.flat_sightings = 1 if det.flat else 0
            track.profiles.append(det.rounded)
            track.flat_profiles.append(det.flat)
            track.trail.append((now, *self._ground(det.x, det.y)))
            self._tracks.append(track)
            self._next_id += 1

    @staticmethod
    def _correct(track, det, R, now, reacquired=False):
        if np is None:
            return
        # A segment that split or merged since the last scan (its size jumped), or a
        # track found again after missed scans, moves its centroid without the object
        # moving: keep the position, but do not let that jump read as a velocity.
        track.shape_jump = reacquired or abs(det.extent - track.extent) > SHAPE_JUMP_CM
        if track.shape_jump:
            # A re-associated fragment is a new identity for display purposes;
            # it cannot inherit another object's person label or shape evidence.
            track.person = False
            track.profiles.clear()
            track.flat_profiles.clear()
        P = _mat(track.cov)
        if track.shape_jump:
            P[2, 2] += SHAPE_JUMP_SPEED_SIGMA_CM_S ** 2
            P[3, 3] += SHAPE_JUMP_SPEED_SIGMA_CM_S ** 2
        H = _mat([[1, 0, 0, 0], [0, 1, 0, 0]])
        S = H @ P @ H.T + R
        K = P @ H.T @ np.linalg.inv(S)
        nu = np.array([det.x - track.state[0], det.y - track.state[1]])
        state = np.array(track.state) + K @ nu
        P = (np.eye(4) - K @ H) @ P
        track.state = state.tolist()
        track.cov = (0.5 * (P + P.T)).tolist()
        track.extent = 0.7 * track.extent + 0.3 * det.extent
        track.sightings += 1
        track.flat_sightings += 1 if det.flat else 0
        track.profiles.append(det.rounded)
        track.flat_profiles.append(det.flat)
        track.hits += 1
        track.misses = 0
        track.last_seen = now

    @staticmethod
    def _band_residual(value, lo, hi):
        return 0.0 if lo <= value <= hi else (lo - value if value < lo else value - hi)

    def _scene_drift(self, now, ego, known, v_ego):
        """Ground velocity along x that the still scene shows, or None.

        Only while the wheels report driving, with SCENE_MIN_TRACKS measured,
        confirmed tracks mostly agreeing on one velocity, and only a drift that a
        wheel-speed error can explain: along the travel axis, between "the robot
        is not moving at all" and "it moves no faster than reported" (with the
        report's own sigma either side).
        """
        if not (known and ego.mode == "F" and abs(v_ego) >= SCENE_MIN_EGO_CM_S):
            self._drift = None
            return None
        held = self._drift
        if (held is not None and 0.0 <= now - held[1] <= SCENE_HOLD_S
                and abs(v_ego - held[2]) <= SCENE_HOLD_EGO_CM_S):
            held = held[0]
        else:
            held = None
        velocities = [(track.state[2], track.state[3]) for track in self._tracks
                      if track.hits >= CONFIRM_HITS and track.misses == 0
                      and track.last_seen == now and not track.shape_jump]
        if len(velocities) < SCENE_MIN_TRACKS:
            return held
        mid_x = sorted(vx for vx, _ in velocities)[len(velocities) // 2]
        mid_y = sorted(vy for _, vy in velocities)[len(velocities) // 2]
        agree = sum(1 for vx, vy in velocities
                    if math.hypot(vx - mid_x, vy - mid_y) <= SCENE_AGREE_CM_S)
        if agree < SCENE_AGREE_FRACTION * len(velocities):
            return held  # a noisy scan; the wheel-speed error has not changed
        sigma = EGO_SPEED_SIGMA_CM_S + 0.1 * abs(v_ego)
        if (abs(mid_y) > SCENE_LATERAL_MAX_CM_S
                or not min(0.0, v_ego) - sigma <= mid_x <= max(0.0, v_ego) + sigma):
            self._drift = None
            return None
        self._drift = (mid_x, now, v_ego)
        return mid_x

    def _moves_through_scene(self, track, now, drift, yaw_allow):
        """True if the track moves like a walker relative to the drifting still scene."""
        x, y, vx, vy = track.state
        lo, hi = drift - SCENE_AGREE_CM_S, drift + SCENE_AGREE_CM_S
        relative = self._beyond_rotation(x, y, self._band_residual(vx, lo, hi), vy, yaw_allow)
        return (relative >= PERSON_MIN_SPEED_CM_S
                and self._travelled(track, now, lo, hi, PERSON_MIN_TRAVEL_S, yaw_allow,
                                    window_max=PERSON_WINDOW_MAX_S) >= PERSON_MIN_TRAVEL_CM)

    def _classify(self, track, ego_lo, ego_hi, v_ego, now, yaw_allow=0.0, drift=None):
        """Moving if the ground motion is beyond every still-object explanation."""
        if track.misses > 0 or track.last_seen != now:
            # Coasting on a prediction is not evidence of anything.
            track.moving_streak = 0
            track.moving = False
            return
        if (track.person and sum(track.flat_profiles) >= PERSON_PROFILE_CONFIRM
                and sum(track.flat_profiles) >= FLAT_VETO_FRACTION * len(track.flat_profiles)):
            # Even without a scan gap, association can hand a label to a
            # similar-sized panel. Persistent contradictory shape retires the
            # display label. Stopping alone never does, and no motion gate
            # consumes either of these shape votes.
            track.person = False
        x, y, vx, vy = track.state
        # A still object's estimated ground velocity lies in [v_ego - hi, v_ego - lo].
        lo, hi = v_ego - ego_hi, v_ego - ego_lo
        residual = self._beyond_rotation(x, y, self._band_residual(vx, lo, hi), vy, yaw_allow)
        P = track.cov
        sigma = math.sqrt(max(0.0, 0.5 * (P[2][2] + P[3][3])))
        fast = residual >= FAST_MOVING_CM_S
        significant = (track.hits >= CONFIRM_HITS and not track.shape_jump
                       and residual >= MOVING_MIN_CM_S and residual >= MOVING_SIGMAS * sigma
                       and self._travelled(track, now, lo, hi,
                                           FAST_NET_WINDOW_MIN_S if fast else NET_WINDOW_MIN_S,
                                           yaw_allow) >= NET_DISPLACEMENT_MIN_CM)
        track.moving_streak = track.moving_streak + 1 if significant else 0
        needed = FAST_MOVING_CONFIRM if fast else MOVING_CONFIRM
        track.moving = track.moving_streak >= needed
        if (track.moving and not track.person and track.hits >= PERSON_MIN_HITS
                and PERSON_MIN_EXTENT_CM <= track.extent <= PERSON_MAX_EXTENT_CM
                and PERSON_MIN_SPEED_CM_S <= residual <= PERSON_MAX_SPEED_CM_S
                and track.flat_sightings < FLAT_VETO_FRACTION * max(1, track.sightings)
                and sum(track.profiles) >= PERSON_PROFILE_CONFIRM
                and sum(track.profiles) >= PERSON_PROFILE_FRACTION * len(track.profiles)
                and self._travelled(track, now, lo, hi, PERSON_MIN_TRAVEL_S, yaw_allow,
                                    window_max=PERSON_WINDOW_MAX_S) >= PERSON_MIN_TRAVEL_CM
                and (drift is None or self._moves_through_scene(track, now, drift, yaw_allow))):
            track.person = True

    def _beyond_rotation(self, x, y, dx, dy, yaw_allow_span):
        """Size of (dx, dy) at (x, y) after taking off what a rotation of the robot
        by up to yaw_allow_span (rad, or rad/s for a velocity) could explain."""
        px, py = x - self.geometry.rotation_x_cm, y
        rng = math.hypot(px, py)
        if rng < 1e-6:
            return math.hypot(dx, dy)
        ux, uy = px / rng, py / rng
        radial = dx * ux + dy * uy
        tangential = -dx * uy + dy * ux
        tangential = max(0.0, abs(tangential) - rng * yaw_allow_span)
        return math.hypot(radial, tangential)

    def _travelled(self, track, now, lo, hi, window_min, yaw_allow=0.0,
                   window_max=NET_WINDOW_MAX_S):
        """Net ground displacement (cm) over the last window_min..window_max
        seconds that neither the robot's speed error nor its rotation explains;
        0.0 when the trail is too short."""
        oldest = None
        for t, gx, gy in track.trail:
            if now - t <= window_max:
                oldest = (t, gx, gy)
                break
        if oldest is None or now - oldest[0] < window_min - 1e-6:
            return 0.0
        t0, x0, y0 = oldest
        _, x1, y1 = track.trail[-1]
        span = now - t0
        # Into the robot's present axes: its speed error lies along x.
        c, s = math.cos(-self._ego_heading), math.sin(-self._ego_heading)
        gx, gy = x1 - x0, y1 - y0
        dx, dy = c * gx - s * gy, s * gx + c * gy
        # A still object's ground x drifts by (v_ego - v_true) * span.
        dx = self._band_residual(dx, lo * span, hi * span)
        return self._beyond_rotation(track.state[0], track.state[1], dx, dy, yaw_allow * span)

    @staticmethod
    def _view(track, ttc):
        x, y, vx, vy = track.state
        kind = "person" if track.person else "moving" if track.moving else "still"
        return TrackView(track.track_id, round(x, 1), round(y, 1), round(vx, 1), round(vy, 1),
                         track.moving, None if ttc is None else round(ttc, 2), kind,
                         round(track.extent, 1))

    def _footprint(self, radius):
        g = self.geometry
        return (-(g.rear_cm + REAR_MARGIN_CM + radius), g.front_cm + FRONT_MARGIN_CM + radius,
                -(g.half_width_cm + SIDE_MARGIN_CM + radius), g.half_width_cm + SIDE_MARGIN_CM + radius)

    @staticmethod
    def time_to_box(x, y, vx, vy, box, horizon=HORIZON_S):
        """First t in [0, horizon] with (x, y) + t (vx, vy) inside box, else None."""
        x_lo, x_hi, y_lo, y_hi = box
        t_enter, t_exit = 0.0, horizon
        for p, v, lo, hi in ((x, vx, x_lo, x_hi), (y, vy, y_lo, y_hi)):
            if abs(v) < 1e-9:
                if p < lo or p > hi:
                    return None
                continue
            t1, t2 = (lo - p) / v, (hi - p) / v
            if t1 > t2:
                t1, t2 = t2, t1
            t_enter, t_exit = max(t_enter, t1), min(t_exit, t2)
            if t_enter > t_exit:
                return None
        return t_enter

    @staticmethod
    def time_to_circle(x, y, vx, vy, centre_x, radius, horizon=HORIZON_S):
        px, py = x - centre_x, y
        a = vx * vx + vy * vy
        c = px * px + py * py - radius * radius
        if c <= 0.0:
            return 0.0
        if a < 1e-9:
            return None
        b = 2.0 * (px * vx + py * vy)
        disc = b * b - 4.0 * a * c
        if disc < 0.0:
            return None
        t = (-b - math.sqrt(disc)) / (2.0 * a)
        return t if 0.0 <= t <= horizon else None

    def _decide(self, now, ego, known, v_ego=0.0):
        g = self.geometry
        stop = slow = turn = False
        reason = ""
        best = None
        views = []
        for track in self._tracks:
            x, y, vx, vy = track.state
            ttc = None
            if track.moving:
                sigma_vy = math.sqrt(max(0.0, track.cov[3][3]))
                lateral_real = abs(vy) >= max(LATERAL_MIN_CM_S, LATERAL_SIGMAS * sigma_vy)
                rvx, rvy = vx - v_ego, (vy if lateral_real else 0.0)
                box = self._footprint(track.radius())
                ttc = self.time_to_box(x, y, rvx, rvy, box)
                rng = math.hypot(x, y)
                closing = -(x * rvx + y * rvy) / rng if rng > 1e-6 else 0.0
                if ttc is not None and x < -g.rear_cm and rvx > 0.0 and v_ego >= 0.0:
                    # Catching up from behind: stopping would only make the contact
                    # harder, and forward route segments ignore what is behind them.
                    # Not while backing (reverse parking, 2026-11-16, negative
                    # [DRIVE:F]): then the robot is the one closing on it.
                    ttc = None
                elif ttc == 0.0 and closing < CLOSING_MIN_CM_S:
                    # Already within the margins but abeam or moving off: it is
                    # passing, not coming at the robot.
                    ttc = None
                if not known and closing < UNKNOWN_EGO_CLOSING_MIN_CM_S:
                    # Robot motion unknown: a spin or the robot's own speed could
                    # explain this; it is not evidence of anything coming at it.
                    track.ttc = None
                    views.append(self._view(track, None))
                    continue
                if ttc is not None and ttc <= STOP_TTC_S:
                    stop = True
                elif ttc is not None and ttc <= SLOW_TTC_S:
                    slow = True
                elif (x > -g.rear_cm and rng <= CAUTION_RANGE_CM
                      and closing >= CLOSING_MIN_CM_S):
                    slow = True
                turn_ttc = self.time_to_circle(x, y, vx, vy, g.rotation_x_cm,
                                               g.turn_radius_cm + track.radius())
                if turn_ttc == 0.0:
                    # Already inside the circle: only while it is still closing in
                    # (as for the footprint), never for something passing or leaving.
                    cx = x - g.rotation_x_cm
                    centre_range = math.hypot(cx, y)
                    centre_closing = (-(cx * vx + y * vy) / centre_range
                                      if centre_range > 1e-6 else 0.0)
                    if centre_closing < CLOSING_MIN_CM_S:
                        turn_ttc = None
                if turn_ttc is not None and turn_ttc <= TURN_TTC_S:
                    turn = True
                if ttc is not None and (best is None or ttc < best[0]):
                    best = (ttc, rng, closing)
            track.ttc = ttc
            views.append(self._view(track, ttc))
        if stop:
            self._stop_until = now + STOP_HOLD_S
        if stop or slow:
            self._slow_until = now + SLOW_HOLD_S
        if turn:
            self._turn_until = now + STOP_HOLD_S
        stop_out = now <= self._stop_until
        slow_out = not stop_out and now <= self._slow_until
        turn_out = now <= self._turn_until
        if best is not None:
            reason = (f"moving object {best[1] / 100.0:.1f} m away, closing "
                      f"{best[2] / 100.0:.1f} m/s, contact in {best[0]:.1f} s")
            self._last_reason = reason
        elif stop_out or slow_out:
            reason = self._last_reason
        if ego.mode == "T" and known:
            ego_text = "turning"
        elif known:
            ego_text = "stopped" if ego.mode == "S" else f"{ego.speed_cm_s:.0f} cm/s"
        else:
            ego_text = "unknown"
        return MotionAssessment(stop_out, slow_out, turn_out, reason, ego_text, tuple(views))
