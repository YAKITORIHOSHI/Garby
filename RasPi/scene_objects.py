"""Objects standing in the hallway, found against the fitted walls (2026-11-14).

The owner: "When garby is centered, it was already obvious that you can see the
walls - could you automatically detect if there was an object from there? or
even walking person". With a wall line on each side, every return of a scan is
one of three things:

* wall      - within WALL_BAND_CM of a fitted wall line;
* beyond    - behind a wall line (seen through a door or an opening);
* interior  - between the walls, clear of both.

Interior returns are grouped into objects. An object touching a wall and
reaching only a little into the hallway is a wall fixture (post, extinguisher,
door frame). One spanning most of the hallway is its end wall or a closed door.
Everything else is an object standing in the hallway, with its distance ahead,
its width and the free width left on each side.

What it is not
--------------
* It needs walls: no fitted wall, no objects. Station legs and turns have none.
* It is one scan with no memory, so it finds still things the moving-object
  tracker cannot. Whether something moves, or is a likely person, comes from
  motion_tracker.py; ``scene_markers`` joins the two for the map.
* It only ADDS evidence. An object in the robot's own path within
  OBJECT_CAUTION_RANGE_CM asks for caution speed (F=W), confirmed over
  CAUTION_CONFIRM_SCANS scans. STOP distances are unchanged, and a bypassed
  (read-only) front group bypasses this as it does the slow zone.
* The LiDAR plane is about chest height. Low things are the sonar's.
"""
from dataclasses import dataclass
import math
import time

try:  # numpy ships with ROS 2 on the Pi.
    import numpy as np
except ImportError:  # pragma: no cover - stripped install
    np = None


# ---------------------------------------------------------------------------
# Tunables (cm, s). Estimates until a floor run measures them.
# ---------------------------------------------------------------------------
WALL_BAND_CM = 6.0               # returns this close to a wall line are the wall
ONE_WALL_MAX_CM = 260.0          # with one wall fitted, the hallway is taken to be this wide at most
SCENE_RANGE_CM = 780.0           # as far as the tracker looks
CLUSTER_GAP_CM = 30.0            # neighbouring interior returns further apart are two objects
CLUSTER_MAX_RAY_GAP = 6          # so are returns this many rays apart
MIN_POINTS = 3
FIXTURE_GAP_CM = 25.0            # an object starting this close to a wall ...
FIXTURE_REACH_CM = 45.0          # ... and reaching no further in than this is a wall fixture
END_WALL_FRACTION = 0.7          # of the hallway width: its end wall or a closed door
END_WALL_ONE_WALL_CM = 150.0     # the same when only one wall is fitted
OBJECT_CAUTION_RANGE_CM = 400.0  # caution speed for an object in the path this near (from the LiDAR)
# Objects are marked, and caution asked for, only in a real hallway: both walls
# fitted and a width the executor's wall fusion also accepts (HeadingControl
# WIDTH_MIN/MAX_CM). On the owner's cluttered bench (2026-11-12 recording) desk
# edges fitted as single "walls" made 202 of 440 scans ask for caution, and
# marked 2888 pieces of clutter, before this rule.
HALLWAY_WIDTH_MIN_CM = 120.0
HALLWAY_WIDTH_MAX_CM = 450.0
CAUTION_CONFIRM_SCANS = 3
CAUTION_HOLD_S = 1.0
MARKER_MATCH_CM = 45.0           # a track this close to an object is the same thing
MAX_MARKERS = 16


@dataclass(frozen=True)
class SceneObject:
    x_cm: float                  # centroid, robot frame (x forward from the LiDAR, y left)
    y_cm: float
    along_cm: float              # its nearest edge along the hallway, + = ahead of the LiDAR
    offset_cm: float             # centroid across the hallway from the robot, + = left
    width_cm: float              # across the hallway
    gap_left_cm: float | None    # free width to each wall; None when that wall is not fitted
    gap_right_cm: float | None
    kind: str                    # "object", "fixture" or "end"
    in_path: bool                # overlaps the corridor the body sweeps driving on
    points: int


@dataclass(frozen=True)
class SceneAssessment:
    objects: tuple = ()
    walls: int = 0               # fitted walls this scan (0 = no scene)
    width_cm: float | None = None  # hallway width when both walls are fitted

    @property
    def in_hallway(self):
        """Both walls fitted, a plausible hallway width apart."""
        return (self.width_cm is not None
                and HALLWAY_WIDTH_MIN_CM <= self.width_cm <= HALLWAY_WIDTH_MAX_CM)

    def standing(self):
        """Objects standing in the hallway (not fixtures, not its end wall).

        Empty outside a real hallway: against one fitted line, or two an
        implausible distance apart, "between the walls" means nothing.
        """
        if not self.in_hallway:
            return ()
        return tuple(obj for obj in self.objects if obj.kind == "object")

    def caution_object(self, geometry):
        """The nearest object in the robot's path beyond the slow zone, or None."""
        best = None
        for obj in self.standing():
            if (obj.in_path
                    and geometry.front_slow_cm < obj.along_cm <= OBJECT_CAUTION_RANGE_CM
                    and (best is None or obj.along_cm < best.along_cm)):
                best = obj
        return best


def _wall_margin(xs, ys, wall, sign):
    """Distance of each point inside the hallway from one wall line (cm)."""
    theta = math.radians(wall.angle_deg)
    return wall.distance_cm - sign * (ys * math.cos(theta) - xs * math.sin(theta))


def analyze_scene(frame, hallway, geometry):
    """Objects of one converted scan against its own wall fits."""
    if (np is None or frame is None or hallway is None or len(frame) == 0
            or (hallway.left is None and hallway.right is None)):
        return SceneAssessment()
    xs, ys, index = frame.xs, frame.ys, frame.index
    left, right = hallway.left, hallway.right
    # The hallway direction in the robot frame is minus the robot's heading in it.
    theta = math.radians(-hallway.heading_deg) if hallway.heading_deg is not None else 0.0
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    along = xs * cos_t + ys * sin_t
    across = ys * cos_t - xs * sin_t

    half = geometry.half_width_cm
    body = (xs >= -geometry.rear_cm) & (xs <= geometry.front_cm) & (np.abs(ys) <= half)
    interior = ~body & (np.abs(along) <= SCENE_RANGE_CM)
    margin_left = margin_right = None
    if left is not None:
        margin_left = _wall_margin(xs, ys, left, 1.0)
        interior &= margin_left > WALL_BAND_CM
    if right is not None:
        margin_right = _wall_margin(xs, ys, right, -1.0)
        interior &= margin_right > WALL_BAND_CM
    if margin_left is None and margin_right is not None:
        interior &= margin_right <= ONE_WALL_MAX_CM
    if margin_right is None and margin_left is not None:
        interior &= margin_left <= ONE_WALL_MAX_CM
    members = np.nonzero(interior)[0]
    walls = (left is not None) + (right is not None)
    hallway_width = (round(left.distance_cm + right.distance_cm, 1)
                     if left is not None and right is not None else None)
    if members.size < MIN_POINTS:
        return SceneAssessment((), walls, hallway_width)

    # Group in ray order: a gap in rays or a jump in position starts another object.
    gap = np.hypot(np.diff(xs[members]), np.diff(ys[members]))
    breaks = np.nonzero((np.diff(index[members]) > CLUSTER_MAX_RAY_GAP)
                        | (gap > CLUSTER_GAP_CM))[0] + 1
    corridor = geometry.corridor_half_width_cm
    objects = []
    for group in np.split(members, breaks):
        if group.size < MIN_POINTS:
            continue
        g_along, g_across = along[group], across[group]
        width = float(g_across.max() - g_across.min())
        gap_left = None if margin_left is None else float(margin_left[group].min())
        gap_right = None if margin_right is None else float(margin_right[group].min())
        # How far the object reaches in from the wall it is nearest to.
        nearest_gap, reach = math.inf, math.inf
        for margins in (margin_left, margin_right):
            if margins is None:
                continue
            low = float(margins[group].min())
            if low < nearest_gap:
                nearest_gap, reach = low, float(margins[group].max())
        end_width = (END_WALL_FRACTION * hallway_width if hallway_width is not None
                     else END_WALL_ONE_WALL_CM)
        if width >= end_width:
            kind = "end"
        elif nearest_gap <= FIXTURE_GAP_CM and reach <= FIXTURE_REACH_CM:
            kind = "fixture"
        else:
            kind = "object"
        ahead = g_along > geometry.front_cm
        in_path = bool(np.any(ahead & (np.abs(g_across) <= corridor)))
        nearest_edge = float(g_along[ahead].min()) if bool(np.any(ahead)) else float(g_along.max())
        objects.append(SceneObject(
            round(float(xs[group].mean()), 1), round(float(ys[group].mean()), 1),
            round(nearest_edge, 1), round(float(g_across.mean()), 1), round(width, 1),
            None if gap_left is None else round(gap_left, 1),
            None if gap_right is None else round(gap_right, 1),
            kind, in_path, int(group.size)))
    objects.sort(key=lambda obj: math.hypot(obj.x_cm, obj.y_cm))
    return SceneAssessment(tuple(objects), walls, hallway_width)


class SceneMonitor:
    """Caution for an object in the path, confirmed over scans and held briefly."""

    def __init__(self, geometry, clock=time.monotonic):
        self.geometry = geometry
        self._clock = clock
        self._streak = 0
        self._until = -1.0
        self.scene = SceneAssessment()
        self.reason = ""

    def reset(self):
        self._streak = 0
        self._until = -1.0
        self.scene = SceneAssessment()
        self.reason = ""

    def update(self, scene, now=None):
        """Record this scan's scene; returns True while caution speed is asked for."""
        now = self._clock() if now is None else now
        self.scene = scene if scene is not None else SceneAssessment()
        obj = self.scene.caution_object(self.geometry)
        self._streak = self._streak + 1 if obj is not None else 0
        if obj is not None and self._streak >= CAUTION_CONFIRM_SCANS:
            self._until = now + CAUTION_HOLD_S
            self.reason = (f"object {obj.along_cm / 100.0:.1f} m ahead, "
                           f"{obj.width_cm:.0f} cm wide, in the path")
        return self.caution(now)

    def caution(self, now=None):
        now = self._clock() if now is None else now
        return now <= self._until


def scene_markers(scene, motion):
    """Map markers: (forward m, left m, kind, vx m/s, vy m/s), nearest first.

    kind: "person" (likely person), "moving" (moving object) or "object"
    (standing in the hallway). Fixtures, the end wall and still clutter outside
    the walls are not marked. A standing object the tracker already marks as a
    person or a mover is not marked twice.
    """
    markers = []
    tracked = []
    for track in getattr(motion, "tracks", ()) or ():
        if track.kind in ("person", "moving"):
            tracked.append((track.x_cm, track.y_cm))
            markers.append((track.x_cm / 100.0, track.y_cm / 100.0, track.kind,
                            track.vx_cm_s / 100.0, track.vy_cm_s / 100.0))
    for obj in (scene.standing() if scene is not None else ()):
        if any(math.hypot(obj.x_cm - x, obj.y_cm - y) <= MARKER_MATCH_CM for x, y in tracked):
            continue
        markers.append((obj.x_cm / 100.0, obj.y_cm / 100.0, "object", 0.0, 0.0))
    markers.sort(key=lambda marker: math.hypot(marker[0], marker[1]))
    return tuple(markers[:MAX_MARKERS])
