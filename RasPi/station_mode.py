"""Station legs: the station's own walls do not hold the robot (2026-11-14).

The owner: "Don't be bothered with walls during the first and parking, garby's
station can be found at the side of hallways which was at the walls."

The robot parks against the hallway wall. Leaving, it has that wall right
behind it; parking, it drives straight at it and turns round beside it. To the
ordinary LiDAR limits that wall is an obstacle 20 cm away, and the robot never
leaves or never finishes parking (2026-11-14 live run:
artifacts/manual-disposal-hold-2026-11-14).

The executor names the route step in progress in a [LEG:..] notice and flags
the steps driven at the station (RouteResume.h, STEP_STATION). While such a
notice is fresh, returns that belong to a LONG STRAIGHT piece of the scan are
taken to be the station's walls and are treated as "nothing there" for every
LiDAR hold. So are short, truly straight pieces JOINED to such a wall and near
the robot: the wall steps, corners and frames of the station itself (the
recorded station scans have a 13 cm step and a 35 cm return beside the robot;
without them the robot could never finish parking). Everything else, a person,
a bin, a box standing free, a chair, holds the robot exactly as on any other
step. A person's outline is curved: its pieces fail the straightness test even
when the person stands against the wall.

What this is not
----------------
* It is not a bypass. Missing or stale scans, the sector limits, the chassis
  zones and the moving-object prediction all still run, on the scan with the
  wall returns taken out.
* It cannot tell the station wall from another long flat surface standing at
  the station (a door leaf, a board, the side of a cart), nor a flat-sided box
  pushed against the wall from a wall step. Those are ignored too on these few
  steps. The low front sonar still covers the way ahead, with its
  own station rule (SonarSweep.h, STATION_PARK_GAP_CM).
* The notice is telemetry. If it stops arriving (controller restart, link
  loss, the end of the step) station mode ends within FRESH_S and the robot is
  back on the stricter rules: a lost message can only make it more careful.
* GARBY_STATION_WALL_MIN_GAP_CM (default 0 = off) keeps a wall return that is
  this close ahead of the front edge as an obstacle. It needs a measured
  chassis outline (GARBY_CHASSIS_*) and a parked gap larger than the value,
  or the robot will stop short of its parking spot.
"""
from dataclasses import dataclass
import math
import re
import threading
import time

try:  # numpy ships with ROS 2 on the Pi.
    import numpy as np
except ImportError:  # pragma: no cover - stripped install
    np = None

FRESH_S = 3.5                    # the executor repeats the notice every 2 s
STEP_STATION = 0x01              # RouteResume.h
STEP_WALL_AHEAD_FWD = 0x02
STEP_WALL_AHEAD_REV = 0x04
WALL_MIN_LENGTH_CM = 60.0        # a straight piece this long is wall
WALL_MAX_RMS_CM = 2.5            # as the hallway wall fit accepts
WALL_SPLIT_CM = 5.0              # a point this far off a piece's chord is a corner
WALL_MIN_POINTS = 8
# Station structure joined to a wall: steps, corners, frames.
STEP_MIN_LENGTH_CM = 10.0
STEP_SPLIT_CM = 1.5
STEP_MAX_DEVIATION_CM = 0.8      # no return further than this from the piece's own line
STEP_MAX_RANGE_CM = 150.0        # near the robot, where the X3's noise is a few millimetres
STEP_END_TRIM = 3                # returns at each end that may be a rounded corner
CORNER_MAX_CM = 8.0              # a leftover this small between two wall pieces is their corner

_LEG = re.compile(r"\[LEG:([OR])([FB]):(\d{1,3})/(\d{1,3}):([BSHLRK]):(\d{1,3})\]")
# K (2026-11-16): backing into the parking spot at the end of a route.
_OPS = {"B": "calibration bumps", "S": "straight", "H": "long hallway",
        "L": "left turn", "R": "right turn", "K": "backing into place"}


@dataclass(frozen=True)
class RouteLeg:
    route: str = ""              # "O" outbound, "R" return; "" = no step in progress
    reverse: bool = False        # the way back of a reverse run
    step: int = 0                # 0-based
    count: int = 0
    op: str = ""
    flags: int = 0
    received_at: float = 0.0

    @property
    def station(self):
        return bool(self.route and self.flags & STEP_STATION)

    @property
    def backing(self):
        """Reverse parking in progress. Display and hold reasons only: the rear
        clearance itself goes out in every path packet (R=), notice or not."""
        return bool(self.route and self.op == "K" and not self.reverse)

    def describe(self):
        if not self.route:
            return ""
        text = (f"{'Return' if self.route == 'R' else 'Outbound'} step {self.step + 1} of "
                f"{self.count}: {_OPS.get(self.op, self.op)}")
        if self.reverse:
            text += " (way back)"
        if self.station:
            text += " · station step, station walls ignored"
        return text


class StationMode:
    """Thread-safe latest [LEG:..] notice and whether a station step is in progress."""

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._lock = threading.Lock()
        self._leg = RouteLeg()

    def update_from_line(self, line):
        """Parse a [LEG:..] notice. True when the line was one (valid or not)."""
        text = str(line).strip()
        if not text.startswith("[LEG:"):
            return False
        leg = RouteLeg()
        match = _LEG.fullmatch(text)
        if match:
            route, direction, step, count, op, flags = match.groups()
            step, count, flags = int(step), int(count), int(flags)
            if step < count <= 255 and flags <= 255:
                leg = RouteLeg(route, direction == "B", step, count, op, flags, self._clock())
        # "[LEG:0]" and anything malformed both mean: no station step.
        with self._lock:
            self._leg = leg
        return True

    def invalidate(self):
        with self._lock:
            self._leg = RouteLeg()

    def leg(self, now=None):
        """The step in progress, or an empty RouteLeg once the notice is stale."""
        now = self._clock() if now is None else now
        with self._lock:
            leg = self._leg
        if not leg.route or not 0.0 <= now - leg.received_at <= FRESH_S:
            return RouteLeg()
        return leg

    def active(self, now=None):
        return self.leg(now).station


def _chord_offsets(gx, gy):
    """(chord length, distance of every point from the chord between the ends)."""
    ax, ay, bx, by = gx[0], gy[0], gx[-1], gy[-1]
    length = math.hypot(bx - ax, by - ay)
    if length < 1e-6:
        return 0.0, None
    return length, np.abs((gx - ax) * (by - ay) - (gy - ay) * (bx - ax)) / length


def _line_deviation(gx, gy):
    """(rms, largest) distance of the points from their own best straight line."""
    dx, dy = gx - gx.mean(), gy - gy.mean()
    sxx, syy, sxy = float(np.mean(dx * dx)), float(np.mean(dy * dy)), float(np.mean(dx * dy))
    theta = 0.5 * math.atan2(2.0 * sxy, sxx - syy)   # direction of the line
    across = np.abs(-dx * math.sin(theta) + dy * math.cos(theta))
    return float(math.sqrt(float(np.mean(across * across)))), float(across.max())


def _wall_pieces(xs, ys, group, out):
    """Split one ray-ordered segment at its corners; keep the long straight pieces."""
    if len(group) < WALL_MIN_POINTS:
        return
    gx, gy = xs[group], ys[group]
    length, off = _chord_offsets(gx, gy)
    if off is None:
        return
    worst = int(np.argmax(off))
    if off[worst] > WALL_SPLIT_CM and 0 < worst < len(group) - 1:
        _wall_pieces(xs, ys, group[:worst + 1], out)
        _wall_pieces(xs, ys, group[worst:], out)
        return
    if length >= WALL_MIN_LENGTH_CM and _line_deviation(gx, gy)[0] <= WALL_MAX_RMS_CM:
        out.append(group)


def _step_pieces(xs, ys, rs, group, out):
    """Short, truly straight pieces of a run of returns joined to a wall."""
    if len(group) < WALL_MIN_POINTS:
        return
    gx, gy = xs[group], ys[group]
    length, off = _chord_offsets(gx, gy)
    if off is None:
        return
    worst = int(np.argmax(off))
    if off[worst] > STEP_SPLIT_CM and 0 < worst < len(group) - 1:
        _step_pieces(xs, ys, rs, group[:worst + 1], out)
        _step_pieces(xs, ys, rs, group[worst:], out)
        return
    if length < STEP_MIN_LENGTH_CM or float(rs[group].max()) > STEP_MAX_RANGE_CM:
        return
    # The beam rounds a corner over a few returns: the piece between its ends
    # must be straight, and then the ends belong to it.
    for trim in range(STEP_END_TRIM + 1):
        inner = group[trim:len(group) - trim]
        if len(inner) < WALL_MIN_POINTS:
            return
        ix, iy = xs[inner], ys[inner]
        if (math.hypot(ix[-1] - ix[0], iy[-1] - iy[0]) >= STEP_MIN_LENGTH_CM
                and _line_deviation(ix, iy)[1] <= STEP_MAX_DEVIATION_CM):
            out.append(group)
            return


def _runs(mask):
    """Runs of consecutive True positions of a 1-D boolean array."""
    positions = np.nonzero(mask)[0]
    if positions.size == 0:
        return []
    return np.split(positions, np.nonzero(np.diff(positions) > 1)[0] + 1)


def station_wall_points(frame, geometry, min_gap_cm=0.0):
    """Boolean array over the frame's points: True for a station wall return."""
    from motion_tracker import segment_scan  # the same segmentation the tracker uses
    if np is None or frame is None or len(frame) == 0:
        return None
    xs, ys, rs = frame.xs, frame.ys, frame.rs
    wall = np.zeros(len(frame), dtype=bool)
    for group in segment_scan(frame.index, xs, ys, rs, frame.angle_increment):
        pieces = []
        _wall_pieces(xs, ys, group, pieces)
        if not pieces:
            continue   # nothing here is joined to a wall
        for piece in pieces:
            wall[piece] = True
        # What is left of this same continuous outline: steps and corners of the
        # wall. Positions are counted along the outline: a segment that crosses
        # the scan's seam is not in index order.
        for run in _runs(~wall[group]):
            steps = []
            _step_pieces(xs, ys, rs, group[run], steps)
            for piece in steps:
                wall[piece] = True
        # A few returns left between two wall pieces are the corner they share.
        is_wall = wall[group]
        for run in _runs(~is_wall):
            first, last = group[run[0]], group[run[-1]]
            if (run[0] > 0 and run[-1] < len(group) - 1 and is_wall[run[0] - 1]
                    and is_wall[run[-1] + 1]
                    and math.hypot(xs[last] - xs[first], ys[last] - ys[first]) <= CORNER_MAX_CM):
                wall[group[run]] = True
    if min_gap_cm > 0.0:
        # A wall this close ahead of the front edge is about to be touched.
        imminent = ((np.abs(ys) <= geometry.half_width_cm) & (xs > 0.0)
                    & (xs <= geometry.front_cm + min_gap_cm))
        wall &= ~imminent
    return wall


def without_station_walls(ranges, frame, geometry, min_gap_cm=0.0):
    """(ranges, frame, wall return count) with the station's wall returns taken out.

    A taken-out ray reads +inf: "nothing within range", which the sector limits
    count as a valid clear ray, so the scan keeps its coverage.
    """
    from scan_frame import ScanFrame
    wall = station_wall_points(frame, geometry, min_gap_cm)
    if wall is None or not bool(wall.any()):
        return ranges, frame, 0
    cleared = list(ranges)
    for ray in frame.index[wall].tolist():
        cleared[ray] = math.inf
    keep = ~wall
    return (cleared, ScanFrame(frame.index[keep], frame.xs[keep], frame.ys[keep], frame.rs[keep],
                               frame.angle_increment, frame.ray_count), int(wall.sum()))
