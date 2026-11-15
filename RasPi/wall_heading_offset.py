"""Learn the LiDAR's mounting angle from how the robot drifts (2026-11-14).

The owner: the robot is centred, but "it was running a bit tilt ... this may get
off its posture on the center".

The wall fit reports the robot's heading relative to the hallway (H). If the
LiDAR is mounted a degree or two rotated on the chassis (GARBY_LIDAR_YAW_OFFSET_DEG
was never measured), every H carries that error. The executor then holds a
heading that is not the hallway's: the robot drifts sideways until the centring
threshold pulls it back, again and again.

Physics gives the true heading without trusting H: while the robot drives, its
sideways movement per distance travelled is the sine of its TRUE heading in the
hallway, whatever the controller is doing. Over a stretch of driving:

    mean true heading = asin(change of lateral position / distance travelled)
    mounting error    = mean measured H - mean true heading

Lateral position comes from the two wall distances, distance from the executor's
reported speed ([DRIVE:F]). A speed scale error only scales the (small) true
heading term, and the mean true heading of a robot that stays in its hallway is
close to zero anyway.

Limits, by design: it learns only while driving straight with both walls
fitted, moves at most MAX_STEP_DEG per stretch, never beyond +-MAX_OFFSET_DEG,
and is cleared when the LiDAR mounting settings change. It corrects only the H
sent for centring. It is heading evidence, never path permission.
"""
import json
import logging
import math
import os
from pathlib import Path
import statistics
import tempfile
import time

logger = logging.getLogger(__name__)

WINDOW_MIN_CM = 300.0            # distance per stretch
WINDOW_MAX_S = 15.0              # a stretch slower than this starts again
MIN_SAMPLES = 14
EDGE_SAMPLES = 5                 # median lateral position at each end of the stretch
MIN_SPEED_CM_S = 20.0
MAX_SCAN_GAP_S = 0.8
MAX_HEADING_DEG = 10.0           # beyond this the robot is turning, not driving a hallway
WIDTH_AGREE_CM = 8.0             # a door, post or alcove changes the width: start again
MAX_STEP_DEG = 0.1               # per stretch
MAX_OFFSET_DEG = 3.0
SAVE_INTERVAL_S = 60.0
REPORT_STEP_DEG = 0.1


def default_path():
    configured = os.environ.get("GARBY_WALL_HEADING_OFFSET_FILE")
    if configured:
        return Path(configured).expanduser()
    return Path.home() / ".local" / "state" / "garby" / "wall-heading-offset.json"


class WallHeadingOffset:
    def __init__(self, path=None, *, mounting=(0.0, False), enabled=True,
                 clock=time.monotonic):
        self.path = Path(path) if path is not None else default_path()
        self.mounting = [float(mounting[0]), bool(mounting[1])]
        self.enabled = enabled
        self._clock = clock
        self.offset_deg = 0.0
        self.windows = 0
        self._samples = []         # (time, travel cm, lateral cm, heading deg)
        self._travel = 0.0
        self._width = None
        self._last_time = None
        self._last_saved = None
        self._saved_offset = 0.0
        self._reported = 0.0
        self._load()

    # -- persistence -----------------------------------------------------
    def _load(self):
        if not self.enabled:
            return
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
            offset = float(data["offset_deg"])
            if (data.get("mounting") != self.mounting or not math.isfinite(offset)
                    or abs(offset) > MAX_OFFSET_DEG):
                return  # another mounting, or a damaged file: learn again from zero
            self.offset_deg = self._saved_offset = self._reported = offset
            self.windows = int(data.get("windows", 0))
        except FileNotFoundError:
            return
        except (OSError, ValueError, TypeError, KeyError):
            logger.warning("Wall heading offset file %s is unreadable; learning from zero",
                           self.path)

    def save(self, force=False):
        """Write the learned value (at most every SAVE_INTERVAL_S unless forced)."""
        if not self.enabled or abs(self.offset_deg - self._saved_offset) < 0.01:
            return
        now = self._clock()
        if not force and self._last_saved is not None and now - self._last_saved < SAVE_INTERVAL_S:
            return
        self._last_saved = now
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile("w", dir=self.path.parent, delete=False,
                                             encoding="utf-8") as handle:
                json.dump({"offset_deg": round(self.offset_deg, 3), "windows": self.windows,
                           "mounting": self.mounting}, handle)
                temporary = handle.name
            os.replace(temporary, self.path)
            self._saved_offset = self.offset_deg
        except OSError as exc:
            logger.warning("Wall heading offset not saved (%s)", exc)

    # -- use ---------------------------------------------------------------
    def correct(self, heading_deg):
        """H with the learned mounting error taken out."""
        return heading_deg - self.offset_deg if self.enabled else heading_deg

    def equivalent_mount_yaw_deg(self):
        """The GARBY_LIDAR_YAW_OFFSET_DEG that would remove the learned error."""
        yaw, reflected = self.mounting
        return yaw - self.offset_deg if reflected else yaw + self.offset_deg

    # -- learning ----------------------------------------------------------
    def _restart(self):
        self._samples = []
        self._travel = 0.0
        self._width = None

    def observe(self, scan_time, left_cm, right_cm, heading_deg, ego):
        """One scan: wall distances and the UNCORRECTED H, with the executor's speed."""
        if not self.enabled:
            return
        last, self._last_time = self._last_time, scan_time
        driving = (ego is not None and ego.fresh(scan_time) and ego.mode == "F"
                   and ego.speed_cm_s >= MIN_SPEED_CM_S)
        usable = (driving and left_cm is not None and right_cm is not None
                  and heading_deg is not None and abs(heading_deg) <= MAX_HEADING_DEG)
        dt = None if last is None else scan_time - last
        if not usable or dt is None or dt <= 0.0 or dt > MAX_SCAN_GAP_S:
            self._restart()
            if not usable:
                return
        width = left_cm + right_cm
        if self._width is not None and abs(width - self._width) > WIDTH_AGREE_CM:
            self._restart()
        if self._width is None:
            self._width = width
        elif dt is not None:
            self._travel += ego.speed_cm_s * dt
        self._samples.append((scan_time, self._travel, (right_cm - left_cm) / 2.0, heading_deg))
        if scan_time - self._samples[0][0] > WINDOW_MAX_S:
            self._restart()
            return
        if self._travel >= WINDOW_MIN_CM and len(self._samples) >= MIN_SAMPLES:
            self._learn()
            self._restart()

    def _learn(self):
        first, last = self._samples[:EDGE_SAMPLES], self._samples[-EDGE_SAMPLES:]
        distance = statistics.fmean(s[1] for s in last) - statistics.fmean(s[1] for s in first)
        if distance < 0.5 * WINDOW_MIN_CM:
            return
        drift = (statistics.median(s[2] for s in last)
                 - statistics.median(s[2] for s in first)) / distance
        if abs(drift) > math.sin(math.radians(MAX_HEADING_DEG)):
            return
        true_heading = math.degrees(math.asin(drift))
        measured = statistics.fmean(s[3] for s in self._samples)
        error = measured - true_heading
        step = max(-MAX_STEP_DEG, min(MAX_STEP_DEG, error - self.offset_deg))
        self.offset_deg = max(-MAX_OFFSET_DEG, min(MAX_OFFSET_DEG, self.offset_deg + step))
        self.windows += 1
        if abs(self.offset_deg - self._reported) >= REPORT_STEP_DEG - 1e-9:
            self._reported = self.offset_deg
            logger.info(
                "Wall heading offset learned: %+.1f deg after %d stretches (this stretch "
                "measured %+.1f). Equivalent setting: GARBY_LIDAR_YAW_OFFSET_DEG=%.1f",
                self.offset_deg, self.windows, error, self.equivalent_mount_yaw_deg())
        self.save()
