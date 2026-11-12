"""Moving-object prediction against a ray-cast model of the owner's hallway.

The model is the EAC hallway from the owner's measurements (2026-11-12): walls
95 cm either side of the centre line, pillars 13 cm deep (82 cm), an open door
to a room 345 cm away, a door leaf swung into the hallway. People are 40 cm
torsos (the LiDAR is at chest height on top of the bin). It is a geometric
model with Gaussian range noise and dropped rays; it cannot show real people,
real stopping distance or the X3's behaviour on dark clothing.
"""
import math
from pathlib import Path
import random
import unittest

import numpy as np

from hallway_geometry import ChassisGeometry
from motion_tracker import (EgoMotion, EgoMotionState, MotionTracker, STOP_TTC_S,
                            detect_objects)

SCAN_HZ = 5.75
STEP = math.radians(0.53)
COUNT = int(round(2 * math.pi / STEP))
ANGLE_MIN = -math.pi
RANGE_MAX_M = 8.0


class Hallway:
    """World frame: x along the hallway, y left. The robot drives along y = 0."""

    def __init__(self, seed=7, noise_cm=1.0, dropout=0.03):
        self.rng = random.Random(seed)
        self.noise_cm = noise_cm
        self.dropout = dropout
        self.half_width = 95.0
        self.pillars = [(x0, side) for x0 in range(-800, 3000, 400) for side in (1, -1)]
        self.door_gap = (520.0, 610.0)       # open door on the right, room wall at 345
        self.room_wall = 345.0
        self.leaf = ((900.0, 95.0), (964.0, 31.0))  # a left door leaf swung 45 degrees in
        self.people = []                    # [x, y, vx, vy, radius]

    def add_person(self, x, y, vx, vy, radius=20.0):
        self.people.append([x, y, vx, vy, radius])

    def step_people(self, dt):
        for person in self.people:
            person[0] += person[2] * dt
            person[1] += person[3] * dt

    def scan(self, robot_x, robot_y=0.0, heading_deg=0.0):
        """Robot-frame ranges (metres) for a robot at (robot_x, robot_y), heading + = left."""
        angles = ANGLE_MIN + np.arange(COUNT) * STEP + math.radians(heading_deg)
        c, s = np.cos(angles), np.sin(angles)
        best = np.full(COUNT, np.inf)
        with np.errstate(divide="ignore", invalid="ignore"):
            # Side walls, with the open door on the right.
            t_left = np.where(s > 1e-9, (self.half_width - robot_y) / s, np.inf)
            t_right = np.where(s < -1e-9, (-self.half_width - robot_y) / s, np.inf)
            x_right = robot_x + t_right * c
            in_door = (x_right >= self.door_gap[0]) & (x_right <= self.door_gap[1])
            t_room = np.where(s < -1e-9, (-self.room_wall - robot_y) / s, np.inf)
            t_right = np.where(in_door, t_room, t_right)
        best = np.minimum(best, np.minimum(t_left, t_right))
        # Pillars: 40 cm along the wall, 13 cm deep.
        for x0, side in self.pillars:
            y_lo, y_hi = (82.0, 95.0) if side > 0 else (-95.0, -82.0)
            best = np.minimum(best, _ray_box(robot_x, robot_y, c, s, x0, x0 + 40.0, y_lo, y_hi))
        # Door leaf (a thin segment).
        best = np.minimum(best, _ray_segment(robot_x, robot_y, c, s, *self.leaf))
        for px, py, _, _, radius in self.people:
            best = np.minimum(best, _ray_circle(robot_x, robot_y, c, s, px, py, radius))
        ranges = best / 100.0
        noise = np.array([self.rng.gauss(0.0, self.noise_cm / 100.0) for _ in range(COUNT)])
        ranges = ranges + noise
        drop = np.array([self.rng.random() < self.dropout for _ in range(COUNT)])
        ranges[drop | (ranges > RANGE_MAX_M) | ~np.isfinite(ranges)] = np.inf
        return ranges.tolist()


def _ray_box(ox, oy, c, s, x_lo, x_hi, y_lo, y_hi):
    with np.errstate(divide="ignore", invalid="ignore"):
        tx1, tx2 = (x_lo - ox) / c, (x_hi - ox) / c
        ty1, ty2 = (y_lo - oy) / s, (y_hi - oy) / s
    t_enter = np.maximum(np.minimum(tx1, tx2), np.minimum(ty1, ty2))
    t_exit = np.minimum(np.maximum(tx1, tx2), np.maximum(ty1, ty2))
    hit = (t_exit >= t_enter) & (t_enter > 0)
    return np.where(hit, t_enter, np.inf)


def _ray_segment(ox, oy, c, s, a, b):
    (ax, ay), (bx, by) = a, b
    ex, ey = bx - ax, by - ay
    denom = c * ey - s * ex
    with np.errstate(divide="ignore", invalid="ignore"):
        t = ((ax - ox) * ey - (ay - oy) * ex) / denom
        u = ((ax - ox) * s - (ay - oy) * c) / denom
    hit = np.isfinite(t) & (t > 0) & (u >= 0) & (u <= 1)
    return np.where(hit, t, np.inf)


def _ray_circle(ox, oy, c, s, px, py, radius):
    dx, dy = px - ox, py - oy
    along = dx * c + dy * s
    closest2 = dx * dx + dy * dy - along * along
    half = np.sqrt(np.maximum(radius * radius - closest2, 0.0))
    t = along - half
    return np.where((closest2 <= radius * radius) & (t > 0), t, np.inf)


GEOMETRY = ChassisGeometry()


def run(world, *, seconds, robot_speed, ego_mode="F", report=True, start_x=0.0,
        stop_on_stop=False):
    """Drive the robot through the world; returns per-scan (t, robot_x, assessment)."""
    tracker = MotionTracker(GEOMETRY)
    dt = 1.0 / SCAN_HZ
    robot_x = start_x
    speed = robot_speed
    history = []
    for k in range(int(seconds * SCAN_HZ)):
        t = 100.0 + k * dt
        if not report:
            ego = EgoMotion()
        elif ego_mode == "F":
            ego = EgoMotion("F" if speed else "S", speed, 0.0, t)
        else:
            ego = EgoMotion(ego_mode, 0.0, 55.0 if ego_mode == "T" else 0.0, t)
        result = tracker.update(world.scan(robot_x), angle_min=ANGLE_MIN, angle_increment=STEP,
                                range_min=0.10, range_max=RANGE_MAX_M, scan_time=t, ego=ego)
        history.append((t, robot_x, result))
        if stop_on_stop and result.stop:
            speed = 0.0
        robot_x += speed * dt
        world.step_people(dt)
    return history


def first_stop(history, world_person_x0, person_vx):
    """(time from start, person-to-robot distance) at the first STOP."""
    t0 = history[0][0]
    for t, robot_x, result in history:
        if result.stop:
            person_x = world_person_x0 + person_vx * (t - t0)
            return t - t0, person_x - robot_x
    return None


class StillWorldTest(unittest.TestCase):
    def test_empty_hallway_has_no_objects(self):
        world = Hallway()
        detections = detect_objects(world.scan(-200.0), angle_min=ANGLE_MIN, angle_increment=STEP,
                                    range_min=0.10, range_max=RANGE_MAX_M,
                                    body=(30.0, 30.0, 30.0))
        for det in detections:  # pillar faces may appear; walls never do
            self.assertLessEqual(det.extent, 90.0)
            self.assertGreaterEqual(abs(det.y), 60.0)

    def test_driving_past_pillars_doors_and_a_leaf_is_never_a_moving_object(self):
        for report in (True, False):
            with self.subTest(drive_reports=report):
                world = Hallway(noise_cm=2.0, dropout=0.05)
                history = run(world, seconds=14.0, robot_speed=71.0, report=report, start_x=-300.0)
                self.assertFalse(any(r.stop or r.slow for _, _, r in history))
                self.assertEqual(max(r.moving_count for _, _, r in history), 0)

    def test_starting_and_braking_with_late_drive_reports_never_stops_for_still_things(self):
        # The executor reports wheel speed on a change of 8 cm/s or more, at most
        # every 150 ms, and BLE adds ~100 ms: during a ramp the Pi's figure lags.
        for seed in range(4):
            with self.subTest(seed=seed):
                world = Hallway(noise_cm=2.0, dropout=0.05, seed=seed)
                tracker = MotionTracker(GEOMETRY)
                dt = 1.0 / SCAN_HZ
                robot_x, speed, reported, sent_at = -150.0, 0.0, 0.0, 49.0
                bad = known = scans = 0
                for k in range(int(16 * SCAN_HZ)):
                    t = 50.0 + k * dt
                    phase = k * dt
                    target = 71.0 if (1.0 < phase < 6.0 or 8.0 < phase < 14.0) else 0.0
                    rate = 61.0 if target > speed else 178.0   # 4800 / 14000 steps/s^2
                    speed = min(target, speed + rate * dt) if target > speed else max(target, speed - rate * dt)
                    # On a change (>= 8 cm/s, >= 150 ms apart) or every 2 s, as the executor.
                    if ((abs(speed - reported) >= 8.0 and t - sent_at >= 0.15)
                            or t - sent_at >= 2.0):
                        reported, sent_at = speed, t
                    # Stamped on arrival at the Pi, 250 ms later: often after this
                    # scan's acquisition time.
                    mode = "S" if reported < 1.0 else "F"
                    ego = EgoMotion(mode, reported, 0.0, sent_at + 0.25)
                    known += ego.fresh(t)
                    result = tracker.update(world.scan(robot_x), angle_min=ANGLE_MIN,
                                            angle_increment=STEP, range_min=0.10,
                                            range_max=RANGE_MAX_M, scan_time=t, ego=ego)
                    bad += result.stop or result.slow
                    scans += 1
                    robot_x += speed * dt
                self.assertEqual(bad, 0)
                # The report that arrived after a scan was acquired still counts.
                self.assertGreaterEqual(known, 0.95 * scans)

    def test_a_thin_object_lost_and_another_found_nearby_is_not_a_mover(self):
        # A general guard for objects dropping out and appearing nearby. (The real
        # case of this kind is replayed from the bench recording below.)
        for seed in range(6):
            with self.subTest(seed=seed):
                world = Hallway(noise_cm=1.0, dropout=0.02, seed=seed)
                leg_a = [383.0, 44.0, 0.0, 0.0, 2.5]
                tracker = MotionTracker(GEOMETRY)
                dt = 1.0 / SCAN_HZ
                bad = 0
                for k in range(int(8 * SCAN_HZ)):
                    t = 90.0 + k * dt
                    if k == 6:
                        world.people.append(leg_a)                    # A: a young track
                    if k == 10:
                        world.people.remove(leg_a)                    # A drops out
                    if k == 12:
                        world.people.append([325.0, 43.0, 0.0, 0.0, 2.5])  # B appears
                    if k == 16:
                        world.people.append(leg_a)                    # A is back
                    result = tracker.update(world.scan(300.0), angle_min=ANGLE_MIN,
                                            angle_increment=STEP, range_min=0.10,
                                            range_max=RANGE_MAX_M, scan_time=t,
                                            ego=EgoMotion("S", 0.0, 0.0, t))
                    bad += result.stop or result.slow or result.turn_stop or result.moving_count > 0
                self.assertEqual(bad, 0)

    def test_standing_still_for_a_long_time_stays_quiet(self):
        world = Hallway(noise_cm=2.0, dropout=0.08, seed=11)
        history = run(world, seconds=40.0, robot_speed=0.0, start_x=850.0)
        self.assertFalse(any(r.stop or r.slow or r.turn_stop for _, _, r in history))


class PeopleTest(unittest.TestCase):
    def test_person_walking_toward_the_robot_is_stopped_for_metres_away(self):
        world = Hallway()
        world.add_person(700.0, 0.0, -140.0, 0.0)
        history = run(world, seconds=6.0, robot_speed=71.0)
        hit = first_stop(history, 700.0, -140.0)
        self.assertIsNotNone(hit)
        assert hit is not None
        elapsed, gap = hit
        print(f"INFO walker 1.4 m/s head-on: STOP after {elapsed:.2f} s, {gap / 100:.2f} m apart")
        # The static box would act at 95 cm from the LiDAR; prediction must act far earlier.
        self.assertGreater(gap, 300.0)

    def test_walker_is_still_stopped_when_the_robots_own_speed_is_unknown(self):
        # No [DRIVE] reports (older executor, or within 2 s of a reconnect).
        world = Hallway()
        world.add_person(700.0, 0.0, -140.0, 0.0)
        history = run(world, seconds=6.0, robot_speed=71.0, report=False)
        hit = first_stop(history, 700.0, -140.0)
        self.assertIsNotNone(hit)
        assert hit is not None
        self.assertGreater(hit[1], 300.0)

    def test_runner_is_stopped_while_still_far(self):
        world = Hallway()
        world.add_person(760.0, 10.0, -450.0, 0.0)
        history = run(world, seconds=3.0, robot_speed=71.0)
        hit = first_stop(history, 760.0, -450.0)
        self.assertIsNotNone(hit)
        assert hit is not None
        elapsed, gap = hit
        print(f"INFO runner 4.5 m/s head-on: STOP after {elapsed:.2f} s, {gap / 100:.2f} m apart")
        # Stopping takes ~0.9 s (latency + braking); the runner closes at 5.2 m/s.
        self.assertGreater(gap, 5.2 * 100.0 * 0.9)

    def test_person_crossing_into_the_robots_path_stops_it(self):
        # In the path from 0.2 s to 1.3 s; the robot's nose would reach that line at 1.1 s.
        world = Hallway()
        world.add_person(150.0, 90.0, 0.0, -120.0)
        history = run(world, seconds=3.0, robot_speed=71.0)
        self.assertTrue(any(r.stop for _, _, r in history))

    def test_person_crossing_well_ahead_does_not_stop_the_robot(self):
        # Gone from the path 1.7 s before the robot gets there: a slow-down at most.
        world = Hallway()
        world.add_person(260.0, 85.0, 0.0, -120.0)
        history = run(world, seconds=3.0, robot_speed=71.0)
        self.assertFalse(any(r.stop for _, _, r in history))

    def test_person_walking_away_ahead_neither_stops_nor_slows(self):
        world = Hallway()
        world.add_person(300.0, 0.0, 130.0, 0.0)
        history = run(world, seconds=6.0, robot_speed=71.0)
        self.assertFalse(any(r.stop or r.slow for _, _, r in history))
        self.assertGreater(max(r.moving_count for _, _, r in history), 0)

    def test_person_passing_beside_the_robot_slows_it_but_does_not_stop_it(self):
        # Centre 69 cm left: body edge 19 cm from the robot's side, 6 cm from the wall.
        world = Hallway()
        world.add_person(650.0, 69.0, -130.0, 0.0)
        history = run(world, seconds=5.0, robot_speed=71.0)
        self.assertFalse(any(r.stop for _, _, r in history))
        self.assertTrue(any(r.slow for _, _, r in history))

    def test_person_walking_against_the_wall_does_not_stop_the_robot(self):
        # Touching the wall the body merges with it in the scan until close; it
        # must never read as an object on course.
        world = Hallway()
        world.add_person(650.0, 72.0, -130.0, 0.0)
        history = run(world, seconds=6.0, robot_speed=71.0)
        self.assertFalse(any(r.stop for _, _, r in history))

    def test_person_walking_toward_a_stopped_robot_keeps_it_stopped(self):
        world = Hallway()
        world.add_person(500.0, 5.0, -100.0, 0.0)
        history = run(world, seconds=3.5, robot_speed=0.0)
        self.assertTrue(any(r.stop for _, _, r in history[-6:]))

    def test_person_catching_up_from_behind_does_not_stop_the_robot(self):
        world = Hallway()
        world.add_person(-300.0, 0.0, 160.0, 0.0)
        history = run(world, seconds=3.0, robot_speed=71.0)
        self.assertFalse(any(r.stop for _, _, r in history))

    def test_stop_releases_once_the_person_has_passed(self):
        world = Hallway()
        world.add_person(500.0, 55.0, -130.0, 0.0)   # passes close beside the robot
        history = run(world, seconds=9.0, robot_speed=71.0, stop_on_stop=True)
        stops = [r.stop for _, _, r in history]
        self.assertTrue(any(stops))
        self.assertFalse(any(stops[-8:]), "STOP must clear after the person is behind the robot")

    def test_a_spin_without_drive_reports_never_holds_the_robot(self):
        # An older executor sends no [DRIVE] lines: the predictor cannot know the
        # robot is spinning, and every still object sweeps past sideways.
        for seed in range(3):
            with self.subTest(seed=seed):
                world = Hallway(noise_cm=2.0, dropout=0.05, seed=seed)
                tracker = MotionTracker(GEOMETRY)
                dt = 1.0 / SCAN_HZ
                bad = 0
                for k in range(int(12 * SCAN_HZ)):
                    t = 70.0 + k * dt
                    heading = (55.0 * k * dt) % 360.0 if (k * dt) % 4.0 < 2.0 else 0.0
                    result = tracker.update(world.scan(300.0, 0.0, heading), angle_min=ANGLE_MIN,
                                            angle_increment=STEP, range_min=0.10,
                                            range_max=RANGE_MAX_M, scan_time=t, ego=EgoMotion())
                    bad += result.stop or result.slow or result.turn_stop
                self.assertEqual(bad, 0)

    def test_spinning_resets_tracking(self):
        world = Hallway()
        world.add_person(400.0, 0.0, -140.0, 0.0)
        history = run(world, seconds=2.0, robot_speed=0.0, ego_mode="T")
        self.assertTrue(all(not r.tracks for _, _, r in history))


class GeometryTest(unittest.TestCase):
    def test_time_to_box(self):
        box = (-40.0, 70.0, -65.0, 65.0)
        self.assertAlmostEqual(MotionTracker.time_to_box(270.0, 0.0, -100.0, 0.0, box), 2.0)
        self.assertIsNone(MotionTracker.time_to_box(270.0, 80.0, -100.0, 0.0, box))
        self.assertIsNone(MotionTracker.time_to_box(270.0, 0.0, 100.0, 0.0, box))
        self.assertEqual(MotionTracker.time_to_box(0.0, 0.0, 0.0, 0.0, box), 0.0)

    def test_time_to_circle(self):
        self.assertAlmostEqual(MotionTracker.time_to_circle(150.0, 0.0, -50.0, 0.0, 0.0, 50.0), 2.0)
        self.assertIsNone(MotionTracker.time_to_circle(150.0, 80.0, -50.0, 0.0, 0.0, 50.0))
        self.assertEqual(MotionTracker.time_to_circle(10.0, 0.0, 0.0, 0.0, 0.0, 50.0), 0.0)

    def test_stop_ttc_matches_the_stopping_budget(self):
        # Detection ~0.35 s + BLE/bridge ~0.2 s + braking 0.4 s at 14000 steps/s^2.
        self.assertGreaterEqual(STOP_TTC_S, 0.35 + 0.2 + 0.4)


class EgoMotionStateTest(unittest.TestCase):
    def test_parses_drive_lines(self):
        clock = [10.0]
        state = EgoMotionState(clock=lambda: clock[0])
        self.assertTrue(state.update_from_line("[DRIVE:F:71]"))
        self.assertEqual(state.snapshot(), EgoMotion("F", 71.0, 0.0, 10.0))
        self.assertTrue(state.update_from_line("[DRIVE:T:-48]"))
        self.assertEqual(state.snapshot().mode, "T")
        self.assertTrue(state.update_from_line("[DRIVE:S]"))
        self.assertTrue(state.snapshot().fresh(12.0))
        self.assertFalse(state.snapshot().fresh(20.0))
        # A report that arrived just after the scan was acquired still applies to it.
        self.assertTrue(state.snapshot().fresh(9.7))
        self.assertFalse(state.snapshot().fresh(9.0))
        for bad in ("[DRIVE:F]", "[DRIVE:F:x]", "[DRIVE:Q:1]", "DRIVE:S", "[DRIVE:F:9999]",
                    "[DRIVE:S:1]", "[DRIVE:T:1.5]"):
            self.assertFalse(state.update_from_line(bad), bad)
        state.invalidate()
        self.assertIsNone(state.snapshot().mode)


RECORDING = (Path(__file__).resolve().parents[1] /
             "artifacts/obstacle-sweep-hold-2026-11-12/bench/scans-bench-static.npz")


@unittest.skipUnless(RECORDING.exists(), "bench recording not deployed with RasPi/")
class RealBenchRecordingTest(unittest.TestCase):
    """440 real X3 scans of the owner's cluttered desk (isolated run, 2026-11-12).

    Nothing moved on purpose. A 5 cm table leg dropped out for three scans and its
    young track was re-matched to another object 60 cm nearer: before the
    re-acquisition rule that read as a 1 m/s mover (and a turn hold).
    """

    def test_a_still_cluttered_scene_never_produces_a_mover_or_a_hold(self):
        from lidar_geometry import robot_scan_angles
        data = np.load(RECORDING)
        for ego_known in (True, False):
            with self.subTest(drive_reports=ego_known):
                tracker = MotionTracker(GEOMETRY)
                events = 0
                for row, length, meta in zip(data["ranges"], data["lengths"], data["meta"]):
                    angle_min, increment, range_min, range_max, _stamp, received = meta
                    amin, inc = robot_scan_angles(angle_min, increment, yaw_offset_deg=0.0,
                                                  reflect_left_right=True)
                    ego = EgoMotion("S", 0.0, 0.0, received) if ego_known else EgoMotion()
                    result = tracker.update(row[:int(length)].tolist(), angle_min=amin,
                                            angle_increment=inc, range_min=range_min,
                                            range_max=range_max, scan_time=received, ego=ego)
                    events += (result.stop or result.slow or result.turn_stop
                               or result.moving_count > 0)
                self.assertEqual(events, 0)


if __name__ == "__main__":
    unittest.main()
