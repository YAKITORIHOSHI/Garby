"""Hallway objects found against the fitted walls, on the ray-cast hallway model.

The model is the one of test_motion_tracker.py (walls 95 cm either side,
pillars 13 cm deep, an open door, a door leaf). It is geometry with range
noise; it cannot show real clutter or the X3 on dark or shiny surfaces.
"""
import raspi_path  # noqa: F401  (puts the deployable RasPi/ folder on sys.path)
from dataclasses import replace
import unittest

from hallway_geometry import ChassisGeometry, analyze_hallway
from motion_tracker import MotionAssessment, TrackView
from scan_frame import build_scan_frame
from scene_objects import (CAUTION_CONFIRM_SCANS, SceneAssessment, SceneMonitor,
                           analyze_scene, scene_markers)
from test_motion_tracker import ANGLE_MIN, RANGE_MAX_M, STEP, Hallway

GEOMETRY = ChassisGeometry()
SCAN = dict(angle_min=ANGLE_MIN, angle_increment=STEP, range_min=0.10, range_max=RANGE_MAX_M)


def clear_world(seed=7):
    """The model hallway without its door leaf (which reaches into the path)."""
    world = Hallway(seed=seed)
    world.leaf = ((5000.0, 95.0), (5000.1, 95.0))
    return world


def scene_of(world, robot_x=0.0, robot_y=0.0, heading_deg=0.0, hallway_edit=None):
    scan = world.scan(robot_x, robot_y, heading_deg)
    frame = build_scan_frame(scan, lidar_y_cm=GEOMETRY.lidar_y_cm, **SCAN)
    hallway = analyze_hallway(scan, geometry=GEOMETRY, frame=frame, **SCAN)
    if hallway_edit is not None:
        hallway = hallway_edit(hallway)
    return analyze_scene(frame, hallway, GEOMETRY), hallway


class SceneObjectTest(unittest.TestCase):
    def test_an_empty_hallway_has_walls_fixtures_and_no_standing_object(self):
        for robot_x in (0.0, 137.0, 480.0, 811.0):
            with self.subTest(robot_x=robot_x):
                scene, hallway = scene_of(clear_world(), robot_x)
                self.assertEqual(scene.walls, 2)
                self.assertEqual(scene.standing(), ())
                self.assertIsNone(scene.caution_object(GEOMETRY))

    def test_a_box_in_the_path_is_an_object_with_its_width_and_free_sides(self):
        world = clear_world()
        world.add_person(300.0, 0.0, 0.0, 0.0, radius=20.0)
        scene, _ = scene_of(world)
        standing = scene.standing()
        self.assertEqual(len(standing), 1)
        box = standing[0]
        self.assertTrue(box.in_path)
        self.assertAlmostEqual(box.along_cm, 280.0, delta=8.0)
        self.assertAlmostEqual(box.width_cm, 38.0, delta=8.0)
        self.assertAlmostEqual(box.gap_left_cm, 75.0, delta=8.0)
        self.assertAlmostEqual(box.gap_right_cm, 75.0, delta=8.0)
        self.assertIs(scene.caution_object(GEOMETRY), box)

    def test_the_same_box_with_the_robot_off_centre_and_turned(self):
        world = clear_world()
        world.add_person(300.0, 0.0, 0.0, 0.0, radius=20.0)
        for robot_y, heading in ((12.0, 4.0), (-15.0, -5.0)):
            with self.subTest(robot_y=robot_y, heading=heading):
                scene, hallway = scene_of(world, 0.0, robot_y, heading)
                self.assertIsNotNone(hallway.heading_deg)
                standing = scene.standing()
                self.assertEqual(len(standing), 1)
                self.assertTrue(standing[0].in_path)
                self.assertAlmostEqual(standing[0].offset_cm, -robot_y, delta=8.0)

    def test_an_object_beside_the_path_is_reported_but_asks_for_nothing(self):
        world = clear_world()
        world.add_person(300.0, 62.0, 0.0, 0.0, radius=12.0)
        scene, _ = scene_of(world)
        standing = scene.standing()
        self.assertEqual(len(standing), 1)
        self.assertFalse(standing[0].in_path)
        self.assertIsNone(scene.caution_object(GEOMETRY))

    def test_a_post_on_the_wall_is_a_fixture_not_an_object(self):
        world = clear_world()
        world.add_person(260.0, 88.0, 0.0, 0.0, radius=8.0)   # an extinguisher on the wall
        scene, _ = scene_of(world)
        self.assertEqual(scene.standing(), ())
        self.assertTrue(any(obj.kind == "fixture" for obj in scene.objects))

    def test_the_end_of_the_hallway_is_not_an_object(self):
        world = clear_world()
        world.leaf = ((420.0, -95.0), (420.0, 95.0))
        scene, _ = scene_of(world)
        self.assertEqual(scene.standing(), ())
        self.assertTrue(any(obj.kind == "end" for obj in scene.objects))
        self.assertIsNone(scene.caution_object(GEOMETRY))

    def test_the_room_seen_through_an_open_door_is_not_in_the_hallway(self):
        scene, _ = scene_of(clear_world(), 500.0)   # beside the open door
        self.assertEqual(scene.standing(), ())

    def test_one_wall_still_finds_the_object(self):
        world = clear_world()
        world.add_person(300.0, 0.0, 0.0, 0.0, radius=20.0)
        scene, _ = scene_of(world, hallway_edit=lambda h: replace(h, right=None))
        box = next(obj for obj in scene.objects if obj.kind == "object" and obj.in_path)
        self.assertIsNone(box.gap_right_cm)
        self.assertAlmostEqual(box.gap_left_cm, 75.0, delta=8.0)
        # Measured, but marking and caution speed need a real two-wall hallway.
        self.assertIsNone(scene.width_cm)
        self.assertFalse(scene.in_hallway)
        self.assertEqual(scene.standing(), ())
        self.assertIsNone(scene.caution_object(GEOMETRY))
        self.assertEqual(scene_markers(scene, MotionAssessment()), ())

    def test_caution_needs_a_plausible_hallway_width(self):
        world = clear_world()
        world.add_person(300.0, 0.0, 0.0, 0.0, radius=20.0)
        scene, _ = scene_of(world)
        self.assertAlmostEqual(scene.width_cm, 190.0, delta=4.0)
        self.assertIsNotNone(scene.caution_object(GEOMETRY))
        for width in (90.0, 600.0):
            self.assertIsNone(replace(scene, width_cm=width).caution_object(GEOMETRY))
            self.assertEqual(replace(scene, width_cm=width).standing(), ())


class RealRecordingTest(unittest.TestCase):
    """Real X3 scans. Neither scene is a hallway: nothing may be marked or asked for."""

    @staticmethod
    def replay(path, reflect):
        import numpy as np
        from lidar_geometry import robot_scan_angles
        data = np.load(path)
        monitor = SceneMonitor(GEOMETRY)
        marked = caution = 0
        for k, (row, length, meta) in enumerate(zip(data["ranges"], data["lengths"], data["meta"])):
            angle_min, increment, range_min, range_max = (float(value) for value in meta[:4])
            amin, inc = robot_scan_angles(angle_min, increment, yaw_offset_deg=0.0,
                                          reflect_left_right=reflect)
            scan = dict(angle_min=amin, angle_increment=inc, range_min=range_min,
                        range_max=range_max)
            ranges = row[:int(length)].tolist()
            frame = build_scan_frame(ranges, lidar_y_cm=GEOMETRY.lidar_y_cm, **scan)
            hallway = analyze_hallway(ranges, geometry=GEOMETRY, frame=frame, **scan)
            scene = analyze_scene(frame, hallway, GEOMETRY)
            caution += monitor.update(scene, now=100.0 + 0.17 * k)
            marked += len(scene_markers(scene, MotionAssessment()))
        return len(data["ranges"]), marked, caution

    def test_the_cluttered_bench_marks_nothing_and_asks_for_nothing(self):
        from test_motion_tracker import RECORDING
        scans, marked, caution = self.replay(RECORDING, True)
        self.assertEqual(scans, 440)
        # Two of the 440 scans fit two "walls" in the clutter; none lasts three scans.
        self.assertLessEqual(marked, 4)
        self.assertEqual(caution, 0)

    def test_the_station_scans_of_2026_11_14_ask_for_nothing(self):
        from pathlib import Path
        path = (raspi_path.ROOT / "artifacts"
                / "manual-disposal-hold-2026-11-14" / "scans-base.npz")
        if not path.is_file():
            self.skipTest("station recording not in this checkout (artifacts/ is not published)")
        scans, _marked, caution = self.replay(path, True)
        self.assertEqual(scans, 15)
        self.assertEqual(caution, 0)

    def test_no_walls_means_no_scene(self):
        world = clear_world()
        world.add_person(300.0, 0.0, 0.0, 0.0, radius=20.0)
        scene, _ = scene_of(world, hallway_edit=lambda h: replace(h, left=None, right=None,
                                                                  heading_deg=None))
        self.assertEqual(scene, SceneAssessment())
        self.assertEqual(analyze_scene(None, None, GEOMETRY), SceneAssessment())

    def test_an_object_inside_the_slow_zone_is_left_to_the_zones(self):
        world = clear_world()
        world.add_person(150.0, 0.0, 0.0, 0.0, radius=20.0)   # nearest edge 130 cm: SLOW zone
        scene, _ = scene_of(world)
        self.assertEqual(len(scene.standing()), 1)
        self.assertIsNone(scene.caution_object(GEOMETRY))

    def test_an_object_beyond_the_caution_range_asks_for_nothing_yet(self):
        world = clear_world()
        world.add_person(520.0, 0.0, 0.0, 0.0, radius=20.0)
        scene, _ = scene_of(world)
        self.assertEqual(len(scene.standing()), 1)
        self.assertIsNone(scene.caution_object(GEOMETRY))


class SceneMonitorTest(unittest.TestCase):
    def test_caution_needs_consecutive_scans_and_is_held_briefly(self):
        world = clear_world()
        world.add_person(300.0, 0.0, 0.0, 0.0, radius=20.0)
        monitor = SceneMonitor(GEOMETRY)
        results = [monitor.update(scene_of(world)[0], now=10.0 + 0.17 * k) for k in range(4)]
        self.assertEqual(results, [False] * (CAUTION_CONFIRM_SCANS - 1) + [True, True])
        self.assertIn("in the path", monitor.reason)
        empty = scene_of(clear_world())[0]
        self.assertTrue(monitor.update(empty, now=11.0))    # held
        self.assertFalse(monitor.update(empty, now=12.0))   # released
        # One sighting between empty scans never asks for caution.
        self.assertFalse(monitor.update(scene_of(world)[0], now=12.2))
        self.assertFalse(monitor.update(empty, now=12.4))

    def test_reset_and_a_missing_scene_clear_it(self):
        monitor = SceneMonitor(GEOMETRY)
        self.assertFalse(monitor.update(None, now=1.0))
        monitor.reset()
        self.assertFalse(monitor.caution(now=1.0))


class MarkerTest(unittest.TestCase):
    def test_person_mover_and_object_markers_without_doubles(self):
        world = clear_world()
        world.add_person(300.0, 0.0, 0.0, 0.0, radius=20.0)
        world.add_person(520.0, 40.0, 0.0, 0.0, radius=15.0)
        scene, _ = scene_of(world)
        motion = MotionAssessment(tracks=(
            TrackView(1, 290.0, 2.0, -120.0, 0.0, True, 2.5, "person", 38.0),
            TrackView(2, 150.0, -200.0, 0.0, 60.0, True, None, "moving", 50.0),
            TrackView(3, 700.0, 60.0, 0.0, 0.0, False, None, "still", 20.0)))
        markers = scene_markers(scene, motion)
        self.assertEqual([marker[2] for marker in markers], ["moving", "person", "object"])
        person = markers[1]
        self.assertAlmostEqual(person[0], 2.9)
        self.assertAlmostEqual(person[3], -1.2)
        self.assertAlmostEqual(markers[2][0], 5.1, delta=0.15)

    def test_no_scene_and_no_tracks_give_no_markers(self):
        self.assertEqual(scene_markers(SceneAssessment(), MotionAssessment()), ())
        self.assertEqual(scene_markers(None, None), ())


if __name__ == "__main__":
    unittest.main()
