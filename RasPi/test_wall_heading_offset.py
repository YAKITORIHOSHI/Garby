"""The LiDAR mounting-angle learner on a kinematic drive, and through real wall fits.

The drive is a model: lateral position integrates the sine of the true heading
at the reported speed. It cannot show wheel slip or a real X3's range bias.
"""
import json
import math
from pathlib import Path
import random
import tempfile
import unittest

import hallway_geometry as hg
from lidar_geometry import robot_scan_angles
from motion_tracker import EgoMotion
from test_hallway_geometry import HALLWAY, world_scan
import wall_heading_offset as who

SCAN_S = 1.0 / 5.75


def drive(learner, *, mount_error, metres, speed=71.0, speed_scale=1.0, seed=1, walls=True,
          moving=True, start=1000.0):
    """A robot whose controller chases the (wrong) measured heading and re-centres."""
    rng = random.Random(seed)
    lateral, t = 0.0, start
    engaged = False
    for _ in range(int(metres * 100.0 / (speed * SCAN_S))):
        # The executor holds measured H at zero, so the true heading is -error,
        # until centring (5 cm in, 2 cm out) steers back toward the centre.
        engaged = abs(lateral) > 5.0 if not engaged else abs(lateral) > 2.0
        true_heading = -mount_error + (-math.copysign(3.0, lateral) if engaged else 0.0)
        true_heading += rng.gauss(0.0, 0.3)
        measured = true_heading + mount_error + rng.gauss(0.0, 0.1)
        left = 95.0 - lateral + rng.gauss(0.0, 0.4)
        right = 95.0 + lateral + rng.gauss(0.0, 0.4)
        ego = EgoMotion("F" if moving else "S", speed * speed_scale if moving else 0.0, 0.0, t)
        learner.observe(t, left if walls else None, right, measured, ego)
        if moving:
            lateral += speed * SCAN_S * math.sin(math.radians(true_heading))
        t += SCAN_S
    return t


class LearnerTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.path = Path(self.directory.name) / "offset.json"
        self.addCleanup(self.directory.cleanup)

    def learner(self, **options):
        return who.WallHeadingOffset(self.path, **options)

    def test_a_two_degree_mounting_error_is_learned_within_one_hallway_run(self):
        for error in (2.0, -1.4):
            with self.subTest(mount_error=error):
                self.path.unlink(missing_ok=True)
                learner = self.learner()
                drive(learner, mount_error=error, metres=76.0)
                self.assertAlmostEqual(learner.offset_deg, error, delta=0.35)
                self.assertAlmostEqual(learner.correct(error), 0.0, delta=0.35)

    def test_a_straight_lidar_learns_nothing_to_speak_of(self):
        learner = self.learner()
        drive(learner, mount_error=0.0, metres=150.0)
        self.assertLess(abs(learner.offset_deg), 0.15)

    def test_it_moves_slowly_and_never_beyond_its_bound(self):
        learner = self.learner()
        drive(learner, mount_error=2.0, metres=3.6)   # one stretch
        self.assertLessEqual(abs(learner.offset_deg), who.MAX_STEP_DEG + 1e-9)
        self.assertEqual(learner.windows, 1)
        drive(learner, mount_error=8.0, metres=400.0, start=5000.0)
        self.assertLessEqual(abs(learner.offset_deg), who.MAX_OFFSET_DEG)

    def test_a_wrong_speed_scale_still_converges(self):
        learner = self.learner()
        drive(learner, mount_error=2.0, metres=120.0, speed_scale=0.85)
        self.assertAlmostEqual(learner.offset_deg, 2.0, delta=0.45)

    def test_nothing_is_learned_without_both_walls_or_while_stopped(self):
        for options in (dict(walls=False), dict(moving=False)):
            with self.subTest(**options):
                learner = self.learner()
                drive(learner, mount_error=2.0, metres=40.0, **options)
                self.assertEqual((learner.offset_deg, learner.windows), (0.0, 0))

    def test_a_scan_gap_a_width_change_or_stale_speed_starts_the_stretch_again(self):
        learner = self.learner()
        t = 100.0
        for k in range(60):
            # A new hallway width every 8 scans: never 3 m of one hallway.
            width = 190.0 + (20.0 if (k // 8) % 2 else 0.0)
            learner.observe(t, width / 2, width / 2, 2.0, EgoMotion("F", 71.0, 0.0, t))
            t += SCAN_S
        self.assertEqual(learner.windows, 0)
        for k in range(60):
            learner.observe(t, 95.0, 95.0, 2.0, EgoMotion("F", 71.0, 0.0, t))
            t += 1.0   # scans a second apart
        self.assertEqual(learner.windows, 0)
        for k in range(60):
            learner.observe(t, 95.0, 95.0, 2.0, EgoMotion("F", 71.0, 0.0, t - 30.0))
            t += SCAN_S
        self.assertEqual(learner.windows, 0)

    def test_the_value_is_saved_and_restored_for_the_same_mounting_only(self):
        learner = self.learner(mounting=(0.0, True))
        drive(learner, mount_error=2.0, metres=76.0)
        learner.save(force=True)
        saved = json.loads(self.path.read_text())
        self.assertAlmostEqual(saved["offset_deg"], learner.offset_deg, places=2)
        self.assertAlmostEqual(self.learner(mounting=(0.0, True)).offset_deg,
                               learner.offset_deg, places=2)
        self.assertEqual(self.learner(mounting=(1.5, True)).offset_deg, 0.0)
        self.assertEqual(self.learner(mounting=(0.0, False)).offset_deg, 0.0)

    def test_a_damaged_or_out_of_range_file_is_ignored(self):
        for content in ("not json", json.dumps({"offset_deg": 40.0, "mounting": [0.0, False]}),
                        json.dumps({"offset_deg": "x", "mounting": [0.0, False]})):
            self.path.write_text(content)
            self.assertEqual(self.learner().offset_deg, 0.0)

    def test_disabled_learns_and_corrects_nothing(self):
        learner = self.learner(enabled=False)
        drive(learner, mount_error=2.0, metres=76.0)
        self.assertEqual(learner.offset_deg, 0.0)
        self.assertEqual(learner.correct(1.5), 1.5)
        self.assertFalse(self.path.exists())


class MountingSignTests(unittest.TestCase):
    """The suggested GARBY_LIDAR_YAW_OFFSET_DEG really removes the measured error."""

    def measured_heading(self, mount_yaw_deg, configured_yaw_deg, reflected):
        # A LiDAR rotated mount_yaw_deg on a robot standing straight in the hallway.
        scan = world_scan(HALLWAY, heading_deg=mount_yaw_deg, noise_cm=0.3)
        raw_min, raw_inc = scan["angle_min"], scan["angle_increment"]
        if reflected:   # the driver's angles run the other way round on this mount
            raw_min, raw_inc = -raw_min, -raw_inc
        angle_min, increment = robot_scan_angles(
            raw_min, raw_inc, yaw_offset_deg=configured_yaw_deg, reflect_left_right=reflected)
        return hg.analyze_hallway(scan["ranges"], angle_min=angle_min, angle_increment=increment,
                                  range_min=0.1, range_max=8.0).heading_deg

    def test_the_equivalent_setting_zeroes_the_heading_for_both_orientations(self):
        with tempfile.TemporaryDirectory() as directory:
            for reflected in (False, True):
                with self.subTest(reflected=reflected):
                    error = self.measured_heading(2.0, 0.0, reflected)
                    self.assertAlmostEqual(abs(error), 2.0, delta=0.2)
                    learner = who.WallHeadingOffset(Path(directory) / f"{reflected}.json",
                                                    mounting=(0.0, reflected))
                    learner.offset_deg = error
                    fixed = self.measured_heading(2.0, learner.equivalent_mount_yaw_deg(),
                                                  reflected)
                    self.assertAlmostEqual(fixed, 0.0, delta=0.2)


if __name__ == "__main__":
    unittest.main()
