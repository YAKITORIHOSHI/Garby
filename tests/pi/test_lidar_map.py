"""Measured scan display geometry: bounded detail without invented walls."""
import raspi_path  # noqa: F401  (puts the deployable RasPi/ folder on sys.path)
from dataclasses import FrozenInstanceError
import math
import unittest

from lidar_map import MAX_MAP_POINTS, LidarMapSnapshot, build_lidar_map_snapshot, _scan_geometry


class LidarMapTests(unittest.TestCase):
    def snapshot(self, ranges, **overrides):
        options = dict(angle_min=0.0, angle_increment=math.pi / 180,
                       range_min=0.1, range_max=8.0, acquired_at=100.0,
                       yaw_offset_deg=0.0)
        options.update(overrides)
        return build_lidar_map_snapshot(ranges, **options)

    def test_regular_degree_scan_keeps_every_bin_in_both_directions(self):
        for increment in (math.pi / 180, -math.pi / 180):
            with self.subTest(increment=increment):
                snapshot = self.snapshot([2] * 360, angle_min=-math.pi,
                                         angle_increment=increment)
                self.assertEqual(len(snapshot.points_m), 360)

    def test_complete_scan_has_360_actual_angles_and_nearest_hit_per_bin(self):
        ranges = [2.0] * 1440
        ranges[1] = 0.4
        snapshot = self.snapshot(ranges, angle_increment=math.pi / 720)
        self.assertEqual(len(snapshot.points_m), MAX_MAP_POINTS)
        near = min(snapshot.points_m, key=lambda point: math.hypot(*point))
        self.assertAlmostEqual(math.hypot(*near), .4)
        self.assertAlmostEqual(math.atan2(near[1], near[0]), math.pi / 720)

    def test_cardinal_angles_and_mount_yaw_use_forward_left_coordinates(self):
        snapshot = self.snapshot([1.0] * 4, angle_increment=math.pi / 2,
                                 yaw_offset_deg=90)
        for expected, actual in zip(((1, 0), (0, 1), (-1, 0), (0, -1)), snapshot.points_m):
            for coordinate, value in zip(expected, actual):
                self.assertAlmostEqual(coordinate, value)
        # Index zero rotates to LEFT, and has a unique shorter measurement.
        snapshot = self.snapshot([.5, 1, 1, 1], angle_increment=math.pi / 2,
                                 yaw_offset_deg=90)
        near = min(snapshot.points_m, key=lambda point: math.hypot(*point))
        self.assertAlmostEqual(near[0], 0)
        self.assertAlmostEqual(near[1], .5)
        self.assertEqual(snapshot.yaw_offset_deg, 90)

    def test_partial_clockwise_scan_keeps_gaps_and_applies_negative_yaw(self):
        snapshot = self.snapshot([1, math.inf, 2], angle_min=math.pi / 2,
                                 angle_increment=-math.pi / 4, yaw_offset_deg=-90)
        self.assertEqual(len(snapshot.points_m), 2)
        self.assertTrue(any(abs(forward - 1) < 1e-9 and abs(left) < 1e-9
                            for forward, left in snapshot.points_m))
        self.assertTrue(any(abs(forward) < 1e-9 and abs(left + 2) < 1e-9
                            for forward, left in snapshot.points_m))

    def test_only_finite_in_range_positive_measured_returns_are_plotted(self):
        snapshot = self.snapshot([math.inf, -math.inf, math.nan, 0, -.1,
                                  .05, 9, None, "invalid", .1, 8])
        self.assertEqual(len(snapshot.points_m), 2)
        self.assertEqual(sorted(round(math.hypot(*point), 6)
                                for point in snapshot.points_m), [.1, 8])
        self.assertEqual(self.snapshot([math.inf] * 360).points_m, ())

    def test_invalid_metadata_cannot_create_a_display_snapshot(self):
        for field, invalid in (("angle_min", math.nan), ("angle_increment", 0),
                               ("angle_increment", math.inf), ("range_min", -1),
                               ("range_max", math.inf), ("range_max", .05),
                               ("acquired_at", math.nan), ("yaw_offset_deg", math.inf),
                               ("range_min", None), ("angle_min", "invalid")):
            with self.subTest(field=field, invalid=invalid):
                self.assertIsNone(self.snapshot([1, 1], **{field: invalid}))
        self.assertIsNone(self.snapshot([1] * 4, angle_min=1e308, angle_increment=1e308))

    def test_geometry_cache_is_bounded_and_never_keeps_oversized_scan(self):
        _scan_geometry.cache_clear()
        bound = int(_scan_geometry.cache_info().maxsize or 0)
        self.assertGreater(bound, 0)
        for offset in range(bound + 8):
            self.snapshot([1] * 360, yaw_offset_deg=offset)
        self.assertEqual(_scan_geometry.cache_info().currsize, bound)
        before = _scan_geometry.cache_info()
        snapshot = self.snapshot([1] * 5000, angle_increment=2 * math.pi / 5000)
        self.assertEqual(len(snapshot.points_m), MAX_MAP_POINTS)
        self.assertEqual(_scan_geometry.cache_info(), before)

    def test_snapshot_is_immutable_and_keeps_source_acquisition_time(self):
        snapshot = self.snapshot([1], acquired_at=12.5)
        self.assertIsInstance(snapshot, LidarMapSnapshot)
        self.assertEqual(snapshot.acquired_at, 12.5)
        self.assertIsInstance(snapshot.points_m, tuple)
        with self.assertRaises(FrozenInstanceError):
            snapshot.acquired_at = 13


if __name__ == "__main__":
    unittest.main()
