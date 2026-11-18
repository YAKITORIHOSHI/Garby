"""Steering geometry and nearest-clearance evidence from real scan processing."""
import raspi_path  # noqa: F401  (puts the deployable RasPi/ folder on sys.path)
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import test_runtime as fixtures


class NudgeLidarTests(unittest.TestCase):
    def setUp(self):
        self.runtime = fixtures.load_runtime()
        self.node = self.runtime.LidarDistanceReader(None, None)

    @staticmethod
    def fields(packet):
        return dict(part.split("=", 1) for part in packet.split("|")[1:]
                    if "=" in part)

    def deliver(self, *, index=None, distance=2.0):
        scan = fixtures.RuntimeTests.scan()
        if index is not None:
            scan.ranges[index] = distance
        self.node.scan_callback(scan)
        path, sides = self.node.build_status_packets()
        return path, self.fields(sides)

    def test_thin_side_object_survives_wall_median_and_nearest_history(self):
        self.deliver()
        path, fields = self.deliver(index=270, distance=0.30)
        self.assertIn("F=C|B=C", path)  # outside the existing 22 cm hard stop
        self.assertEqual(fields["L"], "200.0")
        self.assertEqual(fields["CL"], "30.0")
        self.assertEqual(fields["CR"], "200.0")
        for _ in range(2):
            _, fields = self.deliver()
            self.assertEqual(fields["CL"], "30.0")
        _, fields = self.deliver()
        self.assertEqual(fields["CL"], "200.0")

    def test_thin_right_object_has_an_independent_clearance(self):
        _, fields = self.deliver(index=90, distance=0.30)
        self.assertEqual(fields["R"], "200.0")
        self.assertEqual(fields["CR"], "30.0")
        self.assertEqual(fields["CL"], "200.0")

    @staticmethod
    def corridor_scan(left_m, right_m):
        scan = fixtures.RuntimeTests.scan()
        for index in range(len(scan.ranges)):
            sine = math.sin(scan.angle_min + index * scan.angle_increment)
            distance = ((left_m if sine > 0 else right_m) / abs(sine)
                        if abs(sine) > 1e-8 else math.inf)
            scan.ranges[index] = distance if distance <= scan.range_max else math.inf
        return scan

    def test_wide_parallel_hallway_has_realistic_clear_path_and_wall_estimates(self):
        self.node.scan_callback(self.corridor_scan(1.2, 1.8))
        path, sides = self.node.build_status_packets()
        fields = self.fields(sides)
        self.assertIn("F=C|B=C", path)
        self.assertAlmostEqual(float(fields["L"]), 120, delta=3)
        self.assertAlmostEqual(float(fields["R"]), 180, delta=3)
        self.assertGreater(float(fields["CF"]), self.runtime.THRESHOLD_CM)
        self.assertGreater(float(fields["CL"]), 40)
        self.assertGreater(float(fields["CR"]), 40)

    def status_for(self, scan):
        """Evaluate one scan with no earlier nearest-distance history."""
        node = self.runtime.LidarDistanceReader(None, None)
        node.scan_callback(scan)
        path, sides = node.build_status_packets()
        return path, self.fields(sides)

    @staticmethod
    def add_post(scan, *, forward_m, lateral_m, depth_m=0.30):
        """Add a wall post's flat face, a thin object only some rays can see."""
        for step in range(int(depth_m * 100) + 1):
            forward = forward_m - depth_m / 2 + step / 100
            index = 180 + round(math.degrees(math.atan2(lateral_m, forward)))
            index %= len(scan.ranges)
            scan.ranges[index] = min(scan.ranges[index], math.hypot(forward, lateral_m))

    def test_measured_hallway_and_post_widths_keep_moving(self):
        # Measured by the owner on 2026-11-05: 95/95 cm centered, 82/82 cm at a post.
        for left, right in ((0.95, 0.95), (0.82, 0.82),  # centered
                            (0.70, 1.20), (1.20, 0.70),  # 25 cm off-center, hallway
                            (0.60, 1.04), (1.04, 0.60)):  # 22 cm off-center, at a post
            with self.subTest(left=left, right=right):
                path, _ = self.status_for(self.corridor_scan(left, right))
                self.assertIn("F=C|B=C", path)

    def test_driving_past_a_post_stays_clear_for_the_whole_approach(self):
        for forward_cm in range(300, -101, -10):
            scan = self.corridor_scan(0.95, 0.95)
            self.add_post(scan, forward_m=forward_cm / 100, lateral_m=0.82)
            self.add_post(scan, forward_m=forward_cm / 100, lateral_m=-0.82)
            self.node.scan_callback(scan)
            path, _ = self.node.build_status_packets()
            with self.subTest(post_forward_cm=forward_cm):
                self.assertIn("F=C|B=C", path)

    def test_wall_too_close_to_a_front_diagonal_still_stops(self):
        for left, right in ((0.50, 0.50), (0.50, 1.40), (1.40, 0.50)):
            with self.subTest(left=left, right=right):
                path, fields = self.status_for(self.corridor_scan(left, right))
                self.assertIn("F=O", path)
                self.assertLessEqual(float(fields["CF"]),
                                     self.runtime.FRONT_DIAGONAL_THRESHOLD_CM)

    def test_front_diagonal_limit_is_inclusive_and_reports_nearest_clearance(self):
        limit_cm = self.runtime.FRONT_DIAGONAL_THRESHOLD_CM
        for index in (225, 135):  # FRONT_LEFT, FRONT_RIGHT
            for distance_cm, expected in ((limit_cm, "F=O"), (limit_cm + 1, "F=C")):
                with self.subTest(index=index, distance_cm=distance_cm):
                    scan = fixtures.RuntimeTests.scan()
                    scan.ranges[index] = distance_cm / 100
                    path, fields = self.status_for(scan)
                    self.assertIn(expected, path)
                    # A return left clear for PATH must still veto steering taps.
                    self.assertEqual(float(fields["CF"]), distance_cm)

    def test_straight_ahead_stop_distance_is_not_shortened(self):
        # Past the 95 cm stop, the chassis-width corridor to 160 cm is the
        # caution-speed zone (W); beyond it the path is plain clear.
        for distance_m, expected in ((0.95, "F=O"), (0.96, "F=W"), (1.60, "F=W"), (1.61, "F=C")):
            with self.subTest(distance_m=distance_m):
                scan = fixtures.RuntimeTests.scan()
                scan.ranges[180] = distance_m
                path, _ = self.status_for(scan)
                self.assertIn(expected, path)

    def test_every_return_in_the_body_lane_keeps_the_full_front_limit(self):
        # Shortening only the diagonals must not open a gap ahead of the robot:
        # anything within the side envelope laterally still stops at 95 cm.
        body_cm = self.runtime.LidarDistanceReader.SIDE_OBSTACLE_THRESHOLD_CM
        for lateral_cm in (-body_cm, -11.0, 0.0, 11.0, body_cm):
            for forward_cm in range(15, 96, 5):  # the LiDAR's minimum range is 10 cm
                distance_cm = math.hypot(forward_cm, lateral_cm)
                bearing_deg = round(math.degrees(math.atan2(lateral_cm, forward_cm)))
                if distance_cm > self.runtime.THRESHOLD_CM or abs(bearing_deg) > 67:
                    continue  # beyond the limit, or in the LEFT/RIGHT sector
                with self.subTest(forward_cm=forward_cm, lateral_cm=lateral_cm):
                    scan = fixtures.RuntimeTests.scan()
                    scan.ranges[180 + bearing_deg] = distance_cm / 100
                    path, _ = self.status_for(scan)
                    self.assertIn("F=O", path)

    def test_isolated_front_obstacle_uses_nearest_not_wall_geometry(self):
        self.deliver()
        path, fields = self.deliver(index=180, distance=0.80)
        self.assertIn("F=O", path)
        self.assertEqual(fields["F"], "200.0")
        self.assertEqual(fields["CF"], "80.0")

    def test_front_diagonal_object_participates_in_both_clearance_groups(self):
        _, fields = self.deliver(index=225, distance=0.80)
        self.assertEqual(fields["CF"], "80.0")
        self.assertEqual(fields["CL"], "80.0")
        self.assertEqual(fields["CR"], "200.0")

    def test_read_only_bypass_preserves_nearest_nudge_clearance(self):
        self.runtime.front_disabled.set()
        path, fields = self.deliver(index=180, distance=0.80)
        self.assertIn("F=C|B=C", path)
        self.assertEqual(fields["CF"], "80.0")
        self.assertEqual(fields["F"], "200.0")

    def test_missing_current_sector_cannot_borrow_old_clearance(self):
        self.deliver()
        scan = fixtures.RuntimeTests.scan()
        for index in range(293, 338):  # entire BACK_LEFT safety sector
            scan.ranges[index] = math.nan
        self.node.scan_callback(scan)
        path, sides = self.node.build_status_packets()
        fields = self.fields(sides)
        self.assertIn("F=C|B=S", path)
        self.assertNotIn("CL", fields)
        self.assertEqual(fields["CF"], "200.0")
        self.assertEqual(fields["CR"], "200.0")

    def test_missing_front_does_not_publish_previous_front_clearance(self):
        self.deliver()
        scan = fixtures.RuntimeTests.scan()
        for index in range(158, 203):
            scan.ranges[index] = math.nan
        self.node.scan_callback(scan)
        path, sides = self.node.build_status_packets()
        self.assertIn("F=S|B=C", path)
        self.assertNotIn("CF", self.fields(sides))

    def test_repeated_polls_reuse_acquisition_but_advance_path_sequence(self):
        self.node.scan_callback(fixtures.RuntimeTests.scan())
        first_path, first_sides = self.node.build_status_packets()
        for _ in range(4):
            path, sides = self.node.build_status_packets()
            self.assertNotEqual(path.split("|", 1)[0], first_path.split("|", 1)[0])
            self.assertEqual(self.fields(sides)["A"], self.fields(first_sides)["A"])
        self.node.scan_callback(fixtures.RuntimeTests.scan())
        self.assertEqual(int(self.fields(self.node.build_status_packets()[1])["A"]), 2)

    def test_rejected_source_acquisitions_never_advance_identifier(self):
        ros_ns = [100_100_000_000]
        self.node.get_clock = lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=ros_ns[0]))

        def deliver_at(stamp_ns):
            scan = fixtures.RuntimeTests.scan()
            scan.header.stamp.sec, scan.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
            self.node.scan_callback(scan)

        deliver_at(100_000_000_000)
        self.assertEqual(self.node._scan_acquisition_id, 1)
        for stamp_ns in (100_000_000_000, 99_900_000_000, 99_000_000_000, 102_000_000_000):
            with self.subTest(stamp_ns=stamp_ns):
                deliver_at(stamp_ns)
                self.assertEqual(self.node._scan_acquisition_id, 1)
                self.assertTrue(self.node.build_status_packets()[1].endswith("|STABLE"))
        ros_ns[0] = 100_300_000_000
        deliver_at(100_200_000_000)
        self.assertEqual(self.node._scan_acquisition_id, 2)

    def test_stale_source_omits_numeric_clearance_and_acquisition_evidence(self):
        with patch.object(self.runtime.time, "monotonic", return_value=100.0):
            self.deliver()
        with patch.object(self.runtime.time, "monotonic", return_value=100.81):
            path, sides = self.node.build_status_packets()
        self.assertIn("F=S|B=S", path)
        self.assertTrue(sides.endswith("|STABLE"))
        self.assertEqual(self.fields(sides), {})

    def test_uint32_wrap_and_normal_packet_fit_current_att_budget(self):
        self.node._scan_acquisition_id = 0xFFFFFFFE
        self.node._path_seq = 0xFFFFFFFE
        self.node.scan_callback(fixtures.RuntimeTests.scan([8.0] * 360))
        path, sides = self.node.build_status_packets()
        self.assertTrue(path.startswith("P:4294967295|"))
        self.assertEqual(self.fields(sides)["A"], "4294967295")
        self.assertLessEqual(len(sides.encode("ascii")), 244)
        self.node.scan_callback(fixtures.RuntimeTests.scan())
        path, sides = self.node.build_status_packets()
        self.assertTrue(path.startswith("P:0|"))
        self.assertEqual(self.fields(sides)["A"], "0")

    def test_pathological_distance_cannot_overflow_steering_frame(self):
        scan = fixtures.RuntimeTests.scan([1e40] * 360)
        scan.range_max = 1e41
        self.node.scan_callback(scan)
        path, sides = self.node.build_status_packets()
        self.assertIn("F=C|B=C", path)
        self.assertTrue(sides.endswith("|STABLE"))
        self.assertLessEqual(len(sides.encode("ascii")), 244)


if __name__ == "__main__":
    unittest.main()
