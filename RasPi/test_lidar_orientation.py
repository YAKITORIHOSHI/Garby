"""Reported mount handedness through the real scan producer, without hardware."""
import math
import queue
import unittest
from unittest.mock import patch

import test_runtime as fixtures


class LidarOrientationTests(unittest.TestCase):
    @staticmethod
    def runtime(*, yaw=0, reflect=None):
        options = {"GARBY_LIDAR_YAW_OFFSET_DEG": str(yaw)}
        # Isolate the deployment defaults from the host's own environment.
        with patch.dict("os.environ", options):
            if reflect is None:
                with patch.dict("os.environ") as environment:
                    environment.pop("GARBY_LIDAR_REFLECT_LEFT_RIGHT", None)
                    return fixtures.load_runtime(lidar_reflection=None)
            with patch.dict("os.environ", {"GARBY_LIDAR_REFLECT_LEFT_RIGHT": reflect}):
                return fixtures.load_runtime(lidar_reflection=None)

    @staticmethod
    def scan(yaw):
        scan = fixtures.RuntimeTests.scan([3.0] * 360)
        # The reported mount presents physical LEFT at the old -90-degree
        # robot bearing, and physical RIGHT at +90. Keep forward/back fixed.
        for old_bearing, distance in ((-90, 1.2), (90, 1.8)):
            for delta in range(-11, 12):
                scan.ranges[(180 + old_bearing - yaw + delta) % 360] = distance
        for old_bearing, distance in ((-90, .30), (90, .50), (0, .80), (180, .25)):
            scan.ranges[(180 + old_bearing - yaw) % 360] = distance
        return scan

    def produce(self, runtime, scan):
        frames = queue.Queue()
        node = runtime.LidarDistanceReader(None, frames)
        with patch.object(runtime.time, "monotonic", return_value=100.0):
            node.scan_callback(scan)
            path, packet = node.build_status_packets()
        fields = dict(part.split("=", 1) for part in packet.split("|")[1:] if "=" in part)
        return node, frames.get_nowait(), path, fields

    def assert_point(self, snapshot, *, distance, forward, left):
        point, = [point for point in snapshot.points_m
                  if abs(math.hypot(*point) - distance) < 1e-8]
        self.assertAlmostEqual(point[0], forward)
        self.assertAlmostEqual(point[1], left)

    def test_reported_mount_default_maps_summaries_walls_and_nearest_clearance(self):
        for yaw in (0, 30, -90, 180):
            with self.subTest(yaw=yaw):
                runtime = self.runtime(yaw=yaw)
                node, frame, path, fields = self.produce(runtime, self.scan(yaw))
                self.assertEqual(fields["CL"], "30.0")
                self.assertEqual(fields["CR"], "50.0")
                self.assertEqual(fields["L"], "120.0")
                self.assertEqual(fields["R"], "180.0")
                self.assertEqual(frame[0]["LEFT"][0], 1.2)
                self.assertEqual(frame[0]["RIGHT"][0], 1.8)
                self.assertEqual(fields["CF"], "80.0")
                self.assertIn("F=O|B=O", path)
                self.assertEqual(node.history["FRONT"][-1], 80.0)
                self.assertEqual(node.history["BACK"][-1], 25.0)
                snapshot = frame[5]
                for distance, forward, left in ((.3, 0, .3), (.5, 0, -.5),
                                                (.8, .8, 0), (.25, -.25, 0)):
                    self.assert_point(snapshot, distance=distance, forward=forward, left=left)
                self.assertEqual(snapshot.acquired_at, node._last_scan_time)
                self.assertEqual(frame[6], snapshot.acquired_at)

    def test_alternate_mount_can_disable_reflection_consistently(self):
        runtime = self.runtime(yaw=30, reflect="0")
        _, frame, _, fields = self.produce(runtime, self.scan(30))
        self.assertEqual(fields["CL"], "50.0")
        self.assertEqual(fields["CR"], "30.0")
        self.assertEqual(frame[0]["LEFT"][0], 1.8)
        self.assertEqual(frame[0]["RIGHT"][0], 1.2)
        self.assert_point(frame[5], distance=.3, forward=0, left=-.3)
        self.assert_point(frame[5], distance=.5, forward=0, left=.5)

    def test_default_reflection_preserves_bypass_scope_and_nearest_veto(self):
        runtime = self.runtime()
        runtime.front_disabled.set()
        node, _, path, fields = self.produce(runtime, self.scan(0))
        self.assertIn("F=C|B=O", path)
        self.assertEqual(fields["CF"], "80.0")
        self.assertEqual(fields["CL"], "30.0")
        self.assertEqual(fields["CR"], "50.0")
        # Bypass does not rescue missing physical LEFT coverage or stale data.
        scan = self.scan(0)
        for index in range(68, 113):
            scan.ranges[index] = math.nan
        with patch.object(runtime.time, "monotonic", return_value=100.1):
            node.scan_callback(scan)
            path, packet = node.build_status_packets()
            self.assertIn("F=S|B=S", path)
            self.assertNotIn("|CL=", packet)
        with patch.object(runtime.time, "monotonic", return_value=101.0):
            self.assertIn("F=S|B=S", node.build_status_packets()[0])

    def test_invalid_mount_option_retains_reported_default(self):
        runtime = self.runtime(reflect="invalid")
        self.assertTrue(runtime.LIDAR_REFLECT_LEFT_RIGHT)
        _, _, _, fields = self.produce(runtime, self.scan(0))
        self.assertEqual(fields["CL"], "30.0")

    def test_default_reflection_keeps_invalid_metadata_fail_closed(self):
        for field, value in (("angle_min", math.nan), ("angle_min", math.inf),
                             ("angle_increment", math.nan), ("angle_increment", math.inf),
                             ("angle_increment", 0.0), ("range_min", None),
                             ("range_max", None)):
            with self.subTest(field=field, value=value):
                runtime = self.runtime()
                frames = queue.Queue()
                node = runtime.LidarDistanceReader(None, frames)
                with patch.object(runtime.time, "monotonic", return_value=100.0):
                    node.scan_callback(fixtures.RuntimeTests.scan())
                    self.assertTrue(runtime.lidar_healthy.is_set())
                self.assertIsNotNone(frames.get_nowait()[5])
                invalid = fixtures.RuntimeTests.scan()
                setattr(invalid, field, value)
                with patch.object(runtime.time, "monotonic", return_value=100.1):
                    node.scan_callback(invalid)
                    self.assertIn("F=S|B=S", node.build_status_packets()[0])
                self.assertFalse(runtime.lidar_healthy.is_set())
                if not frames.empty():
                    frame = frames.get_nowait()
                    self.assertIsNone(frame[5])
                    self.assertTrue(all(value is None for value in frame[0].values()))

    def test_reflection_only_exchanges_lateral_sectors_even_with_yaw(self):
        mirror = {"FRONT": "FRONT", "BACK": "BACK", "LEFT": "RIGHT", "RIGHT": "LEFT",
                  "FRONT_LEFT": "FRONT_RIGHT", "FRONT_RIGHT": "FRONT_LEFT",
                  "BACK_LEFT": "BACK_RIGHT", "BACK_RIGHT": "BACK_LEFT"}
        scan = fixtures.RuntimeTests.scan([1 + index / 360 for index in range(360)])
        for yaw in (0, 30, -90):
            with self.subTest(yaw=yaw):
                original = self.produce(self.runtime(yaw=yaw, reflect="0"), scan)[0]
                reflected = self.produce(self.runtime(yaw=yaw, reflect="1"), scan)[0]
                for side, opposite in mirror.items():
                    self.assertEqual(reflected.history[side][-1], original.history[opposite][-1])


if __name__ == "__main__":
    unittest.main()
