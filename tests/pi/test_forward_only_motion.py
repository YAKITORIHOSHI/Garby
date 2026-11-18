"""Forward-only route segments ignore what lies behind the body (owner, 2026-11-14).

Every route segment drives forward; turns spin in place and keep their own sweep
(T=). The base spot puts a wall 22.5 cm behind the LiDAR and a wall step beside
the rear corner. Before this change both held a manual disposal start forever.
"""
import raspi_path  # noqa: F401  (puts the deployable RasPi/ folder on sys.path)
import queue
import time
import types
import unittest
from pathlib import Path

import numpy as np

import test_runtime as fixtures
from hallway_geometry import ChassisGeometry, analyze_hallway

RECORDING = (raspi_path.ROOT /
             "artifacts/manual-disposal-hold-2026-11-14/scans-base.npz")


def recorded_scans():
    data = np.load(RECORDING)
    for row, length, meta in zip(data["ranges"], data["lengths"], data["meta"]):
        angle_min, increment, range_min, range_max = (float(value) for value in meta)
        stamp_ns = time.time_ns()  # acquired now: the runtime rejects old/future stamps
        yield types.SimpleNamespace(
            ranges=[float(value) for value in row[:int(length)]],
            angle_min=angle_min, angle_increment=increment,
            range_min=range_min, range_max=range_max,
            header=types.SimpleNamespace(stamp=types.SimpleNamespace(
                sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)))


class SideBandGeometryTests(unittest.TestCase):
    @staticmethod
    def points(*xy):
        # One return per (forward, left) point in cm, as in test_hallway_geometry.
        import math
        ranges = [math.inf] * 3600
        for x, y in xy:
            angle = math.atan2(y, x)
            ranges[int(round((angle + math.pi) / (2 * math.pi / 3600))) % 3600] = math.hypot(x, y) / 100
        return analyze_hallway(ranges, angle_min=-math.pi, angle_increment=2 * math.pi / 3600,
                               range_min=0.01, range_max=8.0, geometry=ChassisGeometry())

    def test_a_return_beside_the_rear_half_does_not_hold_a_forward_move(self):
        self.assertFalse(self.points((-20.0, 30.5)).side_left)
        self.assertFalse(self.points((-25.0, -31.0)).side_right)

    def test_a_person_beside_the_front_half_still_holds(self):
        self.assertTrue(self.points((15.0, 31.0)).side_left)
        self.assertTrue(self.points((2.0, -31.5)).side_right)

    def test_the_spin_sweep_still_sees_the_rear_corner(self):
        self.assertTrue(self.points((-20.0, 30.5)).turn_back)


class RuntimeForwardOnlyTests(unittest.TestCase):
    def setUp(self):
        self.runtime = fixtures.load_runtime()

    def test_an_object_close_behind_does_not_hold_forward_motion(self):
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        scan = fixtures.RuntimeTests.scan()
        scan.ranges[0] = 0.33  # 3 cm behind the assumed rear edge
        node.scan_callback(scan)
        path = node.build_status_packets()[0]
        self.assertIn("F=C|B=C", path)
        self.assertIn("T=O", path)  # a spin would still sweep it

    def test_a_close_side_object_still_holds_both_directions(self):
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        scan = fixtures.RuntimeTests.scan()
        scan.ranges[90] = 0.15
        node.scan_callback(scan)
        self.assertIn("F=O|B=O", node.build_status_packets()[0])

    def test_the_real_base_spot_releases_a_forward_start(self):
        for read_only in (False, True):
            with self.subTest(read_only=read_only):
                runtime = fixtures.load_runtime()
                if read_only:
                    runtime.front_disabled.set()
                    runtime.back_disabled.set()
                node = runtime.LidarDistanceReader(None, queue.Queue())
                paths = []
                for scan in recorded_scans():
                    node.scan_callback(scan)
                    paths.append(node.build_status_packets()[0])
                self.assertEqual(len(paths), 15)
                for path in paths:
                    # C = GO; W = GO:SLOW (something within the 160 cm slow zone).
                    # A bypassed front group bypasses its slow zone as well.
                    self.assertRegex(path, r"\|F=C\|B=C\|" if read_only else r"\|F=[CW]\|B=C\|")
                if not read_only:
                    # The wall behind lies inside the spin circle: turns stay held.
                    self.assertTrue(all("|T=O|" in path for path in paths))


if __name__ == "__main__":
    unittest.main()
