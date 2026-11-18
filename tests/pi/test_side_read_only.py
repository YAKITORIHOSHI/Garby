"""No hardware: the sides read-only control (owner, 2026-11-16: "add read only for sides
too, so that the motor can be tested and run")."""
import raspi_path  # noqa: F401  (puts the deployable RasPi/ folder on sys.path)
import queue
import unittest

import test_runtime as fixtures
from test_reverse_parking import BACKING_AT_DESTINATION, at

# 1 cm beside the body, ahead of the spin centre: a side band. It is also inside the circle
# the body sweeps when it spins, which belongs to the front group's turn check.
BESIDE_RIGHT = (2.0, -31.0)
BESIDE_LEFT = (2.0, 31.0)
# Inside the 22 cm side-sector limit, measured from the LiDAR (the bench's RIGHT hold).
CLOSE_LEFT = (0.0, 21.0)


class SideReadOnlyTests(unittest.TestCase):
    def setUp(self):
        self.runtime = fixtures.load_runtime()
        for event in (self.runtime.side_disabled, self.runtime.front_disabled,
                      self.runtime.back_disabled):
            self.addCleanup(event.clear)

    def path(self, *points):
        scan = fixtures.RuntimeTests.scan()
        for point in points:
            at(scan, *point)
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        node.scan_callback(scan)
        return node, node.build_status_packets()[0].split("|", 1)[1]

    def test_something_beside_the_robot_holds_every_motion_until_sides_are_read_only(self):
        for point in (BESIDE_RIGHT, BESIDE_LEFT, CLOSE_LEFT):
            with self.subTest(point=point):
                # Each case starts with every group on, even if an earlier one failed.
                for event in (self.runtime.side_disabled, self.runtime.front_disabled):
                    event.clear()
                node, path = self.path(point)
                self.assertEqual(path, "F=O|B=O|T=O|R=O")
                self.assertIn("Side clearance blocked", node.obstacle_hold_reason())
                self.runtime.side_disabled.set()
                node, path = self.path(point)
                # Driving forward, backing: free. Beside the body, a spin would sweep
                # through it: the front group's turn check holds that until front
                # read-only too. The sector point lies inside the assumed body, which the
                # spin sweep does not count.
                self.assertEqual(path, "F=C|B=C|T=C|R=C" if point == CLOSE_LEFT else "F=C|B=C|T=O|R=C")
                self.assertEqual(node.obstacle_hold_reason(), "")
                self.runtime.front_disabled.set()
                _, path = self.path(point)
                self.assertEqual(path, "F=C|B=C|T=C|R=C")
                self.runtime.side_disabled.clear()
                self.runtime.front_disabled.clear()

    def test_front_and_back_read_only_still_do_not_bypass_the_sides(self):
        self.runtime.front_disabled.set()
        self.runtime.back_disabled.set()
        _, path = self.path(BESIDE_RIGHT)
        self.assertTrue(path.startswith("F=O|B=O"))

    def test_sides_read_only_bypasses_only_the_sides(self):
        self.runtime.side_disabled.set()
        _, path = self.path(BESIDE_RIGHT, (40.0, 0.0))   # and something in the front box
        self.assertTrue(path.startswith("F=O|B=C"))

    def test_missing_or_stale_scans_are_never_made_clear(self):
        self.runtime.side_disabled.set()
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        self.assertTrue(node.build_status_packets()[0].endswith("|F=S|B=S"))

    def test_the_bands_beside_the_rear_half_while_backing(self):
        runtime = self.runtime
        runtime.ble_connected.set()
        try:
            runtime.ble_notify_handler("[RUNNING]")
            runtime.ble_notify_handler(BACKING_AT_DESTINATION)
            _, path = self.path((-20.0, 30.5))
            self.assertTrue(path.endswith("|R=O"))
            runtime.side_disabled.set()
            _, path = self.path((-20.0, 30.5))
            self.assertTrue(path.endswith("|R=C"))
        finally:
            runtime.ble_connected.clear()
            runtime.station_mode.invalidate()
            runtime.publish_robot_state("UNKNOWN")

    def test_readings_and_zones_stay_visible_while_read_only(self):
        self.runtime.side_disabled.set()
        node, _ = self.path(BESIDE_RIGHT)
        self.assertIn("RIGHT_BAND", node._active_zones())


if __name__ == "__main__":
    unittest.main()
