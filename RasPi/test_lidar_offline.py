"""Raw synthetic scans through the production runtime; never connects hardware."""
import math
import queue
import unittest
from unittest.mock import patch
import test_runtime as fixtures


class LidarOfflineTests(unittest.TestCase):
    def setUp(self):
        self.runtime = fixtures.load_runtime()
        self.node = self.runtime.LidarDistanceReader(None, queue.Queue())

    def path(self):
        return self.node.build_status_packets()[0]

    def test_missing_lidar_holds_both_directions(self):
        self.assertIn('F=S|B=S', self.path())
        self.assertFalse(self.runtime.lidar_healthy.is_set())

    def test_fresh_clear_scan_allows_both_directions(self):
        self.node.scan_callback(fixtures.RuntimeTests.scan())
        self.assertIn('F=C|B=C', self.path())

    def test_periodic_obstacle_snapshot_cannot_demote_urgent_scan_stop(self):
        scan = fixtures.RuntimeTests.scan()
        scan.ranges[180] = 0.45
        self.node.scan_callback(scan)  # producer queues urgent P1
        mailbox = self.runtime.ble_send_queue
        mailbox.put_nowait("SENSOR:US=100")
        mailbox.put_nowait("[HEALTH:1:UUUUU]")
        mailbox.put_nowait(self.path())  # periodic producer queues normal P2
        # 45 cm ahead is also inside the spin sweep, so T=O as well.
        self.assertEqual(mailbox.get_nowait(), "P:2|F=O|B=C|T=O")

    def test_periodic_stale_snapshot_cannot_demote_watchdog_stop(self):
        with patch.object(self.runtime.time, 'monotonic', return_value=100):
            self.node.scan_callback(fixtures.RuntimeTests.scan())
        with patch.object(self.runtime.time, 'monotonic', return_value=100.81):
            self.node._watchdog_callback()  # watchdog queues urgent P1
            mailbox = self.runtime.ble_send_queue
            mailbox.put_nowait("SENSOR:US=100")
            mailbox.put_nowait(self.path())
        self.assertEqual(mailbox.get_nowait(), "P:2|F=S|B=S")

    def test_single_chair_leg_at_front_angles_blocks_forward(self):
        # Angles are in the software frame; verify physical yaw when fitted.
        for angle in (-60, -30, 0, 30, 60):
            with self.subTest(angle=angle):
                scan = fixtures.RuntimeTests.scan()
                scan.ranges[angle + 180] = 0.45
                self.node.scan_callback(scan)
                self.assertIn('F=O', self.path())

    def test_rear_obstacle_blocks_reverse(self):
        scan = fixtures.RuntimeTests.scan(); scan.ranges[0] = 0.20
        self.node.scan_callback(scan)
        self.assertIn('B=O', self.path())

    def test_close_side_chair_blocks_both_directions(self):
        for index in (90, 270):
            with self.subTest(index=index):
                scan = fixtures.RuntimeTests.scan(); scan.ranges[index] = 0.15
                self.node.scan_callback(scan)
                self.assertIn('F=O|B=O', self.path())

    def test_scan_loss_after_clear_revokes_permission(self):
        with patch.object(self.runtime.time, 'monotonic', return_value=100):
            self.node.scan_callback(fixtures.RuntimeTests.scan())
            self.assertIn('F=C|B=C', self.path())
        with patch.object(self.runtime.time, 'monotonic', return_value=100.81):
            self.assertIn('F=S|B=S', self.path())

    def test_chair_removed_recovers_with_new_scans(self):
        scan = fixtures.RuntimeTests.scan(); scan.ranges[180] = 0.45
        self.node.scan_callback(scan)
        self.assertIn('F=O', self.path())
        for _ in range(3):
            self.node.scan_callback(fixtures.RuntimeTests.scan())
        self.assertIn('F=C|B=C', self.path())

    def test_invalid_scan_never_grants_permission(self):
        for value in (0.0, math.nan, -1.0):
            with self.subTest(value=value):
                self.node.scan_callback(fixtures.RuntimeTests.scan([value] * 360))
                self.assertIn('F=S|B=S', self.path())


if __name__ == '__main__':
    unittest.main()
