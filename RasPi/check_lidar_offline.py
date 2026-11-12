#!/usr/bin/env python3
"""Test production LiDAR decisions without ROS, LiDAR, BLE, Firebase or motors."""
import unittest
from test_lidar_offline import LidarOfflineTests

if __name__ == '__main__':
    print('OFFLINE: synthetic raw scans, production scan processing, no hardware connections.', flush=True)
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(LidarOfflineTests))
    raise SystemExit(0 if result.wasSuccessful() else 1)
