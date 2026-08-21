import math
import threading
import time
import unittest

from bridge_core import (
    CoalescingUpdateWorker,
    ExponentialBackoff,
    LIDAR_SECTOR_NAMES,
    SENSOR_SPECS,
    SensorTransitionTracker,
    build_health_status,
    clamp_heading_error_cm,
    format_lidar_blockage,
    lidar_status_code,
    nearest_lidar_sector,
    partition_lidar_samples,
    robust_near_distance_cm,
    robust_wall_distance_cm,
    select_steering_samples,
)


class SensorTrackerTests(unittest.TestCase):
    def make_tracker(self):
        return SensorTransitionTracker(stale_after_s=2.0, live_publish_period_s=1.0)

    def test_initial_unavailable_sentinels_are_emitted_once(self):
        tracker = self.make_tracker()
        first = tracker.collect_due(0.0)
        self.assertEqual(set(first), set(SENSOR_SPECS))
        self.assertEqual(first["ultrasonic"], 999.0)
        self.assertEqual(first["mq4"], -1)
        self.assertEqual(tracker.collect_due(0.1), {})

    def test_repeated_explicit_unavailable_does_not_republish(self):
        tracker = self.make_tracker()
        tracker.collect_due(0.0)
        tracker.mark_unavailable(["mq4"])
        tracker.mark_unavailable(["mq4"])
        self.assertEqual(tracker.collect_due(0.2), {})

    def test_recovery_rearms_exactly_one_future_sentinel(self):
        tracker = self.make_tracker()
        tracker.collect_due(0.0)
        self.assertTrue(tracker.ingest("mq4", 150, 0.2))
        self.assertEqual(tracker.collect_due(0.2)["mq4"], 150)
        tracker.mark_unavailable(["mq4"])
        self.assertEqual(tracker.collect_due(0.3)["mq4"], -1)
        self.assertNotIn("mq4", tracker.collect_due(0.4))

    def test_partial_sensor_failure_does_not_overwrite_live_sensor(self):
        tracker = self.make_tracker()
        tracker.collect_due(0.0)
        tracker.ingest("mq4", 100, 0.1)
        tracker.ingest("mq135", 200, 0.1)
        tracker.collect_due(0.1)
        tracker.mark_unavailable(["mq4"])
        due = tracker.collect_due(0.2)
        self.assertEqual(due, {"mq4": -1})
        self.assertEqual(tracker.latest_for_transport("mq135", 0.2), 200)

    def test_live_refresh_is_rate_bounded(self):
        tracker = self.make_tracker()
        tracker.collect_due(0.0)
        tracker.ingest("ultrasonic", 42.2, 0.1)
        self.assertIn("ultrasonic", tracker.collect_due(0.1))
        tracker.ingest("ultrasonic", 43.0, 0.4)
        self.assertNotIn("ultrasonic", tracker.collect_due(0.4))
        self.assertIn("ultrasonic", tracker.collect_due(1.2))


class BackoffTests(unittest.TestCase):
    def test_backoff_is_bounded_and_resettable(self):
        backoff = ExponentialBackoff(1.0, 4.0, 2.0)
        self.assertEqual([backoff.next_delay() for _ in range(5)], [1.0, 2.0, 4.0, 4.0, 4.0])
        backoff.reset()
        self.assertEqual(backoff.next_delay(), 1.0)


class HealthTests(unittest.TestCase):
    def test_unknown_thermal_sources_do_not_report_false_ok(self):
        health = build_health_status(
            ros_running=True, lidar_scan_fresh=True, ble_connected=True,
            sensor_link_fresh=True, firebase_online=True,
        )
        self.assertIsNone(health["thermalWarning"])

    def test_available_thermal_source_preserves_ok_and_warning_states(self):
        ok = build_health_status(
            ros_running=True, lidar_scan_fresh=True, ble_connected=True,
            sensor_link_fresh=True, firebase_online=True,
            cpu_temperature_c=55.0, throttled_flags=0,
        )
        warn = build_health_status(
            ros_running=True, lidar_scan_fresh=True, ble_connected=True,
            sensor_link_fresh=True, firebase_online=True,
            cpu_temperature_c=82.0, throttled_flags=0,
        )
        self.assertFalse(ok["thermalWarning"])
        self.assertTrue(warn["thermalWarning"])


class LidarTests(unittest.TestCase):
    def test_lidar_representatives_and_tilt_clamp(self):
        self.assertEqual(robust_near_distance_cm([0.1, 1.0, 1.0, 1.0, 1.0]), 10.0)
        self.assertEqual(robust_wall_distance_cm([0.8, 1.0, 1.2, 1.4]), 110.0)
        self.assertEqual(clamp_heading_error_cm(99.0, 40.0), 40.0)
        self.assertEqual(clamp_heading_error_cm(-99.0, 40.0), -40.0)

    def test_lidar_named_sector_orientation_and_boundaries(self):
        expected = {
            0: "FRONT", 45: "FRONT_LEFT", 90: "LEFT", 135: "BACK_LEFT",
            180: "BACK", 225: "BACK_RIGHT", 270: "RIGHT", 315: "FRONT_RIGHT",
        }
        for degrees, name in expected.items():
            self.assertEqual(nearest_lidar_sector(math.radians(degrees))[0], name)
        self.assertEqual(nearest_lidar_sector(math.radians(22.4))[0], "FRONT")
        self.assertEqual(nearest_lidar_sector(math.radians(22.6))[0], "FRONT_LEFT")

    def test_lidar_nearest_sectors_have_no_angular_gaps(self):
        for degree in range(3600):
            name, error = nearest_lidar_sector(math.radians(degree / 10.0))
            self.assertIn(name, LIDAR_SECTOR_NAMES)
            self.assertLessEqual(error, math.radians(22.5) + 1e-9)

    def test_positive_infinity_is_clear_at_range_max(self):
        safety, _steering, valid = partition_lidar_samples(
            [float("inf")], angle_min=0.0, angle_increment=0.0,
            range_min=0.05, range_max=8.0, steering_cone_deg=22.0,
        )
        self.assertEqual(valid, 1)
        self.assertEqual(safety["FRONT"], [8.0])

    def test_between_cone_obstacle_is_safety_visible_but_not_steering_input(self):
        # 22.4° is inside FRONT safety Voronoi sector but outside a 20° steering cone.
        ranges = [1.0]
        safety, steering, valid = partition_lidar_samples(
            ranges,
            angle_min=math.radians(22.4), angle_increment=0.0,
            range_min=0.05, range_max=12.0, steering_half_width_deg=20.0,
        )
        self.assertEqual(valid, 1)
        self.assertEqual(safety["FRONT"], [1.0])
        self.assertEqual(steering["FRONT"], [])

    def test_trash_ultrasonic_cannot_change_path_classification(self):
        # Classification is solely a function of LiDAR status text.
        self.assertEqual(lidar_status_code(format_lidar_blockage([])[1]), "C")
        self.assertEqual(lidar_status_code(format_lidar_blockage(["FRONT"])[1]), "O")
        self.assertEqual(lidar_status_code("LIDAR_STALE"), "S")

    def test_sparse_steering_cone_falls_back_to_gap_free_samples(self):
        safety = [1.0, 1.1, 1.2]
        narrow = [1.0]
        self.assertIs(select_steering_samples(narrow, safety, minimum_points=3), safety)


class UpdateWorkerTests(unittest.TestCase):
    def test_update_worker_coalesces_to_one_atomic_map(self):
        calls = []
        event = threading.Event()

        def update(payload):
            calls.append(payload)
            event.set()

        worker = CoalescingUpdateWorker(update, batch_window_s=0.03)
        worker.start()
        worker.submit({"a": 1})
        worker.submit({"b": 2})
        worker.submit({"a": 3})
        self.assertTrue(event.wait(1.0))
        worker.stop()
        self.assertEqual(calls[0], {"a": 3, "b": 2})

    def test_update_retry_backoff_is_not_bypassed_by_new_samples(self):
        call_times = []
        success = threading.Event()

        def update(payload):
            call_times.append(time.monotonic())
            if len(call_times) == 1:
                raise RuntimeError("simulated outage")
            success.set()

        worker = CoalescingUpdateWorker(
            update,
            batch_window_s=0.0,
            retry_backoff=ExponentialBackoff(0.12, 0.12, 1.0),
        )
        worker.start()
        worker.submit({"a": 1})
        deadline = time.monotonic() + 1.0
        while len(call_times) < 1 and time.monotonic() < deadline:
            time.sleep(0.005)
        worker.submit({"a": 2, "b": 3})
        self.assertTrue(success.wait(1.0))
        worker.stop()
        self.assertGreaterEqual(call_times[1] - call_times[0], 0.10)


if __name__ == "__main__":
    unittest.main()
