import math
import threading
import time
import unittest

from bridge_core import (
    CoalescingUpdateWorker,
    ExponentialBackoff,
    LIDAR_SECTOR_NAMES,
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


class SensorTransitionTrackerTests(unittest.TestCase):
    def make_tracker(self):
        return SensorTransitionTracker(
            stale_after_s=5.0,
            live_publish_period_s=10.0,
            started_at=100.0,
        )

    def test_initial_unavailable_sentinels_are_emitted_once(self):
        tracker = self.make_tracker()
        self.assertEqual(tracker.collect_due(now=104.9), {})
        self.assertEqual(
            tracker.collect_due(now=105.0),
            {"ultrasonic": 999.0, "mq4": -1, "mq137": -1, "mq135": -1},
        )
        self.assertEqual(tracker.collect_due(now=200.0), {})

    def test_repeated_explicit_unavailable_does_not_republish(self):
        tracker = self.make_tracker()
        tracker.ingest("mq4", -1, now=101.0)
        self.assertEqual(tracker.collect_due(now=101.0), {"mq4": -1})
        for sample_time in (102.0, 103.0, 110.0):
            tracker.ingest("mq4", -1, now=sample_time)
            self.assertNotIn("mq4", tracker.collect_due(now=sample_time))

    def test_recovery_rearms_exactly_one_future_sentinel(self):
        tracker = self.make_tracker()
        tracker.mark_unavailable(["ultrasonic"])
        self.assertEqual(tracker.collect_due(now=101.0), {"ultrasonic": 999.0})

        self.assertTrue(tracker.ingest("ultrasonic", 42.5, now=102.0))
        self.assertEqual(tracker.collect_due(now=102.0), {"ultrasonic": 42.5})

        self.assertNotIn("ultrasonic", tracker.collect_due(now=106.9))
        self.assertEqual(tracker.collect_due(now=107.0), {"ultrasonic": 999.0})
        self.assertEqual(tracker.collect_due(now=120.0), {})

    def test_partial_sensor_failure_does_not_overwrite_live_sensor(self):
        tracker = self.make_tracker()
        tracker.ingest("mq135", 321, now=104.0)
        due = tracker.collect_due(now=105.0)
        self.assertEqual(due["mq135"], 321)
        self.assertEqual(due["ultrasonic"], 999.0)
        self.assertEqual(due["mq4"], -1)
        self.assertEqual(due["mq137"], -1)

    def test_live_refresh_is_rate_bounded(self):
        tracker = self.make_tracker()
        tracker.ingest("mq137", 10, now=100.1)
        self.assertEqual(tracker.collect_due(now=100.1), {"mq137": 10})
        tracker.ingest("mq137", 11, now=101.0)
        self.assertEqual(tracker.collect_due(now=101.0), {})
        tracker.ingest("mq137", 12, now=109.9)
        self.assertNotIn("mq137", tracker.collect_due(now=109.9))
        self.assertEqual(tracker.collect_due(now=110.1), {"mq137": 12})


class ReliabilityPrimitiveTests(unittest.TestCase):
    def test_backoff_is_bounded_and_resettable(self):
        backoff = ExponentialBackoff(1.0, 4.0)
        self.assertEqual([backoff.next_delay() for _ in range(5)], [1.0, 2.0, 4.0, 4.0, 4.0])
        backoff.reset()
        self.assertEqual(backoff.next_delay(), 1.0)

    def test_unknown_thermal_sources_do_not_report_false_ok(self):
        status = build_health_status(
            cpu_temperature_c=None,
            throttled_flags=None,
            ble_connected=True,
            lidar_healthy=False,
            sensor_serial_connected=True,
        )
        self.assertEqual(
            status,
            {
                "bleConnected": True,
                "lidarHealthy": False,
                "sensorSerialConnected": True,
            },
        )
        self.assertNotIn("thermalWarning", status)

    def test_available_thermal_source_preserves_ok_and_warning_states(self):
        cool = build_health_status(
            cpu_temperature_c=54.6,
            throttled_flags=None,
            ble_connected=False,
            lidar_healthy=False,
            sensor_serial_connected=False,
        )
        self.assertEqual(cool["cpuTemperatureC"], 55)
        self.assertIs(cool["thermalWarning"], False)
        self.assertNotIn("throttledFlags", cool)

        throttled = build_health_status(
            cpu_temperature_c=None,
            throttled_flags=0x2,
            ble_connected=False,
            lidar_healthy=False,
            sensor_serial_connected=False,
        )
        self.assertEqual(throttled["throttledFlags"], 0x2)
        self.assertIs(throttled["thermalWarning"], True)

    def test_lidar_representatives_and_tilt_clamp(self):
        samples = [0.20, 1.00, 1.01, 0.99, 1.02, 1.00]
        # The isolated closest speck is rejected for the safety low percentile.
        self.assertAlmostEqual(robust_near_distance_cm(samples), 99.0)
        self.assertAlmostEqual(robust_wall_distance_cm(samples), 100.0)
        self.assertEqual(clamp_heading_error_cm(200.0), 20.0)
        self.assertEqual(clamp_heading_error_cm(-200.0), -20.0)

    def test_lidar_named_sector_orientation_and_boundaries(self):
        centers = {
            0.0: "BACK",
            45.0: "BACK_RIGHT",
            90.0: "RIGHT",
            135.0: "FRONT_RIGHT",
            180.0: "FRONT",
            225.0: "FRONT_LEFT",
            270.0: "LEFT",
            315.0: "BACK_LEFT",
            360.0: "BACK",
            -90.0: "LEFT",
        }
        for angle_deg, expected in centers.items():
            side, error = nearest_lidar_sector(math.radians(angle_deg))
            self.assertEqual(side, expected)
            self.assertAlmostEqual(error, 0.0, places=10)

        boundary_cases = {
            22.499: "BACK",
            22.5: "BACK_RIGHT",
            67.499: "BACK_RIGHT",
            67.5: "RIGHT",
            337.499: "BACK_LEFT",
            337.5: "BACK",
        }
        for angle_deg, expected in boundary_cases.items():
            side, error = nearest_lidar_sector(math.radians(angle_deg))
            self.assertEqual(side, expected)
            self.assertLessEqual(error, math.radians(22.5) + 1e-12)

    def test_lidar_nearest_sectors_have_no_angular_gaps(self):
        counts = {side: 0 for side in LIDAR_SECTOR_NAMES}
        for tenth_degree in range(-3600, 7200):
            side, error = nearest_lidar_sector(math.radians(tenth_degree / 10.0))
            counts[side] += 1
            self.assertLessEqual(error, math.radians(22.5) + 1e-12)
        self.assertTrue(all(count > 0 for count in counts.values()))

    def test_between_cone_obstacle_is_safety_visible_but_not_steering_input(self):
        # BACK-centered narrow steering slice is ±11°. Obstacles at 15° and
        # 20° used to fall in the 23° blind gap before the next centered slice.
        ranges = [2.0, 2.0, 2.0, 0.30, 0.25, 2.0, 2.0, 2.0, 2.0, 2.0]
        safety, steering, valid_count = partition_lidar_samples(
            ranges,
            angle_min=0.0,
            angle_increment=math.radians(5.0),
            range_min=0.03,
            range_max=8.0,
            steering_cone_deg=22.0,
        )

        self.assertEqual(valid_count, len(ranges))
        self.assertEqual(safety["BACK"], [2.0, 2.0, 2.0, 0.30, 0.25])
        self.assertEqual(steering["BACK"], [2.0, 2.0, 2.0])
        self.assertAlmostEqual(robust_near_distance_cm(safety["BACK"]), 30.0)

        chosen = select_steering_samples(steering["BACK"], safety["BACK"])
        self.assertEqual(chosen, steering["BACK"])
        self.assertAlmostEqual(robust_wall_distance_cm(chosen), 200.0)
        self.assertEqual(
            select_steering_samples(steering["BACK"][:2], safety["BACK"]),
            safety["BACK"],
        )

    def test_trash_ultrasonic_cannot_change_path_classification(self):
        tracker = SensorTransitionTracker(
            stale_after_s=5.0,
            live_publish_period_s=10.0,
            started_at=0.0,
        )
        results = []
        for trash_distance_cm in (5.0, 50.0, 190.0, 999.0):
            tracker.ingest("ultrasonic", trash_distance_cm, now=1.0)
            blocked, status = format_lidar_blockage(
                ["FRONT"],
                ["FRONT", "FRONT_LEFT", "FRONT_RIGHT"],
            )
            results.append((blocked, status, lidar_status_code(status)))

        self.assertEqual(results, [(True, "BLOCKED{FRONT}", "O")] * 4)
        self.assertNotIn("HUMAN_DETECTED", " ".join(status for _, status, _ in results))

        # H remains decodable for a legacy sender, but this bridge never derives
        # it from its trash-level UART sensor.
        self.assertEqual(lidar_status_code("BLOCKED{HUMAN_DETECTED,FRONT}"), "H")

    def test_update_worker_coalesces_to_one_atomic_map(self):
        committed = []
        done = threading.Event()

        def commit(batch):
            committed.append(dict(batch))
            done.set()

        worker = CoalescingUpdateWorker(commit, batch_window_s=0.05)
        try:
            worker.enqueue({"a": 1, "old": True})
            worker.enqueue({"a": 2, "b": 3})
            self.assertTrue(done.wait(1.0))
        finally:
            worker.close()
        self.assertEqual(committed, [{"a": 2, "old": True, "b": 3}])

    def test_update_retry_backoff_is_not_bypassed_by_new_samples(self):
        attempts = []
        succeeded = threading.Event()

        def commit(batch):
            attempts.append((time.monotonic(), dict(batch)))
            if len(attempts) == 1:
                raise OSError("offline")
            succeeded.set()

        worker = CoalescingUpdateWorker(
            commit,
            batch_window_s=0.01,
            retry_min_s=0.12,
            retry_max_s=0.12,
        )
        try:
            worker.enqueue({"sensor": 1})
            time.sleep(0.04)
            worker.enqueue({"sensor": 2})
            self.assertTrue(succeeded.wait(1.0))
        finally:
            worker.close()

        self.assertGreaterEqual(attempts[1][0] - attempts[0][0], 0.10)
        self.assertEqual(attempts[1][1]["sensor"], 2)


if __name__ == "__main__":
    unittest.main()
