import math
import threading
import time
import unittest

from bridge_core import (
    AdaptiveTolerance,
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
    side_obstacle_names,
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


class AdaptiveToleranceTests(unittest.TestCase):
    """Watchdog tolerance must adapt to throttling without weakening fail-closed."""

    def make_tolerance(self):
        return AdaptiveTolerance(0.8, 2.0, alpha=0.25, margin_factor=2.0)

    def test_untoleranced_cadence_uses_floor(self):
        tolerance = self.make_tolerance()
        self.assertEqual(tolerance.tolerance_s(), 0.8)

    def test_normal_fast_cadence_stays_at_floor(self):
        tolerance = self.make_tolerance()
        for _ in range(10):
            tolerance.observe(0.25)
        self.assertEqual(tolerance.tolerance_s(), 0.8)

    def test_throttled_cadence_widens_and_caps(self):
        tolerance = self.make_tolerance()
        # Sustained 500 ms cadence (2x CPU throttling): EWMA converges to 0.5,
        # tolerance widens to 1.0 s so the stream is not reported stale.
        for _ in range(20):
            tolerance.observe(0.5)
        self.assertAlmostEqual(tolerance.tolerance_s(), 1.0)
        # Extreme cadence clamps at the hard cap: never unbounded.
        for _ in range(40):
            tolerance.observe(5.0)
        self.assertEqual(tolerance.tolerance_s(), 2.0)

    def test_invalid_intervals_are_ignored(self):
        tolerance = self.make_tolerance()
        tolerance.observe(None)
        tolerance.observe(float("nan"))
        tolerance.observe(-1.0)
        self.assertEqual(tolerance.tolerance_s(), 0.8)
        tolerance.observe(1.0)
        self.assertAlmostEqual(tolerance.tolerance_s(), 2.0)

    def test_reset_returns_to_floor(self):
        tolerance = self.make_tolerance()
        for _ in range(10):
            tolerance.observe(0.9)
        tolerance.reset()
        self.assertEqual(tolerance.tolerance_s(), 0.8)

    def test_widening_never_drops_below_floor_mid_stream(self):
        tolerance = self.make_tolerance()
        tolerance.observe(1.5)
        self.assertEqual(tolerance.tolerance_s(), 2.0)
        # One fast sample narrows the EWMA but never under the floor.
        tolerance.observe(0.25)
        self.assertGreaterEqual(tolerance.tolerance_s(), 0.8)

    def test_jittered_throttling_cadence(self):
        tolerance = self.make_tolerance()
        # Jitter between 0.3s and 0.6s (avg 0.45s). With margin 2.0x, tolerance stays between 0.8 and 1.2
        for i in range(30):
            interval = 0.3 if i % 2 == 0 else 0.6
            tolerance.observe(interval)
        self.assertGreaterEqual(tolerance.tolerance_s(), 0.8)
        self.assertLessEqual(tolerance.tolerance_s(), 1.2)

    def test_zero_interval_burst(self):
        tolerance = self.make_tolerance()
        tolerance.observe(0.5)
        # Bursty executor delivery: several 0.0s intervals
        for _ in range(5):
            tolerance.observe(0.0)
        self.assertGreaterEqual(tolerance.tolerance_s(), 0.8)

    def test_boundary_floor_equals_cap(self):
        tolerance = AdaptiveTolerance(1.0, 1.0)
        tolerance.observe(0.1)
        self.assertEqual(tolerance.tolerance_s(), 1.0)
        tolerance.observe(5.0)
        self.assertEqual(tolerance.tolerance_s(), 1.0)

    def test_step_throttling_gradual_convergence(self):
        tolerance = self.make_tolerance()
        for _ in range(10):
            tolerance.observe(0.25)
        self.assertEqual(tolerance.tolerance_s(), 0.8)
        for _ in range(25):
            tolerance.observe(0.45)
        self.assertAlmostEqual(tolerance.tolerance_s(), 0.9, places=2)


class ThrottlingResilienceTests(unittest.TestCase):
    """Test system components under simulated CPU and connection throttling."""

    def test_backoff_pacing_under_connection_throttling(self):
        # 50 ms base, 800 ms max, 5 strikes
        backoff = ExponentialBackoff(0.05, 0.80, 2.0)
        delays = [backoff.next_delay() for _ in range(6)]
        self.assertAlmostEqual(delays[0], 0.05)
        self.assertAlmostEqual(delays[1], 0.10)
        self.assertAlmostEqual(delays[2], 0.20)
        self.assertAlmostEqual(delays[3], 0.40)
        self.assertAlmostEqual(delays[4], 0.80)
        self.assertAlmostEqual(delays[5], 0.80)
        backoff.reset()
        self.assertAlmostEqual(backoff.next_delay(), 0.05)

    def test_update_worker_coalescing_under_cpu_contention(self):
        # When CPU is saturated, multiple sensor updates arrive before the worker flushes
        flushes = []
        barrier = threading.Barrier(2)

        def delayed_worker(batch):
            flushes.append(dict(batch))
            try:
                barrier.wait(timeout=0.5)
            except threading.BrokenBarrierError:
                pass

        worker = CoalescingUpdateWorker(delayed_worker, batch_window_s=0.01)
        worker.start()
        worker.submit({"temp": 60.0, "seq": 1})
        worker.submit({"temp": 65.0, "seq": 2})
        worker.submit({"temp": 72.0, "seq": 3})
        try:
            barrier.wait(timeout=0.5)
        except threading.BrokenBarrierError:
            pass
        worker.stop()
        self.assertGreaterEqual(len(flushes), 1)
        self.assertEqual(flushes[-1]["seq"], 3)
        self.assertEqual(flushes[-1]["temp"], 72.0)

    def test_sensor_tracker_cadence_stretching(self):
        tracker = SensorTransitionTracker(stale_after_s=3.0, live_publish_period_s=1.0)
        tracker.collect_due(0.0)
        tracker.ingest("mq4", 50, now=0.0)
        self.assertEqual(tracker.collect_due(0.0)["mq4"], 50)
        # Ingested within live_publish_period_s (0.5s < 1.0s) -> rate-bounded
        self.assertTrue(tracker.ingest("mq4", 55, now=0.5))
        self.assertNotIn("mq4", tracker.collect_due(0.5))
        # After publish period (1.5s - 0.0s = 1.5s >= 1.0s) -> due for refresh
        self.assertTrue(tracker.ingest("mq4", 60, now=1.5))
        self.assertEqual(tracker.collect_due(1.5)["mq4"], 60)
        # Not stale yet at 4.0s (gap 2.5s < stale_after_s 3.0s)
        self.assertTrue(tracker.is_link_fresh(4.0))
        # Becomes stale after 3.0s of silence (at 4.6s, gap is 3.1s from 1.5s)
        due = tracker.collect_due(4.6)
        self.assertEqual(due.get("mq4"), -1)

    def test_health_status_under_cpu_thermal_throttling(self):
        # Bit 0: under-voltage, Bit 1: frequency capped, Bit 2: currently throttled
        throttled_flags = 0x50005
        status = build_health_status(
            cpu_temperature_c=84.5,
            throttled_flags=throttled_flags,
            ble_connected=True,
            lidar_healthy=True,
            sensor_serial_connected=True,
        )
        self.assertTrue(status["thermalWarning"])
        self.assertEqual(status["throttledFlags"], 0x50005)
        self.assertEqual(status["cpuTemperatureC"], 84.5)


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
    def test_side_obstacle_envelope_is_direction_independent(self):
        self.assertEqual(
            side_obstacle_names({"LEFT": 21.9, "RIGHT": 80.0}),
            ["LEFT"],
        )
        self.assertEqual(
            side_obstacle_names({"LEFT": 22.0, "RIGHT": 21.0}),
            ["LEFT", "RIGHT"],
        )
        self.assertEqual(side_obstacle_names({"LEFT": None, "RIGHT": 22.1}), [])

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
