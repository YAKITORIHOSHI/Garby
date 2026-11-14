"""Exercise the production Pi runtime with ROS/BLE/Firebase peripherals replaced.

No production algorithm is copied into this suite. These tests cover the
runtime wiring that helper-only tests cannot reach.
"""
import asyncio
import importlib.util
import math
from pathlib import Path
import queue
import sys
import tempfile
import threading
import time
import types
import unittest
import weakref
from unittest.mock import patch

class FakeNode:
    def __init__(self, name): pass
    def create_timer(self, *args): return None
    def create_subscription(self, *args): self.subscription_qos = args[-1]
    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=time.time_ns()))
    def get_logger(self):
        # rclpy's RcutilsLogger names: info, warning, error. Records keep the
        # severity a message would reach the console with.
        records = self.__dict__.setdefault("log_records", [])
        return types.SimpleNamespace(
            info=lambda message, **kwargs: records.append(("INFO", message)),
            warning=lambda message, **kwargs: records.append(("WARNING", message)),
            error=lambda message, **kwargs: records.append(("ERROR", message)))

def load_runtime(*, lidar_reflection="0"):
    # Existing fixtures express raw FRONT=0 / LEFT=+90 synthetic geometry.
    # Physical-mount tests pass None to exercise the actual deployment default
    # or their explicit environment, rather than hiding it behind this fixture.
    replacements = {}
    for name in ("serial", "bleak", "firebase_admin", "rclpy", "rclpy.node", "rclpy.qos", "rclpy.executors", "sensor_msgs", "sensor_msgs.msg"):
        replacements[name] = types.ModuleType(name)
    replacements["bleak"].BleakScanner = object
    replacements["bleak"].BleakClient = object
    replacements["rclpy.node"].Node = FakeNode
    replacements["rclpy.qos"].qos_profile_sensor_data = types.SimpleNamespace(depth=5)
    replacements["rclpy.executors"].SingleThreadedExecutor = object
    replacements["sensor_msgs.msg"].LaserScan = object
    spec = importlib.util.spec_from_file_location("garby_runtime_test", Path(__file__).with_name("final_w_serial.py"))
    runtime = importlib.util.module_from_spec(spec)
    # Import creates the operator controller. Tests must neither depend on the
    # robot's persisted emergency stop nor write its request-ID reservation.
    # Retain the directory for as long as this runtime fixture is in use.
    runtime._test_operator_state = tempfile.TemporaryDirectory(prefix="garby-runtime-operator-")
    weakref.finalize(runtime, runtime._test_operator_state.cleanup)
    environment = {
        "GARBY_OPERATOR_STOP_FILE": str(Path(runtime._test_operator_state.name) / "operator-stop")}
    if lidar_reflection is not None:
        environment["GARBY_LIDAR_REFLECT_LEFT_RIGHT"] = lidar_reflection
    with patch.dict(sys.modules, replacements), patch.dict("os.environ", environment):
        spec.loader.exec_module(runtime)
    return runtime

class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.runtime = load_runtime()

    @staticmethod
    def scan(ranges=None, *, stamp_ns=None):
        if stamp_ns is None:
            stamp_ns = time.time_ns()
        return types.SimpleNamespace(ranges=ranges if ranges is not None else [2.0]*360,
            angle_min=-math.pi, angle_increment=math.pi/180,
            range_min=0.1, range_max=8.0,
            header=types.SimpleNamespace(stamp=types.SimpleNamespace(
                sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)))

    def test_invalid_yaw_does_not_crash_before_logger_initialization(self):
        with patch.dict("os.environ", {"GARBY_LIDAR_YAW_OFFSET_DEG": "bad-value"}):
            self.assertEqual(load_runtime().LIDAR_YAW_OFFSET_DEG, 0.0)

    def test_runtime_fixture_isolates_persistent_operator_state(self):
        with tempfile.TemporaryDirectory() as directory:
            live_stop = Path(directory) / "operator-stop"
            live_stop.write_text("Preserve this stop\n")
            with patch.dict("os.environ", {"GARBY_OPERATOR_STOP_FILE": str(live_stop)}):
                runtime = load_runtime()
            self.assertNotEqual(runtime.operator_control.store.path, live_stop)
            self.assertFalse(runtime.operator_control.local_stop)
            runtime.operator_control.store.reserve_ids(256)
            self.assertEqual(live_stop.read_text(), "Preserve this stop\n")
            self.assertFalse(live_stop.with_name("operator-stop.request-id").exists())

    def test_moving_object_prediction_only_adds_stop_and_slow_and_follows_the_front_bypass(self):
        # 2026-11-12: a predicted contact is the generic obstacle code (O), a nearby
        # mover is caution speed (W), a mover heading for the spin circle holds a
        # turn (T=O). The operator's read-only front bypass removes them all.
        node = self.runtime.LidarDistanceReader(None, None)
        node.scan_callback(self.scan())
        assessment = self.runtime.MotionAssessment
        self.assertIn("F=C|B=C|T=C", node.build_status_packets()[0])
        node._motion = assessment(stop=True, reason="test")
        self.assertIn("F=O|B=C", node.build_status_packets()[0])
        node._motion = assessment(slow=True)
        self.assertIn("F=W|B=C", node.build_status_packets()[0])
        node._motion = assessment(turn_stop=True)
        self.assertIn("|T=O", node.build_status_packets()[0])
        self.runtime.front_disabled.set()
        try:
            for value in (assessment(stop=True), assessment(slow=True), assessment(turn_stop=True)):
                node._motion = value
                self.assertIn("F=C|B=C|T=C", node.build_status_packets()[0])
        finally:
            self.runtime.front_disabled.clear()

    def test_an_object_approaching_fast_is_stopped_before_the_static_box(self):
        node = self.runtime.LidarDistanceReader(None, None)
        last_path = ""
        distance = 4.5
        for k in range(9):
            ranges = [7.9] * 360
            distance = 4.5 - 1.5 * 0.12 * k        # 1.5 m/s toward the robot
            for ray in range(176, 185):          # ~40 cm wide straight ahead
                ranges[ray] = distance
            node.scan_callback(self.scan(ranges))
            last_path = node.build_status_packets()[0]
            time.sleep(0.12)
        self.assertGreater(distance, 3.0)        # far outside the 95 cm static box
        self.assertIn("F=O", last_path)
        self.assertIn("MOVING_STOP", node._active_zones())

    def test_drive_lines_feed_the_prediction_and_stay_out_of_the_gui_log(self):
        runtime = self.runtime
        while not runtime.ble_recv_queue.empty():
            runtime.ble_recv_queue.get_nowait()
        runtime.ble_notify_handler("[DRIVE:F:71]")
        self.assertEqual((runtime.ego_motion.snapshot().mode, runtime.ego_motion.snapshot().speed_cm_s),
                         ("F", 71.0))
        self.assertTrue(runtime.ble_recv_queue.empty())
        runtime.ble_notify_handler("[DRIVE:bogus]")
        self.assertEqual(runtime.ego_motion.snapshot().mode, "F")
        runtime.ble_notify_handler("[MCU READY]")
        self.assertIsNone(runtime.ego_motion.snapshot().mode)
        with self.assertLogs(runtime.logger, level="WARNING"):
            runtime.ble_notify_handler("[SONAR:SILENT]")

    def test_headless_scan_skips_display_but_retains_path_decisions(self):
        node = self.runtime.LidarDistanceReader(None, None)
        node.scan_callback(self.scan())
        self.assertIn("F=C|B=C", node.build_status_packets()[0])
        blocked = self.scan(); blocked.ranges[180] = .45
        node.scan_callback(blocked)
        self.assertIn("F=O", node.build_status_packets()[0])

    def test_gui_frame_contains_circular_detail_from_current_acquisition(self):
        frames = queue.Queue(maxsize=5)
        node = self.runtime.LidarDistanceReader(None, frames)
        scan = self.scan([1.0 + index / 360.0 for index in range(360)])
        node.scan_callback(scan)
        frame = frames.get_nowait()
        self.assertEqual(len(frame), 7, "GUI needs scan points and independent acquisition time")
        snapshot = frame[5]
        self.assertGreater(len(snapshot.points_m), 8)
        self.assertLessEqual(len(snapshot.points_m), 360)
        self.assertEqual(snapshot.acquired_at, node._last_scan_time)
        self.assertEqual(snapshot.yaw_offset_deg, self.runtime.LIDAR_YAW_OFFSET_DEG)
        self.assertEqual(frame[6], snapshot.acquired_at)

    def test_headless_does_not_build_map_and_display_build_runs_outside_safety_lock(self):
        headless = self.runtime.LidarDistanceReader(None, None)
        display = self.runtime.LidarDistanceReader(None, queue.Queue())
        builder = self.runtime.build_lidar_map_snapshot

        def assert_unlocked(*args, **kwargs):
            self.assertFalse(display._history_lock._is_owned())
            self.assertIn("F=C|B=C", display.build_status_packets()[0])
            return builder(*args, **kwargs)

        with patch.object(self.runtime, "build_lidar_map_snapshot", side_effect=assert_unlocked) as build:
            headless.scan_callback(self.scan())
            build.assert_not_called()
            display.scan_callback(self.scan())
            build.assert_called_once()

    def test_display_failure_does_not_change_safety_evidence(self):
        frames = queue.Queue()
        node = self.runtime.LidarDistanceReader(None, frames)
        with patch.object(self.runtime, "build_lidar_map_snapshot", side_effect=RuntimeError("display only")):
            with self.assertLogs("combined_app", level="ERROR"):
                node.scan_callback(self.scan())
        self.assertIn("F=C|B=C", node.build_status_packets()[0])
        self.assertFalse(node._lidar_is_stale())
        self.assertIsNone(frames.get_nowait()[5])

    def test_map_keeps_acquisition_age_and_rejected_scans_cannot_refresh_it(self):
        frames = queue.Queue()
        node = self.runtime.LidarDistanceReader(None, frames)
        scan = self.scan()
        with patch.object(node, "_scan_acquisition_timing", return_value=(.35, 1, 1)):
            with patch.object(self.runtime.time, "monotonic", return_value=100):
                node.scan_callback(scan)
        snapshot = frames.get_nowait()[5]
        self.assertAlmostEqual(snapshot.acquired_at, 99.65)
        with patch.object(node, "_scan_acquisition_timing", return_value=None):
            node.scan_callback(scan)
        self.assertTrue(frames.empty())
        self.assertEqual(snapshot.acquired_at, 99.65)
        self.assertIn("F=S|B=S", node.build_status_packets()[0])

    def test_snapshot_keeps_its_deadline_when_evidence_expires_during_construction(self):
        node = self.runtime.LidarDistanceReader(None, None)
        now = [100.0]
        with patch.object(node, "_scan_acquisition_timing", return_value=(0, 1, 1)), \
                patch.object(self.runtime.time, "monotonic", side_effect=lambda: now[0]):
            node.scan_callback(self.scan())
            build = node._build_status_packets_locked
            def delayed_build():
                packets = build()
                now[0] += self.runtime.LIDAR_STALE_TIMEOUT_S + .01
                return packets
            with patch.object(node, "_build_status_packets_locked", side_effect=delayed_build):
                path, sides, deadline = node.build_status_packets_with_deadline()
        self.assertIn("F=C|B=C", path)
        self.assertIn("G=0", sides)
        self.assertAlmostEqual(deadline, 100.0 + self.runtime.LIDAR_STALE_TIMEOUT_S)

    def test_display_geometry_failure_keeps_acquisition_age_in_frame(self):
        frames = queue.Queue()
        node = self.runtime.LidarDistanceReader(None, frames)
        with patch.object(node, "_scan_acquisition_timing", return_value=(.35, 1, 1)), \
                patch.object(self.runtime.time, "monotonic", return_value=100), \
                patch.object(self.runtime, "build_lidar_map_snapshot", return_value=None):
            node.scan_callback(self.scan())
        frame = frames.get_nowait()
        self.assertIsNone(frame[5])
        self.assertEqual(len(frame), 7, "A missing map cannot reset summary freshness to arrival time")
        self.assertAlmostEqual(frame[6], 99.65)

    def test_map_never_reuses_previous_finite_returns_for_missing_or_invalid_scan(self):
        frames = queue.Queue()
        node = self.runtime.LidarDistanceReader(None, frames)
        # Distinct acquisitions must have distinct source stamps even on a
        # coarse/frozen host clock. Duplicate-source rejection is tested below.
        stamp = time.time_ns()
        node.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=stamp + 2))
        node.scan_callback(self.scan(stamp_ns=stamp))
        self.assertEqual(len(frames.get_nowait()[5].points_m), 360)
        node.scan_callback(self.scan([math.inf] * 360, stamp_ns=stamp + 1))
        self.assertEqual(frames.get_nowait()[5].points_m, ())
        self.assertIn("F=C|B=C", node.build_status_packets()[0])
        invalid = self.scan(stamp_ns=stamp + 2)
        invalid.angle_increment = 0
        node.scan_callback(invalid)
        self.assertIsNone(frames.get_nowait()[5])
        self.assertIn("F=S|B=S", node.build_status_packets()[0])

    def test_summary_cannot_refresh_old_distance_when_current_sector_is_missing(self):
        frames = queue.Queue(maxsize=5)
        node = self.runtime.LidarDistanceReader(None, frames)
        node.scan_callback(self.scan())
        self.assertIsNotNone(frames.get_nowait()[0]["FRONT"])
        partial = self.scan()
        for index in range(158, 203):
            partial.ranges[index] = math.nan
        node.scan_callback(partial)
        current = frames.get_nowait()
        self.assertIsNone(current[0]["FRONT"], "Missing current rays cannot relabel a historical range as fresh")
        self.assertIsNotNone(current[0]["BACK"])
        self.assertIn("F=S|B=C", node.build_status_packets()[0])

    def test_read_only_controls_bypass_selected_thresholds_without_changing_readings(self):
        self.assertTrue(self.runtime.ALLOW_RUNTIME_SAFETY_DISABLE)
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        scan = self.scan()
        for index in (0, 45, 135, 180, 225, 315):
            scan.ranges[index] = .15
        node.scan_callback(scan)
        self.assertIn("F=O|B=O", node.build_status_packets()[0])
        self.runtime.front_disabled.set()
        self.assertIn("F=C|B=O", node.build_status_packets()[0])
        self.runtime.back_disabled.set()
        self.assertIn("F=C|B=C", node.build_status_packets()[0])
        for side in node.FRONT_GROUP | node.BACK_GROUP:
            self.assertEqual(node._quick_min(side), 15)
        self.runtime.front_disabled.clear()
        self.assertIn("F=O|B=C", node.build_status_packets()[0])
        self.runtime.front_disabled.set()
        scan = self.scan(list(scan.ranges))
        scan.ranges[90] = .15
        node.scan_callback(scan)
        self.assertIn("F=O|B=O", node.build_status_packets()[0])

    def test_group_toggles_cannot_clear_missing_or_stale_lidar(self):
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        self.runtime.front_disabled.set()
        self.runtime.back_disabled.set()
        self.assertIn("F=S|B=S", node.build_status_packets()[0])
        scan = self.scan()
        for index in range(158, 203):
            scan.ranges[index] = math.nan
        node.scan_callback(scan)
        self.assertIn("F=S|B=C", node.build_status_packets()[0])
        node._last_scan_time = 0
        self.assertIn("F=S|B=S", node.build_status_packets()[0])

    def test_manual_start_uses_end_to_end_readiness_without_sensor_values(self):
        runtime = self.runtime
        runtime.ble_connected.set()
        self.assertFalse(runtime.request_manual_start()["enabled"])
        with self.assertRaises(queue.Empty):
            runtime.ble_send_queue.get_nowait()
        runtime.ble_notify_handler("[SYSTEM READY]")
        runtime.ble_notify_handler("[IDLE]")
        self.assertTrue(runtime.manual_start.status(True)["enabled"])
        self.assertTrue(all(value is None for value in runtime.latest_sensor_readings.values()))
        runtime.front_disabled.set()
        runtime.back_disabled.set()
        runtime.operator_control.notification("[OP STATUS:1:ESTOP=0:SONAR=0]")
        self.assertIs(runtime.operator_control.status(True)["ultrasonic_enabled"], False)
        result = runtime.request_manual_start()
        self.assertTrue(result["pending"])
        packet = runtime.ble_send_queue.get_nowait()
        self.assertTrue(packet.startswith("[START:"))
        runtime.request_manual_start()
        with self.assertRaises(queue.Empty):
            runtime.ble_send_queue.get_nowait()
        runtime.ble_notify_handler(f"[START ACK:{packet[7:-1]}]")
        runtime.ble_notify_handler("[RUNNING]")
        self.assertFalse(runtime.manual_start.status(True)["pending"])
        self.assertEqual(runtime._robot_state, "OUTBOUND")

    def test_controller_reboot_and_ble_loss_cancel_manual_start_requests(self):
        for reset in ("controller", "BLE"):
            with self.subTest(reset=reset):
                runtime = load_runtime()
                runtime.ble_connected.set()
                runtime.ble_notify_handler("[SYSTEM READY]")
                runtime.ble_notify_handler("[IDLE]")
                runtime.request_manual_start()
                if reset == "controller":
                    runtime.ble_notify_handler("[MCU READY]")
                else:
                    service = runtime.BleService(lambda *args: None, None)
                    service._client = client = object()
                    service._connecting = True
                    service._handle_disconnect(client)
                self.assertFalse(runtime.manual_start.status(True)["enabled"])
                self.assertFalse(runtime.manual_start.status(True)["pending"])
                with self.assertRaises(queue.Empty):
                    runtime.ble_send_queue.get_nowait()

    def test_accepted_start_shows_the_executor_motion_hold_in_the_monitor(self):
        import monitor_ui

        class Surface:
            def __init__(self, *args, **options):
                self.options = options

        with patch.object(monitor_ui, "LidarGUI", Surface):
            gui = self.runtime.init_gui()(None, queue.Queue(), queue.Queue())
        runtime = self.runtime
        runtime.ble_connected.set()
        runtime.ble_notify_handler("[SYSTEM READY]")
        runtime.ble_notify_handler("[IDLE]")
        runtime.request_manual_start()
        packet = runtime.ble_send_queue.get_nowait()
        runtime.ble_notify_handler(f"[START ACK:{packet[7:-1]}]")
        runtime.ble_notify_handler("[RUNNING]")
        runtime.ble_notify_handler("[MOTION:STOP:MPU]")
        status = gui.options["read_health"]()["manual_start"]
        self.assertIn("MPU", status["message"])
        self.assertFalse(status["enabled"])
        runtime.ble_notify_handler("[MOTION:CLEAR]")
        self.assertNotIn("MPU", gui.options["read_health"]()["manual_start"]["message"])

    def test_runtime_gui_wires_buttons_to_real_control_events(self):
        try:
            import monitor_ui
        except ImportError:
            self.skipTest("Tkinter is not installed")

        class Surface:
            def __init__(self, *args, **options):
                self.options = options

        with patch.object(monitor_ui, "LidarGUI", Surface):
            gui = self.runtime.init_gui()(None, queue.Queue(), queue.Queue())
        self.assertTrue(gui.options["allow_safety_disable"])
        self.assertIs(gui.options["front_disabled"], self.runtime.front_disabled)
        self.assertIs(gui.options["back_disabled"], self.runtime.back_disabled)
        gui.options["on_checks_changed"]()
        self.assertTrue(self.runtime._status_requested.is_set())
        self.assertIs(gui.options["request_start"], self.runtime.request_manual_start)
        self.assertFalse(gui.options["read_health"]()["manual_start"]["enabled"])

    def test_headless_and_display_modes_produce_identical_safety_packets(self):
        display = self.runtime.LidarDistanceReader(None, queue.Queue(maxsize=5))
        headless = self.runtime.LidarDistanceReader(None, None)
        # G is the scan's age when each snapshot is built, so two nodes polled a
        # millisecond apart differ there by design and nowhere else.
        def without_age(packets):
            return tuple("|".join(part for part in packet.split("|") if not part.startswith("G="))
                         for packet in packets)
        for index in (None, 180, 270, 0, None, None, None):
            scan = self.scan()
            if index is not None: scan.ranges[index] = .15
            display.scan_callback(scan); headless.scan_callback(scan)
            self.assertEqual(without_age(display.build_status_packets()),
                             without_age(headless.build_status_packets()))
        missing = self.scan([math.nan] * 360)
        display.scan_callback(missing); headless.scan_callback(missing)
        self.assertEqual(without_age(display.build_status_packets()),
                         without_age(headless.build_status_packets()))

    def test_idle_cloud_polling_slows_but_destination_polling_stays_fast(self):
        for ready, expected in ((False, 10.0), (True, 2.0)):
            waits = []
            runtime = load_runtime()
            if ready: runtime._ready_to_return.set()
            runtime.firebase = types.SimpleNamespace(get_reset_command=lambda device: {})
            class Stop:
                def is_set(self): return bool(waits)
                def wait(self, delay): waits.append(delay)
            runtime.firebase_reset_command_loop(Stop())
            self.assertEqual(waits, [expected])

    def test_no_first_scan_cannot_produce_clearance(self):
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        self.assertIn("F=S|B=S", node.build_status_packets()[0])
        self.assertEqual(node.subscription_qos.depth, 1)

    def test_obstacle_clears_after_fresh_scans(self):
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        node.scan_callback(self.scan())
        self.assertIn("F=C|B=C", node.build_status_packets()[0])
        blocked = self.scan(); blocked.ranges[180] = 0.45
        node.scan_callback(blocked)
        self.assertIn("F=O", node.build_status_packets()[0])
        for _ in range(3): node.scan_callback(self.scan())
        self.assertIn("F=C|B=C", node.build_status_packets()[0])

    def test_no_scan_yet_is_reported_as_waiting_and_still_holds_stop(self):
        # 2026-11-12 field log: an ERROR "stream stale" 0.15 s after start, nine
        # seconds before the driver had even been launched. No scan is not a
        # lost stream; the fail-closed STOP must be unchanged either way.
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        node.log_records.clear()  # the node's own "started" line
        self.runtime.ble_send_queue.clear()
        node._watchdog_callback()
        self.assertIn("F=S|B=S", self.runtime.ble_send_queue.get_nowait())
        self.assertEqual([level for level, _ in node.log_records], ["INFO"])
        self.assertIn("Waiting for the first LiDAR scan", node.log_records[0][1])
        node._watchdog_callback()  # announced once per outage
        self.assertEqual(len(node.log_records), 1)
        node.scan_callback(self.scan())
        node.log_records.clear()
        node._last_scan_time = 0
        node._watchdog_callback()
        self.assertEqual([level for level, _ in node.log_records], ["ERROR"])
        self.assertIn("stale", node.log_records[0][1])

    def test_missing_side_sector_fails_closed(self):
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        scan = self.scan()
        for i in range(247, 293): scan.ranges[i] = math.nan
        node.scan_callback(scan)
        self.assertIn("F=S|B=S", node.build_status_packets()[0])
        self.assertTrue(self.runtime.lidar_streaming.is_set())
        self.assertFalse(self.runtime.lidar_healthy.is_set())
        node._last_scan_time = 0
        node._watchdog_callback()
        self.assertFalse(self.runtime.lidar_streaming.is_set())

    def test_quality_loss_cannot_urgently_clear_an_existing_front_obstacle(self):
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        scan = self.scan(); scan.ranges[180] = 0.45
        node.scan_callback(scan)
        self.runtime.ble_send_queue.clear()
        scan = self.scan(list(scan.ranges))
        for i in list(range(0, 23)) + list(range(338, 360)):
            scan.ranges[i] = math.nan  # rear sector disappears
        node.scan_callback(scan)
        self.assertIn("F=O|B=S", self.runtime.ble_send_queue.get_nowait())

    def test_new_front_obstacle_cannot_urgently_clear_an_invalid_rear(self):
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        scan = self.scan()
        for i in list(range(0, 23)) + list(range(338, 360)):
            scan.ranges[i] = math.nan
        node.scan_callback(scan)
        self.runtime.ble_send_queue.clear()
        scan = self.scan(list(scan.ranges))
        scan.ranges[180] = 0.45
        node.scan_callback(scan)
        self.assertIn("F=O|B=S", self.runtime.ble_send_queue.get_nowait())

    def test_partial_scan_quality_loss_is_urgent_in_each_direction(self):
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        scan = self.scan()
        for i in list(range(0, 23)) + list(range(338, 360)):
            scan.ranges[i] = math.nan
        node.scan_callback(scan)
        self.runtime.ble_send_queue.clear()
        scan = self.scan(list(scan.ranges))
        for i in range(158, 203): scan.ranges[i] = math.nan
        node.scan_callback(scan)
        try: packet = self.runtime.ble_send_queue.get_nowait()
        except queue.Empty: self.fail("Losing the remaining usable direction must queue an urgent STOP")
        self.assertIn("F=S|B=S", packet)

    def test_side_wall_estimate_requires_current_finite_narrow_returns(self):
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        node.scan_callback(self.scan())
        scan = self.scan()
        for i in range(259, 282): scan.ranges[i] = math.inf
        node.scan_callback(scan)
        path, sides = node.build_status_packets()
        self.assertIn("F=C|B=C", path)
        self.assertNotIn("|L=", sides)
        self.assertIn("|R=200.0", sides)

    def test_invalid_sector_cannot_publish_its_old_steering_clearance(self):
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        node.scan_callback(self.scan())
        scan = self.scan()
        for i in range(158, 203): scan.ranges[i] = math.nan
        node.scan_callback(scan)
        path, sides = node.build_status_packets()
        self.assertIn("F=S|B=C", path)
        self.assertNotIn("|F=", sides)

    def test_recovered_side_wall_does_not_mix_pre_dropout_estimates(self):
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        for _ in range(3): node.scan_callback(self.scan())
        missing = self.scan()
        for i in range(248, 293): missing.ranges[i] = math.nan
        node.scan_callback(missing)
        recovered = self.scan()
        for i in range(248, 293): recovered.ranges[i] = 1.2
        node.scan_callback(recovered)
        self.assertIn("|L=120.0", node.build_status_packets()[1])

    def test_recovered_scan_does_not_mix_pre_outage_wall_history(self):
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        with patch.object(self.runtime.time, "monotonic", return_value=100.0):
            for _ in range(3): node.scan_callback(self.scan())
        with patch.object(self.runtime.time, "monotonic", return_value=101.0):
            node.scan_callback(self.scan([1.2] * 360))
            self.assertIn("|L=120.0", node.build_status_packets()[1])

    def test_invalid_geometry_does_not_kill_ros_callback(self):
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        scan = self.scan(); scan.angle_increment = math.nan
        node.scan_callback(scan)
        self.assertIn("F=S|B=S", node.build_status_packets()[0])

    def test_invalid_range_limits_are_not_normalized_into_clearance(self):
        for minimum, maximum in ((math.nan,8.0),(0.1,math.inf),(0.1,math.nan),(0.1,0.0),(-1.0,8.0)):
            with self.subTest(minimum=minimum, maximum=maximum):
                node=self.runtime.LidarDistanceReader(None, queue.Queue())
                scan=self.scan();scan.range_min=minimum;scan.range_max=maximum
                node.scan_callback(scan)
                self.assertIn("F=S|B=S",node.build_status_packets()[0])

    def test_zero_range_samples_are_unavailable_even_with_zero_minimum(self):
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        scan = self.scan([0.0] * 360); scan.range_min = 0.0
        node.scan_callback(scan)
        self.assertIn("F=S|B=S", node.build_status_packets()[0])

    def test_slow_cadence_does_not_extend_permission_into_outage(self):
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        with patch.object(self.runtime.time, "monotonic", return_value=99.0): node.scan_callback(self.scan())
        with patch.object(self.runtime.time, "monotonic", return_value=100.0): node.scan_callback(self.scan())
        with patch.object(self.runtime.time, "monotonic", return_value=100.81):
            self.assertIn("F=S|B=S", node.build_status_packets()[0])

    def test_delayed_scan_cannot_refresh_clearance_or_health(self):
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        node.scan_callback(self.scan())
        self.runtime.ble_send_queue.clear()
        scan = self.scan(); scan.header.stamp.sec = 99; scan.header.stamp.nanosec = 0
        node.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=100_000_000_000))
        node.scan_callback(scan)
        self.assertIn("F=S|B=S", node.build_status_packets()[0])
        self.assertFalse(self.runtime.lidar_healthy.is_set())
        self.assertIn("F=S|B=S", self.runtime.ble_send_queue.get_nowait())

    def test_acquisition_age_counts_toward_stale_budget_after_receipt(self):
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        scan = self.scan(); scan.header.stamp.sec = 99; scan.header.stamp.nanosec = 400_000_000
        node.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=100_000_000_000))
        with patch.object(self.runtime.time, "monotonic", return_value=100.0):
            node.scan_callback(scan)
            self.assertIn("F=C|B=C", node.build_status_packets()[0])
        with patch.object(self.runtime.time, "monotonic", return_value=100.21):
            self.assertIn("F=S|B=S", node.build_status_packets()[0])

    def test_frozen_ros_clock_cannot_refresh_a_repeated_acquisition(self):
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        scan = self.scan(); scan.header.stamp.sec = 100; scan.header.stamp.nanosec = 0
        node.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=100_000_000_000))
        with patch.object(self.runtime.time, "monotonic", return_value=100.0):
            node.scan_callback(scan)
            self.assertIn("F=C|B=C", node.build_status_packets()[0])
        for now in (100.1, 102.0, 104.0):
            with self.subTest(now=now), patch.object(self.runtime.time, "monotonic", return_value=now):
                node.scan_callback(scan)
                self.assertIn("F=S|B=S", node.build_status_packets()[0])
                self.assertFalse(self.runtime.lidar_healthy.is_set())

    def test_old_source_stamps_do_not_rebase_and_current_scan_recovers(self):
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        ros_ns = [100_300_000_000]
        node.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=ros_ns[0]))
        def deliver(stamp_ns, monotonic):
            scan = self.scan()
            scan.header.stamp.sec, scan.header.stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
            with patch.object(self.runtime.time, "monotonic", return_value=monotonic):
                node.scan_callback(scan)
                return node.build_status_packets()[0]
        self.assertIn("F=C|B=C", deliver(100_200_000_000, 100.0))
        ros_ns[0] = 100_400_000_000
        self.assertIn("F=S|B=S", deliver(100_100_000_000, 100.1))
        self.assertIn("F=S|B=S", deliver(100_200_000_000, 100.2))
        ros_ns[0] = 100_600_000_000
        self.assertIn("F=C|B=C", deliver(100_500_000_000, 100.3))

    def test_backward_ros_clock_cannot_authorize_an_advancing_future_stamp(self):
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        ros_ns = [100_000_000_000]
        node.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=ros_ns[0]))
        scan = self.scan(); scan.header.stamp.sec = 100; scan.header.stamp.nanosec = 0
        node.scan_callback(scan)
        ros_ns[0] = 99_990_000_000
        scan = self.scan(); scan.header.stamp.sec = 100; scan.header.stamp.nanosec = 10_000_000
        node.scan_callback(scan)
        self.assertIn("F=S|B=S", node.build_status_packets()[0])

    def test_missing_zero_future_or_malformed_stamp_fails_closed(self):
        for stamp in (None, (0, 0), (102, 0), (100, -1), (100, 1_000_000_000)):
            with self.subTest(stamp=stamp):
                node = self.runtime.LidarDistanceReader(None, queue.Queue())
                node.get_clock = lambda: types.SimpleNamespace(now=lambda: types.SimpleNamespace(nanoseconds=100_000_000_000))
                scan = self.scan()
                if stamp is None: del scan.header
                else: scan.header.stamp.sec, scan.header.stamp.nanosec = stamp
                node.scan_callback(scan)
                self.assertIn("F=S|B=S", node.build_status_packets()[0])

    def test_old_clear_cannot_replace_new_urgent_stop(self):
        mailbox = self.runtime.CoalescingBleQueue()
        mailbox.put_nowait("P:11|F=O|B=C", urgent=True)
        mailbox.put_nowait("P:10|F=C|B=C")
        self.assertEqual(mailbox.get_nowait(), "P:11|F=O|B=C")
        with self.assertRaises(queue.Empty): mailbox.get_nowait()

    def test_mailbox_handles_sequence_wrap(self):
        mailbox = self.runtime.CoalescingBleQueue()
        mailbox.put_nowait("P:4294967295|F=C|B=C")
        mailbox.put_nowait("P:0|F=O|B=C", urgent=True)
        self.assertEqual(mailbox.get_nowait(), "P:0|F=O|B=C")

    def test_idle_without_matching_return_id_is_not_completion(self):
        self.runtime.ble_notify_handler("[IDLE]")
        self.assertEqual(self.runtime._reset_feedback, {})
        self.runtime.ble_notify_handler("[RESET DONE:12345]")
        self.runtime.ble_notify_handler("[RESET ACK:12345]")
        self.assertEqual(self.runtime._reset_feedback, {12345: "DONE"})

    def test_status_polls_refresh_control_without_filling_gui_log(self):
        for _ in range(1000): self.runtime.ble_notify_handler("[REQUEST-STATUS]")
        self.assertTrue(self.runtime._status_requested.is_set())
        self.assertTrue(self.runtime.ble_recv_queue.empty())
        self.runtime.ble_notify_handler("[ROUTE FAULT]")
        self.assertEqual(self.runtime.ble_recv_queue.get_nowait(), "[ROUTE FAULT]")

    def test_status_request_burst_preserves_path_but_bounds_steering_traffic(self):
        runtime = self.runtime
        stop = threading.Event()
        times = iter((10.0, 10.01, 10.02, 10.03, 10.30))
        clock = [0.0]
        packets = []
        requests = types.SimpleNamespace(
            wait=lambda timeout: clock.__setitem__(0, next(times)) or True,
            clear=lambda: None,
        )
        snapshots = [0]

        def snapshot():
            snapshots[0] += 1
            if snapshots[0] == 5:
                stop.set()
            return (f"P:{snapshots[0]}|F=O|B=C", f"S:{snapshots[0]}|STABLE")

        runtime.ble_connected.set()
        with patch.object(runtime, "_status_requested", requests), \
             patch.object(runtime, "ble_send_queue", types.SimpleNamespace(
                 put_nowait=lambda packet, **kwargs: packets.append(packet))), \
             patch.object(runtime.time, "monotonic", side_effect=lambda: clock[0]):
            runtime.on_demand_status_responder(types.SimpleNamespace(
                build_status_packets=snapshot), stop)
        self.assertEqual([p for p in packets if p.startswith("P:")],
                         [f"P:{i}|F=O|B=C" for i in range(1, 6)])
        self.assertEqual([p for p in packets if p.startswith("S:")],
                         ["S:1|STABLE", "S:5|STABLE"])
        self.assertEqual(sum(p.startswith("SENSOR:") for p in packets), 1)

    def test_gui_log_retains_only_recent_notification_lines(self):
        # Only the Tk text surface is replaced; the production GUI log pump
        # processes a real queue and produces the retained visible text.
        class TextSurface:
            def __init__(self): self.lines = [f"old-{i}\n" for i in range(300)]
            def configure(self, **kwargs): pass
            def insert(self, position, text): self.lines.extend(text.splitlines(keepends=True))
            def see(self, position): pass
            def index(self, position): return f"{len(self.lines)+1}.0"
            def delete(self, start, end): del self.lines[:int(end.split(".")[0])-1]
            def yview(self): return (0, 1)
        try:
            import tkinter
        except ImportError:
            self.skipTest("Tkinter is not installed")
        gui_type = self.runtime.init_gui()
        gui = gui_type.__new__(gui_type)
        gui.root = types.SimpleNamespace(after=lambda *args: None)
        gui.recv_queue = queue.Queue()
        gui.log_text = TextSurface()
        gui.recv_queue.put_nowait("[ROUTE FAULT]")
        gui._process_ble_recv()
        self.assertLessEqual(len(gui.log_text.lines), 250)
        self.assertIn("[ROUTE FAULT]", gui.log_text.lines[-1])
        self.assertNotIn("old-0\n", gui.log_text.lines)

    def test_destination_notifies_firebase_readiness(self):
        states=[]
        self.runtime.firebase = types.SimpleNamespace(update_robot_state=states.append)
        self.runtime.ble_notify_handler("[OUTBOUND COMPLETE]")
        self.assertTrue(self.runtime._ready_to_return.is_set())
        self.runtime.ble_notify_handler("[RETURNING]")
        self.assertFalse(self.runtime._ready_to_return.is_set())
        self.assertEqual(states, ["AT_DESTINATION", "RETURNING"])

    def test_old_disconnect_cannot_remove_current_client(self):
        service = self.runtime.BleService(lambda *args: None, None)
        service._client = object()
        service._handle_disconnect(object())
        self.assertIsNotNone(service._client)

    def test_explicit_missing_lidar_port_does_not_fall_back_to_sensor_board(self):
        with patch.dict("os.environ", {"GARBY_LIDAR_PORT": "/missing-lidar"}), patch.object(self.runtime.os.path, "exists", return_value=False):
            self.assertIsNone(self.runtime.find_lidar_port())

    def test_configured_lidar_cannot_share_sensor_serial_device(self):
        self.runtime.SERIAL_PORT="/same-device"
        with patch.dict("os.environ",{"GARBY_LIDAR_PORT":"/same-device"}):
            with self.assertRaisesRegex(RuntimeError,"same serial device"):
                self.runtime.find_lidar_port()

    def test_automatic_lidar_selection_excludes_sensor_serial_device(self):
        self.runtime.SERIAL_PORT="/dev/ttyUSB1"
        self.runtime.serial_ports=self.runtime.PortDetector(lister=lambda:{},identify=lambda path:(path,))
        with patch.dict("os.environ",{},clear=True),patch.object(self.runtime.glob,"glob",return_value=[]),patch.object(self.runtime.os.path,"exists",side_effect=lambda path:path=="/dev/ttyUSB1"):
            self.assertIsNone(self.runtime.find_lidar_port())

    def test_weight_unavailability_clears_firebase_freshness(self):
        manager=self.runtime.FirebaseManager.__new__(self.runtime.FirebaseManager)
        writes=[];manager._update=lambda values:writes.append(values)
        manager.update_mcu_load_cell(-1)
        update=writes[0]
        self.assertEqual(update["devices/garby-bin-01/sensors/weight/value"],-1)
        self.assertEqual(update["devices/garby-bin-01/sensors/weight/updatedAt"],0)
        self.assertEqual(update["RASPI/VALUES/LOAD_CELL/updatedAt"],0)

    def test_detected_ports_are_used_when_nothing_is_configured(self):
        calls=[]
        def find(role,exclude=()):
            calls.append((role,exclude))
            return {"lidar":"/dev/ttyUSB1","sensor":"/dev/ttyUSB0"}[role]
        self.runtime.SERIAL_PORT=None
        self.runtime.serial_ports=types.SimpleNamespace(find=find)
        with patch.dict('os.environ',{},clear=True):
            self.assertEqual(self.runtime.find_lidar_port(),"/dev/ttyUSB1")
            self.assertEqual(self.runtime.find_sensor_port(),"/dev/ttyUSB0")
        self.assertEqual([role for role,_ in calls],["lidar","sensor"])

    def test_explicit_ports_win_and_are_never_probed(self):
        def find(*args,**kwargs): self.fail("an explicit port must not be probed")
        self.runtime.serial_ports=types.SimpleNamespace(find=find)
        self.runtime.SERIAL_PORT="/dev/ttyAMA0"
        with patch.dict('os.environ',{"GARBY_LIDAR_PORT":"/dev/ttyUSB3"}), \
             patch.object(self.runtime.os.path,'exists',return_value=True):
            self.assertEqual(self.runtime.find_lidar_port(),"/dev/ttyUSB3")
            self.assertEqual(self.runtime.find_sensor_port(),"/dev/ttyAMA0")

    def test_explicit_port_of_one_device_is_kept_out_of_the_other_search(self):
        seen={}
        def find(role,exclude=()):
            seen[role]=[path for path in exclude if path]
        self.runtime.serial_ports=types.SimpleNamespace(find=find)
        self.runtime.SERIAL_PORT="/dev/ttyAMA0"
        with patch.dict('os.environ',{},clear=True):
            self.runtime.find_lidar_port()
        self.runtime.SERIAL_PORT=None
        with patch.dict('os.environ',{"GARBY_LIDAR_PORT":"/dev/ttyUSB3"}):
            self.runtime.find_sensor_port()
        self.assertEqual(seen,{"lidar":["/dev/ttyAMA0"],"sensor":["/dev/ttyUSB3"]})

    def test_no_detected_lidar_stops_driver_start_with_a_clear_reason(self):
        with patch.object(self.runtime,"find_lidar_port",return_value=None):
            with self.assertRaisesRegex(RuntimeError,"streaming LiDAR"):
                self.runtime.start_lidar_driver()

    def test_sensor_reader_opens_the_detected_port_and_reads_it(self):
        opened=[];ingested=[]
        class Stop:
            def __init__(self): self.count=0
            def is_set(self): return self.count>=1
            def wait(self,_delay=None): self.count+=1
        stop=Stop()
        class FakeSerial:
            def __init__(inner,**kwargs): opened.append(kwargs["port"])
            def reset_input_buffer(inner): pass
            def readline(inner):
                stop.count+=1
                return b"MQ4:123\n"
            def close(inner): pass
        self.runtime.serial.Serial=FakeSerial
        self.runtime.serial.SerialException=OSError
        self.runtime.SERIAL_PORT=None
        self.runtime.serial_ports=types.SimpleNamespace(find=lambda role,exclude=():"/dev/ttyUSB4")
        original=self.runtime.sensor_tracker.ingest
        self.runtime.sensor_tracker.ingest=lambda key,value,now=None:(ingested.append((key,value)),original(key,value,now=now))[1]
        with patch.dict('os.environ',{},clear=True):
            self.runtime.receiveSerial(stop)
        self.assertEqual(opened,["/dev/ttyUSB4"])
        self.assertEqual(ingested,[("mq4",123.0)])

    def test_missing_sensor_board_is_reported_unavailable_and_retried(self):
        class Stop:
            def __init__(self): self.count=0
            def is_set(self): return self.count>=2
            def wait(self,_delay=None): self.count+=1
        searches=[]
        def find(role,exclude=()):
            searches.append(role)
        self.runtime.SERIAL_PORT=None
        self.runtime.serial_ports=types.SimpleNamespace(find=find)
        self.runtime.sensor_serial_connected.set()
        with patch.dict('os.environ',{},clear=True), self.assertLogs("combined_app",level="WARNING") as logs:
            self.runtime.receiveSerial(Stop())
        self.assertEqual(searches,["sensor","sensor"])
        self.assertFalse(self.runtime.sensor_serial_connected.is_set())
        self.assertTrue(any("no serial port is sending sensor lines" in line for line in logs.output))

    def test_weight_unavailable_notification_does_not_authorize_motion(self):
        values=[]
        self.runtime.firebase=types.SimpleNamespace(update_mcu_load_cell=values.append)
        self.runtime.ble_notify_handler("LOAD_CELL:-1")
        self.assertEqual(values,[-1])
        self.assertFalse(self.runtime._ready_to_return.is_set())
        with self.assertRaises(queue.Empty): self.runtime.ble_send_queue.get_nowait()

    def test_old_reset_completion_cannot_overwrite_new_request(self):
        current = {"requestedAt": 20, "status": "pending", "requestedBy": "operator"}
        ref = types.SimpleNamespace(transaction=lambda callback: callback(dict(current)))
        manager = self.runtime.FirebaseManager.__new__(self.runtime.FirebaseManager)
        manager._init_cloud_health()
        self.runtime.db = types.SimpleNamespace(reference=lambda path: ref)
        self.assertFalse(manager.mark_reset_done("test", 10))
        self.assertTrue(manager.mark_reset_done("test", 20))

    def test_return_may_finish_after_acceptance_window(self):
        runtime=self.runtime
        now=[1000.0]
        requested=int(now[0]*1000)
        command={"requestedAt":requested,"status":"pending"}
        outcomes=[]
        class Firebase:
            def get_reset_command(self, device): return dict(command)
            def mark_reset_ack(self, device, request): command["status"]="ack"; return True
            def mark_reset_done(self, device, request): outcomes.append(("done",request)); command["status"]="done"; return True
            def mark_reset_failed(self, device, request, reason): outcomes.append((reason,request)); command["status"]="failed"; return True
        class Stop:
            count=0
            def is_set(self): return self.count>=4
            def wait(self, delay):
                self.count+=1; now[0]+=70
                runtime._reset_feedback[requested]="ACK" if self.count<3 else "DONE"
        runtime.firebase=Firebase(); runtime.ble_connected.set()
        with patch.object(runtime.time,"time",side_effect=lambda:now[0]), patch.object(runtime.time,"monotonic",side_effect=lambda:now[0]):
            runtime.firebase_reset_command_loop(Stop())
        self.assertEqual(outcomes,[("done",requested)])

    def test_lost_completion_is_recovered_by_idempotent_outcome_query(self):
        runtime=self.runtime
        now=[1000.0]; requested=int(now[0]*1000); polls=[]; outcomes=[]
        runtime._reset_feedback[requested]="ACK"
        runtime.ble_connected.set()
        class Firebase:
            def get_reset_command(self, device): return {"requestedAt":requested,"status":"ack"}
            def mark_reset_done(self, device, request): outcomes.append(request); return True
            def mark_reset_failed(self, *args): raise AssertionError("valid return must not fail")
        class Stop:
            count=0
            def is_set(self): return self.count>=3
            def wait(self, delay):
                self.count+=1; now[0]+=10
                try: polls.append(runtime.ble_send_queue.get_nowait())
                except queue.Empty: pass
                if len(polls)>=2: runtime.ble_notify_handler(f"[RESET DONE:{requested}]")
        runtime.firebase=Firebase()
        with patch.object(runtime.time,"time",side_effect=lambda:now[0]), patch.object(runtime.time,"monotonic",side_effect=lambda:now[0]):
            runtime.firebase_reset_command_loop(Stop())
        self.assertEqual(polls,[f"[RESET:{requested}]",f"[RESET:{requested}]"])
        self.assertEqual(outcomes,[requested])

class StatusStreamTests(unittest.TestCase):
    """The status responder feeds the bridge's path watchdog; it must be bounded and survive."""
    class Node:
        def __init__(self): self.seq = 0
        def build_status_packets(self):
            self.seq += 1
            return f"P:{self.seq}|F=C|B=C", f"S:{self.seq}|STABLE"

    def setUp(self):
        self.runtime = load_runtime()
        self.runtime.ble_connected.set()
        self.stop = threading.Event()
        self.sent = []
        original = self.runtime.ble_send_queue.put_nowait
        def record(msg, urgent=False, expires_at=None):
            self.sent.append(msg)
            original(msg, urgent=urgent, expires_at=expires_at)
        self.runtime.ble_send_queue.put_nowait = record

    def run_responder(self, node, seconds, flood=False):
        worker = threading.Thread(
            target=self.runtime.on_demand_status_responder, args=(node, self.stop), daemon=True)
        worker.start()
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if flood:
                self.runtime._status_requested.set()
            time.sleep(0.01)
        alive = worker.is_alive()
        self.stop.set(); worker.join(timeout=2)
        return alive

    def paths(self):
        return [m for m in self.sent if m.startswith("P:")]

    def test_request_flood_cannot_exceed_the_scan_rate_budget(self):
        self.run_responder(self.Node(), 1.0, flood=True)
        # 100 Hz of requests would otherwise each cost an acknowledged GATT write.
        self.assertLessEqual(len(self.paths()), 8)
        self.assertGreaterEqual(len(self.paths()), 4)

    def test_unrequested_stream_keeps_the_watchdog_fed(self):
        self.run_responder(self.Node(), 0.9)
        self.assertGreaterEqual(len(self.paths()), 3)

    def test_path_and_steering_queue_the_same_acquisition_deadline(self):
        deadline = time.monotonic() + self.runtime.LIDAR_STALE_TIMEOUT_S
        class Snapshot(self.Node):
            def build_status_packets_with_deadline(inner):
                path, sides = inner.build_status_packets()
                return path, sides, deadline
        self.run_responder(Snapshot(), .35)
        entries = []
        while True:
            try:
                entries.append(self.runtime.ble_send_queue.get_entry_nowait())
            except queue.Empty:
                break
        evidence = {message[0]: expiry for message, expiry in entries if message.startswith(("P:", "S:"))}
        self.assertEqual(evidence, {"P": deadline, "S": deadline})

    def test_unexpected_exception_cannot_silently_end_the_status_stream(self):
        class Flaky(self.Node):
            def build_status_packets(inner):
                inner.seq += 1
                if inner.seq == 1:
                    raise RuntimeError("boom")
                return f"P:{inner.seq}|F=C|B=C", f"S:{inner.seq}|STABLE"
        with self.assertLogs("combined_app", level="ERROR"):
            alive = self.run_responder(Flaky(), 0.7)
        self.assertTrue(alive)
        self.assertTrue(self.paths())


class ScanCallbackGuardTests(unittest.TestCase):
    def test_malformed_metadata_clears_health_and_queues_stop_immediately(self):
        runtime = load_runtime()
        node = runtime.LidarDistanceReader(None, None)
        node.scan_callback(RuntimeTests.scan())
        self.assertTrue(runtime.lidar_healthy.is_set())
        self.assertTrue(runtime.lidar_streaming.is_set())
        runtime.ble_send_queue.clear()
        malformed = RuntimeTests.scan()
        malformed.range_min = None
        with self.assertLogs("combined_app", level="ERROR"):
            node.scan_callback(malformed)
        self.assertFalse(runtime.lidar_healthy.is_set())
        self.assertFalse(runtime.lidar_streaming.is_set())
        self.assertIn("F=S|B=S", runtime.ble_send_queue.get_nowait())
        self.assertFalse(any(node._sector_data_valid.values()))
        self.assertTrue(all(not history for history in node.steering_history.values()))
        # The watchdog coalesces this unavailable interval, and a later good
        # acquisition restores normal processing without retaining error data.
        node._watchdog_callback()
        with self.assertRaises(queue.Empty):
            runtime.ble_send_queue.get_nowait()
        node.scan_callback(RuntimeTests.scan())
        self.assertTrue(runtime.lidar_healthy.is_set())
        self.assertTrue(runtime.lidar_streaming.is_set())
        self.assertIn("F=C|B=C", node.build_status_packets()[0])
        with self.assertRaises(queue.Empty):
            runtime.ble_send_queue.get_nowait()

    def test_processing_exception_fails_closed_and_keeps_the_executor_alive(self):
        runtime = load_runtime()
        node = runtime.LidarDistanceReader(None, queue.Queue(maxsize=5))
        node.scan_callback(RuntimeTests.scan())
        self.assertFalse(node._lidar_is_stale())
        with patch.object(runtime, "partition_lidar_samples", side_effect=RuntimeError("boom")):
            with self.assertLogs("combined_app", level="ERROR"):
                node.scan_callback(RuntimeTests.scan())
        self.assertTrue(node._lidar_is_stale())
        self.assertIn("F=S", node.build_status_packets()[0])


class PiHealthTests(unittest.TestCase):
    def setUp(self):
        self.runtime = load_runtime()

    def hwmon(self, directory, alarm="0", name="rpi_volt"):
        sensor = Path(directory) / "hwmon2"
        sensor.mkdir()
        (sensor / "name").write_text(name + "\n")
        (sensor / "in0_lcrit_alarm").write_text(alarm + "\n")
        return str(Path(directory) / "hwmon*")

    def test_undervoltage_is_visible_without_video_group_access(self):
        with tempfile.TemporaryDirectory() as directory:
            pattern = self.hwmon(directory, alarm="1")
            with patch.object(self.runtime, "HWMON_GLOB", pattern), \
                    patch.object(self.runtime.os, "access", return_value=False):
                self.assertEqual(self.runtime._read_throttled_flags() & 0x1, 0x1)

    def test_healthy_supply_reports_zero_not_unknown(self):
        with tempfile.TemporaryDirectory() as directory:
            pattern = self.hwmon(directory, alarm="0")
            with patch.object(self.runtime, "HWMON_GLOB", pattern), \
                    patch.object(self.runtime.os, "access", return_value=False):
                self.assertEqual(self.runtime._read_throttled_flags(), 0)

    def test_no_voltage_source_stays_unknown(self):
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(self.runtime, "HWMON_GLOB", str(Path(directory) / "hwmon*")), \
                    patch.object(self.runtime.os, "access", return_value=False):
                self.assertIsNone(self.runtime._read_throttled_flags())

    def test_firmware_flags_are_preferred_when_vcio_is_accessible(self):
        result = types.SimpleNamespace(stdout="throttled=0x50005\n")
        with patch.object(self.runtime.os, "access", return_value=True), \
                patch.object(self.runtime.subprocess, "run", return_value=result):
            self.assertEqual(self.runtime._read_throttled_flags(), 0x50005)

    def test_throttling_is_logged_without_firebase_and_only_on_change(self):
        flags = iter([0, 0x1, 0x1, 0])
        class Stop:
            def __init__(self): self.count = 0
            def is_set(self): return self.count >= 4
            def wait(self, _delay): self.count += 1
        self.runtime.firebase = None
        with patch.object(self.runtime, "_read_throttled_flags", side_effect=lambda: next(flags)), \
                patch.object(self.runtime, "_read_cpu_temperature_c", return_value=50.0), \
                self.assertLogs("combined_app", level="INFO") as logs:
            self.runtime.raspi_heartbeat_loop(Stop())
        changes = [line for line in logs.output if "Pi health" in line]
        self.assertEqual(len(changes), 2, changes)
        self.assertIn("undervoltage-now", changes[0])

    def test_cloud_heartbeat_keeps_its_slower_period(self):
        class Firebase:
            def __init__(self): self.beats = 0
            def update_raspi_heartbeat(self, status): self.beats += 1
        class Stop:
            def __init__(self): self.count = 0
            def is_set(self): return self.count >= 5
            def wait(self, _delay): self.count += 1
        self.runtime.firebase = Firebase()
        with patch.object(self.runtime, "_read_throttled_flags", return_value=0), \
                patch.object(self.runtime, "_read_cpu_temperature_c", return_value=50.0):
            self.runtime.raspi_heartbeat_loop(Stop())
        self.assertEqual(self.runtime.firebase.beats, 1)


class BridgeDiagnosticTests(unittest.TestCase):
    def test_bridge_diagnostics_reach_the_pi_log(self):
        runtime = load_runtime()
        with self.assertLogs("combined_app", level="INFO") as logs:
            runtime.ble_notify_handler("[BRIDGE DIAG fw=1 reset=BROWNOUT drops=2 last_drop=0x208]")
            runtime.ble_notify_handler("[BRIDGE BOOT:BROWNOUT]")
        self.assertTrue(any("last_drop=0x208" in line for line in logs.output))
        self.assertTrue(any("BRIDGE BOOT:BROWNOUT" in line for line in logs.output))


class DriverOwnershipTests(unittest.TestCase):
    def setUp(self):
        self.runtime=load_runtime()
        self.process=types.SimpleNamespace(pid=12345,returncode=None,poll=lambda:None)
        self.stop=types.SimpleNamespace(wait=lambda delay:False,is_set=lambda:False)

    def test_startup_owns_only_its_child_and_never_runs_global_cleanup(self):
        node=types.SimpleNamespace(count_publishers=lambda topic:1)
        with patch.object(self.runtime,"find_lidar_port",return_value="/fake-lidar"),patch.object(self.runtime.subprocess,"Popen",return_value=self.process) as launch,patch.object(self.runtime.subprocess,"run") as command:
            self.assertIs(self.runtime.start_lidar_driver(self.stop,node),self.process)
        command.assert_not_called()
        self.assertEqual(launch.call_args.kwargs["start_new_session"],self.runtime.os.name=="posix")

    def launch_command(self, environment):
        node=types.SimpleNamespace(count_publishers=lambda topic:1)
        with patch.object(self.runtime,"find_lidar_port",return_value="/fake-lidar"),patch.object(self.runtime.subprocess,"Popen",return_value=self.process) as launch,patch.dict('os.environ',environment):
            if "GARBY_LIDAR_FIXED_RESOLUTION" not in environment:
                self.runtime.os.environ.pop("GARBY_LIDAR_FIXED_RESOLUTION",None)
            self.runtime.start_lidar_driver(self.stop,node)
        return launch.call_args.args[0]

    def test_x3_driver_keeps_every_measured_point_of_each_revolution(self):
        # 2026-11-12 bench: the SDK sized fixed resolution at 660 bins (startup
        # mean rounded down to tens) while the X3 sent 666-667 points per turn,
        # so "Real points 666 > fixed points 660" dropped the surplus from every
        # scan and left 1-degree gaps beside the seam. Each point keeps its angle.
        command=self.launch_command({})
        self.assertIn("fixed_resolution:=false",command)
        self.assertNotIn("fixed_resolution:=true",command)
        for value,expected in (("1","true"),("true","true"),("0","false"),("junk","false")):
            with self.subTest(value=value):
                self.assertIn(f"fixed_resolution:={expected}",
                              self.launch_command({"GARBY_LIDAR_FIXED_RESOLUTION":value}))

    class TwoWaits:
        """Stop the supervisor at its second wait, whatever the path."""
        def __init__(self): self.waits=0
        def is_set(self): return self.waits>=2
        def wait(self,delay=None):
            self.waits+=1
            return self.is_set()

    def test_supervisor_tells_the_map_why_the_lidar_is_unavailable(self):
        phases=[]
        def no_port(stop_event,node):
            phases.append(self.runtime.lidar_driver_status.snapshot()["phase"])
            raise RuntimeError("No serial port is streaming LiDAR data; check its USB cable and power")
        with patch.object(self.runtime,"start_lidar_driver",side_effect=no_port),self.assertLogs("combined_app",level="ERROR"):
            self.runtime.LidarDriverSupervisor(self.TwoWaits()).run()
        self.assertEqual(phases[0],"searching")
        status=self.runtime.lidar_driver_status.snapshot()
        self.assertEqual(status["phase"],"failed")
        self.assertIn("No serial port is streaming LiDAR data",status["message"])

    def test_supervisor_reports_running_then_a_driver_exit(self):
        seen=[]
        process=types.SimpleNamespace(returncode=3,poll=lambda:3)
        def started(stop_event,node):
            return process
        original=self.runtime.lidar_driver_status.set
        def record(phase,message):
            seen.append(phase)
            original(phase,message)
        with patch.object(self.runtime,"start_lidar_driver",side_effect=started),patch.object(self.runtime,"_terminate_process"),patch.object(self.runtime.lidar_driver_status,"set",side_effect=record),self.assertLogs("combined_app",level="ERROR"):
            self.runtime.LidarDriverSupervisor(self.TwoWaits()).run()
        self.assertEqual(seen,["searching","running","failed"])
        self.assertIn("exited (code 3)",self.runtime.lidar_driver_status.snapshot()["message"])

    def test_publisher_exception_cleans_up_the_new_child(self):
        def fails(topic): raise RuntimeError("DDS failure")
        node=types.SimpleNamespace(count_publishers=fails)
        with patch.object(self.runtime,"find_lidar_port",return_value="/fake-lidar"),patch.object(self.runtime.subprocess,"Popen",return_value=self.process),patch.object(self.runtime,"_terminate_process") as terminate:
            with self.assertRaisesRegex(RuntimeError,"DDS failure"):
                self.runtime.start_lidar_driver(self.stop,node)
        terminate.assert_called_once_with(self.process)

    def test_cancelled_startup_cleans_up_the_new_child(self):
        self.stop.is_set=lambda:True
        with patch.object(self.runtime,"find_lidar_port",return_value="/fake-lidar"),patch.object(self.runtime.subprocess,"Popen",return_value=self.process),patch.object(self.runtime,"_terminate_process") as terminate:
            with self.assertRaisesRegex(RuntimeError,"cancelled"):
                self.runtime.start_lidar_driver(self.stop,None)
        terminate.assert_called_once_with(self.process)

    def test_early_driver_exit_is_reported_and_cleaned_up(self):
        self.process.poll=lambda:1;self.process.returncode=1
        with patch.object(self.runtime,"find_lidar_port",return_value="/fake-lidar"),patch.object(self.runtime.subprocess,"Popen",return_value=self.process),patch.object(self.runtime,"_terminate_process") as terminate:
            with self.assertRaisesRegex(RuntimeError,"exited early"):
                self.runtime.start_lidar_driver(self.stop,None)
        terminate.assert_called_once_with(self.process)

    def test_parent_exit_does_not_leave_an_owned_driver_child(self):
        process=types.SimpleNamespace(garby_process_group=12345,poll=lambda:0,wait=lambda timeout:0)
        with patch.object(self.runtime.os,"killpg",create=True) as kill,patch.object(self.runtime.signal,"SIGKILL",9,create=True):
            self.runtime._terminate_process(process)
            self.assertEqual(kill.call_args_list[0].args,(12345,self.runtime.signal.SIGTERM))
            self.assertEqual(kill.call_args_list[1].args,(12345,self.runtime.signal.SIGKILL))


class BleWriteTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self): self.runtime=load_runtime()

    async def test_all_writes_are_serialized_and_return_is_acknowledged(self):
        active=0; maximum=0; writes=[]
        async def write(characteristic,payload,response):
            nonlocal active,maximum
            active+=1; maximum=max(maximum,active)
            await asyncio.sleep(0)
            writes.append((payload,response));active-=1
        client=types.SimpleNamespace(is_connected=True,write_gatt_char=write)
        service=self.runtime.BleService(lambda *args:None,None)
        service._client=client; service._write_char=object();service._write_lock=asyncio.Lock()
        self.runtime.ble_connected.set()
        self.runtime.ble_notify_handler("[SYSTEM READY]")
        self.runtime.ble_notify_handler("[IDLE]")
        packet = self.runtime.manual_start.request(True)
        await asyncio.gather(service._write_message("P:1|F=C|B=C"),
                             service._write_message("[RESET:12345]"), service._write_message(packet))
        self.assertEqual(maximum,1)
        self.assertIn((packet.encode(), True), writes)
        self.assertTrue(all(response for _,response in writes))
        with self.assertRaises(ValueError): await service._write_message("SENSOR:US=2\nGO\nGO")

    async def test_start_expiring_behind_another_write_is_never_delivered(self):
        runtime = self.runtime
        writes = []

        async def write(*args, **kwargs):
            writes.append(args)

        service = runtime.BleService(lambda *args: None, None)
        service._client = types.SimpleNamespace(is_connected=True, write_gatt_char=write)
        service._write_char = object()
        service._write_lock = asyncio.Lock()
        runtime.ble_connected.set()
        runtime.manual_start.note_ready(100)
        runtime.manual_start.note_state("IDLE", 100)
        packet = runtime.manual_start.request(True, 100)
        with patch("bridge_core.time.monotonic", return_value=103):
            await service._write_message(packet)
        self.assertEqual(writes, [])
        self.assertFalse(runtime.manual_start.status(True, 103)["pending"])

    async def test_a_second_queued_copy_cannot_repeat_a_start_attempt(self):
        runtime = self.runtime
        writes = []

        async def write(characteristic, payload, **kwargs):
            writes.append(payload)

        service = runtime.BleService(lambda *args: None, None)
        service._client = types.SimpleNamespace(is_connected=True, write_gatt_char=write)
        service._write_char = object()
        service._write_lock = asyncio.Lock()
        runtime.ble_connected.set()
        runtime.ble_notify_handler("[SYSTEM READY]")
        runtime.ble_notify_handler("[IDLE]")
        packet = runtime.manual_start.request(True)
        await service._write_message(packet)
        await service._write_message(packet)
        self.assertEqual(writes, [packet.encode()])
        self.assertTrue(runtime.manual_start.status(True)["pending"])

    async def test_failed_start_write_is_not_automatically_retried(self):
        runtime = self.runtime
        service = runtime.BleService(lambda *args: None, None)
        service._stop_event = asyncio.Event()
        service._write_lock = asyncio.Lock()
        service._write_char = object()
        writes = []

        async def fail(characteristic, payload, **kwargs):
            writes.append(payload)
            if payload == b"[OP STATUS]":
                return
            service._stop_event.set()
            raise TimeoutError("uncertain write")

        service._client = client = types.SimpleNamespace(is_connected=True, write_gatt_char=fail)
        runtime.ble_connected.set()
        runtime.ble_notify_handler("[SYSTEM READY]")
        runtime.ble_notify_handler("[IDLE]")
        runtime.request_manual_start()
        await service._drain_loop(client)
        self.assertEqual(sum(packet.startswith(b"[START:") for packet in writes), 1)
        with self.assertRaises(queue.Empty):
            runtime.ble_send_queue.get_nowait()

    async def test_lost_start_ack_is_recovered_by_a_query_without_replaying_start(self):
        runtime = self.runtime
        service = runtime.BleService(lambda *args: None, runtime.ble_notify_handler)
        service._stop_event = asyncio.Event()
        service._write_lock = asyncio.Lock()
        service._write_char = object()
        runtime.ble_connected.set()
        runtime.ble_notify_handler("[SYSTEM READY]")
        runtime.ble_notify_handler("[IDLE]")
        runtime.request_manual_start()
        writes = []

        async def receive(characteristic, payload, **kwargs):
            writes.append(payload.decode())
            if payload.startswith(b"[START STATUS:"):
                request_id = payload.decode()[14:-1]
                runtime.ble_notify_handler(f"[START ACK:{request_id}]")
                runtime.ble_notify_handler("[RUNNING]")
                service._stop_event.set()

        service._client = client = types.SimpleNamespace(is_connected=True, write_gatt_char=receive)
        # Allow the real service loop to tick, without waiting on BLE hardware.
        try:
            await asyncio.wait_for(service._drain_loop(client), timeout=1.5)
        except asyncio.TimeoutError:
            self.fail("A lost acknowledgement left the start pending without querying the executor")
        self.assertEqual(sum(message.startswith("[START:") for message in writes), 1)
        self.assertTrue(any(message.startswith("[START STATUS:") for message in writes))
        self.assertFalse(runtime.manual_start.status(True)["pending"])
        self.assertEqual(runtime._robot_state, "OUTBOUND")

    async def test_uncertain_start_write_queries_the_result_instead_of_resending(self):
        runtime = self.runtime
        service = runtime.BleService(lambda *args: None, runtime.ble_notify_handler)
        service._stop_event = asyncio.Event()
        service._write_lock = asyncio.Lock()
        service._write_char = object()
        writes = []

        async def receive(characteristic, payload, **kwargs):
            writes.append(payload.decode())
            if payload.startswith(b"[START:"):
                # Delivery is uncertain; a timeout cannot establish that the
                # executor did not consume it, so never send START again.
                runtime.ble_notify_handler("[RUNNING]")
                raise TimeoutError("acknowledged write result lost")
            if payload.startswith(b"[START STATUS:"):
                runtime.ble_notify_handler(f"[START ACK:{payload.decode()[14:-1]}]")
                service._stop_event.set()

        service._client = client = types.SimpleNamespace(is_connected=True, write_gatt_char=receive)
        runtime.ble_connected.set()
        runtime.ble_notify_handler("[SYSTEM READY]")
        runtime.ble_notify_handler("[IDLE]")
        runtime.request_manual_start()
        await asyncio.wait_for(service._drain_loop(client), timeout=1.5)
        self.assertEqual(sum(message.startswith("[START:") for message in writes), 1)
        self.assertTrue(any(message.startswith("[START STATUS:") for message in writes))
        self.assertFalse(runtime.manual_start.status(True)["pending"])
        self.assertEqual(runtime._robot_state, "OUTBOUND")

    async def test_failed_urgent_write_never_reinserts_old_sequence(self):
        service=self.runtime.BleService(lambda *args:None,None)
        self.runtime.ble_send_queue.put_nowait("P:5|F=O|B=C",urgent=True)
        await service._urgent_write("P:4|F=C|B=C")
        self.assertEqual(self.runtime.ble_send_queue.get_nowait(),"P:5|F=O|B=C")
        self.assertTrue(self.runtime._status_requested.is_set())

    async def test_steering_write_respects_negotiated_no_response_limit(self):
        writes=[]
        async def write(characteristic,payload,response): writes.append(response)
        service=self.runtime.BleService(lambda *args:None,None)
        service._client=types.SimpleNamespace(is_connected=True,write_gatt_char=write)
        service._write_lock=asyncio.Lock()
        steering="S:1|L=1.10|R=1.80|F=2.50|B=2.50"
        service._write_char=types.SimpleNamespace(max_write_without_response_size=20)
        await service._write_message(steering)
        service._write_char=types.SimpleNamespace(max_write_without_response_size=244)
        await service._write_message(steering)
        self.assertEqual(writes,[True,False])

if __name__ == "__main__": unittest.main()
