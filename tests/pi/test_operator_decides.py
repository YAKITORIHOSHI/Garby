"""No hardware: GARBY moves only when the operator decides (owner, 2026-11-16: "Why does it
start when I turn my lidar read-only, let me decide when will it move such as release stop
and dispose manually").

Field log 2026-11-16: a Manual disposal route held on the bench survived the monitor being
closed (09:03); the next monitor (09:18) found it still OUTBOUND and it drove on as soon as
the LiDAR cleared it, and again when read-only was switched on.
"""
import raspi_path  # noqa: F401  (puts the deployable RasPi/ folder on sys.path)
import unittest

import test_runtime as fixtures


class OperatorDecidesTests(unittest.TestCase):
    def setUp(self):
        self.runtime = fixtures.load_runtime()
        self.runtime.ble_connected.set()
        for event in (self.runtime.front_disabled, self.runtime.back_disabled,
                      self.runtime.side_disabled):
            self.addCleanup(event.clear)
        self.addCleanup(self.runtime.ble_connected.clear)
        self.addCleanup(self.runtime.publish_robot_state, "UNKNOWN")

    def stopped(self):
        return self.runtime.operator_control.blocks_motion()

    def pause(self):
        return self.runtime.read_operator_pause()

    def test_a_route_still_running_when_the_monitor_starts_waits_for_the_operator(self):
        with self.assertLogs("combined_app", level="WARNING") as logs:
            self.runtime.ble_notify_handler("[RUNNING]")
        self.assertTrue(self.stopped())
        self.assertIn("was still running", self.pause())
        self.assertTrue(any("Operator action requested: ESTOP" in line for line in logs.output))

    def test_a_monitor_that_starts_idle_never_stops_a_route_it_started_itself(self):
        self.runtime.ble_notify_handler("[IDLE]")
        self.runtime.ble_notify_handler("[RUNNING]")   # a Manual disposal in this session
        self.assertFalse(self.stopped())
        # A Bluetooth drop and reconnect mid-route is not a new monitor: the route goes on.
        self.runtime.publish_robot_state("UNKNOWN")
        self.runtime.ble_notify_handler("[RUNNING]")
        self.assertFalse(self.stopped())
        self.assertEqual(self.pause(), "")

    def test_changing_read_only_during_a_route_stops_it_until_the_operator_chooses(self):
        self.runtime.ble_notify_handler("[IDLE]")
        self.runtime.side_disabled.set()                 # idle: just a setting
        self.runtime.safety_checks_changed()
        self.assertFalse(self.stopped())
        self.runtime.ble_notify_handler("[RUNNING]")
        with self.assertLogs("combined_app", level="WARNING") as logs:
            self.runtime.front_disabled.set()            # during a route: stop, then decide
            self.runtime.safety_checks_changed()
        self.assertTrue(self.stopped())
        self.assertIn("Front read only", self.pause())
        self.assertTrue(any("LiDAR read-only changed: Front read only" in line for line in logs.output))
        # Already stopped: further changes are just settings; the choice is still the operator's.
        self.runtime.back_disabled.set()
        self.runtime.safety_checks_changed()
        self.assertTrue(self.stopped())
        self.runtime.request_operator_action("RELEASE")
        self.assertEqual(self.pause(), "")

    def test_the_phone_and_headless_toggles_follow_the_same_rule(self):
        self.runtime.ble_notify_handler("[IDLE]")
        self.runtime.ble_notify_handler("[RUNNING]")
        self.runtime.cloud_set_read_only("back", True)
        self.assertTrue(self.stopped())
        self.assertIn("Back read only", self.pause())

    def test_the_monitor_and_the_phone_show_why_it_stopped(self):
        self.runtime.ble_notify_handler("[RUNNING]")
        self.assertIn("was still running", self.runtime.read_operator_pause())
        self.assertIn("Continue run", self.runtime.read_operator_pause())
        self.assertIn("was still running", self.runtime.build_operator_mirror()["message"])


if __name__ == "__main__":
    unittest.main()
