"""No hardware: developer request identity, deadlines, leases and sensor telemetry."""
import raspi_path  # noqa: F401  (puts the deployable RasPi/ folder on sys.path)
import unittest

from developer_support import DeveloperTestControl
import developer_support as developer


class DeveloperControlTests(unittest.TestCase):
    def test_developer_speed_settings_start_at_30_cm_per_second(self):
        minimum = 30 * developer.STEPS_PER_CM
        self.assertGreaterEqual(developer.HOLD_SLOW_HZ, minimum)
        self.assertGreater(developer.HOLD_FAST_HZ, developer.HOLD_SLOW_HZ)
        self.assertGreaterEqual(developer.RUN_SPEED_RANGE[0], minimum)
        self.assertGreaterEqual(developer.RUN_DEFAULT_SPEED, minimum)
        why, _ = DeveloperTestControl.run_program([("F", 100)], 2350, 4800)
        self.assertIn("Max speed", why)
        why, _ = DeveloperTestControl.run_program([("F", 100)], 2351, 4800)
        self.assertEqual(why, "")

    def setUp(self):
        self.now = 100.0
        self.control = DeveloperTestControl(clock=lambda: self.now, wall_clock=lambda: 2000)
        self.control.receive("[TEST READY:1]")

    def start(self, action="F", sensors=7):
        packet = self.control.request(action, sensors, True, True)
        self.assertIsNotNone(packet)
        self.assertTrue(self.control.can_send(packet, True, True))
        self.control.mark_sent(packet)
        return packet, packet.split(":")[1]

    def test_an_uncertain_start_is_never_retransmitted_and_keep_cannot_start_a_test(self):
        packet, identity = self.start()
        self.assertFalse(self.control.can_send(packet, True, True))
        keep = self.control.next_packet(True, False)
        self.assertEqual(keep, f"[TEST KEEP:{identity}]")
        self.control.mark_sent(keep)
        self.assertIsNone(self.control.next_packet(True, False))
        self.now += .21
        self.assertEqual(self.control.next_packet(True, False), keep)
        self.now += 5
        self.assertIsNone(self.control.next_packet(True, False))
        self.control.receive(f"[TEST ACK:{identity}]")
        self.assertFalse(self.control.busy())
        self.assertIn("did not confirm", self.control.status(True, True)["message"])

    def test_the_writer_rechecks_state_deadline_and_emergency_stop(self):
        packet = self.control.request("1", 0, True, True)
        self.assertFalse(self.control.can_send(packet, True, False))
        self.assertFalse(self.control.can_send(packet, True, True, stopped=True))
        self.now += 2.01
        self.assertFalse(self.control.can_send(packet, True, True))
        self.assertFalse(self.control.busy())

    def test_disconnect_never_replays_or_renews_an_old_test(self):
        packet, identity = self.start()
        self.control.receive(f"[TEST ACK:{identity}]")
        self.control.disconnect()
        self.control.receive(f"[TEST ACK:{identity}]")
        self.assertIsNone(self.control.next_packet(True, True))
        self.assertFalse(self.control.status(True, True)["enabled"])
        self.assertIn("connection changed", self.control.status(True, True)["message"])

    def test_invalid_result_numbers_cannot_be_presented_as_a_measured_turn(self):
        _, identity = self.start()
        self.control.receive(f"[TEST RESULT:{identity}:D1=9999999999:D2=-9999999999:M=1:Y=9999999999]")
        self.assertEqual(self.control.status(True, True)["result"], "")
        self.control.receive(f"[TEST RESULT:{identity}:D1=0:D2=0:M=0:Y=0]")
        self.assertIn("gyro: not measured", self.control.status(True, True)["result"])

    def test_limits_and_mpu_required_turns_are_checked_before_queuing(self):
        self.assertIsNone(self.control.request("L", 1, True, True))
        self.assertIsNone(self.control.request("R", 0, True, True))
        for action, sensors in (("GO", 7), ("F", 8), ("F", True), ("F", -1)):
            self.assertIsNone(self.control.request(action, sensors, True, True))
        packet, identity = self.start("L", 2)
        self.assertTrue(packet.endswith(":L:10:2]"))
        self.control.receive(f"[TEST FAILED:{identity}:NOT_READY]")
        self.assertFalse(self.control.busy())

    def test_90_degree_turns_are_separate_requests_and_always_require_the_mpu(self):
        for action in ("L90", "R90"):
            with self.subTest(action=action):
                self.setUp()
                self.assertIsNone(self.control.request(action, 1, True, True))
                packet, identity = self.start(action, 2)
                self.assertTrue(packet.endswith(f":{action[0]}:90:2]"))
                self.control.receive(f"[TEST ACK:{identity}]")
                self.assertIn("90", self.control.status(True, True)["message"])
                self.now += 13
                self.assertEqual(self.control.next_packet(True, False), f"[TEST KEEP:{identity}]")
                self.now += 17.51
                self.assertIsNone(self.control.next_packet(True, False))
                self.assertFalse(self.control.busy())

    def test_missing_or_unchanged_mpu_notices_explain_floor_contact_without_assuming_the_cause(self):
        for code in ("NO_ROTATION", "HELD_MPU", "MPU_DATA_LOST"):
            with self.subTest(code=code):
                self.setUp()
                _, identity = self.start("R", 2)
                self.control.receive(f"[TEST FAILED:{identity}:{code}]")
                message = self.control.status(True, True)["message"]
                self.assertIn("floor", message)
                self.assertIn("MPU", message)
        self.assertIn("connection", message)

    def test_only_matching_confirmed_outcomes_complete_a_test(self):
        _, identity = self.start()
        self.control.receive("[TEST DONE:123]")
        self.assertTrue(self.control.busy())
        self.control.receive(f"[TEST ACK:{identity}]")
        self.control.receive(f"[TEST RESULT:{identity}:D1=400:D2=400:M=1:Y=-123]")
        self.control.receive(f"[TEST DONE:{identity}]")
        status = self.control.status(True, True)
        self.assertFalse(status["pending"])
        self.assertIn("finished", status["message"])
        self.assertIn("-1.23", status["result"])
        self.assertFalse(status["enabled"])  # waits for fresh idle capability

    def test_sensor_readings_expire_independently_of_motion_permission(self):
        self.control.receive("[TEST DATA:M=0:Y=1200:W=1:H=-250]")
        sensors = self.control.status(True, True)["sensors"]
        self.assertFalse(sensors["mpu_ready"])
        self.assertTrue(sensors["walls_ready"])
        self.assertEqual(sensors["wall_deg"], -2.5)
        self.assertFalse(self.control.busy())
        self.now += 12.1
        self.assertEqual(self.control.status(True, True)["sensors"], {})
        self.assertFalse(self.control.status(True, True)["enabled"])

    def test_storage_failure_does_not_queue_a_motion_request(self):
        class Store:
            def load_id_floor(self): return 0
            def reserve_ids(self, value): raise OSError("disk unavailable")
        control = DeveloperTestControl(store=Store(), clock=lambda: self.now)
        control.receive("[TEST READY:1]")
        self.assertIsNone(control.request("F", 0, True, True))
        self.assertFalse(control.busy())

    def test_absolute_test_timeout_ends_the_lease_despite_acknowledgement(self):
        _, identity = self.start()
        self.control.receive(f"[TEST ACK:{identity}]")
        self.now += 8.51
        self.assertIsNone(self.control.next_packet(True, False))
        self.assertFalse(self.control.busy())


    def test_every_unavailable_panel_says_why(self):
        self.assertEqual(self.control.status(True, True)["blocked"], "")
        self.assertIn("not connected", self.control.status(False, True)["blocked"])
        self.assertEqual(self.control.status(True, False, blocked="Emergency stop is on.")["blocked"],
                         "Emergency stop is on.")
        self.assertIn("busy", self.control.status(True, False)["blocked"])
        fresh = DeveloperTestControl(clock=lambda: self.now)
        self.assertIn("DEVTEST firmware", fresh.status(True, True)["blocked"])
        self.now += 12.1  # one late status reply: still a reason, not a silent grey panel
        self.assertIn("confirm it is ready", self.control.status(True, True)["blocked"])
        self.control.receive("[TEST READY:1]")
        self.now += 11.0  # the bridge polls every 5 s: one missed poll keeps the panel usable
        self.assertTrue(self.control.status(True, True)["enabled"])

    def test_holds_are_explained_and_readiness_keeps_the_last_outcome(self):
        _, identity = self.start("L", 7)
        self.control.receive(f"[TEST FAILED:{identity}:HELD_LIDAR]")
        self.control.receive("[TEST READY:1]")
        status = self.control.status(True, True)
        self.assertIn("Turn left", status["message"])
        self.assertIn("Untick LiDAR", status["message"])
        self.assertTrue(status["enabled"])
        _, identity = self.start("R", 2)
        self.control.receive(f"[TEST RESULT:{identity}:D1=-1117:D2=1117:M=1:Y=0]")
        self.control.receive(f"[TEST FAILED:{identity}:NO_ROTATION]")
        status = self.control.status(True, True)
        self.assertIn("wheels lifted", status["message"])
        self.assertIn("-1117 steps", status["result"])
        self.assertIn("cm", status["result"])
        self.control.receive("[TEST READY:1]")
        _, identity = self.start("F", 7)
        self.control.receive(f"[TEST FAILED:{identity}:SOMETHING_NEW]")
        self.assertIn("something new", self.control.status(True, True)["message"])

    def test_backward_is_a_bounded_reverse_parking_check(self):
        # Owner, 2026-11-16: backward could not be seen at all with the wheels lifted.
        packet, identity = self.start("B", 1)
        self.assertTrue(packet.endswith(":B:400:1]"))
        self.control.receive(f"[TEST ACK:{identity}]")
        self.assertIn("Backward", self.control.status(True, True)["message"])
        self.control.receive(f"[TEST RESULT:{identity}:D1=-400:D2=-400:M=0:Y=0]")
        self.control.receive(f"[TEST FAILED:{identity}:HELD_REAR]")
        status = self.control.status(True, True)
        self.assertIn("behind", status["message"])
        self.assertIn("Untick LiDAR", status["message"])
        self.assertIn("-400 steps", status["result"])
        self.assertIn("-5.1 cm", status["result"])

    def test_a_held_drive_runs_only_while_the_window_says_held_and_has_no_time_limit(self):
        # Owner, 2026-11-16: arrows that drive only while held; no step value, no limit.
        packet = self.control.request_hold("W", True, True, True)
        self.assertTrue(packet.endswith(":W:4702:0]"))
        self.assertTrue(self.control.can_send(packet, True, True))
        self.control.mark_sent(packet)
        identity = packet.split(":")[1]
        self.control.receive(f"[TEST ACK:{identity}]")
        keeps = 0
        for _ in range(500):  # 20 s held; the BLE writer looks every 20 ms, this every 40
            self.now += 0.04
            self.control.still_held()
            message = self.control.next_packet(True, False)
            if message:
                self.assertEqual(message, f"[TEST KEEP:{identity}]")
                self.control.mark_sent(message)
                keeps += 1
        self.assertGreater(keeps, 150)  # about one every 0.12 s
        self.assertTrue(self.control.busy())
        self.assertIn("Drive forward", self.control.status(True, False)["message"])
        # A frozen window stops saying "held": the KEEPs stop, so the robot's lease ends it.
        self.now += 0.6
        self.assertIsNone(self.control.next_packet(True, False))
        self.control.release()
        end = self.control.next_packet(True, False)
        self.assertEqual(end, f"[TEST END:{identity}]")
        self.assertTrue(self.control.can_send(end, True, False, stopped=True))  # a stop is never held back
        self.control.mark_sent(end)
        self.now += 0.2
        self.assertIsNone(self.control.next_packet(True, False))
        self.control.receive(f"[TEST DONE:{identity}]")
        self.assertFalse(self.control.busy())

    def test_a_phone_nudge_lets_go_by_itself_after_its_time(self):
        # Owner, 2026-11-18: "Tap = short nudge". Nobody says "held"; the Pi lets go.
        packet = self.control.request_hold("S", False, True, True, pulse_s=developer.PHONE_NUDGE_S)
        self.assertTrue(packet.endswith(":S:2351:0]"))
        self.control.mark_sent(packet)
        identity = packet.split(":")[1]
        self.control.receive(f"[TEST ACK:{identity}]")
        keeps = 0
        while self.now < 100.0 + developer.PHONE_NUDGE_S - 0.05:
            self.now += 0.04
            message = self.control.next_packet(True, False)
            if message:
                self.assertEqual(message, f"[TEST KEEP:{identity}]")
                self.control.mark_sent(message)
                keeps += 1
        self.assertGreaterEqual(keeps, 4)  # kept alive without still_held()
        self.now = 100.0 + developer.PHONE_NUDGE_S + 0.01
        end = self.control.next_packet(True, False)
        self.assertEqual(end, f"[TEST END:{identity}]")
        self.assertIn("nudge over", self.control.status(True, False)["message"])
        self.control.mark_sent(end)
        self.now += 0.5
        self.assertIsNone(self.control.next_packet(True, False))  # one END, no KEEP after
        self.control.receive(f"[TEST DONE:{identity}]")
        self.assertFalse(self.control.busy())

    def test_a_nudge_cannot_be_asked_to_last_long(self):
        for pulse in (0, -1, 2 * developer.PHONE_NUDGE_S + 0.01, "0.6", float("nan")):
            self.assertIsNone(self.control.request_hold("W", False, True, True, pulse_s=pulse), pulse)

    def test_stop_ends_whatever_runs_the_fastest_way_it_can(self):
        self.assertFalse(self.control.stop("from the phone"))  # nothing running
        # A held drive or nudge: [TEST END] at once.
        packet = self.control.request_hold("W", True, True, True, pulse_s=developer.PHONE_NUDGE_S)
        self.control.mark_sent(packet)
        identity = packet.split(":")[1]
        self.assertTrue(self.control.stop("from the phone"))
        self.assertEqual(self.control.next_packet(True, False), f"[TEST END:{identity}]")
        self.control.receive(f"[TEST DONE:{identity}]")
        # A bench test: its KEEP lease ends, so the robot brakes within 800 ms.
        self.control.receive("[TEST READY:1]")
        self.start("F", 7)
        self.assertTrue(self.control.stop("from the phone"))
        self.assertFalse(self.control.busy())
        self.assertIsNone(self.control.next_packet(True, True))
        self.assertIn("stopped from the phone", self.control.status(True, True)["message"])

    def test_a_tap_released_before_sending_never_reaches_the_robot(self):
        packet = self.control.request_hold("A", False, True, True)
        self.assertTrue(packet.endswith(":A:2351:0]"))
        self.control.release()
        self.assertFalse(self.control.busy())
        self.assertFalse(self.control.can_send(packet, True, True))
        self.assertIsNone(self.control.next_packet(True, True))

    def test_held_drives_need_a_ready_idle_robot_and_a_known_arrow(self):
        self.assertIsNone(self.control.request_hold("X", True, True, True))
        self.assertIsNone(self.control.request_hold("W", True, True, False))
        self.assertIsNone(self.control.request_hold("W", True, False, True))
        _, identity = self.start("F", 7)
        self.assertIsNone(self.control.request_hold("S", False, True, True))  # one at a time
        self.control.release()  # not a held drive: nothing to release
        self.assertTrue(self.control.busy())

    def test_an_unanswered_arrow_names_the_firmware_it_needs(self):
        self.control.mark_sent(self.control.request_hold("W", False, True, True))
        self.now += 5.1
        message = self.control.status(True, True)["message"]
        self.assertIn("did not confirm", message)
        self.assertIn(developer.REMOTE_FIRMWARE, message)

    def test_a_release_the_robot_never_confirms_is_reported(self):
        packet = self.control.request_hold("D", True, True, True)
        self.control.mark_sent(packet)
        self.control.receive(f"[TEST ACK:{packet.split(':')[1]}]")
        self.control.release()
        self.control.mark_sent(self.control.next_packet(True, False))
        self.now += 5.1
        self.assertFalse(self.control.busy())
        self.assertIn("did not confirm", self.control.status(True, True)["message"])

    def test_an_unconfirmed_backward_test_names_the_firmware_it_needs(self):
        # Older boards drop the B request without an answer.
        self.start("B", 7)
        self.now += 5.1
        self.assertFalse(self.control.busy())
        message = self.control.status(True, True)["message"]
        self.assertIn("did not confirm", message)
        self.assertIn("BACKTEST-20261116", message)

if __name__ == "__main__":
    unittest.main()
