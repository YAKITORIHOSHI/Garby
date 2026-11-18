"""No hardware: RunStart (owner, 2026-11-16), the Developer window's list of steps.

No LiDAR, sonar or gyro gates and no time limit by the owner's choice; the KEEP lease and the
emergency stop end a list. The executor keeps its exact place: Continue run (C) finishes it,
Reverse to start (U) takes it back, Discard ([RUN CLEAR]) forgets it.
"""
import raspi_path  # noqa: F401  (puts the deployable RasPi/ folder on sys.path)
import unittest

from developer_support import DeveloperTestControl, RUNSTART_FIRMWARE, parse_run_program


class RunStartControlTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.control = DeveloperTestControl(clock=lambda: self.now, wall_clock=lambda: 2000)
        self.control.receive("[TEST READY:1]")

    def send(self, packet):
        self.assertIsNotNone(packet)
        self.assertTrue(self.control.can_send(packet, True, True))
        self.control.mark_sent(packet)
        return packet.split(":")[1]

    def run_status(self):
        return self.control.status(True, True)["run"]

    def test_a_list_becomes_one_bounded_packet_with_its_speed_and_acceleration(self):
        packet = self.control.request_run([("F", 10000), ("L", 980), ("C", 500)], 2351, 4800, True, True)
        self.assertEqual(packet, f"[TEST:{self.control._id}:P:2351:4800:F10000,L980,C500]")
        self.assertLessEqual(len(packet), 244)

    def test_out_of_range_lists_say_why_and_send_nothing(self):
        cases = (([], 2351, 4800, "at least one step"),
                 ([("F", 0)], 2351, 4800, "Step 1"),
                 ([("F", 100001)], 2351, 4800, "Step 1"),
                 ([("X", 10)], 2351, 4800, "choose what it does"),
                 ([("F", 10)] * 21, 2000, 4800, "At most 20"),
                 ([("F", 10)], 8001, 4800, "Max speed"),
                 ([("F", 10)], 9, 4800, "Max speed"),
                 ([("F", 10)], 2351, 99, "Acceleration"),
                 ([("F", 10)], 2351, 20001, "Acceleration"))
        for steps, speed, accel, words in cases:
            self.assertIsNone(self.control.request_run(steps, speed, accel, True, True))
            self.assertIn(words, self.run_status()["message"])
        longest = [("F", 100000)] + [("C", 100000)] * 19
        packet = self.control.request_run(longest, 8000, 20000, True, True)
        self.assertTrue(packet.startswith(f"[TEST:{self.control._id}:P:8000:20000:"))  # owner: up to 8000
        self.assertLessEqual(len(packet), 244)

    def test_a_list_is_kept_alive_without_a_time_limit_and_reports_its_place(self):
        identity = self.send(self.control.request_run([("F", 40000)], 4000, 4800, True, True))
        self.control.receive(f"[TEST ACK:{identity}]")
        for _ in range(300):  # over 60 s of KEEPs
            self.now += 0.21
            keep = self.control.next_packet(True, False)
            self.assertEqual(keep, f"[TEST KEEP:{identity}]")
            self.control.mark_sent(keep)
        self.assertTrue(self.run_status()["running"])
        self.assertTrue(self.run_status()["locked"])
        self.control.receive(f"[RUN AT:{identity}:0:12000]")
        self.assertEqual(self.run_status()["progress"], {"id": identity, "step": 0, "done": 12000})
        self.control.receive(f"[TEST DONE:{identity}]")
        self.control.receive(f"[RUN SAVED:{identity}:0:40000:4000:4800:F40000]")
        status = self.run_status()
        self.assertFalse(status["running"])
        self.assertEqual(status["place"], "end")
        self.assertFalse(status["locked"])  # finished: the rows can be edited again
        self.assertIn("Reverse to start", status["message"])

    def test_an_emergency_stop_part_way_offers_continue_and_reverse_from_the_saved_place(self):
        identity = self.send(self.control.request_run([("F", 4000), ("L", 980)], 2351, 4800, True, True))
        self.control.receive(f"[TEST ACK:{identity}]")
        self.control.stopped_by_emergency()
        self.assertIsNone(self.control.next_packet(True, True, stopped=True))  # no KEEP after the stop
        self.control.receive(f"[RUN SAVED:{identity}:1:420:2351:4800:F4000,L980]")
        status = self.run_status()
        self.assertEqual(status["place"], "partway")
        self.assertTrue(status["locked"])
        self.assertEqual(status["saved"]["steps"], [("F", 4000), ("L", 980)])
        self.assertIn("Continue run", status["message"])
        # Continue while the stop is on waits for the release (done by the operator gate).
        self.control.queue_run_after_release("C")
        self.assertTrue(self.run_status()["waiting_release"])
        self.assertIsNone(self.control.next_packet(True, True, stopped=True))
        self.now += 1
        self.assertIsNone(self.control.next_packet(True, True, stopped=False))  # not ready yet
        self.control.receive("[TEST READY:1]")  # the released robot reports idle and ready
        packet = self.control.next_packet(True, True, stopped=False)
        self.assertEqual(packet, f"[TEST:{self.control._id}:C:{identity}]")
        self.assertTrue(self.run_status()["running"])

    def test_reverse_to_start_names_the_saved_list_and_an_unreleased_stop_times_out(self):
        self.control.receive("[RUN SAVED:555:1:420:2351:4800:F4000,L980]")
        packet = self.control.request_run_resume("U", True, True)
        self.assertEqual(packet, f"[TEST:{self.control._id}:U:555]")
        self.control.cancel("x")
        self.control.queue_run_after_release("U")
        self.now += 10.5
        self.assertIsNone(self.control.next_packet(True, True, stopped=True))
        self.assertFalse(self.run_status()["waiting_release"])
        self.assertIn("not released in time", self.run_status()["message"])

    def test_continue_at_the_end_or_reverse_at_the_start_does_nothing(self):
        self.control.receive("[RUN SAVED:556:0:4000:2351:4800:F4000]")
        self.assertIsNone(self.control.request_run_resume("C", True, True))
        self.control.receive("[RUN SAVED:557:0:0:2351:4800:F4000]")
        self.assertIsNone(self.control.request_run_resume("U", True, True))
        self.assertIsNotNone(self.control.request_run_resume("C", True, True))  # run it again

    def test_discard_is_sent_once_even_while_stopped_and_unlocks_on_confirmation(self):
        self.control.receive("[RUN SAVED:600:1:20:2351:4800:F100,B100]")
        self.assertTrue(self.run_status()["locked"])
        self.assertTrue(self.control.request_run_clear(True))
        packet = self.control.next_packet(True, False, stopped=True)
        self.assertEqual(packet, "[RUN CLEAR:600]")
        self.assertTrue(self.control.can_send(packet, True, False, stopped=True))
        self.control.mark_sent(packet)
        self.assertIsNone(self.control.next_packet(True, False, stopped=True))
        self.control.receive("[RUN SAVED:0]")
        self.assertIsNone(self.run_status()["saved"])
        self.assertFalse(self.run_status()["locked"])
        self.assertIn("forgotten", self.run_status()["message"])

    def test_malformed_or_impossible_saved_lists_are_ignored(self):
        for line in ("[RUN SAVED:1:5:0:2351:4800:F100]", "[RUN SAVED:1:0:101:2351:4800:F100]",
                     "[RUN SAVED:01:0:0:2351:4800:F100]", "[RUN SAVED:1:0:0:2351:4800:X100]",
                     "[RUN SAVED:1:0:0:2351:4800:F100,]"):
            self.control.receive(line)
            self.assertFalse(self.run_status()["known"], line)

    def test_an_unanswered_list_names_the_firmware_it_needs(self):
        self.send(self.control.request_run([("F", 100)], 2351, 4800, True, True))
        self.now += 5.1
        self.assertFalse(self.control.busy())
        self.assertIn(RUNSTART_FIRMWARE, self.run_status()["message"])

    def test_a_lost_connection_forgets_what_the_robot_had_saved(self):
        self.control.receive("[RUN SAVED:700:1:20:2351:4800:F100,B100]")
        self.control.disconnect()
        self.assertFalse(self.run_status()["known"])
        self.assertIsNone(self.control.request_run_resume("C", True, True))

    def test_the_phones_list_is_read_exactly_or_not_at_all(self):
        # 2026-11-18: the phone sends its rows as "F4000,L980".
        self.assertEqual(parse_run_program("F4000,L980,C100000"),
                         [("F", 4000), ("L", 980), ("C", 100000)])
        for bad in (None, "", 7, "F", "F0", "F01", "f10", "X10", "F10,", ",F10", "F10,,L2",
                    "F1000000", "F10 ", "F1.5", ",".join(["F100000"] * 21)):
            self.assertIsNone(parse_run_program(bad), bad)
        # Twenty is the window's limit; run_program refuses a 21st step with its own words.
        twenty_one = parse_run_program(",".join(["F1"] * 21))
        self.assertIn("At most 20", DeveloperTestControl.run_program(twenty_one, 2351, 4800)[0])


if __name__ == "__main__":
    unittest.main()
