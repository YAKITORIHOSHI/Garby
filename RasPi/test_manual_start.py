"""Operator starts need fresh controller evidence and never replay themselves."""

import threading
import unittest
from unittest.mock import patch

from bridge_core import ManualStartControl


class ManualStartTests(unittest.TestCase):
    def setUp(self):
        self.control = ManualStartControl()

    def ready(self, now=100):
        self.control.note_ready(now)
        self.control.note_state("IDLE", now)

    def test_ble_alone_and_partial_handshakes_cannot_start(self):
        self.assertIsNone(self.control.request(True, 100))
        self.control.note_state("IDLE", 100)
        self.assertIsNone(self.control.request(True, 100))
        self.control.reset()
        self.control.note_ready(100)
        self.assertIsNone(self.control.request(True, 100))
        self.control.note_state("IDLE", 100)
        self.assertIsNone(self.control.request(False, 100))
        self.assertTrue(self.control.status(True, 100)["enabled"])

    def test_stale_handshake_or_state_cannot_enable_start(self):
        self.ready()
        self.control.note_ready(108)
        self.assertIsNone(self.control.request(True, 108))
        self.control.note_state("IDLE", 108)
        self.assertIsNotNone(self.control.request(True, 108))
        self.control.reset()
        self.ready()
        self.control.note_state("IDLE", 108)
        self.assertIsNone(self.control.request(True, 108))

    def test_running_returning_destination_and_fault_are_not_idle(self):
        self.ready()
        for state in ("UNKNOWN", "OUTBOUND", "AT_DESTINATION", "RETURNING", "FAULT"):
            with self.subTest(state=state):
                self.control.note_state(state, 100)
                self.assertIsNone(self.control.request(True, 100))

    def test_one_click_queues_one_request_until_matching_outcome(self):
        self.ready()
        packet = self.control.request(True, 100)
        request_id = packet[7:-1]
        self.assertIsNone(self.control.request(True, 100))
        self.assertTrue(self.control.can_send(packet, True, 100.5))
        self.assertFalse(self.control.outcome("wrong-id", True))
        self.assertTrue(self.control.status(True, 101)["pending"])
        self.assertTrue(self.control.outcome(request_id, True))
        self.assertFalse(self.control.outcome(request_id, True))
        self.assertFalse(self.control.status(True, 101)["enabled"])
        self.control.note_state("OUTBOUND", 101)
        self.control.note_state("IDLE", 102)
        self.assertTrue(self.control.status(True, 102)["enabled"])
        self.assertIn("Ready:", self.control.status(True, 102)["message"])

    def test_delayed_queue_or_changed_state_cancels_instead_of_starting_later(self):
        for condition in ("expired", "disconnected", "running", "new_session"):
            with self.subTest(condition=condition):
                self.control.reset()
                self.ready()
                packet = self.control.request(True, 100)
                if condition == "running":
                    self.control.note_state("OUTBOUND", 101)
                elif condition == "new_session":
                    self.control.reset()
                    self.ready(101)
                now = 103 if condition == "expired" else 101
                self.assertFalse(self.control.can_send(packet, condition != "disconnected", now))
                self.assertFalse(self.control.status(True, now)["pending"])

    def test_ack_timeout_needs_new_status_before_another_click(self):
        self.ready()
        packet = self.control.request(True, 100)
        self.assertFalse(self.control.status(True, 105.1)["enabled"])
        self.assertFalse(self.control.status(True, 105.1)["pending"])
        self.assertFalse(self.control.can_send(packet, True, 105.1))
        self.control.note_state("OUTBOUND", 105.2)
        self.assertIsNone(self.control.request(True, 105.2))
        self.control.note_state("IDLE", 105.3)
        self.assertIsNotNone(self.control.request(True, 105.3))

    def test_rejection_is_visible_and_ids_increase_across_connection_reset(self):
        with patch("bridge_core.time.time", return_value=12345):
            self.ready()
            first = self.control.request(True, 100)
            self.control.outcome(first[7:-1], False, "NOT_READY")
            self.assertIn("NOT_READY", self.control.status(True, 100)["message"])
            self.control.reset()
            self.ready()
            second = self.control.request(True, 100)
        self.assertGreater(int(second[7:-1]), int(first[7:-1]))

    def test_simultaneous_clicks_cannot_create_two_starts(self):
        self.ready()
        barrier = threading.Barrier(3)
        packets = []

        def click():
            barrier.wait()
            packets.append(self.control.request(True, 100))

        threads = [threading.Thread(target=click) for _ in range(2)]
        for thread in threads:
            thread.start()
        barrier.wait()
        for thread in threads:
            thread.join(timeout=2)
            self.assertFalse(thread.is_alive())
        self.assertEqual(sum(packet is not None for packet in packets), 1)

    def test_only_an_attempted_start_is_queried_and_queries_are_rate_limited(self):
        self.ready()
        packet = self.control.request(True, 100)
        self.assertIsNone(self.control.next_query(True, 101))
        self.control.mark_sent(packet, 101)
        self.assertIsNone(self.control.next_query(True, 101.5))
        query = self.control.next_query(True, 102)
        self.assertEqual(query, f"[START STATUS:{packet[7:-1]}]")
        self.assertIsNone(self.control.next_query(True, 102.5))
        self.control.note_state("OUTBOUND", 102.5)
        self.assertTrue(self.control.can_query(query, True, 103))
        self.assertIsNotNone(self.control.next_query(True, 103))

    def test_stale_session_disconnected_and_expired_queries_are_not_written(self):
        for condition in ("disconnected", "reset", "expired", "acknowledged"):
            with self.subTest(condition=condition):
                self.control.reset()
                self.ready()
                packet = self.control.request(True, 100)
                self.control.mark_sent(packet, 100)
                query = self.control.next_query(True, 101)
                if condition == "reset":
                    self.control.reset()
                    self.ready(101)
                elif condition == "acknowledged":
                    self.control.outcome(packet[7:-1], True)
                now = 106 if condition == "expired" else 101
                self.assertFalse(self.control.can_query(query, condition != "disconnected", now))
                self.assertIsNone(self.control.next_query(condition != "disconnected", now))


if __name__ == "__main__":
    unittest.main()
