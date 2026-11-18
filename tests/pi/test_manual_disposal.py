"""Manual disposal uses the real start controller and isolated cloud boundaries."""
import raspi_path  # noqa: F401  (puts the deployable RasPi/ folder on sys.path)
import copy
import queue
import time
import types
import unittest
from unittest.mock import Mock, patch

from test_runtime import load_runtime


class ManualDisposalRegressionTests(unittest.TestCase):
    def test_monitor_cancel_does_not_send_and_accept_rechecks_readiness(self):
        import monitor_ui
        gui = monitor_ui.LidarGUI.__new__(monitor_ui.LidarGUI)
        gui.root = None
        gui.request_start = Mock()
        gui.read_health = Mock(return_value={"manual_start": {"enabled": True}})
        gui._refresh_start = Mock()
        with patch.object(monitor_ui.messagebox, "askyesno", return_value=False) as confirm:
            gui._request_start()
        confirm.assert_called_once()
        gui.request_start.assert_not_called()
        gui.read_health.side_effect = [
            {"manual_start": {"enabled": True}},
            {"manual_start": {"enabled": False}},
        ]
        with patch.object(monitor_ui.messagebox, "askyesno", return_value=True):
            gui._request_start()
        gui.request_start.assert_not_called()

    def test_executor_capability_is_required_and_does_not_grant_go(self):
        runtime = load_runtime()
        runtime.ble_connected.set()
        runtime.ble_notify_handler("[SYSTEM READY]")
        runtime.ble_notify_handler("[IDLE]")
        self.assertFalse(runtime.manual_disposal_available())
        runtime.ble_notify_handler("[DISPOSAL READY:1]")
        self.assertTrue(runtime.manual_disposal_available())
        with self.assertRaises(queue.Empty):
            runtime.ble_send_queue.get_nowait()
        runtime.ble_notify_handler("[RUNNING]")
        self.assertFalse(runtime.manual_disposal_available())
        runtime.publish_robot_state("UNKNOWN")
        self.assertFalse(runtime.manual_disposal_available())


class DisposalRelayTests(unittest.TestCase):
    def setUp(self):
        from disposal_support import DisposalRelay
        self.runtime = load_runtime()
        self.runtime.ble_connected.set()
        self.runtime.ble_notify_handler("[SYSTEM READY]")
        self.runtime.ble_notify_handler("[IDLE]")
        self.runtime.ble_notify_handler("[DISPOSAL READY:1]")
        self.manager = self.runtime.FirebaseManager.__new__(self.runtime.FirebaseManager)
        self.manager._init_cloud_health()
        self.command = {"version": 1, "requestId": "a" * 32,
                        "requestedAt": int(time.time() * 1000), "requestedBy": "operator",
                        "status": "pending"}
        self.sent = []
        self.claim_failure = False
        self.before_transaction = None
        def transaction(callback):
            if self.before_transaction:
                self.before_transaction()
                self.before_transaction = None
            self.command = callback(copy.deepcopy(self.command))
            if self.claim_failure:
                self.claim_failure = False
                raise OSError("commit response lost")
            return copy.deepcopy(self.command)
        self.runtime.db = types.SimpleNamespace(reference=lambda path: types.SimpleNamespace(
            get=lambda: copy.deepcopy(self.command), transaction=transaction))
        self.relay = DisposalRelay(self.runtime.manual_start, self.sent.append,
                                   self.runtime.manual_disposal_available)

    def poll(self, **kwargs):
        self.relay.poll(self.manager, "garby-bin-01", self.runtime.ble_connected.is_set(),
                        self.runtime.operator_control.blocks_motion(), **kwargs)

    def test_claim_precedes_one_start_and_matching_ack_is_the_only_success(self):
        self.poll()
        self.assertEqual(self.command["status"], "dispatching")
        self.assertEqual(len(self.sent), 1)
        packet = self.sent[0]
        self.assertEqual(packet, f"[START:{self.command['controllerRequestId']}]")
        self.runtime.manual_start.mark_sent(packet)
        self.poll()
        self.runtime.ble_notify_handler("[START ACK:999]")
        self.poll()
        self.assertEqual(self.command["status"], "dispatching")
        self.runtime.ble_notify_handler(f"[START ACK:{packet[7:-1]}]")
        self.runtime.publish_robot_state("UNKNOWN")  # ACK survives a subsequent link loss.
        self.poll()
        self.assertEqual(self.command["status"], "accepted")
        self.poll()
        self.assertEqual(self.sent, [packet])

    def test_replaced_request_cannot_be_claimed_or_overwritten_by_an_old_outcome(self):
        original = dict(self.command)
        def replace():
            self.command["requestId"] = "b" * 32
        self.before_transaction = replace
        self.poll()
        self.assertEqual(self.sent, [])
        self.assertEqual(self.command["status"], "pending")
        self.assertFalse(self.manager.finish_disposal("test", original, "accepted"))
        self.assertEqual(self.command["requestId"], "b" * 32)

    def test_uncertain_claim_and_process_restart_query_only(self):
        from disposal_support import DisposalRelay
        self.claim_failure = True
        with self.assertRaises(OSError):
            self.poll()
        self.assertEqual(self.sent, [])
        controller_id = self.command["controllerRequestId"]
        self.runtime.manual_start.reset()
        self.relay = DisposalRelay(self.runtime.manual_start, self.sent.append, lambda: True)
        self.poll()
        query = self.runtime.manual_start.next_query(True)
        self.assertEqual(query, f"[START STATUS:{controller_id}]")
        self.assertFalse(self.runtime.manual_start.can_send(f"[START:{controller_id}]", True))
        self.runtime.ble_notify_handler(f"[START FAILED:{controller_id}:NOT_RECEIVED]")
        self.poll()
        self.assertEqual(self.command["status"], "failed")
        self.assertEqual(self.sent, [])

    def test_missing_ack_expires_as_unconfirmed_and_never_replays_start(self):
        self.poll()
        self.poll(now_ms=self.command["claimedAt"] + 15_001)
        self.assertEqual(self.command["status"], "unconfirmed")
        self.poll()
        self.assertEqual(len(self.sent), 1)

    def test_confirmed_ack_survives_a_long_cloud_outage_without_replaying(self):
        self.poll()
        packet = self.sent[0]
        self.runtime.ble_notify_handler(f"[START ACK:{packet[7:-1]}]")
        self.poll(now_ms=self.command["claimedAt"] + 60_000)
        self.assertEqual(self.command["status"], "accepted")
        self.assertEqual(self.sent, [packet])

    def test_invalid_expired_future_and_unready_requests_never_start(self):
        saved = dict(self.command)
        for update in ({"requestedAt": 0}, {"requestedAt": True}, {"requestedAt": "123"},
                       {"requestedAt": saved["requestedAt"] - 30_001},
                       {"requestedAt": saved["requestedAt"] + 5_001},
                       {"requestId": "1"}, {"requestedBy": ""}, {"version": True}):
            with self.subTest(update=update):
                self.command = {**saved, **update}
                self.poll(now_ms=saved["requestedAt"])
                self.assertEqual(self.command["status"], "failed")
        self.command = dict(saved)
        self.runtime.publish_robot_state("UNKNOWN")
        self.poll()
        self.assertEqual(self.command["status"], "failed")
        self.assertEqual(self.sent, [])

    def test_emergency_stop_blocks_pending_and_cannot_turn_uncertainty_into_success(self):
        self.runtime.request_operator_action("ESTOP")
        self.poll()
        self.assertEqual(self.command["reason"], "emergency_stop")
        self.assertEqual(self.sent, [])

    def test_slow_claim_or_state_change_prevents_late_queue_admission(self):
        self.before_transaction = lambda: self.runtime.manual_start.note_state("OUTBOUND")
        self.poll()
        self.assertEqual(self.command["status"], "failed")
        self.assertEqual(self.sent, [])

    def test_a_claim_taking_longer_than_queue_budget_never_sends(self):
        with patch.object(self.runtime.time, "monotonic", return_value=100):
            self.runtime.manual_start.note_ready()
            self.runtime.manual_start.note_state("IDLE")
            self.runtime._disposal_ready_at = 100
        def slow_claim():
            clock.return_value = 102.01
        self.before_transaction = slow_claim
        with patch.object(self.runtime.time, "monotonic", return_value=100) as clock:
            self.poll()
        self.assertEqual(self.sent, [])
        self.assertEqual(self.command["status"], "failed")

    def test_claim_after_ble_reconnect_queries_without_requeueing_start(self):
        self.poll()
        packet = self.sent[0]
        self.runtime.manual_start.mark_sent(packet)
        self.runtime.publish_robot_state("UNKNOWN")
        self.runtime.ble_connected.clear()
        self.poll()
        self.runtime.ble_connected.set()
        self.poll()
        query = self.runtime.manual_start.next_query(True)
        self.assertEqual(query, f"[START STATUS:{packet[7:-1]}]")
        self.runtime.ble_notify_handler(f"[START ACK:{packet[7:-1]}]")
        self.poll()
        self.assertEqual(self.command["status"], "accepted")
        self.assertEqual(self.sent, [packet])

    def test_claimed_stop_stays_unconfirmed_and_replaced_requester_is_untouched(self):
        self.poll()
        self.runtime.request_operator_action("ESTOP")
        self.poll()
        self.assertEqual(self.command["status"], "unconfirmed")
        original = dict(self.command)
        self.command["status"] = "pending"
        self.command["requestedBy"] = "different-operator"
        self.assertFalse(self.manager.finish_disposal("test", original, "accepted"))
        self.assertEqual(self.command["status"], "pending")

    def test_expired_or_missing_capability_and_busy_local_start_reject_cloud_intent(self):
        saved = dict(self.command)
        self.runtime._disposal_ready_at = time.monotonic() - 7.6
        self.poll()
        self.assertEqual(self.sent, [])
        self.assertEqual(self.command["status"], "failed")
        self.command = dict(saved)
        self.runtime.ble_notify_handler("[DISPOSAL READY:1]")
        local_packet = self.runtime.manual_start.request(True)
        self.assertIsNotNone(local_packet)
        self.poll()
        self.assertEqual(self.sent, [])
        self.assertEqual(self.command["status"], "failed")
        self.assertTrue(self.runtime.manual_start.can_send(local_packet, True))

    def test_terminal_result_is_immutable_even_when_callback_retries(self):
        for status in ("accepted", "failed", "unconfirmed"):
            self.command["status"] = status
            original = dict(self.command)
            self.assertFalse(self.manager.claim_disposal("test", original, "1234"))
            self.manager.finish_disposal("test", original, "failed", "late")
            self.assertEqual(self.command, original)


if __name__ == "__main__":
    unittest.main()
