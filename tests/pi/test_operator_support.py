"""Operator actions must remain stopped on stale, lost or mismatched evidence."""
import raspi_path  # noqa: F401  (puts the deployable RasPi/ folder on sys.path)
import asyncio
from pathlib import Path
import tempfile
import types
import unittest
from unittest.mock import AsyncMock, patch

from operator_support import OperatorControl, StopLatchStore, reboot_pi
from test_runtime import load_runtime


class OperatorControlTests(unittest.TestCase):
    def setUp(self):
        self.control = OperatorControl()

    def ready(self, *, stopped=False, sonar=True, now=100):
        self.control.note_state("IDLE", now)
        self.control.notification(f"[OP STATUS:1:ESTOP={int(stopped)}:SONAR={int(sonar)}]", now)

    def acknowledge(self, packet, now=100):
        _, request_id, action = packet[1:-1].split(":")
        self.control.mark_sent(packet, now)
        return self.control.notification(f"[OP ACK:{request_id}:{action}]", now)

    def test_unknown_disconnected_running_and_stale_cannot_change_sonar(self):
        self.assertIsNone(self.control.request("SONAR_OFF", True, 100))
        self.ready()
        self.assertIsNone(self.control.request("SONAR_OFF", False, 100))
        self.control.note_state("OUTBOUND", 100)
        self.assertIsNone(self.control.request("SONAR_OFF", True, 100))
        self.control.note_state("IDLE", 100)
        self.assertIsNone(self.control.request("SONAR_OFF", True, 108))

    def test_emergency_stop_latches_offline_and_is_first_after_reconnection(self):
        packet = self.control.request("ESTOP", False, 100)
        self.assertTrue(self.control.blocks_motion())
        self.assertIsNone(self.control.next_packet(False, 100))
        self.control.disconnect()
        packet = self.control.next_packet(True, 101)
        self.assertTrue(packet.endswith(":ESTOP]"))
        self.assertTrue(self.control.can_send(packet, True, 101))
        self.acknowledge(packet, 101)
        self.assertEqual(self.control.next_packet(True, 101), "[OP STATUS]")
        self.ready(stopped=True, now=101)
        self.assertTrue(self.control.status(True, 101)["emergency_stopped"])

    def test_uncertain_stop_is_repeatable_but_release_and_reboot_never_are(self):
        packet = self.control.request("ESTOP", True, 100)
        self.control.mark_sent(packet, 100)
        self.assertEqual(self.control.next_packet(True, 101), packet)
        self.acknowledge(packet)
        self.ready(stopped=True)
        for action in ("RELEASE", "REBOOT"):
            packet = self.control.request(action, True, 100)
            self.assertTrue(self.control.can_send(packet, True, 100))
            self.control.mark_sent(packet, 100)
            self.assertFalse(self.control.can_send(packet, True, 100.1))
            self.assertNotEqual(self.control.next_packet(True, 101), packet)
            self.control.disconnect()
            self.ready(stopped=True)

    def test_only_matching_attempted_ack_changes_displayed_setting(self):
        self.ready()
        packet = self.control.request("SONAR_OFF", True, 100)
        request_id = packet.split(":")[1]
        ack = f"[OP ACK:{request_id}:SONAR_OFF]"
        self.control.notification(ack, 100)
        self.assertTrue(self.control.status(True, 100)["ultrasonic_enabled"])
        self.control.mark_sent(packet, 100)
        self.control.notification(f"[OP ACK:{request_id}:SONAR_ON]", 100)
        self.control.notification("[OP ACK:123:SONAR_OFF]", 100)
        self.assertTrue(self.control.status(True, 100)["pending"])
        self.control.notification(ack, 100)
        self.ready(sonar=False)
        self.assertFalse(self.control.status(True, 100)["ultrasonic_enabled"])

    def test_expired_or_disconnected_commands_cannot_be_written_later(self):
        for action in ("SONAR_OFF", "RELEASE", "REBOOT"):
            self.control = OperatorControl()
            self.ready(stopped=True)
            packet = self.control.request(action, True, 100)
            self.assertFalse(self.control.can_send(packet, True, 103))
            self.assertFalse(self.control.status(True, 103)["pending"])
            packet = self.control.request(action, True, 103)
            self.control.disconnect()
            self.ready(stopped=True, now=103)
            self.assertFalse(self.control.can_send(packet, True, 103))

    def test_reboot_requires_confirmed_stop_and_matching_ack_once(self):
        self.ready()
        self.assertIsNone(self.control.request("REBOOT", True, 100))
        self.ready(stopped=True)
        packet = self.control.request("REBOOT", True, 100)
        self.assertFalse(self.control.reboot_allowed())
        self.assertEqual(self.acknowledge(packet), "REBOOT")
        self.assertTrue(self.control.reboot_allowed())
        self.assertIsNone(self.acknowledge(packet))
        self.control.disconnect()
        self.assertTrue(self.control.blocks_motion())
        self.control.reboot_result("Permission denied")
        self.assertFalse(self.control.reboot_allowed())
        self.assertIn("Permission denied", self.control.status(False, 100)["message"])
        stop = self.control.next_packet(True, 101)
        self.acknowledge(stop, 101)
        self.ready(stopped=True, now=101)
        self.assertIn("Permission denied", self.control.status(True, 101)["message"])
        # Over ssh (owner, 2026-11-16) systemd refuses; the message says how to allow it.
        self.control.reboot_result("Call to Reboot failed: Interactive authentication required.")
        message = self.control.status(True, 101)["message"]
        self.assertIn("ssh", message)
        self.assertIn("49-garby-reboot.rules", message)
        self.assertIn("STOP remains latched", message)

    def test_new_stop_cancels_pending_release_and_reboot(self):
        for action in ("RELEASE", "REBOOT"):
            self.control = OperatorControl()
            self.ready(stopped=True)
            packet = self.control.request(action, True, 100)
            self.control.mark_sent(packet, 100)
            self.control.request("ESTOP", True, 100)
            self.assertIsNone(self.acknowledge(packet))
            self.assertTrue(self.control.blocks_motion())
            self.assertFalse(self.control.reboot_allowed())

    def test_delayed_ack_cannot_reboot_or_release_after_timeout(self):
        for action in ("RELEASE", "REBOOT"):
            self.control = OperatorControl()
            self.ready(stopped=True)
            packet = self.control.request(action, True, 100)
            request_id = packet.split(":")[1]
            self.control.mark_sent(packet, 100)
            self.assertIsNone(self.control.notification(f"[OP ACK:{request_id}:{action}]", 106))
            self.assertTrue(self.control.blocks_motion())
            self.assertFalse(self.control.reboot_allowed())

    def test_stop_after_committed_reboot_keeps_coordinated_pi_reboot(self):
        self.ready(stopped=True)
        packet = self.control.request("REBOOT", True, 100)
        self.assertEqual(self.acknowledge(packet), "REBOOT")
        self.assertIsNone(self.control.request("ESTOP", True, 101))
        self.assertTrue(self.control.reboot_allowed())
        self.assertTrue(self.control.blocks_motion())

    def test_second_stop_cannot_invalidate_durable_reboot_when_disk_becomes_readonly(self):
        store = types.SimpleNamespace(load=lambda: True, save=lambda stopped: (_ for _ in ()).throw(OSError("readonly")))
        self.control = OperatorControl(store)
        self.ready(stopped=True)
        packet = self.control.request("REBOOT", True, 100)
        self.assertEqual(self.acknowledge(packet), "REBOOT")
        self.control.request("ESTOP", True, 101)
        self.assertTrue(self.control.reboot_allowed())

    def stopped_with_route(self, now=100):
        self.control.note_state("IDLE", now)
        self.control.notification("[OP STATUS:1:ESTOP=1:SONAR=1:RESUME=1]", now)

    def test_continue_needs_a_confirmed_stop_and_a_route_to_continue(self):
        # 2026-11-14: Continue run releases the stop into the route it interrupted.
        self.ready(stopped=True)  # older executor status: no RESUME field
        status = self.control.status(True, 100)
        self.assertTrue(status["can_release"])
        self.assertFalse(status["can_continue"])
        self.assertIsNone(status["route_to_continue"])
        self.assertIsNone(self.control.request("CONTINUE", True, 100))
        self.control.notification("[OP STATUS:1:ESTOP=1:SONAR=1:RESUME=0]", 100)
        self.assertFalse(self.control.status(True, 100)["can_continue"])
        self.assertIsNone(self.control.request("CONTINUE", True, 100))
        self.stopped_with_route()
        self.assertTrue(self.control.status(True, 100)["can_continue"])
        self.control.note_state("OUTBOUND", 100)
        self.assertFalse(self.control.status(True, 100)["can_continue"])
        self.assertIsNone(self.control.request("CONTINUE", True, 100))
        self.stopped_with_route()
        self.assertFalse(self.control.status(False, 100)["can_continue"])
        self.assertFalse(self.control.status(True, 108)["can_continue"])  # stale status
        self.control.notification("[OP STATUS:1:ESTOP=0:SONAR=1:RESUME=1]", 100)
        self.assertFalse(self.control.status(True, 100)["can_continue"])  # not stopped

    def test_continue_is_a_confirmed_release_that_is_never_replayed(self):
        self.acknowledge(self.control.request("ESTOP", True, 100))
        self.stopped_with_route()
        packet = self.control.request("CONTINUE", True, 100)
        self.assertTrue(packet.endswith(":CONTINUE]"))
        self.assertTrue(self.control.blocks_motion())
        self.assertTrue(self.control.can_send(packet, True, 100))
        self.control.mark_sent(packet, 100)
        self.assertFalse(self.control.can_send(packet, True, 100.1))
        self.assertNotEqual(self.control.next_packet(True, 101), packet)
        self.assertTrue(self.control.blocks_motion())  # sending alone never releases
        self.acknowledge(packet, 101)
        self.assertFalse(self.control.blocks_motion())
        # Like RELEASE, an ACK updates only its field: a fresh full status is needed.
        status = self.control.status(True, 101)
        self.assertFalse(status["supported"])
        self.assertFalse(status["can_continue"])
        self.assertEqual(self.control.next_packet(True, 101), "[OP STATUS]")
        self.control.note_state("OUTBOUND", 101)
        self.control.notification("[OP STATUS:1:ESTOP=0:SONAR=1:RESUME=0]", 101)
        status = self.control.status(True, 101)
        self.assertFalse(status["emergency_stopped"])
        self.assertFalse(status["can_continue"] or status["can_release"])

    def test_continue_with_a_different_acknowledgement_or_lost_route_does_not_release(self):
        self.acknowledge(self.control.request("ESTOP", True, 100))
        self.stopped_with_route()
        packet = self.control.request("CONTINUE", True, 100)
        _, request_id, _ = packet[1:-1].split(":")
        self.control.mark_sent(packet, 100)
        self.control.notification(f"[OP ACK:{request_id}:RELEASE]", 100)
        self.assertTrue(self.control.blocks_motion())
        self.control.notification(f"[OP FAILED:{request_id}:NO_ROUTE]", 100)
        self.assertTrue(self.control.blocks_motion())
        self.assertIn("NO_ROUTE", self.control.status(True, 100)["message"])
        # The route disappears between the request and the write: cancel, never send.
        self.stopped_with_route(101)
        packet = self.control.request("CONTINUE", True, 101)
        self.control.notification("[OP STATUS:1:ESTOP=1:SONAR=1:RESUME=0]", 101)
        self.assertFalse(self.control.can_send(packet, True, 101))
        self.assertTrue(self.control.blocks_motion())

    def stopped_with_reverse(self, now=100):
        self.control.note_state("IDLE", now)
        self.control.notification("[OP STATUS:1:ESTOP=1:SONAR=1:RESUME=1:REV=1]", now)

    def test_reverse_needs_a_confirmed_stop_and_a_route_to_reverse(self):
        # 2026-11-14: Reverse to start releases the stop into the way back.
        self.stopped_with_route()   # CONTINUE-20261114 firmware: no REV field
        status = self.control.status(True, 100)
        self.assertTrue(status["can_continue"])
        self.assertFalse(status["can_reverse"])
        self.assertIsNone(status["route_to_reverse"])
        self.assertIsNone(self.control.request("REVERSE", True, 100))
        self.control.notification("[OP STATUS:1:ESTOP=1:SONAR=1:RESUME=1:REV=0]", 100)
        self.assertFalse(self.control.status(True, 100)["can_reverse"])
        self.assertIsNone(self.control.request("REVERSE", True, 100))
        self.stopped_with_reverse()
        status = self.control.status(True, 100)
        self.assertTrue(status["can_reverse"] and status["can_continue"] and status["can_release"])
        self.control.note_state("REVERSING", 100)
        self.assertFalse(self.control.status(True, 100)["can_reverse"])
        self.stopped_with_reverse()
        self.assertFalse(self.control.status(False, 100)["can_reverse"])
        self.assertFalse(self.control.status(True, 108)["can_reverse"])  # stale status
        self.control.notification("[OP STATUS:1:ESTOP=0:SONAR=1:RESUME=1:REV=1]", 100)
        self.assertFalse(self.control.status(True, 100)["can_reverse"])  # not stopped
        for malformed in ("[OP STATUS:1:ESTOP=1:SONAR=1:REV=1:RESUME=1]",
                          "[OP STATUS:1:ESTOP=1:SONAR=1:RESUME=1:REV=2]"):
            self.control.disconnect()
            self.control.note_state("IDLE", 100)
            self.control.notification(malformed, 100)
            self.assertFalse(self.control.status(True, 100)["supported"], malformed)

    def test_reverse_is_a_confirmed_release_that_is_never_replayed(self):
        self.acknowledge(self.control.request("ESTOP", True, 100))
        self.stopped_with_reverse()
        packet = self.control.request("REVERSE", True, 100)
        self.assertTrue(packet.endswith(":REVERSE]"))
        self.assertTrue(self.control.blocks_motion())
        self.assertTrue(self.control.can_send(packet, True, 100))
        self.control.mark_sent(packet, 100)
        self.assertFalse(self.control.can_send(packet, True, 100.1))
        self.assertNotEqual(self.control.next_packet(True, 101), packet)
        self.assertTrue(self.control.blocks_motion())  # sending alone never releases
        _, request_id, _ = packet[1:-1].split(":")
        self.control.notification(f"[OP ACK:{request_id}:CONTINUE]", 101)
        self.assertTrue(self.control.blocks_motion())  # another action's acknowledgement
        self.acknowledge(packet, 101)
        self.assertFalse(self.control.blocks_motion())
        status = self.control.status(True, 101)
        self.assertFalse(status["supported"] or status["can_reverse"])
        self.control.note_state("REVERSING", 101)
        self.control.notification("[OP STATUS:1:ESTOP=0:SONAR=1:RESUME=0:REV=0]", 101)
        status = self.control.status(True, 101)
        self.assertFalse(status["emergency_stopped"])
        self.assertFalse(status["can_reverse"] or status["can_continue"] or status["can_release"])

    def test_a_refused_or_lost_reverse_does_not_release(self):
        self.acknowledge(self.control.request("ESTOP", True, 100))
        self.stopped_with_reverse()
        packet = self.control.request("REVERSE", True, 100)
        _, request_id, _ = packet[1:-1].split(":")
        self.control.mark_sent(packet, 100)
        self.control.notification(f"[OP FAILED:{request_id}:NO_ROUTE]", 100)
        self.assertTrue(self.control.blocks_motion())
        self.assertIn("NO_ROUTE", self.control.status(True, 100)["message"])
        # The route record disappears between the request and the write: never sent.
        self.stopped_with_reverse(101)
        packet = self.control.request("REVERSE", True, 101)
        self.control.notification("[OP STATUS:1:ESTOP=1:SONAR=1:RESUME=1:REV=0]", 101)
        self.assertFalse(self.control.can_send(packet, True, 101))
        self.assertTrue(self.control.blocks_motion())

    def test_failed_release_persistence_reapplies_stop(self):
        store = types.SimpleNamespace(load=lambda: True, save=lambda stopped: (_ for _ in ()).throw(OSError("disk")))
        self.control = OperatorControl(store)
        self.ready(stopped=True)
        packet = self.control.request("RELEASE", True, 100)
        self.acknowledge(packet)
        self.assertTrue(self.control.blocks_motion())
        self.assertTrue(self.control.next_packet(True, 100).endswith(":ESTOP]"))

    def test_stop_survives_process_restart_until_confirmed_release(self):
        with tempfile.TemporaryDirectory() as directory:
            store = StopLatchStore(Path(directory) / "config/stop")
            self.control = OperatorControl(store)
            self.control.request("ESTOP", False, 100)
            self.control = OperatorControl(store)
            self.assertTrue(self.control.blocks_motion())
            self.ready(stopped=True)
            packet = self.control.request("RELEASE", True, 100)
            self.assertTrue(store.load())
            self.acknowledge(packet)
            self.assertFalse(store.load())
            self.assertFalse(OperatorControl(store).blocks_motion())

    def test_storage_failure_keeps_stop_and_disables_restart(self):
        store = types.SimpleNamespace(load=lambda: False, save=lambda stopped: (_ for _ in ()).throw(OSError("disk")))
        self.control = OperatorControl(store)
        self.control.request("ESTOP", False, 100)
        self.ready(stopped=True)
        self.assertTrue(self.control.blocks_motion())
        self.assertFalse(self.control.status(True, 100)["can_reset"])
        self.assertFalse(self.control.status(True, 100)["can_release"])

    def test_request_ids_survive_clock_rollback_process_restart_and_release(self):
        with tempfile.TemporaryDirectory() as directory:
            store = StopLatchStore(Path(directory) / "stop")
            self.control = OperatorControl(store)
            self.ready()
            with patch("operator_support.time.time", return_value=123456):
                first = self.control.request("SONAR_OFF", True, 100)
            first_id = int(first.split(":")[1])
            self.control = OperatorControl(store)
            self.ready(stopped=True)
            with patch("operator_support.time.time", return_value=100):
                release = self.control.request("RELEASE", True, 100)
            self.assertGreater(int(release.split(":")[1]), first_id)
            self.acknowledge(release)
            self.assertFalse(store.load())
            self.assertTrue(store.id_path.is_file())
            self.control = OperatorControl(store)
            with patch("operator_support.time.time", return_value=100):
                stop = self.control.request("ESTOP", False, 100)
            self.assertGreater(int(stop.split(":")[1]), int(release.split(":")[1]))

    def test_reboot_uses_orderly_fixed_command_and_reports_permission_failure(self):
        with patch("operator_support.subprocess.run", return_value=types.SimpleNamespace(
                returncode=1, stdout="", stderr="Access denied")) as run:
            with self.assertRaisesRegex(RuntimeError, "Access denied"):
                reboot_pi()
        self.assertEqual(run.call_args.args[0], ["systemctl", "--no-ask-password", "reboot"])
        self.assertFalse(run.call_args.kwargs["check"])


class RuntimeOperatorTests(unittest.TestCase):
    def setUp(self):
        self.runtime = load_runtime()
        self.runtime.operator_control = OperatorControl()
        self.runtime.ble_connected.set()

    def test_stop_cancels_queued_manual_start_and_return(self):
        runtime = self.runtime
        runtime.front_disabled.set()
        runtime.back_disabled.set()
        runtime.ble_send_queue.put_nowait("[START:1]")
        runtime.ble_send_queue.put_nowait("[RESET:2]")
        runtime.request_operator_action("ESTOP")
        self.assertFalse(runtime.request_manual_start()["enabled"])
        self.assertEqual(list(runtime.ble_send_queue._items), [])

    def test_serialized_writer_suppresses_motion_and_never_releases_stop_from_scan(self):
        runtime = self.runtime
        runtime.front_disabled.set()
        runtime.back_disabled.set()
        runtime.request_operator_action("ESTOP")
        client = types.SimpleNamespace(is_connected=True, write_gatt_char=AsyncMock())
        service = runtime.BleService(None, None)
        service._client = client
        service._write_char = types.SimpleNamespace(max_write_without_response_size=244)
        service._write_lock = asyncio.Lock()
        async def writes():
            await service._write_message("[RESET:2]")
            await service._write_message("[START:1]")
            await service._write_message("P:12|F=C|B=C")
        asyncio.run(writes())
        client.write_gatt_char.assert_awaited_once()
        self.assertEqual(client.write_gatt_char.call_args.args[1], b"P:12|F=S|B=S")
        self.assertTrue(client.write_gatt_char.call_args.kwargs["response"])

    def test_reboot_worker_only_starts_for_attempted_matching_ack(self):
        runtime = self.runtime
        control = runtime.operator_control
        control.note_state("IDLE")
        control.notification("[OP STATUS:1:ESTOP=1:SONAR=1]")
        packet = control.request("REBOOT", True)
        request_id = packet.split(":")[1]
        with patch.object(runtime.threading, "Thread") as worker:
            runtime.ble_notify_handler(f"[OP ACK:{request_id}:REBOOT]")
            worker.assert_not_called()
            control.mark_sent(packet)
            runtime.ble_notify_handler(f"[OP ACK:{request_id}:REBOOT]")
            worker.assert_called_once()
            runtime.ble_notify_handler(f"[OP ACK:{request_id}:REBOOT]")
            worker.assert_called_once()


if __name__ == "__main__":
    unittest.main()
