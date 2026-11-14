import asyncio
import types
import time
import unittest
from unittest.mock import patch

from ble_support import (matches_advertisement, discover_robot, NotificationDecoder,
                         DiscoveryDiagnostics, SERVICE_UUID)
from test_runtime import load_runtime


class DiscoveryTests(unittest.TestCase):
    def test_service_identifies_robot_without_a_name(self):
        self.assertTrue(matches_advertisement(
            types.SimpleNamespace(address="AA",name=None),
            types.SimpleNamespace(local_name=None,service_uuids=[SERVICE_UUID.upper()]),
            address=None,name="GarbyESP32",service_uuid=SERVICE_UUID))

    def test_configured_address_does_not_fall_back_to_a_different_robot(self):
        self.assertFalse(matches_advertisement(
            types.SimpleNamespace(address="BB",name="GarbyESP32"),
            types.SimpleNamespace(local_name="GarbyESP32",service_uuids=[SERVICE_UUID]),
            address="AA",name="GarbyESP32",service_uuid=SERVICE_UUID))

    def test_unrelated_substring_name_is_not_a_robot(self):
        self.assertFalse(matches_advertisement(
            types.SimpleNamespace(address="AA",name="fakeGarbyESP32"),
            types.SimpleNamespace(local_name=None,service_uuids=[]),
            address=None,name="GarbyESP32",service_uuid=SERVICE_UUID))


class NotificationTests(unittest.TestCase):
    def test_default_mtu_chunks_reassemble_exact_return_identity(self):
        decoder=NotificationDecoder(); line="[RESET DONE:1234567890001]\n"
        result=[]
        for offset in range(0,len(line),19):
            chunk=("~" if offset==0 else "+")+line[offset:offset+19]
            self.assertLessEqual(len(chunk),20)
            result.extend(decoder.feed(chunk.encode("ascii")))
        self.assertEqual(result,[line.strip()])

    def test_new_start_discards_partial_or_lost_notification(self):
        decoder=NotificationDecoder()
        self.assertEqual(decoder.feed(b"~[RESET DONE:123"),[])
        self.assertEqual(decoder.feed(b"~[IDLE]\n"),["[IDLE]"])
        decoder.reset()
        self.assertEqual(decoder.feed(b"+456]\n"),[])

    def test_legacy_and_invalid_notifications_are_bounded(self):
        decoder=NotificationDecoder()
        self.assertEqual(decoder.feed(b"[IDLE]\n[RUNNING]"),["[IDLE]","[RUNNING]"])
        self.assertEqual(decoder.feed(b"~"+b"X"*257),[])
        self.assertEqual(decoder.pending,"")
        self.assertEqual(decoder.feed(b"~\xff\n"),[])


class DiscoveryEvidenceTests(unittest.IsolatedAsyncioTestCase):
    @staticmethod
    def advertisement(address, *, garby=True):
        return (types.SimpleNamespace(address=address, name=None),
                types.SimpleNamespace(local_name=None,
                                      service_uuids=[SERVICE_UUID] if garby else []))

    async def test_strict_pin_candidates_are_unique_bounded_and_from_one_scan(self):
        events = [self.advertisement("PRIVATE", garby=False)]
        events += [self.advertisement("BB"), self.advertisement("bb")]
        events += [self.advertisement(str(index)) for index in range(10)]
        scans = []
        class Scanner:
            @staticmethod
            async def find_device_by_filter(callback, **options):
                scans.append(options)
                for device, advertisement in events:
                    if callback(device, advertisement):
                        return device
                return None
        evidence = DiscoveryDiagnostics()
        with patch.dict("os.environ", {"GARBY_BLE_ADAPTER": "hci9"}):
            result = await discover_robot(Scanner, address="AA", timeout=4.0,
                                          diagnostics=evidence)
        self.assertIsNone(result)
        self.assertEqual(evidence.event_count, len(events))
        self.assertEqual(evidence.rejected_addresses, ["BB", "0", "1"])
        self.assertNotIn("PRIVATE", str(evidence.__dict__))
        self.assertEqual(len(scans), 1)
        self.assertEqual(scans[0]["timeout"], 4.0)
        self.assertEqual(scans[0]["scanning_mode"], "active")
        self.assertEqual(scans[0]["adapter"], "hci9")
        self.assertNotIn("service_uuids", scans[0])

    async def test_success_returns_original_device_with_or_without_diagnostics(self):
        target, advertisement = self.advertisement("AA", garby=False)
        class Scanner:
            @staticmethod
            async def find_device_by_filter(callback, **options):
                return target if callback(target, advertisement) else None
        evidence = DiscoveryDiagnostics()
        self.assertIs(await discover_robot(Scanner, address="AA", diagnostics=evidence), target)
        self.assertEqual(evidence.event_count, 1)
        self.assertEqual(evidence.rejected_addresses, [])
        self.assertIs(await discover_robot(Scanner, address="AA"), target)

    async def test_reused_diagnostics_reset_for_each_scan(self):
        evidence = DiscoveryDiagnostics()
        evidence.reset(adapter="hci9", address="AA")
        device, advertisement = self.advertisement("BB")
        evidence.observe(device, advertisement, matched=False, address="AA",
                         name="GarbyESP32", service_uuid=SERVICE_UUID)
        class Scanner:
            @staticmethod
            async def find_device_by_filter(callback, **options):
                return None
        with patch.dict("os.environ", {"GARBY_BLE_ADAPTER": ""}):
            self.assertIsNone(await discover_robot(Scanner, diagnostics=evidence))
        self.assertEqual(evidence.event_count, 0)
        self.assertEqual(evidence.rejected_addresses, [])
        self.assertEqual(evidence.adapter, "platform default")
        self.assertEqual(evidence.pinned_address, "unset")

    async def test_diagnostic_counter_and_stored_fields_stay_bounded(self):
        evidence = DiscoveryDiagnostics()
        evidence.reset(adapter="hci0\n" + "A" * 1000, address="AA" * 1000)
        evidence.event_count = evidence.MAX_EVENTS - 1
        device, advertisement = self.advertisement("B\n" * 1000)
        for _ in range(3):
            evidence.observe(device, advertisement, matched=False, address="AA",
                             name="GarbyESP32", service_uuid=SERVICE_UUID)
        self.assertEqual(evidence.event_count, evidence.MAX_EVENTS)
        self.assertEqual(len(evidence.rejected_addresses), 1)
        for value in (evidence.adapter, evidence.pinned_address, *evidence.rejected_addresses):
            self.assertLessEqual(len(value), evidence.MAX_FIELD_LENGTH)
            self.assertNotIn("\n", value)


class ConnectionTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.runtime=load_runtime()
        self.service=self.runtime.BleService(lambda *args:None,lambda *args:None)
        self.service._loop=asyncio.get_running_loop()
        self.service._stop_event=asyncio.Event()
        self.service._write_lock=asyncio.Lock()
        self.retries=[]
        self.service._schedule_retry=self.retries.append
        self.target=types.SimpleNamespace(address="AA:BB:CC:DD:EE:FF",name=None)
        self.clients=[]
        self.options=[]
        owner=self
        class Scanner:
            @staticmethod
            async def find_device_by_filter(callback,**kwargs):
                owner.options.append(kwargs)
                adv=types.SimpleNamespace(local_name=None,service_uuids=[SERVICE_UUID])
                return owner.target if callback(owner.target,adv) else None
        class Client:
            def __init__(self,device,**kwargs):
                self.device=device;self.arguments=kwargs;self.is_connected=False;self.disconnects=0;self.writes=[]
                self.notify_options=None
                self.services=[types.SimpleNamespace(uuid=owner.runtime.SERVICE_UUID,characteristics=[
                    types.SimpleNamespace(uuid=owner.runtime.WRITE_CHAR_UUID,properties=["write","write-without-response"],max_write_without_response_size=20),
                    types.SimpleNamespace(uuid=owner.runtime.NOTIFY_CHAR_UUID,properties=["notify"])])]
                owner.clients.append(self)
            async def connect(self): self.is_connected=True
            async def disconnect(self): self.disconnects+=1;self.is_connected=False
            async def start_notify(self,characteristic,callback,**kwargs):
                self.notify_options=kwargs;self.notification_callback=callback
            async def write_gatt_char(self,characteristic,payload,response): self.writes.append((payload,response))
        self.runtime.BleakScanner=Scanner;self.runtime.BleakClient=Client

    async def asyncTearDown(self):
        self.service._stop_event.set()
        if self.service._drain_task:
            self.service._drain_task.cancel()
            await asyncio.gather(self.service._drain_task,return_exceptions=True)

    async def test_scanned_device_is_used_and_ready_requires_subscription_and_write(self):
        await self.service._connect_guarded()
        client=self.clients[0]
        self.assertIs(client.device,self.target)
        self.assertEqual(client.notify_options,{"bluez":{"use_start_notify":True}})
        self.assertEqual(client.writes,[(b"[RASPI READY]",True)])
        self.assertTrue(self.runtime.ble_connected.is_set())
        self.assertEqual(self.retries,[])

    async def test_discovery_wrong_pin_reports_observed_rejected_candidate(self):
        self.runtime.BLE_MAC_ADDRESS = "00:00:00:00:00:00"
        with patch.dict("os.environ", {"GARBY_BLE_ADAPTER": "hci9"}):
            with self.assertLogs("combined_app", level="WARNING") as logs:
                await self.service._connect_guarded()
        message = "\n".join(logs.output)
        self.assertIn("rejected by pinned address", message)
        self.assertIn(self.target.address, message)
        self.assertIn("events=1", message)
        self.assertIn("adapter=hci9", message)
        self.assertNotIn("not advertising", message)
        self.assertFalse(self.runtime.ble_connected.is_set())
        self.assertEqual(self.clients, [])
        self.assertEqual(len(self.options), 1, "diagnostics must use the existing scan")
        self.assertEqual(len(self.retries), 1)

    async def test_expired_lidar_clear_is_replaced_by_stop_at_ble_write(self):
        await self.service._connect_guarded()
        client = self.clients[0]
        self.service._drain_task.cancel()
        await asyncio.gather(self.service._drain_task, return_exceptions=True)
        self.runtime.front_disabled.set()
        self.runtime.back_disabled.set()
        await self.service._write_message(
            "P:42|F=C|B=C", expected_client=client,
            expires_at=time.monotonic() - 0.1,
        )
        self.assertEqual(client.writes[-1], (b"P:42|F=S|B=S", True))

    async def test_steering_age_keeps_queue_and_writer_lock_wait(self):
        await self.service._connect_guarded()
        client = self.clients[0]
        self.service._drain_task.cancel()
        await asyncio.gather(self.service._drain_task, return_exceptions=True)
        runtime = self.runtime
        runtime.ble_send_queue.clear()
        deadline = 100.0 + runtime.LIDAR_STALE_TIMEOUT_S
        runtime.ble_send_queue.put_nowait("S:41|CF=300|CL=95|CR=95|A=1|G=0", expires_at=deadline)
        runtime.ble_send_queue.put_nowait("S:42|CF=300|CL=95|CR=95|A=2|G=10", expires_at=deadline + .01)
        packet, retained_deadline = runtime.ble_send_queue.get_entry_nowait()
        now = [100.2]
        await self.service._write_lock.acquire()
        with patch.object(runtime.time, "monotonic", side_effect=lambda: now[0]):
            writer = asyncio.create_task(self.service._write_message(
                packet, expected_client=client, expires_at=retained_deadline))
            await asyncio.sleep(0)
            self.assertFalse(writer.done())
            now[0] = 100.6
            self.service._write_lock.release()
            await writer
        sent = client.writes[-1][0].decode()
        self.assertTrue(sent.startswith("S:42|"))
        age = int(next(part[2:] for part in sent.split("|") if part.startswith("G=")))
        self.assertGreaterEqual(age, 589)
        self.assertLessEqual(age, 591)

    async def test_steering_expiring_behind_writer_becomes_stable(self):
        await self.service._connect_guarded()
        client = self.clients[0]
        self.service._drain_task.cancel()
        await asyncio.gather(self.service._drain_task, return_exceptions=True)
        now = [100.1]
        await self.service._write_lock.acquire()
        with patch.object(self.runtime.time, "monotonic", side_effect=lambda: now[0]):
            writer = asyncio.create_task(self.service._write_message(
                "S:42|CF=300|CL=95|CR=95|A=1|G=100", expected_client=client,
                expires_at=100.8))
            await asyncio.sleep(0)
            now[0] = 100.81
            self.service._write_lock.release()
            await writer
        self.assertEqual(client.writes[-1], (b"S:42|STABLE", False))

    async def test_steering_age_growth_cannot_exceed_the_gatt_packet_limit(self):
        await self.service._connect_guarded()
        client = self.clients[0]
        self.service._drain_task.cancel()
        await asyncio.gather(self.service._drain_task, return_exceptions=True)
        prefix = "S:42|CF=300|CL=95|CR=95|A=1|G=0|"
        packet = prefix + "X" * (244 - len(prefix))
        with patch.object(self.runtime.time, "monotonic", return_value=100.6):
            await self.service._write_message(packet, expected_client=client, expires_at=100.8)
        self.assertEqual(client.writes[-1], (b"S:42|STABLE", False))

    async def test_read_only_lidar_control_allows_fresh_path_and_start(self):
        await self.service._connect_guarded()
        client = self.clients[0]
        self.service._drain_task.cancel()
        await asyncio.gather(self.service._drain_task, return_exceptions=True)
        self.runtime.front_disabled.set()
        self.runtime.back_disabled.set()
        self.runtime.ble_notify_handler("[SYSTEM READY]")
        self.runtime.ble_notify_handler("[IDLE]")
        await self.service._write_message(
            "P:43|F=C|B=C", expected_client=client,
        )
        self.assertEqual(client.writes[-1], (b"P:43|F=C|B=C", True))
        self.assertTrue(self.runtime.request_manual_start()["pending"])
        packet = self.runtime.ble_send_queue.get_nowait()
        await self.service._write_message(packet, expected_client=client)
        self.assertEqual(client.writes[-1], (packet.encode(), True))

    async def test_disconnect_during_subscription_cannot_publish_a_ready_session(self):
        base=self.runtime.BleakClient
        owner=self
        class Drops(base):
            async def start_notify(self,*args,**kwargs):
                self.is_connected=False
                owner.service._handle_disconnect(self)
        self.runtime.BleakClient=Drops
        await self.service._connect_guarded()
        self.assertIsNone(self.service._client)
        self.assertFalse(self.runtime.ble_connected.is_set())
        self.assertEqual(len(self.retries),1)
        self.assertEqual(self.clients[0].disconnects,1)

    async def test_failed_connect_is_cleaned_up_and_can_retry(self):
        base=self.runtime.BleakClient
        class Fails(base):
            async def connect(self): raise RuntimeError("connection failed")
        self.runtime.BleakClient=Fails
        await self.service._connect_guarded()
        self.assertEqual(self.clients[0].disconnects,1)
        self.assertFalse(self.service._connecting)
        self.assertIsNone(self.service._candidate_client)
        self.runtime.BleakClient=base
        await self.service._connect_guarded()
        self.assertTrue(self.runtime.ble_connected.is_set())

    async def test_missing_gatt_is_cleaned_up(self):
        base=self.runtime.BleakClient
        class WrongFirmware(base):
            def __init__(self,*args,**kwargs):
                super().__init__(*args,**kwargs);self.services=[]
        self.runtime.BleakClient=WrongFirmware
        await self.service._connect_guarded()
        self.assertIsNone(self.service._client)
        self.assertEqual(self.clients[0].disconnects,1)
        self.assertFalse(self.runtime.ble_connected.is_set())

    async def test_connection_cancellation_cleans_up_candidate(self):
        base=self.runtime.BleakClient
        class Cancelled(base):
            async def connect(self): raise asyncio.CancelledError()
        self.runtime.BleakClient=Cancelled
        with self.assertRaises(asyncio.CancelledError): await self.service._connect_guarded()
        self.assertEqual(self.clients[0].disconnects,1)
        self.assertEqual(self.retries,[])

    async def test_configured_wrong_address_never_connects_to_other_robot(self):
        self.runtime.BLE_MAC_ADDRESS="00:00:00:00:00:00"
        await self.service._connect_guarded()
        self.assertEqual(self.clients,[])
        self.assertEqual(len(self.retries),1)

    async def test_connection_timeout_cleans_up_and_retries(self):
        base=self.runtime.BleakClient
        class Hangs(base):
            async def connect(self): await asyncio.Event().wait()
        self.runtime.BleakClient=Hangs
        self.service.CONNECT_TIMEOUT=0.01
        await self.service._connect_guarded()
        self.assertEqual(self.clients[0].disconnects,1)
        self.assertIsNone(self.service._client)
        self.assertEqual(len(self.retries),1)

    async def test_notification_timeout_cannot_publish_ready(self):
        base=self.runtime.BleakClient
        class Hangs(base):
            async def start_notify(self,*args,**kwargs): await asyncio.Event().wait()
        self.runtime.BleakClient=Hangs
        self.service.NOTIFY_TIMEOUT=0.01
        await self.service._connect_guarded()
        self.assertEqual(self.clients[0].disconnects,1)
        self.assertEqual(self.clients[0].writes,[])
        self.assertFalse(self.runtime.ble_connected.is_set())
        self.assertEqual(len(self.retries),1)

    async def test_failed_readiness_write_cannot_publish_ready(self):
        base=self.runtime.BleakClient
        class Fails(base):
            async def write_gatt_char(self,*args,**kwargs): raise RuntimeError("write failed")
        self.runtime.BleakClient=Fails
        await self.service._connect_guarded()
        self.assertEqual(self.clients[0].disconnects,1)
        self.assertIsNone(self.service._client)
        self.assertFalse(self.runtime.ble_connected.is_set())
        self.assertEqual(len(self.retries),1)

    async def test_stop_during_readiness_write_cannot_publish_ready(self):
        base=self.runtime.BleakClient;owner=self
        class Stops(base):
            async def write_gatt_char(self,*args,**kwargs): owner.service._stop_event.set()
        self.runtime.BleakClient=Stops
        await self.service._connect_guarded()
        self.assertFalse(self.runtime.ble_connected.is_set())
        self.assertEqual(self.clients[0].disconnects,1)

    async def test_wrong_characteristic_properties_reject_firmware(self):
        base=self.runtime.BleakClient
        class WrongProperties(base):
            def __init__(self,*args,**kwargs):
                super().__init__(*args,**kwargs)
                self.services[0].characteristics[0].properties=["write-without-response"]
        self.runtime.BleakClient=WrongProperties
        await self.service._connect_guarded()
        self.assertEqual(self.clients[0].disconnects,1)
        self.assertFalse(self.runtime.ble_connected.is_set())

    async def test_old_notifications_cannot_publish_or_corrupt_new_session(self):
        received=[];self.service.notify_cb=received.append
        await self.service._connect_guarded()
        old=self.clients[0];old.is_connected=False
        self.service._handle_disconnect(old)
        await asyncio.gather(self.service._drain_task,return_exceptions=True)
        await self.service._connect_guarded()
        current=self.clients[1]
        old.notification_callback(None,b"~[OUTBOUND COMPLETE]\n")
        current.notification_callback(None,b"~[RESET DONE:123")
        old.notification_callback(None,b"~[ROUTE FAULT]\n")
        current.notification_callback(None,b"+456]\n")
        self.assertEqual(received,["[RESET DONE:123456]"])
        self.assertTrue(self.runtime.ble_connected.is_set())

    async def test_shutdown_waits_for_in_progress_candidate_cleanup(self):
        base=self.runtime.BleakClient;owner=self;started=asyncio.Event()
        class Hangs(base):
            async def connect(self): started.set();await asyncio.Event().wait()
            async def disconnect(self):
                await asyncio.sleep(0.01)
                await super().disconnect()
        self.runtime.BleakClient=Hangs
        task=asyncio.create_task(self.service._run_service())
        await asyncio.wait_for(started.wait(),timeout=1)
        self.service._stop_event.set()
        await asyncio.wait_for(task,timeout=1)
        self.assertEqual(self.clients[0].disconnects,1)
        self.assertIsNone(self.service._candidate_client)
        self.assertIsNone(self.service._client)
        self.assertFalse(self.runtime.ble_connected.is_set())
        self.assertEqual(self.retries,[])

    async def test_persistent_write_failure_invalidates_stuck_client_and_retries(self):
        await self.service._connect_guarded()
        client=self.clients[0]
        self.service._drain_task.cancel()
        await asyncio.gather(self.service._drain_task,return_exceptions=True)
        async def failed_write(*args,**kwargs): raise RuntimeError("write failed")
        async def failed_disconnect(): raise RuntimeError("BlueZ disconnect failed")
        client.write_gatt_char=failed_write;client.disconnect=failed_disconnect
        self.service.WRITE_RETRY_BASE_S=0.001
        self.service.WRITE_RETRY_MAX_S=0.001
        self.runtime.ble_send_queue.put_nowait("[RESET:1234567890001]",urgent=True)
        await asyncio.wait_for(self.service._drain_loop(client),timeout=1)
        self.assertIsNone(self.service._client)
        self.assertFalse(self.runtime.ble_connected.is_set())
        self.assertEqual(self.runtime._robot_state,"UNKNOWN")
        self.assertEqual(len(self.retries),1)

    async def test_link_loss_log_records_session_length_and_last_write_age(self):
        await self.service._connect_guarded()
        client=self.clients[0];client.is_connected=False
        with self.assertLogs("combined_app",level="WARNING") as logs:
            self.service._handle_disconnect(client)
            await asyncio.gather(self.service._drain_task,return_exceptions=True)
        self.assertTrue(any("session" in line and "last write" in line for line in logs.output),logs.output)

    async def test_link_loss_closes_the_old_session_before_reconnecting(self):
        # Bleak only closes a session's D-Bus connection in disconnect(). BlueZ then
        # keeps its notification registration, so each leaked session re-subscribes
        # on reconnect and every notification arrives once more (seen live: 1, 2, 3, 4).
        base=self.runtime.BleakClient;owner=self;closed_when_reconnecting=[]
        class SlowClose(base):
            async def connect(self):
                if owner.clients[0] is not self:
                    closed_when_reconnecting.append(owner.clients[0].disconnects)
                await super().connect()
            async def disconnect(self):
                await asyncio.sleep(0.01)
                await super().disconnect()
        self.runtime.BleakClient=SlowClose
        await self.service._connect_guarded()
        old=self.clients[0];old.is_connected=False
        self.service._handle_disconnect(old)
        await asyncio.gather(self.service._drain_task,return_exceptions=True)
        await self.service._connect_guarded()
        self.assertEqual(closed_when_reconnecting,[1])

    async def test_failing_close_of_the_old_session_cannot_block_reconnecting(self):
        await self.service._connect_guarded()
        old=self.clients[0];old.is_connected=False
        async def failed_disconnect(): raise RuntimeError("BlueZ disconnect failed")
        old.disconnect=failed_disconnect
        self.service._handle_disconnect(old)
        await asyncio.gather(self.service._drain_task,return_exceptions=True)
        await self.service._connect_guarded()
        self.assertTrue(self.runtime.ble_connected.is_set())
        self.assertIs(self.service._client,self.clients[1])


if __name__ == "__main__": unittest.main()
