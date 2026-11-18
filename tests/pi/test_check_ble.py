"""Check the standalone diagnostic against a BLE substitute, without ROS."""
import raspi_path  # noqa: F401  (puts the deployable RasPi/ folder on sys.path)
import asyncio
import types
import unittest
from unittest.mock import patch

import check_ble
from ble_support import SERVICE_UUID, WRITE_CHAR_UUID, NOTIFY_CHAR_UUID


class DiagnosticTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        self.target=types.SimpleNamespace(name="GarbyESP32",address="AA:BB:CC:DD:EE:FF")
        self.clients=[];owner=self
        class Scanner:
            @staticmethod
            async def find_device_by_filter(callback,**options):
                adv=types.SimpleNamespace(local_name="GarbyESP32",service_uuids=[SERVICE_UUID])
                return owner.target if callback(owner.target,adv) else None
        class Client:
            def __init__(self,device,**options):
                self.device=device;self.is_connected=False;self.disconnects=0;self.writes=[]
                self.services=[types.SimpleNamespace(uuid=SERVICE_UUID,characteristics=[
                    types.SimpleNamespace(uuid=WRITE_CHAR_UUID,properties=["write"]),
                    types.SimpleNamespace(uuid=NOTIFY_CHAR_UUID,properties=["notify"])])]
                owner.clients.append(self)
            async def connect(self): self.is_connected=True
            async def disconnect(self): self.disconnects+=1;self.is_connected=False
            async def start_notify(self,characteristic,callback,**options): self.callback=callback
            async def write_gatt_char(self,characteristic,payload,response):
                self.writes.append((payload,response));self.callback(None,b"~CONNECTED...\n")
        self.backend=types.ModuleType("bleak")
        self.backend.BleakClient=Client;self.backend.BleakScanner=Scanner
        self.backend_patch=patch.dict("sys.modules",{"bleak":self.backend})
        self.backend_patch.start()
        self.environment_patch=patch.dict("os.environ",{},clear=True)
        self.environment_patch.start()

    async def asyncTearDown(self):
        self.environment_patch.stop();self.backend_patch.stop()

    async def test_scan_only_never_connects_or_writes(self):
        await check_ble.probe(connect=False)
        self.assertEqual(self.clients,[])

    async def test_wrong_pin_reports_rejected_garby_instead_of_absent_advertising(self):
        with patch.dict("os.environ", {"GARBY_BLE_ADDRESS": "00:00:00:00:00:00",
                                       "GARBY_BLE_ADAPTER": "hci9"}):
            with self.assertRaises(RuntimeError) as raised:
                await check_ble.probe(connect=False)
        message = str(raised.exception)
        self.assertIn("rejected by pinned address", message)
        self.assertIn(self.target.address, message)
        self.assertIn("events=1", message)
        self.assertIn("adapter=hci9", message)
        self.assertIn("pinned_address=00:00:00:00:00:00", message)
        self.assertNotIn("no GARBY advertisement", message)
        self.assertEqual(self.clients, [])

    async def test_zero_events_reports_scan_evidence_without_claiming_radio_absence(self):
        class SilentScanner:
            @staticmethod
            async def find_device_by_filter(callback, **options):
                return None
        self.backend.BleakScanner = SilentScanner
        with self.assertRaises(RuntimeError) as raised:
            await check_ble.probe(connect=False)
        message = str(raised.exception)
        self.assertIn("no advertisement events received", message)
        self.assertIn("events=0", message)
        self.assertIn("adapter=platform default", message)
        self.assertIn("pinned_address=unset", message)
        self.assertEqual(self.clients, [])

    async def test_other_events_report_unmatched_and_do_not_expose_unrelated_devices(self):
        self.target.name = "Private unrelated device"
        class OtherScanner:
            @staticmethod
            async def find_device_by_filter(callback, **options):
                callback(types.SimpleNamespace(name="Private unrelated device", address="99"),
                         types.SimpleNamespace(local_name="Private unrelated device", service_uuids=[]))
                return None
        self.backend.BleakScanner = OtherScanner
        with self.assertRaises(RuntimeError) as raised:
            await check_ble.probe(connect=False)
        message = str(raised.exception)
        self.assertIn("advertisement events received, none matched", message)
        self.assertIn("events=1", message)
        self.assertNotIn("Private unrelated device", message)
        self.assertNotIn("99", message)
        self.assertEqual(self.clients, [])

    async def test_round_trip_writes_only_readiness_and_disconnects(self):
        await check_ble.probe(connect=True)
        self.assertIs(self.clients[0].device,self.target)
        self.assertEqual(self.clients[0].writes,[(b"[RASPI READY]",True)])
        self.assertEqual(self.clients[0].disconnects,1)

    async def test_gatt_failure_identifies_phase_and_cleans_up(self):
        base=self.backend.BleakClient
        class WrongFirmware(base):
            def __init__(self,*args,**options): super().__init__(*args,**options);self.services=[]
        self.backend.BleakClient=WrongFirmware
        with self.assertRaisesRegex(RuntimeError,"CONNECTION/GATT FAIL"):
            await check_ble.probe(connect=True)
        self.assertEqual(self.clients[0].disconnects,1)
        self.assertEqual(self.clients[0].writes,[])

    async def test_notification_failure_is_explicit_and_cleans_up(self):
        base=self.backend.BleakClient
        class Fails(base):
            async def start_notify(self,*args,**options): raise asyncio.TimeoutError()
        self.backend.BleakClient=Fails
        with self.assertRaisesRegex(RuntimeError,"NOTIFY FAIL: TimeoutError"):
            await check_ble.probe(connect=True)
        self.assertEqual(self.clients[0].disconnects,1)
        self.assertEqual(self.clients[0].writes,[])
