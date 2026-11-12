"""Bench producer shares discovery/GATT/framing while keeping synthetic sensing."""
import asyncio
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

from ble_support import SERVICE_UUID, WRITE_CHAR_UUID, NOTIFY_CHAR_UUID


class BenchTests(unittest.IsolatedAsyncioTestCase):
    async def asyncSetUp(self):
        firebase=types.ModuleType("firebase_admin")
        firebase.credentials=types.SimpleNamespace();firebase.db=types.SimpleNamespace()
        backend=types.ModuleType("bleak")
        self.clients=[];owner=self
        self.target=types.SimpleNamespace(name=None,address="AA:BB:CC:DD:EE:FF")
        class Scanner:
            @staticmethod
            async def find_device_by_filter(callback,**options):
                advertisement=types.SimpleNamespace(local_name=None,service_uuids=[SERVICE_UUID])
                return owner.target if callback(owner.target,advertisement) else None
        class Client:
            def __init__(self,device,**options):
                self.device=device;self.is_connected=False;self.writes=[];self.disconnects=0
                self.services=[types.SimpleNamespace(uuid=SERVICE_UUID,characteristics=[
                    types.SimpleNamespace(uuid=WRITE_CHAR_UUID,properties=["write"],max_write_without_response_size=20),
                    types.SimpleNamespace(uuid=NOTIFY_CHAR_UUID,properties=["notify"])])]
                owner.clients.append(self)
            async def connect(self): self.is_connected=True
            async def start_notify(self,characteristic,callback,**options): self.callback=callback;self.notify_options=options
            async def write_gatt_char(self,characteristic,payload,response): self.writes.append((payload,response))
            async def disconnect(self): self.disconnects+=1;self.is_connected=False
        backend.BleakClient=Client;backend.BleakScanner=Scanner
        spec=importlib.util.spec_from_file_location("garby_bench_test",Path(__file__).with_name("final_w_serial-simulator.py"))
        self.bench=importlib.util.module_from_spec(spec)
        with patch.dict(sys.modules,{"firebase_admin":firebase,"bleak":backend}),patch.dict("os.environ",{},clear=True):
            spec.loader.exec_module(self.bench)
        self.received=[]
        self.service=self.bench.BleService(lambda *args:None,self.received.append)
        self.service._loop=asyncio.get_running_loop()
        self.service._stop_event=asyncio.Event();self.service._write_lock=asyncio.Lock()

    async def test_bench_uses_scanned_device_and_acknowledged_ready_after_notify(self):
        await self.service._connect()
        client=self.clients[0]
        self.assertIs(client.device,self.target)
        self.assertEqual(client.notify_options,{"bluez":{"use_start_notify":True}})
        self.assertEqual(client.writes,[(b"[RASPI READY]",True)])
        client.callback(None,b"~[RESET DONE:123")
        client.callback(None,b"+456]\n")
        self.assertEqual(self.received,["[RESET DONE:123456]"])
        self.service._handle_disconnect(client)
        client.callback(None,b"~[OUTBOUND COMPLETE]\n")
        self.assertEqual(self.received,["[RESET DONE:123456]"])

    async def test_stop_cancels_pending_bench_setup_and_disconnects(self):
        started=asyncio.Event();base=self.bench.BleakClient
        class Hangs(base):
            async def connect(self): started.set();await asyncio.Event().wait()
        self.bench.BleakClient=Hangs
        task=asyncio.create_task(self.service._run_service())
        await asyncio.wait_for(started.wait(),timeout=1)
        self.service._stop_event.set()
        await asyncio.wait_for(task,timeout=1)
        self.assertEqual(self.clients[0].disconnects,1)
        self.assertIsNone(self.service._candidate_client)
        self.assertIsNone(self.service._client)
