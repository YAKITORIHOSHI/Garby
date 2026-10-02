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
import threading
import types
import unittest
from unittest.mock import patch

class FakeNode:
    def __init__(self, name): pass
    def create_timer(self, *args): return None
    def create_subscription(self, *args): self.subscription_qos = args[-1]
    def get_logger(self):
        return types.SimpleNamespace(info=lambda *args: None, error=lambda *args: None)

def load_runtime():
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
    with patch.dict(sys.modules, replacements):
        spec.loader.exec_module(runtime)
    return runtime

class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.runtime = load_runtime()

    @staticmethod
    def scan(ranges=None):
        return types.SimpleNamespace(ranges=ranges if ranges is not None else [2.0]*360,
            angle_min=-math.pi, angle_increment=math.pi/180,
            range_min=0.1, range_max=8.0)

    def test_invalid_yaw_does_not_crash_before_logger_initialization(self):
        with patch.dict("os.environ", {"GARBY_LIDAR_YAW_OFFSET_DEG": "bad-value"}):
            self.assertEqual(load_runtime().LIDAR_YAW_OFFSET_DEG, 0.0)

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

    def test_missing_side_sector_fails_closed(self):
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        scan = self.scan()
        for i in range(247, 293): scan.ranges[i] = math.nan
        node.scan_callback(scan)
        self.assertIn("F=S|B=S", node.build_status_packets()[0])

    def test_invalid_geometry_does_not_kill_ros_callback(self):
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        scan = self.scan(); scan.angle_increment = math.nan
        node.scan_callback(scan)
        self.assertIn("F=S|B=S", node.build_status_packets()[0])

    def test_slow_cadence_does_not_extend_permission_into_outage(self):
        node = self.runtime.LidarDistanceReader(None, queue.Queue())
        with patch.object(self.runtime.time, "monotonic", return_value=99.0): node.scan_callback(self.scan())
        with patch.object(self.runtime.time, "monotonic", return_value=100.0): node.scan_callback(self.scan())
        with patch.object(self.runtime.time, "monotonic", return_value=100.81):
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

    def test_old_reset_completion_cannot_overwrite_new_request(self):
        current = {"requestedAt": 20, "status": "pending", "requestedBy": "operator"}
        ref = types.SimpleNamespace(transaction=lambda callback: callback(dict(current)))
        manager = self.runtime.FirebaseManager.__new__(self.runtime.FirebaseManager)
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
        await asyncio.gather(service._write_message("P:1|F=C|B=C"),service._write_message("[RESET:12345]"))
        self.assertEqual(maximum,1)
        self.assertTrue(all(response for _,response in writes))
        with self.assertRaises(ValueError): await service._write_message("SENSOR:US=2\nGO\nGO")

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
