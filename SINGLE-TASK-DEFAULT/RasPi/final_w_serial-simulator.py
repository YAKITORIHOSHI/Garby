#!/usr/bin/env python3
"""
MANUAL SIMULATION / VISUALIZATION HARNESS.

This tool shares GARBY's LiDAR-only blockage formatter, thresholds, sensor
sentinels, BLE request token, and backward-compatible command vocabulary. It
intentionally simplifies production behaviour: eight direction cards provide
already-aggregated distances, and the tool emits the legacy combined
PATH/BACK_PATH/SIDES payload. It does not consume raw LaserScan angles, run the
production gap-free 45-degree safety partition, or certify collision safety.

The GUI defaults are 100 cm per LiDAR direction, 999 for the bin trash-level
ultrasonic, and -1 for unavailable MQ sensors. Trash level is telemetry only
and never changes simulated path classification.
"""

import os, sys, time, queue, threading, asyncio, random, math, logging, signal, subprocess, glob, gc
from collections import deque
from datetime import datetime
from statistics import median

from bridge_core import format_lidar_blockage

# pyrefly: ignore [missing-import]
from bleak import BleakScanner, BleakClient

# Custom modules (optional)
try:
    # pyrefly: ignore [missing-import]
    from GPIO_RESET import kill_gpio_users
except ImportError:
    def kill_gpio_users():
        print("[GPIO] kill_gpio_users not available, skipping...")

# ═══ Firebase Manager ═══
# pyrefly: ignore [missing-import]
import firebase_admin
# pyrefly: ignore [missing-import]
from firebase_admin import credentials, db

class FirebaseManager:
    def __init__(self):
        cred = credentials.Certificate(
            "/home/garby/Desktop/garby-thesis-firebase-adminsdk-fbsvc-54fb448489.json"
        )
        firebase_admin.initialize_app(cred, {
            "databaseURL": "https://garby-thesis-default-rtdb.asia-southeast1.firebasedatabase.app"
        })

    def update_app(self, reset_state):
        db.reference("/APP").update({"resetState": reset_state})

    def update_mcu_states(self, is_blocked=None, is_fully_loaded=None,
                          is_sim_registered=None, is_started=None):
        updates = {}
        if is_blocked is not None:        updates["isBlocked"] = is_blocked
        if is_fully_loaded is not None:   updates["isFullyLoaded"] = is_fully_loaded
        if is_sim_registered is not None: updates["isSimModuleRegistered"] = is_sim_registered
        if is_started is not None:        updates["isStarted"] = is_started
        db.reference("/MCU/STATES").update(updates)

    def update_mcu_load_cell(self, value):
        db.reference("/MCU/VALUES").update({"LOAD_CELL": value})

    def get_raspi_launch_time(self):
        return db.reference("/RASPI/STATES/launchTime").get()

    def update_raspi_states(self, launch_time):
        db.reference("/RASPI/STATES").update({"launchTime": launch_time})

    def update_raspi_sensors(self, air_quality=None, ammonia=None,
                             methane=None, ultrasonic_distance=None):
        if ultrasonic_distance is not None:
            db.reference("/RASPI/VALUES/ULTRASONIC_SENSOR").update(
                {"CM_DISTANCE": ultrasonic_distance})
        if air_quality is not None:
            db.reference("/RASPI/VALUES/MQ135_SENSOR").update(
                {"AIR_QUALITY": air_quality})
        if ammonia is not None:
            db.reference("/RASPI/VALUES/MQ137").update(
                {"AMMONIA": ammonia})
        if methane is not None:
            db.reference("/RASPI/VALUES/MQ4_SENSOR").update(
                {"METHANE": methane})

# ═══ Executor Service ═══
from threading import Thread
from queue import Queue

class Task:
    def __init__(self, func, permanent=False):
        self.func = func
        self.permanent = permanent

class ExecutorService:
    def __init__(self, workers=2):
        self.queue = Queue()
        self.workers = workers
        for i in range(workers):
            Thread(target=self._worker, daemon=True, name=f"Worker-{i+1}").start()

    def submit(self, func, permanent=False):
        self.queue.put(Task(func, permanent))

    def _worker(self):
        while True:
            task = self.queue.get()
            crashed = False
            try:
                task.func()
            except Exception:
                import traceback
                traceback.print_exc()
                crashed = True
            finally:
                if task.permanent and crashed:
                    self.queue.put(task)
                self.queue.task_done()

# ═══ Configuration ═══
WRITE_CHAR_UUID  = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
NOTIFY_CHAR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a9"
DEVICE_NAME       = "GarbyESP32"

THRESHOLD_CM      = 95.0                  # front stop distance (increased from 50cm for 1.05m/s travel speed reaction time)
BACK_THRESHOLD_CM = 35.0                  # 35 cm back threshold


SIDES_TOLERANCE   = 5.0
BLE_CMD_PERIOD    = 0.25

REQUEST_TOKEN = "[REQUEST-STATUS]"
RASPI_READY   = "[RASPI READY]"

firebase = None

# Default sensor readings: ultrasonic=999, MQ sensors=-1
latest_sensor_readings = {"ultrasonic": 999.0, "mq4": -1, "mq137": -1, "mq135": -1}
sensor_readings_lock = threading.Lock()

serial_stop = threading.Event()
ble_send_queue = queue.Queue()
ble_recv_queue = queue.Queue()

_status_requested = threading.Event()
_robot_running = threading.Event()
front_disabled = threading.Event()
back_disabled  = threading.Event()

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S"
)
logger = logging.getLogger("combined_app_sim")

# Module‑level ble_service placeholder – will be set after BLE connects
ble_service = None

# ═══ Simulation Sensor Loop ═══
def simulated_sensor_updater(stop_event):
    logger.info("Simulated sensor updater started.")
    last_firebase_update = 0.0
    while not stop_event.is_set():
        with sensor_readings_lock:
            us   = latest_sensor_readings.get("ultrasonic")
            m4   = latest_sensor_readings.get("mq4")
            m137 = latest_sensor_readings.get("mq137")
            m135 = latest_sensor_readings.get("mq135")

        now = time.time()
        if now - last_firebase_update >= 2.0:
            last_firebase_update = now
            if firebase:
                try:
                    firebase.update_raspi_sensors(
                        ultrasonic_distance=us if us is not None else 999,
                        methane=m4 if m4 is not None else -1,
                        ammonia=m137 if m137 is not None else -1,
                        air_quality=m135 if m135 is not None else -1
                    )
                except Exception as e:
                    logger.error(f"Firebase sensor update error: {e}")

        time.sleep(0.1)
    logger.info("Simulated sensor updater stopped.")

# ═══ On-demand status responder ═══
def on_demand_status_responder(lidar_node, stop_event):
    logger.info("On-demand status responder started.")
    while not stop_event.is_set():
        fired = _status_requested.wait(timeout=0.1)
        if not fired:
            continue
        _status_requested.clear()

        with sensor_readings_lock:
            readings = dict(latest_sensor_readings)

        us      = readings.get("ultrasonic")
        mq4_v   = readings.get("mq4")
        mq137_v = readings.get("mq137")
        mq135_v = readings.get("mq135")

        us_str    = f"{us:.1f}"  if us      is not None else "999"
        mq4_str   = str(mq4_v)   if mq4_v   is not None else "-1"
        mq137_str = str(mq137_v) if mq137_v is not None else "-1"
        mq135_str = str(mq135_v) if mq135_v is not None else "-1"

        sensor_msg = (f"SENSOR:US={us_str}|MQ4={mq4_str}"
                      f"|MQ137={mq137_str}|MQ135={mq135_str}")

        if lidar_node is not None:
            combined_cmd = lidar_node.build_combined_cmd()
        else:
            combined_cmd = "PATH:CLEAR|BACK_PATH:CLEAR|SIDES:STABLE"

        for msg in (sensor_msg, combined_cmd):
            try:
                ble_send_queue.put_nowait(msg)
            except queue.Full:
                pass

    logger.info("On-demand status responder stopped.")

# ═══ BLE notification handler ═══
def ble_notify_handler(msg: str):
    try:
        ble_recv_queue.put_nowait(msg)
    except queue.Full:
        pass

    stripped = msg.strip()
    if stripped == REQUEST_TOKEN:
        _robot_running.set()
        _status_requested.set()
        return
    if stripped in ("[RESET]", "[IDLE]"):
        _robot_running.clear()

# ═══ BLE Service with urgent direct write ═══
class BleService(threading.Thread):
    RETRY_DELAY     = 3.0
    SCAN_TIMEOUT    = 5.0
    CONNECT_TIMEOUT = 6.0
    DRAIN_INTERVAL  = 0.1

    def __init__(self, status_cb, notify_cb):
        super().__init__(daemon=True, name="ble-service")
        self.status_cb    = status_cb
        self.notify_cb    = notify_cb
        self._loop        = None
        self._client      = None
        self._write_char  = None
        self._notify_char = None
        self._connecting  = False
        self._stop_event  = asyncio.Event()

    def stop(self):
        if self._loop and not self._loop.is_closed():
            self._loop.call_soon_threadsafe(lambda: self._stop_event.set())

    def disconnect_client(self, timeout=3.0):
        if self._loop and not self._loop.is_closed():
            future = asyncio.run_coroutine_threadsafe(self._do_disconnect(), self._loop)
            try:
                future.result(timeout=timeout)
            except Exception as e:
                logger.warning(f"BLE disconnect timeout/error: {e}")
        self.stop()
        if self.is_alive():
            self.join(timeout=1.0)

    async def _do_disconnect(self):
        if self._client and self._client.is_connected:
            try:
                await self._client.disconnect()
                logger.info("BLE client disconnected.")
            except Exception as e:
                logger.error(f"Error during BLE disconnect: {e}")
        self._stop_event.set()
        self._loop.call_soon(self._loop.stop)

    def run(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._stop_event = asyncio.Event()
        self._loop.call_soon(lambda: asyncio.ensure_future(self._connect(), loop=self._loop))
        self._loop.run_forever()

    def _schedule_retry(self):
        if self._loop and not self._loop.is_closed():
            self._connecting = False
            _robot_running.clear()
            self._loop.call_later(
                self.RETRY_DELAY,
                lambda: asyncio.ensure_future(self._connect(), loop=self._loop)
            )

    def _status(self, text: str, color: str):
        self.status_cb(text, color)

    def _on_disconnect(self, _client):
        self._client      = None
        self._write_char  = None
        self._notify_char = None
        _robot_running.clear()
        logger.info("BLE device disconnected.")
        self._status("Disconnected — retry 3 s", "#f85149")
        self._schedule_retry()

    def _notification_handler(self, sender, data):
        try:
            msg = data.decode('utf-8')
        except Exception:
            msg = str(data)
        if self.notify_cb:
            self.notify_cb(msg)

    async def _connect(self):
        if self._connecting:
            return
        self._connecting = True
        self._status("Scanning…", "#d29922")
        try:
            devices = await BleakScanner.discover(timeout=self.SCAN_TIMEOUT)
        except Exception as exc:
            logger.error(f"BLE scan error: {exc}")
            self._status("Scan error — retry 3 s", "#f85149")
            self._schedule_retry()
            return

        target = next((d for d in devices if d.name and d.name == DEVICE_NAME), None)
        if target is None:
            logger.warning(f"'{DEVICE_NAME}' not found.")
            self._status("Not found — retry 3 s", "#f85149")
            self._schedule_retry()
            return

        logger.info(f"Found {target.name} [{target.address}]")
        self._status(f"Connecting {target.address}…", "#d29922")
        client = BleakClient(target.address, disconnected_callback=self._on_disconnect)
        try:
            await asyncio.wait_for(client.connect(), timeout=self.CONNECT_TIMEOUT)
        except Exception as exc:
            logger.error(f"BLE connect error: {exc}")
            self._status("Connect failed — retry 3 s", "#f85149")
            try:
                await client.disconnect()
            except Exception:
                pass
            self._schedule_retry()
            return

        if not client.is_connected:
            logger.warning("connect() returned but is_connected is False.")
            self._status("Connect failed — retry 3 s", "#f85149")
            self._schedule_retry()
            return

        write_char  = None
        notify_char = None
        for svc in client.services:
            for c in svc.characteristics:
                if c.uuid.lower() == WRITE_CHAR_UUID.lower():
                    write_char = c
                if c.uuid.lower() == NOTIFY_CHAR_UUID.lower():
                    notify_char = c

        if write_char is None:
            logger.warning(f"Write characteristic {WRITE_CHAR_UUID} not found — disconnecting.")
            self._status("Write char not found", "#f85149")
            await client.disconnect()
            self._schedule_retry()
            return

        if notify_char is None:
            logger.warning(f"Notify characteristic {NOTIFY_CHAR_UUID} not found.")
            self._status("Notify char missing", "#d29922")
        else:
            try:
                await client.start_notify(notify_char, self._notification_handler)
                logger.info("Subscribed to BLE notifications.")
            except Exception as e:
                logger.error(f"Failed to subscribe to notifications: {e}")

        self._client      = client
        self._write_char  = write_char
        self._notify_char = notify_char
        self._connecting  = False
        self._status("CONNECTED", "#3fb950")
        logger.info("BLE connected.")

        try:
            ble_send_queue.put_nowait(RASPI_READY)
            logger.info(f"[PROTOCOL] Sent {RASPI_READY}")
        except queue.Full:
            pass

        asyncio.ensure_future(self._drain_loop(), loop=self._loop)

    async def _drain_loop(self):
        while not self._stop_event.is_set() and self._client and self._client.is_connected:
            try:
                msg = ble_send_queue.get_nowait()
            except queue.Empty:
                await asyncio.sleep(self.DRAIN_INTERVAL)
                continue
            try:
                await self._client.write_gatt_char(self._write_char, msg.encode("utf-8"), response=False)
            except Exception as exc:
                logger.error(f"BLE write error: {exc}")
                try:
                    await self._client.disconnect()
                except Exception:
                    pass
                self._schedule_retry()
                return
        logger.info("BLE drain loop exiting.")

    async def _urgent_write(self, msg):
        """Called from LiDAR callback via threadsafe schedule."""
        if not self._client or not self._client.is_connected:
            return
        try:
            await self._client.write_gatt_char(self._write_char, msg.encode("utf-8"), response=False)
        except Exception as e:
            logger.error(f"Urgent BLE write error: {e}")

# ═══ Simulated LiDAR Distance Reader ═══
class SimulatedLidarReader:
    """Manual eight-direction visualizer, not a raw-scan safety simulator."""
    SIDES = {
        "FRONT":       180.0,
        "FRONT_LEFT":  225.0,
        "LEFT":        270.0,
        "BACK_LEFT":   315.0,
        "BACK":        0.0,
        "BACK_RIGHT":  45.0,
        "RIGHT":       90.0,
        "FRONT_RIGHT": 135.0
    }
    FRONT_GROUP = {"FRONT", "FRONT_LEFT", "FRONT_RIGHT"}
    BACK_GROUP  = {"BACK",  "BACK_LEFT",  "BACK_RIGHT"}

    CONE_DEG       = 22.0  # display metadata only; no raw angular partition here
    HISTORY_DEPTH  = 6

    def __init__(self, data_queue: queue.Queue, ble_service=None):
        self.data_queue = data_queue
        self.ble_service = ble_service
        self.history    = {side: deque(maxlen=self.HISTORY_DEPTH) for side in self.SIDES}
        # Pre-populate history with 100.0 cm default
        for side in self.SIDES:
            for _ in range(self.HISTORY_DEPTH):
                self.history[side].append(100.0)

        self.last_sides_status   = "SIDES:STABLE"
        self._prev_front_blocked = False
        self._prev_back_blocked  = False
        self._last_urgent_send   = 0
        self._urgent_cooldown    = 0.2

    def _quick_min(self, side: str, window=5):
        valid = [v for v in self.history[side] if v is not None][-window:]
        if not valid:
            return None
        return min(valid)

    def _smoothed_distance(self, side: str):
        valid = [v for v in self.history[side] if v is not None]
        if not valid:
            return None
        return median(valid)

    def _collect_blocked_sides(self, group, threshold_cm):
        blocked = []
        for side in group:
            d = self._quick_min(side)
            if d is None or d <= threshold_cm:
                blocked.append(side)
        return blocked

    def _format_blocked_status(self, blocked_sides, all_sides):
        return format_lidar_blockage(blocked_sides, all_sides)

    def _get_blocked_status(self, group, threshold_cm):
        blocked = self._collect_blocked_sides(group, threshold_cm)
        return self._format_blocked_status(blocked, group)

    def _send_immediate_status(self, front_blocked, back_blocked):
        front_status = "CLEAR"
        back_status  = "CLEAR"
        if front_blocked:
            blocked = self._collect_blocked_sides(self.FRONT_GROUP, THRESHOLD_CM)
            _, front_status = self._format_blocked_status(blocked, self.FRONT_GROUP)
        if back_blocked:
            blocked = self._collect_blocked_sides(self.BACK_GROUP, BACK_THRESHOLD_CM)
            _, back_status = self._format_blocked_status(blocked, self.BACK_GROUP)

        left_med  = self._smoothed_distance("LEFT")
        right_med = self._smoothed_distance("RIGHT")
        front_med = self._smoothed_distance("FRONT")
        back_med  = self._smoothed_distance("BACK")
        front_left_med  = self._smoothed_distance("FRONT_LEFT")
        front_right_med = self._smoothed_distance("FRONT_RIGHT")
        back_left_med   = self._smoothed_distance("BACK_LEFT")
        back_right_med  = self._smoothed_distance("BACK_RIGHT")

        sides_parts = []
        if left_med is not None and right_med is not None:
            sides_parts.append(f"LEFT={left_med:.1f}|RIGHT={right_med:.1f}")
        if front_med is not None:
            sides_parts.append(f"FRONT={front_med:.1f}")
        if back_med is not None:
            sides_parts.append(f"BACK={back_med:.1f}")
        if front_left_med is not None:
            sides_parts.append(f"FRONT_LEFT={front_left_med:.1f}")
        if front_right_med is not None:
            sides_parts.append(f"FRONT_RIGHT={front_right_med:.1f}")
        if back_left_med is not None:
            sides_parts.append(f"BACK_LEFT={back_left_med:.1f}")
        if back_right_med is not None:
            sides_parts.append(f"BACK_RIGHT={back_right_med:.1f}")

        if sides_parts:
            sides_cmd = "|".join(sides_parts)
        else:
            sides_cmd = "STABLE"

        combined = f"PATH:{front_status}|BACK_PATH:{back_status}|SIDES:{sides_cmd}"
        if self.ble_service and self.ble_service._loop:
            asyncio.run_coroutine_threadsafe(
                self.ble_service._urgent_write(combined),
                self.ble_service._loop
            )
        else:
            try:
                ble_send_queue.put_nowait(combined)
            except queue.Full:
                pass

    def update_distances(self, distances_dict: dict):
        """Update simulated LiDAR distances for all sides (values in cm)."""
        for side in self.SIDES:
            val_cm = distances_dict.get(side)
            if val_cm is not None and val_cm > 0:
                self.history[side].append(val_cm)
            else:
                self.history[side].append(100.0)

        data_out = {}
        for s in self.SIDES:
            h = [v for v in self.history[s] if v is not None]
            if h:
                med_cm = median(h)
                data_out[s] = (med_cm / 100.0, med_cm / 100.0, len(h))
            else:
                data_out[s] = None

        front_blocked, _ = self._get_blocked_status(self.FRONT_GROUP, THRESHOLD_CM)
        back_blocked, _  = self._get_blocked_status(self.BACK_GROUP,  BACK_THRESHOLD_CM)
        if front_disabled.is_set(): front_blocked = False
        if back_disabled.is_set():  back_blocked = False

        now = time.monotonic()
        if (front_blocked or back_blocked) and (
            front_blocked != self._prev_front_blocked or back_blocked != self._prev_back_blocked
        ):
            if now - self._last_urgent_send > self._urgent_cooldown:
                self._send_immediate_status(front_blocked, back_blocked)
                self._last_urgent_send = now

        self._prev_front_blocked = front_blocked
        self._prev_back_blocked = back_blocked

        if self.data_queue.full():
            try: self.data_queue.get_nowait()
            except queue.Empty: pass
        self.data_queue.put_nowait(
            (data_out, 0.03, 8.0, self.CONE_DEG, self.last_sides_status)
        )

    def build_combined_cmd(self) -> str:
        front_blocked, front_status = self._get_blocked_status(self.FRONT_GROUP, THRESHOLD_CM)
        back_blocked,  back_status  = self._get_blocked_status(self.BACK_GROUP,  BACK_THRESHOLD_CM)
        if front_disabled.is_set(): front_blocked, front_status = False, "CLEAR"
        if back_disabled.is_set():  back_blocked,  back_status  = False, "CLEAR"

        left_med  = self._smoothed_distance("LEFT")
        right_med = self._smoothed_distance("RIGHT")
        front_med = self._smoothed_distance("FRONT")
        back_med  = self._smoothed_distance("BACK")
        front_left_med  = self._smoothed_distance("FRONT_LEFT")
        front_right_med = self._smoothed_distance("FRONT_RIGHT")
        back_left_med   = self._smoothed_distance("BACK_LEFT")
        back_right_med  = self._smoothed_distance("BACK_RIGHT")

        sides_parts = []
        if left_med is not None and right_med is not None:
            sides_parts.append(f"LEFT={left_med:.1f}|RIGHT={right_med:.1f}")
        if front_med is not None:
            sides_parts.append(f"FRONT={front_med:.1f}")
        if back_med is not None:
            sides_parts.append(f"BACK={back_med:.1f}")
        if front_left_med is not None:
            sides_parts.append(f"FRONT_LEFT={front_left_med:.1f}")
        if front_right_med is not None:
            sides_parts.append(f"FRONT_RIGHT={front_right_med:.1f}")
        if back_left_med is not None:
            sides_parts.append(f"BACK_LEFT={back_left_med:.1f}")
        if back_right_med is not None:
            sides_parts.append(f"BACK_RIGHT={back_right_med:.1f}")

        # Multi-Point Angle of Vision (heading tilt estimate from all sides)
        tilt_val = 0.0
        has_tilt = False
        if front_left_med is not None and front_right_med is not None and back_left_med is not None and back_right_med is not None:
            tilt_val += 0.5 * (front_left_med - front_right_med) + 0.5 * (back_right_med - back_left_med)
            has_tilt = True
        elif front_left_med is not None and front_right_med is not None:
            tilt_val += (front_left_med - front_right_med) * 0.7
            has_tilt = True
        if left_med is not None and right_med is not None:
            tilt_val += (left_med - right_med) * 0.3
            has_tilt = True
        if has_tilt:
            sides_parts.append(f"TILT={tilt_val:.1f}")

        if sides_parts:
            sides_cmd = "|".join(sides_parts)
        else:
            sides_cmd = "STABLE"

        self.last_sides_status = f"SIDES:{sides_cmd}"
        combined = f"PATH:{front_status}|BACK_PATH:{back_status}|SIDES:{sides_cmd}"
        return combined

# ═══ GUI ═══
def init_gui(sim_node: SimulatedLidarReader):
    import tkinter as tk
    from tkinter import font as tkfont
    from tkinter import scrolledtext

    class LidarGUI:
        ARROWS = {
            "FRONT":       "▲ FRONT",
            "FRONT_RIGHT": "⬈ FRONT-RIGHT",
            "RIGHT":       "RIGHT ▶",
            "BACK_RIGHT":  "⬊ BACK-RIGHT",
            "BACK":        "▼ BACK",
            "BACK_LEFT":   "⬋ BACK-LEFT",
            "LEFT":        "◀ LEFT",
            "FRONT_LEFT":  "⬉ FRONT-LEFT"
        }
        ALERT_SIDES = {"FRONT", "FRONT_RIGHT", "FRONT_LEFT",
                       "BACK", "BACK_LEFT", "BACK_RIGHT"}
        FRONT_GROUP = {"FRONT", "FRONT_LEFT", "FRONT_RIGHT"}
        BACK_GROUP  = {"BACK", "BACK_LEFT", "BACK_RIGHT"}
        BASE_CARD_W = 190
        BASE_CARD_H = 130

        def __init__(self, root: tk.Tk, data_queue: queue.Queue, recv_queue: queue.Queue):
            self.root = root
            self.data_queue = data_queue
            self.recv_queue = recv_queue
            self.sim_node   = sim_node
            root.title("YDLidar – 8 Direction Distance Monitor + BLE (SIMULATION)")
            root.configure(bg="#0d1117")
            root.geometry("1000x700")
            root.minsize(800, 500)
            root.bind('<F11>', self.toggle_fullscreen)

            self._scalable_fonts = []
            side_fnt  = tkfont.Font(family="Helvetica", size=10, weight="bold")
            val_fnt   = tkfont.Font(family="Courier",   size=18, weight="bold")
            sensor_val_fnt = tkfont.Font(family="Courier", size=14, weight="bold")
            small_fnt = tkfont.Font(family="Helvetica", size=9)
            title_fnt = tkfont.Font(family="Helvetica", size=12, weight="bold")

            hdr = tk.Frame(root, bg="#0d1117")
            hdr.pack(fill="x", padx=16, pady=(12, 4))
            tk.Label(hdr, text="● YDLidar 8-Direction Distance Monitor (SIM)",
                     bg="#0d1117", fg="#58a6ff", font=title_fnt).pack(side="left")
            self.status_lbl = tk.Label(hdr, text="Waiting…", bg="#0d1117",
                                       fg="#8b949e", font=small_fnt)
            self.status_lbl.pack(side="right")

            bottom_frame = tk.Frame(root, bg="#0d1117")
            bottom_frame.pack(side="bottom", fill="x")

            self.footer = tk.Label(bottom_frame,
                                   text="manual 8-direction visualization | not raw scan geometry",
                                   bg="#0d1117", fg="#8b949e", font=small_fnt)
            self.footer.pack(pady=(2, 6))

            ble_bar = tk.Frame(bottom_frame, bg="#0d1117")
            ble_bar.pack(fill="x", padx=16, pady=(0, 6))
            tk.Label(ble_bar, text="BLE:", bg="#0d1117",
                     fg="#8b949e", font=small_fnt).pack(side="left")
            self.ble_status_lbl = tk.Label(ble_bar, text="● Initialising…",
                                           bg="#0d1117", fg="#d29922", font=small_fnt)
            self.ble_status_lbl.pack(side="left", padx=(4, 0))

            btn_fnt = tkfont.Font(family="Helvetica", size=9, weight="bold")
            self.front_btn = tk.Button(
                ble_bar, text="FRONT: ENABLED", font=btn_fnt,
                bg="#238636", fg="#ffffff", activebackground="#2ea043",
                activeforeground="#ffffff", relief="flat", padx=10, pady=2,
                command=self._toggle_front
            )
            self.front_btn.pack(side="right", padx=(6, 0))

            self.back_btn = tk.Button(
                ble_bar, text="BACK: ENABLED", font=btn_fnt,
                bg="#238636", fg="#ffffff", activebackground="#2ea043",
                activeforeground="#ffffff", relief="flat", padx=10, pady=2,
                command=self._toggle_back
            )
            self.back_btn.pack(side="right", padx=(6, 0))

            log_frame = tk.Frame(bottom_frame, bg="#0d1117")
            log_frame.pack(fill="x", padx=16, pady=(0, 8))
            tk.Label(log_frame, text="BLE RECEIVED MESSAGES", bg="#0d1117",
                     fg="#8b949e", font=small_fnt).pack(anchor="w")
            self.log_text = scrolledtext.ScrolledText(
                log_frame, height=6, bg="#161b22", fg="#e6edf3",
                insertbackground="white", font=("Courier", 9), wrap=tk.WORD,
                highlightbackground="#30363d", highlightthickness=1
            )
            self.log_text.pack(fill="x", pady=(2, 0))
            self.log_text.config(state=tk.DISABLED)

            sensor_bar = tk.Frame(bottom_frame, bg="#0d1117")
            sensor_bar.pack(fill="x", padx=16, pady=(0, 12))
            tk.Label(sensor_bar, text="SENSOR READINGS (Edit fields to simulate values)", bg="#0d1117",
                     fg="#8b949e", font=small_fnt).pack(anchor="w")
            sensor_row = tk.Frame(sensor_bar, bg="#0d1117")
            sensor_row.pack(fill="x")
            self.sensor_labels = {}
            self.sensor_entries = {}
            sensor_defs = [
                ("ultrasonic", "ULTRASONIC",          "cm", "999"),
                ("mq4",        "MQ4 (METHANE)",        "",   "-1"),
                ("mq137",      "MQ137 (AMMONIA)",      "",   "-1"),
                ("mq135",      "MQ135 (AIR QUALITY)",  "",   "-1"),
            ]
            for key, name, unit, default_val in sensor_defs:
                cell = tk.Frame(sensor_row, bg="#161b22", highlightbackground="#30363d",
                                highlightthickness=1)
                cell.pack(side="left", expand=True, fill="x", padx=4)
                tk.Label(cell, text=name, bg="#161b22", fg="#e6edf3",
                         font=small_fnt).pack(pady=(4, 0))
                val_lbl = tk.Label(cell, text=default_val, bg="#161b22",
                                   fg="#58a6ff", font=sensor_val_fnt)
                val_lbl.pack(pady=(1, 2))
                entry = tk.Entry(cell, bg="#0d1117", fg="#e6edf3", insertbackground="white",
                                 font=small_fnt, justify="center", width=8)
                entry.insert(0, default_val)
                entry.pack(pady=(0, 4))
                self.sensor_labels[key] = (val_lbl, unit)
                self.sensor_entries[key] = entry

            compass = tk.Frame(root, bg="#0d1117")
            compass.pack(fill="both", expand=True, padx=14, pady=6)
            for i in range(3):
                compass.grid_rowconfigure(i, weight=1, uniform="card")
                compass.grid_columnconfigure(i, weight=1, uniform="card")

            self.cards = {}
            self.distance_entries = {}
            layout = [
                ("FRONT_LEFT",  0, 0), ("FRONT",       0, 1), ("FRONT_RIGHT", 0, 2),
                ("LEFT",        1, 0),                         ("RIGHT",       1, 2),
                ("BACK_LEFT",   2, 0), ("BACK",         2, 1), ("BACK_RIGHT",  2, 2),
            ]
            for side, row, col in layout:
                card = tk.Frame(compass, bg="#161b22", highlightbackground="#30363d",
                                highlightthickness=1, width=self.BASE_CARD_W, height=self.BASE_CARD_H)
                card.grid(row=row, column=col, padx=6, pady=6, sticky="nsew")
                card.grid_propagate(False)
                lbl = tk.Label(card, text=self.ARROWS[side], bg="#161b22",
                               fg="#e6edf3", font=side_fnt)
                lbl.place(relx=0.05, rely=0.06)
                self._track_font(lbl, side_fnt)
                alert_lbl = tk.Label(card, text="", bg="#f85149", fg="#ffffff",
                                     font=tkfont.Font(family="Helvetica", size=8, weight="bold"),
                                     padx=4, pady=1)
                alert_lbl.place_forget()
                val = tk.Label(card, text="100.0 cm", bg="#161b22", fg="#58a6ff", font=val_fnt)
                val.place(relx=0.05, rely=0.24)
                self._track_font(val, val_fnt)
                bar_canvas = tk.Canvas(card, bg="#30363d", bd=0, highlightthickness=0)
                bar_canvas.place(relx=0.05, rely=0.55, relwidth=0.9, relheight=0.07)
                bar_fill = bar_canvas.create_rectangle(0, 0, 0, 0, fill="#58a6ff", outline="")
                avg_lbl = tk.Label(card, text="avg: 100.0cm", bg="#161b22", fg="#8b949e", font=small_fnt)
                avg_lbl.place(relx=0.05, rely=0.68)
                self._track_font(avg_lbl, small_fnt)
                pts_lbl = tk.Label(card, text="pts: 6", bg="#161b22", fg="#8b949e", font=small_fnt)
                pts_lbl.place(relx=0.63, rely=0.68)
                self._track_font(pts_lbl, small_fnt)

                dist_entry = tk.Entry(card, bg="#0d1117", fg="#e6edf3", insertbackground="white",
                                      font=small_fnt, justify="center", width=7)
                dist_entry.insert(0, "100")
                dist_entry.place(relx=0.5, rely=0.92, anchor="s", relwidth=0.7)
                self.distance_entries[side] = dist_entry

                self.cards[side] = {
                    "val": val, "bar_canvas": bar_canvas, "bar_fill": bar_fill,
                    "avg": avg_lbl, "pts": pts_lbl, "card": card, "alert": alert_lbl,
                }

            center_card = tk.Frame(compass, bg="#161b22", highlightbackground="#30363d",
                                   highlightthickness=1, width=self.BASE_CARD_W, height=self.BASE_CARD_H)
            center_card.grid(row=1, column=1, padx=6, pady=6, sticky="nsew")
            center_card.grid_propagate(False)
            self.center_card = center_card
            tk.Label(center_card, text="🤖", bg="#161b22",
                     font=("Helvetica", 36)).place(relx=0.5, rely=0.4, anchor="center")
            tk.Label(center_card, text="GARBY (SIM)", bg="#161b22",
                     fg="#8b949e", font=small_fnt).place(relx=0.5, rely=0.78, anchor="center")

            compass.bind('<Configure>', self._on_compass_configure)
            self._base_card_w = self.BASE_CARD_W
            self._base_card_h = self.BASE_CARD_H
            self._compass = compass

            self._process_updates()
            self._process_ble_recv()

        def toggle_fullscreen(self, event=None):
            current = self.root.attributes('-fullscreen')
            self.root.attributes('-fullscreen', not current)

        def _toggle_front(self):
            if front_disabled.is_set():
                front_disabled.clear()
                self.front_btn.config(text="FRONT: ENABLED", bg="#238636", activebackground="#2ea043")
            else:
                front_disabled.set()
                self.front_btn.config(text="FRONT: DISABLED", bg="#f85149", activebackground="#ff6b63")

        def _toggle_back(self):
            if back_disabled.is_set():
                back_disabled.clear()
                self.back_btn.config(text="BACK: ENABLED", bg="#238636", activebackground="#2ea043")
            else:
                back_disabled.set()
                self.back_btn.config(text="BACK: DISABLED", bg="#f85149", activebackground="#ff6b63")

        def _track_font(self, widget, font_src):
            if isinstance(font_src, tkfont.Font):
                self._scalable_fonts.append((
                    widget, font_src.cget('family'),
                    font_src.cget('size'), font_src.cget('weight')
                ))
            elif isinstance(font_src, tuple):
                family = font_src[0]; size = font_src[1]
                weight = font_src[2] if len(font_src) > 2 else 'normal'
                self._scalable_fonts.append((widget, family, size, weight))

        def _on_compass_configure(self, event):
            w = event.width; h = event.height
            card_w = max(10, (w - 36) // 3)
            card_h = max(10, (h - 36) // 3)
            for info in self.cards.values():
                info['card'].config(width=card_w, height=card_h)
            self.center_card.config(width=card_w, height=card_h)
            scale = card_w / self._base_card_w if self._base_card_w else 1.0
            for widget, family, base_size, weight in self._scalable_fonts:
                new_size = max(1, int(base_size * scale))
                try:
                    widget.config(font=(family, new_size, weight))
                except Exception:
                    pass

        def get_simulated_distances(self):
            """Read distances (cm) from the entry fields, default to 100.0 if empty/invalid."""
            dists = {}
            for side, entry in self.distance_entries.items():
                val_str = entry.get().strip()
                if val_str == "":
                    dists[side] = 100.0
                else:
                    try:
                        dists[side] = float(val_str)
                    except ValueError:
                        dists[side] = 100.0
            return dists

        def _sync_sensor_entries(self):
            """Sync manual entry fields to latest_sensor_readings dict."""
            with sensor_readings_lock:
                # Ultrasonic
                us_str = self.sensor_entries["ultrasonic"].get().strip()
                try:
                    latest_sensor_readings["ultrasonic"] = float(us_str) if us_str != "" else 999.0
                except ValueError:
                    latest_sensor_readings["ultrasonic"] = 999.0

                # Gas sensors
                for k in ("mq4", "mq137", "mq135"):
                    v_str = self.sensor_entries[k].get().strip()
                    try:
                        latest_sensor_readings[k] = int(v_str) if v_str != "" else -1
                    except ValueError:
                        latest_sensor_readings[k] = -1

        def _process_updates(self):
            self._sync_sensor_entries()
            dists = self.get_simulated_distances()
            if self.sim_node:
                self.sim_node.update_distances(dists)

            try:
                data, range_min, range_max, cone_deg, sides_status = self.data_queue.get_nowait()
                self.update(data, range_min, range_max, cone_deg, sides_status)
            except queue.Empty:
                pass
            self._update_sensor_readings_display()
            self.root.after(100, self._process_updates)

        def _process_ble_recv(self):
            while True:
                try:
                    msg = self.recv_queue.get_nowait()
                except queue.Empty:
                    break
                timestamp = datetime.now().strftime("%H:%M:%S")
                line = f"[{timestamp}] {msg}\n"
                self.log_text.config(state=tk.NORMAL)
                self.log_text.insert(tk.END, line)
                self.log_text.see(tk.END)
                self.log_text.config(state=tk.DISABLED)
            self.root.after(200, self._process_ble_recv)

        def _update_sensor_readings_display(self):
            with sensor_readings_lock:
                readings = dict(latest_sensor_readings)
            for key, (lbl, unit) in self.sensor_labels.items():
                val = readings.get(key)
                if val is None:
                    if key == "ultrasonic":
                        lbl.config(text=f"999 {unit}".strip(), fg="#d29922")
                    else:
                        lbl.config(text="-1", fg="#d29922")
                elif key == "ultrasonic":
                    lbl.config(text=f"{val:.1f} {unit}", fg="#58a6ff")
                else:
                    color = "#f85149" if val >= 700 else "#58a6ff"
                    lbl.config(text=f"{val}", fg=color)

        def set_ble_status(self, text, color):
            self.root.after(0, lambda: self.ble_status_lbl.config(text=f"● {text}", fg=color))

        def update(self, data, range_min, range_max, cone_deg, sides_status=None):
            self.status_lbl.config(text="● Live (sim)", fg="#3fb950")

            def dist_color_cm(d):
                return "#3fb950" if d <= 200 else ("#d29922" if d <= 400 else "#f85149")

            for side, info in data.items():
                if side in ("LEFT", "RIGHT"):
                    continue
                w = self.cards[side]
                if info is None:
                    w["val"].config(text="N/A", fg="#8b949e")
                    w["bar_canvas"].coords(w["bar_fill"], 0, 0, 0, 0)
                    w["avg"].config(text="avg: —"); w["pts"].config(text="pts: —")
                    w["card"].config(highlightbackground="#30363d")
                    if side in self.ALERT_SIDES: w["alert"].place_forget()
                    continue
                min_d, avg_d, count = info
                bar_w = w["bar_canvas"].winfo_width(); bar_h = w["bar_canvas"].winfo_height()
                ratio = min(min_d / range_max, 1.0) if range_max > 0 else 0
                min_cm = min_d * 100; avg_cm = avg_d * 100
                color = dist_color_cm(min_cm)
                w["val"].config(text=f"{min_cm:.1f} cm", fg=color)
                w["bar_canvas"].coords(w["bar_fill"], 0, 0, int(ratio * bar_w), bar_h)
                w["bar_canvas"].itemconfig(w["bar_fill"], fill=color)
                w["avg"].config(text=f"avg: {avg_cm:.1f}cm"); w["pts"].config(text=f"pts: {count}")
                alert = False; threshold = None
                if side in self.FRONT_GROUP:
                    threshold = THRESHOLD_CM; alert = (min_cm <= THRESHOLD_CM) and not front_disabled.is_set()
                elif side in self.BACK_GROUP:
                    threshold = BACK_THRESHOLD_CM; alert = (min_cm <= BACK_THRESHOLD_CM) and not back_disabled.is_set()
                if (side in self.FRONT_GROUP and front_disabled.is_set()) or \
                   (side in self.BACK_GROUP  and back_disabled.is_set()):
                    w["card"].config(highlightbackground="#8b949e")
                else:
                    w["card"].config(highlightbackground="#f85149" if alert else "#3fb950")
                if side in self.ALERT_SIDES:
                    if alert:
                        w["alert"].config(text=f"⚠ ≤{threshold:.0f}cm")
                        w["alert"].place(relx=0.5, rely=0.04, anchor="n")
                    else:
                        w["alert"].place_forget()

            left_info  = data.get("LEFT")
            right_info = data.get("RIGHT")
            left_cm    = left_info[0]  * 100 if left_info  else None
            right_cm   = right_info[0] * 100 if right_info else None

            nudge = None
            if left_cm is not None and right_cm is not None:
                diff = left_cm - right_cm
                if abs(diff) <= SIDES_TOLERANCE: nudge = "stable"
                elif diff < 0:                   nudge = "nudge_right"
                else:                            nudge = "nudge_left"

            if sides_status is None:
                if   nudge == "stable":       sides_status = "SIDES:STABLE"
                elif nudge == "nudge_right":  sides_status = "SIDES:NUDGE-RIGHT"
                elif nudge == "nudge_left":   sides_status = "SIDES:NUDGE-LEFT"

            for side in ("LEFT", "RIGHT"):
                info = data.get(side); w = self.cards[side]
                if info is None:
                    w["val"].config(text="N/A", fg="#8b949e")
                    w["bar_canvas"].coords(w["bar_fill"], 0, 0, 0, 0)
                    w["avg"].config(text="avg: —"); w["pts"].config(text="pts: —")
                    w["card"].config(highlightbackground="#30363d"); continue
                min_d, avg_d, count = info
                bar_w = w["bar_canvas"].winfo_width(); bar_h = w["bar_canvas"].winfo_height()
                ratio = min(min_d / range_max, 1.0) if range_max > 0 else 0
                min_cm = min_d * 100; avg_cm = avg_d * 100
                if   sides_status == "SIDES:STABLE":       fg = "#3fb950"
                elif sides_status == "SIDES:NUDGE-RIGHT":  fg = "#f85149" if side == "LEFT"  else "#3fb950"
                elif sides_status == "SIDES:NUDGE-LEFT":   fg = "#f85149" if side == "RIGHT" else "#3fb950"
                else:                                      fg = dist_color_cm(min_cm)
                w["val"].config(text=f"{min_cm:.1f} cm", fg=fg)
                w["bar_canvas"].coords(w["bar_fill"], 0, 0, int(ratio * bar_w), bar_h)
                w["bar_canvas"].itemconfig(w["bar_fill"], fill=fg)
                w["avg"].config(text=f"avg: {avg_cm:.1f}cm"); w["pts"].config(text=f"pts: {count}")
                if sides_status and "NUDGE" in sides_status:
                    w["card"].config(highlightbackground="#d29922")
                elif sides_status == "SIDES:STABLE":
                    w["card"].config(highlightbackground="#3fb950")
                else:
                    w["card"].config(highlightbackground="#30363d")

            self.footer.config(
                text=(f"range: {range_min*100:.1f} – {range_max*100:.1f} cm | "
                      "manual direction cards | not safety certification")
            )

    return LidarGUI

# ═══ Headless key listener ═══
def start_headless_key_listener():
    if not sys.stdin.isatty():
        logger.info("stdin is not a TTY — headless front/back key toggles disabled.")
        return
    import tty, termios

    def _listener():
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while True:
                ch = sys.stdin.read(1)
                if not ch: continue
                ch = ch.lower()
                if ch == 'f':
                    if front_disabled.is_set(): front_disabled.clear(); logger.info("FRONT re-enabled.")
                    else: front_disabled.set(); logger.info("FRONT disabled.")
                elif ch == 'b':
                    if back_disabled.is_set(): back_disabled.clear(); logger.info("BACK re-enabled.")
                    else: back_disabled.set(); logger.info("BACK disabled.")
                elif ch == 'q':
                    os.kill(os.getpid(), signal.SIGINT); return
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    threading.Thread(target=_listener, daemon=True, name="key-listener").start()

# ═══ Main ═══
def main():
    import argparse
    parser = argparse.ArgumentParser(description="Combined Sensor + LiDAR Application (SIMULATION)")
    parser.add_argument("--headless", action="store_true", help="Run without GUI")
    args = parser.parse_args()

    global firebase, ble_service
    firebase = FirebaseManager()
    kill_gpio_users()
    time.sleep(1)

    existing = firebase.get_raspi_launch_time()
    if not existing:
        launch_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        firebase.update_raspi_states(launch_time=launch_time)

    executor = ExecutorService(workers=2)
    executor.submit(lambda: simulated_sensor_updater(serial_stop), permanent=True)

    data_queue = queue.Queue(maxsize=2)
    ble_service = None
    node = SimulatedLidarReader(data_queue, ble_service)

    executor.submit(lambda: on_demand_status_responder(node, serial_stop), permanent=True)

    ble = None
    gui_root = None
    shutdown_started = threading.Event()

    def shutdown():
        if shutdown_started.is_set():
            return
        shutdown_started.set()
        logger.info("Shutting down...")

        def _force_kill():
            time.sleep(5.0)
            logger.warning("Shutdown taking too long — forcing process exit.")
            os._exit(1)
        threading.Thread(target=_force_kill, daemon=True, name="shutdown-watchdog").start()

        serial_stop.set()

        try:
            if ble: ble.disconnect_client()
        except Exception as e:
            logger.error(f"BLE cleanup error: {e}")

        if gui_root is not None:
            try:
                gui_root.quit()
                gui_root.destroy()
            except Exception as e:
                logger.error(f"GUI teardown error: {e}")

        logger.info("Shutdown complete.")
        os._exit(0)

    signal.signal(signal.SIGINT,  lambda s, f: shutdown())
    signal.signal(signal.SIGTERM, lambda s, f: shutdown())

    if args.headless:
        def ble_cb_status(text, color):
            pass
        ble = BleService(ble_cb_status, ble_notify_handler)
        ble.start()
        ble_service = ble
        node.ble_service = ble
        start_headless_key_listener()
        try:
            while True:
                # In headless simulation, update distances to default 100.0 cm continuously
                node.update_distances({s: 100.0 for s in SimulatedLidarReader.SIDES})
                try:
                    ble_recv_queue.get_nowait()  # drain silently
                except queue.Empty:
                    pass
                time.sleep(0.1)
        except KeyboardInterrupt:
            shutdown()
    else:
        import tkinter as tk
        root = tk.Tk()
        gui_root = root
        LidarGUI = init_gui(node)
        gui = LidarGUI(root, data_queue, ble_recv_queue)

        def ble_cb_status(text, color):
            gui.set_ble_status(text, color)

        ble = BleService(ble_cb_status, ble_notify_handler)
        ble.start()
        ble_service = ble
        node.ble_service = ble
        root.protocol("WM_DELETE_WINDOW", shutdown)
        try:
            root.mainloop()
        except KeyboardInterrupt:
            shutdown()

if __name__ == '__main__':
    main()
