#!/usr/bin/env python3
"""
Combined application – Fast LiDAR + BLE with immediate obstacle push.
"""

import os, sys, time, queue, threading, asyncio, random, math, logging, signal, subprocess, glob
from collections import deque
from datetime import datetime
from statistics import median

from bridge_core import (
    CoalescingUpdateWorker,
    ExponentialBackoff,
    SENSOR_SPECS,
    SensorTransitionTracker,
    build_health_status,
    clamp_heading_error_cm,
    format_lidar_blockage,
    lidar_status_code,
    partition_lidar_samples,
    robust_near_distance_cm,
    robust_wall_distance_cm,
    select_steering_samples,
)

import serial
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
# Firebase is operational telemetry, not a motion prerequisite. If its SDK is
# absent, keep LiDAR/BLE safety alive and report the integration as disabled.
try:
    # pyrefly: ignore [missing-import]
    import firebase_admin
    # pyrefly: ignore [missing-import]
    from firebase_admin import credentials, db
    FIREBASE_IMPORT_ERROR = None
except ImportError as exc:
    firebase_admin = None
    credentials = None
    db = None
    FIREBASE_IMPORT_ERROR = exc

class FirebaseManager:
    """Non-blocking, atomic Firebase RTDB bridge.

    Every write is a root-level multi-location update. Calls made by BLE, UART,
    and ROS threads only coalesce into a bounded in-memory map; one dedicated
    writer performs network I/O and retries failures with exponential backoff.
    """

    DEFAULT_CREDENTIALS = "/home/garby/Desktop/garby-thesis-firebase-adminsdk-fbsvc-54fb448489.json"
    DEFAULT_DATABASE_URL = (
        "https://garby-thesis-default-rtdb.asia-southeast1.firebasedatabase.app"
    )

    def __init__(self, credential_path=None, database_url=None):
        if firebase_admin is None or credentials is None or db is None:
            raise RuntimeError(f"firebase-admin unavailable: {FIREBASE_IMPORT_ERROR}")
        credential_path = credential_path or os.environ.get(
            "GARBY_FIREBASE_CREDENTIALS", self.DEFAULT_CREDENTIALS
        )
        database_url = database_url or os.environ.get(
            "GARBY_FIREBASE_DATABASE_URL", self.DEFAULT_DATABASE_URL
        )
        if not os.path.isfile(credential_path):
            raise FileNotFoundError(f"Firebase credential file not found: {credential_path}")

        if not firebase_admin._apps:
            cred = credentials.Certificate(credential_path)
            firebase_admin.initialize_app(cred, {"databaseURL": database_url})
        self._root = db.reference("/")
        self._last_write_error_log = 0.0
        self._status_lock = threading.Lock()
        self._last_status_fields = {}
        self._writer = CoalescingUpdateWorker(self._commit_root_update)

    def _commit_root_update(self, updates):
        try:
            self._root.update(dict(updates))
        except Exception as exc:
            now = time.monotonic()
            if now - self._last_write_error_log >= 15.0:
                logger.error("Firebase write failed; retrying with backoff: %s", exc)
                self._last_write_error_log = now
            raise

    def _update(self, updates, *, urgent=False):
        self._writer.enqueue(updates, urgent=urgent)

    def close(self):
        self._writer.close(flush=True, timeout=3.0)

    def update_app(self, reset_state):
        self._update({"APP/resetState": reset_state}, urgent=True)

    def get_reset_command(self, device_id):
        app_ready = db.reference("/APP/isReadyToReset").get()
        command = db.reference(f"/devices/{device_id}/commands/reset").get() or {}
        return app_ready is True, command

    def mark_reset_ack(self, device_id, requested_at):
        prefix = f"devices/{device_id}/commands/reset"
        self._update({
            f"{prefix}/requestedAt": requested_at,
            f"{prefix}/status": "ack",
            f"{prefix}/ackAt": int(time.time() * 1000),
        }, urgent=True)

    def mark_reset_done(self, device_id, requested_at):
        prefix = f"devices/{device_id}/commands/reset"
        self._update({
            f"{prefix}/requestedAt": requested_at,
            f"{prefix}/status": "done",
            f"{prefix}/doneAt": int(time.time() * 1000),
            "APP/isReadyToReset": False,
        }, urgent=True)

    def mark_reset_failed(self, device_id, requested_at, reason):
        prefix = f"devices/{device_id}/commands/reset"
        self._update({
            f"{prefix}/requestedAt": requested_at,
            f"{prefix}/status": "failed",
            f"{prefix}/failedAt": int(time.time() * 1000),
            f"{prefix}/reason": reason,
            "APP/isReadyToReset": False,
        }, urgent=True)

    def update_mcu_states(self, is_blocked=None, is_fully_loaded=None,
                          is_sim_registered=None, is_started=None):
        updates = {}
        if is_blocked is not None:        updates["isBlocked"] = is_blocked
        if is_fully_loaded is not None:   updates["isFullyLoaded"] = is_fully_loaded
        if is_sim_registered is not None: updates["isSimModuleRegistered"] = is_sim_registered
        if is_started is not None:        updates["isStarted"] = is_started
        self._update({f"MCU/STATES/{key}": value for key, value in updates.items()})

    def update_mcu_load_cell(self, value):
        updated_at = int(time.time() * 1000)
        self._update({
            "MCU/VALUES/LOAD_CELL": value,
            "MCU/VALUES/updatedAt": updated_at,
            "RASPI/VALUES/LOAD_CELL/WEIGHT_IN_KG": value,
            "RASPI/VALUES/LOAD_CELL/updatedAt": updated_at,
            f"devices/{RESET_DEVICE_ID}/sensors/weight/value": value,
            f"devices/{RESET_DEVICE_ID}/sensors/weight/unit": "kg",
            f"devices/{RESET_DEVICE_ID}/sensors/weight/sensorType": "LOAD_CELL",
            f"devices/{RESET_DEVICE_ID}/sensors/weight/updatedAt": updated_at
        })

    def get_raspi_launch_time(self):
        return db.reference("/RASPI/STATES/launchTime").get()

    def update_raspi_states(self, launch_time):
        self._update({"RASPI/STATES/launchTime": launch_time}, urgent=True)

    def update_raspi_heartbeat(self, status_fields=None):
        now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        last_seen = int(time.time() * 1000)
        updates = {
            "RASPI/STATES/recent_uptime": now,
            "RASPI/STATES/lastSeen": last_seen,
            f"devices/{RESET_DEVICE_ID}/status/recent_uptime": now,
            f"devices/{RESET_DEVICE_ID}/status/lastSeen": last_seen,
        }
        # Diagnostics are change-only. lastSeen remains a low-rate heartbeat.
        if status_fields:
            with self._status_lock:
                for key, value in status_fields.items():
                    if value is not None and self._last_status_fields.get(key) != value:
                        updates[f"devices/{RESET_DEVICE_ID}/status/{key}"] = value
                        self._last_status_fields[key] = value
        self._update(updates)

    def update_raspi_sensors(self, air_quality=None, ammonia=None,
                             methane=None, ultrasonic_distance=None):
        updated_at = int(time.time() * 1000)
        updates = {}
        if ultrasonic_distance is not None:
            updates.update({
                "RASPI/VALUES/ULTRASONIC_SENSOR/CM_DISTANCE": ultrasonic_distance,
                "RASPI/VALUES/ULTRASONIC_SENSOR/updatedAt": updated_at,
                f"devices/{RESET_DEVICE_ID}/sensors/level/value": ultrasonic_distance,
                f"devices/{RESET_DEVICE_ID}/sensors/level/unit": "cm",
                f"devices/{RESET_DEVICE_ID}/sensors/level/sensorType": "ULTRASONIC_SENSOR",
                f"devices/{RESET_DEVICE_ID}/sensors/level/updatedAt": updated_at
            })
        if air_quality is not None:
            updates.update({
                "RASPI/VALUES/MQ135_SENSOR/AIR_QUALITY": air_quality,
                "RASPI/VALUES/MQ135_SENSOR/updatedAt": updated_at,
                f"devices/{RESET_DEVICE_ID}/sensors/mq135/value": air_quality,
                f"devices/{RESET_DEVICE_ID}/sensors/mq135/unit": "ppm",
                f"devices/{RESET_DEVICE_ID}/sensors/mq135/sensorType": "MQ135_SENSOR",
                f"devices/{RESET_DEVICE_ID}/sensors/mq135/updatedAt": updated_at
            })
        if ammonia is not None:
            updates.update({
                "RASPI/VALUES/MQ137/AMMONIA": ammonia,
                "RASPI/VALUES/MQ137/updatedAt": updated_at,
                f"devices/{RESET_DEVICE_ID}/sensors/mq137/value": ammonia,
                f"devices/{RESET_DEVICE_ID}/sensors/mq137/unit": "ppm",
                f"devices/{RESET_DEVICE_ID}/sensors/mq137/sensorType": "MQ137",
                f"devices/{RESET_DEVICE_ID}/sensors/mq137/updatedAt": updated_at
            })
        if methane is not None:
            updates.update({
                "RASPI/VALUES/MQ4_SENSOR/METHANE": methane,
                "RASPI/VALUES/MQ4_SENSOR/updatedAt": updated_at,
                f"devices/{RESET_DEVICE_ID}/sensors/mq4/value": methane,
                f"devices/{RESET_DEVICE_ID}/sensors/mq4/unit": "ppm",
                f"devices/{RESET_DEVICE_ID}/sensors/mq4/sensorType": "MQ4_SENSOR",
                f"devices/{RESET_DEVICE_ID}/sensors/mq4/updatedAt": updated_at
            })
        if updates:
            self._update(updates)

# ═══ Executor Service ═══
from threading import Thread
from queue import Queue

class Task:
    def __init__(self, func, permanent=False):
        self.func = func
        self.permanent = permanent
        self.restart_backoff = ExponentialBackoff(1.0, 30.0)

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
            restart = False
            try:
                task.func()
                restart = task.permanent and not serial_stop.is_set()
            except Exception:
                import traceback
                traceback.print_exc()
                restart = task.permanent and not serial_stop.is_set()
            finally:
                if restart:
                    delay = task.restart_backoff.next_delay()
                    logger.warning("Background task exited; restart in %.1f s", delay)
                    serial_stop.wait(delay)
                    self.queue.put(task)
                self.queue.task_done()

# ═══ Configuration ═══
def _resolve_sensor_serial_port():
    configured = os.environ.get("GARBY_SENSOR_SERIAL_PORT")
    if configured:
        return configured
    for candidate in ("/dev/serial0", "/dev/ttyAMA0", "/dev/ttyS0", "/dev/ttyUSB1", "/dev/ttyACM0"):
        if os.path.exists(candidate):
            return candidate
    return "/dev/ttyAMA0"

SERIAL_PORT = _resolve_sensor_serial_port()
SERIAL_BAUD = int(os.environ.get("GARBY_SENSOR_SERIAL_BAUD", "9600"))

SERVICE_UUID     = os.environ.get("GARBY_BLE_SERVICE_UUID", "4fafc201-1fb5-459e-8fcc-c5c9c331914b")
WRITE_CHAR_UUID  = os.environ.get("GARBY_BLE_WRITE_CHAR_UUID", "beb5483e-36e1-4688-b7f5-ea07361b26a8")
NOTIFY_CHAR_UUID = os.environ.get("GARBY_BLE_NOTIFY_CHAR_UUID", "beb5483e-36e1-4688-b7f5-ea07361b26a9")
DEVICE_NAME      = os.environ.get("GARBY_BLE_DEVICE_NAME", "GarbyESP32")
BLE_MAC_ADDRESS  = os.environ.get("GARBY_BLE_ADDRESS", None)

THRESHOLD_CM      = 95.0                  # front stop distance
BACK_THRESHOLD_CM = 35.0                  # back stop distance

# Safety/transport timing. A 10 Hz LiDAR should update about every 100 ms;
# 0.8 s therefore allows several missed scans but never permits seconds of
# blind travel. Telemetry is intentionally slower than the safety stream.
LIDAR_STALE_TIMEOUT_S = 0.8
SENSOR_TX_PERIOD_S    = 1.0
SENSOR_OFFLINE_DB_DELAY_S = 8.0
SENSOR_DB_PERIOD_S = 5.0
RASPI_HEARTBEAT_PERIOD_S = 30.0
FIREBASE_RESET_POLL_S = 2.0
LOAD_CELL_CHANGE_KG = 0.05
LOAD_CELL_REFRESH_S = 30.0
ALLOW_RUNTIME_SAFETY_DISABLE = False

# Software LiDAR convention is 0 deg FRONT / 90 deg LEFT. Keep the physical
# mounting correction configurable instead of silently baking in an unverified
# chassis orientation. 180 reproduces the old opposite-facing convention.
try:
    LIDAR_YAW_OFFSET_DEG = float(os.environ.get("GARBY_LIDAR_YAW_OFFSET_DEG", "0"))
    if not math.isfinite(LIDAR_YAW_OFFSET_DEG):
        raise ValueError
except ValueError:
    LIDAR_YAW_OFFSET_DEG = 0.0
    logger.warning("Invalid GARBY_LIDAR_YAW_OFFSET_DEG; using 0 degrees")

SIDES_TOLERANCE   = 5.0
BLE_CMD_PERIOD    = 0.25

REQUEST_TOKEN = "[REQUEST-STATUS]"
RASPI_READY   = "[RASPI READY]"
RESET_DEVICE_ID = "garby-bin-01"
RESET_REQUEST_MAX_AGE_MS = 120_000
RESET_REQUEST_FUTURE_TOLERANCE_MS = 30_000

firebase = None
ser = None
ultrasonic = None
mq4 = mq137 = mq135 = None

latest_sensor_readings = {"ultrasonic": None, "mq4": None, "mq137": None, "mq135": None}
sensor_readings_lock = threading.Lock()
sensor_tracker = SensorTransitionTracker(
    stale_after_s=SENSOR_OFFLINE_DB_DELAY_S,
    live_publish_period_s=SENSOR_DB_PERIOD_S,
)

serial_stop = threading.Event()
ble_connected = threading.Event()
lidar_healthy = threading.Event()
sensor_serial_connected = threading.Event()
_load_cell_lock = threading.Lock()
_last_load_cell_value = None
_last_load_cell_publish = 0.0

class CoalescingBleQueue:
    """Small latest-value mailbox for BLE traffic.

    PATH, SIDES, and SENSOR messages replace older messages of the same kind,
    preventing stale clear-path packets from accumulating behind newer STOP
    packets. Control messages are retained. Urgent safety messages are placed
    at the front of the deque.
    """
    def __init__(self, maxlen=12):
        self._maxlen = maxlen
        self._items = deque()
        self._lock = threading.Lock()

    @staticmethod
    def _kind(msg):
        if msg.startswith(("P:", "PATH:")):
            return "PATH"
        if msg.startswith(("S:", "SIDES:")):
            return "SIDES"
        if msg.startswith("SENSOR:"):
            return "SENSOR"
        if msg == "[RESET]":
            return "RESET"
        if msg == RASPI_READY:
            return "READY"
        return "CONTROL"

    def put_nowait(self, msg, urgent=False):
        kind = self._kind(msg)
        with self._lock:
            if kind != "CONTROL":
                self._items = deque(
                    item for item in self._items if self._kind(item) != kind
                )
            while len(self._items) >= self._maxlen:
                # Never discard the next urgent PATH/RESET just to retain stale
                # steering or telemetry. Search from the tail (newest/lowest
                # priority) for an expendable item first.
                remove_index = None
                for index in range(len(self._items) - 1, -1, -1):
                    if self._kind(self._items[index]) in ("SIDES", "SENSOR", "READY", "CONTROL"):
                        remove_index = index
                        break
                if remove_index is None:
                    self._items.pop()
                else:
                    del self._items[remove_index]
            if urgent:
                self._items.appendleft(msg)
            else:
                self._items.append(msg)

    def get_nowait(self):
        with self._lock:
            if not self._items:
                raise queue.Empty
            return self._items.popleft()

    def clear(self):
        with self._lock:
            self._items.clear()

    def discard_kind(self, kind):
        with self._lock:
            self._items = deque(
                item for item in self._items if self._kind(item) != kind
            )

ble_send_queue = CoalescingBleQueue(maxlen=12)
ble_recv_queue = queue.Queue(maxsize=100)

_status_requested = threading.Event()
_robot_running = threading.Event()
_reset_completed = threading.Event()
front_disabled = threading.Event()
back_disabled  = threading.Event()

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S"
)
logger = logging.getLogger("combined_app")

# Module‑level ble_service placeholder – will be set after BLE connects
ble_service = None

# ═══ Serial Sensor Reader ═══
_SERIAL_LABEL_TO_KEY = {
    "ULTRASONIC": "ultrasonic",
    "MQ4": "mq4",
    "MQ137": "mq137",
    "MQ135": "mq135",
}

_SENSOR_FIREBASE_KEYS = {
    "ultrasonic": "ultrasonic_distance",
    "mq4": "methane",
    "mq137": "ammonia",
    "mq135": "air_quality",
}


def _sync_transport_sensor_values():
    values = sensor_tracker.latest_for_transport()
    with sensor_readings_lock:
        latest_sensor_readings.update(values)


def _queue_sensor_batch(batch):
    if not batch or not firebase:
        return False
    kwargs = {_SENSOR_FIREBASE_KEYS[key]: value for key, value in batch.items()}
    firebase.update_raspi_sensors(**kwargs)
    return True


def receiveSerial(stop_event):
    """Read UART sensors with reconnect/backoff and per-sensor liveness."""
    global ser, ultrasonic, mq4, mq137, mq135
    logger.info("Serial sensor bridge started.")
    reconnect_backoff = ExponentialBackoff(
        1.0,
        30.0,
        jitter=lambda delay: delay * random.uniform(0.9, 1.1),
    )
    pending_firebase = {}
    last_protocol_line = 0.0

    while not stop_event.is_set():
        if ser is None:
            try:
                ser = serial.Serial(
                    port=SERIAL_PORT,
                    baudrate=SERIAL_BAUD,
                    timeout=0.25,
                    write_timeout=0.5,
                )
                try:
                    ser.reset_input_buffer()
                except Exception:
                    pass
                reconnect_backoff.reset()
                logger.info("Opened sensor serial %s at %d baud.", SERIAL_PORT, SERIAL_BAUD)
            except Exception as exc:
                ser = None
                sensor_serial_connected.clear()
                sensor_tracker.mark_unavailable()
                pending_firebase.update(sensor_tracker.collect_due())
                _sync_transport_sensor_values()
                if pending_firebase and _queue_sensor_batch(pending_firebase):
                    pending_firebase.clear()
                delay = reconnect_backoff.next_delay()
                logger.warning("Sensor serial unavailable (%s); retry in %.1f s", exc, delay)
                stop_event.wait(delay)
                continue

        try:
            raw = ser.readline()
            now = time.monotonic()
            if raw:
                line = raw[:160].decode("utf-8", errors="replace").strip()
                label, separator, raw_value = line.partition(":")
                key = _SERIAL_LABEL_TO_KEY.get(label.strip().upper()) if separator else None
                if key is not None:
                    # Ignore corrupted numeric text without converting a single
                    # bad UART byte into an outage. Explicit -1/999 and numeric
                    # out-of-range values do represent unavailable hardware.
                    try:
                        numeric_value = float(raw_value.strip())
                    except ValueError:
                        numeric_value = None
                    if numeric_value is not None and math.isfinite(numeric_value):
                        is_valid = sensor_tracker.ingest(key, numeric_value, now=now)
                        last_protocol_line = now
                        sensor_serial_connected.set()
                        if is_valid:
                            value = SENSOR_SPECS[key].normalize(numeric_value)
                            if key == "ultrasonic":
                                ultrasonic = value
                            elif key == "mq4":
                                mq4 = value
                            elif key == "mq137":
                                mq137 = value
                            elif key == "mq135":
                                mq135 = value

            if last_protocol_line and now - last_protocol_line >= SENSOR_OFFLINE_DB_DELAY_S:
                sensor_serial_connected.clear()

            pending_firebase.update(sensor_tracker.collect_due(now=now))
            _sync_transport_sensor_values()
            if pending_firebase and _queue_sensor_batch(pending_firebase):
                pending_firebase.clear()
        except (serial.SerialException, OSError) as exc:
            logger.warning("Sensor serial link lost: %s", exc)
            sensor_serial_connected.clear()
            sensor_tracker.mark_unavailable()
            pending_firebase.update(sensor_tracker.collect_due())
            _sync_transport_sensor_values()
            if pending_firebase and _queue_sensor_batch(pending_firebase):
                pending_firebase.clear()
            try:
                ser.close()
            except Exception:
                pass
            ser = None
            stop_event.wait(reconnect_backoff.next_delay())
        except Exception as exc:
            # An unexpected parser bug must not create a tight crash/restart loop.
            logger.exception("Unexpected serial bridge error: %s", exc)
            stop_event.wait(0.5)

    if ser is not None:
        try:
            ser.close()
        except Exception:
            pass
        ser = None
    sensor_serial_connected.clear()
    logger.info("Serial sensor bridge stopped.")


def _read_cpu_temperature_c():
    for path in (
        "/sys/class/thermal/thermal_zone0/temp",
        "/sys/class/hwmon/hwmon0/temp1_input",
    ):
        try:
            with open(path, "r", encoding="ascii") as handle:
                raw = float(handle.read().strip())
            return raw / 1000.0 if raw > 200.0 else raw
        except (OSError, ValueError):
            continue
    return None


def _read_throttled_flags():
    try:
        result = subprocess.run(
            ["vcgencmd", "get_throttled"],
            capture_output=True,
            text=True,
            timeout=1.0,
            check=False,
        )
        text = result.stdout.strip().split("=", 1)[-1]
        return int(text, 16)
    except (OSError, ValueError, subprocess.SubprocessError):
        return None


def collect_health_status():
    cpu_temp = _read_cpu_temperature_c()
    throttled_flags = _read_throttled_flags()
    return build_health_status(
        cpu_temperature_c=cpu_temp,
        throttled_flags=throttled_flags,
        ble_connected=ble_connected.is_set(),
        lidar_healthy=lidar_healthy.is_set(),
        sensor_serial_connected=sensor_serial_connected.is_set(),
    )


def raspi_heartbeat_loop(stop_event):
    logger.info("RasPi heartbeat/health updater started.")
    while not stop_event.is_set():
        try:
            if firebase:
                firebase.update_raspi_heartbeat(collect_health_status())
        except Exception as e:
            logger.error(f"RasPi heartbeat update failed: {e}")
        stop_event.wait(RASPI_HEARTBEAT_PERIOD_S)
    logger.info("RasPi heartbeat updater stopped.")


def firebase_connection_loop(stop_event):
    """Initialize Firebase without making robot safety depend on the network."""
    global firebase
    if firebase_admin is None:
        logger.warning("Firebase disabled because firebase-admin is not installed: %s",
                       FIREBASE_IMPORT_ERROR)
        stop_event.wait()
        return
    backoff = ExponentialBackoff(
        2.0,
        60.0,
        jitter=lambda delay: delay * random.uniform(0.9, 1.1),
    )
    while not stop_event.is_set() and firebase is None:
        try:
            manager = FirebaseManager()
            firebase = manager
            try:
                existing = manager.get_raspi_launch_time()
                if not existing:
                    manager.update_raspi_states(
                        launch_time=datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                    )
            except Exception as exc:
                # SDK/network recovery is automatic; the write worker and reset
                # poller will retry without restarting the safety bridge.
                logger.warning("Firebase initial read unavailable: %s", exc)
            logger.info("Firebase bridge initialized.")
            break
        except Exception as exc:
            delay = backoff.next_delay()
            logger.error("Firebase initialization failed (%s); retry in %.1f s", exc, delay)
            stop_event.wait(delay)

    stop_event.wait()

def firebase_reset_command_loop(stop_event):
    logger.info("Firebase reset command bridge started.")
    in_flight_requested_at = 0
    completed_requested_at = None
    last_reset_relay_at = 0.0
    poll_backoff = ExponentialBackoff(2.0, 30.0)

    while not stop_event.is_set():
        wait_s = FIREBASE_RESET_POLL_S
        try:
            if not firebase:
                stop_event.wait(FIREBASE_RESET_POLL_S)
                continue

            app_ready, command = firebase.get_reset_command(RESET_DEVICE_ID)
            poll_backoff.reset()
            is_command = isinstance(command, dict)
            status = str(command.get("status", "")).lower() if is_command else ""
            requested_at_raw = command.get("requestedAt", 0) if is_command else 0
            try:
                requested_at = int(requested_at_raw or 0)
            except (TypeError, ValueError):
                requested_at = 0

            # Recover an acknowledged-but-incomplete reset after a Pi restart.
            should_send = app_ready or status in ("pending", "ack")
            now_ms = int(time.time() * 1000)
            if should_send and requested_at <= 0 and not app_ready:
                if requested_at != completed_requested_at:
                    firebase.mark_reset_failed(
                        RESET_DEVICE_ID,
                        requested_at,
                        "missing_reset_timestamp"
                    )
                    completed_requested_at = requested_at
                    logger.warning("[RESET] Rejected timestamp-less Firebase reset request.")
                stop_event.wait(FIREBASE_RESET_POLL_S)
                continue
            if should_send and requested_at <= 0:
                requested_at = in_flight_requested_at or now_ms
            age_ms = now_ms - requested_at
            request_is_fresh = (
                should_send and
                -RESET_REQUEST_FUTURE_TOLERANCE_MS <= age_ms <= RESET_REQUEST_MAX_AGE_MS
            )
            if should_send and not request_is_fresh:
                if requested_at != completed_requested_at:
                    firebase.mark_reset_failed(
                        RESET_DEVICE_ID,
                        requested_at,
                        "stale_or_future_reset_request"
                    )
                    logger.warning("[RESET] Rejected stale/future Firebase reset request.")
                completed_requested_at = requested_at
                if in_flight_requested_at == requested_at:
                    in_flight_requested_at = 0
                stop_event.wait(FIREBASE_RESET_POLL_S)
                continue

            is_new_request = (
                request_is_fresh and
                requested_at != in_flight_requested_at and
                requested_at != completed_requested_at
            )
            if is_new_request:
                _reset_completed.clear()
                ble_send_queue.put_nowait("[RESET]", urgent=True)
                last_reset_relay_at = time.monotonic()
                firebase.mark_reset_ack(RESET_DEVICE_ID, requested_at)
                in_flight_requested_at = requested_at
                logger.info("[RESET] Firebase request relayed to robot.")

            if (in_flight_requested_at and not _reset_completed.is_set() and
                    time.monotonic() - last_reset_relay_at >= 3.0):
                ble_send_queue.put_nowait("[RESET]", urgent=True)
                last_reset_relay_at = time.monotonic()
                logger.info("[RESET] Re-sent in-flight Firebase reset request.")

            if in_flight_requested_at and _reset_completed.is_set():
                firebase.mark_reset_done(RESET_DEVICE_ID, in_flight_requested_at)
                completed_requested_at = in_flight_requested_at
                in_flight_requested_at = 0
                _reset_completed.clear()
                logger.info("[RESET] Robot returned to IDLE; Firebase marked done.")
        except Exception as e:
            logger.error(f"Firebase reset bridge error: {e}")
            wait_s = poll_backoff.next_delay()

        stop_event.wait(wait_s)

    logger.info("Firebase reset command bridge stopped.")

# ═══ On-demand status responder ═══
def on_demand_status_responder(lidar_node, stop_event):
    logger.info("On-demand status responder started.")
    last_sensor_send = 0.0
    fallback_seq = 0
    while not stop_event.is_set():
        # Wake immediately when requested by MCU, or tick periodically at BLE_CMD_PERIOD (250 ms)
        fired = _status_requested.wait(timeout=BLE_CMD_PERIOD)
        _status_requested.clear()

        if not ble_connected.is_set():
            continue

        if lidar_node is not None:
            path_msg, sides_msg = lidar_node.build_status_packets()
        else:
            # Fail closed if the LiDAR node is unavailable.
            fallback_seq = (fallback_seq + 1) & 0xFFFFFFFF
            path_msg = f"P:{fallback_seq}|F=S|B=S"
            sides_msg = f"S:{fallback_seq}|STABLE"

        # Safety status always goes first. The mailbox coalesces older status
        # so a stale CLEAR cannot sit behind a newer STOP.
        ble_send_queue.put_nowait(path_msg)
        ble_send_queue.put_nowait(sides_msg)

        now = time.monotonic()
        if now - last_sensor_send < SENSOR_TX_PERIOD_S:
            continue
        last_sensor_send = now

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
        ble_send_queue.put_nowait(sensor_msg)

    logger.info("On-demand status responder stopped.")

# ═══ BLE notification handler ═══
def ble_notify_handler(msg: str):
    global _last_load_cell_value, _last_load_cell_publish
    try:
        ble_recv_queue.put_nowait(msg)
    except queue.Full:
        pass

    stripped = msg.strip()
    if stripped in (REQUEST_TOKEN, "CONNECTED...", "[BLE CONNECTION ESTABLISHED]"):
        _robot_running.set()
        _status_requested.set()
        return
    if stripped.startswith("LOAD_CELL:"):
        try:
            value = float(stripped.split(":", 1)[1])
            if not math.isfinite(value) or value < 0.0 or value > 200.0:
                raise ValueError("load-cell value outside 0..200 kg")
            now = time.monotonic()
            with _load_cell_lock:
                changed = (
                    _last_load_cell_value is None
                    or abs(value - _last_load_cell_value) >= LOAD_CELL_CHANGE_KG
                )
                refresh_due = now - _last_load_cell_publish >= LOAD_CELL_REFRESH_S
                if firebase and (changed or refresh_due):
                    firebase.update_mcu_load_cell(round(value, 2))
                    _last_load_cell_value = value
                    _last_load_cell_publish = now
        except Exception as e:
            logger.error(f"Load-cell Firebase update failed: {e}")
        return
    if stripped in ("[RESET]", "[IDLE]"):
        _robot_running.clear()
        if stripped == "[IDLE]":
            _reset_completed.set()

# ═══ BLE Service with urgent direct write ═══
class BleService(threading.Thread):
    RETRY_MIN_S    = 2.0
    RETRY_MAX_S    = 30.0
    SCAN_TIMEOUT   = 5.0
    CONNECT_TIMEOUT = 6.0
    WRITE_TIMEOUT = 2.0
    DRAIN_INTERVAL  = 0.02

    def __init__(self, status_cb, notify_cb):
        super().__init__(daemon=True, name="ble-service")
        self.status_cb    = status_cb
        self.notify_cb    = notify_cb
        self._loop        = None
        self._client      = None
        self._write_char  = None
        self._notify_char = None
        self._connecting  = False
        self._stop_event  = None
        self._write_lock  = None
        self._retry_handle = None
        self._connect_task = None
        self._drain_task = None
        self._retry_backoff = ExponentialBackoff(
            self.RETRY_MIN_S,
            self.RETRY_MAX_S,
            jitter=lambda delay: delay * random.uniform(0.9, 1.1),
        )

    def stop(self):
        if self._loop and not self._loop.is_closed() and self._stop_event:
            self._loop.call_soon_threadsafe(self._stop_event.set)

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
        if self._stop_event:
            self._stop_event.set()

    def run(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        self._stop_event = asyncio.Event()
        self._write_lock = asyncio.Lock()
        try:
            self._loop.run_until_complete(self._run_service())
        finally:
            pending = asyncio.all_tasks(self._loop)
            for task in pending:
                task.cancel()
            if pending:
                self._loop.run_until_complete(
                    asyncio.gather(*pending, return_exceptions=True)
                )
            self._loop.close()
            ble_connected.clear()

    async def _run_service(self):
        self._schedule_connect(0.0)
        await self._stop_event.wait()
        if self._retry_handle:
            self._retry_handle.cancel()
            self._retry_handle = None
        for task in (self._connect_task, self._drain_task):
            if task and not task.done():
                task.cancel()
        client = self._client
        if client and client.is_connected:
            try:
                await asyncio.wait_for(client.disconnect(), timeout=2.0)
            except Exception:
                pass

    def _schedule_connect(self, delay):
        if self._stop_event and self._stop_event.is_set():
            return
        if self._retry_handle:
            self._retry_handle.cancel()

        def launch():
            self._retry_handle = None
            if not self._stop_event.is_set():
                self._connect_task = self._loop.create_task(self._connect_guarded())

        self._retry_handle = self._loop.call_later(delay, launch)

    async def _connect_guarded(self):
        try:
            await self._connect()
        except asyncio.CancelledError:
            raise
        except Exception as exc:
            logger.exception("Unexpected BLE connection failure: %s", exc)
            client = self._client
            self._client = None
            self._write_char = None
            self._notify_char = None
            if client and getattr(client, "is_connected", False):
                try:
                    await asyncio.wait_for(client.disconnect(), timeout=2.0)
                except Exception:
                    pass
            self._schedule_retry("BLE error")

    def _schedule_retry(self, reason="Disconnected"):
        if not self._loop or self._loop.is_closed() or self._stop_event.is_set():
            return
        self._connecting = False
        _robot_running.clear()
        ble_connected.clear()
        delay = self._retry_backoff.next_delay()
        self._status(f"{reason} — retry {delay:.0f} s", "#f85149")
        self._schedule_connect(delay)

    def _status(self, text: str, color: str):
        try:
            self.status_cb(text, color)
        except Exception:
            logger.debug("BLE status callback failed", exc_info=True)

    def _on_disconnect(self, _client):
        if self._loop and not self._loop.is_closed():
            self._loop.call_soon_threadsafe(self._handle_disconnect, _client)

    def _handle_disconnect(self, disconnected_client):
        # Ignore a late callback from an old session after a new one connected.
        if self._client is not None and disconnected_client is not self._client:
            return
        self._client = None
        self._write_char = None
        self._notify_char = None
        if self._drain_task and not self._drain_task.done():
            self._drain_task.cancel()
        _robot_running.clear()
        ble_connected.clear()
        ble_send_queue.clear()
        logger.info("BLE device disconnected.")
        self._schedule_retry("Disconnected")

    def _notification_handler(self, sender, data):
        try:
            msg = data.decode('utf-8')
        except Exception:
            msg = str(data)
        if self.notify_cb:
            parts = [part.strip() for part in msg.splitlines() if part.strip()]
            for part in parts or [msg]:
                self.notify_cb(part)

    async def _connect(self):
        if self._connecting:
            return
        if self._stop_event.is_set() or (self._client and self._client.is_connected):
            return
        self._connecting = True
        target = None
        target_address = BLE_MAC_ADDRESS or getattr(self, "_cached_address", None)
        client = None

        if target_address:
            try:
                self._status(f"Direct {target_address[-5:]}…", "#d29922")
                logger.info(f"Attempting direct connect to BLE address [{target_address}]")
                direct_client = BleakClient(target_address, disconnected_callback=self._on_disconnect)
                await asyncio.wait_for(direct_client.connect(), timeout=5.0)
                if direct_client.is_connected:
                    client = direct_client
                    self._cached_address = target_address
            except Exception as exc:
                logger.info(f"Direct connect failed ({exc}); falling back to BLE scan.")
                client = None

        if client is None:
            self._status("Scanning…", "#d29922")
            target_address = None
            try:
                discovered = await BleakScanner.discover(
                    timeout=self.SCAN_TIMEOUT, return_adv=True
                )
                clean_service_uuid = SERVICE_UUID.lower().replace("-", "")
                clean_write_uuid = WRITE_CHAR_UUID.lower().replace("-", "")
                clean_notify_uuid = NOTIFY_CHAR_UUID.lower().replace("-", "")

                for device, adv in discovered.values():
                    dev_name = (device.name or "").strip()
                    adv_name = (adv.local_name or "").strip()
                    name_match = bool(
                        (dev_name and DEVICE_NAME.lower() in dev_name.lower())
                        or (adv_name and DEVICE_NAME.lower() in adv_name.lower())
                    )
                    service_uuids = [
                        str(s).lower().replace("-", "")
                        for s in (adv.service_uuids or [])
                    ]
                    uuid_match = bool(
                        clean_service_uuid in service_uuids
                        or clean_write_uuid in service_uuids
                        or clean_notify_uuid in service_uuids
                    )
                    if name_match or uuid_match:
                        target = device
                        target_address = device.address
                        break
            except TypeError:
                devices = await BleakScanner.discover(timeout=self.SCAN_TIMEOUT)
                clean_service_uuid = SERVICE_UUID.lower().replace("-", "")
                clean_write_uuid = WRITE_CHAR_UUID.lower().replace("-", "")
                clean_notify_uuid = NOTIFY_CHAR_UUID.lower().replace("-", "")

                for d in devices:
                    dev_name = (d.name or "").strip()
                    name_match = bool(dev_name and DEVICE_NAME.lower() in dev_name.lower())
                    raw_uuids = (
                        d.metadata.get("uuids", [])
                        if hasattr(d, "metadata") and isinstance(d.metadata, dict)
                        else []
                    )
                    uuids = [str(u).lower().replace("-", "") for u in raw_uuids]
                    uuid_match = bool(
                        clean_service_uuid in uuids
                        or clean_write_uuid in uuids
                        or clean_notify_uuid in uuids
                    )
                    if name_match or uuid_match:
                        target = d
                        target_address = d.address
                        break
            except Exception as exc:
                logger.warning(f"BLE scanner error: {exc}")

            if target_address is None:
                logger.warning(f"'{DEVICE_NAME}' not found.")
                self._schedule_retry("Not found")
                return

            disp_name = getattr(target, "name", None) or DEVICE_NAME
            logger.info(f"Connecting to BLE device {disp_name} [{target_address}]")
            self._status(f"Connecting {target_address}…", "#d29922")
            client = BleakClient(target_address, disconnected_callback=self._on_disconnect)
            try:
                await asyncio.wait_for(client.connect(), timeout=self.CONNECT_TIMEOUT)
            except Exception as exc:
                logger.error(f"BLE connect error: {exc}")
                try:
                    await client.disconnect()
                except Exception:
                    pass
                self._schedule_retry("Connect failed")
                return

        if not client or not client.is_connected:
            logger.warning("connect() returned but is_connected is False.")
            self._schedule_retry("Connect failed")
            return

        self._cached_address = target_address
        # Install the session before characteristic setup so disconnect
        # callbacks can be correlated to the correct client.
        self._client = client

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
            self._schedule_retry("Write char missing")
            return

        if notify_char is None:
            logger.warning(f"Notify characteristic {NOTIFY_CHAR_UUID} not found.")
            await client.disconnect()
            self._schedule_retry("Notify char missing")
            return
        else:
            try:
                await asyncio.wait_for(
                    client.start_notify(notify_char, self._notification_handler),
                    timeout=3.0,
                )
                logger.info("Subscribed to BLE notifications.")
            except Exception as exc:
                logger.error(f"Notify subscribe error: {exc}")
                await client.disconnect()
                self._schedule_retry("Notify sub failed")
                return

        self._write_char  = write_char
        self._notify_char = notify_char
        self._connecting  = False
        self._retry_backoff.reset()
        _robot_running.set()
        ble_connected.set()
        _status_requested.set()

        self._status("Connected", "#3fb950")
        logger.info("BLE connected.")

        # Seed the connection epoch with [RASPI READY]
        try:
            ble_send_queue.put_nowait(RASPI_READY)
            logger.info(f"[PROTOCOL] Sent {RASPI_READY}")
        except queue.Full:
            pass

        self._drain_task = self._loop.create_task(self._drain_loop(client))

    async def _write_message(self, msg, expected_client=None):
        client = self._client
        write_char = self._write_char
        if (
            not client
            or not client.is_connected
            or not write_char
            or (expected_client is not None and client is not expected_client)
        ):
            raise ConnectionError("BLE link is not writable")
        payload = msg.encode("utf-8")
        if len(payload) > 180:
            logger.warning("BLE payload is unexpectedly large (%d bytes): %s",
                           len(payload), msg[:60])
        # Safety/control messages use an acknowledged GATT write. Steering and
        # telemetry may use write-without-response, but all writes share one
        # lock so the GATT client is never called concurrently.
        acknowledged = msg.startswith(("P:", "[RESET]", "[RASPI READY]"))
        async with self._write_lock:
            if client is not self._client or not client.is_connected:
                raise ConnectionError("BLE session changed before write")
            await asyncio.wait_for(
                client.write_gatt_char(write_char, payload, response=acknowledged),
                timeout=self.WRITE_TIMEOUT,
            )

    async def _drain_loop(self, session_client):
        consecutive_write_failures = 0
        while (
            not self._stop_event.is_set()
            and self._client is session_client
            and session_client.is_connected
        ):
            try:
                msg = ble_send_queue.get_nowait()
            except queue.Empty:
                await asyncio.sleep(self.DRAIN_INTERVAL)
                continue
            try:
                await self._write_message(msg, expected_client=session_client)
                consecutive_write_failures = 0
            except Exception as exc:
                consecutive_write_failures += 1
                logger.warning(f"BLE write glitch ({consecutive_write_failures}/3): {exc}")
                if consecutive_write_failures < 3:
                    # Never reinsert an old path packet: a newer STOP may already
                    # be queued. Ask the producer for a fresh sequenced snapshot.
                    if msg.startswith("P:"):
                        _status_requested.set()
                    elif msg in ("[RESET]", RASPI_READY):
                        ble_send_queue.put_nowait(msg, urgent=True)
                    await asyncio.sleep(0.05)
                    continue
                logger.error(f"BLE persistent write failure: {exc}")
                try:
                    await asyncio.wait_for(session_client.disconnect(), timeout=2.0)
                except Exception:
                    pass
                self._schedule_retry("Write failed")
                return
        logger.info("BLE drain loop exiting.")

    async def _urgent_write(self, msg):
        """Serialized, acknowledged safety write from the LiDAR callback."""
        if msg.startswith("P:"):
            ble_send_queue.discard_kind("PATH")
        try:
            await self._write_message(msg)
        except Exception as exc:
            logger.error(f"Urgent BLE write error: {exc}")
            ble_send_queue.put_nowait(msg, urgent=True)

# ═══ GUI ═══
def init_gui():
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
            root.title("YDLidar – 8 Direction Distance Monitor + BLE")
            root.configure(bg="#0d1117")
            root.geometry("900x600")
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
            tk.Label(hdr, text="● YDLidar 8-Direction Distance Monitor",
                     bg="#0d1117", fg="#58a6ff", font=title_fnt).pack(side="left")
            self.status_lbl = tk.Label(hdr, text="Waiting…", bg="#0d1117",
                                       fg="#8b949e", font=small_fnt)
            self.status_lbl.pack(side="right")

            bottom_frame = tk.Frame(root, bg="#0d1117")
            bottom_frame.pack(side="bottom", fill="x")

            self.footer = tk.Label(bottom_frame, text="range: — cm | steering cone: ±11°",
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
            tk.Label(sensor_bar, text="SENSOR READINGS", bg="#0d1117",
                     fg="#8b949e", font=small_fnt).pack(anchor="w")
            sensor_row = tk.Frame(sensor_bar, bg="#0d1117")
            sensor_row.pack(fill="x")
            self.sensor_labels = {}
            sensor_defs = [
                ("ultrasonic", "ULTRASONIC",          "cm"),
                ("mq4",        "MQ4 (METHANE)",        ""),
                ("mq137",      "MQ137 (AMMONIA)",      ""),
                ("mq135",      "MQ135 (AIR QUALITY)",  ""),
            ]
            for key, name, unit in sensor_defs:
                cell = tk.Frame(sensor_row, bg="#161b22", highlightbackground="#30363d",
                                highlightthickness=1)
                cell.pack(side="left", expand=True, fill="x", padx=4)
                tk.Label(cell, text=name, bg="#161b22", fg="#e6edf3",
                         font=small_fnt).pack(pady=(6, 0))
                val_lbl = tk.Label(cell, text="—", bg="#161b22",
                                   fg="#58a6ff", font=sensor_val_fnt)
                val_lbl.pack(pady=(2, 6))
                self.sensor_labels[key] = (val_lbl, unit)

            compass = tk.Frame(root, bg="#0d1117")
            compass.pack(fill="both", expand=True, padx=14, pady=6)
            for i in range(3):
                compass.grid_rowconfigure(i, weight=1, uniform="card")
                compass.grid_columnconfigure(i, weight=1, uniform="card")

            self.cards = {}
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
                val = tk.Label(card, text="— cm", bg="#161b22", fg="#58a6ff", font=val_fnt)
                val.place(relx=0.05, rely=0.26)
                self._track_font(val, val_fnt)
                bar_canvas = tk.Canvas(card, bg="#30363d", bd=0, highlightthickness=0)
                bar_canvas.place(relx=0.05, rely=0.62, relwidth=0.9, relheight=0.08)
                bar_fill = bar_canvas.create_rectangle(0, 0, 0, 0, fill="#58a6ff", outline="")
                avg_lbl = tk.Label(card, text="avg: —", bg="#161b22", fg="#8b949e", font=small_fnt)
                avg_lbl.place(relx=0.05, rely=0.78)
                self._track_font(avg_lbl, small_fnt)
                pts_lbl = tk.Label(card, text="pts: —", bg="#161b22", fg="#8b949e", font=small_fnt)
                pts_lbl.place(relx=0.63, rely=0.78)
                self._track_font(pts_lbl, small_fnt)
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
            tk.Label(center_card, text="GARBY", bg="#161b22",
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

        def _process_updates(self):
            try:
                data, range_min, range_max, cone_deg, sides_status = self.data_queue.get_nowait()
                self.update(data, range_min, range_max, cone_deg, sides_status)
            except queue.Empty:
                pass
            self._update_sensor_readings()
            self.root.after(40, self._process_updates)

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
            self.root.after(40, self._process_ble_recv)

        def _update_sensor_readings(self):
            with sensor_readings_lock:
                readings = dict(latest_sensor_readings)
            for key, (lbl, unit) in self.sensor_labels.items():
                val = readings.get(key)
                if val is None:
                    # Not available yet — show the same sentinel that gets
                    # sent over BLE (999 for ultrasonic, -1 for gas sensors)
                    # so the GUI matches what the ESP32 actually receives.
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
            self.status_lbl.config(text="● Live", fg="#3fb950")

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
                      f"safety sectors: 45° gap-free | steering cone: ±{cone_deg/2:.0f}°")
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
        try:
            old_settings = termios.tcgetattr(fd)
        except Exception:
            return
        try:
            tty.setcbreak(fd)
            while not serial_stop.is_set():
                ch = sys.stdin.read(1)
                if not ch:
                    break
                ch = ch.lower()
                if ch == 'f':
                    if front_disabled.is_set(): front_disabled.clear(); logger.info("FRONT re-enabled.")
                    else: front_disabled.set(); logger.info("FRONT disabled.")
                elif ch == 'b':
                    if back_disabled.is_set(): back_disabled.clear(); logger.info("BACK re-enabled.")
                    else: back_disabled.set(); logger.info("BACK disabled.")
                elif ch == 'q':
                    os.kill(os.getpid(), signal.SIGINT); return
        except Exception:
            pass
        finally:
            try:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            except Exception:
                pass

    threading.Thread(target=_listener, daemon=True, name="key-listener").start()

# ═══ LiDAR port / driver ═══
def find_lidar_port():
    configured = os.environ.get("GARBY_LIDAR_PORT")
    candidates = ([configured] if configured else []) + sorted(
        glob.glob("/dev/serial/by-id/*ydlidar*")
        + glob.glob("/dev/serial/by-id/*YDLIDAR*")
    ) + ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0"]
    for port in candidates:
        if port and os.path.exists(port):
            return port
    return None

def _terminate_process(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        proc.terminate()
        proc.wait(timeout=2.0)
    except Exception:
        try:
            proc.kill()
            proc.wait(timeout=1.0)
        except Exception:
            pass


def _cleanup_stale_driver_state():
    """Stop leftover LiDAR drivers and clear stale ROS 2 DDS state.

    Must run BEFORE rclpy.init(). While the safety node is alive, stopping the
    ROS daemon or deleting /dev/shm/fastrtps_* shared-memory segments can break
    DDS discovery, so the running node would never receive the new driver's
    /scan messages (the "driver started but node gets no scan" failure).
    """
    try:
        subprocess.run(
            ["pkill", "-f", "ydlidar_ros2_driver"],
            capture_output=True,
            timeout=3.0,
            check=False,
        )
        subprocess.run(
            ["ros2", "daemon", "stop"],
            capture_output=True,
            timeout=3.0,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        pass
    for path in glob.glob("/dev/shm/fastrtps_*"):
        try:
            os.remove(path)
        except OSError:
            pass


def start_lidar_driver(stop_event=None, ros_node=None):
    port = find_lidar_port()
    if port is None:
        raise RuntimeError("[LiDAR] No ttyUSB port found!")
    try:
        subprocess.run(
            ["pkill", "-f", "ydlidar_ros2_driver"],
            capture_output=True,
            timeout=3.0,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        pass
    if stop_event and stop_event.wait(1.0):
        raise RuntimeError("LiDAR startup cancelled")
    logger.info(f"Starting LiDAR driver on {port}")
    # Defaults match the bench-tested YDLIDAR X3 config
    # (ros2_ws/src/ydlidar_ros2_driver/config/ydlidar_x3.yaml). In particular
    # the X3 does NOT use DTR motor control; forcing it glitches the serial
    # stream and produces the "Checksum error" storm that prevents any valid
    # scan from ever being assembled/published.
    baudrate = os.environ.get("GARBY_LIDAR_BAUDRATE", "115200")
    lidar_type = os.environ.get("GARBY_LIDAR_TYPE", "1")
    sample_rate = os.environ.get("GARBY_LIDAR_SAMPLE_RATE", "3")
    motor_dtr = os.environ.get("GARBY_LIDAR_DTR", "true")
    frequency = os.environ.get("GARBY_LIDAR_FREQUENCY", "10.0")
    command = [
        "ros2", "run", "ydlidar_ros2_driver", "ydlidar_ros2_driver_node",
        "--ros-args",
        "-p", f"port:={port}",
        "-p", f"baudrate:={baudrate}",
        "-p", f"lidar_type:={lidar_type}",
        "-p", "device_type:=0",
        "-p", f"sample_rate:={sample_rate}",
        "-p", "isSingleChannel:=true",
        "-p", "fixed_resolution:=true",
        "-p", "reversion:=false",
        "-p", "inverted:=false",
        "-p", "intensity:=false",
        "-p", f"support_motor_dtr:={motor_dtr}",
        "-p", f"frequency:={frequency}",
        "-p", "range_max:=8.0",
        "-p", "range_min:=0.1",
        "-p", "angle_max:=180.0",
        "-p", "angle_min:=-180.0",
        "-p", "abnormal_check_count:=4",
        "-p", "invalid_range_is_inf:=false",
        "-p", "frame_id:=laser_frame",
        "-p", "auto_reconnect:=true",
    ]
    # Suppress verbose driver SDK output by default unless explicitly enabled with GARBY_LIDAR_DRIVER_LOGS=1
    suppress_driver_logs = os.environ.get("GARBY_LIDAR_DRIVER_LOGS", "0") != "1"
    proc = subprocess.Popen(
        command,
        stdout=subprocess.DEVNULL if suppress_driver_logs else None,
        stderr=subprocess.DEVNULL if suppress_driver_logs else None,
        env=os.environ.copy(),
    )
    for _ in range(30):
        if stop_event and stop_event.is_set():
            _terminate_process(proc)
            raise RuntimeError("LiDAR startup cancelled")
        if proc.poll() is not None:
            raise RuntimeError(f"LiDAR driver exited early with code {proc.returncode}")
        # In-process verification: check if healthy scan was received or if node sees publisher
        if lidar_healthy.is_set() or (ros_node is not None and ros_node.count_publishers("/scan") > 0):
            break
        if stop_event:
            stop_event.wait(1.0)
        else:
            time.sleep(1.0)
    else:
        _terminate_process(proc)
        raise RuntimeError("/scan topic publisher never appeared")
    return proc


class LidarDriverSupervisor(threading.Thread):
    """Restart a failed/stalled ROS driver with bounded backoff."""

    STARTUP_GRACE_PERIOD_S = 30.0
    RESTART_STALE_AFTER_S = 15.0

    def __init__(self, stop_event, ros_node=None):
        super().__init__(daemon=True, name="lidar-driver-supervisor")
        self.stop_event = stop_event
        self.ros_node = ros_node
        self._process = None
        self._process_lock = threading.Lock()
        self._backoff = ExponentialBackoff(
            2.0,
            30.0,
            jitter=lambda delay: delay * random.uniform(0.9, 1.1),
        )

    def stop(self):
        with self._process_lock:
            proc = self._process
        _terminate_process(proc)

    def run(self):
        while not self.stop_event.is_set():
            try:
                proc = start_lidar_driver(self.stop_event, self.ros_node)
                with self._process_lock:
                    self._process = proc
                logger.info("LiDAR driver is online.")
                started_at = time.monotonic()
                stale_since = None
                healthy_since = None

                while not self.stop_event.wait(1.0):
                    if proc.poll() is not None:
                        logger.error("LiDAR driver exited with code %s", proc.returncode)
                        break
                    now = time.monotonic()
                    if lidar_healthy.is_set():
                        stale_since = None
                        healthy_since = healthy_since or now
                        if now - healthy_since >= 10.0:
                            self._backoff.reset()
                    else:
                        healthy_since = None
                        in_startup_grace = (now - started_at) < self.STARTUP_GRACE_PERIOD_S
                        if not in_startup_grace:
                            stale_since = stale_since or now
                            if now - stale_since >= self.RESTART_STALE_AFTER_S:
                                logger.error(
                                    "LiDAR produced no healthy scan for %.0f s; restarting driver.",
                                    self.RESTART_STALE_AFTER_S,
                                )
                                break
            except Exception as exc:
                if not self.stop_event.is_set():
                    logger.error("LiDAR driver startup failed: %s", exc)
            finally:
                lidar_healthy.clear()
                with self._process_lock:
                    proc = self._process
                    self._process = None
                _terminate_process(proc)

            if not self.stop_event.is_set():
                self.stop_event.wait(self._backoff.next_delay())

# ═══ ROS 2 Node – FAST VERSION ═══
# pyrefly: ignore [missing-import]
import rclpy
# pyrefly: ignore [missing-import]
from rclpy.node import Node
# pyrefly: ignore [missing-import]
from rclpy.qos import qos_profile_sensor_data
# pyrefly: ignore [missing-import]
from rclpy.executors import SingleThreadedExecutor
# pyrefly: ignore [missing-import]
from sensor_msgs.msg import LaserScan

class LidarDistanceReader(Node):
    SIDES = {
        "FRONT":         0.0,
        "FRONT_LEFT":   45.0,
        "LEFT":         90.0,
        "BACK_LEFT":   135.0,
        "BACK":        180.0,
        "BACK_RIGHT":  225.0,
        "RIGHT":       270.0,
        "FRONT_RIGHT": 315.0,
    }
    FRONT_GROUP = {"FRONT", "FRONT_LEFT", "FRONT_RIGHT"}
    BACK_GROUP  = {"BACK",  "BACK_LEFT",  "BACK_RIGHT"}
    CONE_DEG          = 22.0  # centered steering slice inside each 45° safety sector
    HISTORY_DEPTH     = 3
    MIN_VALID_POINTS  = 3
    OUTLIER_RATIO     = 2.5
    OUTLIER_CONFIRM   = 1      # faster response to real-world movement

    def __init__(self, gui, data_queue: queue.Queue, ble_service=None):
        super().__init__('lidar_distance_reader')
        self.gui        = gui
        self.data_queue = data_queue
        self.ble_service = ble_service
        self.history    = {side: deque(maxlen=self.HISTORY_DEPTH) for side in self.SIDES}
        self.steering_history = {
            side: deque(maxlen=self.HISTORY_DEPTH) for side in self.SIDES
        }
        self._pending_jump = {side: [None, 0] for side in self.SIDES}  # [candidate_cm, streak]
        self.last_sides_status  = "SIDES:STABLE"
        self._prev_front_blocked = False
        self._prev_back_blocked = False
        self._last_urgent_send = 0
        self._urgent_cooldown = 0.05
        self._last_scan_time = 0.0
        self._first_scan_received = False
        self._front_data_valid = False
        self._back_data_valid = False
        self._lidar_stale_announced = False
        self._path_seq = 0
        self._path_seq_lock = threading.Lock()
        self._history_lock = threading.RLock()
        self._watchdog_timer = self.create_timer(0.25, self._watchdog_callback)
        # One ROS subscription only. Multiple subscriptions to /scan caused the
        # same scan to be processed repeatedly and distorted confirmation logic.
        self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.get_logger().info(
            f'Lidar Distance Reader started '
            f'(fail-closed, yaw offset {LIDAR_YAW_OFFSET_DEG:.1f} deg)'
        )

    def _next_path_seq(self):
        with self._path_seq_lock:
            self._path_seq = (self._path_seq + 1) & 0xFFFFFFFF
            return self._path_seq

    def _lidar_is_stale(self):
        # Safety has no startup grace: until the first scan is received the path
        # is unknown and therefore STOP. Driver supervision has its own grace.
        if not self._first_scan_received or self._last_scan_time <= 0.0:
            return True
        return (time.monotonic() - self._last_scan_time) > LIDAR_STALE_TIMEOUT_S

    def _watchdog_callback(self):
        if not self._lidar_is_stale():
            self._lidar_stale_announced = False
            return
        lidar_healthy.clear()
        if self._lidar_stale_announced:
            return
        self._lidar_stale_announced = True
        self.get_logger().error(
            f"LiDAR stream stale for > {LIDAR_STALE_TIMEOUT_S:.1f} s; issuing fail-closed STOP"
        )
        seq = self._next_path_seq()
        path_msg = f"P:{seq}|F=S|B=S"
        if not self._dispatch_urgent_path(path_msg):
            ble_send_queue.put_nowait(path_msg, urgent=True)

    def _dispatch_urgent_path(self, path_msg):
        service = self.ble_service
        loop = service._loop if service else None
        if not service or not loop or loop.is_closed() or not ble_connected.is_set():
            return False
        try:
            asyncio.run_coroutine_threadsafe(service._urgent_write(path_msg), loop)
            return True
        except RuntimeError:
            return False

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

    def _steering_smoothed_distance(self, side: str):
        valid = [v for v in self.steering_history[side] if v is not None]
        if not valid:
            return None
        return median(valid)

    def _collect_blocked_sides(self, group, threshold_cm):
        """Return list of side names in *group* whose quick-min distance
        is ≤ threshold_cm. None represents open space with no obstacles."""
        blocked = []
        for side in group:
            d = self._quick_min(side)
            if d is not None and d <= threshold_cm:
                blocked.append(side)
        return blocked

    def _format_blocked_status(self, blocked_sides, all_sides):
        """Return LiDAR-only path state; UART ultrasonic is trash level."""
        return format_lidar_blockage(blocked_sides, all_sides)

    def _get_blocked_status(self, group, threshold_cm):
        blocked = self._collect_blocked_sides(group, threshold_cm)
        return self._format_blocked_status(blocked, group)

    @staticmethod
    def _status_code(status):
        return lidar_status_code(status)

    def _send_immediate_status(self, front_blocked, back_blocked):
        front_status = "CLEAR"
        back_status = "CLEAR"
        with self._history_lock:
            if front_blocked:
                blocked = self._collect_blocked_sides(self.FRONT_GROUP, THRESHOLD_CM)
                _, front_status = self._format_blocked_status(blocked, self.FRONT_GROUP)
            if back_blocked:
                blocked = self._collect_blocked_sides(self.BACK_GROUP, BACK_THRESHOLD_CM)
                _, back_status = self._format_blocked_status(blocked, self.BACK_GROUP)
        seq = self._next_path_seq()
        path_msg = (f"P:{seq}|F={self._status_code(front_status)}"
                    f"|B={self._status_code(back_status)}")
        if not self._dispatch_urgent_path(path_msg):
            ble_send_queue.put_nowait(path_msg, urgent=True)

    def scan_callback(self, msg: LaserScan):
        self._last_scan_time = time.monotonic()
        first_scan = not self._first_scan_received
        self._first_scan_received = True
        self._lidar_stale_announced = False
        if first_scan:
            logger.info("First LiDAR scan received with %d range samples.", len(msg.ranges))

        # One pass creates two geometries: gap-free nearest 45° sectors for
        # collision safety, and centered 22° slices for clean wall steering.
        # There are no eight-way comparisons and no safety blind angles.
        ranges = msg.ranges
        angle_min = msg.angle_min + math.radians(LIDAR_YAW_OFFSET_DEG)
        angle_inc = msg.angle_increment
        minimum_range = max(0.03, float(msg.range_min or 0.03))
        maximum_range = min(8.0, float(msg.range_max or 8.0))
        safety_dists, steering_dists, valid_points = partition_lidar_samples(
            ranges,
            angle_min=angle_min,
            angle_increment=angle_inc,
            range_min=minimum_range,
            range_max=maximum_range,
            steering_cone_deg=self.CONE_DEG,
        )

        front_data_valid = all(
            len(safety_dists[side]) >= self.MIN_VALID_POINTS
            for side in self.FRONT_GROUP
        )
        back_data_valid = all(
            len(safety_dists[side]) >= self.MIN_VALID_POINTS
            for side in self.BACK_GROUP
        )
        prior_quality = self._front_data_valid and self._back_data_valid
        self._front_data_valid = front_data_valid
        self._back_data_valid = back_data_valid
        if front_data_valid and back_data_valid:
            lidar_healthy.set()
        else:
            lidar_healthy.clear()

        # A scan can arrive on time yet be unusable (NaNs, missing sectors, bad
        # range metadata). Treat that transition as safety-critical immediately.
        if prior_quality and not (front_data_valid and back_data_valid):
            seq = self._next_path_seq()
            degraded = (
                f"P:{seq}|F={'C' if front_data_valid else 'S'}"
                f"|B={'C' if back_data_valid else 'S'}"
            )
            if not self._dispatch_urgent_path(degraded):
                ble_send_queue.put_nowait(degraded, urgent=True)

        urgent_status = None
        with self._history_lock:
            for side in self.SIDES:
                dists = safety_dists[side]
                if len(dists) >= self.MIN_VALID_POINTS:
                    # Keep independent estimates: nearest return for collision
                    # safety and median wall geometry for smooth centering/tilt.
                    # Temporal confirmation handles non-emergency jump noise.
                    new_cm = robust_near_distance_cm(dists)
                    narrow_wall_dists = steering_dists[side]
                    wall_source = select_steering_samples(
                        narrow_wall_dists,
                        dists,
                        minimum_points=self.MIN_VALID_POINTS,
                    )
                    wall_cm = robust_wall_distance_cm(wall_source)
                    self.steering_history[side].append(wall_cm)
                    current_median = self._smoothed_distance(side)

                    # Fast-track close obstacles in either travel direction.
                    emergency_close = (
                        side in self.FRONT_GROUP and new_cm <= THRESHOLD_CM + 5.0
                    ) or (
                        side in self.BACK_GROUP and new_cm <= BACK_THRESHOLD_CM + 5.0
                    )
                    if emergency_close:
                        self.history[side].append(new_cm)
                        self._pending_jump[side] = [None, 0]
                    elif current_median is None:
                        self.history[side].append(new_cm)
                        self._pending_jump[side] = [None, 0]
                    else:
                        lo = current_median / self.OUTLIER_RATIO
                        hi = current_median * self.OUTLIER_RATIO
                        if lo <= new_cm <= hi:
                            self.history[side].append(new_cm)
                            self._pending_jump[side] = [None, 0]
                        else:
                            cand, streak = self._pending_jump[side]
                            if cand is not None and abs(new_cm - cand) <= (cand * 0.25):
                                streak += 1
                            else:
                                cand, streak = new_cm, 1
                            if streak >= self.OUTLIER_CONFIRM:
                                self.history[side].append(new_cm)
                                self._pending_jump[side] = [None, 0]
                            else:
                                self._pending_jump[side] = [cand, streak]
                else:
                    self.history[side].append(None)
                    self.steering_history[side].append(None)
                    self._pending_jump[side] = [None, 0]

            data_out = {}
            for side in self.SIDES:
                display_history = (
                    self.steering_history[side]
                    if side in ("LEFT", "RIGHT")
                    else self.history[side]
                )
                valid = [v for v in display_history if v is not None]
                if valid:
                    med_cm = median(valid)
                    data_out[side] = (
                        med_cm / 100.0,
                        med_cm / 100.0,
                        len(valid),
                    )
                else:
                    data_out[side] = None

            front_blocked, _ = self._get_blocked_status(
                self.FRONT_GROUP, THRESHOLD_CM
            )
            back_blocked, _ = self._get_blocked_status(
                self.BACK_GROUP, BACK_THRESHOLD_CM
            )
            bypass_allowed = (
                ALLOW_RUNTIME_SAFETY_DISABLE or not _robot_running.is_set()
            )
            if bypass_allowed and front_disabled.is_set():
                front_blocked = False
            if bypass_allowed and back_disabled.is_set():
                back_blocked = False

            now = time.monotonic()
            changed_to_blocked = (
                (front_blocked or back_blocked)
                and (
                    front_blocked != self._prev_front_blocked
                    or back_blocked != self._prev_back_blocked
                )
            )
            if (
                changed_to_blocked
                and now - self._last_urgent_send > self._urgent_cooldown
            ):
                urgent_status = (front_blocked, back_blocked)
                self._last_urgent_send = now

            self._prev_front_blocked = front_blocked
            self._prev_back_blocked = back_blocked

        if urgent_status is not None:
            self._send_immediate_status(*urgent_status)

        if self.data_queue.full():
            try: self.data_queue.get_nowait()
            except queue.Empty: pass
        self.data_queue.put_nowait(
            (data_out, msg.range_min, msg.range_max, self.CONE_DEG, self.last_sides_status)
        )

    def build_status_packets(self):
        seq = self._next_path_seq()
        if self._lidar_is_stale():
            self.last_sides_status = "SIDES:STABLE"
            return f"P:{seq}|F=S|B=S", f"S:{seq}|STABLE"

        with self._history_lock:
            front_blocked, front_status = self._get_blocked_status(
                self.FRONT_GROUP, THRESHOLD_CM
            )
            back_blocked, back_status = self._get_blocked_status(
                self.BACK_GROUP, BACK_THRESHOLD_CM
            )

            # Fresh-but-incomplete scans are unknown, never clear. Manual GUI
            # bypasses are intentionally unable to override unavailable data.
            if not self._front_data_valid:
                front_blocked, front_status = True, "LIDAR_UNAVAILABLE"
            if not self._back_data_valid:
                back_blocked, back_status = True, "LIDAR_UNAVAILABLE"

            bypass_allowed = ALLOW_RUNTIME_SAFETY_DISABLE or not _robot_running.is_set()
            if self._front_data_valid and bypass_allowed and front_disabled.is_set():
                front_blocked, front_status = False, "CLEAR"
            if self._back_data_valid and bypass_allowed and back_disabled.is_set():
                back_blocked, back_status = False, "CLEAR"

            med = {
                side: self._steering_smoothed_distance(side)
                for side in self.SIDES
            }

        path_msg = (f"P:{seq}|F={self._status_code(front_status)}"
                    f"|B={self._status_code(back_status)}")

        key_map = {
            "LEFT": "L", "RIGHT": "R", "FRONT": "F", "BACK": "B",
            "FRONT_LEFT": "FL", "FRONT_RIGHT": "FR",
            "BACK_LEFT": "BL", "BACK_RIGHT": "BR",
        }
        sides_parts = [
            f"{key_map[side]}={value:.1f}"
            for side, value in med.items() if value is not None
        ]

        fl, fr = med.get("FRONT_LEFT"), med.get("FRONT_RIGHT")
        bl, br = med.get("BACK_LEFT"), med.get("BACK_RIGHT")
        tilt_val = None
        if None not in (fl, fr, bl, br):
            tilt_val = 0.5 * (fl - fr) + 0.5 * (br - bl)
        elif fl is not None and fr is not None:
            tilt_val = fl - fr
        if tilt_val is not None:
            # Receiver weights T directly; bound it so a doorway or far return
            # cannot request a disproportionate correction.
            tilt_val = clamp_heading_error_cm(tilt_val)
            sides_parts.append(f"T={tilt_val:.1f}")

        if sides_parts:
            sides_cmd = "|".join(sides_parts)
            sides_msg = f"S:{seq}|{sides_cmd}"
            self.last_sides_status = "SIDES:" + sides_cmd
        else:
            sides_msg = f"S:{seq}|STABLE"
            self.last_sides_status = "SIDES:STABLE"
        return path_msg, sides_msg

    def build_combined_cmd(self) -> str:
        """Compatibility helper; new transport uses build_status_packets()."""
        path_msg, sides_msg = self.build_status_packets()
        return f"{path_msg}||{sides_msg}"



# ═══ Main ═══
def main():
    import argparse
    parser = argparse.ArgumentParser(description="Combined Sensor + LiDAR App")
    parser.add_argument("--headless", action="store_true", help="Run without GUI")
    args = parser.parse_args()

    global firebase, ser, ble_service
    serial_stop.clear()
    kill_gpio_users()
    time.sleep(1)

    firebase_thread = threading.Thread(
        target=lambda: firebase_connection_loop(serial_stop),
        daemon=True,
        name="firebase-connection",
    )
    firebase_thread.start()

    serial_thread = threading.Thread(
        target=lambda: receiveSerial(serial_stop),
        daemon=True,
        name="uart-sensors",
    )
    serial_thread.start()

    heartbeat_thread = threading.Thread(
        target=lambda: raspi_heartbeat_loop(serial_stop),
        daemon=True,
        name="raspi-heartbeat",
    )
    heartbeat_thread.start()

    reset_thread = threading.Thread(
        target=lambda: firebase_reset_command_loop(serial_stop),
        daemon=True,
        name="firebase-reset",
    )
    reset_thread.start()

    # Clear stale DDS/daemon state BEFORE the ROS node initializes its DDS
    # participant. Doing this later (while the node is live) prevents the node
    # from discovering the LiDAR driver's /scan publisher.
    _cleanup_stale_driver_state()

    rclpy.init()
    data_queue = queue.Queue(maxsize=5)

    ble_service = None
    node = LidarDistanceReader(None, data_queue, ble_service)
    ros_executor = SingleThreadedExecutor()
    ros_executor.add_node(node)
    ros_thread = threading.Thread(
        target=ros_executor.spin,
        daemon=True,
        name="ros2-safety-executor",
    )
    ros_thread.start()

    lidar_supervisor = LidarDriverSupervisor(serial_stop, node)
    lidar_supervisor.start()

    status_responder_thread = threading.Thread(
        target=lambda: on_demand_status_responder(node, serial_stop),
        daemon=True,
        name="status-responder",
    )
    status_responder_thread.start()

    ble = None
    gui_root = None
    shutdown_started = threading.Event()

    def shutdown():
        # Guard against being invoked twice (e.g. SIGINT + WM_DELETE_WINDOW
        # racing each other) — re-entering this would double-close BLE/ROS
        # resources and could itself be a source of a hang.
        if shutdown_started.is_set():
            return
        shutdown_started.set()
        logger.info("Shutting down...")

        # Watchdog: guarantees the process actually exits even if a step
        # below hangs (e.g. rclpy's executor.shutdown() blocking forever on
        # a stuck callback, or the BLE asyncio thread never waking up).
        # This was the root cause of the process staying alive after
        # "dispose" — a blocked cleanup step meant os._exit()/sys.exit()
        # further down was never reached.
        def _force_kill():
            time.sleep(5.0)
            logger.warning("Shutdown taking too long — forcing process exit.")
            os._exit(1)
        threading.Thread(target=_force_kill, daemon=True, name="shutdown-watchdog").start()

        serial_stop.set()
        lidar_supervisor.stop()

        try:
            if ble: ble.disconnect_client()
        except Exception as e:
            logger.error(f"BLE cleanup error: {e}")

        try:
            ros_executor.shutdown(timeout_sec=2.0)
        except Exception as e:
            logger.error(f"ROS executor shutdown error: {e}")

        try:
            node.destroy_node()
        except Exception as e:
            logger.error(f"Node destroy error: {e}")

        try:
            rclpy.shutdown()
        except Exception as e:
            logger.error(f"rclpy shutdown error: {e}")

        try:
            subprocess.run(["pkill", "-f", "ydlidar_ros2_driver"],
                            capture_output=True, timeout=3.0)
        except Exception as e:
            logger.error(f"pkill error: {e}")

        # Explicitly tear down the Tk event loop rather than relying on
        # sys.exit() alone to unwind out of mainloop() — mainloop() only
        # returns once the root window is destroyed.
        if gui_root is not None:
            try:
                gui_root.quit()
                gui_root.destroy()
            except Exception as e:
                logger.error(f"GUI teardown error: {e}")

        try:
            if firebase:
                firebase.close()
        except Exception as e:
            logger.error("Firebase shutdown error: %s", e)

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
            while not serial_stop.is_set():
                # Bounded drain plus wait prevents a notification burst
                # from turning the headless main loop into a CPU hot-loop.
                for _ in range(20):
                    try:
                        ble_recv_queue.get_nowait()
                    except queue.Empty:
                        break
                serial_stop.wait(0.5)
        except KeyboardInterrupt:
            shutdown()
    else:
        import tkinter as tk
        root = tk.Tk()
        gui_root = root
        LidarGUI = init_gui()
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
