#!/usr/bin/env python3
"""
Combined application – Fast LiDAR + BLE with immediate obstacle push.
"""

import os, sys, time, queue, threading, asyncio, random, math, logging, signal, subprocess, glob
import json
from collections import deque
from datetime import datetime
from statistics import median
from copy import copy

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("combined_app")
# Pi software revision, shown by the phone app next to the controller build IDs.
PI_SOFTWARE_ID = "PHONE-20261114"

from bridge_core import (
    CoalescingUpdateWorker,
    CoalescingBleQueue,
    ExponentialBackoff,
    ManualStartControl,
    SENSOR_SPECS,
    SensorTransitionTracker,
    build_health_status,
    clamp_heading_error_cm,
    describe_throttle_flags,
    format_lidar_blockage,
    lidar_status_code,
    partition_lidar_samples,
    robust_near_distance_cm,
    robust_wall_distance_cm,
    select_steering_samples,
    side_obstacle_names,
    THROTTLE_UNDERVOLTAGE_NOW,
)

from port_detection import LIDAR, SENSOR, PortDetector
from runtime_support import SingleInstanceGuard
from operator_support import OperatorControl, StopLatchStore, reboot_pi
from notice_support import HealthReporter, probe_internet
from firebase_support import resolve_firebase_credentials
from disposal_support import DisposalRelay, READY_MAX_AGE_S
from lidar_map import build_lidar_map_snapshot
from lidar_geometry import robot_scan_angles
from hallway_geometry import analyze_hallway, geometry_from_environment
from motion_tracker import EgoMotionState, MotionAssessment, MotionTracker, build_segments
from scan_frame import build_scan_frame
from scene_objects import SceneMonitor, analyze_scene, scene_markers
from wall_heading_offset import WallHeadingOffset
from station_mode import StationMode, without_station_walls
import cloud_control

import serial
# pyrefly: ignore [missing-import]
from bleak import BleakScanner, BleakClient
from ble_support import (
    SERVICE_UUID as DEFAULT_SERVICE_UUID,
    WRITE_CHAR_UUID as DEFAULT_WRITE_CHAR_UUID,
    NOTIFY_CHAR_UUID as DEFAULT_NOTIFY_CHAR_UUID,
    configured_uuid, adapter_options, discover_robot, robot_characteristics, subscribe,
    NotificationDecoder, DiscoveryDiagnostics,
)

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

    DEFAULT_DATABASE_URL = (
        "https://garby-thesis-default-rtdb.asia-southeast1.firebasedatabase.app"
    )

    def __init__(self, credential_path=None, database_url=None):
        if firebase_admin is None or credentials is None or db is None:
            raise RuntimeError(f"firebase-admin unavailable: {FIREBASE_IMPORT_ERROR}")
        credential_path = resolve_firebase_credentials(credential_path)
        database_url = database_url or os.environ.get(
            "GARBY_FIREBASE_DATABASE_URL", self.DEFAULT_DATABASE_URL
        )
        if not os.path.isfile(credential_path):
            raise FileNotFoundError(f"Firebase credential file not found: {credential_path}")

        if not firebase_admin._apps:
            cred = credentials.Certificate(credential_path)
            firebase_admin.initialize_app(cred, {"databaseURL": database_url, "httpTimeout": 2.0})
        self._root = db.reference("/")
        self._init_cloud_health()
        self._last_write_error_log = 0.0
        self._status_lock = threading.Lock()
        self._last_status_fields = {}
        self._writer = CoalescingUpdateWorker(self._commit_root_update)

    def _init_cloud_health(self):
        self._cloud_lock = threading.Lock()
        self._cloud_success_at = None
        self._cloud_last_ok = None

    def _cloud_call(self, operation, *args):
        try:
            result = operation(*args)
        except Exception:
            with self._cloud_lock:
                self._cloud_last_ok = False
            raise
        with self._cloud_lock:
            self._cloud_success_at = time.monotonic()
            self._cloud_last_ok = True
        return result

    def cloud_health(self):
        with self._cloud_lock:
            success_at, last_ok = self._cloud_success_at, self._cloud_last_ok
        pending = self._writer.pending_count()
        recent = success_at is not None and time.monotonic() - success_at <= 60.0
        online = last_ok is True and recent
        state = ("Offline" if last_ok is False else
                 "Connecting" if success_at is None else
                 "Stale" if not recent else "Pending" if pending else "Connected")
        return {"online": online, "state": state, "pending_fields": pending,
                "last_success_at": success_at}

    def _commit_root_update(self, updates):
        try:
            self._cloud_call(self._root.update, dict(updates))
        except Exception as exc:
            now = time.monotonic()
            if now - self._last_write_error_log >= 15.0:
                logger.error("Firebase write failed; retrying with backoff: %s", exc)
                self._last_write_error_log = now
            raise

    def _update(self, updates, *, urgent=False):
        self._writer.enqueue(updates, urgent=urgent)

    def close(self):
        flushed = self._writer.close(flush=True, timeout=3.0)
        if not flushed:
            logger.warning("Firebase shutdown deadline reached; pending telemetry was not fully flushed.")
        return flushed

    def update_app(self, reset_state):
        self._update({"APP/resetState": reset_state}, urgent=True)

    def get_reset_command(self, device_id):
        command = self._cloud_call(db.reference(f"/devices/{device_id}/commands/reset").get) or {}
        # The structured request owns identity and status. A leftover legacy
        # boolean must never resurrect a completed/failed request.
        return command

    def _transition_reset(self, device_id, requested_at, status, **fields):
        ref = db.reference(f"/devices/{device_id}/commands/reset")
        def transition(current):
            if not isinstance(current, dict):
                return current
            if current.get("requestedAt", 0) != requested_at:
                return current
            if current.get("status") not in ("pending", "ack"):
                return current
            return {**current, "status": status, **fields}
        result = self._cloud_call(ref.transaction, transition)
        return isinstance(result, dict) and result.get("requestedAt") == requested_at and result.get("status") == status

    def mark_reset_ack(self, device_id, requested_at):
        return self._transition_reset(device_id, requested_at, "ack", ackAt=int(time.time() * 1000))

    def mark_reset_done(self, device_id, requested_at):
        return self._transition_reset(device_id, requested_at, "done", doneAt=int(time.time() * 1000))

    def mark_reset_failed(self, device_id, requested_at, reason):
        return self._transition_reset(device_id, requested_at, "failed", failedAt=int(time.time() * 1000), reason=reason)

    def get_disposal_command(self, device_id):
        return self._cloud_call(db.reference(f"/devices/{device_id}/commands/dispose").get)

    def _transition_disposal(self, device_id, command, expected, status, **fields):
        ref = db.reference(f"/devices/{device_id}/commands/dispose")
        def transition(current):
            if (not isinstance(current, dict)
                    or any(current.get(key) != command.get(key)
                           for key in ("requestId", "requestedAt", "requestedBy"))
                    or current.get("status") not in expected):
                return current
            return {**current, "status": status, **fields}
        result = self._cloud_call(ref.transaction, transition)
        return (isinstance(result, dict) and result.get("status") == status
                and all(result.get(key) == command.get(key)
                        for key in ("requestId", "requestedAt", "requestedBy"))
                and all(result.get(key) == value for key, value in fields.items()))

    def claim_disposal(self, device_id, command, controller_id):
        return self._transition_disposal(
            device_id, command, ("pending",), "dispatching",
            controllerRequestId=controller_id, claimedAt=int(time.time() * 1000))

    def finish_disposal(self, device_id, command, status, reason=""):
        if status not in ("accepted", "failed", "unconfirmed"):
            raise ValueError("Invalid disposal outcome")
        return self._transition_disposal(
            device_id, command, ("pending", "dispatching"), status,
            reason=reason, resultAt=int(time.time() * 1000))

    # ---- phone control and map (cloud_control.py, 2026-11-14) ----
    def get_commands(self, device_id):
        """The whole commands node: one read serves the operator request and the map lease."""
        return self._cloud_call(db.reference(f"/devices/{device_id}/commands").get) or {}

    def _transition_operator(self, device_id, command, expected, status, **fields):
        ref = db.reference(f"/devices/{device_id}/commands/operator")
        identity = ("requestId", "requestedAt", "requestedBy", "action")
        def transition(current):
            if (not isinstance(current, dict)
                    or any(current.get(key) != command.get(key) for key in identity)
                    or current.get("status") not in expected):
                return current
            return {**current, "status": status, **fields}
        result = self._cloud_call(ref.transaction, transition)
        return (isinstance(result, dict) and result.get("status") == status
                and all(result.get(key) == command.get(key) for key in identity)
                and all(result.get(key) == value for key, value in fields.items()))

    def claim_operator(self, device_id, command):
        return self._transition_operator(device_id, command, ("pending",), "claimed",
                                         claimedAt=int(time.time() * 1000))

    def finish_operator(self, device_id, command, status, reason=""):
        if status not in ("accepted", "failed", "unconfirmed"):
            raise ValueError("Invalid operator outcome")
        return self._transition_operator(device_id, command, ("pending", "claimed"), status,
                                         reason=str(reason)[:120], resultAt=int(time.time() * 1000))

    def update_operator_mirror(self, device_id, mirror):
        self._update({f"devices/{device_id}/operator": mirror})

    def update_map_frame(self, device_id, frame):
        self._update({f"devices/{device_id}/map/frame": frame})

    def update_disposal_availability(self, enabled):
        self._update({
            f"devices/{RESET_DEVICE_ID}/status/manualDisposalReady": bool(enabled),
            f"devices/{RESET_DEVICE_ID}/status/manualDisposalUpdatedAt": int(time.time() * 1000),
        })

    def update_robot_state(self, state):
        ready = state == "AT_DESTINATION"
        self._update({
            "RASPI/STATES/isReadyToReturn": ready,
            f"devices/{RESET_DEVICE_ID}/status/isReadyToReturn": ready,
            f"devices/{RESET_DEVICE_ID}/status/robotState": state,
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
        updated_at = 0 if value == -1 else int(time.time() * 1000)
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
        return self._cloud_call(db.reference("/RASPI/STATES/launchTime").get)

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

# ═══ Configuration ═══
def _resolve_sensor_serial_port():
    """An explicit override, or None to find the board by the data it sends."""
    return os.environ.get("GARBY_SENSOR_SERIAL_PORT") or None

SERIAL_PORT = _resolve_sensor_serial_port()
SERIAL_BAUD = int(os.environ.get("GARBY_SENSOR_SERIAL_BAUD", "9600"))
# Both roles search through one detector, so neither can open the other's port.
serial_ports = PortDetector(
    lidar_baud=int(os.environ.get("GARBY_LIDAR_BAUDRATE", "115200")),
    sensor_baud=SERIAL_BAUD,
)

SERVICE_UUID     = configured_uuid("GARBY_BLE_SERVICE_UUID", DEFAULT_SERVICE_UUID)
WRITE_CHAR_UUID  = configured_uuid("GARBY_BLE_WRITE_CHAR_UUID", DEFAULT_WRITE_CHAR_UUID)
NOTIFY_CHAR_UUID = configured_uuid("GARBY_BLE_NOTIFY_CHAR_UUID", DEFAULT_NOTIFY_CHAR_UUID)
DEVICE_NAME      = os.environ.get("GARBY_BLE_DEVICE_NAME", "GarbyESP32").strip() or "GarbyESP32"
BLE_MAC_ADDRESS  = os.environ.get("GARBY_BLE_ADDRESS", "").strip() or None

THRESHOLD_CM      = 95.0                  # straight-ahead (FRONT sector) stop distance
BACK_THRESHOLD_CM = 35.0                  # back distance shown on the monitor
# Every route segment drives forward and every turn spins in place under its own
# sweep (T=), so a return behind the body never holds a forward move (owner,
# 2026-11-14: the base spot has a wall 22.5 cm behind the LiDAR). The back group
# still reports unavailable data, and side clearance still holds both directions.
# Set this True only for a route with a reverse segment.
BACK_DISTANCE_HOLDS_MOTION = False
# FRONT_LEFT/FRONT_RIGHT also see hallway walls: a wall W cm to the side reads
# about 1.08 W, so 95 cm stopped the robot at any post narrower than 88 cm each
# side. At 60 cm it passes an 82/82 cm post up to about 26 cm off center. Do not
# go lower without a review: under 60 cm the bridge suppresses steering taps
# (FRONT_NUDGE_SUPPRESS_CM), and 60 * sin(22.5 deg) = 23 cm keeps everything
# inside the 22 cm side envelope under the FRONT sector's full 95 cm limit.
FRONT_DIAGONAL_THRESHOLD_CM = 60.0

# Safety/transport timing. A 10 Hz LiDAR should update about every 100 ms;
# 0.8 s therefore allows several missed scans but never permits seconds of
# blind travel. Telemetry is intentionally slower than the safety stream.
LIDAR_STALE_TIMEOUT_S = 0.8
SENSOR_TX_PERIOD_S    = 1.0
NOTICE_HEALTH_PERIOD_S = 5.0
SENSOR_OFFLINE_DB_DELAY_S = 8.0
SENSOR_DB_PERIOD_S = 5.0
RASPI_HEARTBEAT_PERIOD_S = 30.0
RASPI_HEALTH_POLL_S = 5.0
VCIO_DEVICE = "/dev/vcio"
HWMON_GLOB = "/sys/class/hwmon/hwmon*"
FIREBASE_RESET_POLL_S = 2.0
FIREBASE_IDLE_POLL_S = 10.0
LOAD_CELL_CHANGE_KG = 0.05
LOAD_CELL_REFRESH_S = 30.0
ALLOW_RUNTIME_SAFETY_DISABLE = True

# Software LiDAR convention is 0 deg FRONT / 90 deg LEFT. Keep the physical
# mounting yaw configurable. 180 reproduces the old opposite-facing convention.
try:
    LIDAR_YAW_OFFSET_DEG = float(os.environ.get("GARBY_LIDAR_YAW_OFFSET_DEG", "0"))
    if not math.isfinite(LIDAR_YAW_OFFSET_DEG):
        raise ValueError
except ValueError:
    LIDAR_YAW_OFFSET_DEG = 0.0
    logger.warning("Invalid GARBY_LIDAR_YAW_OFFSET_DEG; using 0 degrees")

# The owner reports LEFT/RIGHT reversed in the current physical scan view.
# Reflect the robot's lateral coordinate AFTER yaw, preserving FRONT/BACK.
# Other mounts can explicitly retain the raw handedness with this option = 0.
_lidar_reflection = os.environ.get("GARBY_LIDAR_REFLECT_LEFT_RIGHT", "1").strip().lower()
if _lidar_reflection not in ("0", "1", "false", "true", "no", "yes", "off", "on"):
    logger.warning("Invalid GARBY_LIDAR_REFLECT_LEFT_RIGHT; using enabled")
    _lidar_reflection = "1"
LIDAR_REFLECT_LEFT_RIGHT = _lidar_reflection in ("1", "true", "yes", "on")

# Chassis outline relative to the LiDAR (hallway_geometry.py). Its boxes only add
# STOP/SLOW evidence to the sector limits below, and its wall line fits supply the
# WL/WR/H centering fields. Width 60 cm is the owner's figure; the LiDAR-to-edge
# distances, LiDAR lateral offset and spin centre are unmeasured defaults.
CHASSIS = geometry_from_environment(warn=logger.warning)
# Long-baseline wall heading (hallway_geometry far fit, 2026-11-14). "0" keeps the
# near window's heading only, for a before/after floor comparison.
WALL_FAR_FIT = os.environ.get("GARBY_WALL_FAR_FIT", "1").strip() != "0"

# What the executor's wheels are doing ([DRIVE:...] lines, 2026-11-12), so the
# moving-object prediction can tell the robot's own motion from a person's.
ego_motion = EgoMotionState()
# The LiDAR's mounting angle, learned from sideways drift while driving
# (wall_heading_offset.py, 2026-11-14) and taken out of the H sent for centring.
# GARBY_WALL_HEADING_LEARN=0 turns it off for a before/after floor comparison.
# The route step in progress, from the executor's [LEG:..] notices, and the
# station steps on which the station's own walls do not hold the robot
# (station_mode.py, 2026-11-14). GARBY_STATION_WALL_MIN_GAP_CM (default 0 = off)
# keeps a wall this close ahead of the front edge as an obstacle; it needs a
# measured chassis outline.
station_mode = StationMode()
ROUTE_STATES = ("OUTBOUND", "RETURNING", "REVERSING")
try:
    STATION_WALL_MIN_GAP_CM = max(0.0, float(os.environ.get("GARBY_STATION_WALL_MIN_GAP_CM", "0")))
    if not math.isfinite(STATION_WALL_MIN_GAP_CM):
        raise ValueError
except ValueError:
    STATION_WALL_MIN_GAP_CM = 0.0
_route_leg_logged = ""
wall_heading_offset = WallHeadingOffset(
    mounting=(LIDAR_YAW_OFFSET_DEG, LIDAR_REFLECT_LEFT_RIGHT),
    enabled=os.environ.get("GARBY_WALL_HEADING_LEARN", "1").strip() != "0")

SIDES_TOLERANCE   = 5.0
BLE_CMD_PERIOD    = 0.25
# The controller polls at up to 12.5 Hz, but the X3 delivers about 6 scans/s, so
# extra snapshots repeat the same evidence. Each PATH is an acknowledged write
# (two connection events; 100 ms at a 50 ms interval), so unbounded replies keep
# the link saturated. Urgent STOP packets bypass this floor.
STATUS_MIN_PERIOD_S = 0.15
BLE_STEERING_MAX_BYTES = 244  # current bridge's negotiated-ATT write ceiling

REQUEST_TOKEN = "[REQUEST-STATUS]"
RASPI_READY   = "[RASPI READY]"
RESET_DEVICE_ID = "garby-bin-01"
RESET_REQUEST_MAX_AGE_MS = 120_000
RESET_REQUEST_FUTURE_TOLERANCE_MS = 30_000
RESET_COMPLETION_TIMEOUT_S = 900.0

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
lidar_streaming = threading.Event()


class LidarDriverStatus:
    """What the driver supervisor is doing, for the monitor's LiDAR map only.

    Phases: searching, starting, running, failed. This is presentation, never
    path evidence: scans and their age alone decide STOP/GO.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._state = ("searching", "Looking for the LiDAR", time.monotonic())

    def set(self, phase, message):
        with self._lock:
            if self._state[:2] != (phase, message):
                self._state = (phase, str(message), time.monotonic())

    def snapshot(self):
        with self._lock:
            phase, message, since = self._state
        return {"phase": phase, "message": message, "since": since}


lidar_driver_status = LidarDriverStatus()
sensor_serial_connected = threading.Event()
_load_cell_lock = threading.Lock()
_last_load_cell_value = None
_last_load_cell_publish = 0.0

ble_send_queue = CoalescingBleQueue(maxlen=12)
ble_recv_queue = queue.Queue(maxsize=100)

_status_requested = threading.Event()
_robot_running = threading.Event()
_reset_completed = threading.Event()
_reset_feedback = {}
_reset_feedback_lock = threading.Lock()
_ready_to_return = threading.Event()
_robot_state = "UNKNOWN"
latest_pi_health = {}
_motion_hold = ""
_disposal_ready_at = None
front_disabled = threading.Event()
back_disabled  = threading.Event()
manual_start = ManualStartControl()
operator_control = OperatorControl(StopLatchStore())
health_reporter = HealthReporter(cloud_expected=FIREBASE_IMPORT_ERROR is None)

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
                port = find_sensor_port()
                if port is None:
                    raise FileNotFoundError("no serial port is sending sensor lines")
                ser = serial.Serial(
                    port=port,
                    baudrate=SERIAL_BAUD,
                    timeout=0.25,
                    write_timeout=0.5,
                )
                try:
                    ser.reset_input_buffer()
                except Exception:
                    pass
                reconnect_backoff.reset()
                logger.info("Opened sensor serial %s at %d baud.", port, SERIAL_BAUD)
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


def _read_undervoltage_alarm():
    """Read the kernel's supply alarm, which needs no group membership."""
    for sensor in glob.glob(HWMON_GLOB):
        try:
            with open(os.path.join(sensor, "name"), "r", encoding="ascii") as handle:
                if handle.read().strip() != "rpi_volt":
                    continue
            with open(os.path.join(sensor, "in0_lcrit_alarm"), "r", encoding="ascii") as handle:
                return int(handle.read().strip())
        except (OSError, ValueError):
            continue
    return None


def _read_throttled_flags():
    # vcgencmd reports throttling as well as undervoltage but needs /dev/vcio
    # (video group), so it fails silently for a default user. Fall back to the
    # hwmon alarm, not soc:firmware/get_throttled: that file is deprecated and
    # logs a kernel WARN with a stack trace when read.
    if os.access(VCIO_DEVICE, os.R_OK | os.W_OK):
        try:
            result = subprocess.run(
                ["vcgencmd", "get_throttled"],
                capture_output=True,
                text=True,
                timeout=1.0,
                check=False,
            )
            return int(result.stdout.strip().split("=", 1)[-1], 16)
        except (OSError, ValueError, subprocess.SubprocessError):
            pass
    alarm = _read_undervoltage_alarm()
    if alarm is None:
        return None
    return THROTTLE_UNDERVOLTAGE_NOW if alarm else 0


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


def collect_notice_health():
    """Cloud result and actual sensor freshness, independent of internet probes."""
    with sensor_readings_lock:
        readings = dict(latest_sensor_readings)
    sensors_valid = sensor_serial_connected.is_set() and all(
        SENSOR_SPECS[key].normalize(readings.get(key)) is not None for key in SENSOR_SPECS)
    return {
        "cloud": firebase.cloud_health() if firebase else None,
        "lidar": lidar_healthy.is_set(),
        "sensors": sensors_valid,
        "cpu_temperature_c": _read_cpu_temperature_c(),
        "throttled_flags": _read_throttled_flags(),
    }


def notice_health_loop(stop_event):
    """A stalled probe never blocks Tk, ROS, BLE, or refreshes path clearance."""
    logger.info("SMS health monitor started.")
    while not stop_event.is_set():
        started = time.monotonic()
        try:
            internet = probe_internet()
            health_reporter.observe(internet, **collect_notice_health())
        except Exception:
            logger.exception("SMS health check unavailable; reporting unknown")
            health_reporter.invalidate()
        if ble_connected.is_set() and not stop_event.is_set():
            ble_send_queue.put_nowait(health_reporter.packet())
        stop_event.wait(max(0.0, NOTICE_HEALTH_PERIOD_S - (time.monotonic() - started)))
    logger.info("SMS health monitor stopped.")


def _health_text(status):
    return (f"{describe_throttle_flags(status.get('throttledFlags'))}, "
            f"CPU {status.get('cpuTemperatureC', 'unknown')} C")


def raspi_heartbeat_loop(stop_event):
    """Log supply/thermal changes locally; report to the cloud at its slower period.

    Health must not depend on Firebase: a BLE drop can only be correlated with
    undervoltage or throttling if the change is in this process's own log.
    """
    global latest_pi_health
    logger.info("RasPi heartbeat/health updater started.")
    logged_state = (0, False)
    last_cloud_beat = None
    while not stop_event.is_set():
        try:
            status = collect_health_status()
            latest_pi_health = status
            state = (status.get("throttledFlags"), bool(status.get("thermalWarning")))
            if state != logged_state:
                logged_state = state
                logger.log(logging.WARNING if state[1] else logging.INFO,
                           "Pi health changed: %s", _health_text(status))
            now = time.monotonic()
            if firebase and (last_cloud_beat is None
                             or now - last_cloud_beat >= RASPI_HEARTBEAT_PERIOD_S):
                firebase.update_raspi_heartbeat(status)
                last_cloud_beat = now
        except Exception as e:
            logger.error(f"RasPi heartbeat update failed: {e}")
        stop_event.wait(RASPI_HEALTH_POLL_S)
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
            manager.update_robot_state(_robot_state)
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
    accepted_at = 0.0
    last_reset_relay_at = 0.0
    poll_backoff = ExponentialBackoff(2.0, 30.0)

    while not stop_event.is_set():
        wait_s = (FIREBASE_RESET_POLL_S if in_flight_requested_at or
                  _ready_to_return.is_set() else FIREBASE_IDLE_POLL_S)
        try:
            if not firebase:
                stop_event.wait(FIREBASE_IDLE_POLL_S)
                continue

            command = firebase.get_reset_command(RESET_DEVICE_ID)
            poll_backoff.reset()
            is_command = isinstance(command, dict)
            status = str(command.get("status", "")).lower() if is_command else ""
            requested_at_raw = command.get("requestedAt", 0) if is_command else 0
            try:
                requested_at = int(requested_at_raw or 0)
            except (TypeError, ValueError):
                requested_at = 0

            should_send = status in ("pending", "ack")
            if should_send:
                wait_s = FIREBASE_RESET_POLL_S
            now_ms = int(time.time() * 1000)
            if should_send and requested_at <= 0:
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
            age_ms = now_ms - requested_at
            request_is_fresh = (
                should_send and
                -RESET_REQUEST_FUTURE_TOLERANCE_MS <= age_ms <= (
                    int(RESET_COMPLETION_TIMEOUT_S * 1000) if status == "ack" else RESET_REQUEST_MAX_AGE_MS
                )
            )
            # Acceptance freshness is checked once. A legitimate long return
            # must not expire after 120 seconds while the MCU is still running.
            if should_send and not request_is_fresh and requested_at != in_flight_requested_at:
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
            if operator_control.blocks_motion() and (is_new_request or in_flight_requested_at):
                blocked_id = in_flight_requested_at or requested_at
                firebase.mark_reset_failed(RESET_DEVICE_ID, blocked_id, "operator_emergency_stop")
                completed_requested_at = blocked_id
                in_flight_requested_at = 0
                stop_event.wait(FIREBASE_RESET_POLL_S)
                continue
            if is_new_request:
                if in_flight_requested_at:
                    firebase.mark_reset_failed(RESET_DEVICE_ID, requested_at, "return_already_in_progress")
                    stop_event.wait(FIREBASE_RESET_POLL_S)
                    continue
                in_flight_requested_at = requested_at
                accepted_at = time.monotonic()
                last_reset_relay_at = 0.0

            with _reset_feedback_lock:
                feedback = _reset_feedback.get(in_flight_requested_at)
            if in_flight_requested_at and feedback in ("DONE", "FAILED"):
                if feedback == "DONE":
                    firebase.mark_reset_done(RESET_DEVICE_ID, in_flight_requested_at)
                else:
                    firebase.mark_reset_failed(RESET_DEVICE_ID, in_flight_requested_at, "controller_rejected_or_route_incomplete")
                completed_requested_at = in_flight_requested_at
                in_flight_requested_at = 0
            elif in_flight_requested_at and feedback == "ACK":
                if status != "ack":
                    firebase.mark_reset_ack(RESET_DEVICE_ID, in_flight_requested_at)

            # A notification can be lost while BLE reconnects at arrival.
            # Re-query the same idempotent ID even after ACK so the MCU can
            # return its remembered DONE/FAILED without restarting motion.
            retry_interval = 10.0 if feedback == "ACK" else 3.0
            if (in_flight_requested_at and ble_connected.is_set() and
                    time.monotonic() - last_reset_relay_at >= retry_interval):
                ble_send_queue.put_nowait(f"[RESET:{in_flight_requested_at}]", urgent=True)
                last_reset_relay_at = time.monotonic()

            if in_flight_requested_at and time.monotonic() - accepted_at > RESET_COMPLETION_TIMEOUT_S:
                firebase.mark_reset_failed(RESET_DEVICE_ID, in_flight_requested_at, "completion_unverified")
                completed_requested_at = in_flight_requested_at
                in_flight_requested_at = 0
        except Exception as e:
            logger.error(f"Firebase reset bridge error: {e}")
            wait_s = poll_backoff.next_delay()

        stop_event.wait(wait_s)

    logger.info("Firebase reset command bridge stopped.")


def manual_disposal_available():
    age = time.monotonic() - _disposal_ready_at if _disposal_ready_at is not None else math.inf
    return (0 <= age <= READY_MAX_AGE_S and not operator_control.blocks_motion()
            and manual_start.status(ble_connected.is_set())["enabled"])


def publish_disposal_availability():
    if firebase and hasattr(firebase, "update_disposal_availability"):
        firebase.update_disposal_availability(manual_disposal_available())


def firebase_disposal_command_loop(stop_event):
    relay = DisposalRelay(manual_start,
                          lambda packet: ble_send_queue.put_nowait(packet, urgent=True),
                          manual_disposal_available)
    backoff = ExponentialBackoff(2.0, 30.0)
    while not stop_event.is_set():
        wait_s = 2.0
        try:
            if firebase:
                relay.poll(firebase, RESET_DEVICE_ID, ble_connected.is_set(),
                           operator_control.blocks_motion())
                backoff.reset()
            else:
                wait_s = FIREBASE_IDLE_POLL_S
        except Exception:
            logger.exception("Manual disposal cloud relay unavailable")
            wait_s = backoff.next_delay()
        stop_event.wait(wait_s)

# ═══ On-demand status responder ═══
def on_demand_status_responder(lidar_node, stop_event):
    logger.info("On-demand status responder started.")
    last_status_send = 0.0
    last_sensor_send = 0.0
    last_sides_send = None
    last_error_log = 0.0
    fallback_seq = 0
    while not stop_event.is_set():
        # Wake immediately when requested by MCU, or tick periodically at BLE_CMD_PERIOD (250 ms)
        if _status_requested.wait(timeout=BLE_CMD_PERIOD):
            wait_s = STATUS_MIN_PERIOD_S - (time.monotonic() - last_status_send)
            if wait_s > 0.0:
                stop_event.wait(wait_s)
        _status_requested.clear()

        if not ble_connected.is_set():
            continue

        last_status_send = time.monotonic()
        # The bridge's path watchdog depends on this stream, and it disconnects
        # after 10 s of silence. A dead thread here would make every later
        # reconnect drop again with no visible cause.
        try:
            if lidar_node is not None:
                build_fresh_snapshot = getattr(lidar_node, "build_status_packets_with_deadline", None)
                if callable(build_fresh_snapshot):
                    path_msg, sides_msg, path_deadline = build_fresh_snapshot()
                else:
                    path_msg, sides_msg = lidar_node.build_status_packets()
                    path_deadline = None
            else:
                # Fail closed if the LiDAR node is unavailable.
                fallback_seq = (fallback_seq + 1) & 0xFFFFFFFF
                path_msg = f"P:{fallback_seq}|F=S|B=S"
                sides_msg = f"S:{fallback_seq}|STABLE"
                path_deadline = None

            # Safety status always goes first. The mailbox coalesces older status
            # so a stale CLEAR cannot sit behind a newer STOP.
            ble_send_queue.put_nowait(path_msg, expires_at=path_deadline)

            now = time.monotonic()
            # MCU polls can arrive faster than the normal 250 ms cadence. Keep
            # immediate safety replies, but avoid repeating long steering writes
            # for every poll. Each emitted S retains this snapshot's P sequence.
            if last_sides_send is None or now - last_sides_send >= BLE_CMD_PERIOD:
                ble_send_queue.put_nowait(sides_msg, expires_at=path_deadline)
                last_sides_send = now
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
        except Exception:
            # Missing packets become STOP at the bridge, so continuing is fail-closed.
            if last_status_send - last_error_log >= 5.0:
                last_error_log = last_status_send
                logger.exception("Status snapshot failed; bridge will hold STOP until it recovers")

    logger.info("On-demand status responder stopped.")

# ═══ BLE notification handler ═══
def publish_robot_state(state: str):
    global _robot_state, _motion_hold, _disposal_ready_at
    changed = state != _robot_state
    _robot_state = state
    if state == "UNKNOWN":
        _disposal_ready_at = None
        operator_control.disconnect()
        station_mode.invalidate()  # no controller state: back on the ordinary rules
    else:
        operator_control.note_state(state)
    if state == "UNKNOWN":
        _motion_hold = ""
        manual_start.reset()
        ble_send_queue.discard_kind("START")
        ble_send_queue.discard_kind("START_STATUS")
    else:
        manual_start.note_state(state)
    if changed:
        logger.info("Executor state: %s", state)
    if state == "AT_DESTINATION":
        _ready_to_return.set()
    else:
        _ready_to_return.clear()
    if firebase:
        firebase.update_robot_state(state)
    publish_disposal_availability()


def request_manual_start():
    if operator_control.blocks_motion():
        return {"enabled": False, "pending": False,
                "message": "Emergency stop latched; release stop before starting"}
    packet = manual_start.request(ble_connected.is_set())
    if packet is not None:
        ble_send_queue.put_nowait(packet, urgent=True)
        _status_requested.set()
        logger.info("Manual start requested; bypassing scale/bin trigger thresholds.")
        publish_disposal_availability()
    result = manual_start.status(ble_connected.is_set())
    if packet is None:
        logger.info("Manual start unavailable: %s", result["message"])
    return result


def safety_checks_changed():
    """Request a new path snapshot after a directional threshold bypass changes."""
    _status_requested.set()


def _request_operator_packet(action, source="monitor"):
    packet = operator_control.request(action, ble_connected.is_set())
    if action == "ESTOP":
        manual_start.reset()
        for kind in ("START", "START_STATUS", "RESET"):
            ble_send_queue.discard_kind(kind)
    if packet:
        logger.warning("Operator action requested: %s%s", action,
                       "" if source == "monitor" else f" (from the {source})")
        _status_requested.set()
    publish_disposal_availability()
    return packet


def request_operator_action(action):
    _request_operator_packet(action)
    return operator_control.status(ble_connected.is_set())


def cloud_request_operator_action(action):
    """The phone's request through the monitor's own gate: (request ID or None, message)."""
    packet = _request_operator_packet(action, "phone")
    if packet is None:
        return None, operator_control.status(ble_connected.is_set())["message"]
    return packet[4:-1].split(":", 1)[0], ""


def cloud_set_read_only(group, enabled):
    """Front/back LiDAR read-only from the phone, under the Pi's own permission."""
    if not ALLOW_RUNTIME_SAFETY_DISABLE:
        return False, "not_permitted"
    event = front_disabled if group == "front" else back_disabled
    if enabled:
        event.set()
    else:
        event.clear()
    logger.warning("%s LiDAR distance checks %s (from the phone)", group.capitalize(),
                   "set to read-only" if enabled else "re-enabled")
    safety_checks_changed()
    return True, ""


_launcher_info = {"mode": "monitor", "startedAt": int(time.time() * 1000),
                  "software": PI_SOFTWARE_ID}
_lidar_node_ref = None


def build_operator_mirror():
    node = _lidar_node_ref
    people = objects = 0
    zones = ()
    if node is not None:
        people, objects, zones = node.cloud_view()
    return cloud_control.operator_mirror(
        operator_control.status(ble_connected.is_set()), robot_state=_robot_state,
        front_read_only=front_disabled.is_set(), back_read_only=back_disabled.is_set(),
        read_only_allowed=ALLOW_RUNTIME_SAFETY_DISABLE,
        motion_hold=_motion_hold if _robot_state in ROUTE_STATES else "",
        zones=zones, route_leg=station_mode.leg().describe() if _robot_state in ROUTE_STATES else "",
        people=people, objects=objects, launcher=dict(_launcher_info))


# Read once at start: a later environment change must not move it.
STATUS_FILE = os.environ.get("GARBY_STATUS_FILE") or os.path.join(
    os.path.expanduser("~"), ".local", "state", "garby", "bridge-status.json")


def _write_status_file(mirror):
    """The same status for `run_headless.sh status`, with or without a cloud link."""
    path = STATUS_FILE
    try:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        temporary = f"{path}.{os.getpid()}.tmp"
        with open(temporary, "w", encoding="utf-8") as handle:
            json.dump({**mirror, "pid": os.getpid(), "bleConnected": ble_connected.is_set(),
                       "lidarHealthy": lidar_healthy.is_set(),
                       "cloud": (firebase.cloud_health()["state"] if firebase else "Unavailable")},
                      handle)
        os.replace(temporary, path)
    except OSError:
        pass  # a status file is a convenience; nothing depends on it


def firebase_control_loop(stop_event):
    """Phone control: operator requests, the status mirror and leased map frames."""
    relay = cloud_control.OperatorRelay(cloud_request_operator_action, operator_control.outcome,
                                        cloud_set_read_only)
    publisher = cloud_control.MapPublisher()
    last_poll = last_mirror_at = last_file_at = last_error_log = float("-inf")
    last_mirror = None
    lease = None
    logger.info("Phone control bridge started.")
    while not stop_event.is_set():
        now = time.monotonic()
        try:
            mirror = build_operator_mirror()
            if now - last_file_at >= 5.0:
                last_file_at = now
                _write_status_file(mirror)
            manager = firebase
            if manager is not None and hasattr(manager, "get_commands"):
                if now - last_poll >= cloud_control.COMMAND_POLL_S:
                    last_poll = now
                    commands = manager.get_commands(RESET_DEVICE_ID)
                    commands = commands if isinstance(commands, dict) else {}
                    relay.poll(manager, RESET_DEVICE_ID, commands.get("operator"))
                    lease = commands.get("map")
                    mirror = build_operator_mirror()
                if (cloud_control.mirror_changed(last_mirror, mirror)
                        or now - last_mirror_at >= cloud_control.STATUS_REFRESH_S):
                    last_mirror, last_mirror_at = mirror, now
                    manager.update_operator_mirror(RESET_DEVICE_ID, mirror)
                node = _lidar_node_ref
                hot = bool(latest_pi_health.get("thermalWarning")) or (
                    (latest_pi_health.get("cpuTemperatureC") or 0) >= 75.0)
                if (node is not None and cloud_control.lease_fresh(lease, int(time.time() * 1000))
                        and publisher.due(hot, now)):
                    frame = node.cloud_map_frame(publisher.seq)
                    if frame is not None:
                        manager.update_map_frame(RESET_DEVICE_ID, frame)
        except Exception:
            if now - last_error_log >= 30.0:
                last_error_log = now
                logger.exception("Phone control bridge unavailable; retrying")
        stop_event.wait(0.25)
    logger.info("Phone control bridge stopped.")


def _reboot_after_controller_ack():
    # The bridge and executor delay their own restarts to drain acknowledgement
    # traffic. Keep the BLE worker responsive during the OS request.
    if serial_stop.wait(2.0) or not operator_control.reboot_allowed():
        return
    try:
        reboot_pi()
    except Exception as exc:
        logger.error("Pi reboot failed; emergency stop remains latched: %s", exc)
        operator_control.reboot_result(str(exc))
    else:
        operator_control.reboot_result()


def ble_notify_handler(msg: str):
    global _last_load_cell_value, _last_load_cell_publish, _motion_hold, _disposal_ready_at
    global _route_leg_logged
    stripped = msg.strip()
    if stripped.startswith("[DRIVE:"):
        # Wheel motion for the moving-object prediction: on every change and every
        # 2 s, so it stays out of the GUI log. Telemetry only; it grants nothing.
        ego_motion.update_from_line(stripped)
        return
    if station_mode.update_from_line(stripped):
        # The route step in progress, repeated every 2 s (kept out of the GUI
        # log). Logged once per step: a floor run then shows where it was.
        described = station_mode.leg().describe()
        if described != _route_leg_logged:
            _route_leg_logged = described
            logger.info("Route step: %s", described or "none (route ended)")
        return
    # Status polls drive control but carry no useful GUI log information.
    # Excluding them also leaves room for fault and route notifications.
    if stripped not in (REQUEST_TOKEN, "[SYSTEM READY]"):
        try:
            ble_recv_queue.put_nowait(msg)
        except queue.Full:
            pass
    if stripped.startswith(("[BRIDGE ", "[HEADING ", "[HEADING:START", "[ROUTE ",
                            "[REVERSE COMPLETE")):
        # Firmware ID, reset cause and link timing separate a supply problem from
        # a radio drop. Headless mode would otherwise discard them with the GUI queue.
        # [HEADING ...] lines are the executor's turn, straight and wall-fusion
        # reports: the evidence a floor run leaves for tuning.
        logger.info("%s", stripped)
        return
    if stripped in ("[POWER:LOW]", "[POWER:NORMAL]"):
        logger.info("Executor %s", "low-power hold (blocked; resumes when clear)"
                    if stripped == "[POWER:LOW]" else "back to normal power")
        return
    if stripped in ("[SONAR:SILENT]", "[SONAR:FAULT]"):
        logger.warning("Executor obstacle sonar %s: it cannot see low objects; check the sensor",
                       "heard no echo at any bearing in three full sweeps" if stripped == "[SONAR:SILENT]"
                       else "ECHO line stuck high")
        return
    if stripped.startswith("[OP "):
        if operator_control.notification(stripped) == "REBOOT":
            threading.Thread(target=_reboot_after_controller_ack, daemon=True,
                             name="operator-pi-reboot").start()
        publish_disposal_availability()
        return
    if stripped == "[DISPOSAL READY:1]":
        _disposal_ready_at = time.monotonic()
        publish_disposal_availability()
        return
    if stripped == "[SYSTEM READY]":
        manual_start.note_ready()
        return
    if stripped.startswith("[MOTION:") and stripped.endswith("]"):
        reason = stripped[8:-1]
        holds = {
            "CLEAR": "", "STOP": "LiDAR obstacle", "STOP:HUMAN": "LiDAR obstacle",
            "STOP:MPU": "MPU not ready", "STOP:STALE": "Stale LiDAR data",
            "STOP:WAITING_DATA": "Waiting for LiDAR", "WAITING_FOR_PATH": "Waiting for path data",
            "STOP:CLEARING": "Confirming clearance", "STOP:LINK": "BLE link unavailable",
            "CONFIRMING_CLEARANCE": "Confirming clearance",
            "STOP:MCU_LINK": "Controller UART unavailable", "STOP:PROTOCOL": "Invalid path data",
            "STOP:SEQUENCE": "Path sequence error", "LOCAL_OBSTACLE": "Local sonar obstacle",
            "MOTORS_UNAVAILABLE": "Motor outputs unavailable", "ROUTE_FAULT": "Route fault",
            "STOP:TURN": "Turn sweep blocked", "STOP:TURN_UNKNOWN": "Turn sweep unknown",
            "STOP:TURN_CLEARANCE": "Confirming turn clearance",
        }
        hold = holds.get(reason, reason.replace("_", " "))
        if hold != _motion_hold:
            _motion_hold = hold
            logger.info("Executor motion: %s", hold or "clearance confirmed")
        return
    if stripped.startswith("[START ") and stripped.endswith("]"):
        fields = stripped[7:-1].split(":", 2)
        if len(fields) >= 2 and fields[1].isdecimal():
            outcome, request_id = fields[:2]
            if (outcome == "ACK" and len(fields) == 2) or (outcome == "FAILED" and len(fields) == 3):
                accepted = outcome == "ACK"
                reason = fields[2] if len(fields) == 3 else ""
                if manual_start.outcome(request_id, accepted, reason):
                    logger.info("Manual start %s: %s", request_id,
                                "accepted by executor" if accepted else f"rejected ({reason})")
                    publish_disposal_availability()
        return
    if stripped.startswith("[RESET ") and stripped.endswith("]"):
        try:
            outcome, request_id = stripped[7:-1].split(":", 1)
            if outcome in ("ACK", "DONE", "FAILED") and request_id.isdecimal():
                with _reset_feedback_lock:
                    if _reset_feedback.get(int(request_id)) not in ("DONE", "FAILED"):
                        _reset_feedback[int(request_id)] = outcome
                    while len(_reset_feedback) > 32:
                        del _reset_feedback[next(iter(_reset_feedback))]
        except ValueError:
            pass
        return
    states = {
        "[MCU READY]": "UNKNOWN", "[RUNNING]": "OUTBOUND",
        "[OUTBOUND COMPLETE]": "AT_DESTINATION", "[RETURNING]": "RETURNING",
        "[REVERSING]": "REVERSING",
        "[IDLE]": "IDLE", "[ROUTE FAULT]": "FAULT", "[RETURN REJECTED]": "FAULT",
    }
    if stripped == "[MCU READY]":
        ego_motion.invalidate()  # a restarted controller: wheel state unknown
        station_mode.invalidate()
    if stripped in states:
        publish_robot_state(states[stripped])
    if stripped in (REQUEST_TOKEN, "CONNECTED...", "[BLE CONNECTION ESTABLISHED]"):
        _robot_running.set()
        _status_requested.set()
        return
    if stripped.startswith("LOAD_CELL:"):
        try:
            value = float(stripped.split(":", 1)[1])
            if not math.isfinite(value) or (value != -1 and (value < 0.0 or value > 200.0)):
                raise ValueError("load-cell value outside 0..200 kg or -1 unavailable sentinel")
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

# ═══ BLE service: one connection task and one serialized writer ═══
class BleService(threading.Thread):
    # A 1 s first retry is enough for the ESP32 to restart advertising after a
    # throttled-link drop; the exponential growth still caps storm pressure.
    RETRY_MIN_S    = 1.0
    RETRY_MAX_S    = 30.0
    SCAN_TIMEOUT   = 12.0
    CONNECT_TIMEOUT = 30.0
    NOTIFY_TIMEOUT = 8.0
    WRITE_TIMEOUT = 2.0
    DRAIN_INTERVAL  = 0.02
    # Adaptive write-retry pacing for CPU/connection throttling: transient
    # congestion shows up as slow or timed-out writes. Escalate the retry gap
    # instead of dropping the session after the first glitch; the ESP32 link
    # watchdog tolerates 10 s of silence, so patient retries recover faster
    # than a full rescan cycle.
    WRITE_RETRY_STRIKES = 5
    WRITE_RETRY_BASE_S  = 0.05
    WRITE_RETRY_MAX_S   = 0.8

    def __init__(self, status_cb, notify_cb):
        super().__init__(daemon=True, name="ble-service")
        self.status_cb    = status_cb
        self.notify_cb    = notify_cb
        self._loop        = None
        self._client      = None
        self._candidate_client = None
        self._connection_phase = "idle"
        self._notification_decoder = NotificationDecoder()
        self._ended_client = None
        self._session_started_at = None
        self._last_write_ok_at = None
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
        pending = [task for task in (self._connect_task, self._drain_task)
                   if task and not task.done()]
        for task in pending:
            task.cancel()
        # Let the connection guard finish its bounded candidate cleanup before
        # closing the event loop or disconnecting the established client.
        if pending:
            await asyncio.gather(*pending, return_exceptions=True)
        client = self._client
        if client and client.is_connected:
            await self._close_quietly(client)
        self._client = None
        self._write_char = None
        self._notify_char = None
        ble_connected.clear()
        _robot_running.clear()
        ble_send_queue.clear()
        self._notification_decoder.reset()
        publish_robot_state("UNKNOWN")

    @staticmethod
    async def _close_quietly(client):
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
                if self._connect_task and not self._connect_task.done():
                    self._schedule_connect(0.1)
                    return
                self._connect_task = self._loop.create_task(self._connect_guarded())

        self._retry_handle = self._loop.call_later(delay, launch)

    async def _connect_guarded(self):
        connected = False
        cancelled = False
        try:
            connected = await self._connect()
        except asyncio.CancelledError:
            cancelled = True
            raise
        except Exception as exc:
            logger.warning("BLE %s failed: %s: %s", self._connection_phase, type(exc).__name__, exc)
        finally:
            candidate = self._candidate_client
            self._candidate_client = None
            if not connected and candidate is not None:
                if self._client is candidate:
                    self._client = None
                    self._write_char = None
                    self._notify_char = None
                await self._close_quietly(candidate)
            self._connecting = False
        if not connected and not cancelled:
            self._schedule_retry(f"BLE {self._connection_phase} failed")

    def _schedule_retry(self, reason="Disconnected"):
        if not self._loop or self._loop.is_closed() or self._stop_event.is_set():
            return
        self._connecting = False
        _robot_running.clear()
        ble_connected.clear()
        publish_robot_state("UNKNOWN")
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
        if disconnected_client is not self._client:
            return
        self._client = None
        self._ended_client = disconnected_client
        self._write_char = None
        self._notify_char = None
        if self._drain_task and not self._drain_task.done():
            self._drain_task.cancel()
        _robot_running.clear()
        ble_connected.clear()
        ble_send_queue.clear()
        ego_motion.invalidate()
        publish_robot_state("UNKNOWN")
        self._notification_decoder.reset()
        logger.warning("BLE link lost: %s", self._session_summary())
        if not self._connecting:
            self._schedule_retry("Disconnected")

    def _session_summary(self):
        now = time.monotonic()
        session_s = now - self._session_started_at if self._session_started_at else 0.0
        write_age_s = now - self._last_write_ok_at if self._last_write_ok_at else -1.0
        return (f"session {session_s:.1f} s, last write {write_age_s:.2f} s ago, "
                f"Pi {_health_text(latest_pi_health)}")

    def _notification_handler(self, sender, data):
        if self.notify_cb:
            for part in self._notification_decoder.feed(data):
                self.notify_cb(part)

    async def _connect(self):
        if self._stop_event.is_set():
            return False
        if self._client and self._client.is_connected:
            return True
        self._connecting = True
        # Bleak only closes a session's D-Bus connection in disconnect(), even when
        # the link has already gone. Until then BlueZ keeps that session's
        # notification registration and re-arms it on connect, so every notification
        # arrives once more per leaked session, corrupting lines over two frames.
        ended_client, self._ended_client = self._ended_client, None
        if ended_client is not None:
            await self._close_quietly(ended_client)
        self._connection_phase = "discovery"
        self._status("Scanning for GARBY...", "#d29922")
        ble_send_queue.clear()
        manual_start.reset()
        operator_control.disconnect()
        self._notification_decoder.reset()
        discovery = DiscoveryDiagnostics()
        target = await asyncio.wait_for(discover_robot(
            BleakScanner, address=BLE_MAC_ADDRESS, name=DEVICE_NAME,
            service_uuid=SERVICE_UUID, timeout=self.SCAN_TIMEOUT, diagnostics=discovery),
            timeout=self.SCAN_TIMEOUT + 5.0)
        if target is None:
            raise ConnectionError(
                f"{DEVICE_NAME}: {discovery.failure_message()}; "
                "check selection, adapter/rfkill, ESP32 advertising, and other connected clients")

        self._connection_phase = "connection/GATT discovery"
        logger.info("BLE found %s [%s]; connecting", target.name or DEVICE_NAME, target.address)
        self._status(f"Connecting {target.address}...", "#d29922")
        client = BleakClient(
            target, disconnected_callback=self._on_disconnect,
            timeout=self.CONNECT_TIMEOUT, **adapter_options())
        self._candidate_client = client
        await asyncio.wait_for(client.connect(), timeout=self.CONNECT_TIMEOUT)
        if not client.is_connected or self._stop_event.is_set():
            raise ConnectionError("BLE disconnected before GATT setup")
        self._client = client

        write_char, notify_char = robot_characteristics(
            client, SERVICE_UUID, WRITE_CHAR_UUID, NOTIFY_CHAR_UUID)
        self._connection_phase = "notification subscription"
        await asyncio.wait_for(
            subscribe(client, notify_char,
                      lambda sender, data: self._session_notification(client, sender, data)),
            timeout=self.NOTIFY_TIMEOUT)
        # A disconnect callback can run while start_notify is suspended.
        if self._client is not client or not client.is_connected or self._stop_event.is_set():
            raise ConnectionError("BLE session changed during notification setup")

        self._write_char = write_char
        self._notify_char = notify_char
        self._connection_phase = "readiness write"
        await self._write_message(RASPI_READY, expected_client=client)
        if self._client is not client or not client.is_connected or self._stop_event.is_set():
            raise ConnectionError("BLE disconnected during readiness write")
        self._retry_backoff.reset()
        self._session_started_at = time.monotonic()
        _robot_running.set()
        ble_connected.set()
        _status_requested.set()
        self._connection_phase = "ready"
        self._status("Connected", "#3fb950")
        logger.info("BLE ready: notifications subscribed and readiness write acknowledged")
        self._drain_task = self._loop.create_task(self._drain_loop(client))
        return True

    def _session_notification(self, client, sender, data):
        # Queued callbacks from an old session cannot publish readiness or
        # append fragments to the replacement session's notification decoder.
        if client is self._client and client.is_connected and not self._stop_event.is_set():
            self._notification_handler(sender, data)

    async def _write_message(self, msg, expected_client=None, expires_at=None):
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
        if not payload or len(payload) > 244 or any(c < 32 or c > 126 for c in payload):
            raise ValueError("BLE packet must be one printable line of at most 244 bytes")
        # The no-response limit can remain 20 bytes on older BlueZ versions or
        # while MTU negotiation is pending. Larger steering/telemetry frames
        # must use a write request rather than failing or being truncated.
        no_response_limit = getattr(write_char, "max_write_without_response_size", 20)
        async with self._write_lock:
            if client is not self._client or not client.is_connected:
                raise ConnectionError("BLE session changed before write")
            if msg.startswith("[HEALTH:"):
                # Check acquisition age after waiting for the serialized writer;
                # a delayed queue item must not become fresh evidence on arrival.
                msg = health_reporter.packet()
                payload = msg.encode("utf-8")
            if msg.startswith("[OP") and not operator_control.can_send(msg, ble_connected.is_set()):
                return
            if operator_control.blocks_motion():
                if msg.startswith(("[START", "[RESET")):
                    return
                if msg.startswith("P:"):
                    msg = msg.split("|", 1)[0] + "|F=S|B=S"
                    payload = msg.encode("utf-8")
            # A queued LiDAR snapshot cannot carry old CLEAR evidence across
            # its acquisition-age deadline. Revalidate after writer-lock wait
            # and immediately before starting the GATT operation.
            if (msg.startswith("P:") and expires_at is not None
                    and time.monotonic() >= expires_at):
                msg = msg.split("|", 1)[0] + "|F=S|B=S"
                payload = msg.encode("utf-8")
                logger.warning("Expired LiDAR path replaced with fail-closed STOP before BLE write")
            if msg.startswith("S:") and expires_at is not None:
                # The same acquisition deadline travels with P and S through
                # coalescing and the writer lock. Refresh G at the write boundary
                # so queue delays cannot rejuvenate clearance or misdate walls.
                now = time.monotonic()
                if now >= expires_at:
                    msg = msg.split("|", 1)[0] + "|STABLE"
                else:
                    acquired_at = expires_at - LIDAR_STALE_TIMEOUT_S
                    age_ms = max(0, math.ceil((now - acquired_at) * 1000.0))
                    msg = "|".join(f"G={age_ms}" if part.startswith("G=") else part
                                   for part in msg.split("|"))
                    if len(msg.encode("utf-8")) > BLE_STEERING_MAX_BYTES:
                        msg = msg.split("|", 1)[0] + "|STABLE"
                payload = msg.encode("utf-8")
            # Re-check at the write itself: a blocked writer must not deliver
            # an expired operator action after a reconnect or controller boot.
            if msg.startswith("[START:") and not manual_start.can_send(msg, ble_connected.is_set()):
                logger.warning("Manual start skipped before transmission: %s",
                               manual_start.status(ble_connected.is_set())["message"])
                return
            if msg.startswith("[START STATUS:") and not manual_start.can_query(msg, ble_connected.is_set()):
                return
            if msg.startswith("[START:"):
                manual_start.mark_sent(msg)
            if msg.startswith("[OP:"):
                operator_control.mark_sent(msg)
            acknowledged = (
                msg.startswith(("P:", "[RESET", "[START", "[OP", "[HEALTH:", "[RASPI READY]"))
                or len(payload) > no_response_limit
            )
            await asyncio.wait_for(
                client.write_gatt_char(write_char, payload, response=acknowledged),
                timeout=self.WRITE_TIMEOUT,
            )
            self._last_write_ok_at = time.monotonic()
            if msg.startswith("[START:"):
                logger.info("Manual start %s written to BLE bridge; awaiting executor acknowledgement", msg[7:-1])

    async def _drain_loop(self, session_client):
        consecutive_write_failures = 0
        while (
            not self._stop_event.is_set()
            and self._client is session_client
            and session_client.is_connected
        ):
            query = None if operator_control.blocks_motion() else manual_start.next_query(ble_connected.is_set())
            if query:
                ble_send_queue.put_nowait(query)
            msg = operator_control.next_packet(ble_connected.is_set())
            expires_at = None
            if msg is None:
                try:
                    msg, expires_at = ble_send_queue.get_entry_nowait()
                except queue.Empty:
                    await asyncio.sleep(self.DRAIN_INTERVAL)
                    continue
            try:
                await self._write_message(msg, expected_client=session_client,
                                          expires_at=expires_at)
                consecutive_write_failures = 0
                # Small inter-write yield keeps acknowledged GATT writes from
                # saturating a throttled connection interval back-to-back.
                await asyncio.sleep(0.005)
            except Exception as exc:
                consecutive_write_failures += 1
                # Escalating gaps ride out throttling-induced write stalls;
                # only a persistent failure drops the session.
                retry_delay = min(
                    self.WRITE_RETRY_MAX_S,
                    self.WRITE_RETRY_BASE_S * (2 ** (consecutive_write_failures - 1)),
                )
                logger.warning(
                    "BLE write glitch (%d/%d): %s: %s [packet=%s, connected=%s]",
                    consecutive_write_failures, self.WRITE_RETRY_STRIKES,
                    type(exc).__name__, str(exc) or "no error detail",
                    CoalescingBleQueue._kind(msg), session_client.is_connected,
                )
                if consecutive_write_failures < self.WRITE_RETRY_STRIKES:
                    # Never reinsert an old path packet: a newer STOP may already
                    # be queued. Ask the producer for a fresh sequenced snapshot.
                    if msg.startswith("P:"):
                        _status_requested.set()
                    elif msg.startswith("[RESET") or msg == RASPI_READY:
                        ble_send_queue.put_nowait(msg, urgent=True)
                    await asyncio.sleep(retry_delay)
                    continue
                logger.error("BLE persistent write failure: %s: %s; %s",
                             type(exc).__name__, exc, self._session_summary())
                if self._client is session_client:
                    self._client = None
                    self._write_char = None
                    self._notify_char = None
                    ble_send_queue.clear()
                    ble_connected.clear()
                    _robot_running.clear()
                    publish_robot_state("UNKNOWN")
                    self._notification_decoder.reset()
                await self._close_quietly(session_client)
                self._schedule_retry("Write failed")
                return
        logger.info("BLE drain loop exiting.")

    async def _urgent_write(self, msg):
        """Serialized, acknowledged safety write from the LiDAR callback."""
        try:
            await self._write_message(msg)
        except Exception as exc:
            logger.error(f"Urgent BLE write error: {exc}")
            # Replaying this old sequence could replace a newer STOP snapshot.
            _status_requested.set()

# ═══ GUI ═══
def create_gui_root(headless=False):
    """Check the display before starting workers; fall back to headless mode."""
    if headless:
        return None
    if sys.platform.startswith("linux") and not os.environ.get("DISPLAY"):
        logger.info("No DISPLAY is set; running in headless mode.")
        return None
    try:
        import tkinter as tk
    except ImportError as exc:
        logger.warning("Tkinter is unavailable (%s); running in headless mode.", exc)
        return None
    try:
        return tk.Tk()
    except tk.TclError as exc:
        logger.warning("GUI display is unavailable (%s); running in headless mode.", exc)
        return None


def init_gui():
    # Imported only for desktop mode; headless operation does not need Tcl/Tk.
    from monitor_ui import LidarGUI as MonitorGUI

    def read_sensors():
        with sensor_readings_lock:
            return dict(latest_sensor_readings)

    def read_health():
        cloud = firebase.cloud_health() if firebase else {"online": False, "state": "Unavailable"}
        start_status = manual_start.status(ble_connected.is_set())
        if operator_control.blocks_motion():
            start_status = {"enabled": False, "pending": False,
                            "message": "Emergency stop latched; explicit release required"}
        if (not start_status["pending"] and _robot_state in ROUTE_STATES
                and _motion_hold):
            start_status["message"] = f"Motors held: {_motion_hold}"
        return {
            "lidar": lidar_healthy.is_set(),
            "lidar_stream": lidar_streaming.is_set(),
            "lidar_driver": lidar_driver_status.snapshot(),
            "sensors": sensor_serial_connected.is_set(),
            "firebase": cloud["online"],
            "firebase_status": cloud["state"],
            "robot_state": _robot_state,
            "route_leg": station_mode.leg().describe() if _robot_state in ROUTE_STATES else "",
            "manual_start": start_status,
            "operator_controls": operator_control.status(ble_connected.is_set()),
        }

    class LidarGUI(MonitorGUI):
        def __init__(self, root, data_queue, recv_queue):
            super().__init__(
                root, data_queue, recv_queue,
                read_sensors=read_sensors, read_health=read_health,
                front_disabled=front_disabled, back_disabled=back_disabled,
                allow_safety_disable=ALLOW_RUNTIME_SAFETY_DISABLE,
                on_checks_changed=safety_checks_changed,
                request_start=request_manual_start,
                request_emergency_stop=lambda: request_operator_action("ESTOP"),
                request_release_stop=lambda: request_operator_action("RELEASE"),
                request_continue_run=lambda: request_operator_action("CONTINUE"),
                request_reverse_run=lambda: request_operator_action("REVERSE"),
                request_ultrasonic=lambda enabled: request_operator_action("SONAR_ON" if enabled else "SONAR_OFF"),
                request_system_reset=lambda: request_operator_action("REBOOT"),
                front_threshold_cm=THRESHOLD_CM,
                front_diagonal_threshold_cm=FRONT_DIAGONAL_THRESHOLD_CM,
                back_threshold_cm=BACK_THRESHOLD_CM,
                side_threshold_cm=LidarDistanceReader.SIDE_OBSTACLE_THRESHOLD_CM,
                stale_after_s=LIDAR_STALE_TIMEOUT_S,
            )

    return LidarGUI

# ═══ Headless key listener ═══
def start_headless_key_listener():
    if not sys.stdin.isatty():
        logger.info("stdin is not a TTY — headless operator keys disabled.")
        return
    import tty, termios
    logger.info("Operator keys: f = all-front thresholds, b = all-back thresholds, s = manual disposal (confirm with y), q = quit.")

    def _listener():
        fd = sys.stdin.fileno()
        try:
            old_settings = termios.tcgetattr(fd)
        except Exception:
            return
        try:
            tty.setcbreak(fd)
            confirmation_until = None
            while not serial_stop.is_set():
                ch = sys.stdin.read(1)
                if not ch:
                    break
                ch = ch.lower()
                if confirmation_until is not None:
                    if ch == 'y' and time.monotonic() <= confirmation_until:
                        request_manual_start()
                    else:
                        logger.info("Manual disposal cancelled.")
                    confirmation_until = None
                    if ch != 'q':
                        continue
                if ch == 'f':
                    if not ALLOW_RUNTIME_SAFETY_DISABLE:
                        continue
                    if front_disabled.is_set(): front_disabled.clear(); logger.info("FRONT re-enabled.")
                    else: front_disabled.set(); logger.info("FRONT disabled.")
                    safety_checks_changed()
                elif ch == 'b':
                    if not ALLOW_RUNTIME_SAFETY_DISABLE:
                        continue
                    if back_disabled.is_set(): back_disabled.clear(); logger.info("BACK re-enabled.")
                    else: back_disabled.set(); logger.info("BACK disabled.")
                    safety_checks_changed()
                elif ch == 's':
                    logger.info("Manual disposal: are you sure? Press y within 10 seconds; any other key cancels.")
                    confirmation_until = time.monotonic() + 10.0
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
    """An explicit GARBY_LIDAR_PORT wins; otherwise find the port streaming LiDAR packets."""
    configured = os.environ.get("GARBY_LIDAR_PORT")
    if configured:
        if SERIAL_PORT and os.path.realpath(configured) == os.path.realpath(SERIAL_PORT):
            raise RuntimeError("LiDAR and sensor UART cannot use the same serial device")
        return configured if os.path.exists(configured) else None
    return serial_ports.find(LIDAR, exclude=(SERIAL_PORT,))


def find_sensor_port():
    """An explicit GARBY_SENSOR_SERIAL_PORT wins; otherwise find the port sending sensor lines."""
    if SERIAL_PORT:
        return SERIAL_PORT
    return serial_ports.find(SENSOR, exclude=(os.environ.get("GARBY_LIDAR_PORT"),))


def _terminate_process(proc):
    if proc is None:
        return
    group = getattr(proc, "garby_process_group", None)
    if proc.poll() is not None and group is None:
        return
    try:
        if group is not None:
            os.killpg(group, signal.SIGTERM)
        else:
            proc.terminate()
        proc.wait(timeout=2.0)
        if group is not None:
            # ros2 run is a wrapper; a surviving driver child can retain the
            # serial port even after its parent exits. This is our own session.
            try:
                os.killpg(group, signal.SIGKILL)
            except ProcessLookupError:
                pass
    except Exception:
        try:
            if group is not None:
                os.killpg(group, signal.SIGKILL)
            else:
                proc.kill()
            proc.wait(timeout=1.0)
        except Exception:
            pass


def start_lidar_driver(stop_event=None, ros_node=None):
    port = find_lidar_port()
    if port is None:
        raise RuntimeError("No serial port is streaming LiDAR data; check its USB cable and power")
    if stop_event and stop_event.wait(1.0):
        raise RuntimeError("LiDAR startup cancelled")
    logger.info(f"Starting LiDAR driver on {port}")
    lidar_driver_status.set("starting", "Starting the LiDAR driver")
    # Defaults match the bench-tested YDLIDAR X3 config
    # (ros2_ws/src/ydlidar_ros2_driver/config/ydlidar_x3.yaml). In particular
    # the X3 does NOT use DTR motor control; forcing it glitches the serial
    # stream and produces the "Checksum error" storm that prevents any valid
    # scan from ever being assembled/published.
    baudrate = os.environ.get("GARBY_LIDAR_BAUDRATE", "115200")
    lidar_type = os.environ.get("GARBY_LIDAR_TYPE", "1")
    sample_rate = os.environ.get("GARBY_LIDAR_SAMPLE_RATE", "3")
    # The YDLIDAR X3 uses its own motor control and must not be driven through
    # the serial adapter's DTR line.  Keep the override for other supported
    # models, but make the X3-safe value the default.
    motor_dtr = os.environ.get("GARBY_LIDAR_DTR", "false")
    frequency = os.environ.get("GARBY_LIDAR_FREQUENCY", "10.0")
    # Fixed resolution makes the SDK size every scan from a startup mean rounded
    # to tens (660 on 2026-11-12) and drop whatever a revolution sends beyond it
    # (the X3 sent 666-667: "Real points 666 > fixed points 660" on every scan).
    # Off, the driver bins every point at its own angle; the scan length varies
    # by a few points and every consumer already reads angle_increment.
    fixed_resolution = ("true" if os.environ.get("GARBY_LIDAR_FIXED_RESOLUTION", "0")
                        .strip().lower() in ("1", "true", "yes", "on") else "false")
    command = [
        "ros2", "run", "ydlidar_ros2_driver", "ydlidar_ros2_driver_node",
        "--ros-args",
        "-p", f"port:={port}",
        "-p", f"baudrate:={baudrate}",
        "-p", f"lidar_type:={lidar_type}",
        "-p", "device_type:=0",
        "-p", f"sample_rate:={sample_rate}",
        "-p", "isSingleChannel:=true",
        "-p", f"fixed_resolution:={fixed_resolution}",
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
        # Keep failures visible even when verbose SDK stdout is suppressed.
        stderr=None,
        env=os.environ.copy(),
        start_new_session=(os.name == "posix"),
    )
    if os.name == "posix":
        proc.garby_process_group = proc.pid
    try:
        for _ in range(30):
            if stop_event and stop_event.is_set():
                raise RuntimeError("LiDAR startup cancelled")
            if proc.poll() is not None:
                raise RuntimeError(f"LiDAR driver exited early with code {proc.returncode}")
            if lidar_healthy.is_set() or (ros_node is not None and ros_node.count_publishers("/scan") > 0):
                break
            if stop_event:
                stop_event.wait(1.0)
            else:
                time.sleep(1.0)
        else:
            raise RuntimeError("/scan topic publisher never appeared")
    except BaseException:
        # Ownership has not yet passed to the supervisor. Even cancellation or
        # a node exception must clean up this freshly launched child.
        _terminate_process(proc)
        raise
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
                lidar_driver_status.set("searching", "Looking for the LiDAR")
                proc = start_lidar_driver(self.stop_event, self.ros_node)
                with self._process_lock:
                    self._process = proc
                logger.info("LiDAR driver is online.")
                lidar_driver_status.set("running", "LiDAR driver running")
                started_at = time.monotonic()
                stale_since = None
                healthy_since = None

                while not self.stop_event.wait(1.0):
                    if proc.poll() is not None:
                        logger.error("LiDAR driver exited with code %s", proc.returncode)
                        lidar_driver_status.set(
                            "failed", f"LiDAR driver exited (code {proc.returncode})")
                        break
                    now = time.monotonic()
                    if lidar_streaming.is_set():
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
                                    "LiDAR produced no usable fresh scan for %.0f s; restarting driver.",
                                    self.RESTART_STALE_AFTER_S,
                                )
                                lidar_driver_status.set(
                                    "failed", "No usable scan for "
                                    f"{self.RESTART_STALE_AFTER_S:.0f} s; restarting the driver")
                                break
            except Exception as exc:
                if not self.stop_event.is_set():
                    logger.error("LiDAR driver startup failed: %s", exc)
                    lidar_driver_status.set("failed", str(exc) or type(exc).__name__)
            finally:
                lidar_healthy.clear()
                lidar_streaming.clear()
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

class ZoneLogLimiter:
    """Decide when the chassis-zone set is worth a console line.

    A zone that appears is new stop evidence and is named at once. A zone that
    clears is logged only after hold_s without another line, so an object on a
    zone boundary cannot print one line per scan. Log only: the PATH packet and
    the monitor carry every scan's zones regardless.
    """

    def __init__(self, hold_s=2.0):
        self.hold_s = float(hold_s)
        self._logged = None
        self._logged_at = 0.0

    def update(self, zones, now):
        zones = tuple(zones)
        if zones == self._logged:
            return False
        added = self._logged is None or bool(set(zones) - set(self._logged))
        if not added and now - self._logged_at < self.hold_s:
            return False
        self._logged, self._logged_at = zones, now
        return True


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
    SIDE_GROUP  = {"LEFT", "RIGHT"}
    CLEARANCE_GROUPS = {
        "CF": {"FRONT", "FRONT_LEFT", "FRONT_RIGHT"},
        "CL": {"LEFT", "FRONT_LEFT", "BACK_LEFT"},
        "CR": {"RIGHT", "FRONT_RIGHT", "BACK_RIGHT"},
    }
    # A side return this close is inside the robot's lateral collision
    # envelope.  Report it as blocked in both travel directions so a nearby
    # person, bin, or wall-mounted fire extinguisher cannot be nudged into.
    SIDE_OBSTACLE_THRESHOLD_CM = 22.0
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
        self._prev_side_blocked = False
        self._last_urgent_send = 0
        self._urgent_cooldown = 0.05
        self._last_scan_time = 0.0
        self._last_scan_stamp_ns = None
        self._last_scan_ros_clock_ns = None
        self._scan_acquisition_id = 0
        self._sector_data_valid = {side: False for side in self.SIDES}
        # Zones and wall fits of the current acquisition only (None = unknown).
        self._hallway = None
        # Moving-object prediction (motion_tracker.py). It only adds STOP/SLOW
        # evidence; its failure leaves the sector and zone limits in charge.
        self._tracker = MotionTracker(CHASSIS)
        self._motion = MotionAssessment()
        self._motion_logged = (False, False, False)
        self._last_motion_log = 0.0
        self._last_tracker_error_log = 0.0
        # Objects standing between the fitted walls (scene_objects.py). Like the
        # prediction it only adds caution speed; its failure changes nothing else.
        self._scene = SceneMonitor(CHASSIS)
        self._scene_caution = False
        self._markers = ()
        self._view_logged = (0, 0)
        self._last_view_log = 0.0
        self._last_scene_error_log = 0.0
        self._station_walls = False     # this scan had station wall returns taken out
        self._station_logged = False
        self._cloud_frame = None        # this scan, before any station wall is taken out
        self._last_path_codes = ""
        self._prev_turn_blocked = False
        self._zone_log = ZoneLogLimiter()
        self._first_scan_received = False
        self._front_data_valid = False
        self._back_data_valid = False
        self._lidar_stale_announced = False
        self._last_callback_error_log = 0.0
        self._last_map_error_log = 0.0
        self._path_seq = 0
        self._path_seq_lock = threading.Lock()
        self._history_lock = threading.RLock()
        self._watchdog_timer = self.create_timer(0.25, self._watchdog_callback)
        # One ROS subscription only. Multiple subscriptions to /scan caused the
        # same scan to be processed repeatedly and distorted confirmation logic.
        scan_qos = copy(qos_profile_sensor_data)
        scan_qos.depth = 1
        self.create_subscription(LaserScan, '/scan', self.scan_callback, scan_qos)
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
        # CPU throttling must not extend permission to drive on an old scan.
        if not self._first_scan_received or self._last_scan_time <= 0.0:
            return True
        return (time.monotonic() - self._last_scan_time) > LIDAR_STALE_TIMEOUT_S

    def _watchdog_callback(self):
        if not self._lidar_is_stale():
            self._lidar_stale_announced = False
            return
        lidar_healthy.clear()
        lidar_streaming.clear()
        if self._lidar_stale_announced:
            return
        self._lidar_stale_announced = True
        if not self._first_scan_received:
            # Nothing has been received yet, so nothing has gone stale: the driver
            # is still being found or started. The STOP below is sent either way.
            self.get_logger().info(
                "Waiting for the first LiDAR scan; holding fail-closed STOP")
        else:
            self.get_logger().error(
                f"LiDAR stream stale for > {LIDAR_STALE_TIMEOUT_S:.1f} s; issuing fail-closed STOP"
            )
        seq = self._next_path_seq()
        path_msg = f"P:{seq}|F=S|B=S"
        if not self._dispatch_urgent_path(path_msg):
            ble_send_queue.put_nowait(path_msg, urgent=True)

    def _dispatch_urgent_path(self, path_msg):
        # One mailbox owns ordering for periodic and urgent packets. A second
        # direct-write path could race a newer STOP or create unbounded futures.
        ble_send_queue.put_nowait(path_msg, urgent=True)
        return True

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

    def _nearest_clearance(self, group):
        """Nearest safety history, requiring current coverage of every sector.

        Wall medians intentionally reject isolated objects; they must never
        authorize a steering tap toward one. Missing current rays also cannot
        borrow old clear measurements from another member of the group.
        """
        if not all(self._sector_data_valid[side] for side in group):
            return None
        distances = [self._quick_min(side) for side in group]
        if any(value is None for value in distances):
            return None
        return min(distances)

    def _collect_blocked_sides(self, group, threshold_cm):
        """Return list of side names in *group* whose quick-min distance is
        within its stop limit: threshold_cm, or the shorter limit for a front
        diagonal. None represents open space with no obstacles."""
        blocked = []
        for side in group:
            d = self._quick_min(side)
            limit_cm = (FRONT_DIAGONAL_THRESHOLD_CM
                        if side in ("FRONT_LEFT", "FRONT_RIGHT") else threshold_cm)
            if d is not None and d <= limit_cm:
                blocked.append(side)
        return blocked

    def _format_blocked_status(self, blocked_sides, all_sides):
        """Return LiDAR-only path state; UART ultrasonic is trash level."""
        return format_lidar_blockage(blocked_sides, all_sides)

    def _get_blocked_status(self, group, threshold_cm):
        blocked = self._collect_blocked_sides(group, threshold_cm)
        return self._format_blocked_status(blocked, group)

    def _back_status(self):
        """Back-group distance limit; see BACK_DISTANCE_HOLDS_MOTION."""
        if not BACK_DISTANCE_HOLDS_MOTION:
            return False, "CLEAR"
        return self._get_blocked_status(self.BACK_GROUP, BACK_THRESHOLD_CM)

    def _get_side_obstacle_status(self):
        distances = {
            side: self._quick_min(side)
            for side in self.SIDE_GROUP
        }
        blocked = side_obstacle_names(
            distances, self.SIDE_OBSTACLE_THRESHOLD_CM
        )
        return self._format_blocked_status(blocked, self.SIDE_GROUP)

    def _zone(self, name):
        return bool(self._hallway is not None and getattr(self._hallway, name))

    def _front_bypassed(self):
        return ALLOW_RUNTIME_SAFETY_DISABLE and front_disabled.is_set()

    ZONE_NAMES = (("FRONT_BOX", "front_stop"), ("SLOW", "front_slow"),
                  ("LEFT_BAND", "side_left"), ("RIGHT_BAND", "side_right"),
                  ("TURN_FRONT", "turn_front"), ("TURN_BACK", "turn_back"))

    def _active_zones(self):
        """Chassis zones holding a return in the current scan, for the log and GUI."""
        zones = [name for name, zone in self.ZONE_NAMES if self._zone(zone)]
        if self._scene_caution:
            zones.append("OBJECT_AHEAD")
        if self._station_walls:
            zones.append("STATION_WALLS_IGNORED")
        if self._motion.stop:
            zones.append("MOVING_STOP")
        elif self._motion.slow:
            zones.append("MOVING_SLOW")
        if self._motion.turn_stop:
            zones.append("MOVING_TURN")
        return zones

    def _front_status(self):
        """Sector limits, plus the chassis-width front box they can miss, plus a
        moving object predicted to reach the footprint within STOP_TTC_S."""
        blocked, status = self._get_blocked_status(self.FRONT_GROUP, THRESHOLD_CM)
        if not blocked and self._zone("front_stop"):
            return True, "BLOCKED{FRONT_BOX}"
        if not blocked and self._motion.stop:
            # Generic obstacle code: the LiDAR cannot tell a person from a cart.
            return True, "BLOCKED{MOVING}"
        return blocked, status

    def _side_status(self):
        """Radial side limit, plus a return within side_margin of the body."""
        blocked, status = self._get_side_obstacle_status()
        if not blocked:
            names = [name for name, zone in (("LEFT_BAND", "side_left"),
                                             ("RIGHT_BAND", "side_right"))
                     if self._zone(zone)]
            if names:
                return True, "BLOCKED{" + ",".join(names) + "}"
        return blocked, status

    def _turn_blocked(self, side_blocked):
        """Spin-in-place sweep. Unknown evidence is never a clear sweep."""
        if (self._hallway is None or not self._front_data_valid
                or not self._back_data_valid):
            return None
        bypass_allowed = ALLOW_RUNTIME_SAFETY_DISABLE
        front_hit = self._zone("turn_front") and not (bypass_allowed and front_disabled.is_set())
        back_hit = self._zone("turn_back") and not (bypass_allowed and back_disabled.is_set())
        # Something moving toward the circle the body sweeps: do not start spinning.
        moving_hit = self._motion.turn_stop and not self._front_bypassed()
        return bool(side_blocked or front_hit or back_hit or moving_hit)

    def _update_motion_prediction(self, ranges, angle_min, angle_inc, range_min, range_max,
                                  frame=None):
        """Track moving objects in this acquisition (acquired at _last_scan_time)."""
        try:
            # The wall fit of this same scan gives the robot's heading in the
            # hallway; its change between scans is the robot's own rotation.
            heading = self._hallway.heading_deg if self._hallway is not None else None
            segments = build_segments(
                frame, (CHASSIS.front_cm, CHASSIS.rear_cm, CHASSIS.half_width_cm))
            self._motion = self._tracker.update(
                ranges, angle_min=angle_min, angle_increment=angle_inc,
                range_min=range_min, range_max=range_max,
                scan_time=self._last_scan_time, ego=ego_motion.snapshot(),
                heading_deg=heading, segments=segments)
        except Exception:
            # Prediction only ever adds STOP/SLOW. A fault here must not hold the
            # robot through the sector limits' scan; it starts again next scan.
            self._tracker.reset()
            self._motion = MotionAssessment()
            now = time.monotonic()
            if now - self._last_tracker_error_log >= 30.0:
                self._last_tracker_error_log = now
                logger.exception("Moving-object prediction failed; sector and zone limits remain")
            return
        state = (self._motion.stop, self._motion.slow, self._motion.turn_stop)
        if state != self._motion_logged:
            now = time.monotonic()
            # A new hold is logged at once; a release or a slow-down at most every 2 s.
            if (self._motion.stop and not self._motion_logged[0]) or now - self._last_motion_log >= 2.0:
                self._motion_logged = state
                self._last_motion_log = now
                if self._motion.stop or self._motion.slow or self._motion.turn_stop:
                    action = ("STOP" if self._motion.stop else
                              "turn held" if self._motion.turn_stop else "caution speed")
                    logger.info("Moving-object prediction: %s (%s; robot %s)", action,
                                self._motion.reason or "moving object near", self._motion.ego)
                else:
                    logger.info("Moving-object prediction: clear")

    def cloud_view(self):
        """(likely people, objects between the walls, active zones) for the phone."""
        with self._history_lock:
            if self._lidar_is_stale():
                return 0, 0, ()
            return (self._motion.person_count, len(self._scene.scene.standing()),
                    tuple(self._active_zones()))

    def cloud_map_frame(self, seq):
        """One frame for the phone map, or None when the scan is not current."""
        with self._history_lock:
            return cloud_control.encode_map_frame(
                self._cloud_frame, seq=seq, scan_age_s=time.monotonic() - self._last_scan_time,
                hallway=self._hallway, markers=self._markers, zones=self._active_zones(),
                path=self._last_path_codes, hold=_motion_hold)

    def _apply_station_mode(self, ranges, frame):
        """On a station step, take the station's own wall returns out of the scan.

        Every later consumer (sector limits, chassis zones, prediction, scene)
        sees those rays as "nothing within range". The map keeps drawing them.
        """
        active = (station_mode.active() and ble_connected.is_set()
                  and _robot_state in ROUTE_STATES)
        taken = 0
        if active:
            try:
                ranges, frame, taken = without_station_walls(ranges, frame, CHASSIS,
                                                             STATION_WALL_MIN_GAP_CM)
            except Exception:
                # A fault here leaves the ordinary, stricter rules in charge.
                active = False
                now = time.monotonic()
                if now - self._last_scene_error_log >= 30.0:
                    self._last_scene_error_log = now
                    logger.exception("Station wall detection failed; ordinary LiDAR limits apply")
        self._station_walls = taken > 0
        if active != self._station_logged:
            self._station_logged = active
            logger.info("Station step: %s", "the station's walls are left out of the LiDAR holds"
                        if active else "ended; ordinary LiDAR limits apply")
        return ranges, frame

    def _learn_wall_heading_offset(self):
        """Feed the mounting-angle learner; a fault there only stops the learning."""
        hallway = self._hallway
        try:
            wall_heading_offset.observe(
                self._last_scan_time,
                hallway.left.distance_cm if hallway and hallway.left else None,
                hallway.right.distance_cm if hallway and hallway.right else None,
                hallway.heading_deg if hallway else None, ego_motion.snapshot())
        except Exception:
            wall_heading_offset.enabled = False
            logger.exception("Wall heading offset learning stopped; H is sent uncorrected")

    def _update_scene(self, frame):
        """Objects standing between the walls of this acquisition, and map markers."""
        try:
            scene = analyze_scene(frame, self._hallway, CHASSIS)
            self._scene_caution = self._scene.update(scene, self._last_scan_time)
            self._markers = scene_markers(scene, self._motion)
        except Exception:
            self._scene.reset()
            self._scene_caution = False
            self._markers = ()
            now = time.monotonic()
            if now - self._last_scene_error_log >= 30.0:
                self._last_scene_error_log = now
                logger.exception("Hallway object detection failed; sector and zone limits remain")
            return
        people = self._motion.person_count
        objects = len(scene.standing())
        view = (people, objects)
        now = time.monotonic()
        if view != self._view_logged and (people > self._view_logged[0]
                                          or now - self._last_view_log >= 2.0):
            self._view_logged = view
            self._last_view_log = now
            if people or objects:
                nearest = scene.standing()[0] if objects else None
                logger.info(
                    "Hallway view: %d likely person(s), %d object(s) between the walls%s%s",
                    people, objects,
                    f"; nearest object {nearest.along_cm / 100.0:.1f} m ahead, "
                    f"{nearest.width_cm:.0f} cm wide" if nearest is not None else "",
                    f"; caution speed ({self._scene.reason})" if self._scene_caution else "")
            else:
                logger.info("Hallway view: clear")

    @staticmethod
    def _status_code(status):
        return lidar_status_code(status)

    def _send_immediate_status(self):
        # Use the same complete snapshot as periodic status. An obstacle in
        # one direction must never turn missing data in the other into CLEAR.
        path_msg, _, path_deadline = self.build_status_packets_with_deadline()
        ble_send_queue.put_nowait(path_msg, urgent=True, expires_at=path_deadline)

    def _invalidate_scan_locked(self):
        """Discard unusable acquisition evidence and dispatch its STOP now."""
        self._last_scan_time = 0.0
        self._front_data_valid = self._back_data_valid = False
        self._sector_data_valid = {side: False for side in self.SIDES}
        self._hallway = None
        self._tracker.reset()
        self._motion = MotionAssessment()
        self._scene.reset()
        self._scene_caution = False
        self._markers = ()
        self._cloud_frame = None
        for side in self.SIDES:
            self.history[side].clear()
            self.steering_history[side].clear()
            self._pending_jump[side] = [None, 0]
        self._watchdog_callback()

    def scan_callback(self, msg: LaserScan):
        display_frame = None
        with self._history_lock:
            try:
                display_frame = self._scan_callback_locked(msg)
            except Exception:
                # An exception leaving this callback ends executor.spin(), which
                # silently kills both this subscription and the stale watchdog.
                # Mark the data unusable so status packets report S.
                self._invalidate_scan_locked()
                now = time.monotonic()
                if now - self._last_callback_error_log >= 5.0:
                    self._last_callback_error_log = now
                    logger.exception("LiDAR scan processing failed; holding STOP")

        # Optional drawing geometry is produced after safety/urgent dispatch,
        # outside the history lock. Headless mode never traverses the scan for
        # a map. A display error cannot invalidate or delay a path decision.
        if display_frame is not None:
            frame, acquired_at = display_frame
            scan_snapshot = None
            try:
                scan_snapshot = build_lidar_map_snapshot(
                    msg.ranges, angle_min=msg.angle_min,
                    angle_increment=msg.angle_increment,
                    range_min=msg.range_min, range_max=msg.range_max,
                    acquired_at=acquired_at, yaw_offset_deg=LIDAR_YAW_OFFSET_DEG,
                    reflect_left_right=LIDAR_REFLECT_LEFT_RIGHT, markers=self._markers,
                )
            except Exception:
                now = time.monotonic()
                if now - self._last_map_error_log >= 5.0:
                    self._last_map_error_log = now
                    logger.exception("LiDAR display geometry unavailable")
            if self.data_queue.full():
                try: self.data_queue.get_nowait()
                except queue.Empty: pass
            try:
                # Summary freshness must retain source age even when optional
                # drawing geometry is unavailable.
                self.data_queue.put_nowait((*frame, scan_snapshot, acquired_at))
            except queue.Full:
                pass

    def _scan_acquisition_timing(self, msg):
        """Return acquisition age and advancing source/ROS timestamps."""
        try:
            stamp = msg.header.stamp
            if (stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000):
                return None
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            if stamp_ns <= 0:
                return None
            ros_clock_ns = self.get_clock().now().nanoseconds
            # Receipt of the same acquisition is not a new observation. Keep
            # high-water marks even after rejecting a scan, so alternating old
            # stamps cannot reset the freshness budget under a paused clock.
            if ((self._last_scan_stamp_ns is not None
                 and stamp_ns <= self._last_scan_stamp_ns)
                    or (self._last_scan_ros_clock_ns is not None
                        and ros_clock_ns < self._last_scan_ros_clock_ns)):
                return None
            age = (ros_clock_ns - stamp_ns) / 1_000_000_000.0
        except (AttributeError, TypeError, ValueError, OverflowError):
            return None
        # A small negative age tolerates clock rounding; a future scan cannot
        # grant a longer safety budget after a clock jump or clock mismatch.
        if not math.isfinite(age) or age < -0.1:
            return None
        return max(0.0, age), stamp_ns, ros_clock_ns

    def _scan_callback_locked(self, msg: LaserScan):
        now = time.monotonic()
        scan_timing = self._scan_acquisition_timing(msg)
        scan_age = scan_timing[0] if scan_timing is not None else None
        if scan_age is None or scan_age > LIDAR_STALE_TIMEOUT_S:
            # A new callback can contain old data after CPU/DDS scheduling
            # delays. It must not refresh health, steering, or path permission.
            self._invalidate_scan_locked()
            return
        self._last_scan_stamp_ns = scan_timing[1]
        self._last_scan_ros_clock_ns = scan_timing[2]
        # This identifier belongs to the source acquisition, not a periodic
        # BLE poll. Repeated snapshots retain it, so one scan cannot supply
        # several independent centering confirmations at the bridge.
        self._scan_acquisition_id = (self._scan_acquisition_id + 1) & 0xFFFFFFFF
        if self._first_scan_received and now - self._last_scan_time > LIDAR_STALE_TIMEOUT_S:
            # Do not blend measurements from before a stream outage into the
            # first fresh scan, especially when restoring wall centering.
            for side in self.SIDES:
                self.history[side].clear()
                self.steering_history[side].clear()
                self._pending_jump[side] = [None, 0]
        # Convert the remaining acquisition-age budget to monotonic time so
        # later clock adjustments cannot extend permission on this scan.
        self._last_scan_time = now - scan_age
        first_scan = not self._first_scan_received
        self._first_scan_received = True
        self._lidar_stale_announced = False
        if first_scan:
            logger.info("First LiDAR scan received with %d range samples.", len(msg.ranges))

        # One pass creates two geometries: gap-free nearest 45° sectors for
        # collision safety, and centered 22° slices for clean wall steering.
        # There are no eight-way comparisons and no safety blind angles.
        ranges = msg.ranges
        angle_min, angle_inc = robot_scan_angles(
            msg.angle_min, msg.angle_increment,
            yaw_offset_deg=LIDAR_YAW_OFFSET_DEG,
            reflect_left_right=LIDAR_REFLECT_LEFT_RIGHT,
        )
        # Preserve invalid metadata for the core validator. max/min/defaults
        # previously turned NaN, zero, and infinity into valid-looking limits.
        minimum_range = float(msg.range_min)
        maximum_range = float(msg.range_max)
        # The scan is converted to robot-frame points once for the zones and
        # walls, the moving-object tracker and the hallway-object scene.
        frame = build_scan_frame(
            ranges, angle_min=angle_min, angle_increment=angle_inc,
            range_min=minimum_range, range_max=maximum_range, lidar_y_cm=CHASSIS.lidar_y_cm)
        self._cloud_frame = frame   # the phone map draws every return, like the monitor's
        # On a station step the station's own walls are taken out first, so
        # every consumer below sees the same scan.
        ranges, frame = self._apply_station_mode(ranges, frame)
        safety_dists, steering_dists, valid_points = partition_lidar_samples(
            ranges,
            angle_min=angle_min,
            angle_increment=angle_inc,
            range_min=minimum_range,
            range_max=maximum_range,
            steering_cone_deg=self.CONE_DEG,
        )
        self._sector_data_valid = {
            side: len(safety_dists[side]) >= self.MIN_VALID_POINTS
            for side in self.SIDES
        }
        self._hallway = analyze_hallway(
            ranges, angle_min=angle_min, angle_increment=angle_inc,
            range_min=minimum_range, range_max=maximum_range, geometry=CHASSIS, frame=frame,
            far_fit=WALL_FAR_FIT,
        )
        self._update_motion_prediction(ranges, angle_min, angle_inc, minimum_range,
                                       maximum_range, frame)
        self._update_scene(frame)
        self._learn_wall_heading_offset()
        # Missing coverage is a directional safety constraint, not a lost
        # connection. Show partial measurements without restarting the driver.
        if valid_points >= self.MIN_VALID_POINTS:
            lidar_streaming.set()
        else:
            lidar_streaming.clear()

        front_data_valid = all(
            len(safety_dists[side]) >= self.MIN_VALID_POINTS
            for side in self.FRONT_GROUP
        )
        back_data_valid = all(
            len(safety_dists[side]) >= self.MIN_VALID_POINTS
            for side in self.BACK_GROUP
        )
        side_data_valid = all(
            len(safety_dists[side]) >= self.MIN_VALID_POINTS
            for side in self.SIDE_GROUP
        )
        front_data_valid = front_data_valid and side_data_valid
        back_data_valid = back_data_valid and side_data_valid
        quality_degraded = (
            (self._front_data_valid and not front_data_valid)
            or (self._back_data_valid and not back_data_valid)
        )
        self._front_data_valid = front_data_valid
        self._back_data_valid = back_data_valid
        if front_data_valid and back_data_valid:
            lidar_healthy.set()
        else:
            lidar_healthy.clear()

        send_urgent_status = False
        with self._history_lock:
            for side in self.SIDES:
                dists = safety_dists[side]
                if len(dists) >= self.MIN_VALID_POINTS:
                    # Keep independent estimates: nearest return for collision
                    # safety and median wall geometry for smooth centering/tilt.
                    # Temporal confirmation handles non-emergency jump noise.
                    new_cm = robust_near_distance_cm(dists)
                    narrow_wall_dists = steering_dists[side]
                    # Centering needs an actual wall near +/-90 degrees.
                    # A sparse cone must not borrow doorway/diagonal rays
                    # from the wider safety sector to invent that wall.
                    if side in self.SIDE_GROUP:
                        wall_source = (narrow_wall_dists if len(narrow_wall_dists)
                                       >= self.MIN_VALID_POINTS else [])
                    else:
                        wall_source = select_steering_samples(
                            narrow_wall_dists, dists,
                            minimum_points=self.MIN_VALID_POINTS,
                        )
                    wall_cm = robust_wall_distance_cm(wall_source)
                    if wall_cm is None:
                        self.steering_history[side].clear()
                    else:
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
                    # Missing current clearance must not be refreshed over BLE
                    # by republishing an older valid median.
                    self.steering_history[side].clear()
                    self._pending_jump[side] = [None, 0]

            if self.data_queue is not None:
                data_out = {}
                for side in self.SIDES:
                    # A new frame must not give an old range the acquisition
                    # time of a scan that has no usable current coverage.
                    if not self._sector_data_valid[side]:
                        data_out[side] = None
                        continue
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

            front_blocked, _ = self._front_status()
            back_blocked, _ = self._back_status()
            side_blocked, _ = self._side_status()
            turn_blocked = bool(self._turn_blocked(side_blocked))
            # Side obstacles are direction-independent: the robot must not
            # resume in reverse while the same lateral object is still close.
            reported_front_blocked = front_blocked or side_blocked
            reported_back_blocked = back_blocked or side_blocked
            bypass_allowed = ALLOW_RUNTIME_SAFETY_DISABLE
            if bypass_allowed and not side_blocked:
                if front_disabled.is_set():
                    reported_front_blocked = False
                if back_disabled.is_set():
                    reported_back_blocked = False

            now = time.monotonic()
            changed_to_blocked = (
                (reported_front_blocked or reported_back_blocked)
                and (
                    reported_front_blocked != self._prev_front_blocked
                    or reported_back_blocked != self._prev_back_blocked
                    or side_blocked != self._prev_side_blocked
                )
            ) or (turn_blocked and not self._prev_turn_blocked)
            if (
                quality_degraded
                or (changed_to_blocked
                    and now - self._last_urgent_send > self._urgent_cooldown)
            ):
                send_urgent_status = True
                self._last_urgent_send = now

            self._prev_front_blocked = reported_front_blocked
            self._prev_back_blocked = reported_back_blocked
            self._prev_side_blocked = side_blocked
            self._prev_turn_blocked = turn_blocked
            # The eight sector readings cannot show a chassis-zone block, so a
            # change is logged: otherwise a held robot has no visible reason.
            zones = self._active_zones()
            # Moving-object zones show in the GUI footer; the prediction logs its
            # own line with the reason, so the zone log keeps to the chassis zones.
            chassis_zones = [zone for zone in zones
                             if not zone.startswith(("MOVING_", "STATION_"))]
            if self._zone_log.update(chassis_zones, now):
                logger.info("LiDAR chassis zones: %s",
                            ", ".join(chassis_zones) if chassis_zones else "clear")

        if send_urgent_status:
            self._send_immediate_status()

        if self.data_queue is not None:
            return (
                (data_out, msg.range_min, msg.range_max, self.CONE_DEG,
                 self.last_sides_status + ("|Z=" + ",".join(zones) if zones else "")),
                self._last_scan_time,
            )

    def build_status_packets(self):
        with self._history_lock:
            return self._build_status_packets_locked()

    def build_status_packets_with_deadline(self):
        """Return one path snapshot plus its LiDAR evidence deadline."""
        with self._history_lock:
            deadline = (None if self._lidar_is_stale()
                        else self._last_scan_time + LIDAR_STALE_TIMEOUT_S)
            path_msg, sides_msg = self._build_status_packets_locked()
            return path_msg, sides_msg, deadline

    def _build_status_packets_locked(self):
        seq = self._next_path_seq()
        if self._lidar_is_stale():
            self.last_sides_status = "SIDES:STABLE"
            return f"P:{seq}|F=S|B=S", f"S:{seq}|STABLE"

        with self._history_lock:
            front_blocked, front_status = self._front_status()
            back_blocked, back_status = self._back_status()
            side_blocked, side_status = self._side_status()

            # Fresh-but-incomplete scans are unknown, never clear. Manual GUI
            # bypasses are intentionally unable to override unavailable data.
            if not self._front_data_valid:
                front_blocked, front_status = True, "LIDAR_UNAVAILABLE"
            if not self._back_data_valid:
                back_blocked, back_status = True, "LIDAR_UNAVAILABLE"

            bypass_allowed = ALLOW_RUNTIME_SAFETY_DISABLE
            if self._front_data_valid and bypass_allowed and front_disabled.is_set():
                front_blocked, front_status = False, "CLEAR"
            if self._back_data_valid and bypass_allowed and back_disabled.is_set():
                back_blocked, back_status = False, "CLEAR"

            if side_blocked:
                # This override is intentionally after the manual front/back
                # bypass checks: side collision protection is never disabled
                # by a directional GUI toggle.
                front_blocked, front_status = True, side_status
                back_blocked, back_status = True, side_status

            turn_blocked = self._turn_blocked(side_blocked)
            turn_code = "S" if turn_blocked is None else ("O" if turn_blocked else "C")
            front_code = self._status_code(front_status)
            # W = clear to the 95 cm stop but something in the swept corridor
            # within front_slow_cm, a closing mover, or (2026-11-14) an object
            # standing in the path between the walls within 4 m: drive at caution
            # speed. A bypassed front group bypasses all of these as well.
            if (front_code == "C" and (self._zone("front_slow") or self._motion.slow
                                       or self._scene_caution)
                    and not (bypass_allowed and front_disabled.is_set())):
                front_code = "W"

            med = {
                side: self._steering_smoothed_distance(side)
                for side in self.SIDES
            }
            clearance = {
                key: self._nearest_clearance(group)
                for key, group in self.CLEARANCE_GROUPS.items()
            }
            hallway = self._hallway
            scan_age_ms = max(0, round((time.monotonic() - self._last_scan_time) * 1000.0))

        path_msg = (f"P:{seq}|F={front_code}"
                    f"|B={self._status_code(back_status)}|T={turn_code}")
        self._last_path_codes = path_msg.split("|", 1)[1]

        key_map = {
            "LEFT": "L", "RIGHT": "R", "FRONT": "F", "BACK": "B",
            "FRONT_LEFT": "FL", "FRONT_RIGHT": "FR",
            "BACK_LEFT": "BL", "BACK_RIGHT": "BR",
        }
        sides_parts = [
            f"{key_map[side]}={value:.1f}"
            for side, value in med.items() if value is not None
        ]
        sides_parts.extend(
            f"{key}={value:.1f}"
            for key, value in clearance.items() if value is not None
        )
        sides_parts.append(f"A={self._scan_acquisition_id}")
        # Wall line fits of this acquisition for hallway centering: perpendicular
        # distance from the chassis centerline (cm), the robot's heading relative
        # to the walls (deg, + = rotated left) and the acquisition age at this
        # snapshot (ms). Missing walls are omitted, never filled from history.
        if hallway is not None:
            if hallway.left is not None:
                sides_parts.append(f"WL={hallway.left.distance_cm:.1f}")
            if hallway.right is not None:
                sides_parts.append(f"WR={hallway.right.distance_cm:.1f}")
            if hallway.heading_deg is not None:
                # Less the LiDAR mounting error learned from drift (zero until learned).
                sides_parts.append(f"H={wall_heading_offset.correct(hallway.heading_deg):.2f}")
        sides_parts.append(f"G={scan_age_ms}")

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
            if len(sides_msg.encode("ascii")) <= BLE_STEERING_MAX_BYTES:
                self.last_sides_status = "SIDES:" + sides_cmd
            else:
                # A pathological sensor range/metadata must not create a
                # fragmented or truncated steering command. PATH is preserved.
                sides_msg = f"S:{seq}|STABLE"
                self.last_sides_status = "SIDES:STABLE"
        else:
            sides_msg = f"S:{seq}|STABLE"
            self.last_sides_status = "SIDES:STABLE"
        return path_msg, sides_msg

    def build_combined_cmd(self) -> str:
        """Compatibility helper; new transport uses build_status_packets()."""
        path_msg, sides_msg = self.build_status_packets()
        return f"{path_msg}||{sides_msg}"



# ═══ Main ═══
def enable_file_log(directory):
    """A floor run used to leave no log unless the console was captured."""
    from logging.handlers import TimedRotatingFileHandler
    try:
        directory = os.path.expanduser(directory)
        os.makedirs(directory, exist_ok=True)
        handler = TimedRotatingFileHandler(os.path.join(directory, "bridge.log"),
                                           when="midnight", backupCount=14, encoding="utf-8")
        handler.setFormatter(logging.Formatter(
            "%(asctime)s [%(levelname)s] %(name)s: %(message)s", datefmt="%Y-%m-%d %H:%M:%S"))
        logging.getLogger().addHandler(handler)
        logger.info("Logging to %s (daily files, 14 days kept)", handler.baseFilename)
    except OSError as exc:
        logger.warning("File log unavailable (%s); console only", exc)


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Combined Sensor + LiDAR App")
    parser.add_argument("--headless", action="store_true", help="Run without GUI")
    parser.add_argument("--log-dir", default=os.environ.get("GARBY_LOG_DIR", ""),
                        help="Also write the log to a daily file here (kept 14 days)")
    args = parser.parse_args()
    if args.log_dir:
        enable_file_log(args.log_dir)
    # Acquire before starting workers or touching ROS/serial resources.
    with SingleInstanceGuard():
        _run_application(args)


def _run_application(args):
    global firebase, ser, ble_service, _lidar_node_ref
    gui_root = create_gui_root(args.headless)
    headless = gui_root is None
    _launcher_info.update(mode="headless" if headless else "monitor",
                          startedAt=int(time.time() * 1000))
    logger.info("GARBY Pi software %s starting (%s)", PI_SOFTWARE_ID, _launcher_info["mode"])
    serial_stop.clear()
    data_queue = None if headless else queue.Queue(maxsize=5)
    gui = None
    if gui_root is not None:
        gui = init_gui()(gui_root, data_queue, ble_recv_queue)
        # Map the complete monitor before initializing peripherals. A missing
        # sensor board or slow ROS discovery must not leave a blank window.
        gui_root.update_idletasks()

    def ble_cb_status(text, color):
        if gui is not None:
            gui.set_ble_status(text, color)

    ble = BleService(ble_cb_status, ble_notify_handler)
    ble_service = ble
    ble.start()

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

    notice_thread = threading.Thread(
        target=lambda: notice_health_loop(serial_stop),
        daemon=True,
        name="sms-health-monitor",
    )
    notice_thread.start()

    reset_thread = threading.Thread(
        target=lambda: firebase_reset_command_loop(serial_stop),
        daemon=True,
        name="firebase-reset",
    )
    reset_thread.start()

    threading.Thread(target=lambda: firebase_disposal_command_loop(serial_stop),
                     daemon=True, name="firebase-disposal").start()

    rclpy.init()
    node = LidarDistanceReader(None, data_queue, ble_service)
    _lidar_node_ref = node
    threading.Thread(target=lambda: firebase_control_loop(serial_stop),
                     daemon=True, name="firebase-phone-control").start()
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
        wall_heading_offset.save(force=True)

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

    if headless:
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
        root = gui_root
        root.protocol("WM_DELETE_WINDOW", shutdown)
        try:
            root.mainloop()
        except KeyboardInterrupt:
            shutdown()

if __name__ == '__main__':
    main()
