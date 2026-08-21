"""Hardware-independent reliability primitives for GARBY's Raspberry Pi bridge.

This module intentionally has no BLE, ROS 2, serial, GUI, or Firebase imports.
Keeping these helpers hardware-independent makes the safety-critical data
handling testable on any host before deployment to the Raspberry Pi.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Callable, Iterable, Mapping, MutableMapping, Sequence


@dataclass(frozen=True)
class SensorSpec:
    sentinel: float
    minimum: float
    maximum: float
    integer: bool = False

    def normalize(self, value: object) -> float | int | None:
        """Return a bounded normalized value, or ``None`` when unavailable.

        The transport sentinel is treated as unavailable rather than as a real
        measurement. Booleans are rejected so ``True`` cannot silently become 1.
        """
        if isinstance(value, bool):
            return None
        try:
            numeric = float(value)
        except (TypeError, ValueError, OverflowError):
            return None
        if not math.isfinite(numeric):
            return None
        if math.isclose(numeric, float(self.sentinel), rel_tol=0.0, abs_tol=1e-9):
            return None
        if numeric < self.minimum or numeric > self.maximum:
            return None
        if self.integer:
            return int(round(numeric))
        return round(numeric, 2)


# Transport sentinels are intentionally outside each sensor's live range.
SENSOR_SPECS: dict[str, SensorSpec] = {
    "ultrasonic": SensorSpec(sentinel=999.0, minimum=0.0, maximum=600.0, integer=False),
    "mq4": SensorSpec(sentinel=-1.0, minimum=0.0, maximum=100000.0, integer=True),
    "mq137": SensorSpec(sentinel=-1.0, minimum=0.0, maximum=100000.0, integer=True),
    "mq135": SensorSpec(sentinel=-1.0, minimum=0.0, maximum=100000.0, integer=True),
}


class SensorTransitionTracker:
    """Coalesce live sensor values and emit one unavailable transition.

    The tracker prevents an unavailable sensor from repeatedly overwriting a
    still-useful database value with sentinels. It emits exactly one sentinel
    when a sensor becomes unavailable and re-arms that transition after the
    sensor recovers.
    """

    def __init__(
        self,
        specs: Mapping[str, SensorSpec] = SENSOR_SPECS,
        *,
        stale_after_s: float = 8.0,
        live_publish_period_s: float = 5.0,
    ) -> None:
        self.specs = dict(specs)
        self.stale_after_s = max(0.1, float(stale_after_s))
        self.live_publish_period_s = max(0.05, float(live_publish_period_s))
        self._lock = threading.RLock()
        self._latest: dict[str, float | int | None] = {name: None for name in self.specs}
        self._last_seen: dict[str, float | None] = {name: None for name in self.specs}
        self._last_live_publish: dict[str, float | None] = {name: None for name in self.specs}
        # Initial state is explicitly unavailable and should be published once.
        self._transition_pending: set[str] = set(self.specs)
        self._offline_announced: set[str] = set()

    def _set_unavailable_locked(self, name: str) -> None:
        was_live = self._latest[name] is not None
        self._latest[name] = None
        self._last_seen[name] = None
        if was_live or name not in self._offline_announced:
            self._transition_pending.add(name)

    def ingest(self, name: str, raw_value: object, now: float | None = None) -> bool:
        """Ingest one reading. Return ``True`` only for a valid live reading."""
        if name not in self.specs:
            return False
        if now is None:
            now = time.monotonic()
        normalized = self.specs[name].normalize(raw_value)
        with self._lock:
            if normalized is None:
                self._set_unavailable_locked(name)
                return False

            was_unavailable = self._latest[name] is None
            self._latest[name] = normalized
            self._last_seen[name] = float(now)
            if was_unavailable:
                self._transition_pending.add(name)
            self._offline_announced.discard(name)
            return True

    def mark_unavailable(self, names: Iterable[str] | None = None) -> None:
        with self._lock:
            targets = self.specs.keys() if names is None else names
            for name in targets:
                if name in self.specs:
                    self._set_unavailable_locked(name)

    def _expire_stale_locked(self, now: float) -> None:
        for name, last_seen in list(self._last_seen.items()):
            if last_seen is None or self._latest[name] is None:
                continue
            if now - last_seen > self.stale_after_s:
                self._set_unavailable_locked(name)

    def collect_due(self, now: float | None = None) -> dict[str, float | int]:
        """Return values that should be published now.

        A transition (offline->live or live->offline) is immediate. Stable live
        values are refreshed at a bounded rate; stable offline values stay quiet.
        """
        if now is None:
            now = time.monotonic()
        now = float(now)
        due: dict[str, float | int] = {}
        with self._lock:
            self._expire_stale_locked(now)
            for name, spec in self.specs.items():
                value = self._latest[name]
                transition = name in self._transition_pending
                if value is None:
                    if transition:
                        due[name] = int(spec.sentinel) if spec.integer else spec.sentinel
                        self._transition_pending.discard(name)
                        self._offline_announced.add(name)
                    continue

                last_publish = self._last_live_publish[name]
                if transition or last_publish is None or now - last_publish >= self.live_publish_period_s:
                    due[name] = value
                    self._last_live_publish[name] = now
                    self._transition_pending.discard(name)
                    self._offline_announced.discard(name)
        return due

    def latest_for_transport(self, name: str | None = None, now: float | None = None):
        if now is None:
            now = time.monotonic()
        with self._lock:
            self._expire_stale_locked(float(now))
            if name is not None:
                return self._latest.get(name)
            return dict(self._latest)

    def is_link_fresh(self, now: float | None = None) -> bool:
        """Return true when at least one sensor has a recent valid sample."""
        if now is None:
            now = time.monotonic()
        now = float(now)
        with self._lock:
            self._expire_stale_locked(now)
            return any(value is not None for value in self._latest.values())


class ExponentialBackoff:
    def __init__(
        self,
        initial_s: float = 1.0,
        maximum_s: float = 30.0,
        factor: float = 2.0,
        jitter: Callable[[float], float] | None = None,
    ) -> None:
        # Backward compatibility: an older caller may pass jitter as arg #3.
        if callable(factor) and jitter is None:
            jitter = factor  # type: ignore[assignment]
            factor = 2.0
        self.initial_s = max(0.0, float(initial_s))
        self.maximum_s = max(self.initial_s, float(maximum_s))
        self.factor = max(1.0, float(factor))
        self.jitter = jitter
        self._next = self.initial_s

    def next_delay(self) -> float:
        base = self._next
        self._next = min(self.maximum_s, max(self.initial_s, self._next * self.factor))
        if self.jitter is None:
            return base
        try:
            return max(0.0, float(self.jitter(base)))
        except Exception:
            return base

    def reset(self) -> None:
        self._next = self.initial_s


class CoalescingUpdateWorker:
    """Background latest-value writer with bounded retry pressure.

    ``enqueue`` merges paths atomically. The worker starts automatically because
    the production FirebaseManager constructs it once and immediately enqueues.
    New samples coalesce while a retry is pending but cannot bypass backoff.
    """

    def __init__(
        self,
        update_func: Callable[[dict[str, object]], None],
        *,
        batch_window_s: float = 0.05,
        retry_backoff: ExponentialBackoff | None = None,
        name: str = "garby-update-worker",
    ) -> None:
        self._update_func = update_func
        self._batch_window_s = max(0.0, float(batch_window_s))
        self._backoff = retry_backoff or ExponentialBackoff(0.5, 10.0, 2.0)
        self._name = name
        self._cv = threading.Condition()
        self._pending: dict[str, object] = {}
        self._closing = False
        self._flush_on_close = True
        self._urgent = False
        self._thread = threading.Thread(target=self._run, name=self._name, daemon=True)
        self._thread.start()

    def start(self) -> None:
        # Kept for compatibility with test harnesses/older callers.
        return

    def enqueue(self, values: Mapping[str, object], *, urgent: bool = False) -> None:
        if not values:
            return
        with self._cv:
            if self._closing:
                return
            self._pending.update(values)
            self._urgent = self._urgent or bool(urgent)
            self._cv.notify_all()

    def submit(self, values: Mapping[str, object]) -> None:
        self.enqueue(values)

    def close(self, *, flush: bool = True, timeout: float = 2.0) -> None:
        with self._cv:
            self._closing = True
            self._flush_on_close = bool(flush)
            if not flush:
                self._pending.clear()
            self._cv.notify_all()
        if self._thread.is_alive():
            self._thread.join(timeout=max(0.0, float(timeout)))

    def stop(self, timeout: float = 2.0) -> None:
        self.close(flush=False, timeout=timeout)

    def _wait_until(self, deadline: float) -> bool:
        with self._cv:
            while True:
                now = time.monotonic()
                if now >= deadline:
                    return True
                if self._closing and not self._flush_on_close:
                    return False
                self._cv.wait(timeout=deadline - now)

    def _run(self) -> None:
        retry_not_before = 0.0
        while True:
            with self._cv:
                while not self._pending and not self._closing:
                    self._cv.wait()
                if self._closing and (not self._flush_on_close or not self._pending):
                    return
                urgent = self._urgent
                self._urgent = False

            now = time.monotonic()
            if now < retry_not_before and not self._wait_until(retry_not_before):
                return

            if self._batch_window_s and not urgent:
                if not self._wait_until(time.monotonic() + self._batch_window_s):
                    return

            with self._cv:
                if not self._pending:
                    if self._closing:
                        return
                    continue
                batch = dict(self._pending)
                self._pending.clear()

            try:
                self._update_func(batch)
            except Exception:
                with self._cv:
                    # A newer value already pending wins over the failed old one.
                    for key, value in batch.items():
                        self._pending.setdefault(key, value)
                    if self._closing and not self._flush_on_close:
                        return
                retry_not_before = time.monotonic() + self._backoff.next_delay()
                continue

            self._backoff.reset()
            retry_not_before = 0.0


LIDAR_SECTOR_NAMES: tuple[str, ...] = (
    "FRONT",
    "FRONT_LEFT",
    "LEFT",
    "BACK_LEFT",
    "BACK",
    "BACK_RIGHT",
    "RIGHT",
    "FRONT_RIGHT",
)


def _normalize_angle_rad(angle: float) -> float:
    return angle % (2.0 * math.pi)


def _angular_error_rad(a: float, b: float) -> float:
    return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)


def nearest_lidar_sector(angle_rad: float) -> tuple[str, float]:
    """Return nearest named 45-degree sector and angular error in radians.

    Software convention: 0° FRONT, 90° LEFT, 180° BACK, 270° RIGHT.
    Physical installation must still be verified on the actual chassis.
    """
    angle = _normalize_angle_rad(float(angle_rad))
    width = math.pi / 4.0
    index = int(math.floor((angle + width / 2.0) / width)) % len(LIDAR_SECTOR_NAMES)
    center = index * width
    return LIDAR_SECTOR_NAMES[index], _angular_error_rad(angle, center)


def partition_lidar_samples(
    ranges: Sequence[float],
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    steering_cone_deg: float = 22.0,
    steering_half_width_deg: float | None = None,
) -> tuple[dict[str, list[float]], dict[str, list[float]], int]:
    """Partition one scan into gap-free safety sectors and narrow steering cones.

    ``steering_cone_deg`` is the full centered cone width used by production.
    ``steering_half_width_deg`` remains accepted for older tests/callers.
    """
    safety = {name: [] for name in LIDAR_SECTOR_NAMES}
    steering = {name: [] for name in LIDAR_SECTOR_NAMES}
    half_width_deg = (float(steering_half_width_deg) if steering_half_width_deg is not None
                      else max(0.0, float(steering_cone_deg)) / 2.0)
    steering_half_width = math.radians(half_width_deg)
    valid_points = 0

    for index, raw_distance in enumerate(ranges):
        try:
            distance = float(raw_distance)
        except (TypeError, ValueError, OverflowError):
            continue
        # ROS LaserScan commonly uses +inf for "no return within range". For
        # collision safety that is a valid clear sample at range_max; NaN and
        # negative infinity remain invalid and do not count toward scan health.
        if math.isinf(distance) and distance > 0.0:
            distance = float(range_max)
        elif not math.isfinite(distance):
            continue
        if distance < range_min or distance > range_max:
            continue
        angle = angle_min + index * angle_increment
        name, error = nearest_lidar_sector(angle)
        valid_points += 1
        safety[name].append(distance)
        if error <= steering_half_width:
            steering[name].append(distance)
    return safety, steering, valid_points


def _valid_distances(values_m: Iterable[float]) -> list[float]:
    valid: list[float] = []
    for value in values_m:
        try:
            number = float(value)
        except (TypeError, ValueError, OverflowError):
            continue
        if math.isfinite(number) and number > 0.0:
            valid.append(number)
    return valid


def robust_near_distance_cm(values_m: Iterable[float]) -> float | None:
    """Closest valid LiDAR return in centimeters for collision safety.

    Safety uses the nearest valid point. Noise rejection belongs in the temporal
    history/outlier layer, not in a percentile that could hide a thin obstacle.
    """
    values = _valid_distances(values_m)
    if not values:
        return None
    return round(min(values) * 100.0, 1)


def robust_wall_distance_cm(values_m: Iterable[float]) -> float | None:
    """Median wall-distance estimate in centimeters for hallway centering."""
    values = sorted(_valid_distances(values_m))
    if not values:
        return None
    middle = len(values) // 2
    if len(values) % 2:
        representative = values[middle]
    else:
        representative = (values[middle - 1] + values[middle]) / 2.0
    return round(representative * 100.0, 1)


def select_steering_samples(
    narrow_samples: Sequence[float],
    safety_samples: Sequence[float],
    *,
    minimum_points: int = 3,
) -> Sequence[float]:
    """Prefer the narrow steering cone; fall back to gap-free safety samples."""
    if len(narrow_samples) >= max(1, int(minimum_points)):
        return narrow_samples
    return safety_samples


def clamp_heading_error_cm(value: float, limit_cm: float = 40.0) -> float:
    limit = abs(float(limit_cm))
    return max(-limit, min(limit, float(value)))


def format_lidar_blockage(
    blocked_sides: Iterable[str],
    all_sides: Sequence[str] = LIDAR_SECTOR_NAMES,
) -> tuple[bool, str]:
    """Return ``(is_blocked, status_text)`` in deterministic sector order."""
    blocked = set(blocked_sides)
    allowed = set(all_sides)
    ordered = [name for name in LIDAR_SECTOR_NAMES if name in allowed and name in blocked]
    if not ordered:
        return False, "CLEAR"
    return True, "BLOCKED{" + ",".join(ordered) + "}"


def lidar_status_code(status: str) -> str:
    text = str(status).upper()
    if text == "CLEAR":
        return "C"
    if "STALE" in text or "UNAVAILABLE" in text:
        return "S"
    if "HUMAN" in text:
        return "H"
    return "O"


def build_health_status(
    *,
    cpu_temperature_c: float | None = None,
    throttled_flags: int | None = None,
    ble_connected: bool = False,
    lidar_healthy: bool | None = None,
    sensor_serial_connected: bool | None = None,
    ros_running: bool | None = None,
    lidar_scan_fresh: bool | None = None,
    sensor_link_fresh: bool | None = None,
    firebase_online: bool | None = None,
) -> dict[str, object]:
    """Build a health map without reporting false thermal OK when unknown."""
    lidar_value = lidar_healthy if lidar_healthy is not None else bool(lidar_scan_fresh)
    sensor_value = (sensor_serial_connected if sensor_serial_connected is not None
                    else bool(sensor_link_fresh))
    status: dict[str, object] = {
        "bleConnected": bool(ble_connected),
        "lidarHealthy": bool(lidar_value),
        "sensorSerialConnected": bool(sensor_value),
    }
    if ros_running is not None:
        status["rosRunning"] = bool(ros_running)
    if firebase_online is not None:
        status["firebaseOnline"] = bool(firebase_online)

    temp_valid = False
    if cpu_temperature_c is not None:
        try:
            temperature = float(cpu_temperature_c)
        except (TypeError, ValueError, OverflowError):
            temperature = math.nan
        if math.isfinite(temperature):
            status["cpuTemperatureC"] = round(temperature, 1)
            temp_valid = True

    throttle_valid = throttled_flags is not None
    if throttle_valid:
        status["throttledFlags"] = int(throttled_flags)

    if temp_valid or throttle_valid:
        thermal_warning = False
        if temp_valid and float(status["cpuTemperatureC"]) >= 80.0:
            thermal_warning = True
        if throttle_valid and int(status["throttledFlags"]) != 0:
            thermal_warning = True
        status["thermalWarning"] = thermal_warning
    else:
        status["thermalWarning"] = None
    return status

