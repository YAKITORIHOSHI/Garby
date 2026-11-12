"""Hardware-independent reliability primitives for GARBY's Raspberry Pi bridge.

This module intentionally has no BLE, ROS 2, serial, GUI, or Firebase imports.
Keeping these helpers hardware-independent makes the safety-critical data
handling testable on any host before deployment to the Raspberry Pi.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import math
import queue
from collections import deque
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


class AdaptiveTolerance:
    """Watchdog tolerance that adapts to the observed event cadence.

    A fixed watchdog trips spuriously when a healthy producer is throttled
    (slow CPU, stretched BLE connection interval). This helper tracks the
    observed inter-event period and widens the tolerance up to a hard cap
    while events keep arriving. The fixed floor still applies when no cadence
    is known yet or after ``reset``, so the fail-closed guarantee is
    unchanged: genuine silence always lapses at the floor.
    """

    def __init__(
        self,
        floor_s: float,
        cap_s: float,
        *,
        alpha: float = 0.25,
        margin_factor: float = 2.0,
    ) -> None:
        self.floor_s = max(0.01, float(floor_s))
        self.cap_s = max(self.floor_s, float(cap_s))
        self.alpha = min(1.0, max(0.01, float(alpha)))
        self.margin_factor = max(1.0, float(margin_factor))
        self._observed_s: float | None = None

    def observe(self, interval_s: float | None) -> None:
        """Fold one inter-event interval into the cadence estimate."""
        if interval_s is None:
            return
        interval = float(interval_s)
        if not math.isfinite(interval) or interval < 0.0:
            return
        if self._observed_s is None:
            self._observed_s = interval
        else:
            self._observed_s = (
                self.alpha * interval + (1.0 - self.alpha) * self._observed_s
            )

    def tolerance_s(self) -> float:
        if self._observed_s is None:
            return self.floor_s
        widened = self._observed_s * self.margin_factor
        return min(self.cap_s, max(self.floor_s, widened))

    def reset(self) -> None:
        self._observed_s = None


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
        self._close_deadline = None
        self._discarded = False
        self._in_flight = 0
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

    def _abort_locked(self):
        self._discarded = self._discarded or bool(self._pending) or bool(self._in_flight)
        self._pending.clear()
        self._flush_on_close = False
        self._cv.notify_all()

    def _expired_locked(self):
        if self._closing and self._close_deadline is not None and time.monotonic() >= self._close_deadline:
            self._abort_locked()
        return self._closing and not self._flush_on_close

    def pending_count(self):
        with self._cv:
            return len(self._pending) + self._in_flight

    def close(self, *, flush: bool = True, timeout: float = 2.0) -> bool:
        deadline = time.monotonic() + max(0.0, float(timeout))
        with self._cv:
            self._closing = True
            self._close_deadline = min(self._close_deadline, deadline) if self._close_deadline is not None else deadline
            if not flush:
                self._abort_locked()
            self._cv.notify_all()
        if self._thread.is_alive():
            self._thread.join(timeout=max(0.0, deadline - time.monotonic()))
        with self._cv:
            if self._thread.is_alive():
                # Cannot interrupt an in-flight network call. Prevent any retry
                # or additional write once that call returns.
                self._abort_locked()
            return not self._discarded and not self._thread.is_alive()

    def stop(self, timeout: float = 2.0) -> bool:
        return self.close(flush=False, timeout=timeout)

    def _wait_until(self, deadline: float) -> bool:
        with self._cv:
            while True:
                if self._expired_locked():
                    return False
                now = time.monotonic()
                if now >= deadline:
                    return True
                effective_deadline = min(deadline, self._close_deadline) if self._close_deadline is not None else deadline
                self._cv.wait(timeout=max(0.0, effective_deadline - now))

    def _run(self) -> None:
        retry_not_before = 0.0
        while True:
            with self._cv:
                while not self._pending and not self._closing:
                    self._cv.wait()
                if self._expired_locked() or (self._closing and not self._pending):
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
                if self._expired_locked():
                    return
                if not self._pending:
                    if self._closing:
                        return
                    continue
                batch = dict(self._pending)
                self._pending.clear()
                self._in_flight = len(batch)

            try:
                self._update_func(batch)
            except Exception:
                with self._cv:
                    self._in_flight = 0
                    if self._expired_locked():
                        self._discarded = True
                        return
                    # A newer value already pending wins over the failed old one.
                    for key, value in batch.items():
                        self._pending.setdefault(key, value)
                    if self._closing and not self._flush_on_close:
                        return
                retry_not_before = time.monotonic() + self._backoff.next_delay()
                continue

            with self._cv:
                self._in_flight = 0
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


@lru_cache(maxsize=32)
def _lidar_ray_geometry(count: int, angle_min: float, angle_increment: float):
    """Reuse only immutable angles, never ranges, validity or clearance.

    Without fixed resolution the X3's scan length follows its motor speed
    (667-672 points measured on 2026-11-12), so several lengths alternate.
    """
    return tuple(nearest_lidar_sector(angle_min + index * angle_increment)
                 for index in range(count))


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
    if (not all(math.isfinite(v) for v in (angle_min, angle_increment, range_min, range_max))
            or (angle_increment == 0.0 and len(ranges) > 1)
            or range_min < 0.0 or range_max <= range_min):
        return safety, steering, 0
    half_width_deg = (float(steering_half_width_deg) if steering_half_width_deg is not None
                      else max(0.0, float(steering_cone_deg)) / 2.0)
    steering_half_width = math.radians(half_width_deg)
    valid_points = 0
    # Bound retained geometry even for unusual drivers. Geometry changes select
    # a different key; every ray's distance/validity is still checked every scan.
    count = len(ranges)
    geometry = (_lidar_ray_geometry(count, angle_min, angle_increment)
                if count <= 4096 and math.isfinite(angle_min + max(0, count - 1) * angle_increment)
                else None)

    for index, raw_distance in enumerate(ranges):
        try:
            distance = float(raw_distance)
        except (TypeError, ValueError, OverflowError):
            continue
        # ROS LaserScan commonly uses +inf for "no return within range". For
        # collision safety that is a valid clear sample at range_max; NaN and
        # negative infinity remain invalid and do not count toward scan health.
        no_return = math.isinf(distance) and distance > 0.0
        if no_return:
            distance = float(range_max)
        elif not math.isfinite(distance):
            continue
        if distance <= 0.0 or distance < range_min or distance > range_max:
            continue
        name, error = (geometry[index] if geometry is not None else
                       nearest_lidar_sector(angle_min + index * angle_increment))
        valid_points += 1
        safety[name].append(distance)
        # A clear ray is useful for collision clearance, but it is not a
        # measured wall. L/R centering requires finite lateral wall returns.
        if error <= steering_half_width and not (no_return and name in ("LEFT", "RIGHT")):
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


def side_obstacle_names(
    distances_cm: Mapping[str, float | int | None],
    threshold_cm: float = 22.0,
) -> list[str]:
    """Return close lateral sectors in deterministic left/right order."""
    threshold = float(threshold_cm)
    blocked = []
    for name in ("LEFT", "RIGHT"):
        value = distances_cm.get(name)
        if value is None:
            continue
        try:
            distance = float(value)
        except (TypeError, ValueError, OverflowError):
            continue
        if math.isfinite(distance) and distance <= threshold:
            blocked.append(name)
    return blocked


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


class ManualStartControl:
    """Track an operator start without depending on scale or sensor readings.

    Readiness and IDLE must be confirmed by the executor in the current BLE
    session. Starts expire in the outgoing queue and are never auto-retried.
    """

    READY_TIMEOUT_S = 7.5  # Executor status is queried every five seconds.
    QUEUE_TIMEOUT_S = 2.0
    ACK_TIMEOUT_S = 5.0
    QUERY_INTERVAL_S = 1.0

    def __init__(self):
        self._lock = threading.RLock()
        self._last_id = 0
        self.reset()

    def reset(self):
        with self._lock:
            self._ready_at = self._state_at = None
            self._state = "UNKNOWN"
            self._pending = None
            self._sent_at = self._last_query_at = None
            self._message = ""

    def note_ready(self, now=None):
        with self._lock:
            self._ready_at = time.monotonic() if now is None else now

    def note_state(self, state, now=None):
        with self._lock:
            if self._message.startswith("Start unconfirmed") or (state == "IDLE" and self._state != "IDLE"):
                self._message = ""
            self._state = state
            self._state_at = time.monotonic() if now is None else now

    def _fresh(self, timestamp, now):
        return timestamp is not None and 0 <= now - timestamp <= self.READY_TIMEOUT_S

    def status(self, connected, now=None):
        now = time.monotonic() if now is None else now
        with self._lock:
            if self._pending and now - self._pending[1] > self.ACK_TIMEOUT_S:
                self._pending = None
                self._state_at = None  # Require new state after an uncertain write.
                self._message = "Start unconfirmed; waiting for controller status"
            if not connected:
                reason = "Waiting for BLE connection"
            elif not self._fresh(self._ready_at, now):
                reason = "Waiting for both controllers"
            elif self._pending:
                reason = "Start requested; waiting for acknowledgement"
            elif not self._fresh(self._state_at, now):
                reason = "Waiting for controller status"
            elif self._state != "IDLE":
                reason = "Robot must be idle to start"
            else:
                reason = "Ready: bypass scale and bin sensors"
            enabled = (connected and self._fresh(self._ready_at, now)
                       and self._fresh(self._state_at, now)
                       and self._state == "IDLE" and self._pending is None)
            message = reason
            if self._message and connected and self._fresh(self._ready_at, now) and not self._pending:
                message = self._message
            return {"enabled": bool(enabled), "pending": self._pending is not None,
                    "message": message}

    def request(self, connected, now=None):
        now = time.monotonic() if now is None else now
        with self._lock:
            if not self.status(connected, now)["enabled"]:
                return None
            self._last_id = max(self._last_id + 1, int(time.time() * 1000))
            self._pending = (str(self._last_id), now)
            self._sent_at = self._last_query_at = None
            self._message = ""
            return f"[START:{self._last_id}]"

    def can_send(self, packet, connected, now=None):
        now = time.monotonic() if now is None else now
        with self._lock:
            if (not self._pending or packet != f"[START:{self._pending[0]}]"
                    or self._sent_at is not None):
                return False
            valid = (connected and self._fresh(self._ready_at, now)
                     and self._fresh(self._state_at, now) and self._state == "IDLE"
                     and 0 <= now - self._pending[1] <= self.QUEUE_TIMEOUT_S)
            if not valid:
                self._pending = None
                self._message = "Start cancelled; check connections and try again"
            return bool(valid)

    def mark_sent(self, packet, now=None):
        """Record an attempted write, including one with an uncertain result."""
        now = time.monotonic() if now is None else now
        with self._lock:
            if self._pending and packet == f"[START:{self._pending[0]}]":
                self._sent_at = self._last_query_at = now

    def next_query(self, connected, now=None):
        """Ask for the pending result; never repeat the motion command."""
        now = time.monotonic() if now is None else now
        with self._lock:
            if (not self.status(connected, now)["pending"] or not connected
                    or self._sent_at is None
                    or now - self._last_query_at < self.QUERY_INTERVAL_S):
                return None
            self._last_query_at = now
            return f"[START STATUS:{self._pending[0]}]"

    def can_query(self, packet, connected, now=None):
        now = time.monotonic() if now is None else now
        with self._lock:
            return bool(self.status(connected, now)["pending"] and connected
                        and self._sent_at is not None
                        and packet == f"[START STATUS:{self._pending[0]}]")

    def outcome(self, request_id, accepted, reason=""):
        with self._lock:
            if not self._pending or request_id != self._pending[0]:
                return False
            self._pending = None
            self._message = "Manual start accepted" if accepted else f"Start rejected: {reason}"
            # The next state report decides whether another start is allowed.
            self._state_at = None
            return True


THROTTLE_UNDERVOLTAGE_NOW = 0x1
_THROTTLE_FLAG_NAMES: tuple[tuple[int, str], ...] = (
    (THROTTLE_UNDERVOLTAGE_NOW, "undervoltage-now"),
    (0x2, "arm-capped-now"),
    (0x4, "throttled-now"),
    (0x8, "soft-temp-limit-now"),
    (0x10000, "undervoltage-since-boot"),
    (0x20000, "arm-capped-since-boot"),
    (0x40000, "throttled-since-boot"),
    (0x80000, "soft-temp-limit-since-boot"),
)


def describe_throttle_flags(flags: int | None) -> str:
    """Readable form of the Raspberry Pi firmware ``get_throttled`` bit field."""
    if flags is None:
        return "unknown"
    return ", ".join(name for bit, name in _THROTTLE_FLAG_NAMES if flags & bit) or "none"


class CoalescingBleQueue:
    """Small latest-value mailbox for BLE traffic.

    PATH, SIDES, and SENSOR messages replace older messages of the same kind,
    preventing stale clear-path packets from accumulating behind newer STOP
    packets. Control messages are retained. Urgent safety messages are placed
    before ordinary traffic, with operator ESTOP first. Sequenced paths other
    than all-clear get STOP priority on every replacement, including polls.
    """
    def __init__(self, maxlen=12):
        self._maxlen = maxlen
        self._items = deque()
        self._lock = threading.Lock()
        self._latest_path_seq = None

    @staticmethod
    def _kind(msg):
        if msg.startswith(("P:", "PATH:")):
            return "PATH"
        if msg.startswith(("S:", "SIDES:")):
            return "SIDES"
        if msg.startswith("SENSOR:"):
            return "SENSOR"
        if msg.startswith("[HEALTH:"):
            return "HEALTH"
        if msg == "[RESET]" or msg.startswith("[RESET:"):
            return "RESET"
        if msg == "[RASPI READY]":
            return "READY"
        if msg.startswith("[START:"):
            return "START"
        if msg.startswith("[START STATUS:"):
            return "START_STATUS"
        return "CONTROL"

    @staticmethod
    def _operator_estop(msg):
        # Priority hint only; both firmware nodes still validate the command.
        return msg.startswith("[OP:") and msg.endswith(":ESTOP]")

    def put_nowait(self, msg, urgent=False, expires_at=None):
        kind = self._kind(msg)
        is_estop = self._operator_estop(msg)
        urgent = urgent or is_estop
        if expires_at is not None:
            try:
                expires_at = float(expires_at)
            except (TypeError, ValueError, OverflowError):
                expires_at = 0.0
            if not math.isfinite(expires_at):
                expires_at = 0.0
        entry = (msg, expires_at)
        with self._lock:
            if msg.startswith("P:"):
                seq = int(msg[2:].split("|", 1)[0])
                if self._latest_path_seq is not None:
                    delta = (seq - self._latest_path_seq) & 0xFFFFFFFF
                    if delta == 0 or delta >= 0x80000000:
                        return
                self._latest_path_seq = seq
                # Recognize only ordinary all-clear here, not the complete wire
                # grammar. Blocked, stale or malformed bodies need prompt bridge
                # validation/STOP; a periodic replacement must not demote them.
                # The optional T= turn-sweep token is part of all-clear only
                # when it is itself C; slow (F=W) keeps STOP-class priority.
                tokens = msg.partition("|")[2].split("|")
                urgent = urgent or sorted(tokens) not in (
                    ["B=C", "F=C"], ["B=C", "F=C", "T=C"])
            if kind != "CONTROL":
                retained = deque()
                for item in self._items:
                    if self._kind(item[0]) != kind:
                        retained.append(item)
                self._items = retained
            while len(self._items) >= self._maxlen:
                # Never discard the next urgent PATH/RESET just to retain stale
                # steering or telemetry. Search from the tail (newest/lowest
                # priority) for an expendable item first.
                remove_index = None
                for index in range(len(self._items) - 1, -1, -1):
                    item = self._items[index][0]
                    if (not self._operator_estop(item) and self._kind(item) in
                            ("SIDES", "SENSOR", "HEALTH", "READY", "CONTROL")):
                        remove_index = index
                        break
                if remove_index is None:
                    # Even an urgent PATH cannot evict a queued emergency stop.
                    remove_index = next((index for index in range(len(self._items) - 1, -1, -1)
                                         if not self._operator_estop(self._items[index][0])), None)
                if remove_index is None:
                    if not is_estop:
                        return
                    remove_index = len(self._items) - 1
                del self._items[remove_index]
            if urgent:
                # ESTOP admission keeps emergency entries at the front. Other
                # urgent traffic starts immediately after that emergency prefix.
                index = 0
                if not is_estop:
                    while index < len(self._items) and self._operator_estop(self._items[index][0]):
                        index += 1
                self._items.insert(index, entry)
            else:
                self._items.append(entry)

    def get_entry_nowait(self):
        """Return ``(message, monotonic_deadline)`` for the BLE writer."""
        with self._lock:
            if not self._items:
                raise queue.Empty
            return self._items.popleft()

    def get_nowait(self):
        return self.get_entry_nowait()[0]

    def clear(self):
        with self._lock:
            self._items.clear()
            self._latest_path_seq = None

    def discard_kind(self, kind):
        with self._lock:
            self._items = deque(
                item for item in self._items if self._kind(item[0]) != kind
            )
