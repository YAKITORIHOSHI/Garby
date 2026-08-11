"""Hardware-independent reliability primitives for the GARBY Raspberry Pi bridge.

This module deliberately has no BLE, ROS, serial, or Firebase imports.  Keeping
the state machines here small and deterministic makes the safety-critical
transition behaviour testable on a development machine.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, Iterable, Mapping, Optional


@dataclass(frozen=True)
class SensorSpec:
    """Validation and unavailable-value policy for one hardware sensor."""

    sentinel: float
    minimum: float
    maximum: float
    integer: bool = False

    def normalize(self, raw_value: object) -> Optional[float | int]:
        try:
            value = float(raw_value)
        except (TypeError, ValueError):
            return None
        if not math.isfinite(value):
            return None
        if value == self.sentinel or value < self.minimum or value > self.maximum:
            return None
        return int(round(value)) if self.integer else round(value, 2)


SENSOR_SPECS: Dict[str, SensorSpec] = {
    # 999 and -1 are the database/API contract for unavailable sensors.
    "ultrasonic": SensorSpec(sentinel=999.0, minimum=0.1, maximum=600.0),
    "mq4": SensorSpec(sentinel=-1, minimum=0.0, maximum=100_000.0, integer=True),
    "mq137": SensorSpec(sentinel=-1, minimum=0.0, maximum=100_000.0, integer=True),
    "mq135": SensorSpec(sentinel=-1, minimum=0.0, maximum=100_000.0, integer=True),
}


class SensorTransitionTracker:
    """Coalesce live samples and emit an unavailable sentinel once per outage.

    ``collect_due`` returns only logical state transitions plus bounded-rate live
    refreshes.  Once an unavailable sentinel has been returned, it cannot be
    returned again until a valid sample has first changed that sensor back to
    available.  This is the key invariant that prevents an absent UART sensor
    from hammering Firebase indefinitely.
    """

    def __init__(
        self,
        *,
        specs: Mapping[str, SensorSpec] = SENSOR_SPECS,
        stale_after_s: float = 8.0,
        live_publish_period_s: float = 5.0,
        started_at: Optional[float] = None,
    ) -> None:
        if stale_after_s <= 0 or live_publish_period_s <= 0:
            raise ValueError("sensor timing values must be positive")
        self.specs = dict(specs)
        self.stale_after_s = stale_after_s
        self.live_publish_period_s = live_publish_period_s
        now = time.monotonic() if started_at is None else started_at
        self.started_at = now
        self._last_seen: Dict[str, Optional[float]] = {key: None for key in self.specs}
        self._latest: Dict[str, Optional[float | int]] = {key: None for key in self.specs}
        self._offline_announced: Dict[str, bool] = {key: False for key in self.specs}
        self._transition_pending: Dict[str, float | int] = {}
        self._last_live_publish = now - live_publish_period_s
        self._lock = threading.Lock()

    def ingest(self, key: str, raw_value: object, *, now: Optional[float] = None) -> bool:
        """Record one sample; return ``True`` only when it is valid/live."""
        if key not in self.specs:
            return False
        sample_time = time.monotonic() if now is None else now
        normalized = self.specs[key].normalize(raw_value)
        with self._lock:
            if normalized is None:
                self._set_unavailable_locked(key)
                return False

            was_unavailable = self._last_seen[key] is None or self._offline_announced[key]
            self._last_seen[key] = sample_time
            self._latest[key] = normalized
            if was_unavailable:
                # Availability changed: publish the recovery immediately.
                self._transition_pending[key] = normalized
            self._offline_announced[key] = False
            return True

    def mark_unavailable(self, keys: Optional[Iterable[str]] = None) -> None:
        with self._lock:
            for key in self.specs if keys is None else keys:
                if key in self.specs:
                    self._set_unavailable_locked(key)

    def _set_unavailable_locked(self, key: str) -> None:
        if not self._offline_announced[key]:
            self._transition_pending[key] = self.specs[key].sentinel
            self._offline_announced[key] = True
        self._last_seen[key] = None
        self._latest[key] = None

    def collect_due(self, *, now: Optional[float] = None) -> Dict[str, float | int]:
        """Return and consume the next atomic sensor update batch."""
        check_time = time.monotonic() if now is None else now
        with self._lock:
            for key, seen_at in self._last_seen.items():
                if seen_at is None:
                    if (
                        not self._offline_announced[key]
                        and check_time - self.started_at >= self.stale_after_s
                    ):
                        self._set_unavailable_locked(key)
                elif check_time - seen_at >= self.stale_after_s:
                    self._set_unavailable_locked(key)

            due = dict(self._transition_pending)
            self._transition_pending.clear()

            if check_time - self._last_live_publish >= self.live_publish_period_s:
                for key, value in self._latest.items():
                    if value is not None and self._last_seen[key] is not None:
                        due[key] = value
                self._last_live_publish = check_time
            return due

    def latest_for_transport(self) -> Dict[str, Optional[float | int]]:
        with self._lock:
            return dict(self._latest)

    def is_link_fresh(self, *, now: Optional[float] = None) -> bool:
        check_time = time.monotonic() if now is None else now
        with self._lock:
            seen = [value for value in self._last_seen.values() if value is not None]
            return bool(seen) and check_time - max(seen) < self.stale_after_s


class ExponentialBackoff:
    """Small bounded exponential backoff with deterministic optional jitter."""

    def __init__(
        self,
        minimum_s: float,
        maximum_s: float,
        *,
        factor: float = 2.0,
        jitter: Optional[Callable[[float], float]] = None,
    ) -> None:
        if minimum_s <= 0 or maximum_s < minimum_s or factor < 1:
            raise ValueError("invalid backoff configuration")
        self.minimum_s = minimum_s
        self.maximum_s = maximum_s
        self.factor = factor
        self.jitter = jitter
        self._next = minimum_s

    def reset(self) -> None:
        self._next = self.minimum_s

    def next_delay(self) -> float:
        delay = self._next
        self._next = min(self.maximum_s, self._next * self.factor)
        if self.jitter is not None:
            delay = self.jitter(delay)
        return max(0.0, min(self.maximum_s, delay))


class CoalescingUpdateWorker:
    """Commit atomic multi-location updates without blocking control threads.

    The pending buffer is bounded by the finite set of Firebase paths rather
    than by the number of samples received.  Newer values replace older values.
    Failed commits are merged back behind any newer values and retried using a
    bounded backoff.
    """

    def __init__(
        self,
        commit: Callable[[Mapping[str, object]], None],
        *,
        batch_window_s: float = 0.20,
        retry_min_s: float = 1.0,
        retry_max_s: float = 30.0,
        name: str = "firebase-writer",
    ) -> None:
        self._commit = commit
        self._batch_window_s = batch_window_s
        self._condition = threading.Condition()
        self._pending: Dict[str, object] = {}
        self._urgent = False
        self._stopping = False
        self._flush_on_stop = True
        self._backoff = ExponentialBackoff(retry_min_s, retry_max_s)
        self._thread = threading.Thread(target=self._run, daemon=True, name=name)
        self._thread.start()

    def enqueue(self, updates: Mapping[str, object], *, urgent: bool = False) -> None:
        if not updates:
            return
        with self._condition:
            if self._stopping:
                return
            self._pending.update(updates)
            self._urgent = self._urgent or urgent
            self._condition.notify()

    def close(self, *, flush: bool = True, timeout: float = 3.0) -> None:
        with self._condition:
            self._stopping = True
            self._flush_on_stop = flush
            self._condition.notify_all()
        self._thread.join(timeout=timeout)

    def _run(self) -> None:
        while True:
            with self._condition:
                while not self._pending and not self._stopping:
                    self._condition.wait()
                if self._stopping and (not self._flush_on_stop or not self._pending):
                    return
                if not self._urgent and not self._stopping:
                    self._condition.wait(timeout=self._batch_window_s)
                batch = dict(self._pending)
                self._pending.clear()
                self._urgent = False

            try:
                self._commit(batch)
                self._backoff.reset()
            except Exception:
                # Preserve newer values if they arrived while commit() blocked.
                with self._condition:
                    retry_batch = dict(batch)
                    retry_batch.update(self._pending)
                    self._pending = retry_batch
                    if self._stopping and not self._flush_on_stop:
                        return
                    if self._stopping:
                        # One final flush was attempted; never spin during exit.
                        return
                    # Enqueues may wake the condition, but they must not bypass
                    # the network retry backoff and create an outage hot-loop.
                    retry_at = time.monotonic() + self._backoff.next_delay()
                    while not self._stopping:
                        remaining = retry_at - time.monotonic()
                        if remaining <= 0:
                            break
                        self._condition.wait(timeout=remaining)


def robust_near_distance_cm(distances_m: Iterable[float]) -> Optional[float]:
    """Return a near-obstacle estimate that rejects a single LiDAR speck.

    A low percentile remains responsive to people and narrow objects, while
    avoiding the aggressive behaviour caused by using the absolute minimum of
    a wide cone.  For very small samples we retain the minimum (there is not
    enough information to reject anything safely).
    """
    ordered = sorted(float(value) for value in distances_m if math.isfinite(value) and value > 0)
    if not ordered:
        return None
    index = 0 if len(ordered) < 5 else max(1, int(0.15 * (len(ordered) - 1)))
    return ordered[min(index, len(ordered) - 1)] * 100.0


def robust_wall_distance_cm(distances_m: Iterable[float]) -> Optional[float]:
    ordered = sorted(float(value) for value in distances_m if math.isfinite(value) and value > 0)
    if not ordered:
        return None
    return ordered[len(ordered) // 2] * 100.0


def clamp_heading_error_cm(value: float, limit_cm: float = 20.0) -> float:
    return max(-limit_cm, min(limit_cm, value))


LIDAR_SECTOR_NAMES = (
    "BACK",
    "BACK_RIGHT",
    "RIGHT",
    "FRONT_RIGHT",
    "FRONT",
    "FRONT_LEFT",
    "LEFT",
    "BACK_LEFT",
)


def nearest_lidar_sector(angle_rad: float):
    """Return ``(named_sector, absolute_center_error_rad)`` in O(1).

    Sectors are centered every 45 degrees using the robot's established LiDAR
    orientation: 0° BACK, 90° RIGHT, 180° FRONT, and 270° LEFT. Half-open
    nearest-sector boundaries guarantee every finite angle maps to exactly one
    sector, including wraparound and exact 22.5° boundaries.
    """
    circle = 2.0 * math.pi
    sector_width = math.pi / 4.0
    normalized = angle_rad % circle
    sector_index = int((normalized + sector_width / 2.0) / sector_width) % 8
    center = sector_index * sector_width
    error = abs(normalized - center)
    error = min(error, circle - error)
    return LIDAR_SECTOR_NAMES[sector_index], error


def partition_lidar_samples(
    ranges: Iterable[float],
    *,
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    steering_cone_deg: float = 22.0,
):
    """Split one scan into gap-free safety and narrow steering buckets.

    Every finite in-range point enters exactly one 45° nearest safety sector.
    Points within the narrower centered slice additionally enter its steering
    bucket. Complexity is O(n); there are no per-point eight-way comparisons.
    """
    safety = {side: [] for side in LIDAR_SECTOR_NAMES}
    steering = {side: [] for side in LIDAR_SECTOR_NAMES}
    steering_half_rad = math.radians(steering_cone_deg / 2.0)
    valid_count = 0

    for index, raw_range in enumerate(ranges):
        try:
            distance = float(raw_range)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(distance) or distance < range_min or distance > range_max:
            continue
        side, center_error = nearest_lidar_sector(
            angle_min + index * angle_increment
        )
        safety[side].append(distance)
        valid_count += 1
        if center_error <= steering_half_rad:
            steering[side].append(distance)

    return safety, steering, valid_count


def select_steering_samples(
    narrow_samples: Iterable[float],
    safety_samples: Iterable[float],
    *,
    minimum_points: int = 3,
):
    """Prefer centered wall samples, falling back to the gap-free sector."""
    narrow = list(narrow_samples)
    return narrow if len(narrow) >= minimum_points else list(safety_samples)


def format_lidar_blockage(blocked_sides: Iterable[str], all_sides: Iterable[str]):
    """Classify a path using LiDAR geometry only.

    The Raspberry Pi UART ultrasonic measures trash level inside the bin.  It is
    intentionally absent from this API so that a full/empty bin can never turn
    a hallway obstacle into a person classification or otherwise change motion.
    """
    blocked = list(blocked_sides)
    all_side_set = set(all_sides)
    if not blocked:
        return False, "CLEAR"
    if set(blocked) == all_side_set:
        return True, "BLOCKED{ALL}"
    return True, f"BLOCKED{{{','.join(blocked)}}}"


def lidar_status_code(status: str) -> str:
    """Encode path status, retaining legacy H/S parser compatibility."""
    if status == "CLEAR":
        return "C"
    if "HUMAN_DETECTED" in status:
        return "H"
    if "LIDAR_STALE" in status:
        return "S"
    return "O"


def build_health_status(
    *,
    cpu_temperature_c: Optional[float],
    throttled_flags: Optional[int],
    ble_connected: bool,
    lidar_healthy: bool,
    sensor_serial_connected: bool,
):
    """Build truthful low-rate health telemetry from available evidence.

    Link booleans are always observable within this process. Thermal state is
    different: when neither sysfs temperature nor ``vcgencmd`` is available,
    omitting thermal fields represents UNKNOWN and avoids reporting a false OK.
    """
    status = {
        "bleConnected": bool(ble_connected),
        "lidarHealthy": bool(lidar_healthy),
        "sensorSerialConnected": bool(sensor_serial_connected),
    }
    have_temperature = cpu_temperature_c is not None
    have_throttle_flags = throttled_flags is not None

    if have_temperature:
        status["cpuTemperatureC"] = round(cpu_temperature_c)
    if have_throttle_flags:
        status["throttledFlags"] = int(throttled_flags)
    if have_temperature or have_throttle_flags:
        thermal_bits = (1 << 1) | (1 << 2) | (1 << 3)
        status["thermalWarning"] = bool(
            (have_temperature and cpu_temperature_c >= 75.0)
            or (have_throttle_flags and throttled_flags & thermal_bits)
        )
    return status
