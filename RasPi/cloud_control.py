"""Phone control and the LiDAR map over Firebase (2026-11-14).

The owner: "I was planning to have a headless run of this project and it's
overall control was on the mobile itself."

Three things cross the cloud, all under ``devices/<id>/``:

* ``commands/operator`` - one operator request written by the app. The Pi
  claims it, hands it to the SAME function the monitor's buttons call, and
  writes the outcome the controller confirmed. The phone can never do what the
  monitor cannot, and a request never grants motion: Release, Continue and
  Reverse still pass every gate on the robot.
* ``operator`` - what the monitor's Controls tab knows, so the app can enable
  its buttons from confirmed state instead of guessing.
* ``map/frame`` - one compact scan frame, written only while an app holds a
  fresh lease in ``commands/map`` (nobody watching costs nothing).

Rules that keep it safe
-----------------------
* At most once. A request is claimed in a transaction before it is acted on. A
  claimed request whose outcome this process does not know (a Pi restart) ends
  "unconfirmed" and is never sent again.
* Age. An emergency stop is honoured up to ESTOP_MAX_AGE_MS old: it only ever
  stops. Every other action older than ACTION_MAX_AGE_MS is refused, so a
  release or a continue can never fire late after a reconnect.
* The phone's stop is a convenience, not a safety device: it depends on the
  phone's and the robot's internet links and is polled once a second.
* The read-only toggles obey the Pi's ALLOW_RUNTIME_SAFETY_DISABLE and are off
  again after every restart.
* Health, Firebase, UI and SMS never grant GO. Nothing here is path evidence.
"""
import base64
import math
import re
import time

try:  # numpy ships with ROS 2 on the Pi.
    import numpy as np
except ImportError:  # pragma: no cover - stripped install: no map frames
    np = None

CONTROLLER_ACTIONS = ("ESTOP", "RELEASE", "CONTINUE", "REVERSE", "SONAR_ON", "SONAR_OFF",
                      "REBOOT")
READ_ONLY_ACTIONS = {"FRONT_READONLY_ON": ("front", True), "FRONT_READONLY_OFF": ("front", False),
                     "BACK_READONLY_ON": ("back", True), "BACK_READONLY_OFF": ("back", False),
                     "SIDE_READONLY_ON": ("side", True), "SIDE_READONLY_OFF": ("side", False)}
DEV_ACTIONS = ("DEV_TEST_F", "DEV_TEST_B", "DEV_TEST_L", "DEV_TEST_R", "DEV_TEST_L90",
               "DEV_TEST_R90", "DEV_TEST_M1", "DEV_TEST_M2", "DEV_TEST_I", "DEV_CANCEL",
               "DEV_RUN_START", "DEV_RUN_CONTINUE", "DEV_RUN_UNDO", "DEV_RUN_CLEAR",
               "DEV_HOLD_W", "DEV_HOLD_S", "DEV_HOLD_A", "DEV_HOLD_D", "DEV_HOLD_STOP")
ESTOP_MAX_AGE_MS = 120_000
ACTION_MAX_AGE_MS = 20_000
FUTURE_TOLERANCE_MS = 5_000
RESULT_TIMEOUT_MS = 8_000        # the operator controller's own ACK window is 5 s
COMMAND_POLL_S = 1.0
STATUS_REFRESH_S = 10.0          # the app calls the mirror stale after 30 s
MAP_LEASE_MS = 45_000            # the app renews its lease every 20 s
MAP_PERIOD_S = 0.5
MAP_PERIOD_HOT_S = 1.0           # a hot Pi halves the map rate
MAP_MAX_SCAN_AGE_S = 0.8         # never publish a scan the path logic calls stale
MAP_MAX_MARKERS = 12
MARKER_CODES = {"person": 1, "moving": 2, "object": 3}


def command_problem(command, now_ms):
    """None for a well-formed, fresh pending request; otherwise why it is refused."""
    if not isinstance(command, dict):
        return "invalid_request"
    action = command.get("action")
    if (type(command.get("version")) is not int or command["version"] != 1
            or not isinstance(command.get("requestId"), str)
            or not re.fullmatch(r"[0-9a-f]{32}", command["requestId"])
            or not isinstance(command.get("requestedBy"), str)
            or not 1 <= len(command["requestedBy"].strip()) <= 128
            or type(command.get("requestedAt")) is not int or command["requestedAt"] <= 0
            or not isinstance(action, str)):
        return "invalid_request"
    if action not in CONTROLLER_ACTIONS and action not in READ_ONLY_ACTIONS and action not in DEV_ACTIONS:
        return "unknown_action"
    age = now_ms - command["requestedAt"]
    limit = ESTOP_MAX_AGE_MS if action == "ESTOP" else ACTION_MAX_AGE_MS
    if not -FUTURE_TOLERANCE_MS <= age <= limit:
        return "expired_request"
    return None


class OperatorRelay:
    """One cloud operator request at a time, acted on at most once.

    request_action(action) -> (controller request id or None, message)
    action_outcome(request id) -> None while unknown, else (accepted, detail)
    set_read_only(group, enabled) -> (accepted, reason)
    clock() -> seconds, monotonic: measures how long a claim took
    """

    def __init__(self, request_action, action_outcome, set_read_only, dev_action=None, clock=time.monotonic):
        self._request_action = request_action
        self._action_outcome = action_outcome
        self._set_read_only = set_read_only
        self._dev_action = dev_action
        self._clock = clock
        self._active = None      # {"requestId", "controllerId", "at"} of this process's claim

    def poll(self, manager, device_id, command, now_ms=None):
        now_ms = int(time.time() * 1000) if now_ms is None else now_ms
        if not isinstance(command, dict):
            return
        status = command.get("status")
        if status == "pending":
            self._pending(manager, device_id, command, now_ms)
        elif status == "claimed":
            self._claimed(manager, device_id, command, now_ms)

    def _pending(self, manager, device_id, command, now_ms):
        problem = command_problem(command, now_ms)
        action = command.get("action")
        if action == "ESTOP" and problem in (None, "expired_request"):
            # A stop is applied before anything else, even a late or a
            # duplicated one: it can only ever stop the robot.
            controller_id, _message = self._request_action("ESTOP")
            if problem is not None:
                manager.finish_operator(device_id, command, "failed", problem)
                return
            if manager.claim_operator(device_id, command):
                self._active = {"requestId": command["requestId"], "controllerId": controller_id,
                                "at": now_ms}
            return
        if problem is not None:
            manager.finish_operator(device_id, command, "failed", problem)
            return
        # Claim first: after this, no other poll (or process) acts on it.
        claim_started = self._clock()
        if not manager.claim_operator(device_id, command):
            return
        # The claim is a database transaction and can take seconds: the request
        # must still be fresh now, when it is acted on.
        acted_ms = now_ms + int((self._clock() - claim_started) * 1000)
        if command_problem(command, acted_ms):
            manager.finish_operator(device_id, command, "failed", "expired_request")
            return
        if action in READ_ONLY_ACTIONS:
            group, enabled = READ_ONLY_ACTIONS[action]
            accepted, reason = self._set_read_only(group, enabled)
            manager.finish_operator(device_id, command, "accepted" if accepted else "failed",
                                    reason)
            return
        if action in DEV_ACTIONS:
            if self._dev_action is None:
                manager.finish_operator(device_id, command, "failed", "developer_tests_unavailable")
                return
            accepted, reason = self._dev_action(action, command)
            manager.finish_operator(device_id, command, "accepted" if accepted else "failed",
                                    reason)
            return
        controller_id, message = self._request_action(action)
        if controller_id is None:
            manager.finish_operator(device_id, command, "failed", message or "not_available")
            return
        # The controller's answer is awaited from the moment the action was handed over.
        self._active = {"requestId": command["requestId"], "controllerId": controller_id,
                        "at": acted_ms}

    def _claimed(self, manager, device_id, command, now_ms):
        active = self._active
        if active is None or active["requestId"] != command.get("requestId"):
            # Claimed by a process that is gone: its outcome is unknown and the
            # action is never sent again.
            manager.finish_operator(device_id, command, "unconfirmed",
                                    "controller_result_unverified")
            return
        outcome = (self._action_outcome(active["controllerId"])
                   if active["controllerId"] is not None else None)
        if outcome is not None:
            accepted, detail = outcome
            manager.finish_operator(device_id, command, "accepted" if accepted else "failed",
                                    "" if accepted else detail)
            self._active = None
        elif now_ms - active["at"] > RESULT_TIMEOUT_MS:
            reason = ("stop_requested_controller_unconfirmed" if command.get("action") == "ESTOP"
                      else "controller_result_unverified")
            manager.finish_operator(device_id, command, "unconfirmed", reason)
            self._active = None


def operator_mirror(status, *, robot_state, front_read_only, back_read_only, side_read_only=False,
                    developer=None, read_only_allowed, motion_hold, zones, route_leg, people,
                    objects, launcher, now_ms=None):
    """The ``devices/<id>/operator`` document from the monitor's own status."""
    now_ms = int(time.time() * 1000) if now_ms is None else now_ms
    developer = developer or {}
    return {
        "version": 1,
        "updatedAt": now_ms,
        "supported": bool(status.get("supported")),
        "connected": bool(status.get("connected")),
        "pending": bool(status.get("pending")),
        # None stays absent in RTDB: unknown must not read as released.
        "emergencyStopped": status.get("emergency_stopped"),
        "ultrasonicEnabled": status.get("ultrasonic_enabled"),
        "canConfigure": bool(status.get("can_configure")),
        "canRelease": bool(status.get("can_release")),
        "canContinue": bool(status.get("can_continue")),
        "canReverse": bool(status.get("can_reverse")),
        "canReset": bool(status.get("can_reset")),
        "message": str(status.get("message", ""))[:160],
        "robotState": str(robot_state),
        "frontReadOnly": bool(front_read_only),
        "backReadOnly": bool(back_read_only),
        "sideReadOnly": bool(side_read_only),
        "readOnlyAllowed": bool(read_only_allowed),
        "motionHold": str(motion_hold or "")[:80],
        "zones": ",".join(zones)[:160],
        "routeLeg": str(route_leg or "")[:120],
        "people": int(people),
        "objects": int(objects),
        "developerEnabled": bool(developer.get("enabled")),
        "developerPending": bool(developer.get("pending")),
        "developerResult": str(developer.get("result", ""))[:120],
        # 2026-11-18: what the Developer window shows, so the phone can say why a test is
        # unavailable, how the last one ended and where a RunStart list stands.
        "developerMessage": str(developer.get("message", ""))[:160],
        "developerBlocked": str(developer.get("blocked", ""))[:160],
        "developerRun": developer_run_mirror(developer.get("run")),
        "launcher": launcher,
    }


def developer_run_mirror(run):
    """The RunStart tab's state for the phone. While a list moves only its step number is
    sent: [RUN AT] arrives four times a second and would rewrite the mirror as often."""
    run = run if isinstance(run, dict) else {}
    saved = run.get("saved") if isinstance(run.get("saved"), dict) else None
    running = bool(run.get("running"))
    document = {
        "known": bool(run.get("known")),
        "place": str(run.get("place") or "unknown"),
        "running": running,
        "waitingRelease": bool(run.get("waiting_release")),
        "mode": str(run.get("mode") or ""),
        "message": str(run.get("message", ""))[:160],
    }
    if saved:
        document.update(
            steps=",".join(f"{op}{amount}" for op, amount in saved["steps"])[:200],
            step=int(saved["step"]), speed=int(saved["speed"]), accel=int(saved["accel"]))
        if not running:
            document["done"] = int(saved["done"])
    progress = run.get("progress")
    if running and isinstance(progress, dict):
        document["runningStep"] = int(progress["step"])
    return document


def mirror_changed(previous, current):
    """Whether the mirror differs in anything but its timestamp."""
    if previous is None:
        return True
    return ({k: v for k, v in previous.items() if k != "updatedAt"}
            != {k: v for k, v in current.items() if k != "updatedAt"})


def lease_fresh(lease, now_ms):
    """Whether an app is watching the map right now."""
    if not isinstance(lease, dict) or type(lease.get("leaseAt")) is not int:
        return False
    return -FUTURE_TOLERANCE_MS <= now_ms - lease["leaseAt"] <= MAP_LEASE_MS


def distances_per_degree(frame):
    """Nearest return in each degree, cm, as 360 little-endian uint16 (0 = none).

    Index 0 is straight ahead; the index grows counter-clockwise (to the left).
    """
    nearest = np.full(360, np.inf)
    if frame is not None and len(frame):
        degree = np.rint(np.degrees(np.arctan2(frame.ys, frame.xs))).astype(int) % 360
        np.minimum.at(nearest, degree, np.hypot(frame.xs, frame.ys))
    cm = np.where(np.isfinite(nearest), np.clip(np.rint(nearest), 1, 65535), 0)
    return cm.astype("<u2").tobytes()


def encode_map_frame(frame, *, seq, scan_age_s, hallway=None, markers=(), zones=(), path="",
                     hold="", now_ms=None):
    """The ``devices/<id>/map/frame`` document, about 1.2 kB; None when unusable."""
    if (np is None or frame is None or not math.isfinite(scan_age_s)
            or not 0.0 <= scan_age_s <= MAP_MAX_SCAN_AGE_S):
        return None
    now_ms = int(time.time() * 1000) if now_ms is None else now_ms
    document = {
        "v": 1,
        "seq": int(seq) & 0xFFFFFFFF,
        "at": now_ms,
        "ageMs": int(round(scan_age_s * 1000.0)),
        "d": base64.b64encode(distances_per_degree(frame)).decode("ascii"),
        "z": ",".join(zones)[:160],
        "p": str(path)[:40],
        "hold": str(hold or "")[:80],
        # [forward cm, left cm, kind, vx cm/s, vy cm/s]; kind 1 likely person,
        # 2 moving object, 3 object standing between the walls.
        "m": [[int(round(forward * 100)), int(round(left * 100)), MARKER_CODES[kind],
               int(round(vx * 100)), int(round(vy * 100))]
              for forward, left, kind, vx, vy in list(markers)[:MAP_MAX_MARKERS]
              if kind in MARKER_CODES],
    }
    if hallway is not None:
        if hallway.left is not None:
            document["wl"] = round(hallway.left.distance_cm, 1)
        if hallway.right is not None:
            document["wr"] = round(hallway.right.distance_cm, 1)
        if hallway.heading_deg is not None:
            document["h"] = round(hallway.heading_deg, 2)
    return document


class MapPublisher:
    """Rate limit: MAP_PERIOD_S, MAP_PERIOD_HOT_S while the Pi is hot."""

    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._last = float("-inf")
        self.seq = 0

    def due(self, hot=False, now=None):
        now = self._clock() if now is None else now
        if now - self._last < (MAP_PERIOD_HOT_S if hot else MAP_PERIOD_S):
            return False
        self._last = now
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        return True
