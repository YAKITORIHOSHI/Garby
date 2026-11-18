"""Acknowledged operator actions; no hardware imports or actions at import time."""
import os
from pathlib import Path
import re
import subprocess
import tempfile
import threading
import time


class StopLatchStore:
    """A stop survives process/Pi restarts until an acknowledged explicit release."""

    def __init__(self, path=None):
        self.path = Path(path or os.environ.get("GARBY_OPERATOR_STOP_FILE")
                         or Path.home() / ".config/garby/operator-stop")

    def load(self):
        # Unreadable state must never silently clear a previously requested stop.
        try:
            self.path.stat()
            return True
        except FileNotFoundError:
            return False
        except OSError:
            return True

    def save(self, stopped):
        if stopped:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open("w") as stream:
                stream.write("Emergency stop: explicit release required\n")
                stream.flush()
                os.fsync(stream.fileno())
        else:
            self.path.unlink(missing_ok=True)
        # Ensure directory entry changes survive an orderly OS reboot.
        if os.name == "posix":
            fd = os.open(self.path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)

    @property
    def id_path(self):
        return self.path.with_name(self.path.name + ".request-id")

    def load_id_floor(self):
        try:
            raw = self.id_path.read_text().strip()
        except FileNotFoundError:
            return 0
        if not re.fullmatch(r"[0-9]{1,20}", raw):
            raise OSError("Invalid persisted operator request counter")
        return int(raw)

    def reserve_ids(self, maximum):
        """Reserve a batch before use; rollback/crash cannot reuse transmitted IDs."""
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd, temporary = tempfile.mkstemp(prefix=".operator-id-", dir=self.path.parent)
        try:
            with os.fdopen(fd, "w") as stream:
                stream.write(str(maximum) + "\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.id_path)
            if os.name == "posix":
                directory = os.open(self.path.parent, os.O_RDONLY | os.O_DIRECTORY)
                try:
                    os.fsync(directory)
                finally:
                    os.close(directory)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)


class OperatorControl:
    FRESH_S = 7.5
    ACK_S = 5.0
    QUEUE_S = 2.0
    # CONTINUE (2026-11-14) releases the stop into the route it interrupted.
    # REVERSE (2026-11-14) releases it into the way back to that route's start.
    ACTIONS = {"ESTOP", "RELEASE", "CONTINUE", "REVERSE", "SONAR_OFF", "SONAR_ON", "REBOOT"}
    RELEASING = ("RELEASE", "CONTINUE", "REVERSE")

    def __init__(self, store=None):
        self._lock = threading.RLock()
        self.store = store
        self.local_stop = bool(store and store.load())
        self._persistence_ok = True
        try:
            self._id = store.load_id_floor() if store and hasattr(store, "load_id_floor") else 0
        except OSError:
            self._id = 0
            self._persistence_ok = False
        self._reserved_id = self._id
        self._pending = None
        self._status_at = None
        self._state_at = None
        self._state = "UNKNOWN"
        self._estop = self._sonar = None
        # True only while the executor reports an interrupted route to continue.
        # Older firmware sends no RESUME field: Continue then stays unavailable.
        self._resume = None
        # The same for "Reverse to start" (REV field, firmware REVERSE-20261114 on).
        self._reverse = None
        # The controller's answer per request ID, for the phone's command relay
        # (cloud_control.py). Bounded; unknown IDs read as "no answer yet".
        self._outcomes = {}
        self._last_poll = self._last_stop_send = float("-inf")
        self._message = ""
        self._rebooting = False
        self._reboot_error = None

    def _persist(self, stopped):
        try:
            if self.store:
                self.store.save(stopped)
            self._persistence_ok = True
            return True
        except OSError:
            self._persistence_ok = False
            self._message = "Stop state storage failed; restart/release unavailable"
            return False

    @staticmethod
    def _now(now):
        return time.monotonic() if now is None else now

    def _fresh(self, stamp, now):
        return stamp is not None and 0 <= now - stamp <= self.FRESH_S

    def disconnect(self):
        with self._lock:
            self._pending = None
            self._status_at = self._state_at = None
            self._estop = self._sonar = self._resume = self._reverse = None
            self._last_poll = self._last_stop_send = float("-inf")
            # The local latch and reboot transaction survive BLE loss.

    def note_state(self, state, now=None):
        with self._lock:
            self._state, self._state_at = state, self._now(now)

    def blocks_motion(self):
        with self._lock:
            return self.local_stop or self._estop is True or self._rebooting

    def status(self, connected, now=None):
        now = self._now(now)
        with self._lock:
            if self._pending and now - self._pending["at"] > self.ACK_S:
                self._pending = None
                self._status_at = None
                self._message = "Action unconfirmed; waiting for controller status"
            supported = connected and self._fresh(self._status_at, now)
            idle = self._state == "IDLE" and self._fresh(self._state_at, now)
            free = supported and not self._pending and not self._rebooting
            stopped = self._estop if supported else None
            message = self._message
            if self._pending:
                message = f"{self._pending['action']}: awaiting executor acknowledgement"
            elif self._message.startswith(("Pi reboot failed:", "Stop state storage failed", "Request counter storage failed")):
                message = self._message
            elif self.local_stop and stopped is not True:
                message = "STOP requested locally; executor confirmation unavailable"
            elif not connected and not self._rebooting:
                message = "BLE disconnected; controller state unknown"
            elif not supported and not self._rebooting:
                message = "Waiting for operator-control firmware status"
            if self._reboot_error:
                message = f"Pi reboot failed: {self._reboot_error}; STOP remains latched"
            return {
                "supported": bool(supported), "connected": bool(connected),
                "pending": self._pending is not None or self._rebooting,
                "emergency_stopped": stopped,
                "ultrasonic_enabled": self._sonar if supported else None,
                "can_configure": bool(free and idle),
                "can_release": bool(free and idle and stopped and self._persistence_ok),
                "can_continue": bool(free and idle and stopped and self._persistence_ok
                                     and self._resume is True),
                "route_to_continue": self._resume if supported else None,
                "can_reverse": bool(free and idle and stopped and self._persistence_ok
                                    and self._reverse is True),
                "route_to_reverse": self._reverse if supported else None,
                "can_reset": bool(free and stopped and self.local_stop and self._persistence_ok),
                "message": message or "Controller settings confirmed",
            }

    def request(self, action, connected, now=None):
        now = self._now(now)
        if action not in self.ACTIONS:
            raise ValueError("Unknown operator action")
        with self._lock:
            if action == "ESTOP":
                self.local_stop = True  # Stop locally even if persistence/link fails.
                if self._rebooting:
                    return None  # A committed controller reboot must finish on the Pi too.
                self._persist(True)
            else:
                status = self.status(connected, now)
                gate = ("can_release" if action == "RELEASE" else
                        "can_continue" if action == "CONTINUE" else
                        "can_reverse" if action == "REVERSE" else
                        "can_reset" if action == "REBOOT" else "can_configure")
                if not status[gate]:
                    return None
                if action == "REBOOT":
                    self._reboot_error = None
            self._id = max(self._id + 1, int(time.time() * 1000))
            self._message = ""
            if self.store and hasattr(self.store, "reserve_ids") and self._id > self._reserved_id:
                try:
                    self.store.reserve_ids(self._id + 255)
                    self._reserved_id = self._id + 255
                except OSError:
                    self._persistence_ok = False
                    self._message = "Request counter storage failed; restart/release unavailable"
                    if action != "ESTOP":
                        return None
            self._pending = {"id": str(self._id), "action": action, "at": now, "sent": None}
            return f"[OP:{self._id}:{action}]"

    def next_packet(self, connected, now=None):
        """Dedicated priority lane: a scan flood cannot evict an emergency stop."""
        now = self._now(now)
        with self._lock:
            self.status(connected, now)
            if not connected or self._rebooting:
                return None
            if self.local_stop and self._estop is not True and self._pending is None:
                self.request("ESTOP", connected, now)
            if self._pending:
                pending = self._pending
                repeat_stop = (pending["action"] == "ESTOP"
                               and now - self._last_stop_send >= 1.0)
                if pending["sent"] is None or repeat_stop:
                    return f"[OP:{pending['id']}:{pending['action']}]"
            if now - self._last_poll >= 2.0:
                self._last_poll = now
                return "[OP STATUS]"
            return None

    def can_send(self, packet, connected, now=None):
        now = self._now(now)
        with self._lock:
            self.status(connected, now)
            if packet == "[OP STATUS]":
                return connected and not self._rebooting
            pending = self._pending
            if (not connected or not pending or self._rebooting
                    or packet != f"[OP:{pending['id']}:{pending['action']}]" ):
                return False
            if pending["action"] == "ESTOP":
                return self.local_stop
            if pending["sent"] is not None:
                return False  # Uncertain action writes are never replayed.
            valid = self._fresh(self._status_at, now) and now - pending["at"] <= self.QUEUE_S
            if pending["action"] in ("SONAR_ON", "SONAR_OFF") + self.RELEASING:
                valid = valid and self._state == "IDLE" and self._fresh(self._state_at, now)
            if pending["action"] in self.RELEASING + ("REBOOT",):
                valid = valid and self._estop is True and self._persistence_ok
            if pending["action"] == "CONTINUE":
                valid = valid and self._resume is True
            if pending["action"] == "REVERSE":
                valid = valid and self._reverse is True
            if not valid:
                self._pending = None
                self._message = "Action cancelled; check controller state and try again"
            return bool(valid)

    def mark_sent(self, packet, now=None):
        with self._lock:
            if self._pending and packet == f"[OP:{self._pending['id']}:{self._pending['action']}]":
                self._pending["sent"] = self._now(now)
                if self._pending["action"] == "ESTOP":
                    self._last_stop_send = self._now(now)

    def notification(self, packet, now=None):
        """Return REBOOT only for a matching, attempted reboot acknowledged by MCU."""
        now = self._now(now)
        with self._lock:
            self.status(True, now)  # Late callbacks cannot revive an expired action.
            match = re.fullmatch(r"\[OP STATUS:1:ESTOP=([01]):SONAR=([01])"
                                 r"(?::RESUME=([01]))?(?::REV=([01]))?\]", packet)
            if match:
                estop, sonar, resume, reverse = match.groups()
                self._estop, self._sonar = estop == "1", sonar == "1"
                self._resume = None if resume is None else resume == "1"
                self._reverse = None if reverse is None else reverse == "1"
                self._status_at = now
                if self._estop and not self.local_stop:
                    self.local_stop = True
                    self._persist(True)
                return None
            match = re.fullmatch(r"\[OP (ACK|FAILED):([0-9]{1,20}):([A-Z_]+)\]", packet)
            if not match or not self._pending:
                return None
            outcome, request_id, detail = match.groups()
            pending = self._pending
            if request_id != pending["id"] or pending["sent"] is None:
                return None
            action = pending["action"]
            if outcome == "ACK" and detail != action:
                return None
            self._pending = None
            self._outcomes[request_id] = (outcome == "ACK", detail)
            while len(self._outcomes) > 16:
                del self._outcomes[next(iter(self._outcomes))]
            if outcome == "FAILED":
                self._message = f"Action rejected: {detail}"
                return None
            self._message = f"{action}: confirmed by executor"
            if action == "ESTOP":
                self._estop = True
            elif action in self.RELEASING:
                # CONTINUE and REVERSE are the same release; the executor then
                # runs the route on, or back to its start.
                self._estop = False
                self._resume = False
                self._reverse = False
                if self._persist(False):
                    self.local_stop = False
                # If persistence fails, next_packet reapplies the stop.
            elif action.startswith("SONAR_"):
                self._sonar = action == "SONAR_ON"
            elif action == "REBOOT":
                self._rebooting = True
                self._message = "Controller restarts acknowledged; requesting Pi reboot"
                return "REBOOT"
            # ACK updates only its field; require a complete fresh status again.
            self._status_at = None
            self._last_poll = float("-inf")
            return None

    def outcome(self, request_id):
        """(accepted, detail) once the controller answered that request, else None."""
        with self._lock:
            return self._outcomes.get(str(request_id))

    def reboot_result(self, error=None):
        with self._lock:
            self._rebooting = error is None
            self._reboot_error = reboot_error_text(error) if error else None
            self._message = (f"Pi reboot failed: {self._reboot_error}; STOP remains latched"
                             if error else "Pi reboot requested; STOP remains latched")

    def reboot_allowed(self):
        with self._lock:
            return self._rebooting and self.local_stop and self._persistence_ok


# Ubuntu lets only the user at the Pi's own screen reboot without a password. From an
# ssh session (the owner's ssh -Y, 2026-11-16) systemd answers this; a one-time rule
# file (RasPi/49-garby-reboot.rules) allows the robot account to reboot.
REBOOT_NEEDS_PERMISSION = ("Interactive authentication required", "Access denied")
REBOOT_PERMISSION_HINT = (
    "the Pi does not allow a reboot from this ssh session. One-time fix: sudo install -m 644 "
    "RasPi/49-garby-reboot.rules /etc/polkit-1/rules.d/ (see DEPLOYMENT_AND_ACCEPTANCE.md). "
    "The bridge and executor restart by themselves")


def reboot_error_text(error):
    text = str(error)
    if any(marker in text for marker in REBOOT_NEEDS_PERMISSION):
        return REBOOT_PERMISSION_HINT
    return text[:100]


def reboot_pi():
    """Orderly OS reboot. No shell, forced reboot, privilege changes or password prompt."""
    result = subprocess.run(["systemctl", "--no-ask-password", "reboot"],
                            capture_output=True, text=True, timeout=10, check=False)
    if result.returncode:
        raise RuntimeError((result.stderr.strip() or result.stdout.strip()
                            or f"systemctl exited {result.returncode}")[:240])
