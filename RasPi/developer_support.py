"""Finite, acknowledged developer tests. No hardware access in this module."""
import math
import re
import threading
import time

# Nominal wheel geometry (MAIN_EXECUTOR_CODE.h: 1600 steps/rev, 65 mm wheel), unmeasured.
# Only used to make step counts readable; counters cannot prove wheel travel.
STEPS_PER_CM = 1600 / (3.14159265 * 6.5)
DEVELOPER_MIN_CM_S = 30
DEVELOPER_MIN_SPEED_HZ = math.ceil(DEVELOPER_MIN_CM_S * STEPS_PER_CM)

ACTION_NAMES = {"I": "Check sensors", "F": "Forward", "B": "Backward", "1": "Motor 1",
                "2": "Motor 2", "L": "Turn left", "R": "Turn right",
                "L90": "Turn left 90°", "R90": "Turn right 90°",
                "W": "Drive forward", "S": "Drive backward", "A": "Spin left", "D": "Spin right",
                "P": "RunStart list", "C": "Continue run", "U": "Reverse to start"}
TURN90_ACTIONS = ("L90", "R90")
TURN_ACTIONS = ("L", "R", *TURN90_ACTIONS)
TURN90_FIRMWARE = "TURNSCALE-20261116"
# Remote control (owner, 2026-11-16): the arrows drive only while held, with no step
# value, no time limit and no sensor gates; letting go sends [TEST END]. The window
# must keep saying "held" (still_held) or the KEEPs stop and the robot's 500 ms lease
# ends the drive, so a frozen window or a dropped ssh session cannot keep it going.
HOLD_ACTIONS = ("W", "S", "A", "D")
HOLD_SLOW_HZ, HOLD_FAST_HZ = DEVELOPER_MIN_SPEED_HZ, 2 * DEVELOPER_MIN_SPEED_HZ
# The phone's arrows (owner, 2026-11-18: "Tap = short nudge"). The phone reaches the Pi
# through Firebase, polled once a second, so it cannot keep saying "held". Each tap is a
# held drive the Pi itself lets go of this long after sending it: about 18 cm Slow, 36 cm
# Fast. A slow or lost internet link cannot make it longer; the KEEP lease still applies.
PHONE_NUDGE_S = 0.6
# Backward (owner, 2026-11-16) is new in this build on both boards; older boards drop it.
BACKWARD_FIRMWARE = "BACKTEST-20261116"
REMOTE_FIRMWARE = "TURNSCALE-20261116"  # both boards accept the new 30/60 cm/s settings
# RunStart (owner, 2026-11-16): the Developer window's own list of steps, e.g. forward 10000
# then turn left 980. No LiDAR, sonar or gyro check and no time limit, by the owner's choice;
# the 800 ms KEEP lease and the emergency stop still end it. The executor keeps the list
# and its exact place, so after a stop it can continue (C) or go back to its start (U).
RUNSTART_FIRMWARE = "TURNSCALE-20261116"  # 2351..8000 steps/s lists (owner, 2026-11-16)
RUN_OPS = {"F": "Forward", "B": "Backward", "L": "Turn left", "R": "Turn right",
           "C": "Bump calibrate"}
RUN_MAX_STEPS, RUN_MAX_STEP = 20, 100000
RUN_SPEED_RANGE, RUN_ACCEL_RANGE = (DEVELOPER_MIN_SPEED_HZ, 8000), (100, 20000)
RUN_DEFAULT_SPEED, RUN_DEFAULT_ACCEL = DEVELOPER_MIN_SPEED_HZ, 4800
RUN_ACTIONS = ("P", "C", "U")
_RUN_ITEM = re.compile(r"([FBLRC])([1-9][0-9]{0,5})")


def parse_run_program(text):
    """The phone's list, "F4000,L980" (2026-11-18), as [(op, steps)]; None when malformed.
    Counts and ranges are then checked by DeveloperTestControl.run_program, as for the window."""
    if not isinstance(text, str) or not text or len(text) > RUN_MAX_STEPS * 8:
        return None
    steps = []
    for item in text.split(","):
        match = _RUN_ITEM.fullmatch(item)
        if match is None:
            return None
        steps.append((match[1], int(match[2])))
    return steps

# Executor/bridge outcome codes ([A-Z_]+ on the wire) in plain words for the monitor.
FAILURE_TEXT = {
    "EMERGENCY_STOP": "stopped by the emergency stop.",
    "LEASE_EXPIRED": "stopped: the robot lost contact with the Pi (safety cut-off).",
    "TEST_TIMEOUT": "stopped at the test's time limit.",
    "HELD_LIDAR": "held by the LiDAR: it sees something too close. Untick LiDAR to test without it.",
    "HELD_REAR": ("held by the LiDAR rear check: something is behind GARBY, or the rear is not "
                  "confirmed clear. Untick LiDAR to test without it."),
    "HELD_SONAR": "held by the sonar: it sees something in front. Untick Sonar to test without it.",
    "HELD_MPU": ("held: the MPU cannot measure rotation without a heading reading. Put GARBY "
                 "on the floor with its wheels down for turns. If the MPU is still not detected, "
                 "check its connection."),
    "MPU_DATA_LOST": ("stopped: MPU readings stopped arriving, so rotation cannot be measured. "
                      "Keep GARBY on the floor with its wheels down and check the MPU connection."),
    "NO_ROTATION": ("the MPU cannot detect GARBY rotating. Put GARBY on the floor with its "
                    "wheels down. With the wheels lifted, only the wheels turn. If already "
                    "on the floor, check wheel grip and the MPU."),
    "WRONG_WAY": "stopped: the robot turned the wrong way twice.",
    "TURN_UNSETTLED": "stopped: the MPU still sees rotation after braking. Check grip and the MPU.",
    "TURN_ACCURACY": "stopped: the settled MPU angle is outside the turn's one-degree tolerance.",
    "MOTION_REJECTED": "the motor controller stopped the move.",
    "SPEED_REJECTED": "the motor controller refused the test speed.",
    "NOT_IDLE": "the robot was busy. Wait until it is idle and try again.",
    "NOT_READY": "the robot was not ready yet. Wait a few seconds and try again.",
    "ROUTE_FAULT": "the robot has a route fault. Press Reset on the Controls tab first.",
    "MOTORS_UNAVAILABLE": "the motor drivers are not available.",
    "REBOOT_PENDING": "a restart is pending.",
    "RESET_REQUESTED": "stopped: a reset was requested.",
    "UART_FAULT": "stopped: communication error between the two boards.",
    "UART_BUSY": "the bridge could not pass the test on. Try again.",
    "SESSION_CHANGED": "stopped: the robot connection restarted.",
    "LINK_OR_STOP": "stopped: the link to the robot dropped.",
    "INVALID_TEST": "the robot refused the request. Try again.",
    "INVALID_RUN": "the robot refused the list: a step, speed or acceleration is out of range.",
    "NO_RUN": ("the robot has no saved list to continue: it restarted or something else moved "
               "it. Start the list again."),
    "ID_REUSED": "the robot refused the request. Try again.",
    "STALE_REQUEST": "the robot refused the request. Try again.",
}


def failure_text(code):
    return FAILURE_TEXT.get(code, "stopped (" + code.replace("_", " ").lower() + ").")


class DeveloperTestControl:
    # The bridge asks for robot status every 5 s; 12 s rides out one late reply, so
    # the buttons do not grey out between polls. The executor and bridge re-check
    # readiness on every request, so this freshness only affects the panel.
    FRESH_S = 12.0
    QUEUE_S = 2.0
    ACK_S = 5.0
    LIMIT_S = 8.5
    TURN90_LIMIT_S = 30.5  # includes readiness waits, braking and fresh MPU settling
    KEEP_S = 0.2
    HOLD_KEEP_S = 0.12
    HOLD_ALIVE_S = 0.5
    ACTIONS = {"I": 0, "F": 400, "B": 400, "1": 100, "2": 100,
               "L": 10, "R": 10, "L90": 90, "R90": 90}
    _ID = r"[1-9][0-9]{0,15}"

    def __init__(self, store=None, clock=None, wall_clock=None):
        self._lock = threading.RLock()
        self.store = store
        self._clock = clock or time.monotonic
        self._wall_clock = wall_clock or time.time
        self._ready_at = self._data_at = None
        self._ever_ready = False
        self._pending = None
        self._message = ""
        self._result = ""
        self._sensors = None
        # RunStart: the list the executor reports saved ({} parsed, None = nothing saved,
        # False = not reported yet), the running list's place, a Discard to send, and a
        # Continue/Reverse waiting for the emergency stop to be released.
        self._run_saved = False
        self._run_at = None
        self._run_clear = None
        self._after_release = None
        self._run_message = ""  # the last RunStart outcome, apart from the bench tests'
        self._storage_ok = True
        try:
            self._id = store.load_id_floor() if store else 0
        except OSError:
            self._id = 0
            self._storage_ok = False
        self._reserved = self._id

    def _fresh(self, stamp):
        return stamp is not None and 0 <= self._clock() - stamp <= self.FRESH_S

    def _name(self):
        pending = self._pending
        return f"“{ACTION_NAMES.get(pending['action'], 'Test')}”" if pending else "The test"

    def stopped_by_emergency(self):
        """The emergency stop was pressed: end the pending test (its lease lapses at once)."""
        with self._lock:
            if self._pending:
                run = self._pending.get("run")
                self.cancel(f"{self._name()} stopped by the emergency stop." +
                            (" Continue run or Reverse to start when you are ready." if run else ""))
            self._after_release = None

    def _say_run(self, message):
        self._message = self._run_message = message

    def cancel(self, message):
        with self._lock:
            if self._pending and self._pending.get("run"):
                self._run_message = message
            self._pending = None
            self._ready_at = None
            self._message = message

    def disconnect(self):
        with self._lock:
            if self._pending:
                self.cancel(f"{self._name()} stopped: the robot connection changed.")
            else:
                self._ready_at = None
            self._ever_ready = False
            self._data_at = None
            self._sensors = None
            self._run_saved = False
            self._run_at = None
            self._run_clear = None
            self._after_release = None

    def run_active(self):
        with self._lock:
            return bool(self._pending and self._pending.get("run"))

    def busy(self):
        with self._lock:
            self._expire()
            return self._pending is not None

    def _expire(self):
        pending = self._pending
        if pending is None:
            return
        now = self._clock()
        name = self._name()
        if pending["sent"] is None and now - pending["at"] > self.QUEUE_S:
            self.cancel(f"{name} could not be sent in time. Try again.")
        elif pending.get("end_sent_at") is not None and now - pending["end_sent_at"] > self.ACK_S:
            self.cancel(f"{name}: the robot did not confirm it stopped. A held drive also "
                        "stops by itself half a second after the Pi goes quiet.")
        elif pending["sent"] is not None:
            elapsed = now - pending["sent"]
            if not pending["acked"] and elapsed > self.ACK_S:
                hint = (f" Backward needs the {BACKWARD_FIRMWARE} firmware on both boards."
                        if pending["action"] == "B" else
                        f" RunStart needs the {RUNSTART_FIRMWARE} firmware on both boards."
                        if pending.get("run") else
                        f" The 90° turns need the {TURN90_FIRMWARE} firmware on both boards."
                        if pending["action"] in TURN90_ACTIONS else
                        f" The remote control needs the {REMOTE_FIRMWARE} firmware on both boards."
                        if pending["hold"] else "")
                self.cancel(f"{name}: the robot did not confirm it; any movement was stopped "
                            "automatically. Try again." + hint)
            elif elapsed > (self.TURN90_LIMIT_S if pending["action"] in TURN90_ACTIONS else self.LIMIT_S) \
                    and not pending.get("hold") and not pending.get("run"):
                self.cancel(f"{name} passed its time limit and was stopped.")

    def _blocked(self, connected, eligible, supported, blocked):
        """Plain reason the buttons are unavailable, or "" when a test may start."""
        if self._pending is not None or self._after_release is not None:
            return ""
        if not self._storage_ok:
            return "The Pi cannot save test numbers (storage error). Restart GARBY."
        if not connected:
            return "Robot not connected. Wait for the Bluetooth link."
        if not eligible:
            return blocked or "The robot is busy. Wait until it is idle."
        if not supported:
            if self._ever_ready:
                return "Waiting for the robot to confirm it is ready (a few seconds)…"
            return ("Waiting for the robot to report test support. If this stays, upload the "
                    "DEVTEST firmware to both boards.")
        return ""

    def status(self, connected, eligible, blocked=""):
        with self._lock:
            self._expire()
            supported = connected and self._fresh(self._ready_at)
            sensors = self._sensors if connected and self._fresh(self._data_at) else None
            pending = self._pending
            why = self._blocked(connected, eligible, supported, blocked)
            waiting = self._after_release is not None
            return {
                "enabled": bool(pending is None and not waiting and not why),
                "supported": bool(supported), "pending": pending is not None or waiting,
                "active": bool(pending and pending["sent"] is not None),
                "blocked": why,
                "message": self._message,
                "sensors": dict(sensors) if sensors else {}, "result": self._result,
                "run": self._run_status(connected, pending, waiting),
            }

    def _run_status(self, connected, pending, waiting):
        """What the RunStart tab shows: the saved list and where the running one is."""
        saved = self._run_saved if connected else False
        running = bool(pending and pending.get("run"))
        place = "unknown" if saved is False else "none" if saved is None else saved["place"]
        return {
            "known": saved is not False,
            "saved": dict(saved, steps=list(saved["steps"])) if saved else None,
            "place": place,
            "running": running, "waiting_release": waiting,
            "mode": pending["action"] if running else None,
            "progress": dict(self._run_at) if running and self._run_at
                        and self._run_at["id"] == pending["id"] else None,
            # Rows stay locked while a list moves or stands part-way: Continue and
            # Reverse act on exactly the steps shown.
            "locked": running or waiting or place == "partway",
            "message": self._run_message,
        }

    def request(self, action, sensors, connected, eligible):
        with self._lock:
            if action not in self.ACTIONS or type(sensors) is not int or not 0 <= sensors <= 7:
                return None
            if action in TURN_ACTIONS and not sensors & 2:
                self._message = "Turns need the gyro (MPU) ticked."
                return None
            if not self.status(connected, eligible)["enabled"] or not self._next_id():
                return None
            wire_action = action[0] if action in TURN90_ACTIONS else action
            packet = f"[TEST:{self._id}:{wire_action}:{self.ACTIONS[action]}:{sensors}]"
            return self._begin(packet, action)

    def _next_id(self):
        self._id = max(self._id + 1, int(self._wall_clock() * 1000))
        if self._id >= 10**16:
            self._storage_ok = False
            return False
        if self.store and self._id > self._reserved:
            try:
                self.store.reserve_ids(self._id + 255)
                self._reserved = self._id + 255
            except OSError:
                self._storage_ok = False
                return False
        return True

    def _begin(self, packet, action, hold=False, run=False, pulse_s=None):
        self._pending = {"id": str(self._id), "packet": packet, "at": self._clock(),
                         "sent": None, "acked": False, "keep_at": float("-inf"),
                         "action": action, "hold": hold, "held_at": self._clock(),
                         "ending": False, "end_sent_at": None, "run": run,
                         "pulse_s": pulse_s}
        self._message = f"Sending {self._name()} to the robot…"
        if run:
            self._run_message = self._message
        self._result = ""
        return packet

    @staticmethod
    def run_program(steps, speed, accel):
        """("", wire text) for a valid RunStart list, else (plain reason, "")."""
        if not steps:
            return "Add at least one step.", ""
        if len(steps) > RUN_MAX_STEPS:
            return f"At most {RUN_MAX_STEPS} steps.", ""
        items = []
        for number, (op, amount) in enumerate(steps, start=1):
            if op not in RUN_OPS:
                return f"Step {number}: choose what it does.", ""
            if type(amount) is not int or not 1 <= amount <= RUN_MAX_STEP:
                return f"Step {number}: steps must be a whole number from 1 to {RUN_MAX_STEP}.", ""
            items.append(f"{op}{amount}")
        low, high = RUN_SPEED_RANGE
        if type(speed) is not int or not low <= speed <= high:
            return f"Max speed must be a whole number from {low} to {high} steps/s.", ""
        low, high = RUN_ACCEL_RANGE
        if type(accel) is not int or not low <= accel <= high:
            return f"Acceleration must be a whole number from {low} to {high} steps/s².", ""
        return "", ",".join(items)

    def request_run(self, steps, speed, accel, connected, eligible):
        """Start a RunStart list: [TEST:<id>:P:<speed>:<accel>:<op><steps>,...]."""
        with self._lock:
            why, program = self.run_program(steps, speed, accel)
            if why:
                self._say_run(why)
                return None
            if not self.status(connected, eligible)["enabled"] or not self._next_id():
                return None
            self._run_at = None
            return self._begin(f"[TEST:{self._id}:P:{speed}:{accel}:{program}]", "P", run=True)

    def _resume_packet(self, mode):
        saved = self._run_saved
        if mode not in ("C", "U") or not saved or not self._next_id():
            return None
        self._run_at = None
        return self._begin(f"[TEST:{self._id}:{mode}:{saved['id']}]", mode, run=True)

    def request_run_resume(self, mode, connected, eligible):
        """Continue run (C) or Reverse to start (U) of the saved list, from where it is."""
        with self._lock:
            saved = self._run_saved
            if not saved or (mode == "C" and saved["place"] == "end") or (mode == "U" and saved["place"] == "start"):
                self._say_run("Nothing to " + ("continue." if mode == "C" else "reverse."))
                return None
            if not self.status(connected, eligible)["enabled"]:
                return None
            return self._resume_packet(mode)

    def queue_run_after_release(self, mode, seconds=10.0):
        """Continue/Reverse pressed while the emergency stop is on: the stop is released
        first (the same confirmed release as the main window), then the list moves."""
        with self._lock:
            if self._pending is None and self._run_saved and mode in ("C", "U"):
                self._after_release = {"mode": mode, "until": self._clock() + seconds}
                self._say_run("Releasing the emergency stop, then " + (
                    "continuing the list…" if mode == "C" else "reversing to the start…"))

    def cancel_after_release(self, message):
        with self._lock:
            if self._after_release is not None:
                self._after_release = None
                self._say_run(message)

    def request_run_clear(self, connected):
        """Discard: the executor forgets the saved list ([RUN CLEAR:<id>]). Never while one runs."""
        with self._lock:
            saved = self._run_saved
            if not connected or not saved or (self._pending and self._pending.get("run")):
                return False
            self._run_clear = {"id": saved["id"], "sent": None}
            self._say_run("Forgetting the saved list…")
            return True

    def request_hold(self, action, fast, connected, eligible, pulse_s=None):
        """Remote control: one held arrow. No sensor gates (mask 0), Slow or Fast.

        With pulse_s (the phone's nudge) nobody holds the arrow: the Pi lets go by itself
        pulse_s after sending the start, exactly as if the arrow were released then."""
        with self._lock:
            if action not in HOLD_ACTIONS:
                return None
            if pulse_s is not None and not (isinstance(pulse_s, (int, float))
                                            and 0 < pulse_s <= 2 * PHONE_NUDGE_S):
                return None
            if not self.status(connected, eligible)["enabled"] or not self._next_id():
                return None
            speed = HOLD_FAST_HZ if fast else HOLD_SLOW_HZ
            return self._begin(f"[TEST:{self._id}:{action}:{speed}:0]", action, hold=True,
                               pulse_s=pulse_s)

    def still_held(self):
        """The window still shows the arrow pressed; called every 100 ms while held."""
        with self._lock:
            pending = self._pending
            if pending and pending["hold"] and not pending["ending"]:
                pending["held_at"] = self._clock()

    def release(self):
        """The arrow was let go: stop now ([TEST END]), or never send an unsent start."""
        with self._lock:
            pending = self._pending
            if not pending or not pending["hold"] or pending["ending"]:
                return
            if pending["sent"] is None:
                self.cancel(f"{self._name()}: let go before it started; nothing moved.")
                return
            pending["ending"] = True
            self._message = f"{self._name()}: stopping…"

    def stop(self, who):
        """End whatever runs now, as asked by `who` (e.g. "from the phone"). A held drive or
        nudge gets [TEST END] at once; a bench test or list loses its KEEP lease, so the robot
        brakes within 800 ms; a Continue/Reverse waiting for the release is dropped.
        Returns False when nothing was running."""
        with self._lock:
            self._expire()
            pending = self._pending
            if pending is not None and pending["hold"]:
                self.release()
                return True
            if pending is not None:
                run = pending.get("run")
                self.cancel(f"{self._name()} stopped {who}." +
                            (" Continue run or Reverse to start picks it up from there." if run else ""))
                return True
            if self._after_release is not None:
                self.cancel_after_release(f"Continue/Reverse cancelled {who}.")
                return True
            return False

    def next_packet(self, connected, eligible, stopped=False):
        with self._lock:
            self._expire()
            clear = self._run_clear
            if connected and clear and clear["sent"] is None:
                return f"[RUN CLEAR:{clear['id']}]"  # forgets only; allowed while stopped
            waiting = self._after_release
            if waiting is not None:
                if self._clock() > waiting["until"]:
                    self._after_release = None
                    self._say_run("Continue/Reverse was not sent: the emergency stop was not "
                                  "released in time. Try again.")
                elif connected and not stopped and eligible and self._fresh(self._ready_at):
                    self._after_release = None
                    packet = self._resume_packet(waiting["mode"])
                    if packet and self.can_send(packet, connected, eligible):
                        return packet
            if stopped or not connected:
                if self._pending:
                    self.cancel(f"{self._name()} stopped: emergency stop or lost connection.")
                return None
            pending = self._pending
            if pending is None:
                return None
            if pending["sent"] is None:
                return pending["packet"] if self.can_send(pending["packet"], connected, eligible) else None
            if pending["hold"]:
                if pending["ending"]:
                    return None if pending["end_sent_at"] is not None else f"[TEST END:{pending['id']}]"
                if pending.get("pulse_s") is not None:
                    if self._clock() - pending["sent"] >= pending["pulse_s"]:
                        # The phone's nudge is over: let go, exactly as a released arrow.
                        pending["ending"] = True
                        self._message = f"{self._name()}: nudge over, stopping…"
                        return f"[TEST END:{pending['id']}]"
                elif self._clock() - pending["held_at"] > self.HOLD_ALIVE_S:
                    return None  # the window stopped saying "held": let the lease end it
                if self._clock() - pending["keep_at"] >= self.HOLD_KEEP_S:
                    return f"[TEST KEEP:{pending['id']}]"
                return None
            if self._clock() - pending["keep_at"] >= self.KEEP_S:
                return f"[TEST KEEP:{pending['id']}]"
            return None

    def can_send(self, packet, connected, eligible, stopped=False):
        with self._lock:
            self._expire()
            pending = self._pending
            if (pending is not None and pending["hold"] and pending["ending"]
                    and packet == f"[TEST END:{pending['id']}]"):
                return pending["end_sent_at"] is None  # a stop: never held back
            clear = self._run_clear
            if packet.startswith("[RUN CLEAR:"):
                return bool(connected and clear and clear["sent"] is None
                            and packet == f"[RUN CLEAR:{clear['id']}]")
            if stopped or not connected or pending is None:
                return False
            if pending["hold"] and pending["ending"]:
                return False
            if packet == pending["packet"]:
                return pending["sent"] is None and eligible and self._fresh(self._ready_at)
            return pending["sent"] is not None and packet == f"[TEST KEEP:{pending['id']}]"

    def mark_sent(self, packet):
        """Call before the GATT attempt: an uncertain start must never replay."""
        with self._lock:
            clear = self._run_clear
            if clear and packet == f"[RUN CLEAR:{clear['id']}]":
                clear["sent"] = self._clock()
            pending = self._pending
            if pending is None:
                return
            if packet == pending["packet"] and pending["sent"] is None:
                pending["sent"] = self._clock()
                self._message = f"Waiting for the robot to start {self._name()}…"
            elif packet == f"[TEST KEEP:{pending['id']}]":
                pending["keep_at"] = self._clock()
            elif packet == f"[TEST END:{pending['id']}]":
                pending["end_sent_at"] = self._clock()

    def _format_result(self, action, d1, d2, measured, yaw_cdeg):
        def wheel(name, steps):
            return f"{name} {steps:+d} steps (≈{steps / STEPS_PER_CM:+.1f} cm)"
        gyro = f"gyro turned {yaw_cdeg / 100:+.2f}°" if measured else "gyro: not measured"
        if action == "I":
            return "No movement. " + gyro[0].upper() + gyro[1:] + "."
        return f"{wheel('Motor 1', d1)}, {wheel('motor 2', d2)}; {gyro}."

    def receive(self, line):
        with self._lock:
            self._expire()
            if line == "[TEST READY:1]":
                self._ready_at = self._clock()
                self._ever_ready = True
                return True
            data = re.fullmatch(r"\[TEST DATA:M=([01]):Y=(-?[0-9]{1,10}):W=([01]):H=(-?[0-9]{1,6})\]", line)
            if data:
                mpu, yaw, walls, wall = map(int, data.groups())
                if not -(2**31) <= yaw <= 2**31 - 1 or abs(wall) > 18000:
                    return True
                self._sensors = {"mpu_ready": bool(mpu), "yaw_deg": yaw / 100,
                                 "walls_ready": bool(walls), "wall_deg": wall / 100}
                self._data_at = self._clock()
                return True
            if line.startswith("[RUN "):
                return self._receive_run(line)
            result = re.fullmatch(rf"\[TEST RESULT:({self._ID}):D1=(-?[0-9]{{1,10}}):D2=(-?[0-9]{{1,10}}):M=([01]):Y=(-?[0-9]{{1,10}})\]", line)
            outcome = re.fullmatch(rf"\[TEST (ACK|DONE|FAILED):({self._ID})(?::([A-Z_]+))?\]", line)
            if not result and not outcome:
                return line.startswith("[TEST ")
            pending = self._pending
            identity = result[1] if result else outcome[2]
            if pending is None or pending["sent"] is None or identity != pending["id"]:
                return True
            name = self._name()
            if result:
                if any(not -(2**31) <= int(result[index]) <= 2**31 - 1 for index in (2, 3, 5)):
                    return True
                self._result = self._format_result(pending["action"], int(result[2]), int(result[3]),
                                                   result[4] == "1", int(result[5]))
            elif outcome[1] == "ACK" and outcome[3] is None:
                pending["acked"] = True
                self._message = f"{name} running — EMERGENCY STOP stops it at once."
                if pending.get("run"):
                    self._run_message = self._message
            elif outcome[1] == "DONE" and outcome[3] is None:
                self.cancel(f"{name} stopped." if pending["hold"] else
                            ("Reverse to start finished: back where the list began." if pending["action"] == "U"
                             else "The list finished. Reverse to start can take it back.")
                            if pending.get("run") else
                            f"{name} finished. Normal safety checks are back on.")
            elif outcome[1] == "FAILED" and outcome[3]:
                resumable = pending.get("run") and outcome[3] not in ("INVALID_RUN", "NO_RUN", "NOT_IDLE",
                                                                      "NOT_READY", "ID_REUSED", "STALE_REQUEST")
                self.cancel(f"{name}: {failure_text(outcome[3])}" +
                            (" Continue run or Reverse to start picks it up from there." if resumable else ""))
            return True

    _RUN_SAVED = re.compile(r"\[RUN SAVED:(?:0|([1-9][0-9]{0,15}):([0-9]{1,2}):([0-9]{1,6}):([0-9]{1,4}):"
                            r"([0-9]{1,5}):([FBLRC][1-9][0-9]{0,5}(?:,[FBLRC][1-9][0-9]{0,5}){0,19}))\]")

    def _receive_run(self, line):
        at = re.fullmatch(rf"\[RUN AT:({self._ID}):([0-9]{{1,2}}):([0-9]{{1,6}})\]", line)
        if at:
            self._run_at = {"id": at[1], "step": int(at[2]), "done": int(at[3])}
            return True
        saved = self._RUN_SAVED.fullmatch(line)
        if not saved:
            return True
        clear = self._run_clear
        if saved[1] is None:
            self._run_saved = None
            if clear and clear["sent"] is not None:
                self._run_clear = None
                self._say_run("Saved list forgotten. You can edit the steps again.")
            return True
        steps = [(item[0], int(item[1:])) for item in saved[6].split(",")]
        step, done = int(saved[2]), int(saved[3])
        if step >= len(steps) or done > steps[step][1]:
            return True
        place = ("start" if step == 0 and done == 0 else
                 "end" if step == len(steps) - 1 and done == steps[step][1] else "partway")
        self._run_saved = {"id": saved[1], "step": step, "done": done, "speed": int(saved[4]),
                           "accel": int(saved[5]), "steps": steps, "place": place}
        if clear and clear["sent"] is not None and clear["id"] != saved[1]:
            self._run_clear = None
        return True
