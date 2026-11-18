"""Small, hardware-independent Tk dashboard for the GARBY bridge.

Uses system fonts, flat white panels and a slate/blue palette; the LiDAR map is
a dark radar scope. Color reinforces written states; it never substitutes for
them. No image, theme or chart package is needed. Operator actions use runtime
callbacks; the controller owns motion.
"""

import math
import logging
import queue
import threading
import time
from datetime import datetime
from itertools import islice
import tkinter as tk
from tkinter import font as tkfont
from tkinter import ttk, messagebox

from bridge_core import SENSOR_SPECS
from developer_support import (DeveloperTestControl, RUN_ACCEL_RANGE, RUN_DEFAULT_ACCEL,
                               RUN_DEFAULT_SPEED, RUN_MAX_STEP, RUN_MAX_STEPS, RUN_OPS,
                               RUN_SPEED_RANGE, TURN_ACTIONS)
from lidar_map import MAX_MAP_POINTS

logger = logging.getLogger(__name__)


COLORS = {
    "background": "#f3f4f6", "surface": "#ffffff", "text": "#1f2937",
    "muted": "#526173", "border": "#d1d5db", "accent": "#2457b2",
    "ok": "#166534", "warning": "#92400e", "danger": "#b42318",
}
# The LiDAR map is a radar scope: dark, green, readable at arm's length.
RADAR = {
    "canvas": "#07130c", "scope": "#0b2415", "scope_off": "#121a16",
    "ring": "#1f5636", "ring_label": "#62b088", "spoke": "#17432b",
    "text": "#d4f7e2", "muted": "#8aa597", "point": "#3df58a", "nearest": "#ffd166",
    "sweep": "#3dff8f", "robot": "#e8fff1", "ok": "#62e89f", "warning": "#ffc24d",
    "danger": "#ff6f61", "threshold": "#8a3a34",
    # Hallway markers (scene_objects.scene_markers): likely person, moving object,
    # object standing between the walls.
    "person": "#ff2d2d", "moving": "#ffb020", "object": "#ff8a3d", "marker_edge": "#ffffff",
}
# Robot frame, degrees counter-clockwise from FRONT (bridge_core sectors).
# Canvas anchors keep each label outside the scope on its own side.
DIRECTIONS = (
    ("FRONT", "Front", 0.0, "s"), ("FRONT_LEFT", "Front left", 45.0, "se"),
    ("LEFT", "Left", 90.0, "e"), ("BACK_LEFT", "Rear left", 135.0, "ne"),
    ("BACK", "Rear", 180.0, "n"), ("BACK_RIGHT", "Rear right", 225.0, "nw"),
    ("RIGHT", "Right", 270.0, "w"), ("FRONT_RIGHT", "Front right", 315.0, "sw"),
)
SECTOR_WIDTH_DEG = 45.0
MAX_SCAN_POINTS = MAX_MAP_POINTS
SWEEP_WEDGES = 18
SWEEP_WEDGE_DEG = 5.0
SWEEP_PERIOD_S = 2.4
SWEEP_FRAME_MS = 50
MAX_MAP_MARKERS = 16
MARKER_KINDS = ("person", "moving", "object")
MARKER_LABELS = {"person": ("likely person", "likely people"),
                 "moving": ("moving object", "moving objects"),
                 "object": ("object", "objects")}
MARKER_VELOCITY_S = 1.0          # the direction line shows this much travel
LIDAR_UNAVAILABLE_AFTER_S = 45.0
_NO_SOURCE_TIME = object()


def scope_offset(angle_deg, distance_px):
    """Robot-frame angle (CCW from front) to a canvas offset: front up, left left."""
    angle = math.radians(angle_deg)
    return -math.sin(angle) * distance_px, -math.cos(angle) * distance_px


def canvas_arc_start(angle_deg):
    """Tk arcs start counter-clockwise from 3 o'clock; robot front is 12 o'clock."""
    return (90.0 + angle_deg - SECTOR_WIDTH_DEG / 2.0) % 360.0


def _mix(color_a, color_b, fraction):
    a = [int(color_a[i:i + 2], 16) for i in (1, 3, 5)]
    b = [int(color_b[i:i + 2], 16) for i in (1, 3, 5)]
    return "#" + "".join(f"{round(x + (y - x) * fraction):02x}" for x, y in zip(a, b))


def sweep_colors(count=SWEEP_WEDGES):
    """A soft glow behind the bright beam line, fading into the scope.

    The trail starts well below the beam's brightness so text drawn over the
    scope stays readable while the beam passes under it.
    """
    span = max(1, count - 1)
    return tuple(_mix(RADAR["sweep"], RADAR["scope"], 0.5 + 0.5 * (index / span) ** 0.8)
                 for index in range(count))


def sector_label(name, reading, *, live, single_line=False):
    """Text and palette key for one sector label on the map.

    The written word carries the state ("close", "read only"); color only
    reinforces it. Without a live scan every distance is withheld.
    """
    if not live:
        value, tone = "--", "muted"
    else:
        value, tone = reading["text"], reading["tone"]
        if reading["detail"] == "Close obstacle":
            value += " · close"
        elif reading["detail"] == "Reading only":
            value += " · read only"
        if tone == "text":
            tone = "ok"
    return (f"{name}  {value}" if single_line else f"{name}\n{value}"), tone


class LidarAvailability:
    """Decide what the LiDAR map shows: live, loading, attention or unavailable.

    loading (the radar sweep) covers finding and starting the LiDAR and short
    stream gaps. unavailable needs evidence: a failure the driver supervisor
    reported, or no scan for timeout_s. A failure stays shown while the
    supervisor retries, until a live scan clears it. attention is a fresh scan
    the runtime has flagged unusable. Display only: never path evidence.
    """

    _RETRY_TEXT = {"searching": "Retrying: looking for the LiDAR",
                   "starting": "Retrying: starting the LiDAR driver",
                   "running": "Driver restarted; waiting for its first scan"}

    def __init__(self, timeout_s=LIDAR_UNAVAILABLE_AFTER_S):
        self.timeout_s = float(timeout_s)
        self._waiting_since = None
        self._failure = None
        self._ever_live = False

    def update(self, now, *, scan_recent, stream_ok, driver=None):
        """Return (mode, title, detail) for this poll."""
        driver = driver if isinstance(driver, dict) else {}
        phase = driver.get("phase") if isinstance(driver.get("phase"), str) else None
        if scan_recent and stream_ok:
            self._waiting_since, self._failure, self._ever_live = None, None, True
            return "live", "Live scan", ""
        if phase == "failed":
            self._failure = str(driver.get("message") or "The LiDAR driver failed")
        if scan_recent:
            # The LiDAR is sending, but the runtime flagged the scan unusable.
            return ("attention", "Scan needs attention",
                    "Too few valid returns in the latest scan; motion is held")
        if self._waiting_since is None or not math.isfinite(now - self._waiting_since):
            self._waiting_since = now
        waited = max(0.0, now - self._waiting_since)
        if self._failure is not None:
            retry = self._RETRY_TEXT.get(phase, "Retrying automatically")
            return "unavailable", "LiDAR not available", f"{self._failure}\n{retry}"
        if waited >= self.timeout_s:
            what = "No LiDAR scan for" if self._ever_live else "No LiDAR scan received in"
            return ("unavailable", "LiDAR not available",
                    f"{what} {waited:.0f} s\nCheck its USB cable and power")
        if phase == "searching":
            title = "Searching for LiDAR"
        elif phase == "starting":
            title = "Starting LiDAR"
        else:
            title = "Reconnecting LiDAR" if self._ever_live else "Waiting for first scan"
        return "loading", title, f"{waited:.0f} s"


def scan_display_points(snapshot, now, stale_after_s):
    """Return bounded real hits only while their acquisition is still current."""
    if snapshot is None:
        return ()
    try:
        age = now - snapshot.acquired_at
        if not math.isfinite(age) or not 0 <= age <= stale_after_s:
            return ()
        if not math.isfinite(snapshot.yaw_offset_deg):
            return ()
        return tuple((forward, left) for forward, left in islice(snapshot.points_m, MAX_SCAN_POINTS)
                     if math.isfinite(forward) and math.isfinite(left))
    except (AttributeError, TypeError, ValueError):
        return ()


def scan_display_markers(snapshot, now, stale_after_s):
    """Valid markers of a current acquisition: (forward, left, kind, vx, vy)."""
    if snapshot is None:
        return ()
    try:
        age = now - snapshot.acquired_at
        if not math.isfinite(age) or not 0 <= age <= stale_after_s:
            return ()
        markers = []
        for marker in islice(getattr(snapshot, "markers", ()) or (), MAX_MAP_MARKERS):
            forward, left, kind, vx, vy = marker
            if kind in MARKER_KINDS and all(
                    isinstance(value, (int, float)) and math.isfinite(value)
                    for value in (forward, left, vx, vy)):
                markers.append((float(forward), float(left), kind, float(vx), float(vy)))
        return tuple(markers)
    except (AttributeError, TypeError, ValueError):
        return ()


def marker_summary(markers):
    """"1 likely person · 2 objects" for the map's info line ("" when none)."""
    parts = []
    for kind in MARKER_KINDS:
        count = sum(1 for marker in markers if marker[2] == kind)
        if count:
            parts.append(f"{count} {MARKER_LABELS[kind][count > 1]}")
    return " · ".join(parts)


def scan_canvas_point(forward_m, left_m, center_x, center_y, pixels_per_m):
    """Robot frame: +forward goes up and +left goes left on the display."""
    return (center_x - left_m * pixels_per_m, center_y - forward_m * pixels_per_m)


# Hold codes in plain words (owner, 2026-11-16: holds were not understood). The code
# stays in the text for the log and for debugging; these only add where to look.
HOLD_WORDS = {
    "LEFT_BAND": "something is right beside GARBY's left side",
    "RIGHT_BAND": "something is right beside GARBY's right side",
    "LEFT_BAND_REAR": "something is right beside GARBY's left side, rear half",
    "RIGHT_BAND_REAR": "something is right beside GARBY's right side, rear half",
    "LEFT": "something is close on GARBY's left",
    "RIGHT": "something is close on GARBY's right",
    "SIDE": "something is close beside GARBY",
    "FRONT_BOX": "something is in front of GARBY, in its path",
    "FRONT": "something is in front of GARBY",
    "FRONT_LEFT": "something is in front of GARBY, to the left",
    "FRONT_RIGHT": "something is in front of GARBY, to the right",
    "REAR_BOX": "something is behind GARBY, close to its rear",
    "BACK": "something is behind GARBY",
    "BACK_LEFT": "something is behind GARBY, to the left",
    "BACK_RIGHT": "something is behind GARBY, to the right",
    "MOVING": "something is moving toward GARBY",
}


def hold_in_words(hold):
    """'Side clearance blocked (RIGHT_BAND)' plus where to look, when the code is known."""
    hold = str(hold or "")
    start, end = hold.rfind("("), hold.rfind(")")
    if start < 0 or end < start:
        return hold
    words = [HOLD_WORDS[code] for code in hold[start + 1:end].split(",") if code in HOLD_WORDS]
    return hold + (": " + "; ".join(words) if words else "")


ROUTE_STATES = ("OUTBOUND", "RETURNING", "REVERSING")


def _layout_without_focus(elements):
    """Keep a component's rendering while unwrapping focus-only elements."""
    layout = []
    for name, options in elements:
        options = dict(options)
        if "children" in options:
            options["children"] = _layout_without_focus(options["children"])
        if name.rsplit(".", 1)[-1] == "focus":
            layout.extend(options.get("children", ()))
        else:
            layout.append((name, options))
    return layout


def direction_reading(info, threshold_cm, *, bypassed=False, stale=False):
    """Format a displayed distance without interpreting it as drive permission."""
    if stale:
        return {"text": "-- cm", "detail": "Scan stale", "tone": "warning"}
    if info is None:
        return {"text": "-- cm", "detail": "No reading", "tone": "muted"}
    distance_cm = float(info[0]) * 100
    if not math.isfinite(distance_cm) or distance_cm < 0:
        return {"text": "-- cm", "detail": "No reading", "tone": "muted"}
    if bypassed:
        detail, tone = "Reading only", "warning"
    elif distance_cm <= threshold_cm:
        detail, tone = "Close obstacle", "danger"
    else:
        detail, tone = "Distance", "text"
    # Whole centimetres: the X3's datasheet gives 2 cm systematic error within
    # 1 m and 1% beyond, so a tenth of a centimetre would overstate accuracy.
    # The threshold comparison above still uses the unrounded distance.
    return {"text": f"{distance_cm:.0f} cm", "detail": detail, "tone": tone}


def sensor_reading(key, value):
    """Transport sentinels are unavailable data, not actual sensor values."""
    value = SENSOR_SPECS[key].normalize(value)
    if value is None:
        return {"text": "--", "status": "Unavailable", "tone": "muted"}
    text = f"{value:.1f} cm" if key == "ultrasonic" else str(value)
    return {"text": text, "status": "Reading", "tone": "text"}



def wall_fit_text(sides_status):
    """Hallway wall line fit of the current scan, as sent to the bridge.

    WL/WR are perpendicular wall distances from the chassis centerline and H is
    the robot's heading relative to the walls (+ = rotated left). Offset is the
    lateral position those two walls imply (+ = left of centre). Z lists the
    chassis zones holding a return (hallway_geometry.py), which the eight
    sector cards cannot show.
    """
    fields = {}
    zones = ""
    for part in str(sides_status or "").replace("SIDES:", "", 1).split("|"):
        key, separator, value = part.partition("=")
        if separator and key == "Z":
            zones = "   |   Zones: " + value.replace(",", ", ").replace("_", " ").lower()
        if separator and key in ("WL", "WR", "H"):
            try:
                fields[key] = float(value)
            except ValueError:
                pass
    left, right, heading = fields.get("WL"), fields.get("WR"), fields.get("H")
    if left is None and right is None:
        return "Walls: no line fit" + zones
    text = "Walls"
    if left is not None:
        text += f" L {left:.0f}"
    if right is not None:
        text += f" R {right:.0f}"
    text += " cm"
    if left is not None and right is not None:
        text += f", offset {(right - left) / 2.0:+.0f}"
    if heading is not None:
        text += f", heading {heading:+.1f} deg"
    return text + zones

class LidarGUI:
    POLL_MS = 150
    LOG_LIMIT = 250
    FRONT_GROUP = {"FRONT", "FRONT_LEFT", "FRONT_RIGHT"}
    BACK_GROUP = {"BACK", "BACK_LEFT", "BACK_RIGHT"}

    def __init__(self, root, data_queue, recv_queue, *, read_sensors=None,
                 read_health=None, front_disabled=None, back_disabled=None, side_disabled=None,
                 allow_safety_disable=False, front_threshold_cm=95.0,
                 front_diagonal_threshold_cm=60.0,
                 back_threshold_cm=35.0, side_threshold_cm=22.0,
                 stale_after_s=0.8, on_checks_changed=None, request_start=None,
                 request_emergency_stop=None, request_release_stop=None,
                 request_ultrasonic=None, request_system_reset=None,
                 request_continue_run=None, request_reverse_run=None,
                 request_developer_test=None, request_developer_cancel=None,
                 request_hold=None, hold_alive=None, release_hold=None,
                 request_run=None, request_run_resume=None, request_run_clear=None):
        self.root, self.data_queue, self.recv_queue = root, data_queue, recv_queue
        self.read_sensors = read_sensors or (lambda: {})
        self.read_health = read_health or (lambda: {})
        self.front_disabled = front_disabled or threading.Event()
        self.back_disabled = back_disabled or threading.Event()
        self.side_disabled = side_disabled or threading.Event()
        self.allow_safety_disable = allow_safety_disable
        self.on_checks_changed = on_checks_changed or (lambda: None)
        self.request_start = request_start
        self.request_emergency_stop = request_emergency_stop
        self.request_release_stop = request_release_stop
        self.request_continue_run = request_continue_run
        self.request_reverse_run = request_reverse_run
        self.request_developer_test = request_developer_test
        self.request_developer_cancel = request_developer_cancel
        self.request_hold = request_hold
        self.hold_alive = hold_alive
        self.release_hold = release_hold
        self.request_run = request_run
        self.request_run_resume = request_run_resume
        self.request_run_clear = request_run_clear
        self._run_rows = []
        self._run_loaded_id = None
        self._run_note = ""
        self._holding = None
        self._hold_after_id = None
        self._log_handler = None
        self._log_sequence = 0
        self.request_ultrasonic = request_ultrasonic
        self.request_system_reset = request_system_reset
        self.front_threshold_cm, self.back_threshold_cm = front_threshold_cm, back_threshold_cm
        self.front_diagonal_threshold_cm = front_diagonal_threshold_cm
        self.side_threshold_cm, self.stale_after_s = side_threshold_cm, stale_after_s
        self._ble_status_lock = threading.Lock()
        self._pending_ble_status = None
        self._last_frame = None
        self._last_update_at = None
        self._display_stale = None
        self._current_health = {}
        self._widget_values = {}
        self._messages_visible = False
        self._after_id = None
        self._closed = False
        self._refresh_failed = False
        self._scan_snapshot = None
        self._init_map_state()

        root.report_callback_exception = self._report_callback_exception
        root.title("GARBY - Robot monitor")
        root.configure(bg=COLORS["background"])
        width = min(1024, root.winfo_screenwidth())
        height = min(600, max(480, root.winfo_screenheight() - 60))
        root.geometry(f"{width}x{height}")
        root.minsize(800, 480)
        root.bind("<F11>", self.toggle_fullscreen)
        root.bind("<Escape>", self._leave_fullscreen)
        root.bind("<Destroy>", self._on_destroy, add="+")

        self.font = tkfont.nametofont("TkDefaultFont").copy()
        self.font.configure(size=10)
        self.small_font = self.font.copy()
        self.small_font.configure(size=9)
        self.title_font = self.font.copy()
        self.title_font.configure(size=16, weight="bold")
        self.heading_font = self.font.copy()
        self.heading_font.configure(weight="bold")
        self.map_label_font = self.small_font.copy()
        self.map_label_font.configure(weight="bold")
        self.map_title_font = self.font.copy()
        self.map_title_font.configure(size=13, weight="bold")
        style = ttk.Style(root)
        # The built-in clam theme is available with Tk and behaves consistently
        # on the Pi; it does not load a downloaded theme or font.
        if "clam" in style.theme_names():
            style.theme_use("clam")
        style.configure("Monitor.TButton", font=self.small_font, padding=(10, 7))
        style.configure("Compact.Monitor.TButton", font=self.small_font, padding=(6, 2))
        style.configure("Remote.Monitor.TButton", font=self.heading_font, padding=(14, 10),
                        justify="center", anchor="center")
        for button_style in ("Monitor.TButton", "Compact.Monitor.TButton",
                             "Emergency.Monitor.TButton", "Remote.Monitor.TButton"):
            style.layout(button_style, _layout_without_focus(style.layout("TButton")))

        root.grid_columnconfigure(0, weight=1)
        root.grid_rowconfigure(1, weight=1)
        header = tk.Frame(root, bg=COLORS["background"])
        header.grid(row=0, column=0, sticky="ew", padx=16, pady=(10, 8))
        header.grid_columnconfigure(1, weight=1)
        self._label(header, "GARBY", font=self.title_font,
                    bg=COLORS["background"]).grid(row=0, column=0)
        self.header_subtitle_lbl = self._label(header, "Waiting for both controllers", tone="muted",
                                        bg=COLORS["background"], font=self.small_font,
                                        wraplength=160, width=24, height=2, justify="left")
        self.header_subtitle_lbl.grid(row=0, column=1, sticky="w", padx=(12, 8))
        self.header_subtitle_lbl.grid_remove()
        style.configure("Emergency.Monitor.TButton", font=self.heading_font,
                        padding=(12, 9), foreground="#ffffff", background=COLORS["danger"])
        style.map("Emergency.Monitor.TButton", background=[("active", "#8f1d15")],
                  foreground=[("disabled", "#dedede")])
        style.configure("Monitor.TNotebook.Tab", font=self.font, padding=(14, 7))
        style.layout("Monitor.TNotebook.Tab",
                     _layout_without_focus(style.layout("TNotebook.Tab")))
        self.emergency_btn = ttk.Button(header, text="EMERGENCY STOP",
                                        style="Emergency.Monitor.TButton",
                                        command=self._request_emergency_stop,
                                        state="normal" if request_emergency_stop else "disabled")
        self.emergency_btn.grid(row=0, column=2, padx=8)
        # Developer tests are bench checks in their own window (owner, 2026-11-16, two
        # windows over ssh -Y); this window keeps only the robot's real controls.
        self.developer_open_btn = ttk.Button(header, text="Developer",
                                             style="Monitor.TButton",
                                             command=self.open_developer_tests)
        self.developer_open_btn.grid(row=0, column=3, padx=(0, 8))
        ttk.Button(header, text="Full screen", style="Monitor.TButton",
                   command=self.toggle_fullscreen).grid(row=0, column=4)
        self.header_motion_hold_lbl = self._label(
            header, "", tone="warning", font=self.small_font,
            bg=COLORS["background"], wraplength=740, justify="left")
        self.header_motion_hold_lbl.grid(row=1, column=0, columnspan=5, sticky="w", pady=(4, 0))
        self.header_motion_hold_lbl.grid_remove()

        self.notebook = ttk.Notebook(root, style="Monitor.TNotebook", takefocus=False)
        self.notebook.grid(row=1, column=0, sticky="nsew", padx=12)
        self.lidar_tab = tk.Frame(self.notebook, bg=COLORS["background"])
        self.ble_tab = tk.Frame(self.notebook, bg=COLORS["background"])
        self.controls_tab = tk.Frame(self.notebook, bg=COLORS["background"])
        self.log_tab = tk.Frame(self.notebook, bg=COLORS["background"])
        self.notebook.add(self.lidar_tab, text="LiDAR readings")
        self.notebook.add(self.ble_tab, text="BLE status & messages")
        self.notebook.add(self.controls_tab, text="Controls")
        self.notebook.add(self.log_tab, text="Log")
        self._build_developer_window()
        self.notebook.bind("<<NotebookTabChanged>>", self._tab_changed)
        body = self.lidar_tab
        body.grid_columnconfigure(0, weight=1)
        body.grid_columnconfigure(1, minsize=230)
        body.grid_rowconfigure(0, weight=1)
        surroundings = self._panel(body)
        surroundings.grid(row=0, column=0, sticky="nsew", padx=(0, 10), pady=6)
        surroundings.grid_columnconfigure(0, weight=1)
        surroundings.grid_rowconfigure(0, weight=1)

        # One live LiDAR map replaces the eight number cards: the eight sector
        # distances are written around the scope, each on its own side.
        self.scan_canvas = tk.Canvas(surroundings, bg=RADAR["canvas"],
                                     highlightthickness=0, width=100, height=100)
        self.scan_canvas.grid(row=0, column=0, sticky="nsew", padx=6, pady=6)
        self.scan_canvas.bind("<Configure>", self._resize_scan)
        self._build_map_items()
        # Map notices and metadata live with the bin sensors. Scrolling this
        # fixed-width sidebar keeps long holds readable on the Pi's 800x480 screen
        # without taking height away from the map or expanding the window.
        self.sensor_sidebar = self._panel(body)
        self.sensor_sidebar.grid(row=0, column=1, sticky="nsew", pady=6)
        self.sensor_sidebar.grid_columnconfigure(0, weight=1)
        self.sensor_sidebar.grid_rowconfigure(0, weight=1)
        self.sensor_canvas = tk.Canvas(self.sensor_sidebar, width=212, height=100,
                                      bg=COLORS["surface"], highlightthickness=0)
        self.sensor_canvas.grid(row=0, column=0, sticky="nsew")
        sensor_scroll = ttk.Scrollbar(self.sensor_sidebar, command=self.sensor_canvas.yview)
        sensor_scroll.grid(row=0, column=1, sticky="ns")
        self.sensor_canvas.configure(yscrollcommand=sensor_scroll.set)
        sensor_panel = tk.Frame(self.sensor_canvas, bg=COLORS["surface"])
        self.sensor_panel = sensor_panel
        self._sensor_window = self.sensor_canvas.create_window(0, 0, window=sensor_panel, anchor="nw")
        sensor_panel.bind("<Configure>", lambda event: self.sensor_canvas.configure(
            scrollregion=self.sensor_canvas.bbox("all")))
        self.sensor_canvas.bind("<Configure>", lambda event: self.sensor_canvas.itemconfigure(
            self._sensor_window, width=event.width))
        sensor_panel.grid_columnconfigure(0, weight=1)
        self.ble_tab.grid_columnconfigure(0, minsize=230)
        self.ble_tab.grid_columnconfigure(1, weight=1)
        self.ble_tab.grid_rowconfigure(0, weight=1)
        sidebar = self._panel(self.ble_tab)
        sidebar.grid(row=0, column=0, sticky="nsew", padx=(0, 10), pady=6)
        sidebar.grid_columnconfigure(0, weight=1)
        self._label(sidebar, "Connections", font=self.heading_font).grid(
            row=0, column=0, sticky="w", padx=12, pady=(10, 8))
        self.connection_labels = {}
        for row, (key, title) in enumerate((("ble", "Bluetooth"), ("sensors", "Sensor link"),
                                           ("firebase", "Cloud telemetry")), start=1):
            line = tk.Frame(sidebar, bg=COLORS["surface"])
            line.grid(row=row, column=0, sticky="ew", padx=12, pady=(0, 3))
            caption = self._label(line, title, font=self.small_font, tone="muted")
            label = self._label(line, "Starting", font=self.small_font, tone="warning",
                                wraplength=200, justify="left")
            if key == "ble":
                caption.pack(anchor="w")
                # Reserve two lines so connection retries do not shift sensors.
                label.configure(height=2, anchor="nw")
                label.pack(anchor="w", pady=(2, 0))
            else:
                caption.pack(side="left")
                label.pack(side="right", padx=(8, 0))
            self.connection_labels[key] = label
        self.ble_status_lbl = self.connection_labels["ble"]
        self.subtitle_lbl = self._label(sensor_panel, "Waiting for both controllers", tone="muted",
                                        font=self.small_font, wraplength=188, justify="left")
        self.subtitle_lbl.grid(row=0, column=0, sticky="w", padx=10, pady=(10, 2))
        self.motion_hold_lbl = self._label(sensor_panel, "", tone="warning", font=self.small_font,
                                           wraplength=188, justify="left")
        self.motion_hold_lbl.grid(row=1, column=0, sticky="w", padx=10, pady=(2, 4))
        self.motion_hold_lbl.grid_remove()
        self.status_lbl = self._label(sensor_panel, "Waiting for scan", tone="warning", font=self.small_font,
                                      wraplength=188, justify="left")
        self.status_lbl.grid(row=2, column=0, sticky="w", padx=10, pady=(0, 6))
        self._label(sensor_panel, "Bin sensors", font=self.heading_font).grid(
            row=3, column=0, sticky="w", padx=10, pady=(4, 6))
        sensor_grid = tk.Frame(sensor_panel, bg=COLORS["surface"])
        sensor_grid.grid(row=4, column=0, sticky="nsew", padx=10)
        self.sensor_labels = {}
        self.sensor_status_labels = {}
        for row, (key, name) in enumerate((("ultrasonic", "Bin distance"), ("mq4", "Methane (MQ4)"),
                                          ("mq137", "Ammonia (MQ137)"), ("mq135", "Air quality (MQ135)"))):
            self._label(sensor_grid, name, font=self.small_font, tone="muted").grid(
                row=row * 2, column=0, sticky="w")
            value = self._label(sensor_grid, "--", font=self.heading_font)
            value.grid(row=row * 2, column=1, sticky="e", padx=(8, 0))
            state = self._label(sensor_grid, "Unavailable", font=self.small_font, tone="muted")
            state.grid(row=row * 2 + 1, column=0, columnspan=2, sticky="w")
            self.sensor_labels[key] = value
            self.sensor_status_labels[key] = state
        sensor_grid.grid_columnconfigure(0, weight=1)

        details = tk.Frame(sensor_panel, bg=COLORS["surface"])
        details.grid(row=5, column=0, sticky="ew", padx=10, pady=(12, 4))
        self._label(details, "Surroundings", font=self.heading_font).pack(anchor="w")
        self.map_range_btn = ttk.Button(details, text="Range: 4 m",
                                        style="Compact.Monitor.TButton", command=self._cycle_map_range)
        self.map_range_btn.pack(anchor="w", pady=(4, 6))
        self.safety_lbl = self._label(details, "Threshold checks on", tone="ok", font=self.small_font,
                                      wraplength=188, justify="left")
        self.safety_lbl.pack(anchor="w")
        self.map_caption_lbl = self._label(details, "Robot centred · front up · metres",
                                           tone="muted", font=self.small_font, wraplength=188, justify="left")
        self.map_caption_lbl.pack(anchor="w", pady=(6, 2))
        self.map_info_lbl = self._label(details, "No position history or localization",
                                        tone="muted", font=self.small_font, wraplength=188, justify="left")
        self.map_info_lbl.pack(anchor="w")
        self._label(sensor_panel, f"LiDAR stop distances (red arcs)\nFront {front_threshold_cm:g} / "
                    f"front diagonals {front_diagonal_threshold_cm:g} / rear "
                    f"{back_threshold_cm:g} / sides {side_threshold_cm:g} cm\n"
                    "plus chassis zones below",
                    font=self.small_font, tone="muted", wraplength=188, justify="left").grid(
                        row=6, column=0, sticky="w", padx=10, pady=(8, 4))

        self.log_frame = self._panel(self.ble_tab)
        self.log_frame.grid(row=0, column=1, sticky="nsew", pady=6)
        self.log_frame.grid_columnconfigure(0, weight=1)
        self.log_frame.grid_rowconfigure(1, weight=1)
        log_header = tk.Frame(self.log_frame, bg=COLORS["surface"])
        log_header.grid(row=0, column=0, columnspan=2, sticky="ew", padx=10, pady=(6, 2))
        self._label(log_header, "Recent Bluetooth messages", font=self.small_font,
                    tone="muted").pack(side="left")
        self.clear_logs_btn = ttk.Button(log_header, text="Clear logs", style="Monitor.TButton",
                                          command=self._clear_ble_log, takefocus=True)
        self.clear_logs_btn.pack(side="right", padx=(8, 0))
        self.log_text = tk.Text(self.log_frame, height=4, width=1, wrap="word",
                                bg=COLORS["surface"], fg=COLORS["text"], font=self.small_font,
                                relief="flat", highlightthickness=0, padx=10, pady=4,
                                state="disabled")
        self.log_text.grid(row=1, column=0, sticky="nsew")
        scrollbar = ttk.Scrollbar(self.log_frame, command=self.log_text.yview)
        scrollbar.grid(row=1, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=scrollbar.set)
        self._build_controls()
        self._answer_unavailable_clicks()
        self._build_developer_controls()
        self._build_remote_controls()
        self._build_runstart_controls()
        self._build_log_tab()

        self.footer = self._label(sensor_panel, "Waiting for LiDAR data", tone="muted",
                                  font=self.small_font, wraplength=188, justify="left")
        self.footer.grid(row=7, column=0, sticky="w", padx=10, pady=(4, 10))
        self._poll()

    def _build_developer_window(self):
        window = self.developer_window = tk.Toplevel(self.root)
        window.withdraw()
        window.title("GARBY - Developer tests (bench)")
        window.configure(bg=COLORS["background"])
        window.geometry("840x500")
        window.minsize(800, 440)
        window.protocol("WM_DELETE_WINDOW", self._close_developer_tests)
        window.grid_columnconfigure(0, weight=1)
        window.grid_rowconfigure(1, weight=1)
        top = tk.Frame(window, bg=COLORS["background"])
        top.grid(row=0, column=0, sticky="ew", padx=16, pady=(10, 4))
        top.grid_columnconfigure(1, weight=1)
        self._label(top, "Developer tests", font=self.title_font,
                    bg=COLORS["background"]).grid(row=0, column=0, sticky="w")
        self._label(top, "Short bench checks, one at a time. GARBY's real controls are in the "
                    "main window.", tone="muted", font=self.small_font, bg=COLORS["background"],
                    wraplength=360, justify="left").grid(row=0, column=1, sticky="w", padx=12)
        # A test moves the wheels: the stop must be on this window too.
        self.developer_emergency_btn = ttk.Button(
            top, text="EMERGENCY STOP", style="Emergency.Monitor.TButton",
            command=self._request_emergency_stop,
            state="normal" if self.request_emergency_stop else "disabled")
        self.developer_emergency_btn.grid(row=0, column=2, sticky="e")
        # Two tabs: the finite bench tests, and the remote control (owner, 2026-11-16).
        self.developer_notebook = ttk.Notebook(window, style="Monitor.TNotebook", takefocus=False)
        self.developer_notebook.grid(row=1, column=0, sticky="nsew", padx=12, pady=(0, 8))
        self.developer_tab = tk.Frame(self.developer_notebook, bg=COLORS["background"])
        self.remote_tab = tk.Frame(self.developer_notebook, bg=COLORS["background"])
        # RunStart (owner, 2026-11-16): a list of steps to drive in order.
        self.runstart_tab = tk.Frame(self.developer_notebook, bg=COLORS["background"])
        self.developer_notebook.add(self.developer_tab, text="Bench tests")
        self.developer_notebook.add(self.remote_tab, text="Remote control")
        self.developer_notebook.add(self.runstart_tab, text="RunStart")
        # Minimising the window must let go of a held arrow.
        window.bind("<Unmap>", lambda event: self._hold_end() if event.widget is window else None)

    def open_developer_tests(self):
        self.developer_window.deiconify()
        self.developer_window.lift()
        self._refresh_developer_controls()

    def _close_developer_tests(self):
        # Closing the window while a test runs stops it: the Pi stops renewing its
        # lease and the executor brakes within 800 ms. Otherwise it only hides.
        if self._holding:
            self._hold_end()  # a held arrow is let go, which stops the drive at once
            self.developer_window.withdraw()
            return
        status = dict(self.read_health()).get("developer_tests", {})
        if status.get("pending") and self.request_developer_cancel:
            self.request_developer_cancel()
        self.developer_window.withdraw()

    # ── remote control (owner, 2026-11-16) ──────────────────────────────────────
    HOLD_TICK_MS = 100
    REMOTE_WORDS = {"W": "forward", "S": "backward", "A": "spinning left", "D": "spinning right"}

    def _build_remote_controls(self):
        page = self.remote_tab
        page.grid_columnconfigure(0, weight=1)
        panel = self._panel(page)
        panel.grid(row=0, column=0, sticky="nsew", pady=6)
        panel.grid_columnconfigure(1, weight=1)
        self._label(panel, "Hold an arrow to drive. Let go to stop.", font=self.heading_font).grid(
            row=0, column=0, columnspan=2, sticky="w", padx=12, pady=(10, 4))
        pad = tk.Frame(panel, bg=COLORS["surface"])
        pad.grid(row=1, column=0, rowspan=3, sticky="nw", padx=12, pady=4)
        self.remote_buttons = {}
        for action, text, row, column in (("W", "\u25b2\nForward", 0, 1), ("A", "\u25c0 Left", 1, 0),
                                          ("D", "Right \u25b6", 1, 2), ("S", "Backward\n\u25bc", 2, 1)):
            # No command: a click would be a tap. Press starts, release stops.
            button = ttk.Button(pad, text=text, style="Remote.Monitor.TButton", width=9)
            button.grid(row=row, column=column, padx=3, pady=3, sticky="nsew")
            button.bind("<ButtonPress-1>", lambda event, a=action: self._hold_start(a))
            button.bind("<ButtonRelease-1>", lambda event: self._hold_end())
            self.remote_buttons[action] = button
        speed = tk.Frame(panel, bg=COLORS["surface"])
        speed.grid(row=1, column=1, sticky="nw", padx=12, pady=(6, 2))
        self._label(speed, "Speed", font=self.small_font, tone="muted").pack(anchor="w")
        self.remote_fast_var = tk.BooleanVar(value=False)
        for value, text in ((False, "Slow (about 30 cm/s)"), (True, "Fast (about 60 cm/s)")):
            ttk.Radiobutton(speed, text=text, variable=self.remote_fast_var,
                            value=value).pack(anchor="w")
        self._label(panel, "No sensor checks while driving: GARBY goes where you point it, so "
                    "watch it. Letting go stops it at once; EMERGENCY STOP above always works. "
                    "Left and Right spin in place.", font=self.small_font, tone="muted",
                    wraplength=380, justify="left").grid(row=2, column=1, sticky="nw", padx=12, pady=4)
        self.remote_status_lbl = self._label(panel, "Not driving.", font=self.small_font,
                                             wraplength=380, justify="left")
        self.remote_status_lbl.grid(row=3, column=1, sticky="nw", padx=12, pady=(4, 10))

    def _hold_start(self, action):
        if self._holding:
            return
        self._current_health = dict(self.read_health())
        status = self._current_health.get("developer_tests", {})
        if not (self.request_hold and status.get("enabled")):
            reason = status.get("blocked") or status.get("message") or "the robot is not ready."
            self._set(self.remote_status_lbl, text=f"Not driving: {reason}", fg=COLORS["warning"])
            return
        fast = bool(self.remote_fast_var.get())
        self._holding = action
        result = self.request_hold(action, fast)
        if isinstance(result, dict):
            self._current_health["developer_tests"] = result
        self._set(self.remote_status_lbl, fg=COLORS["ok"],
                  text=f"Driving {self.REMOTE_WORDS[action]} ({'Fast' if fast else 'Slow'}). "
                       "Let go to stop.")
        self._hold_after_id = self.root.after(self.HOLD_TICK_MS, self._hold_tick)

    def _hold_tick(self):
        # The Pi renews the drive only while this keeps arriving: a frozen window or a
        # dropped ssh session stops the KEEPs, and the robot's 500 ms lease ends it.
        self._hold_after_id = None
        if not self._holding:
            return
        if self.hold_alive:
            self.hold_alive()
        self._hold_after_id = self.root.after(self.HOLD_TICK_MS, self._hold_tick)

    def _hold_end(self):
        if not self._holding:
            return
        self._holding = None
        if self._hold_after_id is not None:
            self.root.after_cancel(self._hold_after_id)
            self._hold_after_id = None
        if self.release_hold:
            self.release_hold()
        self._set(self.remote_status_lbl, text="Let go: stopping.", fg=COLORS["muted"])

    def _refresh_remote_controls(self):
        status = self._current_health.get("developer_tests", {})
        enabled = bool(self.request_hold and status.get("enabled"))
        for action, button in self.remote_buttons.items():
            # A greyed arrow still answers a press with the reason; the held one stays live.
            self._set(button, state="normal" if enabled or action == self._holding else "disabled")

    # ── RunStart (owner, 2026-11-16) ────────────────────────────────────────────
    # A list of steps GARBY drives in order: "+" adds a row, the bin deletes it. While a
    # list runs, or stands stopped part-way, the rows are locked: Continue run and Reverse
    # to start act on exactly what is shown. No sensor checks, by the owner's choice.
    RUN_OP_ORDER = ("F", "B", "L", "R", "C")

    def _trash_icon(self):
        """A small bin drawn in pixels: the Pi's Tk fonts may have no emoji."""
        icon = tk.PhotoImage(width=16, height=16)
        ink = COLORS["danger"]
        for box in ((6, 1, 10, 2), (2, 3, 14, 5), (3, 5, 4, 15), (12, 5, 13, 15), (3, 14, 13, 15),
                    (6, 7, 7, 13), (9, 7, 10, 13)):
            icon.put(ink, to=box)
        return icon

    def _build_runstart_controls(self):
        page = self.runstart_tab
        page.grid_columnconfigure(0, weight=1)
        page.grid_rowconfigure(0, weight=1)
        panel = self._panel(page)
        panel.grid(row=0, column=0, sticky="nsew", pady=6)
        panel.grid_columnconfigure(0, weight=1)
        panel.grid_rowconfigure(1, weight=1)
        self._label(panel, "Steps, driven in order", font=self.heading_font).grid(
            row=0, column=0, sticky="w", padx=12, pady=(8, 2))
        self._trash_image = self._trash_icon()
        holder = tk.Frame(panel, bg=COLORS["surface"])
        holder.grid(row=1, column=0, sticky="nsew", padx=(12, 6), pady=2)
        holder.grid_columnconfigure(0, weight=1)
        holder.grid_rowconfigure(0, weight=1)
        canvas = tk.Canvas(holder, bg=COLORS["surface"], highlightthickness=0, width=440, height=190)
        scrollbar = ttk.Scrollbar(holder, orient="vertical", command=canvas.yview)
        self.run_rows_frame = tk.Frame(canvas, bg=COLORS["surface"])
        self.run_rows_frame.bind("<Configure>",
                                 lambda event: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=self.run_rows_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")
        for widget in (canvas, self.run_rows_frame):
            widget.bind("<Button-4>", lambda event: canvas.yview_scroll(-1, "units"))
            widget.bind("<Button-5>", lambda event: canvas.yview_scroll(1, "units"))
        self.run_canvas = canvas
        self.run_add_btn = ttk.Button(holder, text="+  Add step", style="Compact.Monitor.TButton",
                                      command=self._run_add_row)
        self.run_add_btn.grid(row=1, column=0, sticky="w", pady=(4, 0))

        side = tk.Frame(panel, bg=COLORS["surface"])
        side.grid(row=0, column=1, rowspan=2, sticky="nw", padx=(6, 12), pady=(8, 2))
        self._label(side, "Max speed (steps/s)", font=self.small_font, tone="muted").grid(
            row=0, column=0, sticky="w")
        self.run_speed_var = tk.StringVar(value=str(RUN_DEFAULT_SPEED))
        self.run_speed_entry = ttk.Entry(side, textvariable=self.run_speed_var, width=8,
                                         font=self.small_font)
        self.run_speed_entry.grid(row=1, column=0, sticky="w", pady=(0, 4))
        self._label(side, "Acceleration (steps/s\u00b2)", font=self.small_font, tone="muted").grid(
            row=2, column=0, sticky="w")
        self.run_accel_var = tk.StringVar(value=str(RUN_DEFAULT_ACCEL))
        self.run_accel_entry = ttk.Entry(side, textvariable=self.run_accel_var, width=8,
                                         font=self.small_font)
        self.run_accel_entry.grid(row=3, column=0, sticky="w")
        self._label(side, f"Speed {RUN_SPEED_RANGE[0]}\u2013{RUN_SPEED_RANGE[1]}, acceleration "
                    f"{RUN_ACCEL_RANGE[0]}\u2013{RUN_ACCEL_RANGE[1]}. Minimum setting: about 30 cm/s; "
                    "ramps and turn braking can be slower.", font=self.small_font, tone="muted", wraplength=270,
                    justify="left").grid(row=4, column=0, sticky="w", pady=(2, 6))
        actions = tk.Frame(side, bg=COLORS["surface"])
        actions.grid(row=5, column=0, sticky="w")
        self.run_start_btn = ttk.Button(actions, text="Start run", style="Monitor.TButton",
                                        command=self._request_run_start, state="disabled")
        self.run_start_btn.grid(row=0, column=0, sticky="ew", padx=(0, 4), pady=2)
        self.run_continue_btn = ttk.Button(actions, text="Continue run", style="Monitor.TButton",
                                           command=lambda: self._request_run_resume("C"),
                                           state="disabled")
        self.run_continue_btn.grid(row=0, column=1, sticky="ew", pady=2)
        self.run_reverse_btn = ttk.Button(actions, text="Reverse to start", style="Monitor.TButton",
                                          command=lambda: self._request_run_resume("U"),
                                          state="disabled")
        self.run_reverse_btn.grid(row=1, column=0, sticky="ew", padx=(0, 4), pady=2)
        self.run_clear_btn = ttk.Button(actions, text="Discard saved", style="Monitor.TButton",
                                        command=self._request_run_clear, state="disabled")
        self.run_clear_btn.grid(row=1, column=1, sticky="ew", pady=2)
        self.run_status_lbl = self._label(panel, "No list run yet.", font=self.small_font,
                                          wraplength=760, justify="left")
        self.run_status_lbl.grid(row=2, column=0, columnspan=2, sticky="w", padx=12, pady=(2, 0))
        self._label(panel, "No LiDAR, sonar or gyro checks: GARBY drives exactly these steps, "
                    "so keep its way clear. EMERGENCY STOP (top right) stops it at once; then "
                    "Continue run finishes the list or Reverse to start takes it back. Turns spin "
                    "in place; Bump calibrate goes forward in 100-step bumps.",
                    font=self.small_font, tone="muted", wraplength=760, justify="left").grid(
                        row=3, column=0, columnspan=2, sticky="w", padx=12, pady=(2, 8))
        self._run_add_row()

    def _run_add_row(self, op="F", steps=""):
        if len(self._run_rows) >= RUN_MAX_STEPS:
            return
        frame = tk.Frame(self.run_rows_frame, bg=COLORS["surface"])
        number = self._label(frame, "", font=self.small_font, tone="muted", width=3)
        op_var = tk.StringVar(value=RUN_OPS[op])
        combo = ttk.Combobox(frame, textvariable=op_var, state="readonly", width=14,
                             values=[RUN_OPS[code] for code in self.RUN_OP_ORDER], font=self.small_font)
        steps_var = tk.StringVar(value=str(steps))
        entry = ttk.Entry(frame, textvariable=steps_var, width=8, font=self.small_font)
        unit = self._label(frame, "steps", font=self.small_font, tone="muted")
        row = {"frame": frame, "number": number, "op": op_var, "combo": combo,
               "steps": steps_var, "entry": entry}
        row["trash"] = ttk.Button(frame, image=self._trash_image, style="Compact.Monitor.TButton",
                                  command=lambda: self._run_delete_row(row))
        row["mark"] = self._label(frame, "", font=self.small_font, tone="muted", width=18)
        for widget in (number, combo, entry, unit, row["trash"], row["mark"]):
            widget.pack(side="left", padx=(0, 4))
        self._run_rows.append(row)
        self._run_layout_rows()

    def _run_delete_row(self, row):
        run = self._current_health.get("developer_tests", {}).get("run") or {}
        if run.get("locked") or len(self._run_rows) <= 1 or row not in self._run_rows:
            return
        self._run_rows.remove(row)
        row["frame"].destroy()
        self._run_layout_rows()

    def _run_layout_rows(self):
        for index, row in enumerate(self._run_rows):
            row["frame"].grid(row=index, column=0, sticky="w", pady=1)
            self._set(row["number"], text=f"{index + 1}.")
        self._refresh_runstart_controls()

    def _run_collect(self):
        """("", (steps, speed, accel)) from the rows, or (plain reason, None)."""
        codes = {name: code for code, name in RUN_OPS.items()}
        steps = []
        for index, row in enumerate(self._run_rows, start=1):
            text = row["steps"].get().strip()
            if not text.isdigit():
                return f"Step {index}: enter the number of steps (1\u2013{RUN_MAX_STEP}).", None
            steps.append((codes.get(row["op"].get(), ""), int(text)))
        values = []
        for name, text in (("Max speed", self.run_speed_var.get()), ("Acceleration", self.run_accel_var.get())):
            if not text.strip().isdigit():
                return f"{name}: enter a whole number.", None
            values.append(int(text.strip()))
        why, _program = DeveloperTestControl.run_program(steps, values[0], values[1])
        return (why, None) if why else ("", (steps, values[0], values[1]))

    def _run_matches(self, saved):
        shown = [(row["op"].get(), row["steps"].get().strip()) for row in self._run_rows]
        return (shown == [(RUN_OPS[op], str(n)) for op, n in saved["steps"]] and
                self.run_speed_var.get().strip() == str(saved["speed"]) and
                self.run_accel_var.get().strip() == str(saved["accel"]))

    def _run_load(self, saved):
        """Show the robot's saved list (after a monitor restart): Continue acts on it."""
        self._run_loaded_id = saved["id"]  # first: adding rows refreshes this tab
        if not self._run_matches(saved):
            for row in self._run_rows:
                row["frame"].destroy()
            self._run_rows = []
            for op, amount in saved["steps"]:
                self._run_add_row(op, amount)
            self.run_speed_var.set(str(saved["speed"]))
            self.run_accel_var.set(str(saved["accel"]))

    def _request_run_start(self):
        self._current_health = dict(self.read_health())
        status = self._current_health.get("developer_tests", {})
        why, values = self._run_collect()
        if not why and not (self.request_run and status.get("enabled")):
            why = "Not available: " + (status.get("blocked") or status.get("message") or
                                       "the robot is not ready.")
        if why:
            self._run_note = why
            self._refresh_runstart_controls()
            return
        steps, speed, accel = values
        listing = "\n".join(f"{number}. {RUN_OPS[op]} {amount} steps"
                            for number, (op, amount) in enumerate(steps, start=1))
        confirmed = messagebox.askyesno(
            "Start this list?",
            f"{listing}\n\nMax speed {speed} steps/s, acceleration {accel} steps/s\u00b2.\n\n"
            "No LiDAR, sonar or gyro checks: GARBY drives exactly these steps. Keep its way "
            "clear. EMERGENCY STOP stops it at once.", parent=self.developer_window,
            default=messagebox.NO)
        self._current_health = dict(self.read_health())
        self._run_note = ""
        if confirmed and self._current_health.get("developer_tests", {}).get("enabled"):
            result = self.request_run(steps, speed, accel)
            if isinstance(result, dict):
                self._current_health["developer_tests"] = result
        self._refresh_runstart_controls()

    def _request_run_resume(self, mode):
        self._current_health = dict(self.read_health())
        run = self._current_health.get("developer_tests", {}).get("run") or {}
        saved = run.get("saved")
        if not (self.request_run_resume and saved):
            return
        stopped = (self._current_health.get("operator_controls") or {}).get("emergency_stopped") is True
        where = f"step {saved['step'] + 1} of {len(saved['steps'])}"
        confirmed = messagebox.askyesno(
            "Continue run?" if mode == "C" else "Reverse to start?",
            ("Finish the list from where it stopped" if mode == "C" else
             "Drive the list back to where it started") + f" ({where})." +
            ("\n\nThis releases the emergency stop." if stopped else "") +
            "\n\nNo LiDAR, sonar or gyro checks. EMERGENCY STOP stops it at once.",
            parent=self.developer_window, default=messagebox.NO)
        if confirmed:
            result = self.request_run_resume(mode)
            if isinstance(result, dict):
                self._current_health["developer_tests"] = result
        self._run_note = ""
        self._refresh_runstart_controls()

    def _request_run_clear(self):
        if not self.request_run_clear:
            return
        confirmed = messagebox.askyesno(
            "Discard the saved list?",
            "GARBY forgets where the list stopped, so Continue run and Reverse to start are no "
            "longer possible for it. Nothing moves.", parent=self.developer_window,
            default=messagebox.NO)
        if confirmed:
            result = self.request_run_clear()
            if isinstance(result, dict):
                self._current_health["developer_tests"] = result
        self._refresh_runstart_controls()

    def _refresh_runstart_controls(self):
        if not hasattr(self, "run_start_btn"):
            return  # rows built before the buttons
        status = self._current_health.get("developer_tests", {})
        run = status.get("run") or {}
        saved, place = run.get("saved"), run.get("place")
        running, waiting = bool(run.get("running")), bool(run.get("waiting_release"))
        operator = self._current_health.get("operator_controls") or {}
        stopped = operator.get("emergency_stopped") is True
        if saved and place == "partway" and not running and saved["id"] != self._run_loaded_id:
            self._run_load(saved)
        locked = bool(run.get("locked"))
        if running or waiting or locked:
            self._run_note = ""  # a typing hint about the rows is over once they lock
        for row in self._run_rows:
            self._set(row["combo"], state="disabled" if locked else "readonly")
            self._set(row["entry"], state="disabled" if locked else "normal")
            self._set(row["trash"], state="disabled" if locked or len(self._run_rows) <= 1 else "normal")
        self._set(self.run_add_btn, state="disabled" if locked or len(self._run_rows) >= RUN_MAX_STEPS
                  else "normal")
        for entry in (self.run_speed_entry, self.run_accel_entry):
            self._set(entry, state="disabled" if locked else "normal")
        progress = run.get("progress")
        here = (progress["step"], progress["done"]) if running and progress else \
            (saved["step"], saved["done"]) if saved and not running else None
        for index, row in enumerate(self._run_rows):
            mark = ""
            if here is not None and (running or place in ("partway", "end")):
                if index < here[0] or (place == "end" and not running):
                    mark = "\u2714 done"
                elif index == here[0]:
                    total = row["steps"].get().strip()
                    mark = (("\u25b6 " if running else "\u25a0 ") + f"{here[1]}/{total}")
                elif running and run.get("mode") == "U":
                    mark = "\u21a9 undone"
            self._set(row["mark"], text=mark)
        free = not running and not waiting
        resumable = bool(saved and free and self.request_run_resume and
                         (status.get("enabled") or (stopped and operator.get("can_release"))))
        self._set(self.run_start_btn, state="normal" if self.request_run and status.get("enabled")
                  and not locked else "disabled")
        self._set(self.run_continue_btn, state="normal" if resumable and place in ("partway", "start")
                  else "disabled")
        self._set(self.run_reverse_btn, state="normal" if resumable and place in ("partway", "end")
                  else "disabled")
        self._set(self.run_clear_btn, state="normal" if saved and free and self.request_run_clear
                  else "disabled")
        message = run.get("message", "")
        if running and progress:
            total = len(self._run_rows)
            text = (f"\u25b6 {'Reversing' if run.get('mode') == 'U' else 'Running'}: step "
                    f"{progress['step'] + 1} of {total}, {progress['done']} steps done. "
                    "EMERGENCY STOP stops it at once.")
            tone = "warning"
        elif running or waiting:
            text, tone = "\u25b6 " + (message or "Starting\u2026"), "warning"
        elif saved and place == "partway":
            text = (f"Stopped at step {saved['step'] + 1} of {len(saved['steps'])} "
                    f"({saved['done']} of {saved['steps'][saved['step']][1]} steps). Continue run "
                    "finishes it; Reverse to start takes it back; Discard saved unlocks the rows."
                    + (" Either button releases the emergency stop first." if stopped else ""))
            tone = "warning"
        elif self._run_note:
            text, tone = self._run_note, "warning"
        elif not self.request_run:
            text, tone = "RunStart is not available in this window.", "muted"
        elif status.get("enabled"):
            text = ("\u2714 Ready. " + (message + " " if message else "") +
                    ("Reverse to start can take the last list back." if place == "end" else ""))
            tone = "ok"
        else:
            text = "Not available: " + (status.get("blocked") or "waiting for the robot\u2026")
            tone = "warning"
        self._set(self.run_status_lbl, text=text.strip(), fg=COLORS[tone])

    def _build_log_tab(self):
        page = self.log_tab
        page.grid_columnconfigure(0, weight=1)
        page.grid_rowconfigure(0, weight=1)
        panel = self._panel(page)
        panel.grid(row=0, column=0, sticky="nsew", pady=6)
        panel.grid_columnconfigure(0, weight=1)
        panel.grid_rowconfigure(1, weight=1)
        header = tk.Frame(panel, bg=COLORS["surface"])
        header.grid(row=0, column=0, columnspan=2, sticky="ew", padx=10, pady=(6, 2))
        header.grid_columnconfigure(1, weight=1)
        self._label(header, "Pi log, newest at the bottom", font=self.small_font,
                    tone="muted").grid(row=0, column=0, sticky="w")
        self.log_path_lbl = self._label(header, "Not saved to a file", font=self.small_font,
                                        tone="muted")
        self.log_path_lbl.grid(row=0, column=1, sticky="w", padx=12)
        self.clear_log_btn = ttk.Button(header, text="Clear", style="Compact.Monitor.TButton",
                                        command=self._clear_log)
        self.clear_log_btn.grid(row=0, column=2, sticky="e")
        self.log_view = tk.Text(panel, height=4, width=1, wrap="word", bg=COLORS["surface"],
                                fg=COLORS["text"], font=self.small_font, relief="flat",
                                highlightthickness=0, padx=10, pady=4, state="disabled")
        self.log_view.grid(row=1, column=0, sticky="nsew")
        self.log_view.tag_configure("warning", foreground=COLORS["warning"])
        self.log_view.tag_configure("error", foreground=COLORS["danger"])
        scrollbar = ttk.Scrollbar(panel, command=self.log_view.yview)
        scrollbar.grid(row=1, column=1, sticky="ns")
        self.log_view.configure(yscrollcommand=scrollbar.set)

    def attach_log(self, handler, path=None):
        """Show this WindowLogHandler's records; ``path`` is the saved daily file."""
        self._log_handler = handler
        self._set(self.log_path_lbl, text=f"Saved to {path}" if path else "Not saved to a file")

    LOG_VIEW_LIMIT = 800

    def _process_log(self):
        if self._log_handler is None:
            return
        records = self._log_handler.since(self._log_sequence)
        if not records:
            return
        self._log_sequence = records[-1][0]
        records = records[-self.LOG_VIEW_LIMIT:]
        at_bottom = self.log_view.yview()[1] >= 0.99
        self.log_view.configure(state="normal")
        for _, level, text in records:
            tag = "error" if level >= logging.ERROR else "warning" if level >= logging.WARNING else ()
            self.log_view.insert("end", text + "\n", tag)
        excess = int(self.log_view.index("end-1c").split(".")[0]) - (self.LOG_VIEW_LIMIT + 1)
        if excess > 0:
            self.log_view.delete("1.0", f"{excess + 1}.0")
        if at_bottom:
            self.log_view.see("end")
        self.log_view.configure(state="disabled")

    def _clear_log(self):
        self.log_view.configure(state="normal")
        self.log_view.delete("1.0", "end")
        self.log_view.configure(state="disabled")

    def _report_callback_exception(self, exc, value, traceback):
        logger.error("Monitor callback failed", exc_info=(exc, value, traceback))

    # ── greyed-out controls answer a click (owner, 2026-11-16) ────────────────
    def _answer_unavailable_clicks(self):
        """A disabled button does nothing when clicked, which read as "no prompt".
        Each real control now explains why it is unavailable instead."""
        for button, title, reason in (
                (self.start_btn, "Manual disposal is not available now", self._start_unavailable_text),
                (self.release_btn, "Release stop is not available now", self._release_unavailable_text),
                (self.continue_btn, "Continue run is not available now", self._continue_unavailable_text),
                (self.reverse_btn, "Reverse to start is not available now", self._reverse_unavailable_text),
                (self.ultrasonic_btn, "Sonar setting is not available now", self._sonar_unavailable_text),
                (self.reset_btn, "Restart is not available now", self._reset_unavailable_text)):
            button.bind("<ButtonRelease-1>",
                        lambda event, b=button, t=title, r=reason: self._explain_click(event, b, t, r),
                        add="+")

    def _explain_click(self, event, button, title, reason):
        inside = 0 <= event.x < button.winfo_width() and 0 <= event.y < button.winfo_height()
        if not inside or not button.instate(["disabled"]):
            return None
        self._current_health = dict(self.read_health())
        messagebox.showinfo(title, reason(), parent=self.root)
        return "break"

    def _connection_text(self, status):
        if status.get("connected") is False:
            return "GARBY is not connected over Bluetooth yet."
        if status.get("pending"):
            return "An earlier request is still waiting for GARBY's answer."
        return ""

    def _route_text(self):
        health = self._current_health
        state = health.get("robot_state", "UNKNOWN")
        if state not in ROUTE_STATES:
            return ""
        text = f"A route is already running ({state.lower()})."
        hold = health.get("motion_hold")
        if hold:
            text += f"\n\nIts motors are held: {hold_in_words(hold)}."
            text += (" GARBY waits until that is clear. Move the object away, or test on the "
                     "floor; on the bench the Developer tests window can turn the wheels.")
            if hold.startswith("Side clearance"):
                text += " Or set Sides to read only under LiDAR obstacle checks."
        return text + ("\n\nTo end the route: press EMERGENCY STOP, then Release stop on the "
                       "Controls tab.")

    def _start_unavailable_text(self):
        status = self._current_health.get("manual_start", {})
        stopped = self._current_health.get("operator_controls", {}).get("emergency_stopped")
        parts = [status.get("message") or "GARBY has not reported that it is ready."]
        route = self._route_text()
        if route:
            parts.append(route)
        elif stopped is True:
            parts.append("The emergency stop is on. Press Release stop first.")
        elif self._current_health.get("robot_state") == "AT_DESTINATION":
            parts.append("GARBY is parked at the destination. Send it back first.")
        elif self._current_health.get("developer_tests", {}).get("pending"):
            parts.append("A developer test is running. Wait for it to finish.")
        return "\n\n".join(parts)

    def _operator_text(self, default):
        status = self._current_health.get("operator_controls", {})
        return "\n\n".join(part for part in (self._connection_text(status) or default,
                                               self._route_text()) if part)

    def _release_unavailable_text(self):
        stopped = self._current_health.get("operator_controls", {}).get("emergency_stopped")
        return self._operator_text(
            "There is no emergency stop to release. Release stop ends a route after "
            "EMERGENCY STOP." if stopped is not True else
            "Release becomes available once GARBY confirms it has stopped and is idle.")

    def _continue_unavailable_text(self):
        return self._operator_text(
            "Continue run is offered only after EMERGENCY STOP interrupted a route that "
            "GARBY can still finish.")

    def _reverse_unavailable_text(self):
        return self._operator_text(
            "Reverse to start is offered only after EMERGENCY STOP interrupted a route that "
            "GARBY can drive back.")

    def _sonar_unavailable_text(self):
        return self._operator_text(
            "The obstacle sonar can be switched on or off only while GARBY is idle.")

    def _reset_unavailable_text(self):
        return self._operator_text(
            "Press EMERGENCY STOP first. The restart needs a confirmed stop.")

    def _build_developer_controls(self):
        page = self.developer_tab
        page.grid_columnconfigure(0, weight=1)
        panel = self._panel(page)
        panel.grid(row=0, column=0, sticky="nsew", pady=6)
        panel.grid_columnconfigure(0, weight=1)
        panel.grid_columnconfigure(1, weight=1)
        # First thing to read: can a test start now, and if not, why and what to do.
        self.developer_banner_lbl = self._label(
            panel, "Not available: waiting for the robot\u2026", tone="warning",
            font=self.heading_font, wraplength=730, justify="left")
        self.developer_banner_lbl.grid(row=0, column=0, columnspan=2, sticky="w", padx=12, pady=(10, 6))
        gates = tk.Frame(panel, bg=COLORS["surface"])
        gates.grid(row=1, column=0, sticky="nw", padx=12, pady=2)
        self._label(gates, "1. Sensors GARBY obeys during the test", font=self.small_font,
                    tone="muted").grid(row=0, column=0, sticky="w", pady=(0, 2))
        self.developer_gate_vars = {bit: tk.BooleanVar(value=True) for bit in (1, 2, 4)}
        self.developer_gate_checks = []
        for row, (bit, label) in enumerate(((1, "LiDAR: stop if something is close"),
                                           (2, "Gyro (MPU): hold heading; turns need it"),
                                           (4, "Sonar: stop if something is in front")), start=1):
            check = ttk.Checkbutton(gates, text=label, variable=self.developer_gate_vars[bit],
                                    command=self._refresh_developer_controls)
            check.grid(row=row, column=0, sticky="w", pady=1)
            self.developer_gate_checks.append(check)
        self._label(gates, "Untick one to test without it. All are back on after the test.",
                    font=self.small_font, tone="muted", wraplength=330, justify="left").grid(
                        row=4, column=0, sticky="w", pady=(2, 0))
        buttons = tk.Frame(panel, bg=COLORS["surface"])
        buttons.grid(row=1, column=1, sticky="nw", padx=12, pady=2)
        self._label(buttons, "2. Choose a test (90°: 30 s; others: 8 s)", font=self.small_font,
                    tone="muted").grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 2))
        self.developer_buttons = {}
        # Backward (owner, 2026-11-16): the parking step's reverse travel and rear check.
        labels = (("F", "Forward ~5 cm"), ("B", "Backward ~5 cm"),
                  ("1", "Motor 1 only"), ("2", "Motor 2 only"),
                  ("L", "Turn left 10\u00b0"), ("R", "Turn right 10\u00b0"),
                  ("L90", "Turn left 90\u00b0"), ("R90", "Turn right 90\u00b0"),
                  ("I", "Check sensors (no movement)"))
        for index, (action, label) in enumerate(labels):
            button = ttk.Button(buttons, text=label, style="Compact.Monitor.TButton",
                                command=lambda a=action: self._request_developer_test(a), state="disabled")
            button.grid(row=1 + index // 2, column=index % 2, sticky="ew", padx=(0, 8), pady=2)
            self.developer_buttons[action] = button
        self.developer_turn_hint_lbl = self._label(buttons, "", font=self.small_font, tone="muted",
                                                   wraplength=330, justify="left")
        self.developer_turn_hint_lbl.grid(row=6, column=0, columnspan=2, sticky="w", pady=(2, 0))
        self.developer_sensor_lbl = self._label(panel, "Live: no readings yet", font=self.small_font,
                                                wraplength=730, justify="left")
        self.developer_sensor_lbl.grid(row=2, column=0, columnspan=2, sticky="w", padx=12, pady=(6, 2))
        self.developer_status_lbl = self._label(panel, "No test run yet.", font=self.small_font,
                                                wraplength=730, justify="left")
        self.developer_status_lbl.grid(row=3, column=0, columnspan=2, sticky="w", padx=12, pady=2)
        self.developer_result_lbl = self._label(panel, "", font=self.small_font, tone="muted",
                                                wraplength=730, justify="left")
        self.developer_result_lbl.grid(row=4, column=0, columnspan=2, sticky="w", padx=12, pady=(2, 10))

    def _refresh_developer_controls(self):
        status = self._current_health.get("developer_tests", {})
        pending = bool(status.get("pending"))
        enabled = bool(self.request_developer_test and status.get("enabled"))
        mpu = self.developer_gate_vars[2].get()
        for action, button in self.developer_buttons.items():
            allowed = enabled and (action not in TURN_ACTIONS or mpu)
            self._set(button, state="normal" if allowed else "disabled")
        for check in self.developer_gate_checks:
            self._set(check, state="disabled" if pending else "normal")
        if not self.request_developer_test:
            banner, tone = "Developer tests are not available in this window.", "muted"
        elif pending:
            banner, tone = "\u25b6 " + (status.get("message") or "Test running\u2026"), "warning"
        elif enabled:
            banner, tone = "\u2714 Ready. Choose the sensors, then press a test.", "ok"
        else:
            banner = "Not available: " + (status.get("blocked") or "waiting for the robot\u2026")
            tone = "warning"
        self._set(self.developer_banner_lbl, text=banner, fg=COLORS[tone])
        self._set(self.developer_turn_hint_lbl,
                  text="Turns need the gyro (MPU) ticked." if not mpu else
                  "Turns: GARBY must be on the floor, wheels down. The MPU measures the robot turning.",
                  fg=COLORS["warning" if not mpu else "muted"])
        sensors = status.get("sensors", {})
        mpu_text = (f"gyro heading {sensors['yaw_deg']:+.2f}\u00b0"
                    if sensors.get("mpu_ready") else "gyro: no reading")
        wall_text = (f"LiDAR wall angle {sensors['wall_deg']:+.2f}\u00b0"
                     if sensors.get("walls_ready") else "LiDAR wall angle: no wall seen")
        # The wall parking squares up to before backing: angle 0 = square to it.
        rear = self._current_health.get("rear_wall")
        rear_text = (f"rear wall {rear['distance_cm']:.0f} cm, angle {rear['angle_deg']:+.2f}\u00b0"
                     if rear else "no rear wall seen")
        self._set(self.developer_sensor_lbl,
                  text=f"Live: {mpu_text}   |   {wall_text}   |   {rear_text}")
        last = status.get("message", "")
        self._set(self.developer_status_lbl,
                  text="" if pending else (f"Last test: {last}" if last else "No test run yet."))
        self._set(self.developer_result_lbl, text=status.get("result", ""))

    def _request_developer_test(self, action):
        self._current_health = dict(self.read_health())
        status = self._current_health.get("developer_tests", {})
        sensors = sum(bit for bit, value in self.developer_gate_vars.items() if value.get())
        if not self.request_developer_test or not status.get("enabled") or (action in TURN_ACTIONS and not sensors & 2):
            self._refresh_developer_controls()
            return
        confirmed = action == "I"
        if not confirmed:
            names = ((1, "LiDAR"), (2, "Gyro (MPU)"), (4, "Sonar"))
            obeys = ", ".join(name for bit, name in names if sensors & bit) or "nothing"
            ignores = ", ".join(name for bit, name in names if not sensors & bit) or "nothing"
            title = self.developer_buttons[action].cget("text")
            floor_notice = ("GARBY must be on the floor with both wheels touching the ground. "
                            "Do not lift the motors or wheels. The MPU detects the robot turning; "
                            "spinning lifted wheels cannot show rotation.\n\n"
                            if action in TURN_ACTIONS else "")
            confirmed = messagebox.askyesno(
                "Start this test?",
                f"{title}\n\n{floor_notice}Obeys: {obeys}\nIgnores: {ignores}\n\n"
                "GARBY will move briefly. Keep hands clear of the wheels. "
                "EMERGENCY STOP still works.", parent=self.developer_window,
                default=messagebox.NO)
        # A confirmation dialog can outlive IDLE or the controller connection.
        self._current_health = dict(self.read_health())
        if confirmed and self._current_health.get("developer_tests", {}).get("enabled"):
            result = self.request_developer_test(action, sensors)
            if result is not None:
                self._current_health["developer_tests"] = result
        self._refresh_developer_controls()

    def _build_controls(self):
        page = self.controls_tab
        page.grid_columnconfigure(0, weight=1, uniform="controls")
        page.grid_columnconfigure(1, weight=1, uniform="controls")
        page.grid_rowconfigure(1, weight=1)
        self.operator_status_lbl = self._label(
            page, "Waiting for operator-control status", font=self.small_font,
            bg=COLORS["background"], tone="muted", wraplength=740, justify="left", height=2)
        self.operator_status_lbl.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(2, 4))
        left, right = self._panel(page), self._panel(page)
        left.grid(row=1, column=0, sticky="nsew", padx=(0, 5), pady=(0, 6))
        right.grid(row=1, column=1, sticky="nsew", padx=(5, 0), pady=(0, 6))
        for panel in (left, right):
            panel.grid_columnconfigure(0, weight=1)
        self._label(left, "LiDAR obstacle checks", font=self.heading_font).grid(
            row=0, column=0, sticky="w", padx=12, pady=(10, 6))
        if self.allow_safety_disable:
            checks = tk.Frame(left, bg=COLORS["surface"])
            checks.grid(row=1, column=0, sticky="w", padx=12)
            # Sides (owner, 2026-11-16) joins front and back; compact so all three fit 800 px.
            self.front_btn = ttk.Button(checks, style="Compact.Monitor.TButton", command=self._toggle_front)
            self.front_btn.pack(side="left", padx=(0, 4))
            self.back_btn = ttk.Button(checks, style="Compact.Monitor.TButton", command=self._toggle_back)
            self.back_btn.pack(side="left", padx=(0, 4))
            self.sides_btn = ttk.Button(checks, style="Compact.Monitor.TButton", command=self._toggle_sides)
            self.sides_btn.pack(side="left")
        self.safety_note_lbl = self._label(
            left, "Read only keeps distances visible and stops that group holding motion. "
            "Sides covers anything right beside GARBY; a spin still needs front/back. "
            "Missing or stale scans still stop it. LiDAR sees range returns; it does not "
            "identify people.",
            font=self.small_font, tone="warning", wraplength=330, justify="left")
        self.safety_note_lbl.grid(row=2, column=0, sticky="w", padx=12, pady=(6, 8))
        ttk.Separator(left).grid(row=3, column=0, sticky="ew", padx=12)
        self._label(left, "Obstacle ultrasonic + servo", font=self.heading_font).grid(
            row=4, column=0, sticky="w", padx=12, pady=(8, 4))
        self.ultrasonic_btn = ttk.Button(left, text="Ultrasonic: unknown", style="Monitor.TButton",
                                          command=self._request_ultrasonic, state="disabled")
        self.ultrasonic_btn.grid(row=5, column=0, sticky="w", padx=12)
        self.ultrasonic_status_lbl = self._label(
            left, "Controller confirmation required", font=self.small_font,
            tone="muted", wraplength=330, justify="left")
        self.ultrasonic_status_lbl.grid(row=6, column=0, sticky="w", padx=12, pady=(4, 0))
        self._label(left, "Disable stops sonar and holds the servo at its default view. "
                    "Bin sensing stays on. Changes require IDLE.",
                    font=self.small_font, tone="muted", wraplength=330, justify="left").grid(
                        row=7, column=0, sticky="w", padx=12, pady=(5, 8))

        self._label(right, "Motion", font=self.heading_font).grid(
            row=0, column=0, sticky="w", padx=12, pady=(10, 6))
        motion = tk.Frame(right, bg=COLORS["surface"])
        motion.grid(row=1, column=0, sticky="w", padx=12)
        self.start_btn = ttk.Button(motion, text="Manual disposal",
                                    style="Monitor.TButton", command=self._request_start,
                                    state="disabled")
        # Two rows: four buttons do not fit side by side on an 800 px display.
        self.start_btn.grid(row=0, column=0, sticky="w", padx=(0, 6))
        self.release_btn = ttk.Button(motion, text="Release stop", style="Monitor.TButton",
                                      command=self._request_release_stop, state="disabled")
        self.release_btn.grid(row=0, column=1, sticky="w")
        self.continue_btn = ttk.Button(motion, text="Continue run", style="Monitor.TButton",
                                       command=self._request_continue_run, state="disabled")
        self.continue_btn.grid(row=1, column=0, sticky="w", padx=(0, 6), pady=(4, 0))
        self.reverse_btn = ttk.Button(motion, text="Reverse to start", style="Monitor.TButton",
                                      command=self._request_reverse_run, state="disabled")
        self.reverse_btn.grid(row=1, column=1, sticky="w", pady=(4, 0))
        self.start_status_lbl = self._label(right, "Waiting for both controllers", tone="muted",
                                            font=self.small_font, wraplength=330, justify="left", height=2)
        self.start_status_lbl.grid(row=2, column=0, sticky="ew", padx=12, pady=(2, 0))
        self.stop_status_lbl = self._label(right, "Emergency stop: unknown", font=self.small_font,
                                           tone="muted", wraplength=330, justify="left")
        self.stop_status_lbl.grid(row=3, column=0, sticky="w", padx=12, pady=(1, 2))
        # Kept to two lines: the second button row took the space of the third.
        self._label(right, "Continue finishes the stopped route. Reverse drives back to "
                    "its start. Release ends it at idle.",
                    font=self.small_font, tone="muted", wraplength=330, justify="left").grid(
                        row=4, column=0, sticky="w", padx=12, pady=(0, 3))
        ttk.Separator(right).grid(row=5, column=0, sticky="ew", padx=12)
        self.reset_btn = ttk.Button(right, text="Restart Pi + bridge + executor…", style="Monitor.TButton",
                                    command=self._request_system_reset, state="disabled")
        self.reset_btn.grid(row=6, column=0, sticky="w", padx=12, pady=(4, 2))
        self._label(right, "Apply emergency stop before restarting. Restarts both motion MCUs and reboots the Pi. "
                    "The separate bin/gas sensor board has no reset command.", font=self.small_font,
                    tone="muted", wraplength=330, justify="left").grid(
                        row=7, column=0, sticky="w", padx=12, pady=(0, 8))

    def _refresh_operator_controls(self):
        status = self._current_health.get("operator_controls", {})
        ready = bool(status.get("supported") and status.get("connected") and not status.get("pending"))
        for button, callback, key in (
                (self.ultrasonic_btn, self.request_ultrasonic, "can_configure"),
                (self.release_btn, self.request_release_stop, "can_release"),
                (self.continue_btn, self.request_continue_run, "can_continue"),
                (self.reverse_btn, self.request_reverse_run, "can_reverse"),
                (self.reset_btn, self.request_system_reset, "can_reset")):
            self._set(button, state="normal" if ready and callback and status.get(key) else "disabled")
        ultrasonic = status.get("ultrasonic_enabled")
        stopped = status.get("emergency_stopped")
        sonar_text = ("Disable ultrasonic" if ultrasonic is True else
                      "Enable ultrasonic" if ultrasonic is False else "Ultrasonic: unknown")
        self._set(self.ultrasonic_btn, text=sonar_text)
        self._set(self.ultrasonic_status_lbl,
                  text=("Enabled (controller confirmed)" if ultrasonic is True else
                        "Disabled; servo at default view (confirmed)" if ultrasonic is False else
                        "Controller confirmation required"),
                  fg=COLORS["warning" if ultrasonic is False else "muted"])
        self._set(self.stop_status_lbl,
                  text=("Emergency stop: latched (confirmed)" if stopped is True else
                        "Emergency stop: released (confirmed)" if stopped is False else
                        "Emergency stop: unknown / unconfirmed"),
                  fg=COLORS["danger" if stopped is True else "muted"])
        self._set(self.operator_status_lbl,
                  text=status.get("message", "Operator controls unavailable; updated controller firmware required"),
                  fg=COLORS["warning" if status.get("pending") else "muted"])

    def _operator_action(self, callback, allowed_key=None, *args):
        if callback is None:
            return
        if allowed_key is not None:
            # Recheck hardware state at click time; the previous UI poll may be old.
            self._current_health = dict(self.read_health())
            status = self._current_health.get("operator_controls", {})
            if not (status.get("supported") and status.get("connected")
                    and not status.get("pending") and status.get(allowed_key)):
                self._refresh_operator_controls()
                return
        result = callback(*args)
        # The action can change manual-start eligibility as well as operator
        # status. Refresh both from one current snapshot before painting.
        self._current_health = dict(self.read_health())
        if result is not None:
            self._current_health["operator_controls"] = result
        self._refresh_operator_controls()
        self._refresh_start()

    def _request_emergency_stop(self):
        # Never require a readiness snapshot before latching the operator stop.
        self._operator_action(self.request_emergency_stop)

    def _request_release_stop(self):
        self._operator_action(self.request_release_stop, "can_release")

    def _request_continue_run(self):
        # The robot moves again after this, so confirm, then recheck at click time.
        self._current_health = dict(self.read_health())
        if not (self.request_continue_run and
                self._current_health.get("operator_controls", {}).get("can_continue")):
            self._refresh_operator_controls()
            return
        confirmed = messagebox.askyesno(
            "Continue run?",
            "Release the emergency stop and let GARBY finish the stopped route from where it "
            "stopped?\n\nOnly continue if GARBY has not been moved by hand. Obstacle "
            "detection, the heading check and emergency stop remain active.",
            parent=self.root, default=messagebox.NO)
        if confirmed:
            self._operator_action(self.request_continue_run, "can_continue")
        else:
            self._refresh_operator_controls()

    def _request_reverse_run(self):
        # The robot moves again after this, so confirm, then recheck at click time.
        self._current_health = dict(self.read_health())
        if not (self.request_reverse_run and
                self._current_health.get("operator_controls", {}).get("can_reverse")):
            self._refresh_operator_controls()
            return
        confirmed = messagebox.askyesno(
            "Reverse to start?",
            "Release the emergency stop and let GARBY drive back to where this route started?\n\n"
            "GARBY turns around, drives forward along the way it came and faces its starting "
            "direction again. Only do this if GARBY has not been moved by hand. It returns only "
            "as exactly as its wheels repeat their steps. Obstacle detection, the heading check "
            "and emergency stop remain active.",
            parent=self.root, default=messagebox.NO)
        if confirmed:
            self._operator_action(self.request_reverse_run, "can_reverse")
        else:
            self._refresh_operator_controls()

    def _request_ultrasonic(self):
        self._current_health = dict(self.read_health())
        enabled = self._current_health.get("operator_controls", {}).get("ultrasonic_enabled")
        if isinstance(enabled, bool):
            self._operator_action(self.request_ultrasonic, "can_configure", not enabled)

    def _request_system_reset(self):
        self._current_health = dict(self.read_health())
        status = self._current_health.get("operator_controls", {})
        if not (self.request_system_reset and status.get("supported") and status.get("connected")
                and not status.get("pending") and status.get("can_reset")):
            self._refresh_operator_controls()
            return
        accepted = messagebox.askokcancel(
            "Restart GARBY system",
            "Restart the Raspberry Pi, BLE bridge MCU and motor executor MCU?\n\n"
            "The emergency stop must be confirmed first. The controllers will restart stopped; "
            "release and a new manual start are required. The monitor will close during Pi reboot.\n\n"
            "The separate bin/gas sensor board cannot be reset by this command.",
            parent=self.root, icon="warning", default=messagebox.CANCEL)
        if accepted:
            # A modal dialog runs Tk's event loop. Read readiness again after it closes.
            self._operator_action(self.request_system_reset, "can_reset")

    def _label(self, parent, text, *, tone="text", font=None, **options):
        options.setdefault("bg", COLORS["surface"])
        options.setdefault("highlightthickness", 0)
        return tk.Label(parent, text=text, fg=COLORS[tone], font=font or self.font,
                        anchor="w", **options)

    @staticmethod
    def _panel(parent):
        return tk.Frame(parent, bg=COLORS["surface"], highlightthickness=1,
                        highlightbackground=COLORS["border"], highlightcolor=COLORS["border"])

    def _set(self, widget, **options):
        previous = self._widget_values.setdefault(widget, {})
        changed = {key: value for key, value in options.items() if previous.get(key) != value}
        if changed:
            widget.configure(**changed)
            previous.update(changed)

    def toggle_fullscreen(self, event=None):
        self.root.attributes("-fullscreen", not self.root.attributes("-fullscreen"))

    def _leave_fullscreen(self, event=None):
        self.root.attributes("-fullscreen", False)

    def _tab_changed(self, event=None):
        self._messages_visible = self.notebook.select() == str(self.ble_tab)
        self._refresh_header_status()
        self._render_scan()

    def _refresh_header_status(self):
        # The map's status lives in its sidebar. Other tabs retain a visible
        # header status/hold, where that sidebar is not on screen.
        on_map = self.notebook.select() == str(self.lidar_tab)
        if on_map:
            self.header_subtitle_lbl.grid_remove()
        else:
            self.header_subtitle_lbl.grid()
        if not on_map and self.header_motion_hold_lbl.cget("text"):
            self.header_motion_hold_lbl.grid()
        else:
            self.header_motion_hold_lbl.grid_remove()

    def _cycle_map_range(self):
        ranges = (2, 4, 8)
        self._map_range_m = ranges[(ranges.index(self._map_range_m) + 1) % len(ranges)]
        self._set(self.map_range_btn, text=f"Range: {self._map_range_m} m")
        self._map_geometry = None
        self._render_scan()

    def _resize_scan(self, event=None):
        self._render_scan()

    # ── LiDAR map ────────────────────────────────────────────────────────────
    def _init_map_state(self):
        self._map_render_key = None
        self._map_snapshot_rendered = None
        self._map_geometry = None
        self._map_layout = None
        self._map_range_m = 4
        self._map_item_state = {}
        self._map_points_shown = 0
        self._map_markers_shown = 0
        self._availability = LidarAvailability()
        self._lidar_view = ("loading", "Waiting for first scan", "")
        self._sweep_after_id = None
        self._sweep_started_at = None
        self.sector_readings = {side: direction_reading(None, 0) for side, *_ in DIRECTIONS}

    def _build_map_items(self):
        """Create every map item once; rendering only moves or restyles them.

        Creation order is drawing order: scope, rings, spokes, sweep, stop
        arcs, scan points, robot, labels and messages on top.
        """
        canvas = self.scan_canvas
        self._map_scope = canvas.create_oval(0, 0, 0, 0, fill=RADAR["scope"],
                                             outline=RADAR["ring"], width=2)
        self._map_rings = [canvas.create_oval(0, 0, 0, 0, outline=RADAR["ring"], dash=(2, 4))
                           for _ in range(3)]
        self._map_ring_labels = [canvas.create_text(0, 0, text="", fill=RADAR["ring_label"],
                                                    font=self.small_font, anchor="se")
                                 for _ in range(4)]
        self._map_spokes = [canvas.create_line(0, 0, 0, 0, fill=RADAR["spoke"]) for _ in DIRECTIONS]
        self._map_sweep = [canvas.create_arc(0, 0, 0, 0, style="pieslice", fill=color,
                                             outline="", extent=SWEEP_WEDGE_DEG, state="hidden")
                           for color in sweep_colors()]
        self._map_sweep_line = canvas.create_line(0, 0, 0, 0, fill=RADAR["sweep"], width=2,
                                                  state="hidden")
        self._map_thresholds = {side: canvas.create_arc(0, 0, 0, 0, style="arc", outline=RADAR["threshold"],
                                                        extent=SECTOR_WIDTH_DEG, dash=(3, 3),
                                                        state="hidden")
                                for side, *_ in DIRECTIONS}
        self._map_points = [canvas.create_oval(0, 0, 0, 0, fill=RADAR["point"], outline="",
                                               state="hidden") for _ in range(MAX_SCAN_POINTS)]
        # Hallway markers, above the scan points: a direction line, then a dot
        # (person, moving object) or a square (object standing in the hallway).
        self._map_marker_lines = [canvas.create_line(0, 0, 0, 0, fill=RADAR["person"], width=2,
                                                     state="hidden")
                                  for _ in range(MAX_MAP_MARKERS)]
        self._map_marker_dots = [canvas.create_oval(0, 0, 0, 0, fill=RADAR["person"],
                                                    outline=RADAR["marker_edge"], state="hidden")
                                 for _ in range(MAX_MAP_MARKERS)]
        self._map_marker_squares = [canvas.create_polygon(0, 0, 0, 0, 0, 0, 0, 0,
                                                          fill=RADAR["object"],
                                                          outline=RADAR["marker_edge"],
                                                          state="hidden")
                                    for _ in range(MAX_MAP_MARKERS)]
        self._map_robot = canvas.create_polygon(0, 0, 0, 0, 0, 0, fill=RADAR["robot"], outline="")
        self._map_labels = {}
        for side, _, _, anchor in DIRECTIONS:
            justify = "right" if "e" in anchor else "left" if "w" in anchor else "center"
            self._map_labels[side] = canvas.create_text(0, 0, text="", fill=RADAR["muted"],
                                                        font=self.map_label_font, anchor=anchor,
                                                        justify=justify)
        self._map_empty = canvas.create_text(0, 0, text="", fill=RADAR["warning"],
                                             font=self.map_title_font, justify="center")
        self._map_detail = canvas.create_text(0, 0, text="", fill=RADAR["muted"],
                                              font=self.small_font, justify="center")

    def _canvas_set(self, item, **options):
        """itemconfigure only what changed, so an unchanged frame costs nothing."""
        previous = self._map_item_state.setdefault(item, {})
        changed = {key: value for key, value in options.items() if previous.get(key) != value}
        if changed:
            self.scan_canvas.itemconfigure(item, **changed)
            previous.update(changed)

    def _layout_map(self, width, height, range_m):
        canvas = self.scan_canvas
        cx, cy = width / 2, height / 2
        # Side labels need about 90 px beside the scope; front and rear take one
        # line each above and below it.
        try:
            line = int(self.map_label_font.metrics("linespace"))
        except (AttributeError, tk.TclError):
            line = 15
        radius = max(20.0, min(width / 2 - 92, height / 2 - line - 9))
        self._map_layout = (cx, cy, radius)
        canvas.coords(self._map_scope, cx - radius, cy - radius, cx + radius, cy + radius)
        for index, ring in enumerate(self._map_rings, 1):
            r = radius * index / 4
            canvas.coords(ring, cx - r, cy - r, cx + r, cy + r)
        for index, label in enumerate(self._map_ring_labels, 1):
            canvas.coords(label, cx + radius * index / 4 - 2, cy - 2)
            self._canvas_set(label, text=f"{range_m * index / 4:g}")
        for spoke, (_, _, angle, _) in zip(self._map_spokes, DIRECTIONS):
            boundary = angle + SECTOR_WIDTH_DEG / 2
            x0, y0 = scope_offset(boundary, radius * 0.12)
            x1, y1 = scope_offset(boundary, radius)
            canvas.coords(spoke, cx + x0, cy + y0, cx + x1, cy + y1)
        for wedge in self._map_sweep:
            canvas.coords(wedge, cx - radius, cy - radius, cx + radius, cy + radius)
        canvas.coords(self._map_robot, cx, cy - 9, cx - 6, cy + 6, cx + 6, cy + 6)
        self._map_threshold_fits = {}
        for side, _, angle, _ in DIRECTIONS:
            dx, dy = scope_offset(angle, radius + 5)
            if angle % 90:
                # Diagonals move further out sideways, clear of a wide front or
                # rear label ("Front  200 cm · read only") on a short screen.
                dx += math.copysign(16, dx)
            canvas.coords(self._map_labels[side], cx + dx, cy + dy)
            # Stop arcs depend only on scale; too small or beyond the view, hide.
            radius_px = self._threshold_cm(side) / 100.0 * radius / range_m
            self._map_threshold_fits[side] = 6 <= radius_px <= radius
            canvas.coords(self._map_thresholds[side], cx - radius_px, cy - radius_px,
                          cx + radius_px, cy + radius_px)
            self._canvas_set(self._map_thresholds[side], start=canvas_arc_start(angle))
        self._canvas_set(self._map_detail, width=int(radius * 1.6))  # wrap inside the scope
        self._map_geometry = (width, height, range_m)
        # Points, stop arcs and messages depend on the layout: redraw them.
        self._map_render_key = None

    def _threshold_cm(self, side):
        return (self.front_diagonal_threshold_cm if side in ("FRONT_LEFT", "FRONT_RIGHT") else
                self.front_threshold_cm if side in self.FRONT_GROUP else
                self.back_threshold_cm if side in self.BACK_GROUP else self.side_threshold_cm)

    def _bypassed(self, side):
        return bool(self.allow_safety_disable and (
            (side in self.FRONT_GROUP and self.front_disabled.is_set())
            or (side in self.BACK_GROUP and self.back_disabled.is_set())
            or (side in ("LEFT", "RIGHT") and self.side_disabled.is_set())))

    def _refresh_availability(self, now=None):
        """Combine scan age, stream health and the driver's phase into the map's mode."""
        now = time.monotonic() if now is None else now
        recent = False
        if self._last_update_at is not None:
            age = now - self._last_update_at
            recent = math.isfinite(age) and 0 <= age <= self.stale_after_s
        stream = self._current_health.get("lidar_stream", self._current_health.get("lidar"))
        self._lidar_view = self._availability.update(
            now, scan_recent=recent, stream_ok=stream is not False,
            driver=self._current_health.get("lidar_driver"))
        return self._lidar_view

    def _render_scan(self):
        # Scans are replaced, never accumulated. Skip all Canvas work while hidden.
        if self.notebook.select() != str(self.lidar_tab):
            return
        canvas = self.scan_canvas
        width, height = canvas.winfo_width(), canvas.winfo_height()
        if width < 50 or height < 50:
            return
        now = time.monotonic()
        if (width, height, self._map_range_m) != self._map_geometry:
            self._layout_map(width, height, self._map_range_m)
        cx, cy, radius = self._map_layout
        range_m = self._map_range_m
        mode, title, detail = self._refresh_availability(now)
        live = mode == "live"
        snapshot_current = live and self._snapshot_is_current()

        # Sector labels and stop arcs: cheap, change-detected every call.
        for side, name, _, _ in DIRECTIONS:
            reading = self.sector_readings[side]
            text, tone = sector_label(name, reading, live=live,
                                      single_line=side in ("FRONT", "BACK"))
            self._canvas_set(self._map_labels[side], text=text, fill=RADAR[tone])
            arc = self._map_thresholds[side]
            if not live or self._bypassed(side) or not self._map_threshold_fits[side]:
                self._canvas_set(arc, state="hidden")
                continue
            close = reading["detail"] == "Close obstacle"
            self._canvas_set(arc, state="normal", outline=RADAR["danger" if close else "threshold"],
                             width=2 if close else 1, dash="" if close else (3, 3))

        # Loading shows the sweep around the robot; unavailable and attention
        # replace the scope's content with the reason.
        loading = mode == "loading"
        for wedge in self._map_sweep:
            self._canvas_set(wedge, state="normal" if loading else "hidden")
        self._canvas_set(self._map_sweep_line, state="normal" if loading else "hidden")
        self._canvas_set(self._map_scope, fill=RADAR["scope_off" if mode == "unavailable" else "scope"])
        # The reason is written across the scope's centre: clear that space.
        explained = mode in ("unavailable", "attention")
        self._canvas_set(self._map_robot, state="hidden" if explained else "normal")
        for label in self._map_ring_labels:
            self._canvas_set(label, state="hidden" if explained else "normal")
        if loading:
            self._sweep_started_at = self._sweep_started_at or now
            self._draw_sweep(now)
            self._ensure_sweep()
        else:
            self._sweep_started_at = None

        render_key = ((width, height, range_m), mode, snapshot_current)
        if self._scan_snapshot is self._map_snapshot_rendered and render_key == self._map_render_key:
            self._render_message(mode, title, detail, None)
            return
        self._map_render_key = render_key
        # Retain the last rendered object. Several hidden updates may release
        # prior frames; Python can recycle their ids before this view reopens.
        self._map_snapshot_rendered = self._scan_snapshot
        points = scan_display_points(self._scan_snapshot, now, self.stale_after_s) if snapshot_current else ()
        visible = [(forward, left) for forward, left in points
                   if math.hypot(forward, left) <= range_m]
        nearest = min(points, key=lambda point: math.hypot(*point), default=None)
        for index, (forward, left) in enumerate(visible):
            item = self._map_points[index]
            x, y = scan_canvas_point(forward, left, cx, cy, radius / range_m)
            closest = (forward, left) == nearest
            r = 3.5 if closest else 2
            canvas.coords(item, x - r, y - r, x + r, y + r)
            self._canvas_set(item, state="normal", fill=RADAR["nearest" if closest else "point"])
        for item in self._map_points[len(visible):max(len(visible), self._map_points_shown)]:
            self._canvas_set(item, state="hidden")
        self._map_points_shown = len(visible)
        markers = (scan_display_markers(self._scan_snapshot, now, self.stale_after_s)
                   if snapshot_current else ())
        shown = [marker for marker in markers if math.hypot(marker[0], marker[1]) <= range_m]
        scale = radius / range_m
        for index, (forward, left, kind, vx, vy) in enumerate(shown):
            x, y = scan_canvas_point(forward, left, cx, cy, scale)
            line, dot, square = (self._map_marker_lines[index], self._map_marker_dots[index],
                                 self._map_marker_squares[index])
            if kind == "object":
                r = 6
                canvas.coords(square, x - r, y - r, x + r, y - r, x + r, y + r, x - r, y + r)
                self._canvas_set(square, state="normal")
                self._canvas_set(dot, state="hidden")
                self._canvas_set(line, state="hidden")
                continue
            r = 8 if kind == "person" else 6
            canvas.coords(dot, x - r, y - r, x + r, y + r)
            self._canvas_set(dot, state="normal", fill=RADAR[kind])
            self._canvas_set(square, state="hidden")
            tip_x, tip_y = scan_canvas_point(forward + vx * MARKER_VELOCITY_S,
                                             left + vy * MARKER_VELOCITY_S, cx, cy, scale)
            canvas.coords(line, x, y, tip_x, tip_y)
            self._canvas_set(line, state="normal", fill=RADAR[kind])
        for index in range(len(shown), max(len(shown), self._map_markers_shown)):
            for pool in (self._map_marker_lines, self._map_marker_dots, self._map_marker_squares):
                self._canvas_set(pool[index], state="hidden")
        self._map_markers_shown = len(shown)
        self._render_message(mode, title, detail, (points, visible))
        # Kept short: caption and info share one row on an 800 px screen.
        if not snapshot_current:
            info = "Old points hidden · no localization"
        else:
            distance = f" · nearest {math.hypot(*nearest):.2f} m" if nearest else ""
            yaw = self._scan_snapshot.yaw_offset_deg
            info = f"{len(visible)}/{len(points)} hits{distance} · mount yaw {yaw:+g}°"
            seen = marker_summary(markers)
            if seen:
                info = f"{seen} · {info}"
        self._set(self.map_info_lbl, text=info)

    def _render_message(self, mode, title, detail, drawn):
        """The scope's centre message. drawn=None keeps the live message as it was."""
        cx, cy, radius = self._map_layout
        canvas = self.scan_canvas
        if mode == "live":
            if drawn is None:
                return
            points, visible = drawn
            if not self._snapshot_is_current():
                text = "Scan geometry unavailable"
            else:
                text = ("No finite returns" if not points else
                        f"No returns inside {self._map_range_m} m")
            canvas.coords(self._map_empty, cx, cy + 27)
            self._canvas_set(self._map_empty, text=text, fill=RADAR["warning"], font=self.small_font,
                             anchor="n", state="hidden" if visible else "normal")
            self._canvas_set(self._map_detail, state="hidden")
            return
        if mode == "loading":
            canvas.coords(self._map_empty, cx, cy + max(22.0, radius * 0.4))
            canvas.coords(self._map_detail, cx, cy + max(22.0, radius * 0.4) + 22)
            self._canvas_set(self._map_empty, text=title, fill=RADAR["ok"], font=self.map_title_font,
                             anchor="n", state="normal")
            self._canvas_set(self._map_detail, text=detail, anchor="n",
                             state="normal" if detail else "hidden")
            return
        tone = "danger" if mode == "unavailable" else "warning"
        self._canvas_set(self._map_empty, text=title, fill=RADAR[tone], font=self.map_title_font,
                         anchor="n", state="normal")
        self._canvas_set(self._map_detail, text=detail, anchor="n",
                         state="normal" if detail else "hidden")
        # Centre title and wrapped reason together on the scope.
        title_height = self._text_height(self._map_empty, 20)
        detail_height = self._text_height(self._map_detail, 15) if detail else 0
        top = cy - (title_height + 6 + detail_height) / 2
        canvas.coords(self._map_empty, cx, top)
        canvas.coords(self._map_detail, cx, top + title_height + 6)

    def _text_height(self, item, default):
        try:
            box = self.scan_canvas.bbox(item)
        except (AttributeError, tk.TclError):
            return default
        return box[3] - box[1] if box else default

    def _sweep_wanted(self):
        try:
            return (not self._closed and self._lidar_view[0] == "loading"
                    and self.notebook.select() == str(self.lidar_tab)
                    and bool(self.scan_canvas.winfo_viewable()))
        except tk.TclError:
            return False

    def _ensure_sweep(self):
        if self._sweep_after_id is None and self._sweep_wanted():
            self._sweep_after_id = self.root.after(SWEEP_FRAME_MS, self._animate_sweep)

    def _animate_sweep(self):
        self._sweep_after_id = None
        if not self._sweep_wanted() or self._map_layout is None:
            return
        self._draw_sweep(time.monotonic())
        self._sweep_after_id = self.root.after(SWEEP_FRAME_MS, self._animate_sweep)

    def _draw_sweep(self, now):
        """Clockwise like an aircraft radar; the trail fades behind the beam."""
        cx, cy, radius = self._map_layout
        turn = ((now - (self._sweep_started_at or now)) / SWEEP_PERIOD_S) % 1.0
        lead = (90.0 - 360.0 * turn) % 360.0
        for index, wedge in enumerate(self._map_sweep):
            self.scan_canvas.itemconfigure(wedge, start=(lead + index * SWEEP_WEDGE_DEG) % 360.0)
        angle = math.radians(lead)
        self.scan_canvas.coords(self._map_sweep_line, cx, cy,
                                cx + radius * math.cos(angle), cy - radius * math.sin(angle))

    def _snapshot_is_current(self):
        snapshot = self._scan_snapshot
        try:
            age = time.monotonic() - snapshot.acquired_at
            return (math.isfinite(age) and 0 <= age <= self.stale_after_s
                    and math.isfinite(snapshot.yaw_offset_deg))
        except (AttributeError, TypeError, ValueError):
            return False

    def toggle_messages(self):
        # Retained for keyboard/programmatic callers; the tabs own visibility.
        self.notebook.select(self.lidar_tab if self.notebook.select() == str(self.ble_tab)
                             else self.ble_tab)
        self._tab_changed()

    def _toggle_front(self):
        self._toggle_checks(self.front_disabled)

    def _toggle_back(self):
        self._toggle_checks(self.back_disabled)

    def _toggle_sides(self):
        self._toggle_checks(self.side_disabled)

    def _toggle_checks(self, event):
        if not self.allow_safety_disable:
            return
        event.clear() if event.is_set() else event.set()
        self.on_checks_changed()
        self._refresh_safety()
        self._current_health = dict(self.read_health())
        self._refresh_start()
        if self._last_frame is not None:
            self._render_frame(stale=self._display_stale)
        self._render_scan()

    def _refresh_safety(self):
        groups = []
        if self.allow_safety_disable:
            if self.front_disabled.is_set(): groups.append("front")
            if self.back_disabled.is_set(): groups.append("back")
            if self.side_disabled.is_set(): groups.append("sides")
        bypassed = bool(groups)
        self._set(self.safety_lbl,
                  text=f"Threshold bypass: {' + '.join(groups)}" if bypassed
                  else "Threshold checks on",
                  fg=COLORS["warning" if bypassed else "ok"])
        if self.allow_safety_disable:
            for name, event, button in (("Front", self.front_disabled, self.front_btn),
                                        ("Back", self.back_disabled, self.back_btn),
                                        ("Sides", self.side_disabled, self.sides_btn)):
                self._set(button, text=f"{name}: {'read only' if event.is_set() else 'checks on'}")

    def _refresh_start(self):
        status = self._current_health.get("manual_start", {})
        enabled = bool(self.request_start and status.get("enabled"))
        self._set(self.start_btn, state="normal" if enabled else "disabled")
        self._set(self.start_status_lbl, text=status.get("message", "Manual start unavailable"),
                  fg=COLORS["ok" if enabled else "warning" if status.get("pending") else "muted"])

    def _request_start(self):
        # Fetch current readiness at click time, not the previous display poll.
        self._current_health = dict(self.read_health())
        if self.request_start and self._current_health.get("manual_start", {}).get("enabled"):
            confirmed = messagebox.askyesno(
                "Start manual disposal?",
                "Are you sure you want GARBY to go to the collection point now?\n\n"
                "Bin level, gas and weight thresholds do not need to be reached. "
                "Obstacle detection and emergency stop remain active.",
                parent=self.root, default=messagebox.NO)
            # A modal confirmation can outlive controller readiness.
            self._current_health = dict(self.read_health())
            if confirmed and self._current_health.get("manual_start", {}).get("enabled"):
                result = self.request_start()
                if result is not None:
                    self._current_health["manual_start"] = result
        self._refresh_start()

    def set_ble_status(self, text, color):
        # Called by the BLE worker. Even root.after() would cross the Tk thread
        # boundary; only the main-thread polling callback touches widgets.
        with self._ble_status_lock:
            self._pending_ble_status = (text, color)

    def _scan_is_stale(self, now=None):
        if self._last_update_at is None:
            return True
        now = time.monotonic() if now is None else now
        age = now - self._last_update_at
        return (not math.isfinite(age) or not 0 <= age <= self.stale_after_s
                or self._current_health.get("lidar_stream",
                                            self._current_health.get("lidar")) is False)

    def _poll(self):
        if self._closed:
            return
        try:
            self._poll_once()
            self._refresh_failed = False
        except Exception:
            # Tk drops a timer callback that raises. Without finally the UI
            # never refreshes again, even after the peripheral recovers.
            if not self._refresh_failed:
                logger.exception("Monitor refresh failed; retrying")
            self._refresh_failed = True
            self._set(self.start_btn, state="disabled")
            for button in (self.ultrasonic_btn, self.release_btn, self.reset_btn):
                self._set(button, state="disabled")
            for button in self.developer_buttons.values():
                self._set(button, state="disabled")
            self._hold_end()  # never keep driving on a monitor that cannot refresh
            for button in self.remote_buttons.values():
                self._set(button, state="disabled")
            self._set(self.start_status_lbl, text="Monitor refresh failed; retrying",
                      fg=COLORS["warning"])
        finally:
            if not self._closed:
                self._after_id = self.root.after(self.POLL_MS, self._poll)

    def _poll_once(self):
        self._current_health = dict(self.read_health())
        latest = None
        # Bound the work even if a producer floods a queue on a slow Pi.
        for _ in range(20):
            try:
                latest = self.data_queue.get_nowait()
            except queue.Empty:
                break
        if latest is not None:
            self.update(*latest)
        stale = self._scan_is_stale()
        if stale != self._display_stale and self._last_frame is not None:
            self._render_frame(stale=stale)
        self._display_stale = stale
        self._render_scan()
        mode = self._refresh_availability()[0]
        if mode == "live":
            status, tone = (("Scan incomplete", "warning")
                            if self._current_health.get("lidar") is False else ("Live scan", "ok"))
        elif mode == "attention":
            status, tone = "Scan needs attention", "danger"
        elif mode == "unavailable":
            status, tone = "LiDAR not available", "danger"
        elif self._last_update_at is not None:
            status, tone = "Scan stale", "warning"
        else:
            status, tone = "Waiting for scan", "warning"
        self._set(self.status_lbl, text=status, fg=COLORS[tone])
        with self._ble_status_lock:
            pending = self._pending_ble_status
            self._pending_ble_status = None
        if pending:
            text, color = pending
            tone = {"#3fb950": "ok", "#f85149": "danger", "#d29922": "warning"}.get(color, "muted")
            self._set(self.ble_status_lbl, text=text, fg=COLORS[tone])
        for key, available, unavailable in (("sensors", "Receiving", "Waiting for data"),
                                             ("firebase", "Enabled", "Unavailable")):
            connected = bool(self._current_health.get(key))
            label = available if connected else unavailable
            if key == "firebase":
                label = self._current_health.get("firebase_status", label)
            self._set(self.connection_labels[key], text=label,
                      fg=COLORS["ok" if connected else "muted"])
        state = self._current_health.get("robot_state", "UNKNOWN")
        state_title = state.replace("_", " ").capitalize()
        stopped = self._current_health.get("operator_controls", {}).get("emergency_stopped")
        leg = self._current_health.get("route_leg") or ""
        for label in (self.subtitle_lbl, self.header_subtitle_lbl):
            self._set(label, text=f"State: {state_title}" + ("\nStop latched" if stopped is True else
                                                                     f"\n{leg}" if leg else ""),
                      fg=COLORS["danger" if stopped is True else "muted"])
        hold = self._current_health.get("motion_hold", "")
        pause = self._current_health.get("operator_pause", "")
        hold_text = (f"Stopped for you to decide: {pause}" if pause else
                     "Emergency stop: motors stopped" if stopped is True else
                     f"Motors held: {hold_in_words(hold)}" if hold else "")
        for label in (self.motion_hold_lbl, self.header_motion_hold_lbl):
            self._set(label, text=hold_text,
                      fg=COLORS["danger" if stopped is True else "warning"])
        if hold_text:
            self.motion_hold_lbl.grid()
        else:
            self.motion_hold_lbl.grid_remove()
        self._refresh_header_status()
        self._refresh_safety()
        self._refresh_start()
        self._refresh_operator_controls()
        self._refresh_developer_controls()
        self._refresh_remote_controls()
        self._refresh_runstart_controls()
        self._update_sensor_readings()
        self._process_ble_recv()
        self._process_log()

    def _update_sensor_readings(self):
        readings = self.read_sensors()
        for key, label in self.sensor_labels.items():
            reading = sensor_reading(key, readings.get(key))
            self._set(label, text=reading["text"], fg=COLORS[reading["tone"]])
            self._set(self.sensor_status_labels[key], text=reading["status"])

    def _clear_ble_log(self):
        previous_state = self.log_text.cget("state")
        self.log_text.configure(state="normal")
        try:
            self.log_text.delete("1.0", "end")
        finally:
            self.log_text.configure(state=previous_state)

    def _process_ble_recv(self):
        lines = []
        for _ in range(40):
            try:
                message = self.recv_queue.get_nowait()
            except queue.Empty:
                break
            lines.append(f"[{datetime.now():%H:%M:%S}] {message}\n")
        if not lines:
            return
        at_bottom = self.log_text.yview()[1] >= 0.99
        self.log_text.configure(state="normal")
        self.log_text.insert("end", "".join(lines))
        excess = int(self.log_text.index("end-1c").split(".")[0]) - (self.LOG_LIMIT + 1)
        if excess > 0:
            self.log_text.delete("1.0", f"{excess + 1}.0")
        if at_bottom:
            self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def update(self, data, range_min, range_max, cone_deg, sides_status=None,
               scan_snapshot=None, source_acquired_at=_NO_SOURCE_TIME):
        self._last_frame = (data, range_min, range_max, cone_deg, sides_status)
        self._scan_snapshot = scan_snapshot
        # Acquisition belongs to the summaries even if optional drawing geometry
        # fails. Only legacy frames without either source age use receipt time.
        if source_acquired_at is _NO_SOURCE_TIME:
            source_acquired_at = (time.monotonic() if scan_snapshot is None else
                                  getattr(scan_snapshot, "acquired_at", None))
        self._last_update_at = None
        try:
            if (isinstance(source_acquired_at, (float, int))
                    and not isinstance(source_acquired_at, bool)
                    and math.isfinite(source_acquired_at)):
                self._last_update_at = float(source_acquired_at)
        except (ValueError, OverflowError):
            pass
        stale = self._scan_is_stale()
        self._render_frame(stale=stale)
        self._display_stale = stale
        self._render_scan()

    def _render_frame(self, *, stale):
        data, range_min, range_max, cone_deg, _ = self._last_frame
        # The map draws these around the scope; _render_scan places them.
        for side, *_ in DIRECTIONS:
            self.sector_readings[side] = direction_reading(
                data.get(side), self._threshold_cm(side), bypassed=self._bypassed(side), stale=stale)
        # Kept to one short line: it must fit an 800 px wide screen with zones.
        self._set(self.footer, text=f"LiDAR {range_min * 100:g}-{range_max * 100:g} cm   |   "
                  f"{wall_fit_text(None if stale else self._last_frame[4])}")

    def _on_destroy(self, event):
        if event.widget is self.root:
            self._closed = True
            if self._after_id is not None:
                self.root.after_cancel(self._after_id)
                self._after_id = None
            if self._sweep_after_id is not None:
                self.root.after_cancel(self._sweep_after_id)
                self._sweep_after_id = None
