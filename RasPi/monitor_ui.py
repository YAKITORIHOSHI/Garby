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


def scan_canvas_point(forward_m, left_m, center_x, center_y, pixels_per_m):
    """Robot frame: +forward goes up and +left goes left on the display."""
    return (center_x - left_m * pixels_per_m, center_y - forward_m * pixels_per_m)


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
                 read_health=None, front_disabled=None, back_disabled=None,
                 allow_safety_disable=False, front_threshold_cm=95.0,
                 front_diagonal_threshold_cm=60.0,
                 back_threshold_cm=35.0, side_threshold_cm=22.0,
                 stale_after_s=0.8, on_checks_changed=None, request_start=None,
                 request_emergency_stop=None, request_release_stop=None,
                 request_ultrasonic=None, request_system_reset=None):
        self.root, self.data_queue, self.recv_queue = root, data_queue, recv_queue
        self.read_sensors = read_sensors or (lambda: {})
        self.read_health = read_health or (lambda: {})
        self.front_disabled = front_disabled or threading.Event()
        self.back_disabled = back_disabled or threading.Event()
        self.allow_safety_disable = allow_safety_disable
        self.on_checks_changed = on_checks_changed or (lambda: None)
        self.request_start = request_start
        self.request_emergency_stop = request_emergency_stop
        self.request_release_stop = request_release_stop
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
        for button_style in ("Monitor.TButton", "Compact.Monitor.TButton",
                             "Emergency.Monitor.TButton"):
            style.layout(button_style, _layout_without_focus(style.layout("TButton")))

        root.grid_columnconfigure(0, weight=1)
        root.grid_rowconfigure(1, weight=1)
        header = tk.Frame(root, bg=COLORS["background"])
        header.grid(row=0, column=0, sticky="ew", padx=16, pady=(10, 8))
        header.grid_columnconfigure(1, weight=1)
        self._label(header, "GARBY", font=self.title_font,
                    bg=COLORS["background"]).grid(row=0, column=0)
        self.subtitle_lbl = self._label(header, "Waiting for both controllers", tone="muted",
                                        bg=COLORS["background"], font=self.small_font,
                                        wraplength=160, width=24, height=2, justify="left")
        self.subtitle_lbl.grid(row=0, column=1, sticky="w", padx=(12, 8))
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
        ttk.Button(header, text="Full screen", style="Monitor.TButton",
                   command=self.toggle_fullscreen).grid(row=0, column=3)

        self.notebook = ttk.Notebook(root, style="Monitor.TNotebook", takefocus=False)
        self.notebook.grid(row=1, column=0, sticky="nsew", padx=12)
        self.lidar_tab = tk.Frame(self.notebook, bg=COLORS["background"])
        self.ble_tab = tk.Frame(self.notebook, bg=COLORS["background"])
        self.controls_tab = tk.Frame(self.notebook, bg=COLORS["background"])
        self.notebook.add(self.lidar_tab, text="LiDAR readings")
        self.notebook.add(self.ble_tab, text="BLE status & messages")
        self.notebook.add(self.controls_tab, text="Controls")
        self.notebook.bind("<<NotebookTabChanged>>", self._tab_changed)
        body = self.lidar_tab
        body.grid_columnconfigure(0, weight=1)
        body.grid_columnconfigure(1, minsize=230)
        body.grid_rowconfigure(0, weight=1)
        surroundings = self._panel(body)
        surroundings.grid(row=0, column=0, sticky="nsew", padx=(0, 10), pady=6)
        surroundings.grid_columnconfigure(0, weight=1)
        surroundings.grid_rowconfigure(1, weight=1)
        bar = tk.Frame(surroundings, bg=COLORS["surface"])
        bar.grid(row=0, column=0, sticky="ew", padx=12, pady=(6, 2))
        self._label(bar, "Surroundings", font=self.heading_font).pack(side="left")
        self.map_range_btn = ttk.Button(bar, text="Range: 4 m",
                                        style="Compact.Monitor.TButton", command=self._cycle_map_range)
        self.map_range_btn.pack(side="left", padx=10)
        self.status_lbl = self._label(bar, "Waiting for scan", tone="warning", font=self.small_font)
        self.status_lbl.pack(side="right")

        # One live LiDAR map replaces the eight number cards: the eight sector
        # distances are written around the scope, each on its own side.
        self.scan_canvas = tk.Canvas(surroundings, bg=RADAR["canvas"],
                                     highlightthickness=0, width=100, height=100)
        self.scan_canvas.grid(row=1, column=0, sticky="nsew", padx=8, pady=2)
        self.scan_canvas.bind("<Configure>", self._resize_scan)
        self._build_map_items()
        map_footer = tk.Frame(surroundings, bg=COLORS["surface"])
        map_footer.grid(row=2, column=0, sticky="ew", padx=12)
        map_footer.grid_columnconfigure(1, weight=1)
        self.map_caption_lbl = self._label(map_footer, "Robot centred · front up · metres",
                                           tone="muted", font=self.small_font)
        self.map_caption_lbl.grid(row=0, column=0, sticky="w")
        self.map_info_lbl = self._label(map_footer, "No position history or localization",
                                        tone="muted", font=self.small_font)
        self.map_info_lbl.grid(row=0, column=1, sticky="e", padx=(12, 0))

        safety = tk.Frame(surroundings, bg=COLORS["surface"])
        safety.grid(row=3, column=0, sticky="ew", padx=12, pady=(2, 8))
        safety.grid_columnconfigure(0, weight=1)
        self.safety_lbl = self._label(safety, "Threshold checks on", tone="ok", font=self.small_font)
        self.safety_lbl.grid(row=0, column=1, sticky="e")
        sensor_panel = self._panel(body)
        sensor_panel.grid(row=0, column=1, sticky="nsew", pady=6)
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
        self._label(sensor_panel, "Bin sensors", font=self.heading_font).grid(
            row=0, column=0, sticky="w", padx=12, pady=(10, 12))
        sensor_grid = tk.Frame(sensor_panel, bg=COLORS["surface"])
        sensor_grid.grid(row=1, column=0, sticky="nsew", padx=12)
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

        self._label(sensor_panel, f"LiDAR stop distances (red arcs)\nFront {front_threshold_cm:g} / "
                    f"front diagonals {front_diagonal_threshold_cm:g} / rear "
                    f"{back_threshold_cm:g} / sides {side_threshold_cm:g} cm\n"
                    "plus chassis zones (footer)",
                    font=self.small_font, tone="muted", wraplength=205, justify="left").grid(
                        row=2, column=0, sticky="w", padx=12, pady=(18, 8))

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

        self.footer = self._label(root, "Waiting for LiDAR data", tone="muted",
                                  font=self.small_font, bg=COLORS["background"])
        self.footer.grid(row=3, column=0, sticky="w", padx=16, pady=7)
        self._poll()

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
            self.front_btn = ttk.Button(checks, style="Monitor.TButton", command=self._toggle_front)
            self.front_btn.pack(side="left", padx=(0, 6))
            self.back_btn = ttk.Button(checks, style="Monitor.TButton", command=self._toggle_back)
            self.back_btn.pack(side="left")
        self.safety_note_lbl = self._label(
            left, "Read only keeps distances visible and bypasses that group's obstacle "
            "thresholds. Motion can continue; side and stale-scan stops stay active. "
            "LiDAR sees range returns; it does not identify people.", font=self.small_font,
            tone="warning", wraplength=330, justify="left")
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
        self.start_btn = ttk.Button(motion, text="Start (skip scale/sensors)",
                                    style="Monitor.TButton", command=self._request_start,
                                    state="disabled")
        self.start_btn.pack(side="left", padx=(0, 6))
        self.release_btn = ttk.Button(motion, text="Release stop", style="Monitor.TButton",
                                      command=self._request_release_stop, state="disabled")
        self.release_btn.pack(side="left")
        self.start_status_lbl = self._label(right, "Waiting for both controllers", tone="muted",
                                            font=self.small_font, wraplength=330, justify="left", height=2)
        self.start_status_lbl.grid(row=2, column=0, sticky="ew", padx=12, pady=(4, 0))
        self.stop_status_lbl = self._label(right, "Emergency stop: unknown", font=self.small_font,
                                           tone="muted", wraplength=330, justify="left")
        self.stop_status_lbl.grid(row=3, column=0, sticky="w", padx=12, pady=(2, 4))
        self._label(right, "Release aborts the route to idle. Start a new route explicitly.",
                    font=self.small_font, tone="muted", wraplength=330, justify="left").grid(
                        row=4, column=0, sticky="w", padx=12, pady=(0, 8))
        ttk.Separator(right).grid(row=5, column=0, sticky="ew", padx=12)
        self.reset_btn = ttk.Button(right, text="Restart Pi + bridge + executor…", style="Monitor.TButton",
                                    command=self._request_system_reset, state="disabled")
        self.reset_btn.grid(row=6, column=0, sticky="w", padx=12, pady=(8, 4))
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
        self._render_scan()

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
            or (side in self.BACK_GROUP and self.back_disabled.is_set())))

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
        self._render_message(mode, title, detail, (points, visible))
        # Kept short: caption and info share one row on an 800 px screen.
        if not snapshot_current:
            info = "Old points hidden · no localization"
        else:
            distance = f" · nearest {math.hypot(*nearest):.2f} m" if nearest else ""
            yaw = self._scan_snapshot.yaw_offset_deg
            info = f"{len(visible)}/{len(points)} hits{distance} · mount yaw {yaw:+g}°"
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
        bypassed = bool(groups)
        self._set(self.safety_lbl,
                  text=f"Threshold bypass: {' + '.join(groups)}" if bypassed
                  else "Threshold checks on",
                  fg=COLORS["warning" if bypassed else "ok"])
        if self.allow_safety_disable:
            for name, event, button in (("All-front", self.front_disabled, self.front_btn),
                                        ("All-back", self.back_disabled, self.back_btn)):
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
        self._set(self.subtitle_lbl, text=f"State: {state_title}" + ("\nStop latched" if stopped is True else ""),
                  fg=COLORS["danger" if stopped is True else "muted"])
        self._refresh_safety()
        self._refresh_start()
        self._refresh_operator_controls()
        self._update_sensor_readings()
        self._process_ble_recv()

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
