"""Exercise monitor presentation and its thread boundary without robot hardware."""

import raspi_path  # noqa: F401  (puts the deployable RasPi/ folder on sys.path)
import math
import asyncio
import queue
import tempfile
import threading
import time
import types
import logging
import unittest
from unittest.mock import patch

try:
    import tkinter as tk
except ImportError:
    tk = None
if tk is not None:
    import monitor_ui
else:
    monitor_ui = None


@unittest.skipIf(monitor_ui is None, "Tkinter is not installed")
class ReadingTests(unittest.TestCase):
    def test_circular_scan_uses_acquisition_age_and_only_finite_measured_points(self):
        snapshot = types.SimpleNamespace(points_m=((1, 0), (0, 1), (math.inf, 0),
                                                   (math.nan, 1)), acquired_at=100,
                                         yaw_offset_deg=22)
        self.assertEqual(monitor_ui.scan_display_points(snapshot, 100.1, .8),
                         ((1, 0), (0, 1)))
        self.assertEqual(monitor_ui.scan_display_points(snapshot, 100.81, .8), ())
        self.assertEqual(monitor_ui.scan_display_points(snapshot, 99, .8), ())
        self.assertEqual(monitor_ui.scan_display_points(None, 100, .8), ())

    def test_circular_projection_keeps_front_up_and_left_left(self):
        self.assertEqual(monitor_ui.scan_canvas_point(1, 0, 100, 100, 20), (100, 80))
        self.assertEqual(monitor_ui.scan_canvas_point(0, 1, 100, 100, 20), (80, 100))
        self.assertEqual(monitor_ui.scan_canvas_point(-1, -1, 100, 100, 20), (120, 120))

    def test_circular_point_work_is_bounded(self):
        snapshot = types.SimpleNamespace(points_m=((1, 0),) * 10000, acquired_at=100,
                                         yaw_offset_deg=0)
        self.assertEqual(len(monitor_ui.scan_display_points(snapshot, 100, .8)), 360)

    def test_missing_and_invalid_distance_never_look_live(self):
        for info in (None, (math.nan, 1, 1), (math.inf, 1, 1), (-1, 1, 1)):
            with self.subTest(info=info):
                reading = monitor_ui.direction_reading(info, 95)
                self.assertEqual(reading["text"], "-- cm")
                self.assertEqual(reading["detail"], "No reading")

    def test_close_threshold_is_inclusive_and_direction_specific(self):
        for distance_m, threshold, close in ((.95, 95, True), (.96, 95, False),
                                             (.35, 35, True), (.36, 35, False),
                                             (.22, 22, True), (.23, 22, False)):
            with self.subTest(distance_m=distance_m, threshold=threshold):
                reading = monitor_ui.direction_reading((distance_m, distance_m, 3), threshold)
                self.assertEqual(reading["tone"] == "danger", close)
                if close:
                    self.assertEqual(reading["detail"], "Close obstacle")

    def test_stale_snapshot_suppresses_distances_even_when_bypassed(self):
        reading = monitor_ui.direction_reading((2, 2, 3), 95, stale=True, bypassed=True)
        self.assertEqual(reading, {"text": "-- cm", "detail": "Scan stale", "tone": "warning"})

    def test_bypass_has_an_explicit_written_state(self):
        reading = monitor_ui.direction_reading((2, 2, 3), 95, bypassed=True)
        self.assertEqual(reading["detail"], "Reading only")
        self.assertEqual(reading["tone"], "warning")

    def test_transport_sentinels_and_invalid_sensor_values_show_unavailable(self):
        for key in ("ultrasonic", "mq4", "mq137", "mq135"):
            sentinel = 999 if key == "ultrasonic" else -1
            for value in (None, sentinel, math.nan, math.inf, True):
                with self.subTest(key=key, value=value):
                    reading = monitor_ui.sensor_reading(key, value)
                    self.assertEqual(reading["text"], "--")
                    self.assertEqual(reading["status"], "Unavailable")

    def test_zero_is_a_real_reading_and_gas_values_have_no_invented_alarm(self):
        self.assertEqual(monitor_ui.sensor_reading("ultrasonic", 0)["text"], "0.0 cm")
        self.assertEqual(monitor_ui.sensor_reading("mq4", 0)["text"], "0")
        self.assertEqual(monitor_ui.sensor_reading("mq4", 800)["status"], "Reading")
        self.assertEqual(monitor_ui.sensor_reading("mq4", 800)["tone"], "text")


@unittest.skipIf(monitor_ui is None, "Tkinter is not installed")
class DeveloperTurnConfirmationTests(unittest.TestCase):
    """Exercise the real modal callback even when no X display can be opened."""

    def test_floor_prompt_cancellation_dispatch_and_mpu_gate_without_a_display(self):
        calls = []
        gui = object.__new__(monitor_ui.LidarGUI)
        gui.read_health = lambda: {"developer_tests": {"enabled": True}}
        gui.developer_gate_vars = {bit: types.SimpleNamespace(get=lambda: True) for bit in (1, 2, 4)}
        gui.developer_window = object()
        gui.developer_buttons = {
            action: types.SimpleNamespace(cget=lambda key, text=text: text)
            for action, text in (("L90", "Turn left 90°"), ("R90", "Turn right 90°"))
        }
        gui._refresh_developer_controls = lambda: None
        gui.request_developer_test = lambda action, mask: calls.append((action, mask))
        for action in ("L90", "R90"):
            with patch.object(monitor_ui.messagebox, "askyesno", return_value=False) as confirm:
                gui._request_developer_test(action)
            self.assertEqual(calls, [])
            self.assertIn("must be on the floor", confirm.call_args.args[1])
            self.assertIn("Do not lift the motors or wheels", confirm.call_args.args[1])
            self.assertIn("MPU detects the robot turning", confirm.call_args.args[1])
            self.assertEqual(confirm.call_args.kwargs["default"], monitor_ui.messagebox.NO)
        with patch.object(monitor_ui.messagebox, "askyesno", return_value=True):
            gui._request_developer_test("L90")
            gui._request_developer_test("R90")
        self.assertEqual(calls, [("L90", 7), ("R90", 7)])
        gui.developer_gate_vars[2].get = lambda: False
        with patch.object(monitor_ui.messagebox, "askyesno") as confirm:
            gui._request_developer_test("L90")
            gui._request_developer_test("R90")
        confirm.assert_not_called()
        self.assertEqual(len(calls), 2)


@unittest.skipIf(monitor_ui is None, "Tkinter is not installed")
class AvailabilityTests(unittest.TestCase):
    """What the map shows while the LiDAR is found, lost or failing."""

    def test_loading_names_the_driver_phase_then_times_out_as_unavailable(self):
        view = monitor_ui.LidarAvailability(timeout_s=45)
        self.assertEqual(view.update(10, scan_recent=False, stream_ok=True)[:2],
                         ("loading", "Waiting for first scan"))
        for phase, title in (("searching", "Searching for LiDAR"), ("starting", "Starting LiDAR"),
                             ("running", "Waiting for first scan")):
            with self.subTest(phase=phase):
                self.assertEqual(view.update(20, scan_recent=False, stream_ok=True,
                                             driver={"phase": phase})[:2], ("loading", title))
        mode, title, detail = view.update(55, scan_recent=False, stream_ok=True,
                                          driver={"phase": "running"})
        self.assertEqual((mode, title), ("unavailable", "LiDAR not available"))
        self.assertIn("No LiDAR scan received in 45 s", detail)

    def test_reported_failure_stays_shown_through_retries_until_a_live_scan(self):
        view = monitor_ui.LidarAvailability()
        reason = "No serial port is streaming LiDAR data; check its USB cable and power"
        mode, _, detail = view.update(1, scan_recent=False, stream_ok=True,
                                      driver={"phase": "failed", "message": reason})
        self.assertEqual(mode, "unavailable")
        self.assertIn(reason, detail)
        mode, _, detail = view.update(3, scan_recent=False, stream_ok=True,
                                      driver={"phase": "searching"})
        self.assertEqual(mode, "unavailable")  # no flicker back to loading per retry
        self.assertIn("Retrying: looking for the LiDAR", detail)
        self.assertEqual(view.update(4, scan_recent=True, stream_ok=True,
                                     driver={"phase": "running"})[0], "live")
        self.assertEqual(view.update(5, scan_recent=False, stream_ok=True,
                                     driver={"phase": "running"})[:2],
                         ("loading", "Reconnecting LiDAR"))

    def test_fresh_but_unusable_scan_needs_attention_not_loading(self):
        view = monitor_ui.LidarAvailability()
        mode, title, detail = view.update(1, scan_recent=True, stream_ok=False)
        self.assertEqual((mode, title), ("attention", "Scan needs attention"))
        self.assertIn("motion is held", detail)

    def test_malformed_driver_status_is_ignored(self):
        view = monitor_ui.LidarAvailability()
        for driver in (None, "failed", {"phase": ["failed"]}, {"phase": None}, {}):
            with self.subTest(driver=driver):
                self.assertEqual(view.update(1, scan_recent=False, stream_ok=True,
                                             driver=driver)[0], "loading")

    def test_sector_labels_write_the_state_that_color_reinforces(self):
        close = monitor_ui.direction_reading((.5, .5, 3), 95)
        self.assertEqual(monitor_ui.sector_label("Front", close, live=True, single_line=True),
                         ("Front  50 cm · close", "danger"))
        bypassed = monitor_ui.direction_reading((.5, .5, 3), 95, bypassed=True)
        self.assertEqual(monitor_ui.sector_label("Rear left", bypassed, live=True),
                         ("Rear left\n50 cm · read only", "warning"))
        clear = monitor_ui.direction_reading((2, 2, 3), 22)
        self.assertEqual(monitor_ui.sector_label("Left", clear, live=True), ("Left\n200 cm", "ok"))
        self.assertEqual(monitor_ui.sector_label("Left", clear, live=False), ("Left\n--", "muted"))

    def test_scope_geometry_keeps_front_up_left_left_and_arcs_on_their_sector(self):
        for angle, expected in ((0, (0, -10)), (90, (-10, 0)), (180, (0, 10)), (270, (10, 0))):
            with self.subTest(angle=angle):
                dx, dy = monitor_ui.scope_offset(angle, 10)
                self.assertAlmostEqual(dx, expected[0])
                self.assertAlmostEqual(dy, expected[1])
        # Tk measures arcs counter-clockwise from 3 o'clock; FRONT spans 12 o'clock.
        self.assertEqual(monitor_ui.canvas_arc_start(0), 67.5)
        self.assertEqual(monitor_ui.canvas_arc_start(90), 157.5)
        self.assertEqual(monitor_ui.canvas_arc_start(270), 337.5)

    def test_sweep_trail_fades_from_beam_to_scope(self):
        colors = monitor_ui.sweep_colors()
        self.assertEqual(len(colors), monitor_ui.SWEEP_WEDGES)
        self.assertEqual(colors[-1], monitor_ui.RADAR["scope"])
        brightness = [sum(int(color[i:i + 2], 16) for i in (1, 3, 5)) for color in colors]
        self.assertEqual(brightness, sorted(brightness, reverse=True))


@unittest.skipIf(monitor_ui is None, "Tkinter is not installed")
class CircularCanvasTests(unittest.TestCase):
    """Exercise the production renderer without needing a window server."""

    class Canvas:
        def __init__(self):
            self.items = {}
            self.changes = []
            self.viewable = True

        def _create(self, kind, *coords, **options):
            item = len(self.items) + 1
            self.items[item] = {"kind": kind, "coords": coords, **options}
            return item

        def create_oval(self, *coords, **options):
            return self._create("oval", *coords, **options)

        def create_line(self, *coords, **options):
            return self._create("line", *coords, **options)

        def create_text(self, *coords, **options):
            return self._create("text", *coords, **options)

        def create_arc(self, *coords, **options):
            return self._create("arc", *coords, **options)

        def create_polygon(self, *coords, **options):
            return self._create("polygon", *coords, **options)

        def winfo_width(self):
            return 480

        def winfo_height(self):
            return 320

        def winfo_viewable(self):
            return self.viewable

        def coords(self, item, *coords):
            self.items.setdefault(item, {})["coords"] = coords
            self.changes.append((item, coords))

        def itemconfigure(self, item, **options):
            self.items.setdefault(item, {}).update(options)
            self.changes.append((item, options))

    def renderer(self, points=((1, 0), (0, 1), (-1, 0), (0, -1))):
        gui = monitor_ui.LidarGUI.__new__(monitor_ui.LidarGUI)
        gui.lidar_tab = "lidar"
        gui.notebook = types.SimpleNamespace(select=lambda: "lidar")
        gui.scan_canvas = self.Canvas()
        gui.scheduled = []
        gui.root = types.SimpleNamespace(
            after=lambda delay, callback: gui.scheduled.append(callback) or len(gui.scheduled),
            after_cancel=lambda after_id: None)
        gui._closed = False
        gui.small_font = gui.map_label_font = gui.map_title_font = "TkDefaultFont"
        gui.allow_safety_disable = False
        gui.front_disabled, gui.back_disabled = threading.Event(), threading.Event()
        gui.side_disabled = threading.Event()
        gui.front_threshold_cm, gui.front_diagonal_threshold_cm = 95.0, 60.0
        gui.back_threshold_cm, gui.side_threshold_cm = 35.0, 22.0
        gui._init_map_state()
        gui._build_map_items()
        gui._scan_snapshot = types.SimpleNamespace(points_m=points, acquired_at=100, yaw_offset_deg=30)
        gui._last_update_at, gui.stale_after_s = 100, .8
        gui._current_health, gui._widget_values = {"lidar": True}, {}
        gui.map_info_lbl = types.SimpleNamespace()
        gui.map_info_lbl.configure = lambda **options: vars(gui.map_info_lbl).update(options)
        gui.footer = types.SimpleNamespace(configure=lambda **options: None)
        # SimpleNamespace is unhashable, while Tk labels are hashable.
        gui._set = lambda widget, **options: widget.configure(**options)
        return gui

    def frame(self, gui, distances, stale=False):
        data = {side: (distance, distance, 3) for side, distance in distances.items()}
        gui._last_frame = (data, .1, 8, 22, "SIDES:STABLE")
        gui._render_frame(stale=stale)

    def text(self, gui, item):
        return gui.scan_canvas.items[item].get("text")

    def test_real_renderer_projects_measured_points_and_reuses_items(self):
        gui = self.renderer()
        canvas = gui.scan_canvas
        with patch.object(monitor_ui.time, "monotonic", return_value=100.1):
            gui._render_scan()
            front, left, rear, right = [canvas.items[item]["coords"] for item in gui._map_points[:4]]
            self.assertLess((front[1] + front[3]) / 2, 160)
            self.assertLess((left[0] + left[2]) / 2, 240)
            self.assertGreater((rear[1] + rear[3]) / 2, 160)
            self.assertGreater((right[0] + right[2]) / 2, 240)
            self.assertIn("nearest 1.00 m", gui.map_info_lbl.text)
            self.assertIn("mount yaw +30°", gui.map_info_lbl.text)
            known_items = set(canvas.items)
            canvas.changes.clear()
            gui._render_scan()
            self.assertEqual(canvas.changes, [])
            gui._scan_snapshot = types.SimpleNamespace(points_m=((.5, .5),) * 10000,
                                                       acquired_at=100, yaw_offset_deg=0)
            gui._render_scan()
        self.assertEqual(set(canvas.items), known_items)
        self.assertEqual(sum(canvas.items[item]["state"] == "normal" for item in gui._map_points), 360)

    def test_markers_draw_a_red_dot_for_a_likely_person_and_hide_with_the_scan(self):
        gui = self.renderer()
        canvas = gui.scan_canvas
        gui._scan_snapshot = types.SimpleNamespace(
            points_m=((1, 0),), acquired_at=100, yaw_offset_deg=0,
            markers=((2.0, 0.5, "person", -1.2, 0.0), (1.0, -1.0, "moving", 0.0, 0.6),
                     (3.0, 0.0, "object", 0.0, 0.0), (1.0, 1.0, "ghost", 0.0, 0.0),
                     (math.nan, 0.0, "person", 0.0, 0.0), (9.0, 0.0, "person", 0.0, 0.0)))
        with patch.object(monitor_ui.time, "monotonic", return_value=100.1):
            gui._render_scan()
            dot, line = canvas.items[gui._map_marker_dots[0]], canvas.items[gui._map_marker_lines[0]]
            self.assertEqual((dot["state"], dot["fill"]), ("normal", monitor_ui.RADAR["person"]))
            # Left of centre and above it (ahead); the line points back at the robot.
            self.assertLess((dot["coords"][0] + dot["coords"][2]) / 2, 240)
            self.assertLess((dot["coords"][1] + dot["coords"][3]) / 2, 160)
            self.assertGreater(line["coords"][3], line["coords"][1])
            self.assertEqual(canvas.items[gui._map_marker_dots[1]]["fill"], monitor_ui.RADAR["moving"])
            self.assertEqual(canvas.items[gui._map_marker_squares[2]]["state"], "normal")
            self.assertEqual(canvas.items[gui._map_marker_dots[2]]["state"], "hidden")
            # Unknown kinds and invalid numbers are dropped; one beyond the 4 m
            # view is counted but not drawn.
            self.assertEqual(sum(canvas.items[item]["state"] == "normal"
                                 for item in gui._map_marker_dots), 2)
            self.assertIn("2 likely people · 1 moving object · 1 object", gui.map_info_lbl.text)
        with patch.object(monitor_ui.time, "monotonic", return_value=100.81):
            gui._render_scan()
        for pool in (gui._map_marker_dots, gui._map_marker_lines, gui._map_marker_squares):
            self.assertTrue(all(canvas.items[item]["state"] == "hidden" for item in pool))

    def test_a_snapshot_without_markers_draws_none(self):
        gui = self.renderer()
        with patch.object(monitor_ui.time, "monotonic", return_value=100.1):
            gui._render_scan()
        self.assertTrue(all(gui.scan_canvas.items[item]["state"] == "hidden"
                            for item in gui._map_marker_dots))
        self.assertNotIn("person", gui.map_info_lbl.text)

    def test_real_renderer_hides_expired_or_invalid_scan_and_never_invents_walls(self):
        gui = self.renderer()
        with patch.object(monitor_ui.time, "monotonic", return_value=100.1):
            gui._render_scan()
        with patch.object(monitor_ui.time, "monotonic", return_value=100.81):
            gui._render_scan()
        self.assertTrue(all(gui.scan_canvas.items[item]["state"] == "hidden" for item in gui._map_points))
        self.assertEqual(self.text(gui, gui._map_empty), "Reconnecting LiDAR")
        self.assertIn("Old points hidden", gui.map_info_lbl.text)
        gui._scan_snapshot = types.SimpleNamespace(points_m=((math.inf, 0), (math.nan, 0)),
                                                  acquired_at=101, yaw_offset_deg=0)
        gui._last_update_at = 101
        with patch.object(monitor_ui.time, "monotonic", return_value=101.1):
            gui._render_scan()
        self.assertTrue(all(gui.scan_canvas.items[item]["state"] == "hidden" for item in gui._map_points))
        self.assertEqual(self.text(gui, gui._map_empty), "No finite returns")

    def test_hidden_tab_does_no_canvas_work(self):
        gui = self.renderer()
        for tab in ("controls", "ble"):
            gui.notebook.select = lambda: tab
            gui._render_scan()
        self.assertEqual(gui.scan_canvas.changes, [])

    def test_hidden_updates_retain_last_rendered_object_then_draw_newest(self):
        gui = self.renderer()
        with patch.object(monitor_ui.time, "monotonic", return_value=100.1):
            gui._render_scan()
            rendered = gui._map_snapshot_rendered
            gui.notebook.select = lambda: "ble"
            for distance in (2, 3, .5):
                gui._scan_snapshot = types.SimpleNamespace(points_m=((distance, 0),),
                                                           acquired_at=100, yaw_offset_deg=0)
                gui._render_scan()
            self.assertIs(gui._map_snapshot_rendered, rendered)
            gui.scan_canvas.changes.clear()
            gui.notebook.select = lambda: "lidar"
            gui._render_scan()
        self.assertTrue(gui.scan_canvas.changes)
        self.assertIs(gui._map_snapshot_rendered, gui._scan_snapshot)
        self.assertIn("nearest 0.50 m", gui.map_info_lbl.text)

    def test_all_eight_sectors_are_written_on_their_own_side_of_the_scope(self):
        gui = self.renderer()
        distances = {side: 1 + index / 10 for index, (side, *_) in enumerate(monitor_ui.DIRECTIONS)}
        self.frame(gui, distances)
        with patch.object(monitor_ui.time, "monotonic", return_value=100.1):
            gui._render_scan()
        cx, cy, radius = gui._map_layout
        for side, name, angle, _ in monitor_ui.DIRECTIONS:
            with self.subTest(side=side):
                item = gui.scan_canvas.items[gui._map_labels[side]]
                self.assertTrue(item["text"].startswith(name))
                self.assertIn(f"{distances[side] * 100:.0f} cm", item["text"])
                x, y = item["coords"]
                forward, left = math.cos(math.radians(angle)), math.sin(math.radians(angle))
                self.assertGreater(math.hypot(x - cx, y - cy), radius)
                if abs(forward) > .5:
                    self.assertEqual(y < cy, forward > 0)
                if abs(left) > .5:
                    self.assertEqual(x < cx, left > 0)

    def test_close_sector_says_close_and_highlights_only_its_stop_arc(self):
        gui = self.renderer()
        distances = {side: 2.0 for side, *_ in monitor_ui.DIRECTIONS}
        distances["FRONT_LEFT"] = .55  # inside the 60 cm front-diagonal limit
        distances["FRONT"] = .96       # just outside the 95 cm front limit
        self.frame(gui, distances)
        gui._map_range_m = 8
        with patch.object(monitor_ui.time, "monotonic", return_value=100.1):
            gui._render_scan()
        items = gui.scan_canvas.items
        self.assertEqual(items[gui._map_labels["FRONT_LEFT"]]["text"], "Front left\n55 cm · close")
        self.assertEqual(items[gui._map_labels["FRONT_LEFT"]]["fill"], monitor_ui.RADAR["danger"])
        self.assertEqual(items[gui._map_labels["FRONT"]]["text"], "Front  96 cm")
        self.assertEqual(items[gui._map_thresholds["FRONT_LEFT"]]["outline"], monitor_ui.RADAR["danger"])
        self.assertEqual(items[gui._map_thresholds["FRONT"]]["outline"], monitor_ui.RADAR["threshold"])
        # At 8 m the 22 cm side limit is under 6 px: hidden rather than a dot.
        self.assertEqual(items[gui._map_thresholds["LEFT"]]["state"], "hidden")
        gui._map_range_m = 2
        with patch.object(monitor_ui.time, "monotonic", return_value=100.1):
            gui._render_scan()
        self.assertEqual(items[gui._map_thresholds["LEFT"]]["state"], "normal")

    def test_bypassed_group_keeps_readings_says_read_only_and_drops_its_arcs(self):
        gui = self.renderer()
        gui.allow_safety_disable = True
        gui.front_disabled.set()
        self.frame(gui, {side: .3 for side, *_ in monitor_ui.DIRECTIONS})
        with patch.object(monitor_ui.time, "monotonic", return_value=100.1):
            gui._render_scan()
        items = gui.scan_canvas.items
        for side in gui.FRONT_GROUP:
            self.assertIn("30 cm · read only", items[gui._map_labels[side]]["text"])
            self.assertEqual(items[gui._map_thresholds[side]]["state"], "hidden")
        self.assertIn("30 cm · close", items[gui._map_labels["BACK"]]["text"])
        self.assertEqual(items[gui._map_thresholds["BACK"]]["state"], "normal")

    def test_loading_sweeps_only_while_the_map_is_visible(self):
        gui = self.renderer()
        gui._last_update_at = None
        gui._current_health = {"lidar_driver": {"phase": "searching"}}
        with patch.object(monitor_ui.time, "monotonic", return_value=50.0):
            gui._render_scan()
        items = gui.scan_canvas.items
        self.assertEqual(self.text(gui, gui._map_empty), "Searching for LiDAR")
        self.assertTrue(all(items[wedge]["state"] == "normal" for wedge in gui._map_sweep))
        self.assertEqual(items[gui._map_labels["FRONT"]]["text"], "Front  --")
        self.assertTrue(all(items[arc]["state"] == "hidden" for arc in gui._map_thresholds.values()))
        self.assertEqual(len(gui.scheduled), 1)
        starts = [items[wedge].get("start") for wedge in gui._map_sweep]
        with patch.object(monitor_ui.time, "monotonic", return_value=50.6):
            gui.scheduled.pop()()  # one animation frame: the beam moved clockwise
        self.assertNotEqual([items[wedge].get("start") for wedge in gui._map_sweep], starts)
        self.assertEqual(len(gui.scheduled), 1)
        gui.notebook.select = lambda: "ble"
        gui.scheduled.pop()()  # hidden tab: the animation stops itself
        self.assertEqual(gui.scheduled, [])
        gui.notebook.select = lambda: "lidar"
        gui.scan_canvas.viewable = False  # minimised window: no animation either
        gui._render_scan()
        self.assertEqual(gui.scheduled, [])

    def test_reported_failure_shows_the_reason_instead_of_the_sweep(self):
        gui = self.renderer()
        gui._last_update_at = None
        gui._current_health = {"lidar_driver": {
            "phase": "failed", "message": "LiDAR driver exited (code 255)"}}
        with patch.object(monitor_ui.time, "monotonic", return_value=50.0):
            gui._render_scan()
        items = gui.scan_canvas.items
        self.assertEqual(self.text(gui, gui._map_empty), "LiDAR not available")
        self.assertEqual(items[gui._map_empty]["fill"], monitor_ui.RADAR["danger"])
        self.assertIn("LiDAR driver exited (code 255)", self.text(gui, gui._map_detail))
        self.assertTrue(all(items[wedge]["state"] == "hidden" for wedge in gui._map_sweep))
        self.assertEqual(items[gui._map_robot]["state"], "hidden")
        self.assertEqual(gui.scheduled, [])

    def test_queued_snapshot_retains_acquisition_age_for_all_readings(self):
        gui = self.renderer()
        gui._render_frame = unittest.mock.Mock()
        with patch.object(monitor_ui.time, "monotonic", return_value=101):
            gui.update({}, .1, 8, 22, "SIDES:STABLE", gui._scan_snapshot)
        gui._render_frame.assert_called_once_with(stale=True)
        self.assertEqual(gui._last_update_at, 100)

    def test_source_age_survives_absent_optional_geometry(self):
        gui = self.renderer()
        gui._render_frame = unittest.mock.Mock()
        for acquired_at, stale in ((100, True), (100.5, False)):
            with self.subTest(acquired_at=acquired_at):
                gui._render_frame.reset_mock()
                with patch.object(monitor_ui.time, "monotonic", return_value=101):
                    gui.update({}, .1, 8, 22, "SIDES:STABLE", None, acquired_at)
                gui._render_frame.assert_called_once_with(stale=stale)
                self.assertEqual(gui._last_update_at, acquired_at)
                self.assertTrue(all(gui.scan_canvas.items[item]["state"] == "hidden"
                                    for item in gui._map_points))

    def test_invalid_explicit_source_age_never_becomes_receipt_time(self):
        gui = self.renderer()
        gui._render_frame = unittest.mock.Mock()
        for acquired_at in (None, "bad", True, math.nan, math.inf, 10 ** 1000, 102):
            with self.subTest(acquired_at=acquired_at):
                gui._render_frame.reset_mock()
                with patch.object(monitor_ui.time, "monotonic", return_value=101):
                    gui.update({}, .1, 8, 22, "SIDES:STABLE", None, acquired_at)
                gui._render_frame.assert_called_once_with(stale=True)

    def test_invalid_snapshot_source_age_never_becomes_receipt_time(self):
        gui = self.renderer()
        gui._render_frame = unittest.mock.Mock()
        for acquired_at in (None, "bad", True, math.nan, math.inf, 10 ** 1000, 102):
            with self.subTest(acquired_at=acquired_at):
                gui._render_frame.reset_mock()
                snapshot = types.SimpleNamespace(points_m=((1, 0),), acquired_at=acquired_at,
                                                 yaw_offset_deg=0)
                with patch.object(monitor_ui.time, "monotonic", return_value=101):
                    gui.update({}, .1, 8, 22, "SIDES:STABLE", snapshot)
                gui._render_frame.assert_called_once_with(stale=True)

    def test_legacy_frames_without_source_age_keep_receipt_time(self):
        gui = self.renderer()
        gui._render_frame = unittest.mock.Mock()
        for optional_geometry in ((), (None,)):
            with self.subTest(optional_geometry=optional_geometry):
                gui._render_frame.reset_mock()
                with patch.object(monitor_ui.time, "monotonic", return_value=101):
                    gui.update({}, .1, 8, 22, "SIDES:STABLE", *optional_geometry)
                gui._render_frame.assert_called_once_with(stale=False)
                self.assertEqual(gui._last_update_at, 101)


@unittest.skipIf(monitor_ui is None, "Tkinter is not installed")
class DashboardTests(unittest.TestCase):
    def setUp(self):
        try:
            self.root = tk.Tk()
        except tk.TclError as error:
            self.skipTest(f"Tk display unavailable: {error}")
        self.root.withdraw()
        self.addCleanup(self.close_root)
        self.data_queue = queue.Queue(maxsize=5)
        self.recv_queue = queue.Queue()
        self.front_disabled = threading.Event()
        self.back_disabled = threading.Event()
        self.health = {"lidar": True, "sensors": True, "firebase": False,
                       "robot_state": "AT_DESTINATION"}
        self.readings = {"ultrasonic": 23.5, "mq4": 180, "mq137": None, "mq135": 0}

    def close_root(self):
        # Drain ttk's theme-change idle event before destroying its interpreter.
        self.root.update_idletasks()
        self.root.destroy()

    def dashboard(self, **options):
        return monitor_ui.LidarGUI(
            self.root, self.data_queue, self.recv_queue,
            read_health=lambda: self.health, read_sensors=lambda: self.readings,
            front_disabled=self.front_disabled, back_disabled=self.back_disabled, **options)

    @staticmethod
    def frame(distance=2.0):
        return ({side: (distance, distance, 3) for side, *_ in monitor_ui.DIRECTIONS},
                .1, 8, 22, "SIDES:STABLE")

    def poll(self, gui):
        self.root.after_cancel(gui._after_id)
        gui._poll()
        self.root.update_idletasks()

    def test_cloud_badge_uses_measured_connection_state(self):
        gui = self.dashboard()
        self.health.update(firebase=False, firebase_status="Offline")
        self.poll(gui)
        self.assertEqual(gui.connection_labels["firebase"].cget("text"), "Offline")
        self.health.update(firebase=True, firebase_status="Pending")
        self.poll(gui)
        self.assertEqual(gui.connection_labels["firebase"].cget("text"), "Pending")

    def test_map_keeps_sector_readings_and_bypass_controls_accessible(self):
        gui = self.dashboard(allow_safety_disable=True)
        self.root.attributes("-alpha", 0)
        self.root.geometry("800x480")
        self.root.deiconify()
        self.root.update()
        # Stamp the scan once the window is shown: mapping it took 1.1 s on a Wayland
        # desktop, longer than the 0.8 s after which the dashboard calls a scan stale.
        snapshot = types.SimpleNamespace(points_m=((1, 0), (0, .8)),
                                         acquired_at=time.monotonic(), yaw_offset_deg=0)
        gui.update(*self.frame(), snapshot)
        self.root.update_idletasks()
        self.assertTrue(gui.scan_canvas.winfo_ismapped())
        self.assert_widgets_visible([gui.map_range_btn, gui.map_info_lbl, gui.map_caption_lbl,
                                     gui.status_lbl, gui.emergency_btn])
        self.assert_map_labels_fit(gui)
        items = gui.scan_canvas.find_all()
        gui._toggle_front()
        gui._toggle_back()
        self.assertEqual(self.reading(gui, "FRONT", "detail"), "Reading only")
        self.assertEqual(self.reading(gui, "BACK", "detail"), "Reading only")
        self.assertEqual(self.reading(gui, "LEFT", "detail"), "Distance")
        self.assertEqual(self.map_label(gui, "FRONT"), "Front  200 cm · read only")
        self.assertEqual(self.map_label(gui, "LEFT"), "Left\n200 cm")
        self.assert_map_labels_fit(gui)  # the widest labels
        gui.map_range_btn.invoke()
        self.root.update()
        self.assertEqual(gui.scan_canvas.find_all(), items)
        self.assertEqual(gui._map_range_m, 8)
        self.assertTrue(self.front_disabled.is_set())
        self.assertTrue(self.back_disabled.is_set())

    def test_lidar_failure_reaches_the_status_and_the_map(self):
        gui = self.dashboard()
        self.health["lidar_driver"] = {
            "phase": "failed", "message": "No serial port is streaming LiDAR data; check its USB cable and power"}
        self.root.attributes("-alpha", 0)
        self.root.geometry("800x480")
        self.root.deiconify()
        self.root.update()
        self.poll(gui)
        self.assertEqual(gui.status_lbl.cget("text"), "LiDAR not available")
        canvas = gui.scan_canvas
        self.assertEqual(canvas.itemcget(gui._map_empty, "text"), "LiDAR not available")
        self.assertIn("check its USB cable", canvas.itemcget(gui._map_detail, "text"))
        self.assertIsNone(gui._sweep_after_id)
        self.assert_map_labels_fit(gui)
        # The message is centred on the scope and clear of the eight labels.
        cx, cy, radius = gui._map_layout
        for item in (gui._map_empty, gui._map_detail):
            x1, y1, x2, y2 = canvas.bbox(item)
            self.assertAlmostEqual((x1 + x2) / 2, cx, delta=2)
            self.assertLess(max(abs(y1 - cy), abs(y2 - cy)), radius)
            for label in gui._map_labels.values():
                a = canvas.bbox(label)
                self.assertFalse(x1 < a[2] and a[0] < x2 and y1 < a[3] and a[1] < y2,
                                 canvas.itemcget(label, "text"))
        self.health["lidar_driver"] = {"phase": "running", "message": "LiDAR driver running"}
        self.data_queue.put_nowait(self.frame())
        self.poll(gui)
        self.assertEqual(gui.status_lbl.cget("text"), "Live scan")

    def test_loading_sweep_runs_while_shown_and_stops_on_close(self):
        gui = self.dashboard()
        self.health["lidar_driver"] = {"phase": "starting", "message": "Starting the LiDAR driver"}
        self.root.attributes("-alpha", 0)
        self.root.geometry("800x480")
        self.root.deiconify()
        self.root.update()
        self.poll(gui)
        self.assertEqual(gui.scan_canvas.itemcget(gui._map_empty, "text"), "Starting LiDAR")
        self.assertEqual(gui.status_lbl.cget("text"), "Waiting for scan")
        self.assertIsNotNone(gui._sweep_after_id)
        sweep_id = gui._sweep_after_id
        gui._on_destroy(types.SimpleNamespace(widget=self.root))
        self.assertIsNone(gui._sweep_after_id)
        self.assertNotIn(sweep_id, self.root.tk.call("after", "info"))

    def test_queue_burst_renders_the_newest_snapshot(self):
        gui = self.dashboard()
        for distance in (1, 2, 3, 4, 5):
            self.data_queue.put_nowait(self.frame(distance))
        self.poll(gui)
        self.assertTrue(self.data_queue.empty())
        self.assertEqual(self.reading(gui, "FRONT"), "500 cm")
        self.assertEqual(gui.status_lbl.cget("text"), "Live scan")

    def test_ble_worker_only_updates_mailbox_until_main_thread_polls(self):
        gui = self.dashboard()
        initial_text = gui.ble_status_lbl.cget("text")
        errors = []
        def worker():
            try:
                gui.set_ble_status("Connecting", "#d29922")
                gui.set_ble_status("Connected", "#3fb950")
            except Exception as error:
                errors.append(error)
        with patch.object(self.root, "after", side_effect=AssertionError("Tk called from worker")):
            thread = threading.Thread(target=worker)
            thread.start()
            thread.join(timeout=2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(gui.ble_status_lbl.cget("text"), initial_text)
        self.poll(gui)
        self.assertEqual(gui.ble_status_lbl.cget("text"), "Connected")
        self.assertEqual(gui.ble_status_lbl.cget("fg"), monitor_ui.COLORS["ok"])

    def test_expired_scan_hides_previously_valid_distance(self):
        gui = self.dashboard()
        with patch.object(monitor_ui.time, "monotonic", return_value=100):
            gui.update(*self.frame())
        with patch.object(monitor_ui.time, "monotonic", return_value=100.81):
            self.poll(gui)
        self.assertEqual(gui.status_lbl.cget("text"), "Scan stale")
        for side in gui.sector_readings:
            self.assertEqual(self.reading(gui, side), "-- cm")
            self.assertEqual(self.reading(gui, side, "detail"), "Scan stale")

    def test_front_diagonals_use_their_own_stop_distance(self):
        gui = self.dashboard()
        self.data_queue.put_nowait(self.frame(.80))
        self.poll(gui)
        self.assertEqual(self.reading(gui, "FRONT", "detail"), "Close obstacle")
        for side in ("FRONT_LEFT", "FRONT_RIGHT"):
            self.assertEqual(self.reading(gui, side, "detail"), "Distance")

    def test_partial_fresh_scan_keeps_valid_readings_visible(self):
        gui = self.dashboard()
        self.health.update(lidar=False, lidar_stream=True)
        self.data_queue.put_nowait(self.frame())
        self.poll(gui)
        self.assertEqual(gui.status_lbl.cget("text"), "Scan incomplete")
        self.assertNotEqual(self.reading(gui, "FRONT"), "-- cm")
        self.health['lidar_stream'] = False
        self.poll(gui)
        self.assertEqual(self.reading(gui, "FRONT"), "-- cm")

    def test_failed_core_health_cannot_be_presented_as_live_scan(self):
        gui = self.dashboard()
        self.health["lidar"] = False
        self.data_queue.put_nowait(self.frame())
        self.poll(gui)
        self.assertEqual(gui.status_lbl.cget("text"), "Scan needs attention")
        self.assertEqual(self.reading(gui, "FRONT"), "-- cm")

    def test_missing_sector_does_not_reuse_old_value(self):
        gui = self.dashboard()
        gui.update(*self.frame())
        new_frame = self.frame()
        new_frame[0]["LEFT"] = None
        gui.update(*new_frame)
        self.assertEqual(self.reading(gui, "LEFT"), "-- cm")
        self.assertEqual(self.reading(gui, "LEFT", "detail"), "No reading")

    def test_locked_safety_controls_cannot_change_core_flags(self):
        gui = self.dashboard()
        gui._toggle_front()
        gui._toggle_back()
        self.assertFalse(self.front_disabled.is_set())
        self.assertFalse(self.back_disabled.is_set())
        self.assertFalse(hasattr(gui, "front_btn"))
        self.front_disabled.set()  # A stale ignored flag must not imply a bypass.
        self.poll(gui)
        self.assertEqual(gui.safety_lbl.cget("text"), "Threshold checks on")

    def test_explicitly_allowed_bypass_updates_flags_and_written_warning(self):
        gui = self.dashboard(allow_safety_disable=True)
        gui.update(*self.frame())
        self.assertIn("it does not identify people", gui.safety_note_lbl.cget("text"))
        gui._toggle_front()
        self.assertTrue(self.front_disabled.is_set())
        self.assertEqual(gui.safety_lbl.cget("text"), "Threshold bypass: front")
        self.assertEqual(self.reading(gui, "FRONT", "detail"), "Reading only")
        self.assertEqual(self.reading(gui, "BACK", "detail"), "Distance")
        gui._toggle_front()
        self.assertFalse(self.front_disabled.is_set())

    def test_group_toggles_keep_distances_and_notify_runtime_immediately(self):
        changes = []
        gui = self.dashboard(allow_safety_disable=True,
                             on_checks_changed=lambda: changes.append(True))
        gui.update(*self.frame(.15))
        gui.front_btn.invoke()
        for side in gui.FRONT_GROUP:
            self.assertEqual(self.reading(gui, side), "15 cm")
            self.assertEqual(self.reading(gui, side, "detail"), "Reading only")
        for side in gui.BACK_GROUP | {"LEFT", "RIGHT"}:
            self.assertEqual(self.reading(gui, side, "detail"), "Close obstacle")
        gui.back_btn.invoke()
        self.assertEqual(gui.safety_lbl.cget("text"),
                         "Threshold bypass: front + back")
        for side in gui.BACK_GROUP:
            self.assertEqual(self.reading(gui, side, "detail"), "Reading only")
        gui.front_btn.invoke()
        self.assertEqual(self.reading(gui, "FRONT", "detail"), "Close obstacle")
        self.assertEqual(len(changes), 3)

    def test_manual_start_requires_current_readiness_and_disables_while_pending(self):
        calls = []

        def start():
            calls.append(True)
            self.health["manual_start"] = {"enabled": False, "pending": True,
                                           "message": "Waiting for acknowledgement"}
            return self.health["manual_start"]

        gui = self.dashboard(request_start=start)
        gui._request_start()
        self.assertEqual(calls, [])
        self.assertTrue(gui.start_btn.instate(["disabled"]))
        self.health["manual_start"] = {"enabled": True, "message": "Ready to start"}
        self.poll(gui)
        self.assertTrue(gui.start_btn.instate(["!disabled"]))
        self.health["manual_start"]["enabled"] = False  # Lost between display and click.
        gui.start_btn.invoke()
        self.assertEqual(calls, [])
        self.health["manual_start"]["enabled"] = True
        self.poll(gui)
        with patch.object(monitor_ui.messagebox, "askyesno", return_value=True) as confirm:
            gui.start_btn.invoke()
        confirm.assert_called_once()
        gui.start_btn.invoke()
        self.assertEqual(calls, [True])
        self.assertTrue(gui.start_btn.instate(["disabled"]))
        self.assertEqual(gui.start_status_lbl.cget("text"), "Waiting for acknowledgement")

    def test_one_failed_refresh_does_not_freeze_the_monitor_after_a_start(self):
        gui = self.dashboard(request_start=lambda: {"enabled": False, "pending": True})
        self.health["manual_start"] = {"enabled": True, "message": "Ready"}
        self.poll(gui)
        with patch.object(monitor_ui.messagebox, "askyesno", return_value=True):
            gui.start_btn.invoke()
        self.root.after_cancel(gui._after_id)
        # A peripheral/health snapshot may fail once during a transition. The
        # monitor must schedule its next update even when this one cannot draw.
        gui.read_health = lambda: (_ for _ in ()).throw(RuntimeError("snapshot unavailable"))
        with self.assertLogs("monitor_ui", level="ERROR"):
            gui._poll()
        self.assertIn(gui._after_id, self.root.tk.call("after", "info"))
        gui.read_health = lambda: {"robot_state": "OUTBOUND", "manual_start": {
            "enabled": False, "message": "Manual start accepted"}}
        self.poll(gui)
        self.assertEqual(gui.start_status_lbl.cget("text"), "Manual start accepted")
        self.assertIn("Outbound", gui.subtitle_lbl.cget("text"))

    def test_bypass_toggle_preserves_pending_start_and_status_query(self):
        from test_runtime import load_runtime
        operator_state = tempfile.TemporaryDirectory(prefix="garby-ui-operator-")
        self.addCleanup(operator_state.cleanup)
        with patch.dict("os.environ", {
                "GARBY_OPERATOR_STOP_FILE": f"{operator_state.name}/operator-stop"}):
            runtime = load_runtime()
        runtime.ble_connected.set()
        runtime.ble_notify_handler("[SYSTEM READY]")
        runtime.ble_notify_handler("[IDLE]")
        gui = runtime.init_gui()(self.root, self.data_queue, runtime.ble_recv_queue)
        self.root.update()
        writes = []
        errors = []
        worker_ready = threading.Event()
        service = runtime.BleService(lambda *args: None, runtime.ble_notify_handler)

        async def run_ble():
            service._loop = asyncio.get_running_loop()
            service._stop_event = asyncio.Event()
            service._write_lock = asyncio.Lock()
            service._write_char = object()

            async def write(characteristic, payload, **kwargs):
                writes.append(payload.decode())
                # Lose the first ACK, then answer the read-only query through
                # the real notification parser while the Tk event loop runs.
                if payload.startswith(b"[START STATUS:"):
                    runtime.ble_notify_handler(f"[START ACK:{payload.decode()[14:-1]}]")
                    runtime.ble_notify_handler("[RUNNING]")
                    runtime.ble_notify_handler("[MOTION:STOP:MPU]")

            service._client = client = types.SimpleNamespace(is_connected=True, write_gatt_char=write)
            worker_ready.set()
            await service._drain_loop(client)

        def worker():
            try:
                asyncio.run(run_ble())
            except Exception as error:
                errors.append(error)

        thread = threading.Thread(target=worker, daemon=True)
        thread.start()
        self.assertTrue(worker_ready.wait(1))
        try:
            with patch.object(monitor_ui.messagebox, "askyesno", return_value=True):
                gui.start_btn.invoke()
            self.assertTrue(gui.start_btn.instate(["disabled"]))
            deadline = time.monotonic() + 1
            while not any(message.startswith("[START:") for message in writes) \
                    and time.monotonic() < deadline:
                self.root.update()
                time.sleep(.01)
            self.assertEqual(sum(message.startswith("[START:") for message in writes), 1)
            gui.front_btn.invoke()
            self.assertTrue(runtime.front_disabled.is_set())
            self.assertTrue(runtime.manual_start.status(True)["pending"])
            deadline = time.monotonic() + 2
            while "MPU" not in gui.start_status_lbl.cget("text") and time.monotonic() < deadline:
                self.root.update()
                time.sleep(.01)
            self.assertIn("MPU", gui.start_status_lbl.cget("text"))
            self.assertFalse(runtime.manual_start.status(True)["pending"])
            self.assertTrue(runtime.front_disabled.is_set())
            gui.front_btn.invoke()
            self.assertFalse(runtime.front_disabled.is_set())
            gui.toggle_messages()
            self.root.update()
            self.assertEqual(gui.log_frame.winfo_manager(), "grid")
            self.assertEqual(sum(message.startswith("[START:") for message in writes), 1)
            self.assertTrue(any(message.startswith("[START STATUS:") for message in writes))
        finally:
            service.stop()
            thread.join(timeout=1)
        self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])

    def test_log_is_bounded_and_does_not_interrupt_manual_scrollback(self):
        gui = self.dashboard()
        for index in range(250):
            self.recv_queue.put_nowait(f"message-{index}")
        for _ in range(7):
            gui._process_ble_recv()
        text = gui.log_text.get("1.0", "end-1c")
        self.assertEqual(len(text.splitlines()), 250)
        self.assertIn("message-249", text)
        self.assertIn("message-0\n", text)
        self.recv_queue.put_nowait("message-250")
        gui._process_ble_recv()
        text = gui.log_text.get("1.0", "end-1c")
        self.assertEqual(len(text.splitlines()), 250)
        self.assertIn("message-250", text)
        self.assertNotIn("message-0\n", text)
        self.recv_queue.put_nowait("\n".join(f"detail-{index}" for index in range(260)))
        gui._process_ble_recv()
        text = gui.log_text.get("1.0", "end-1c")
        self.assertEqual(len(text.splitlines()), 250)
        self.assertTrue(text.startswith("detail-10\n"))
        self.assertIn("detail-259", text)
        self.assertEqual(gui.log_text.cget("state"), "disabled")
        with patch.object(gui.log_text, "yview", return_value=(0, .5)), \
                patch.object(gui.log_text, "see") as scroll:
            self.recv_queue.put_nowait("new message while reviewing")
            gui._process_ble_recv()
            scroll.assert_not_called()

    def test_clear_logs_removes_displayed_history_and_keeps_incoming_messages(self):
        gui = self.dashboard()
        self.root.after_cancel(gui._after_id)
        gui.notebook.select(gui.ble_tab)
        self.root.attributes("-alpha", 0)
        self.root.deiconify()
        gui.clear_logs_btn.focus_force()
        self.root.update()
        for index in range(251):
            self.recv_queue.put_nowait(f"older BLE message-{index}")
        while not self.recv_queue.empty():
            gui._process_ble_recv()
        self.assertEqual(len(gui.log_text.get("1.0", "end-1c").splitlines()), 250)
        self.assertIn("older BLE message", gui.log_text.get("1.0", "end-1c"))
        connection_text = gui.connection_labels["ble"].cget("text")
        self.recv_queue.put_nowait("new BLE message")

        gui.clear_logs_btn.event_generate("<KeyPress-space>")

        self.assertEqual(gui.log_text.get("1.0", "end-1c"), "")
        self.assertEqual(gui.log_text.cget("state"), "disabled")
        self.assertEqual(gui.connection_labels["ble"].cget("text"), connection_text)
        self.assertEqual(self.recv_queue.qsize(), 1)
        gui._process_ble_recv()
        self.assertIn("new BLE message", gui.log_text.get("1.0", "end-1c"))
        self.assertNotIn("older BLE message", gui.log_text.get("1.0", "end-1c"))
        self.assertEqual(gui.log_text.cget("state"), "disabled")

    def test_log_work_per_poll_is_bounded(self):
        gui = self.dashboard()
        for index in range(100):
            self.recv_queue.put_nowait(str(index))
        gui._process_ble_recv()
        self.assertEqual(self.recv_queue.qsize(), 60)

    def test_sensor_absence_is_readable_and_recovers(self):
        gui = self.dashboard()
        self.assertEqual(gui.sensor_labels["mq137"].cget("text"), "--")
        self.assertEqual(gui.sensor_status_labels["mq137"].cget("text"), "Unavailable")
        self.readings["mq137"] = 175
        self.poll(gui)
        self.assertEqual(gui.sensor_labels["mq137"].cget("text"), "175")
        self.assertEqual(gui.sensor_status_labels["mq137"].cget("text"), "Reading")

    def test_message_panel_can_be_opened_and_closed_on_small_display(self):
        gui = self.dashboard()
        self.root.geometry("800x480")
        self.root.update_idletasks()
        gui.toggle_messages()
        self.root.update_idletasks()
        self.assertEqual(gui.notebook.select(), str(gui.ble_tab))
        gui.toggle_messages()
        self.assertEqual(gui.notebook.select(), str(gui.lidar_tab))
        self.assertEqual(len(gui._map_labels), 8)

    def test_small_display_keeps_the_map_and_sensors_visible(self):
        self.readings.update(ultrasonic=600, mq4=100000, mq137=100000, mq135=100000)
        gui = self.dashboard()
        # A mapped transparent window computes real child geometry, unlike a
        # withdrawn Tk root whose child sizes can remain 1x1.
        self.root.attributes("-alpha", 0)
        widgets = list(gui.sensor_labels.values()) + list(gui.sensor_status_labels.values())
        for geometry in ("800x480", "1024x600"):
            with self.subTest(geometry=geometry):
                self.root.geometry(geometry)
                self.root.deiconify()
                self.root.update()
                gui.update(*self.frame(8))  # fresh, so the labels carry distances
                self.root.update_idletasks()
                self.assertTrue(gui.scan_canvas.winfo_ismapped())
                self.assertEqual(self.map_label(gui, "FRONT"), "Front  800 cm")
                self.assert_widgets_visible(widgets)
                self.assertGreater(gui.scan_canvas.winfo_height(), 320)
                for label in (gui.map_info_lbl, gui.map_caption_lbl, gui.status_lbl, gui.motion_hold_lbl):
                    self.assertTrue(str(label).startswith(str(gui.sensor_panel)))
                gui.sensor_canvas.yview_moveto(1)
                self.root.update_idletasks()
                self.assert_widgets_visible([gui.map_info_lbl, gui.map_caption_lbl])
                gui.sensor_canvas.yview_moveto(0)
                self.assert_map_labels_fit(gui)
                gui.toggle_messages()
                self.root.update()
                self.assertFalse(gui.scan_canvas.winfo_ismapped())
                self.assertTrue(gui.log_text.winfo_ismapped())
                self.assert_widgets_visible([gui.log_text, gui.clear_logs_btn,
                                             gui.ble_status_lbl, gui.emergency_btn])
                gui.toggle_messages()
                self.root.update()
                self.assertTrue(gui.scan_canvas.winfo_ismapped())

    def test_sides_read_only_toggle_marks_the_side_readings_and_notifies_runtime(self):
        # Owner, 2026-11-16: "add read only for sides too, so that the motor can be tested and run".
        changes, sides = [], threading.Event()
        gui = self.dashboard(allow_safety_disable=True, side_disabled=sides,
                             on_checks_changed=lambda: changes.append(True))
        gui.update(*self.frame(.15))
        self.assertIn("Sides: checks on", gui.sides_btn.cget("text"))
        gui.sides_btn.invoke()
        self.assertTrue(sides.is_set())
        self.assertEqual(changes, [True])
        self.assertIn("Sides: read only", gui.sides_btn.cget("text"))
        self.assertEqual(gui.safety_lbl.cget("text"), "Threshold bypass: sides")
        for side in ("LEFT", "RIGHT"):
            self.assertEqual(self.reading(gui, side), "15 cm")
            self.assertEqual(self.reading(gui, side, "detail"), "Reading only")
        self.assertEqual(self.reading(gui, "FRONT", "detail"), "Close obstacle")
        gui.front_btn.invoke()
        self.assertEqual(gui.safety_lbl.cget("text"), "Threshold bypass: front + sides")
        gui.sides_btn.invoke()
        self.assertFalse(sides.is_set())
        self.assertEqual(self.reading(gui, "LEFT", "detail"), "Close obstacle")

    def test_optional_bypass_controls_leave_small_display_values_visible(self):
        gui = self.dashboard(allow_safety_disable=True)
        gui.update(*self.frame(8))
        self.root.attributes("-alpha", 0)
        self.root.geometry("800x480")
        self.root.deiconify()
        self.root.update()
        gui.update(*self.frame(8))
        gui._toggle_front()
        gui._toggle_back()
        self.root.update_idletasks()
        self.assertIn("read only", self.map_label(gui, "FRONT_RIGHT"))
        self.assert_map_labels_fit(gui)
        gui.notebook.select(gui.controls_tab)
        self.root.update()
        self.assert_widgets_visible([gui.front_btn, gui.back_btn, gui.sides_btn, gui.start_btn,
                                     gui.start_status_lbl,
                                     gui.ultrasonic_btn, gui.reset_btn, gui.release_btn, gui.continue_btn,
                                     gui.reverse_btn, gui.emergency_btn])

    def operator_status(self, **changes):
        self.health["operator_controls"] = {
            "supported": True, "connected": True, "pending": False,
            "emergency_stopped": False, "ultrasonic_enabled": True,
            "can_configure": True, "can_release": False, "can_reset": False,
            "message": "Controller confirmed", **changes}
        return self.health["operator_controls"]

    def test_tabs_separate_readings_connections_and_controls_with_global_stop(self):
        stop_requests = []
        gui = self.dashboard(allow_safety_disable=True,
                             request_emergency_stop=lambda: stop_requests.append("stop"))
        self.root.attributes("-alpha", 0)
        self.root.geometry("800x480")
        self.root.deiconify()
        self.root.update()
        self.assertEqual([gui.notebook.tab(tab, "text") for tab in gui.notebook.tabs()],
                         ["LiDAR readings", "BLE status & messages", "Controls", "Log"])
        self.assertEqual(str(gui.notebook.cget("takefocus")), "0")

        def element_names(layout):
            for name, options in layout:
                yield name
                yield from element_names(options.get("children", ()))

        style = monitor_ui.ttk.Style(self.root)
        tab_elements = list(element_names(style.layout("Monitor.TNotebook.Tab")))
        self.assertNotIn("Notebook.focus", tab_elements)
        self.assertIn("Notebook.label", tab_elements)
        self.assertIn("Notebook.focus", list(element_names(style.layout("TNotebook.Tab"))))
        for button_style in ("Monitor.TButton", "Compact.Monitor.TButton",
                             "Emergency.Monitor.TButton"):
            elements = list(element_names(style.layout(button_style)))
            self.assertNotIn("Button.focus", elements)
            self.assertIn("Button.border", elements)
            self.assertIn("Button.label", elements)
        self.assertIn("Button.focus", list(element_names(style.layout("TButton"))))
        self.assertEqual(int(gui.log_text.cget("highlightthickness")), 0)
        for distance, tone in ((2, "ok"), (.2, "danger")):
            gui.update(*self.frame(distance))
            for item in gui._map_labels.values():
                self.assertEqual(gui.scan_canvas.itemcget(item, "fill"), monitor_ui.RADAR[tone])
        self.assertEqual(style.lookup("Monitor.TNotebook.Tab", "background", ("selected",)),
                         style.lookup("TNotebook.Tab", "background", ("selected",)))
        for index, (tab, visible) in enumerate(
                ((gui.lidar_tab, gui.scan_canvas), (gui.ble_tab, gui.log_text),
                 (gui.controls_tab, gui.ultrasonic_btn), (gui.log_tab, gui.log_view))):
            # Exercise ttk's actual mouse binding, including its focus behavior.
            coordinates = []
            for x in range(gui.notebook.winfo_width()):
                try:
                    if gui.notebook.index(f"@{x},10") == index:
                        coordinates.append(x)
                except tk.TclError:
                    pass
            self.assertTrue(coordinates)
            x = coordinates[len(coordinates) // 2]
            gui.notebook.event_generate("<ButtonPress-1>", x=x, y=10)
            gui.notebook.event_generate("<ButtonRelease-1>", x=x, y=10)
            self.root.update()
            self.assertEqual(gui.notebook.select(), str(tab))
            self.assert_widgets_visible([visible, gui.emergency_btn])
            self.assertEqual(gui.emergency_btn.tk_focusPrev().tk_focusNext(), gui.emergency_btn)
            gui.emergency_btn.focus_force()
            self.root.update()
            gui.emergency_btn.event_generate("<KeyPress-space>")
            self.assertEqual(stop_requests, ["stop"] * (index + 1))

    def test_unknown_status_never_claims_enabled_or_released(self):
        gui = self.dashboard(request_ultrasonic=lambda enabled: None,
                             request_release_stop=lambda: None, request_system_reset=lambda: None,
                             request_continue_run=lambda: None, request_reverse_run=lambda: None)
        self.assertEqual(gui.ultrasonic_btn.cget("text"), "Ultrasonic: unknown")
        self.assertIn("unconfirmed", gui.stop_status_lbl.cget("text"))
        for button in (gui.ultrasonic_btn, gui.release_btn, gui.reset_btn, gui.continue_btn,
                       gui.reverse_btn):
            self.assertTrue(button.instate(["disabled"]))

    def test_ultrasonic_uses_confirmed_state_and_rechecks_at_click(self):
        calls = []
        status = self.operator_status()
        def toggle(enabled):
            calls.append(enabled)
            status.update(pending=True, message="Awaiting sonar acknowledgement")
            return dict(status)
        gui = self.dashboard(request_ultrasonic=toggle)
        self.assertEqual(gui.ultrasonic_btn.cget("text"), "Disable ultrasonic")
        status["can_configure"] = False
        gui.ultrasonic_btn.invoke()
        self.assertEqual(calls, [])
        status["can_configure"] = True
        self.poll(gui)
        gui.ultrasonic_btn.invoke()
        self.assertEqual(calls, [False])
        self.assertTrue(gui.ultrasonic_btn.instate(["disabled"]))
        self.assertEqual(gui.ultrasonic_status_lbl.cget("text"), "Enabled (controller confirmed)")
        status.update(pending=False, ultrasonic_enabled=False)
        self.poll(gui)
        self.assertIn("default view", gui.ultrasonic_status_lbl.cget("text"))
        self.assertEqual(gui.ultrasonic_btn.cget("text"), "Enable ultrasonic")

    def test_emergency_stop_needs_no_connection_or_confirmation(self):
        calls = []
        gui = self.dashboard(request_emergency_stop=lambda: calls.append(True))
        with patch.object(monitor_ui.messagebox, "askokcancel") as confirm:
            gui.emergency_btn.invoke()
        self.assertEqual(calls, [True])
        confirm.assert_not_called()

    def test_pending_or_disconnected_operator_controls_cannot_release_or_reset(self):
        for field in ("pending", "connected", "supported"):
            calls = []
            status = self.operator_status(can_release=True, can_reset=True)
            gui = self.dashboard(request_release_stop=lambda: calls.append("release"),
                                 request_system_reset=lambda: calls.append("reset"))
            status[field] = field == "pending"
            with patch.object(monitor_ui.messagebox, "askokcancel") as confirm:
                gui._request_release_stop()
                gui._request_system_reset()
            self.assertEqual(calls, [])
            confirm.assert_not_called()
            # Close this dashboard before constructing the next on this Tk root.
            self.root.after_cancel(gui._after_id)
            gui._closed = True
            for child in self.root.winfo_children():
                child.destroy()

    def test_continue_run_confirms_and_rechecks_after_dialog(self):
        # 2026-11-14: Continue run releases the stop into the stopped route.
        calls = []
        status = self.operator_status(emergency_stopped=True, can_release=True, can_continue=True)
        gui = self.dashboard(request_continue_run=lambda: calls.append(True) or dict(status),
                             request_release_stop=lambda: None)
        self.assertTrue(gui.continue_btn.instate(["!disabled"]))
        with patch.object(monitor_ui.messagebox, "askyesno", return_value=False) as confirm:
            gui.continue_btn.invoke()
        self.assertEqual(calls, [])
        self.assertIn("moved by hand", confirm.call_args.args[1])
        def lose_route(*args, **kwargs):
            status["can_continue"] = False
            return True
        with patch.object(monitor_ui.messagebox, "askyesno", side_effect=lose_route):
            gui.continue_btn.invoke()
        self.assertEqual(calls, [])
        self.assertTrue(gui.continue_btn.instate(["disabled"]))
        status["can_continue"] = True
        self.poll(gui)
        with patch.object(monitor_ui.messagebox, "askyesno", return_value=True):
            gui.continue_btn.invoke()
        self.assertEqual(calls, [True])

    def test_continue_run_is_unavailable_without_a_stopped_route(self):
        calls = []
        self.operator_status(emergency_stopped=True, can_release=True, can_continue=False)
        gui = self.dashboard(request_continue_run=lambda: calls.append(True),
                             request_release_stop=lambda: None)
        self.assertTrue(gui.continue_btn.instate(["disabled"]))
        self.assertTrue(gui.release_btn.instate(["!disabled"]))
        with patch.object(monitor_ui.messagebox, "askyesno") as confirm:
            gui._request_continue_run()
        confirm.assert_not_called()
        self.assertEqual(calls, [])

    def test_reverse_to_start_confirms_and_rechecks_after_dialog(self):
        # 2026-11-14: Reverse to start releases the stop into the way back.
        calls = []
        status = self.operator_status(emergency_stopped=True, can_release=True, can_continue=True,
                                      can_reverse=True)
        gui = self.dashboard(request_reverse_run=lambda: calls.append(True) or dict(status),
                             request_release_stop=lambda: None, request_continue_run=lambda: None)
        self.assertTrue(gui.reverse_btn.instate(["!disabled"]))
        with patch.object(monitor_ui.messagebox, "askyesno", return_value=False) as confirm:
            gui.reverse_btn.invoke()
        self.assertEqual(calls, [])
        self.assertIn("moved by hand", confirm.call_args.args[1])
        self.assertIn("where this route started", confirm.call_args.args[1])
        def lose_route(*args, **kwargs):
            status["can_reverse"] = False
            return True
        with patch.object(monitor_ui.messagebox, "askyesno", side_effect=lose_route):
            gui.reverse_btn.invoke()
        self.assertEqual(calls, [])
        self.assertTrue(gui.reverse_btn.instate(["disabled"]))
        status["can_reverse"] = True
        self.poll(gui)
        with patch.object(monitor_ui.messagebox, "askyesno", return_value=True):
            gui.reverse_btn.invoke()
        self.assertEqual(calls, [True])

    def test_reverse_to_start_is_unavailable_without_a_route_and_on_older_firmware(self):
        calls = []
        self.operator_status(emergency_stopped=True, can_release=True, can_continue=True)
        gui = self.dashboard(request_reverse_run=lambda: calls.append(True),
                             request_release_stop=lambda: None, request_continue_run=lambda: None)
        self.assertTrue(gui.reverse_btn.instate(["disabled"]))
        self.assertTrue(gui.continue_btn.instate(["!disabled"]))
        with patch.object(monitor_ui.messagebox, "askyesno") as confirm:
            gui._request_reverse_run()
        confirm.assert_not_called()
        self.assertEqual(calls, [])

    def test_the_route_step_is_shown_under_the_state(self):
        gui = self.dashboard()
        self.health.update(robot_state="OUTBOUND",
                           route_leg="Outbound step 2 of 15: straight · station step, station walls ignored")
        self.poll(gui)
        self.assertIn("State: Outbound", gui.subtitle_lbl.cget("text"))
        self.assertIn("station walls ignored", gui.subtitle_lbl.cget("text"))
        self.health.update(route_leg="")
        self.poll(gui)
        self.assertNotIn("step", gui.subtitle_lbl.cget("text"))

    def test_a_running_route_displays_its_hold_in_the_map_sidebar_and_above_other_tabs(self):
        gui = self.dashboard()
        self.root.deiconify()
        self.health.update(robot_state="OUTBOUND", motion_hold="Side clearance blocked (LEFT_BAND)")
        for tab in (gui.lidar_tab, gui.ble_tab, gui.controls_tab):
            gui.notebook.select(tab)
            self.poll(gui)
            label = gui.motion_hold_lbl if tab == gui.lidar_tab else gui.header_motion_hold_lbl
            self.assertTrue(label.winfo_ismapped())
            self.assertIn("Motors held", gui.motion_hold_lbl.cget("text"))
            self.assertIn("LEFT_BAND", gui.motion_hold_lbl.cget("text"))
        self.health.update(operator_controls={"emergency_stopped": True})
        self.poll(gui)
        self.assertIn("Emergency stop", gui.motion_hold_lbl.cget("text"))
        self.assertNotIn("LEFT_BAND", gui.motion_hold_lbl.cget("text"))
        self.health.update(motion_hold="", operator_controls={})
        self.poll(gui)
        self.assertFalse(gui.motion_hold_lbl.winfo_ismapped())

    def test_developer_motion_confirmation_names_the_selected_checks(self):
        calls = []
        self.health["developer_tests"] = {"enabled": True, "pending": False}
        gui = self.dashboard(request_developer_test=lambda action, sensors: calls.append((action, sensors)))
        gui.developer_gate_vars[1].set(False)
        with patch.object(monitor_ui.messagebox, "askyesno", return_value=True) as confirm:
            gui.developer_buttons["F"].invoke()
        self.assertEqual(calls, [("F", 6)])
        self.assertIn("Ignores: LiDAR", confirm.call_args.args[1])
        self.assertIn("Obeys: Gyro (MPU), Sonar", confirm.call_args.args[1])
        self.assertEqual(confirm.call_args.kwargs["default"], monitor_ui.messagebox.NO)
        gui.developer_gate_vars[2].set(False)
        gui._refresh_developer_controls()
        self.assertTrue(gui.developer_buttons["L"].instate(["disabled"]))
        self.assertTrue(gui.developer_buttons["R"].instate(["disabled"]))
        with patch.object(monitor_ui.messagebox, "askyesno", return_value=False):
            gui.developer_buttons["1"].invoke()
        self.assertEqual(calls, [("F", 6)])

    def test_developer_banner_explains_greyed_out_buttons_and_running_tests(self):
        # Owner, 2026-11-15: greyed-out buttons with no reason were the hardest part.
        self.health["developer_tests"] = {"enabled": False, "pending": False,
            "blocked": "Emergency stop is on. Release it on the Controls tab first.",
            "message": "\u201cForward\u201d finished. Normal safety checks are back on."}
        gui = self.dashboard(request_developer_test=lambda *args: None)
        self.poll(gui)
        self.assertIn("Not available: Emergency stop is on", gui.developer_banner_lbl.cget("text"))
        self.assertTrue(gui.developer_buttons["F"].instate(["disabled"]))
        self.assertIn("Last test:", gui.developer_status_lbl.cget("text"))
        self.health["developer_tests"] = {"enabled": True, "pending": False, "blocked": ""}
        self.poll(gui)
        self.assertIn("Ready", gui.developer_banner_lbl.cget("text"))
        self.assertFalse(gui.developer_buttons["F"].instate(["disabled"]))
        gui.developer_gate_vars[2].set(False)
        gui._refresh_developer_controls()
        self.assertIn("need the gyro", gui.developer_turn_hint_lbl.cget("text"))
        self.health["developer_tests"] = {"enabled": False, "pending": True, "blocked": "",
            "message": "\u201cForward\u201d running \u2014 EMERGENCY STOP stops it at once."}
        self.poll(gui)
        self.assertIn("running", gui.developer_banner_lbl.cget("text"))
        self.assertTrue(all(check.instate(["disabled"]) for check in gui.developer_gate_checks))

    def test_90_degree_buttons_require_floor_confirmation_and_the_mpu(self):
        calls = []
        self.health["developer_tests"] = {"enabled": True, "pending": False}
        gui = self.dashboard(request_developer_test=lambda action, sensors: calls.append((action, sensors)))
        for action, direction in (("L90", "left"), ("R90", "right")):
            with self.subTest(action=action):
                self.assertIn(f"{direction} 90", gui.developer_buttons[action].cget("text"))
                with patch.object(monitor_ui.messagebox, "askyesno", return_value=False) as confirm:
                    gui.developer_buttons[action].invoke()
                self.assertEqual(calls, [])
                self.assertIn("must be on the floor", confirm.call_args.args[1])
                self.assertIn("Do not lift the motors or wheels", confirm.call_args.args[1])
                self.assertIn("MPU detects the robot turning", confirm.call_args.args[1])
                self.assertEqual(confirm.call_args.kwargs["default"], monitor_ui.messagebox.NO)
        with patch.object(monitor_ui.messagebox, "askyesno", return_value=True):
            gui.developer_buttons["L90"].invoke()
            gui.developer_buttons["R90"].invoke()
        self.assertEqual(calls, [("L90", 7), ("R90", 7)])
        gui.developer_gate_vars[2].set(False)
        gui._refresh_developer_controls()
        self.assertTrue(gui.developer_buttons["L90"].instate(["disabled"]))
        self.assertTrue(gui.developer_buttons["R90"].instate(["disabled"]))

    def test_developer_dialog_rechecks_readiness_and_sensor_reading_needs_no_motion_confirmation(self):
        calls = []
        self.health["developer_tests"] = {"enabled": True, "pending": False}
        gui = self.dashboard(request_developer_test=lambda action, sensors: calls.append((action, sensors)))
        def disconnect(*args, **kwargs):
            self.health["developer_tests"]["enabled"] = False
            return True
        with patch.object(monitor_ui.messagebox, "askyesno", side_effect=disconnect):
            gui.developer_buttons["F"].invoke()
        self.assertEqual(calls, [])
        self.health["developer_tests"]["enabled"] = True
        self.poll(gui)
        with patch.object(monitor_ui.messagebox, "askyesno") as confirm:
            gui.developer_buttons["I"].invoke()
        confirm.assert_not_called()
        self.assertEqual(calls, [("I", 7)])

    def test_developer_panel_fits_the_minimum_monitor_size_with_pending_controls_disabled(self):
        self.health["developer_tests"] = {"enabled": False, "pending": True,
            "message": "Developer test active; emergency stop remains available",
            "sensors": {"mpu_ready": True, "yaw_deg": 123.4, "walls_ready": True, "wall_deg": -2.5},
            "result": "Step counters: motor 1 400, motor 2 400; MPU change unavailable"}
        gui = self.dashboard(request_developer_test=lambda *args: None)
        self.root.geometry("800x480")
        self.root.deiconify()
        self.poll(gui)
        self.root.update()
        # The main window's header still fits its 800 px minimum with the new button.
        self.assert_widgets_visible([gui.emergency_btn, gui.developer_open_btn])
        gui.open_developer_tests()
        gui.developer_window.geometry("800x480")  # its minimum, like the Pi's own screen
        self.poll(gui)
        self.root.update()
        self.assert_widgets_visible([gui.developer_buttons["I"], gui.developer_buttons["R"],
                                     gui.developer_buttons["L90"], gui.developer_buttons["R90"],
                                     gui.developer_status_lbl, gui.developer_result_lbl,
                                     gui.developer_emergency_btn])
        for button in gui.developer_buttons.values():
            self.assertTrue(button.instate(["disabled"]))
        self.assertIn("+123.40", gui.developer_sensor_lbl.cget("text"))
        self.assertIn("-2.50", gui.developer_sensor_lbl.cget("text"))
        self.assert_widgets_visible([gui.developer_buttons["B"], gui.developer_turn_hint_lbl])

    def test_backward_test_button_and_live_rear_wall_reading(self):
        # Owner, 2026-11-16: backward could not be tried on the bench; the rear wall
        # that parking squares up to is shown live so the fit can be checked first.
        calls = []
        self.health["developer_tests"] = {"enabled": True, "pending": False}
        self.health["rear_wall"] = {"distance_cm": 152.34, "angle_deg": -0.84}
        gui = self.dashboard(request_developer_test=lambda action, sensors: calls.append((action, sensors)))
        self.poll(gui)
        self.assertIn("Backward", gui.developer_buttons["B"].cget("text"))
        self.assertIn("rear wall 152 cm", gui.developer_sensor_lbl.cget("text"))
        self.assertIn("-0.84", gui.developer_sensor_lbl.cget("text"))
        gui.developer_gate_vars[2].set(False)
        gui._refresh_developer_controls()
        self.assertFalse(gui.developer_buttons["B"].instate(["disabled"]))  # backing needs no turn
        with patch.object(monitor_ui.messagebox, "askyesno", return_value=True) as confirm:
            gui.developer_buttons["B"].invoke()
        self.assertEqual(calls, [("B", 5)])
        self.assertIn("Backward", confirm.call_args.args[1])
        self.health["rear_wall"] = None
        self.poll(gui)
        self.assertIn("no rear wall seen", gui.developer_sensor_lbl.cget("text"))

    def test_reset_confirmation_can_cancel_and_rechecks_after_dialog(self):
        calls = []
        status = self.operator_status(emergency_stopped=True, can_reset=True)
        gui = self.dashboard(request_system_reset=lambda: calls.append(True))
        with patch.object(monitor_ui.messagebox, "askokcancel", return_value=False) as confirm:
            gui.reset_btn.invoke()
        self.assertEqual(calls, [])
        self.assertIn("bin/gas", confirm.call_args.args[1])
        def lose_connection(*args, **kwargs):
            status["connected"] = False
            return True
        with patch.object(monitor_ui.messagebox, "askokcancel", side_effect=lose_connection):
            gui.reset_btn.invoke()
        self.assertEqual(calls, [])
        status["connected"] = True
        self.poll(gui)
        with patch.object(monitor_ui.messagebox, "askokcancel", return_value=True):
            gui.reset_btn.invoke()
        self.assertEqual(calls, [True])

    def click(self, widget):
        # A real pointer press and release, which also reaches a greyed-out button.
        self.root.update()
        widget.event_generate("<ButtonPress-1>", x=4, y=4)
        widget.event_generate("<ButtonRelease-1>", x=4, y=4)
        self.root.update()

    def test_developer_tests_open_in_their_own_window_with_their_own_emergency_stop(self):
        # Owner, 2026-11-16 (ssh -Y): "separate the developer to actual control", two windows.
        stops, cancels = [], []
        self.health["developer_tests"] = {"enabled": True, "pending": False}
        gui = self.dashboard(request_developer_test=lambda *args: None,
                             request_emergency_stop=lambda: stops.append("stop"),
                             request_developer_cancel=lambda: cancels.append("cancel"))
        tabs = [gui.notebook.tab(tab, "text") for tab in gui.notebook.tabs()]
        self.assertNotIn("Developer tests", tabs)
        self.assertEqual(gui.developer_open_btn.cget("text"), "Developer")
        self.assertIn("Log", tabs)
        self.assertEqual(gui.developer_window.state(), "withdrawn")
        self.assertIs(gui.developer_tab.winfo_toplevel(), gui.developer_window)
        gui.developer_open_btn.invoke()
        self.root.update()
        self.assertEqual(gui.developer_window.state(), "normal")
        self.assertIn("Developer tests", gui.developer_window.title())
        gui.developer_emergency_btn.invoke()
        self.assertEqual(stops, ["stop"])
        gui._close_developer_tests()
        self.assertEqual(gui.developer_window.state(), "withdrawn")
        self.assertEqual(cancels, [])  # nothing running: closing only hides it
        self.health["developer_tests"] = {"enabled": False, "pending": True,
                                          "message": "\u201cForward\u201d running"}
        gui.open_developer_tests()
        self.poll(gui)
        gui._close_developer_tests()
        self.assertEqual(cancels, ["cancel"])  # closing a running test stops it
        with patch.object(monitor_ui.messagebox, "askyesno", return_value=False) as confirm:
            self.health["developer_tests"] = {"enabled": True, "pending": False}
            gui.open_developer_tests()
            self.poll(gui)
            gui.developer_buttons["F"].invoke()
        self.assertIs(confirm.call_args.kwargs["parent"], gui.developer_window)

    def test_remote_control_drives_only_while_an_arrow_is_held(self):
        # Owner, 2026-11-16: Up/Down/Left/Right, no step value, runs only while held,
        # Slow/Fast switch, no sensor checks, in the Developer tests window.
        calls = []
        self.health["developer_tests"] = {"enabled": True, "pending": False}
        gui = self.dashboard(request_developer_test=lambda *args: None,
                             request_hold=lambda action, fast: calls.append(("hold", action, fast)),
                             hold_alive=lambda: calls.append(("alive",)),
                             release_hold=lambda: calls.append(("release",)))
        self.root.deiconify()
        gui.open_developer_tests()
        tabs = [gui.developer_notebook.tab(tab, "text") for tab in gui.developer_notebook.tabs()]
        self.assertEqual(tabs, ["Bench tests", "Remote control", "RunStart"])
        gui.developer_notebook.select(gui.remote_tab)
        self.root.update()
        self.assertEqual(set(gui.remote_buttons), {"W", "S", "A", "D"})
        for action, arrow in (("W", "\u25b2"), ("S", "\u25bc"), ("A", "\u25c0"), ("D", "\u25b6")):
            self.assertIn(arrow, gui.remote_buttons[action].cget("text"))
        self.assert_widgets_visible(list(gui.remote_buttons.values()) + [gui.developer_emergency_btn])
        gui.remote_fast_var.set(True)
        gui.remote_buttons["W"].event_generate("<ButtonPress-1>", x=4, y=4)
        self.assertEqual(calls, [("hold", "W", True)])
        self.assertIn("forward", gui.remote_status_lbl.cget("text").lower())
        self.health["developer_tests"] = {"enabled": False, "pending": True}
        self.poll(gui)
        self.assertFalse(gui.remote_buttons["W"].instate(["disabled"]))  # the held arrow stays live
        gui._hold_tick()
        gui._hold_tick()
        self.assertEqual(calls.count(("alive",)), 2)
        gui.remote_buttons["W"].event_generate("<ButtonRelease-1>", x=4, y=4)
        self.assertEqual(calls[-1], ("release",))
        gui._hold_tick()
        self.assertEqual(calls.count(("alive",)), 2)  # let go: the window stops saying "held"
        self.assertIn("stop", gui.remote_status_lbl.cget("text").lower())
        # Not ready: a press says why and asks for nothing.
        calls.clear()
        self.health["developer_tests"] = {"enabled": False, "pending": False,
                                          "blocked": "A route is running. Wait for it to finish first."}
        self.poll(gui)
        gui.remote_buttons["S"].event_generate("<ButtonPress-1>", x=4, y=4)
        gui.remote_buttons["S"].event_generate("<ButtonRelease-1>", x=4, y=4)
        self.assertEqual(calls, [])
        self.assertIn("A route is running", gui.remote_status_lbl.cget("text"))
        # Closing or minimising the window lets go of a held arrow.
        self.health["developer_tests"] = {"enabled": True, "pending": False}
        self.poll(gui)
        gui.remote_fast_var.set(False)
        gui.remote_buttons["A"].event_generate("<ButtonPress-1>", x=4, y=4)
        gui._close_developer_tests()
        self.assertEqual(calls, [("hold", "A", False), ("release",)])

    def test_a_greyed_out_control_says_why_when_clicked(self):
        # Owner, 2026-11-16: Manual disposal showed no Yes/Cancel prompt and nothing moved.
        # A route was already running, held by the right side band.
        starts = []
        self.health.update(robot_state="OUTBOUND", motion_hold="Side clearance blocked (RIGHT_BAND)",
                           manual_start={"enabled": False, "pending": False,
                                         "message": "Robot must be idle to start"},
                           operator_controls={"supported": True, "connected": True,
                                              "emergency_stopped": False, "can_release": False})
        gui = self.dashboard(request_start=lambda: starts.append("start"),
                             request_release_stop=lambda: None)
        self.root.deiconify()
        gui.notebook.select(gui.controls_tab)
        self.poll(gui)
        self.assertTrue(gui.start_btn.instate(["disabled"]))
        with patch.object(monitor_ui.messagebox, "showinfo") as info:
            self.click(gui.start_btn)
        info.assert_called_once()
        title, text = info.call_args.args[:2]
        self.assertIn("Manual disposal", title)
        self.assertIn("route is already running", text)
        self.assertIn("RIGHT_BAND", text)
        self.assertIn("right side", text)
        self.assertIn("EMERGENCY STOP", text)
        self.assertIn("Sides to read only", text)
        self.assertEqual(starts, [])
        with patch.object(monitor_ui.messagebox, "showinfo") as info:
            self.click(gui.release_btn)
        self.assertIn("no emergency stop", info.call_args.args[1].lower())
        # An available button still asks first and never shows the explanation.
        self.health.update(robot_state="IDLE", motion_hold="",
                           manual_start={"enabled": True, "pending": False, "message": "Ready"})
        self.poll(gui)
        with patch.object(monitor_ui.messagebox, "showinfo") as info, \
                patch.object(monitor_ui.messagebox, "askyesno", return_value=False) as confirm:
            self.click(gui.start_btn)
        info.assert_not_called()
        confirm.assert_called_once()
        self.assertEqual(starts, [])

    def test_a_route_the_pi_stopped_says_why_and_what_to_choose(self):
        # Owner, 2026-11-16: "let me decide when will it move".
        gui = self.dashboard()
        self.health.update(robot_state="IDLE", operator_controls={"emergency_stopped": True},
                           operator_pause="LiDAR read-only changed (Sides read only) while a route "
                                          "was running. Choose Continue run, Reverse to start, or "
                                          "Release stop (Controls tab).")
        self.poll(gui)
        text = gui.motion_hold_lbl.cget("text")
        self.assertTrue(text.startswith("Stopped for you to decide:"))
        self.assertIn("Sides read only", text)
        self.assertIn("Release stop", text)

    @staticmethod
    def run_state(**changes):
        state = {"known": True, "saved": None, "place": "none", "running": False,
                 "waiting_release": False, "mode": None, "progress": None, "locked": False,
                 "message": ""}
        state.update(changes)
        return state

    def test_runstart_builds_a_list_of_steps_and_locks_it_once_started(self):
        # Owner, 2026-11-16: a RunStart tab with rows of a dropdown and a steps box, "+"
        # and a bin, rows that cannot be edited once started, max speed and acceleration.
        calls = []
        self.health["developer_tests"] = {"enabled": True, "pending": False, "run": self.run_state()}
        gui = self.dashboard(request_developer_test=lambda *args: None,
                             request_run=lambda steps, speed, accel: calls.append(("run", steps, speed, accel)),
                             request_run_resume=lambda mode: calls.append(("resume", mode)),
                             request_run_clear=lambda: calls.append(("clear",)))
        gui.open_developer_tests()
        gui.developer_notebook.select(gui.runstart_tab)
        self.poll(gui)
        self.assertEqual(len(gui._run_rows), 1)
        self.assertEqual(tuple(gui._run_rows[0]["combo"].cget("values")),
                         ("Forward", "Backward", "Turn left", "Turn right", "Bump calibrate"))
        self.assertIn(str(gui._trash_image), str(gui._run_rows[0]["trash"].cget("image")))
        self.assertEqual(str(gui._run_rows[0]["trash"].cget("state")), "disabled")  # keep one row
        gui._run_rows[0]["steps"].set("10000")
        gui.run_add_btn.invoke()
        gui._run_rows[1]["op"].set("Turn left")
        gui._run_rows[1]["steps"].set("980")
        gui.run_add_btn.invoke()
        gui._run_rows[2]["steps"].set("5")
        self.assertEqual([gui._run_rows[i]["number"].cget("text") for i in range(3)], ["1.", "2.", "3."])
        gui._run_rows[2]["trash"].invoke()
        self.assertEqual(len(gui._run_rows), 2)
        gui.run_speed_var.set("3000")
        gui.run_accel_var.set("6000")
        with patch.object(monitor_ui.messagebox, "askyesno", return_value=True) as confirm:
            gui.run_start_btn.invoke()
        self.assertEqual(calls, [("run", [("F", 10000), ("L", 980)], 3000, 6000)])
        self.assertIn("No LiDAR, sonar or gyro checks", confirm.call_args.args[1])
        self.assertIs(confirm.call_args.kwargs["parent"], gui.developer_window)
        gui._run_rows[1]["steps"].set("x")
        with patch.object(monitor_ui.messagebox, "askyesno") as confirm:
            gui.run_start_btn.invoke()
        confirm.assert_not_called()  # a bad value is explained, not sent
        self.assertIn("Step 2", gui.run_status_lbl.cget("text"))
        gui._run_rows[1]["steps"].set("980")

        # Running: every row and the speeds are locked, progress is marked per step.
        self.health["developer_tests"] = {"enabled": False, "pending": True, "run": self.run_state(
            running=True, locked=True, mode="P", progress={"id": "9", "step": 1, "done": 420})}
        self.poll(gui)
        for widget in (gui._run_rows[0]["entry"], gui._run_rows[1]["combo"], gui._run_rows[0]["trash"],
                       gui.run_add_btn, gui.run_speed_entry, gui.run_accel_entry, gui.run_start_btn):
            self.assertEqual(str(widget.cget("state")), "disabled")
        self.assertIn("done", gui._run_rows[0]["mark"].cget("text"))
        self.assertIn("420/980", gui._run_rows[1]["mark"].cget("text"))
        self.assertIn("step 2 of 2", gui.run_status_lbl.cget("text"))

        # Stopped part-way by the emergency stop: Continue run / Reverse to start, which
        # release the stop first; the rows stay locked.
        saved = {"id": "9", "step": 1, "done": 420, "speed": 3000, "accel": 6000,
                 "steps": [("F", 10000), ("L", 980)], "place": "partway"}
        self.health["operator_controls"] = {"emergency_stopped": True, "can_release": True}
        self.health["developer_tests"] = {"enabled": False, "pending": False, "blocked": "Emergency stop is on.",
                                          "run": self.run_state(saved=saved, place="partway", locked=True)}
        self.poll(gui)
        self.assertEqual(str(gui.run_continue_btn.cget("state")), "normal")
        self.assertEqual(str(gui.run_reverse_btn.cget("state")), "normal")
        self.assertEqual(str(gui.run_start_btn.cget("state")), "disabled")
        self.assertEqual(str(gui._run_rows[0]["entry"].cget("state")), "disabled")
        self.assertIn("Stopped at step 2 of 2", gui.run_status_lbl.cget("text"))
        with patch.object(monitor_ui.messagebox, "askyesno", return_value=True) as confirm:
            gui.run_continue_btn.invoke()
        self.assertEqual(calls[-1], ("resume", "C"))
        self.assertIn("releases the emergency stop", confirm.call_args.args[1])
        with patch.object(monitor_ui.messagebox, "askyesno", return_value=True):
            gui.run_reverse_btn.invoke()
            gui.run_clear_btn.invoke()
        self.assertEqual(calls[-2:], [("resume", "U"), ("clear",)])

        # Finished: the rows can be edited again and Reverse to start takes it back.
        self.health["operator_controls"] = {"emergency_stopped": False}
        self.health["developer_tests"] = {"enabled": True, "pending": False, "run": self.run_state(
            saved=dict(saved, step=1, done=980, place="end"), place="end")}
        self.poll(gui)
        self.assertEqual(str(gui._run_rows[0]["entry"].cget("state")), "normal")
        self.assertEqual(str(gui.run_reverse_btn.cget("state")), "normal")
        self.assertEqual(str(gui.run_continue_btn.cget("state")), "disabled")

    def test_runstart_shows_a_part_way_list_the_robot_saved_before_a_monitor_restart(self):
        saved = {"id": "77", "step": 1, "done": 20, "speed": 1500, "accel": 3000,
                 "steps": [("B", 300), ("C", 250), ("R", 50)], "place": "partway"}
        self.health["developer_tests"] = {"enabled": True, "pending": False, "run": self.run_state(
            saved=saved, place="partway", locked=True)}
        gui = self.dashboard(request_developer_test=lambda *args: None, request_run=lambda *a: None,
                             request_run_resume=lambda mode: None, request_run_clear=lambda: None)
        self.poll(gui)
        self.assertEqual([(row["op"].get(), row["steps"].get()) for row in gui._run_rows],
                         [("Backward", "300"), ("Bump calibrate", "250"), ("Turn right", "50")])
        self.assertEqual((gui.run_speed_var.get(), gui.run_accel_var.get()), ("1500", "3000"))
        self.assertIn("20/250", gui._run_rows[1]["mark"].cget("text"))

    def test_hold_line_says_in_plain_words_where_the_obstacle_is(self):
        gui = self.dashboard()
        self.health.update(robot_state="OUTBOUND", motion_hold="Side clearance blocked (RIGHT_BAND)")
        self.poll(gui)
        text = gui.motion_hold_lbl.cget("text")
        self.assertIn("RIGHT_BAND", text)
        self.assertIn("right beside GARBY's right side", text)
        self.health.update(motion_hold="Front clearance blocked (FRONT_BOX)")
        self.poll(gui)
        self.assertIn("in front of GARBY", gui.motion_hold_lbl.cget("text"))

    def test_log_tab_shows_the_pi_log_with_warnings_marked(self):
        import log_support
        handler = log_support.WindowLogHandler()
        source = logging.Logger("garby-window-test")
        source.addHandler(handler)
        gui = self.dashboard()
        gui.attach_log(handler, "/home/garby/garby-logs/bridge.log")
        source.info("hello from the bridge")
        source.warning("watch out")
        self.poll(gui)
        content = gui.log_view.get("1.0", "end")
        self.assertIn("hello from the bridge", content)
        self.assertIn("watch out", content)
        self.assertTrue(gui.log_view.tag_ranges("warning"))
        self.assertIn("garby-logs/bridge.log", gui.log_path_lbl.cget("text"))
        gui.clear_log_btn.invoke()
        self.assertEqual(gui.log_view.get("1.0", "end").strip(), "")
        source.error("after clearing")
        self.poll(gui)
        self.assertIn("after clearing", gui.log_view.get("1.0", "end"))
        self.assertNotIn("hello from the bridge", gui.log_view.get("1.0", "end"))

    @staticmethod
    def reading(gui, side, key="text"):
        return gui.sector_readings[side][key]

    @staticmethod
    def map_label(gui, side):
        return gui.scan_canvas.itemcget(gui._map_labels[side], "text")

    def assert_map_labels_fit(self, gui):
        """All eight labels inside the canvas, outside the scope, apart."""
        canvas = gui.scan_canvas
        width, height = canvas.winfo_width(), canvas.winfo_height()
        cx, cy, radius = gui._map_layout
        boxes = {}
        for side, item in gui._map_labels.items():
            x1, y1, x2, y2 = canvas.bbox(item)
            with self.subTest(side=side, text=canvas.itemcget(item, "text")):
                self.assertGreaterEqual(x1, 0)
                self.assertGreaterEqual(y1, 0)
                self.assertLessEqual(x2, width)
                self.assertLessEqual(y2, height)
                nearest = (min(max(cx, x1), x2), min(max(cy, y1), y2))
                self.assertGreaterEqual(math.hypot(nearest[0] - cx, nearest[1] - cy), radius - 2)
            boxes[side] = (x1, y1, x2, y2)
        sides = list(boxes)
        for index, first in enumerate(sides):
            for second in sides[index + 1:]:
                a, b = boxes[first], boxes[second]
                self.assertFalse(a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3],
                                 f"{first} label overlaps {second}")

    def assert_widgets_visible(self, widgets):
        for widget in widgets:
            text = widget.cget("text") if "text" in widget.keys() else ""
            with self.subTest(widget=str(widget), text=text):
                self.assertTrue(widget.winfo_ismapped())
                self.assertGreaterEqual(widget.winfo_width(), widget.winfo_reqwidth())
                self.assertGreaterEqual(widget.winfo_height(), widget.winfo_reqheight())
                ancestor = widget.master
                while ancestor is not None:
                    self.assertGreaterEqual(widget.winfo_rootx(), ancestor.winfo_rootx())
                    self.assertGreaterEqual(widget.winfo_rooty(), ancestor.winfo_rooty())
                    self.assertLessEqual(widget.winfo_rootx() + widget.winfo_width(),
                                         ancestor.winfo_rootx() + ancestor.winfo_width())
                    self.assertLessEqual(widget.winfo_rooty() + widget.winfo_height(),
                                         ancestor.winfo_rooty() + ancestor.winfo_height())
                    ancestor = ancestor.master


if __name__ == "__main__":
    unittest.main()
