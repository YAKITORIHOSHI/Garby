"""Resolve desktop availability before starting the ROS and robot workers."""
import raspi_path  # noqa: F401  (puts the deployable RasPi/ folder on sys.path)
import os
import sys
import types
import unittest
from unittest.mock import patch

from test_runtime import load_runtime


class DisplayStartupTests(unittest.TestCase):
    def setUp(self):
        self.runtime = load_runtime()

    def test_explicit_headless_does_not_import_tk(self):
        with patch.dict(sys.modules, {"tkinter": None}):
            self.assertIsNone(self.runtime.create_gui_root(headless=True))

    def test_missing_or_empty_display_uses_headless_without_tk(self):
        with patch.object(sys, "platform", "linux"), patch.dict(sys.modules, {"tkinter": None}):
            for environment in ({}, {"DISPLAY": ""}):
                with self.subTest(environment=environment), patch.dict(os.environ, environment, clear=True):
                    self.assertIsNone(self.runtime.create_gui_root(headless=False))

    def test_unusable_display_falls_back_instead_of_raising(self):
        class DisplayError(Exception):
            pass

        def unavailable():
            raise DisplayError("couldn't connect to display :99")

        tk = types.SimpleNamespace(Tk=unavailable, TclError=DisplayError)
        with patch.dict(os.environ, {"DISPLAY": ":99"}), patch.dict(sys.modules, {"tkinter": tk}):
            self.assertIsNone(self.runtime.create_gui_root(headless=False))

    def test_missing_tk_falls_back_with_a_display_configured(self):
        with patch.dict(os.environ, {"DISPLAY": ":99"}), patch.dict(sys.modules, {"tkinter": None}):
            self.assertIsNone(self.runtime.create_gui_root(headless=False))

    def test_available_display_returns_a_gui_root(self):
        root = object()
        tk = types.SimpleNamespace(Tk=lambda: root, TclError=RuntimeError)
        with patch.dict(os.environ, {"DISPLAY": ":99"}), patch.dict(sys.modules, {"tkinter": tk}):
            self.assertIs(self.runtime.create_gui_root(headless=False), root)

    def test_monitor_is_painted_before_any_backend_worker_starts(self):
        events = []
        root = types.SimpleNamespace(update_idletasks=lambda: events.append("paint"))

        class StartupReached(Exception):
            pass

        def first_worker(*args, **kwargs):
            events.append("worker")
            raise StartupReached()

        def monitor(*args):
            events.append("monitor")
            return types.SimpleNamespace(set_ble_status=lambda *args: None)

        with patch.object(self.runtime, "create_gui_root", return_value=root), \
             patch.object(self.runtime, "init_gui", return_value=monitor), \
             patch.object(self.runtime.time, "sleep"), \
             patch.object(self.runtime, "BleService", first_worker), \
             patch.object(self.runtime, "show_log_in_window",
                          lambda *args: events.append("log window")), \
             patch.object(self.runtime.threading, "Thread", first_worker):
            with self.assertRaises(StartupReached):
                self.runtime._run_application(types.SimpleNamespace(headless=False))
        # Since 2026-11-16 the log moves to the window as soon as the window exists.
        self.assertEqual(events, ["monitor", "log window", "paint", "worker"])

    def test_unexpected_display_failure_happens_before_workers_start(self):
        class DisplayFailure(Exception):
            pass

        def unavailable():
            raise DisplayFailure("unexpected display startup failure")

        def worker_started(*args, **kwargs):
            self.fail("A worker started before the display was initialized")

        tk = types.SimpleNamespace(Tk=unavailable, TclError=RuntimeError)
        with patch.dict(os.environ, {"DISPLAY": ":99"}), patch.dict(sys.modules, {"tkinter": tk}), \
             patch.object(self.runtime.time, "sleep"), patch.object(self.runtime.threading, "Thread", worker_started):
            with self.assertRaises(DisplayFailure):
                self.runtime._run_application(types.SimpleNamespace(headless=False))


if __name__ == "__main__":
    unittest.main()
