"""No hardware: the monitor's log goes to the window and a file, not the terminal."""
import raspi_path  # noqa: F401  (puts the deployable RasPi/ folder on sys.path)
import io
import logging
import os
import sys
import tempfile
import unittest

import log_support


class WindowLogTests(unittest.TestCase):
    def setUp(self):
        self.root = logging.Logger("garby-test-root")
        self.root.setLevel(logging.INFO)
        self.log = logging.Logger("garby-test-child")
        self.log.parent = self.root

    def test_window_keeps_the_newest_records_in_order_with_their_level(self):
        handler = log_support.WindowLogHandler(capacity=3)
        self.root.addHandler(handler)
        for index in range(5):
            self.log.info("line %d", index)
        self.log.warning("careful")
        records = handler.since(0)
        self.assertEqual(len(records), 3)
        self.assertIn("line 3", records[0][2])
        self.assertEqual(records[-1][1], logging.WARNING)
        self.assertIn("[WARNING] garby-test-child: careful", records[-1][2])
        self.assertEqual(handler.since(records[-1][0]), [])
        self.assertEqual([item[0] for item in handler.since(records[0][0])],
                         [records[1][0], records[2][0]])

    def test_monitor_mode_takes_the_terminal_handler_off_and_can_put_it_back(self):
        terminal = logging.StreamHandler(sys.stderr)
        stream = logging.StreamHandler(io.StringIO())  # not the terminal: kept
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        for handler in (terminal, stream):
            self.root.addHandler(handler)
        path = log_support.enable_file_log(directory.name, root=self.root)
        window = log_support.WindowLogHandler()
        removed = log_support.move_console_to_window(window, root=self.root)
        self.assertEqual(removed, [terminal])
        self.assertNotIn(terminal, self.root.handlers)
        self.assertIn(stream, self.root.handlers)
        self.assertIn(window, self.root.handlers)
        self.log.info("in the window and the file")
        self.assertIn("in the window and the file", window.since(0)[-1][2])
        for handler in self.root.handlers:
            handler.flush()
        with open(path, encoding="utf-8") as saved:
            self.assertIn("in the window and the file", saved.read())
        self.assertEqual(log_support.file_log_path(self.root), path)
        self.assertEqual(os.path.basename(path), "bridge.log")
        self.assertEqual(log_support.enable_file_log(directory.name, root=self.root), path)
        log_support.restore_console(removed, root=self.root)
        self.assertIn(terminal, self.root.handlers)
        for handler in list(self.root.handlers):
            if isinstance(handler, logging.FileHandler):
                handler.close()

    def test_an_unwritable_log_folder_is_reported_not_fatal(self):
        blocker = tempfile.NamedTemporaryFile()
        self.addCleanup(blocker.close)
        self.assertIsNone(log_support.enable_file_log(os.path.join(blocker.name, "logs"),
                                                      root=self.root))

    def test_a_child_error_storm_is_counted_not_repeated(self):
        handler = log_support.WindowLogHandler()
        self.root.addHandler(handler)
        now = [0.0]
        stream = io.BytesIO(b"Checksum error\n" * 50 + b"\nport opened\n")
        log_support.log_lines_from(stream, self.log, "LiDAR driver: ", clock=lambda: now[0])
        texts = [text for _, _, text in handler.since(0)]
        self.assertEqual(len(texts), 3)
        self.assertIn("LiDAR driver: Checksum error", texts[0])
        self.assertIn("repeated 49 more times", texts[1])
        self.assertIn("LiDAR driver: port opened", texts[2])
        log_support.log_lines_from(io.StringIO("text stream\n"), self.log, "x: ")
        self.assertIn("x: text stream", handler.since(0)[-1][2])

    def test_a_childs_own_info_lines_are_not_warnings(self):
        # Field log 2026-11-16: the driver's version banner showed as an orange warning.
        handler = log_support.WindowLogHandler()
        self.root.addHandler(handler)
        stream = io.BytesIO(b"[INFO] [1794.8] [ydlidar_ros2_driver_node]: [YDLIDAR INFO] Current ROS "
                            b"Driver Version: 1.0.1\n[YDLIDAR ERROR] Failed to open port\n")
        log_support.log_lines_from(stream, self.log, "LiDAR driver: ")
        levels = [level for _, level, _ in handler.since(0)]
        self.assertEqual(levels, [logging.INFO, logging.WARNING])


class MonitorModeTests(unittest.TestCase):
    def test_monitor_mode_moves_the_log_off_the_terminal_and_says_where_it_is(self):
        import contextlib
        import threading
        import test_runtime as fixtures
        from unittest.mock import patch
        runtime = fixtures.load_runtime()
        root = logging.getLogger()
        saved_handlers, saved_hook = list(root.handlers), threading.excepthook
        terminal = logging.StreamHandler(sys.stderr)
        root.addHandler(terminal)
        directory = tempfile.TemporaryDirectory()
        attached = []
        gui = type("Gui", (), {"attach_log": lambda self, handler, path: attached.append((handler, path))})()
        window = log_support.WindowLogHandler()
        out = io.StringIO()
        try:
            with patch.object(sys, "__stdout__", out), contextlib.redirect_stderr(io.StringIO()) as err:
                path = runtime.show_log_in_window(gui, window, directory.name)
                runtime.logger.warning("held by the side band")
            self.assertNotIn(terminal, root.handlers)
            self.assertEqual(attached, [(window, path)])
            self.assertIn("held by the side band", window.since(0)[-1][2])
            self.assertEqual(err.getvalue(), "")
            self.assertIn("Log tab", out.getvalue())
            self.assertIn(path, out.getvalue())
            self.assertEqual(out.getvalue().count("\n"), 1)
            closed = io.StringIO()
            closed.close()
            with patch.object(sys, "__stdout__", closed):
                runtime.terminal_note("GARBY monitor stopped.")  # a dropped ssh session: no error
            for handler in root.handlers:
                handler.flush()
            with open(path, encoding="utf-8") as saved:
                self.assertIn("held by the side band", saved.read())
        finally:
            for handler in list(root.handlers):
                if handler not in saved_handlers:
                    root.removeHandler(handler)
                    if isinstance(handler, logging.FileHandler):
                        handler.close()
            for handler in saved_handlers:
                if handler not in root.handlers:
                    root.addHandler(handler)
            threading.excepthook = saved_hook
            logging.captureWarnings(False)
            directory.cleanup()


if __name__ == "__main__":
    unittest.main()
