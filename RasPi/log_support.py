"""Where the Pi's log goes (owner, 2026-11-16: "keep the logs on the GUI instead of
printing it on terminal").

In monitor mode the log is shown in the window's Log tab and written to a daily file;
the terminal (the owner's ssh session) gets one line saying where. Headless mode keeps
the console, which run_headless.sh already sends to its file. Nothing here touches Tk:
the monitor reads the buffer from its own polling callback.
"""
import collections
import logging
import os
import sys
import threading
import time
from logging.handlers import TimedRotatingFileHandler

LOG_FORMAT = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"
# The same folder run_headless.sh uses, so every run's log is in one place.
DEFAULT_LOG_DIR = "~/garby-logs"
LOG_FILE_NAME = "bridge.log"


class WindowLogHandler(logging.Handler):
    """Keeps the newest formatted records for the monitor. emit() never waits on Tk."""

    def __init__(self, capacity=1000, level=logging.INFO):
        super().__init__(level)
        self._records = collections.deque(maxlen=capacity)
        self._records_lock = threading.Lock()
        self._sequence = 0
        self.setFormatter(logging.Formatter(LOG_FORMAT, datefmt="%H:%M:%S"))

    def emit(self, record):
        try:
            text = self.format(record)
        except Exception:
            self.handleError(record)
            return
        with self._records_lock:
            self._sequence += 1
            self._records.append((self._sequence, record.levelno, text))

    def since(self, sequence):
        """[(sequence, level, text)] newer than ``sequence``, oldest first."""
        with self._records_lock:
            return [item for item in self._records if item[0] > sequence]


def is_console_handler(handler):
    return (type(handler) is logging.StreamHandler
            and getattr(handler, "stream", None) in (sys.stderr, sys.stdout,
                                                     sys.__stderr__, sys.__stdout__))


def move_console_to_window(window_handler, root=None):
    """Replace the terminal handlers with the window's. Returns them for restore()."""
    root = root or logging.getLogger()
    removed = [handler for handler in root.handlers if is_console_handler(handler)]
    for handler in removed:
        root.removeHandler(handler)
    if window_handler not in root.handlers:
        root.addHandler(window_handler)
    return removed


def restore_console(handlers, root=None):
    root = root or logging.getLogger()
    for handler in handlers:
        if handler not in root.handlers:
            root.addHandler(handler)


def file_log_path(root=None):
    """The active daily log file, or None."""
    for handler in (root or logging.getLogger()).handlers:
        if isinstance(handler, logging.FileHandler):
            return handler.baseFilename
    return None


def enable_file_log(directory, root=None):
    """Daily file, 14 kept. Returns its path, or None (and says why) if it cannot be written."""
    root = root or logging.getLogger()
    existing = file_log_path(root)
    if existing:
        return existing
    try:
        directory = os.path.expanduser(directory)
        os.makedirs(directory, exist_ok=True)
        handler = TimedRotatingFileHandler(os.path.join(directory, LOG_FILE_NAME),
                                           when="midnight", backupCount=14, encoding="utf-8")
    except OSError as exc:
        logging.getLogger("combined_app").warning("File log unavailable (%s)", exc)
        return None
    handler.setFormatter(logging.Formatter(LOG_FORMAT, datefmt="%Y-%m-%d %H:%M:%S"))
    root.addHandler(handler)
    logging.getLogger("combined_app").info("Logging to %s (daily files, 14 days kept)",
                                           handler.baseFilename)
    return handler.baseFilename


def line_level(text):
    """A child's own [INFO] lines are INFO; anything else it writes to stderr is a WARNING.
    (Field log 2026-11-16: the driver's version banner showed as an orange warning.)"""
    upper = text.upper()
    if "INFO" in upper and not any(word in upper for word in ("WARN", "ERROR", "FAIL")):
        return logging.INFO
    return logging.WARNING


def log_lines_from(stream, log, prefix, level=None, repeat_window_s=10.0,
                   clock=time.monotonic):
    """Log each line a child process writes (its stderr) until it closes the stream.

    A line repeated within ``repeat_window_s`` is counted, not logged again, so an SDK
    error storm cannot flood the window. ``level`` None picks it per line (line_level).
    Runs in the calling thread.
    """
    last_text, last_at, repeats = None, None, 0
    try:
        while True:
            raw = stream.readline()
            if not raw:
                break  # end of stream (b"" or "")
            text = (raw.decode("utf-8", "replace") if isinstance(raw, bytes) else raw).rstrip()
            if not text:
                continue
            now = clock()
            if text == last_text and now - last_at < repeat_window_s:
                repeats += 1
                continue
            if repeats:
                log.log(line_level(last_text) if level is None else level,
                        "%s(previous line repeated %d more times)", prefix, repeats)
            log.log(line_level(text) if level is None else level, "%s%s", prefix, text)
            last_text, last_at, repeats = text, now, 0
        if repeats:
            log.log(line_level(last_text) if level is None else level,
                    "%s(previous line repeated %d more times)", prefix, repeats)
    except (OSError, ValueError):
        pass  # the child went away; its exit is reported by the supervisor


def start_line_logger(stream, log, prefix, level=None):
    thread = threading.Thread(target=log_lines_from, args=(stream, log, prefix, level),
                              daemon=True, name="child-stderr-log")
    thread.start()
    return thread
