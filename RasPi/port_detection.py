"""Find the LiDAR and sensor board by what they send, whichever port they are on.

Port numbers and USB names cannot tell them apart: ttyUSB numbers follow plug
order, and both boards can use the same CP2102 chip with the same serial number.
Their data can. Probing only listens. It never transmits and never asserts
DTR/RTS, because those lines reset many ESP32 and Arduino boards.

No ROS, BLE or Firebase imports, so this also runs in preflight and on any host.
"""
import glob
import logging
import os
import re
import threading
import time

LIDAR = "lidar"
SENSOR = "sensor"

SENSOR_LABELS = ("ULTRASONIC", "MQ4", "MQ137", "MQ135")
_SENSOR_LINE = re.compile(
    r"\s*(?:" + "|".join(SENSOR_LABELS) + r")\s*:\s*[-+]?\d+(?:\.\d+)?\s*", re.IGNORECASE)

# The X3 streams continuously, so a short window is enough. The sensor board may
# only report every few seconds and can need time to boot if opening resets it.
LIDAR_PROBE_S = 1.5
SENSOR_PROBE_S = 5.0
# A silent port may belong to a board that is still booting. Retry it, but rarely,
# since opening some USB adapters can still pulse their control lines.
NEGATIVE_RETRY_S = 30.0

_LIDAR_HEADER = b"\xaa\x55"
_LIDAR_HEADER_BYTES = 10
_LIDAR_CHAIN_PACKETS = 3
# Text from a real sensor board is printable. Wrong-baud data is not. A boot
# message at another baud is much shorter than this, so it cannot trigger a reject.
_SENSOR_REJECT_BYTES = 256

_PORT_PATTERNS = ("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/ttyAMA*", "/dev/ttyS*",
                  "/dev/serial0", "/dev/serial1")

logger = logging.getLogger("combined_app")


def looks_like_lidar(data):
    """True when consecutive YDLIDAR packets line up, each header where the last predicts.

    Packet: AA 55 CT LSN FSA(2) LSA(2) CS(2), then LSN samples of 2 or 3 bytes.
    A chance match in other data would have to repeat exactly across several packets.
    """
    start = data.find(_LIDAR_HEADER)
    while start >= 0:
        for sample_bytes in (2, 3):
            position, chained = start, 0
            while position + 4 <= len(data) and data[position:position + 2] == _LIDAR_HEADER:
                samples = data[position + 3]
                if not samples:
                    break
                position += _LIDAR_HEADER_BYTES + samples * sample_bytes
                chained += 1
                if chained >= _LIDAR_CHAIN_PACKETS and data[position:position + 2] == _LIDAR_HEADER:
                    return True
        start = data.find(_LIDAR_HEADER, start + 1)
    return False


def looks_like_sensor(data):
    """True when at least one complete `LABEL:number` sensor line arrived."""
    lines = data.decode("ascii", errors="ignore").split("\n")
    # The last piece has no newline yet, so it may be a truncated value.
    return any(_SENSOR_LINE.fullmatch(line) for line in lines[:-1])


def _cannot_be_sensor(data):
    if len(data) < _SENSOR_REJECT_BYTES:
        return False
    printable = sum(32 <= byte < 127 or byte in (9, 10, 13) for byte in data)
    return printable * 2 < len(data)


def read_port(port, baud, seconds, done=None):
    """Listen on a port for up to `seconds`, stopping early once `done(data)` is true."""
    import serial  # pylint: disable=import-outside-toplevel
    link = serial.Serial()
    link.port = port
    link.baudrate = baud
    link.timeout = 0.05
    link.dtr = False
    link.rts = False
    link.exclusive = True
    link.open()
    data = bytearray()
    try:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            data += link.read(1024)
            if done is not None and done(bytes(data)):
                break
    finally:
        link.close()
    return bytes(data)


def _console_devices():
    """Devices serving a login console: reading them would steal keystrokes."""
    try:
        with open("/proc/consoles", "r", encoding="ascii", errors="ignore") as handle:
            return {line.split()[0] for line in handle if line.strip()}
    except OSError:
        return set()


def candidate_ports():
    """Map each real serial device to its most stable name, USB adapters first."""
    consoles = _console_devices()
    ports = {}
    for pattern in _PORT_PATTERNS:
        for path in sorted(glob.glob(pattern)):
            real = os.path.realpath(path)
            if os.path.basename(real) not in consoles:
                ports.setdefault(real, path)
    for link in sorted(glob.glob("/dev/serial/by-id/*")):
        real = os.path.realpath(link)
        if real in ports:
            ports[real] = link
    return ports


def device_identity(real_path):
    """Identify one physical connection; it changes when a USB device is re-plugged.

    The USB device number is used rather than the node's timestamps, which udev
    changes. A held port must keep its identity: the LiDAR driver takes no OS lock,
    so nothing else would stop a probe from changing the baud under it.
    """
    device = f"/sys/class/tty/{os.path.basename(real_path)}/device"
    path = os.path.realpath(device)
    while path.startswith("/sys/") and path != "/sys":
        try:
            with open(os.path.join(path, "devnum"), "r", encoding="ascii") as handle:
                return (path, handle.read().strip())
        except OSError:
            path = os.path.dirname(path)
    return (real_path,)


class PortDetector:
    """Assign each role a port by signature, and never probe a port another role holds."""

    def __init__(self, *, lidar_baud=115200, sensor_baud=9600, reader=read_port,
                 lister=candidate_ports, identify=device_identity, clock=time.monotonic):
        self._probes = {
            LIDAR: (lidar_baud, LIDAR_PROBE_S, looks_like_lidar, looks_like_lidar),
            SENSOR: (sensor_baud, SENSOR_PROBE_S, looks_like_sensor,
                     lambda data: looks_like_sensor(data) or _cannot_be_sensor(data)),
        }
        self._reader = reader
        self._lister = lister
        self._identify = identify
        self._clock = clock
        self._lock = threading.Lock()
        self._records = {}
        self._claims = {}

    def find(self, role, exclude=()):
        """Return the port for `role`, or None. Holding a role keeps its port reserved."""
        excluded = {os.path.realpath(path) for path in exclude if path}
        ports = self._lister()
        with self._lock:
            self._forget_replaced_devices(ports)
            held = self._claims.get(role)
            if held is not None and held not in excluded:
                return ports[held]
        # The lock covers one probe at a time, so both roles can search in turn.
        for real, port in ports.items():
            if real in excluded:
                continue
            with self._lock:
                if real in self._claims.values():
                    continue
                if self._matches(real, port, role):
                    self._claims[role] = real
                    logger.info("Detected %s on %s", role, port)
                    return port
        return None

    def _forget_replaced_devices(self, ports):
        for real, record in list(self._records.items()):
            if real not in ports or self._identify(real) != record["identity"]:
                del self._records[real]
                for role, held in list(self._claims.items()):
                    if held == real:
                        del self._claims[role]

    def _matches(self, real, port, role):
        record = self._records.get(real)
        if record is None:
            record = {"identity": self._identify(real), "checked": {}}
            self._records[real] = record
        checked = record["checked"].get(role)
        if checked is not None and self._clock() - checked < NEGATIVE_RETRY_S:
            return False
        baud, window, signature, done = self._probes[role]
        record["checked"][role] = self._clock()
        try:
            return signature(self._reader(port, baud, window, done))
        except (OSError, ValueError, ImportError) as exc:
            logger.debug("Could not probe %s for %s: %s", port, role, exc)
            return False
