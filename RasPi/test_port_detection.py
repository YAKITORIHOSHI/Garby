"""Exercise port detection with real captured bytes and real pyserial over ptys."""
import os
import random
import threading
import time
import unittest
from unittest.mock import patch

import port_detection
from port_detection import (
    LIDAR, SENSOR, PortDetector, candidate_ports, looks_like_lidar, looks_like_sensor,
    NEGATIVE_RETRY_S,
)

# Captured from the installed YDLIDAR X3 at 115200 on 2026-11-04. Its first packet has a valid checksum.
REAL_X3_START = bytes.fromhex(
    "aa550828553ab74460389424f0233c23842240230823fc2234230423202314234e303c302c3034305c30"
    "88306030c43008314031f43000001e27b23368347434")


def lidar_packet(samples=40, sample_bytes=2):
    return b"\xaa\x55\x08" + bytes([samples]) + b"\x55\x3a\xb7\x44\x60\x38" + b"\x10" * (samples * sample_bytes)


LIDAR_STREAM = b"\x98\x2a\xa5" + lidar_packet(40) * 4 + lidar_packet(1) + lidar_packet(23) + lidar_packet(40) * 2
SENSOR_STREAM = b"ULTRASONIC:42.5\nMQ4:120\nMQ137:80\nMQ135:95\n"


class SignatureTests(unittest.TestCase):
    def test_real_x3_packet_format_is_recognised(self):
        packet_length = 10 + 2 * REAL_X3_START[3]
        real_packet = REAL_X3_START.ljust(packet_length, b"\x00")
        self.assertTrue(looks_like_lidar(real_packet * 5))

    def test_lidar_stream_with_varying_packet_sizes(self):
        self.assertTrue(looks_like_lidar(LIDAR_STREAM))

    def test_three_byte_sample_lidar_format(self):
        self.assertTrue(looks_like_lidar(lidar_packet(40, sample_bytes=3) * 5))

    def test_headers_that_do_not_chain_are_not_a_lidar(self):
        self.assertFalse(looks_like_lidar(b"\xaa\x55\x08\x28" * 500))
        self.assertFalse(looks_like_lidar(lidar_packet(40) * 2))
        self.assertFalse(looks_like_lidar(b"\xaa\x55\x08\x00" + b"\x00" * 50))

    def test_random_data_is_not_a_lidar(self):
        generator = random.Random(1)
        self.assertFalse(looks_like_lidar(bytes(generator.randrange(256) for _ in range(20000))))

    def test_sensor_text_is_not_a_lidar_and_lidar_is_not_a_sensor(self):
        self.assertFalse(looks_like_lidar(SENSOR_STREAM * 50))
        self.assertFalse(looks_like_sensor(LIDAR_STREAM * 20))

    def test_sensor_lines_are_recognised_with_unavailable_values(self):
        self.assertTrue(looks_like_sensor(b"ULTRASONIC:999\n"))
        self.assertTrue(looks_like_sensor(b"\xff\x00garbage\r\nMQ4:-1\r\n"))
        self.assertTrue(looks_like_sensor(b"mq135: 12.5\r\n"))

    def test_unfinished_or_other_text_is_not_a_sensor_board(self):
        self.assertFalse(looks_like_sensor(b"MQ4:12"))
        self.assertFalse(looks_like_sensor(b"[BOOT] Setup finished\n[STOP] STOP:LINK\n"))
        self.assertFalse(looks_like_sensor(b"MQ4:abc\nMQ9:12\n"))
        self.assertFalse(looks_like_sensor(b"  MQ135: val=12  status=1\n"))

    def test_sensor_labels_match_the_runtime_parser(self):
        import test_runtime
        self.assertEqual(set(port_detection.SENSOR_LABELS),
                         set(test_runtime.load_runtime()._SERIAL_LABEL_TO_KEY))

    def test_wrong_baud_noise_is_rejected_early_but_short_boot_noise_is_not(self):
        reject = port_detection._cannot_be_sensor
        self.assertTrue(reject(bytes([0x80 + (i % 100) for i in range(300)])))
        self.assertFalse(reject(bytes([0x80 + (i % 100) for i in range(100)])))
        self.assertFalse(reject(SENSOR_STREAM * 10))


class FakeBus:
    """Devices keyed by real path; records every open so tests can prove what was probed."""
    def __init__(self, devices):
        self.devices = dict(devices)
        self.identity = {path: 1 for path in devices}
        self.opened = []
        self.now = 1000.0

    def lister(self):
        return {path: path for path in self.devices}

    def identify(self, path):
        return (path, self.identity[path])

    def clock(self):
        return self.now

    def reader(self, port, baud, seconds, done=None):
        self.opened.append((port, baud))
        role_baud = {LIDAR: 115200, SENSOR: 9600}
        kind = self.devices[port]
        if kind is None:
            return b""
        if role_baud[kind] != baud:
            return bytes([0x80 + (i % 100) for i in range(300)])
        return LIDAR_STREAM if kind == LIDAR else SENSOR_STREAM

    def detector(self):
        return PortDetector(reader=self.reader, lister=self.lister,
                            identify=self.identify, clock=self.clock)


class PortDetectorTests(unittest.TestCase):
    def test_devices_are_found_whichever_way_round_they_are_plugged(self):
        for lidar_port, sensor_port in (("/dev/ttyUSB0", "/dev/ttyUSB1"),
                                        ("/dev/ttyUSB1", "/dev/ttyUSB0"),
                                        ("/dev/ttyACM3", "/dev/ttyUSB9"),
                                        ("/dev/ttyUSB2", "/dev/ttyAMA0")):
            with self.subTest(lidar=lidar_port, sensor=sensor_port):
                detector = FakeBus({sensor_port: SENSOR, lidar_port: LIDAR}).detector()
                self.assertEqual(detector.find(LIDAR), lidar_port)
                self.assertEqual(detector.find(SENSOR), sensor_port)

    def test_identical_adapters_are_told_apart_by_their_data(self):
        bus = FakeBus({"/dev/ttyUSB0": SENSOR, "/dev/ttyUSB1": LIDAR, "/dev/ttyUSB2": None})
        self.assertEqual(bus.detector().find(LIDAR), "/dev/ttyUSB1")

    def test_a_port_with_no_matching_data_is_never_selected(self):
        bus = FakeBus({"/dev/ttyUSB0": None, "/dev/ttyAMA10": None})
        detector = bus.detector()
        self.assertIsNone(detector.find(LIDAR))
        self.assertIsNone(detector.find(SENSOR))

    def test_only_the_data_decides_even_when_the_lidar_is_the_only_port(self):
        self.assertIsNone(FakeBus({"/dev/ttyUSB0": SENSOR}).detector().find(LIDAR))

    def test_a_held_port_is_never_opened_again(self):
        # The LiDAR driver takes no OS lock, so a probe changing the baud would corrupt it.
        bus = FakeBus({"/dev/ttyUSB0": LIDAR, "/dev/ttyAMA0": None})
        detector = bus.detector()
        self.assertEqual(detector.find(LIDAR), "/dev/ttyUSB0")
        bus.opened.clear()
        bus.now += NEGATIVE_RETRY_S + 1
        self.assertIsNone(detector.find(SENSOR))
        self.assertNotIn("/dev/ttyUSB0", [port for port, _ in bus.opened])

    def test_holding_a_role_returns_its_port_without_another_probe(self):
        bus = FakeBus({"/dev/ttyUSB0": LIDAR})
        detector = bus.detector()
        detector.find(LIDAR)
        bus.opened.clear()
        self.assertEqual(detector.find(LIDAR), "/dev/ttyUSB0")
        self.assertEqual(bus.opened, [])

    def test_explicitly_configured_port_of_the_other_role_is_left_alone(self):
        bus = FakeBus({"/dev/ttyUSB0": SENSOR, "/dev/ttyUSB1": LIDAR})
        self.assertEqual(bus.detector().find(LIDAR, exclude=("/dev/ttyUSB0",)), "/dev/ttyUSB1")
        self.assertNotIn("/dev/ttyUSB0", [port for port, _ in bus.opened])

    def test_swapping_devices_after_unplugging_is_detected(self):
        bus = FakeBus({"/dev/ttyUSB0": LIDAR, "/dev/ttyUSB1": SENSOR})
        detector = bus.detector()
        self.assertEqual(detector.find(LIDAR), "/dev/ttyUSB0")
        self.assertEqual(detector.find(SENSOR), "/dev/ttyUSB1")
        bus.devices = {"/dev/ttyUSB0": SENSOR, "/dev/ttyUSB1": LIDAR}
        bus.identity = {"/dev/ttyUSB0": 2, "/dev/ttyUSB1": 2}
        self.assertEqual(detector.find(LIDAR), "/dev/ttyUSB1")
        self.assertEqual(detector.find(SENSOR), "/dev/ttyUSB0")

    def test_lidar_moved_to_a_new_port_is_found_and_unplugged_returns_none(self):
        bus = FakeBus({"/dev/ttyUSB0": LIDAR})
        detector = bus.detector()
        self.assertEqual(detector.find(LIDAR), "/dev/ttyUSB0")
        bus.devices = {}
        self.assertIsNone(detector.find(LIDAR))
        bus.devices = {"/dev/ttyUSB5": LIDAR}
        bus.identity = {"/dev/ttyUSB5": 9}
        self.assertEqual(detector.find(LIDAR), "/dev/ttyUSB5")

    def test_same_path_with_same_identity_keeps_its_role(self):
        bus = FakeBus({"/dev/ttyUSB0": LIDAR})
        detector = bus.detector()
        detector.find(LIDAR)
        bus.devices["/dev/ttyUSB0"] = SENSOR  # not re-plugged, so not re-examined
        self.assertIsNone(detector.find(SENSOR))

    def test_silent_port_is_retried_only_after_the_quiet_period(self):
        bus = FakeBus({"/dev/ttyUSB0": None})
        detector = bus.detector()
        self.assertIsNone(detector.find(SENSOR))
        probes = len(bus.opened)
        bus.devices["/dev/ttyUSB0"] = SENSOR  # the board finished booting
        bus.now += NEGATIVE_RETRY_S - 1
        self.assertIsNone(detector.find(SENSOR))
        self.assertEqual(len(bus.opened), probes)
        bus.now += 2
        self.assertEqual(detector.find(SENSOR), "/dev/ttyUSB0")

    def test_a_port_that_cannot_be_opened_is_skipped(self):
        bus = FakeBus({"/dev/ttyUSB0": None, "/dev/ttyUSB1": LIDAR})
        def reader(port, baud, seconds, done=None):
            if port == "/dev/ttyUSB0":
                raise OSError("busy")
            return bus.reader(port, baud, seconds, done)
        detector = PortDetector(reader=reader, lister=bus.lister, identify=bus.identify, clock=bus.clock)
        self.assertEqual(detector.find(LIDAR), "/dev/ttyUSB1")

    def test_custom_baud_rates_are_used(self):
        seen = []
        def reader(port, baud, seconds, done=None):
            seen.append(baud)
            return b""
        detector = PortDetector(lidar_baud=230400, sensor_baud=19200, reader=reader,
                                lister=lambda: {"/dev/ttyUSB0": "/dev/ttyUSB0"},
                                identify=lambda path: (path,))
        detector.find(LIDAR)
        detector.find(SENSOR)
        self.assertEqual(seen, [230400, 19200])


class CandidateListTests(unittest.TestCase):
    def test_lists_all_serial_families_prefers_stable_names_and_skips_consoles(self):
        present = {
            "/dev/ttyUSB7": ["/dev/ttyUSB7"], "/dev/ttyACM12": ["/dev/ttyACM12"],
            "/dev/ttyAMA*": ["/dev/ttyAMA0", "/dev/ttyAMA10"],
            "/dev/serial/by-id/*": ["/dev/serial/by-id/usb-CP2102-0001"],
        }
        def discover(pattern):
            if pattern == "/dev/ttyUSB*": return present["/dev/ttyUSB7"]
            if pattern == "/dev/ttyACM*": return present["/dev/ttyACM12"]
            if pattern == "/dev/serial0": return ["/dev/serial0"]
            return present.get(pattern, [])
        links = {"/dev/serial/by-id/usb-CP2102-0001": "/dev/ttyUSB7", "/dev/serial0": "/dev/ttyAMA0"}
        with patch.object(port_detection.glob, "glob", side_effect=discover), \
             patch.object(port_detection.os.path, "realpath", side_effect=lambda p: links.get(p, p)), \
             patch.object(port_detection, "_console_devices", return_value={"ttyAMA10"}):
            ports = candidate_ports()
        self.assertEqual(ports["/dev/ttyUSB7"], "/dev/serial/by-id/usb-CP2102-0001")
        self.assertEqual(ports["/dev/ttyACM12"], "/dev/ttyACM12")
        self.assertIn("/dev/ttyAMA0", ports)
        self.assertNotIn("/dev/ttyAMA10", ports)
        self.assertEqual(list(ports)[0], "/dev/ttyUSB7")

    def test_probe_never_asserts_reset_lines_or_transmits(self):
        calls = {}
        class FakeSerial:
            def open(self): calls["dtr"], calls["rts"], calls["exclusive"] = self.dtr, self.rts, self.exclusive
            def read(self, size): return b""
            def close(self): calls["closed"] = True
            def write(self, data): calls["wrote"] = True
        import serial
        with patch.object(serial, "Serial", FakeSerial):
            port_detection.read_port("/dev/x", 9600, 0.01)
        self.assertEqual((calls["dtr"], calls["rts"], calls["exclusive"]), (False, False, True))
        self.assertTrue(calls["closed"])
        self.assertNotIn("wrote", calls)


class RealSerialEndToEndTests(unittest.TestCase):
    """Real pyserial reads from pseudo-terminals, swapped in both orders."""
    def serve(self, payload, stop):
        master, slave = os.openpty()
        os.set_blocking(master, False)
        def writer():
            while not stop.is_set():
                try:
                    os.write(master, payload)
                except BlockingIOError:
                    pass
                time.sleep(0.02)
        thread = threading.Thread(target=writer, daemon=True)
        thread.start()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        self.addCleanup(thread.join, 1)
        return os.ttyname(slave)

    def test_both_orders_through_real_serial_reads(self):
        stop = threading.Event()
        lidar = self.serve(LIDAR_STREAM, stop)
        sensor = self.serve(SENSOR_STREAM, stop)
        self.addCleanup(stop.set)  # runs first, before the writers' files close
        for order in ((lidar, sensor), (sensor, lidar)):
            with self.subTest(order=order):
                ports = {port: port for port in order}
                detector = PortDetector(lister=lambda: ports, identify=lambda path: (path,))
                self.assertEqual(detector.find(LIDAR), lidar)
                self.assertEqual(detector.find(SENSOR), sensor)

    def test_sensor_search_leaves_a_silent_port_after_its_window(self):
        master, slave = os.openpty()
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        port = os.ttyname(slave)
        started = time.monotonic()
        with patch.object(port_detection, "LIDAR_PROBE_S", 0.1):
            detector = PortDetector(lister=lambda: {port: port}, identify=lambda path: (path,))
            self.assertIsNone(detector.find(LIDAR))
        self.assertLess(time.monotonic() - started, 2)


if __name__ == "__main__":
    unittest.main()
