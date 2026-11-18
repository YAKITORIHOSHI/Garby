"""Outage notices need observed connectivity, not a failed Firebase API alone."""
import raspi_path  # noqa: F401  (puts the deployable RasPi/ folder on sys.path)
import gc
import io
import asyncio
import queue
import types
import unittest
from unittest.mock import patch
from urllib.error import HTTPError, URLError
import warnings

try:
    from notice_support import HealthReporter, probe_internet
except ImportError:
    HealthReporter = probe_internet = None

from bridge_core import CoalescingBleQueue
from test_runtime import load_runtime


class Response(io.BytesIO):
    def __init__(self, status, data=b"", url=None):
        super().__init__(data)
        self.status, self.url = status, url

    def geturl(self):
        return self.url


class NoticeSupportTests(unittest.TestCase):
    def setUp(self):
        self.assertIsNotNone(HealthReporter, "Background health reporter is missing")
        self.reporter = HealthReporter(cloud_expected=True, started_at=100)

    def observe(self, internet=True, *, cloud=None, temperature=45, flags=0, now=100,
                lidar=True, sensors=True):
        self.reporter.observe(internet, cloud=cloud, lidar=lidar, sensors=sensors,
                              cpu_temperature_c=temperature, throttled_flags=flags, now=now)
        return self.reporter.packet(now)

    def test_cloud_failure_does_not_claim_internet_loss(self):
        self.assertEqual(self.observe(cloud={"online": False, "state": "Offline"}),
                         "[HEALTH:1:UDUUU]")
        self.assertEqual(self.observe(False, cloud={"online": False, "state": "Offline"}),
                         "[HEALTH:1:DXUUU]")

    def test_recent_completed_cloud_operation_can_prove_reachability_when_probe_hosts_are_filtered(self):
        self.assertEqual(self.observe(False, cloud={"online": True, "last_success_at": 99}),
                         "[HEALTH:1:UUUUU]")
        self.assertEqual(self.observe(False, cloud={"online": True, "last_success_at": 80}),
                         "[HEALTH:1:DXUUU]")

    def test_cloud_requires_measured_success_and_has_initialization_grace(self):
        self.assertEqual(self.observe(cloud=None), "[HEALTH:1:UXUUU]")
        self.assertEqual(self.observe(cloud=None, now=159), "[HEALTH:1:UXUUU]")
        self.assertEqual(self.observe(cloud=None, now=160), "[HEALTH:1:UDUUU]")
        self.assertEqual(self.observe(cloud={"online": True, "state": "Pending"}, now=161),
                         "[HEALTH:1:UUUUU]")

    def test_missing_optional_firebase_does_not_generate_an_outage(self):
        self.reporter = HealthReporter(cloud_expected=False, started_at=100)
        self.assertEqual(self.observe(cloud=None, now=1000), "[HEALTH:1:UXUUU]")

    def test_unknown_probe_and_expired_worker_snapshot_are_never_healthy(self):
        self.assertEqual(self.reporter.packet(100), "[HEALTH:1:XXXXX]")
        self.assertEqual(self.observe(None, cloud={"online": True}), "[HEALTH:1:XUUUU]")
        self.assertEqual(self.reporter.packet(116), "[HEALTH:1:XXXXX]")
        self.assertEqual(self.reporter.packet(99), "[HEALTH:1:XXXXX]")

    def test_current_temperature_and_active_power_flags_trigger_thermal_notice(self):
        self.assertEqual(self.observe(temperature=80), "[HEALTH:1:UXUUD]")
        self.assertEqual(self.observe(temperature=77, now=101), "[HEALTH:1:UXUUD]")
        self.assertEqual(self.observe(temperature=74, now=102), "[HEALTH:1:UXUUU]")
        self.assertEqual(self.observe(temperature=40, flags=1), "[HEALTH:1:UXUUD]")

    def test_historical_throttling_does_not_become_permanent_current_fault(self):
        self.assertEqual(self.observe(flags=0xF0000), "[HEALTH:1:UXUUU]")
        self.assertEqual(self.observe(temperature=None, flags=None), "[HEALTH:1:UXUUX]")

    def test_missing_fault_source_cannot_claim_thermal_or_power_recovery(self):
        self.assertEqual(self.observe(temperature=40, flags=1), "[HEALTH:1:UXUUD]")
        self.assertEqual(self.observe(temperature=40, flags=None, now=101), "[HEALTH:1:UXUUX]")
        self.assertEqual(self.observe(temperature=40, flags=0, now=102), "[HEALTH:1:UXUUU]")
        self.assertEqual(self.observe(temperature=85, flags=0, now=103), "[HEALTH:1:UXUUD]")
        self.assertEqual(self.observe(temperature=None, flags=0, now=104), "[HEALTH:1:UXUUX]")

    def test_sensor_and_lidar_faults_remain_distinct_from_connectivity(self):
        self.assertEqual(self.observe(lidar=False), "[HEALTH:1:UXDUU]")
        self.assertEqual(self.observe(sensors=False), "[HEALTH:1:UXUDU]")

    def test_internet_check_falls_back_without_accepting_captive_portal(self):
        urls = []
        def opener(request, timeout):
            urls.append(request.full_url)
            if len(urls) == 1:
                return Response(200, b"<html>Sign in to WiFi</html>", request.full_url)
            return Response(200, b"fl=1\nip=192.0.2.1\ncolo=ABC\n", request.full_url)
        self.assertTrue(probe_internet(opener=opener))
        self.assertEqual(len(urls), 2)

    def test_unreachable_and_redirected_probe_targets_do_not_report_internet_online(self):
        self.assertFalse(probe_internet(opener=lambda *args, **kwargs: (_ for _ in ()).throw(URLError("offline"))))
        def redirect(request, timeout):
            return Response(204, url="https://captive.invalid/login")
        self.assertFalse(probe_internet(opener=redirect))

    def test_http_error_statuses_are_not_online_and_their_responses_are_closed(self):
        # urlopen raises an HTTPError that still holds the response. Left to the
        # garbage collector it raises a ResourceWarning on every health poll.
        urls = []
        def opener(request, timeout):
            urls.append(request.full_url)
            raise HTTPError(request.full_url, 403, "Forbidden", {}, io.BytesIO(b"blocked"))
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always", ResourceWarning)
            self.assertFalse(probe_internet(opener=opener))
            gc.collect()
        self.assertEqual(len(urls), 2)
        self.assertEqual([str(w.message) for w in caught if issubclass(w.category, ResourceWarning)], [])

    def test_successful_204_probe_stops_before_the_second_network_request(self):
        calls = []
        def opener(request, timeout):
            calls.append(timeout)
            return Response(204, url=request.full_url)
        self.assertTrue(probe_internet(opener=opener))
        self.assertEqual(len(calls), 1)
        self.assertLessEqual(calls[0], 2)


class HealthRuntimeTests(unittest.TestCase):
    def test_health_queue_coalesces_without_displacing_the_path_snapshot(self):
        mailbox = CoalescingBleQueue(maxlen=4)
        mailbox.put_nowait("P:12|F=S|B=S", urgent=True)
        for _ in range(25):
            mailbox.put_nowait("[HEALTH:1:DUUUU]")
        mailbox.put_nowait("[HEALTH:1:UUUUU]")
        self.assertEqual(mailbox.get_nowait(), "P:12|F=S|B=S")
        self.assertEqual(mailbox.get_nowait(), "[HEALTH:1:UUUUU]")
        with self.assertRaises(queue.Empty):
            mailbox.get_nowait()

    def test_runtime_packet_uses_sensor_freshness_and_real_cloud_status(self):
        runtime = load_runtime()
        self.assertTrue(hasattr(runtime, "collect_notice_health"), "Runtime SMS health wiring is missing")
        runtime.lidar_healthy.set()
        runtime.sensor_serial_connected.set()
        runtime.latest_sensor_readings.update(ultrasonic=30, mq4=20, mq135=10, mq137=None)
        runtime.firebase = type("Cloud", (), {"cloud_health": lambda self: {"online": False, "state": "Offline"}})()
        with patch.object(runtime, "_read_cpu_temperature_c", return_value=50), patch.object(
                runtime, "_read_throttled_flags", return_value=0):
            health = runtime.collect_notice_health()
        self.assertTrue(health["lidar"])
        self.assertFalse(health["sensors"])
        self.assertFalse(health["cloud"]["online"])

    def test_monitor_failure_invalidates_old_health_without_inventing_cloud_failure(self):
        runtime = load_runtime()
        runtime.health_reporter = HealthReporter(cloud_expected=True, started_at=0)
        runtime.ble_connected.set()
        class OneIteration:
            stopped = False
            def is_set(self):
                return self.stopped
            def wait(self, period):
                self.stopped = True
        with patch.object(runtime, "probe_internet", side_effect=RuntimeError("probe worker failure")):
            runtime.notice_health_loop(OneIteration())
        self.assertEqual(runtime.ble_send_queue.get_nowait(), "[HEALTH:1:XXXXX]")

    def test_a_delayed_health_write_cannot_refresh_old_evidence_at_the_controller(self):
        runtime = load_runtime()
        runtime.health_reporter = HealthReporter(cloud_expected=False, started_at=100)
        runtime.health_reporter.observe(True, cloud=None, lidar=True, sensors=True,
                                        cpu_temperature_c=45, now=100)
        writes = []
        async def write(characteristic, payload, **kwargs):
            writes.append(payload)
        service = runtime.BleService(None, None)
        service._client = types.SimpleNamespace(is_connected=True, write_gatt_char=write)
        service._write_char = object()
        service._write_lock = asyncio.Lock()
        with patch("notice_support.time.monotonic", return_value=116):
            asyncio.run(service._write_message("[HEALTH:1:UXUUU]"))
        self.assertEqual(writes, [b"[HEALTH:1:XXXXX]"])

    def test_probe_time_is_included_in_the_background_monitor_period(self):
        runtime = load_runtime()
        waits = []
        times = iter((100, 104))
        class OneIteration:
            stopped = False
            def is_set(self):
                return self.stopped
            def wait(self, period):
                waits.append(period)
                self.stopped = True
        with patch.object(runtime, "probe_internet", return_value=True), patch.object(
                runtime, "collect_notice_health", return_value=dict(
                    cloud=None, lidar=True, sensors=True)), patch.object(
                runtime.time, "monotonic", side_effect=lambda: next(times, 104)):
            runtime.notice_health_loop(OneIteration())
        self.assertEqual(waits, [1.0])


if __name__ == "__main__":
    unittest.main()
