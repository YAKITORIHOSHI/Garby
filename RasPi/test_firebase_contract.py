"""Production Firebase mapping/transactions with an in-memory network boundary."""
import json
import os
from pathlib import Path
import re
import tempfile
import threading
import types
import unittest
from unittest.mock import patch
from test_runtime import load_runtime
import firebase_support


class FirebaseContractTests(unittest.TestCase):
    def setUp(self):
        self.runtime = load_runtime()
        self.manager = self.runtime.FirebaseManager.__new__(self.runtime.FirebaseManager)
        self.manager._init_cloud_health()
        self.manager._last_write_error_log = 0.0
        self.manager._status_lock = threading.Lock()
        self.manager._last_status_fields = {}
        self.writes = []
        self.manager._root = types.SimpleNamespace(update=lambda data: self.writes.append(dict(data)))
        self.manager._writer = self.runtime.CoalescingUpdateWorker(self.manager._commit_root_update)
        self.addCleanup(self.manager.close)

    def snapshot(self):
        self.manager.close()
        result = {}
        for batch in self.writes:
            result.update(batch)
        return result

    def test_manager_passes_the_discovered_key_to_the_admin_sdk(self):
        certificates = []
        self.runtime.firebase_admin = types.SimpleNamespace(
            _apps={}, initialize_app=lambda *args: None)
        self.runtime.credentials = types.SimpleNamespace(
            Certificate=lambda path: certificates.append(path) or object())
        self.runtime.db = types.SimpleNamespace(reference=lambda path: self.manager._root)
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            (base / "Firebase").mkdir()
            key = base / "Firebase" / "renamed-key.json"
            key.write_text('{"type":"service_account"}')
            with patch.object(firebase_support, "__file__", str(base / "firebase_support.py")), \
                 patch.dict(os.environ, {}, clear=True):
                manager = self.runtime.FirebaseManager()
                self.addCleanup(manager.close)
        self.assertEqual(certificates, [str(key)])

    def test_cloud_health_requires_success_expires_and_reports_failure(self):
        self.manager._init_cloud_health()
        self.assertEqual(self.manager.cloud_health()["state"], "Connecting")
        with patch.object(self.runtime.time, "monotonic", return_value=100):
            self.manager._cloud_call(lambda: None)
            self.assertTrue(self.manager.cloud_health()["online"])
            self.assertEqual(self.manager.cloud_health().get("last_success_at"), 100)
            with self.assertRaises(OSError):
                self.manager._cloud_call(lambda: (_ for _ in ()).throw(OSError("offline")))
            self.assertFalse(self.manager.cloud_health()["online"])
            self.manager._cloud_call(lambda: None)
        with patch.object(self.runtime.time, "monotonic", return_value=161):
            self.assertEqual(self.manager.cloud_health()["state"], "Stale")

    def test_sensor_batch_matches_dashboard_and_legacy_paths(self):
        self.manager.update_raspi_sensors(air_quality=12, ammonia=3, methane=4, ultrasonic_distance=25)
        result = self.snapshot()
        for sensor, legacy, value in (
            ('mq135', 'MQ135_SENSOR/AIR_QUALITY', 12),
            ('mq137', 'MQ137/AMMONIA', 3),
            ('mq4', 'MQ4_SENSOR/METHANE', 4),
            ('level', 'ULTRASONIC_SENSOR/CM_DISTANCE', 25),
        ):
            self.assertEqual(result[f'devices/garby-bin-01/sensors/{sensor}/value'], value)
            self.assertEqual(result[f'RASPI/VALUES/{legacy}'], value)
            self.assertGreater(result[f'devices/garby-bin-01/sensors/{sensor}/updatedAt'], 0)
        self.assertFalse(any('/commands/' in path for path in result))

    def test_unavailable_weight_invalidates_all_timestamp_mirrors(self):
        self.manager.update_mcu_load_cell(-1)
        result = self.snapshot()
        for path in ('MCU/VALUES/updatedAt', 'RASPI/VALUES/LOAD_CELL/updatedAt',
                     'devices/garby-bin-01/sensors/weight/updatedAt'):
            self.assertEqual(result[path], 0)
        self.assertEqual(result['devices/garby-bin-01/sensors/weight/value'], -1)

    def test_readiness_and_heartbeat_do_not_grant_motion(self):
        self.manager.update_robot_state('AT_DESTINATION')
        self.manager.update_raspi_heartbeat({'lidarHealthy': False, 'bleConnected': False})
        result = self.snapshot()
        self.assertTrue(result['devices/garby-bin-01/status/isReadyToReturn'])
        self.assertFalse(result['devices/garby-bin-01/status/lidarHealthy'])
        self.assertGreater(result['devices/garby-bin-01/status/lastSeen'], 0)
        self.assertFalse(any('/commands/' in path or path == 'APP/resetState' for path in result))

    def test_firebase_failure_retries_real_writer_without_dropping_update(self):
        attempts = []
        succeeded = threading.Event()
        def network(data):
            attempts.append(dict(data))
            if len(attempts) == 1:
                raise OSError('simulated outage')
            self.writes.append(dict(data)); succeeded.set()
        self.manager._root = types.SimpleNamespace(update=network)
        self.manager.update_mcu_states(is_blocked=True)
        self.assertTrue(succeeded.wait(3), 'writer never recovered')
        self.assertGreaterEqual(len(attempts), 2)
        self.assertTrue(self.snapshot()['MCU/STATES/isBlocked'])

    def test_terminal_or_replaced_return_requests_are_not_overwritten(self):
        for current in ({'requestedAt': 20, 'status': 'pending'},
                        {'requestedAt': 10, 'status': 'failed'},
                        {'requestedAt': 10, 'status': 'done'}):
            with self.subTest(current=current):
                saved = dict(current)
                def transaction(callback):
                    updated = callback(dict(saved)); saved.clear(); saved.update(updated)
                    return saved
                self.runtime.db = types.SimpleNamespace(reference=lambda path: types.SimpleNamespace(transaction=transaction))
                self.manager.mark_reset_ack('garby-bin-01', 10)
                self.assertEqual(saved, current)

    def test_return_transition_preserves_request_identity(self):
        saved = {'requestedAt': 10, 'status': 'pending', 'requestedBy': 'test-operator'}
        def transaction(callback):
            updated = callback(dict(saved)); saved.clear(); saved.update(updated)
            return saved
        self.runtime.db = types.SimpleNamespace(reference=lambda path: types.SimpleNamespace(transaction=transaction))
        self.assertTrue(self.manager.mark_reset_ack('garby-bin-01', 10))
        self.assertTrue(self.manager.mark_reset_done('garby-bin-01', 10))
        self.assertEqual(saved['requestedBy'], 'test-operator')
        self.assertEqual(saved['status'], 'done')

    def test_android_and_pi_target_same_database_and_device(self):
        mobile_root = os.environ.get('GARBY_MOBILE_APP_ROOT')
        if not mobile_root:
            self.skipTest('Optional cross-repository check: set GARBY_MOBILE_APP_ROOT to a separate app checkout')
        android = Path(mobile_root) / 'app'
        config = json.loads((android / 'google-services.json').read_text())
        constants = (android / 'src/main/java/com/thesis/garby/realtime/Constants.kt').read_text()
        url = re.search(r'const val DATABASE_URL = "([^"]+)"', constants).group(1)
        device = re.search(r'const val DEFAULT_DEVICE_ID = "([^"]+)"', constants).group(1)
        self.assertEqual(url.rstrip('/'), self.manager.DEFAULT_DATABASE_URL.rstrip('/'))
        self.assertEqual(config['project_info']['firebase_url'].rstrip('/'), url.rstrip('/'))
        self.assertEqual(device, self.runtime.RESET_DEVICE_ID)


if __name__ == '__main__':
    unittest.main()
