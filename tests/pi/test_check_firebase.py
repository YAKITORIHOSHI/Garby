"""Check probe error handling and diagnostic cleanup without contacting Firebase."""
import raspi_path  # noqa: F401  (puts the deployable RasPi/ folder on sys.path)
from contextlib import redirect_stdout
import gc
import io
import os
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
import warnings
import check_firebase
import firebase_support


class FirebaseProbeTests(unittest.TestCase):
    def test_anonymous_permission_denial_is_not_reported_as_authenticated_success(self):
        with patch.object(check_firebase, 'urlopen', side_effect=HTTPError('https://example.test', 401, 'denied', {}, None)), redirect_stdout(io.StringIO()) as output:
            self.assertEqual(check_firebase.anonymous_probe('https://example.test'), 2)
        self.assertIn('UNVERIFIED', output.getvalue())

    def test_a_denied_anonymous_read_closes_its_http_response(self):
        # The HTTPError still holds the response. Left to the garbage collector it
        # raises "ResourceWarning: Implicitly cleaning up <HTTPError 401: 'denied'>".
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter('always', ResourceWarning)
            with patch.object(check_firebase, 'urlopen', side_effect=HTTPError(
                    'https://example.test', 401, 'denied', {}, io.BytesIO(b'denied'))), \
                    redirect_stdout(io.StringIO()):
                self.assertEqual(check_firebase.anonymous_probe('https://example.test'), 2)
            gc.collect()
        self.assertEqual([str(w.message) for w in caught if issubclass(w.category, ResourceWarning)], [])

    def test_missing_credential_file_reports_incomplete(self):
        with tempfile.TemporaryDirectory() as temporary, redirect_stdout(io.StringIO()):
            self.assertEqual(check_firebase.authenticated_probe('https://example.test', str(Path(temporary) / 'absent')), 2)

    def test_readback_failure_still_cleans_only_its_diagnostic_record(self):
        calls = []
        class Reference:
            def __init__(self, path): self.path = path; self.deleted = False
            def set(self, value): calls.append(('write', self.path))
            def get(self): return None  # mismatch after write, absence after delete
            def delete(self): calls.append(('delete', self.path)); self.deleted = True
        sdk = types.ModuleType('firebase_admin')
        sdk.credentials = types.SimpleNamespace(Certificate=lambda path: object())
        sdk.db = types.SimpleNamespace(reference=lambda path, app: Reference(path))
        sdk.initialize_app = lambda *a, **kw: object()
        sdk.delete_app = lambda app: calls.append(('close', 'app'))
        with tempfile.TemporaryDirectory() as temporary:
            credential = Path(temporary) / 'fixture.json'; credential.write_text('{}')
            with patch.dict('sys.modules', {'firebase_admin': sdk}), redirect_stdout(io.StringIO()):
                self.assertEqual(check_firebase.authenticated_probe('https://example.test', str(credential), True), 1)
        self.assertEqual(calls[0][0], 'write')
        self.assertEqual(calls[1], ('delete', calls[0][1]))
        self.assertTrue(calls[0][1].startswith('/_diagnostics/garby-check/'))
        self.assertEqual(calls[2], ('close', 'app'))

    def test_write_probe_discovers_credentials_and_removes_its_record(self):
        records = {}
        written = []
        closed = []

        class Reference:
            def __init__(self, path): self.path = path
            def get(self): return records.get(self.path)
            def set(self, value):
                written.append(self.path)
                records[self.path] = value
            def delete(self): records.pop(self.path, None)

        sdk = types.ModuleType('firebase_admin')
        sdk.credentials = types.SimpleNamespace(Certificate=lambda path: object())
        sdk.db = types.SimpleNamespace(reference=lambda path, app: Reference(path))
        sdk.initialize_app = lambda *args, **kwargs: object()
        sdk.delete_app = closed.append
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            (base / 'Firebase').mkdir()
            (base / 'Firebase/key.json').write_text('{"type":"service_account"}')
            with patch.object(firebase_support, '__file__', str(base / 'firebase_support.py')), \
                 patch.dict(os.environ, {}, clear=True), patch.dict(sys.modules, {'firebase_admin': sdk}), \
                 patch.object(sys, 'argv', ['check_firebase.py', '--write-test']), \
                 redirect_stdout(io.StringIO()) as output:
                result = check_firebase.main()
        self.assertEqual(result, 0, output.getvalue())
        self.assertEqual(records, {})
        self.assertEqual(len(written), 1)
        self.assertTrue(written[0].startswith('/_diagnostics/garby-check/'))
        self.assertEqual(len(closed), 1)

    def test_invalid_explicit_or_ambiguous_credentials_do_not_probe_anonymously(self):
        with tempfile.TemporaryDirectory() as directory:
            base = Path(directory)
            (base / 'Firebase').mkdir()
            for name in ('one.json', 'two.json'):
                (base / 'Firebase' / name).write_text('{"type":"service_account"}')
            for arguments in ([], ['--credentials', 'missing.json']):
                with self.subTest(arguments=arguments), \
                     patch.object(firebase_support, '__file__', str(base / 'firebase_support.py')), \
                     patch.dict(os.environ, {}, clear=True), \
                     patch.object(sys, 'argv', ['check_firebase.py', *arguments]), \
                     patch.object(check_firebase, 'urlopen', side_effect=AssertionError('unexpected network request')), \
                     redirect_stdout(io.StringIO()):
                    self.assertEqual(check_firebase.main(), 2)

if __name__ == '__main__':
    unittest.main()
