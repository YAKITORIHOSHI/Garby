"""Portable credential lookup must ignore client configs and avoid guessing."""
import importlib.util
import json
import os
from pathlib import Path
import shutil
import tempfile
import unittest
from unittest.mock import patch

import firebase_support


class FirebaseCredentialTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.base = Path(self.directory.name)
        self.keys = self.base / "Firebase"
        self.keys.mkdir()
        self.environment = patch.dict(os.environ, {}, clear=True)
        self.environment.start()
        self.addCleanup(self.environment.stop)
        self.location = patch.object(firebase_support, "__file__", str(self.base / "firebase_support.py"))
        self.location.start()
        self.addCleanup(self.location.stop)

    def key(self, filename="rotated-key.json"):
        path = self.keys / filename
        path.write_text(json.dumps({"type": "service_account"}), encoding="utf-8")
        return path

    def test_discovers_service_account_and_ignores_android_web_and_invalid_json(self):
        key = self.key()
        (self.keys / "google-services.json").write_text('{"project_info": {}, "client": []}')
        (self.keys / "web.json").write_text('{"apiKey": "client-config"}')
        (self.keys / "broken.json").write_text("not JSON")
        (self.keys / "list.json").write_text("[]")
        self.assertEqual(firebase_support.resolve_firebase_credentials(), str(key))

    def test_explicit_argument_wins_over_environment_and_auto_discovery(self):
        automatic = self.key()
        explicit = self.base / "selected.json"
        explicit.write_text("{}")
        with patch.dict(os.environ, {"GARBY_FIREBASE_CREDENTIALS": str(automatic)}):
            self.assertEqual(firebase_support.resolve_firebase_credentials(explicit), str(explicit))

    def test_relative_override_is_relative_to_runtime_not_launch_directory(self):
        key = self.key()
        with patch.dict(os.environ, {"GARBY_FIREBASE_CREDENTIALS": "Firebase/rotated-key.json"}):
            self.assertEqual(firebase_support.resolve_firebase_credentials(), str(key))

    def test_missing_explicit_file_is_not_replaced_with_another_key(self):
        self.key()
        with patch.dict(os.environ, {"GARBY_FIREBASE_CREDENTIALS": "missing.json"}):
            with self.assertRaises(FileNotFoundError):
                firebase_support.resolve_firebase_credentials()

    def test_no_key_and_multiple_keys_have_actionable_errors(self):
        with self.assertRaisesRegex(FileNotFoundError, "GARBY_FIREBASE_CREDENTIALS"):
            firebase_support.resolve_firebase_credentials()
        self.key("first.json")
        self.key("second.json")
        with self.assertRaisesRegex(ValueError, "GARBY_FIREBASE_CREDENTIALS"):
            firebase_support.resolve_firebase_credentials()

    def test_lookup_follows_a_moved_runtime_directory_and_renamed_key(self):
        # Load an actual copy from the moved project; no CWD or fixed root is used.
        source = Path(__file__).with_name("firebase_support.py")
        project = self.base / "original-project"
        project.mkdir()
        shutil.copy2(source, project / "firebase_support.py")
        (project / "Firebase").mkdir()
        (project / "Firebase" / "new-name.json").write_text('{"type":"service_account"}')
        moved = self.base / "relocated project"
        project.rename(moved)
        spec = importlib.util.spec_from_file_location("relocated_firebase_support", moved / "firebase_support.py")
        relocated = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(relocated)
        self.assertEqual(relocated.resolve_firebase_credentials(), str(moved / "Firebase" / "new-name.json"))


if __name__ == "__main__":
    unittest.main()
