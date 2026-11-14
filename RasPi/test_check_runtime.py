"""Dependency preflight must not silently disable geometry or prediction."""
import contextlib
import io
from pathlib import Path
import unittest
from unittest.mock import patch

import check_runtime


class DependencyTests(unittest.TestCase):
    def test_numpy_is_required_by_preflight_and_both_install_profiles(self):
        output = io.StringIO()
        with patch.object(check_runtime.importlib.util, "find_spec",
                          side_effect=lambda name: None if name == "numpy" else object()), \
                contextlib.redirect_stdout(output):
            self.assertEqual(check_runtime.check_python_dependencies(), ["numpy"])
        self.assertIn("FAIL Python dependency: numpy", output.getvalue())
        folder = Path(__file__).parent
        self.assertIn("numpy>=", (folder / "requirements.txt").read_text())
        self.assertIn("-r requirements.txt", (folder / "requirements-firebase.txt").read_text())

    def test_present_numpy_passes_without_probing_devices(self):
        with patch.object(check_runtime.importlib.util, "find_spec", return_value=object()), \
                contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(check_runtime.check_python_dependencies(), [])


if __name__ == "__main__":
    unittest.main()
