import tempfile
from pathlib import Path
import unittest

from runtime_support import SingleInstanceGuard


class LeaseTests(unittest.TestCase):
    def test_second_instance_is_rejected_until_owner_exits(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"bridge.lock"
            with SingleInstanceGuard(path):
                with self.assertRaises(RuntimeError):
                    with SingleInstanceGuard(path): pass
            with SingleInstanceGuard(path): pass
            self.assertTrue(path.exists())  # Never replace a locked inode.

    def test_exception_releases_the_lease(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"bridge.lock"
            with self.assertRaises(ValueError):
                with SingleInstanceGuard(path): raise ValueError("startup failed")
            with SingleInstanceGuard(path): pass

    def test_separate_configured_resources_can_have_separate_locks(self):
        with tempfile.TemporaryDirectory() as directory:
            with SingleInstanceGuard(Path(directory)/"one.lock"):
                with SingleInstanceGuard(Path(directory)/"two.lock"): pass
