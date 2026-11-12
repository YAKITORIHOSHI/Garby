"""Execute the launcher against temporary ROS/Python shims; no robot access."""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

class LauncherTests(unittest.TestCase):
    def test_ros_setup_can_read_unset_variables_and_preflight_is_forwarded(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            setup = root / 'setup.bash'
            setup.write_text('export ROS_TEST_VALUE="$UNSET_ROS_SETUP_VARIABLE"\n')
            python = root / 'python'
            python.write_text('#!/bin/bash\nprintf "%s\\n" "$@"\n')
            python.chmod(0o755)
            env = dict(os.environ, GARBY_ROS_SETUP=str(setup),
                       GARBY_LIDAR_WORKSPACE_SETUP=str(setup), GARBY_PYTHON_BIN=str(python))
            result = subprocess.run(['bash', str(Path(__file__).with_name('run_bridge.sh')),
                                     '--check-runtime'], env=env, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn('check_runtime.py', result.stdout)
            self.assertNotIn('final_w_serial.py', result.stdout)

    def test_missing_explicit_setup_fails_before_python(self):
        env = dict(os.environ, GARBY_ROS_SETUP='/nonexistent/garby-ros/setup.bash')
        result = subprocess.run(['bash', str(Path(__file__).with_name('run_bridge.sh'))],
                                 env=env, capture_output=True, text=True)
        self.assertNotEqual(result.returncode, 0)

if __name__ == '__main__':
    unittest.main()
