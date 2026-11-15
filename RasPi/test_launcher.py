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

class HeadlessLauncherTests(unittest.TestCase):
    """run_headless.sh against a stand-in bridge: no ROS, no robot, no real state files."""

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        root = Path(self.directory.name)
        setup = root / 'setup.bash'
        setup.write_text('true\n')
        python = root / 'python'
        # The preflight succeeds at once; the "bridge" writes a log line and waits
        # for SIGTERM, as the real one does.
        python.write_text(
            '#!/bin/bash\n'
            'case "$1" in *check_runtime.py) echo preflight-ok; exit 0;; esac\n'
            'case "$*" in *-I*) exec /usr/bin/python3 "$@";; esac\n'
            'log=""; while [ $# -gt 0 ]; do [ "$1" = --log-dir ] && log="$2"; shift; done\n'
            'echo "bridge up" >> "$log/bridge.log"\n'
            'trap "exit 0" TERM\n'
            'while true; do sleep 0.1; done\n')
        python.chmod(0o755)
        self.env = dict(os.environ, GARBY_ROS_SETUP=str(setup), GARBY_LIDAR_WORKSPACE_SETUP=str(setup),
                        GARBY_PYTHON_BIN=str(python), GARBY_STATE_DIR=str(root / 'state'),
                        GARBY_LOG_DIR=str(root / 'logs'))
        self.env.pop('GARBY_STATUS_FILE', None)
        self.root = root
        self.addCleanup(self.run_launcher, 'stop')

    def run_launcher(self, *arguments):
        return subprocess.run(['bash', str(Path(__file__).with_name('run_headless.sh')), *arguments],
                              env=self.env, capture_output=True, text=True, timeout=30)

    def test_start_status_and_stop(self):
        self.assertIn('not running', self.run_launcher('status').stdout)
        started = self.run_launcher('start')
        self.assertEqual(started.returncode, 0, started.stderr)
        self.assertIn('started headless', started.stdout)
        pid = int((self.root / 'state' / 'bridge.pid').read_text())
        command = Path(f'/proc/{pid}/cmdline').read_bytes().replace(b'\0', b' ').decode()
        self.assertIn('final_w_serial.py --headless --log-dir', command)
        self.assertEqual((self.root / 'logs' / 'bridge.log').read_text(), 'bridge up\n')
        self.assertIn('preflight-ok', (self.root / 'logs' / 'last-preflight.txt').read_text())
        # A second start does not launch a second bridge.
        again = self.run_launcher('start')
        self.assertIn(f'already running (PID {pid})', again.stdout)
        (self.root / 'state' / 'bridge-status.json').write_text(
            '{"robotState": "OUTBOUND", "bleConnected": true, "lidarHealthy": true, '
            '"cloud": "Online", "emergencyStopped": false, "motionHold": "LiDAR obstacle", '
            '"routeLeg": "Outbound step 5 of 15: long hallway", "people": 1, "objects": 2, '
            '"message": "Controller settings confirmed"}')
        status = self.run_launcher('status').stdout
        for expected in (f'running (PID {pid})', 'Robot state     OUTBOUND', 'Controllers     connected',
                         'Emergency stop  released', 'Held by         LiDAR obstacle',
                         '1 likely person(s), 2 object(s)'):
            self.assertIn(expected, status)
        stopped = self.run_launcher('stop')
        self.assertEqual(stopped.returncode, 0, stopped.stderr)
        self.assertIn('GARBY stopped', stopped.stdout)
        self.assertFalse(Path(f'/proc/{pid}').exists())
        self.assertFalse((self.root / 'state' / 'bridge.pid').exists())
        self.assertIn('not running', self.run_launcher('status').stdout)

    def test_a_stale_pid_file_is_not_taken_for_a_running_bridge(self):
        (self.root / 'state').mkdir()
        (self.root / 'state' / 'bridge.pid').write_text(str(os.getpid()))   # this test, not a bridge
        self.assertIn('not running', self.run_launcher('status').stdout)
        self.assertIn('not running', self.run_launcher('stop').stdout)
        self.assertTrue(Path(f'/proc/{os.getpid()}').exists())

    def test_unknown_status_never_reads_as_released(self):
        (self.root / 'state').mkdir()
        (self.root / 'state' / 'bridge-status.json').write_text('{"robotState": "UNKNOWN"}')
        status = self.run_launcher('status').stdout
        self.assertIn('Emergency stop  unknown', status)
        self.assertIn('Controllers     not connected', status)

    def test_a_failed_preflight_starts_nothing(self):
        failing = self.root / 'failing-python'
        failing.write_text('#!/bin/bash\necho "LiDAR driver missing" >&2\nexit 3\n')
        failing.chmod(0o755)
        self.env['GARBY_PYTHON_BIN'] = str(failing)
        result = self.run_launcher('start')
        self.assertEqual(result.returncode, 1)
        self.assertIn('LiDAR driver missing', result.stderr)
        self.assertFalse((self.root / 'state' / 'bridge.pid').exists())

    def test_usage(self):
        result = self.run_launcher()
        self.assertEqual(result.returncode, 2)
        self.assertIn('run_headless.sh start', result.stdout)


if __name__ == '__main__':
    unittest.main()
