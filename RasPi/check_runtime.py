"""Read-only preflight on the Raspberry Pi after sourcing ROS and its driver overlay."""
import argparse
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess
import sys

from ble_support import SERVICE_UUID, WRITE_CHAR_UUID, NOTIFY_CHAR_UUID, configured_uuid
from port_detection import LIDAR, SENSOR, PortDetector
from runtime_support import SingleInstanceGuard
from firebase_support import resolve_firebase_credentials

def check_python_dependencies():
    """Report required imports without contacting hardware or cloud services."""
    missing = []
    for package in ("rclpy", "sensor_msgs", "serial", "bleak", "numpy"):
        available = importlib.util.find_spec(package) is not None
        print(f"{'PASS' if available else 'FAIL'} Python dependency: {package}")
        if not available:
            missing.append(package)
    return missing

def _detect_serial_devices(lidar, sensor):
    """Listen for any device without an explicit port. Never transmits."""
    if lidar and sensor:
        return []
    try:
        # Reading a port the running bridge owns would steal or corrupt its data.
        with SingleInstanceGuard():
            detector=PortDetector(
                lidar_baud=int(os.environ.get("GARBY_LIDAR_BAUDRATE","115200")),
                sensor_baud=int(os.environ.get("GARBY_SENSOR_SERIAL_BAUD","9600")))
            found_lidar=lidar or detector.find(LIDAR,exclude=(sensor,))
            found_sensor=sensor or detector.find(SENSOR,exclude=(lidar,))
    except RuntimeError:
        print("UNVERIFIED serial device detection: a bridge is running and owns the ports")
        return []
    missing=[]
    for name, found, hint in (("LiDAR",found_lidar,"check its USB cable and power"),
                              ("Sensor board",found_sensor,"check its cable and power")):
        print(f"{'PASS' if found else 'FAIL'} {name}: "
              f"{'found on ' + found if found else 'no serial port is sending its data; ' + hint}")
        if not found: missing.append(name)
    if not found_sensor and not any(os.path.exists(path) for path in ("/dev/ttyAMA0","/dev/serial0")):
        print("ADVISE GPIO UART is off: a sensor board wired to GPIO14/15 cannot be seen until "
              "dtparam=uart0=on is added to /boot/firmware/config.txt and the Pi reboots")
    return missing

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--require-firebase",action="store_true")
    args=parser.parse_args()
    failures=[]
    for key, default in (("GARBY_BLE_SERVICE_UUID", SERVICE_UUID),
                         ("GARBY_BLE_WRITE_CHAR_UUID", WRITE_CHAR_UUID),
                         ("GARBY_BLE_NOTIFY_CHAR_UUID", NOTIFY_CHAR_UUID)):
        try:
            configured_uuid(key, default)
            print(f"PASS BLE UUID configuration: {key}")
        except ValueError:
            print(f"FAIL malformed BLE UUID: {key}")
            failures.append(key)
    if sys.platform == "linux":
        adapter = os.environ.get("GARBY_BLE_ADAPTER", "hci0").strip() or "hci0"
        if shutil.which("busctl"):
            try:
                result = subprocess.run(
                    ["busctl", "get-property", "org.bluez", f"/org/bluez/{adapter}",
                     "org.bluez.Adapter1", "Powered"], capture_output=True,
                    text=True, timeout=5)
                powered = result.returncode == 0 and result.stdout.strip() == "b true"
                print(f"{'PASS' if powered else 'FAIL'} powered BlueZ adapter: {adapter}")
                if not powered:
                    failures.append("Bluetooth adapter; run RasPi/check_ble.py for details")
            except (OSError, subprocess.TimeoutExpired) as error:
                print(f"FAIL BlueZ adapter check: {error}")
                failures.append("Bluetooth adapter")
        else:
            print("UNVERIFIED BlueZ adapter: busctl unavailable; run RasPi/check_ble.py")
    if sys.platform == "linux":
        # Both are advisory: neither blocks a run, but each hides a field failure.
        supervision_set = False
        try:
            supervision_set = any(line.strip().startswith("ConnectionSupervisionTimeout=")
                                  for line in Path("/etc/bluetooth/main.conf").read_text().splitlines())
        except OSError:
            pass
        if not supervision_set:
            print("ADVISE BlueZ uses a 420 ms supervision timeout until the bridge updates it; "
                  "see BLE_CONNECTION_GUIDE.md")
        if not os.access("/dev/vcio", os.R_OK | os.W_OK):
            print("ADVISE no /dev/vcio access: throttling is unreadable, only undervoltage is "
                  "logged; add this user to the video group")
    failures.extend(check_python_dependencies())
    ros=shutil.which("ros2")
    try:
        available=bool(ros) and subprocess.run([ros,"pkg","prefix","ydlidar_ros2_driver"],capture_output=True,timeout=10).returncode==0
    except (OSError, subprocess.TimeoutExpired):
        available=False
    print(f"{'PASS' if available else 'FAIL'} YDLIDAR ROS package")
    if not available: failures.append("YDLIDAR overlay")
    lidar=os.environ.get("GARBY_LIDAR_PORT"); sensor=os.environ.get("GARBY_SENSOR_SERIAL_PORT")
    for key, value in (("GARBY_LIDAR_PORT",lidar),("GARBY_SENSOR_SERIAL_PORT",sensor)):
        if value:
            available=Path(value).exists() and os.access(value,os.R_OK|os.W_OK)
            print(f"{'PASS' if available else 'FAIL'} {key}: explicit device {value}")
            if not available: failures.append(key)
    if lidar and sensor and os.path.realpath(lidar)==os.path.realpath(sensor):
        print("FAIL LiDAR and sensor board cannot share one serial port")
        failures.append("serial port collision")
    failures.extend(_detect_serial_devices(lidar, sensor))
    try:
        resolve_firebase_credentials()
        firebase=importlib.util.find_spec("firebase_admin") is not None
    except (OSError, ValueError) as error:
        print(f"UNAVAILABLE Firebase credentials: {error}")
        firebase=False
    print(f"{'PASS' if firebase else 'DISABLED'} Firebase SDK and credential-file configuration")
    if args.require_firebase and not firebase: failures.append("Firebase")
    if failures: raise SystemExit("Preflight failed: " + ", ".join(failures))
    print("Runtime prerequisites PASS. Wheels-lifted acceptance is still required.")

if __name__=="__main__": main()
