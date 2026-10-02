"""Read-only preflight on the Raspberry Pi after sourcing ROS and its driver overlay."""
import argparse
import importlib.util
import os
from pathlib import Path
import shutil
import subprocess

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--require-firebase",action="store_true")
    args=parser.parse_args()
    failures=[]
    for package in ("rclpy","sensor_msgs","serial","bleak"):
        available=importlib.util.find_spec(package) is not None
        print(f"{'PASS' if available else 'FAIL'} Python dependency: {package}")
        if not available: failures.append(package)
    ros=shutil.which("ros2")
    available=bool(ros) and subprocess.run([ros,"pkg","prefix","ydlidar_ros2_driver"],capture_output=True,timeout=10).returncode==0
    print(f"{'PASS' if available else 'FAIL'} YDLIDAR ROS package")
    if not available: failures.append("YDLIDAR overlay")
    for key in ("GARBY_LIDAR_PORT","GARBY_SENSOR_SERIAL_PORT"):
        value=os.environ.get(key)
        available=bool(value) and Path(value).exists() and os.access(value,os.R_OK|os.W_OK)
        print(f"{'PASS' if available else 'FAIL'} {key}: explicit accessible device required")
        if not available: failures.append(key)
    lidar=os.environ.get("GARBY_LIDAR_PORT"); sensor=os.environ.get("GARBY_SENSOR_SERIAL_PORT")
    if lidar and sensor and os.path.realpath(lidar)==os.path.realpath(sensor):
        print("FAIL LiDAR and sensor board cannot share one serial port")
        failures.append("serial port collision")
    credentials=os.environ.get("GARBY_FIREBASE_CREDENTIALS")
    firebase=importlib.util.find_spec("firebase_admin") is not None and bool(credentials) and Path(credentials).is_file()
    print(f"{'PASS' if firebase else 'DISABLED'} Firebase SDK and credential-file configuration")
    if args.require_firebase and not firebase: failures.append("Firebase")
    if failures: raise SystemExit("Preflight failed: " + ", ".join(failures))
    print("Runtime prerequisites PASS. Wheels-lifted acceptance is still required.")

if __name__=="__main__": main()
