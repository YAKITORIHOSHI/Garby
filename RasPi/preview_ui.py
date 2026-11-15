"""Preview the real dashboard with sample data; never connects to robot hardware."""

import argparse
import math
import queue
import time
import tkinter as tk

from monitor_ui import DIRECTIONS, LidarGUI
from lidar_map import LidarMapSnapshot


def synthetic_scan_points(obstacle=False):
    """Sample a fictional rectangular room; these are never robot measurements."""
    points = []
    for angle in range(0, 360, 2):
        forward, left = math.cos(math.radians(angle)), math.sin(math.radians(angle))
        limits = []
        if abs(forward) > 1e-9:
            limits.append((2.1 if forward > 0 else 1.8) / abs(forward))
        if abs(left) > 1e-9:
            limits.append((.8 if left > 0 else .85) / abs(left))
        distance = min(limits)
        if obstacle and min(angle, 360 - angle) < 12:
            distance = .6 / forward
        points.append((distance * forward, distance * left))
    return tuple(points)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--size", default="1024x600", help="Window size, e.g. 800x480")
    parser.add_argument("--scenario", default="normal",
                        choices=("normal", "obstacle", "loading", "unavailable", "stale"),
                        help="loading: the radar sweep while the LiDAR starts; unavailable: "
                             "a reported driver failure; stale: one scan, then the stream stops")
    args = parser.parse_args()
    root = tk.Tk()
    scans, messages = queue.Queue(maxsize=5), queue.Queue(maxsize=100)
    sensors = {"ultrasonic": 32.4, "mq4": 180, "mq137": 95, "mq135": 240}
    unavailable = args.scenario == "unavailable"
    # Sample driver phases, as LidarDriverSupervisor reports them to the monitor.
    driver = {"loading": {"phase": "starting", "message": "Starting the LiDAR driver"},
              "unavailable": {"phase": "failed", "message": "No serial port is streaming LiDAR "
                              "data; check its USB cable and power"}}.get(
        args.scenario, {"phase": "running", "message": "LiDAR driver running"})
    driver["since"] = time.monotonic()
    health = {"lidar": not unavailable, "sensors": not unavailable, "lidar_driver": driver,
              "firebase": False, "robot_state": "PREVIEW",
              "operator_controls": {"supported": True, "connected": not unavailable, "pending": False,
                                    "emergency_stopped": False if not unavailable else None,
                                    "ultrasonic_enabled": True if not unavailable else None,
                                    "can_configure": not unavailable, "can_release": False,
                                    "can_continue": False,
                                    "can_reset": False, "message": "Preview only: no robot commands sent"},
              "manual_start": {"enabled": not unavailable,
                               "message": "Preview: ready to start" if not unavailable else "Waiting for both controllers"}}

    def preview_start():
        health["robot_state"] = "OUTBOUND"
        health["operator_controls"]["can_configure"] = False
        health["manual_start"] = {"enabled": False, "message": "Preview: start accepted; no robot commands sent"}
        return health["manual_start"]

    def preview_operator(action, enabled=None):
        status = health["operator_controls"]
        if action == "stop":
            status.update(emergency_stopped=True, can_release=not unavailable,
                          can_continue=not unavailable,
                          can_reset=not unavailable, can_configure=not unavailable)
            health["manual_start"] = {"enabled": False, "message": "Preview: emergency stop applied"}
        elif action == "continue":
            status.update(emergency_stopped=False, can_release=False, can_continue=False,
                          can_reset=False)
            health["robot_state"] = "OUTBOUND"
            health["manual_start"] = {"enabled": False, "message": "Preview: route continued"}
        elif action == "release":
            status.update(emergency_stopped=False, can_release=False, can_continue=False,
                          can_reset=False)
            health["robot_state"] = "IDLE"
            health["manual_start"] = {"enabled": True, "message": "Preview: ready for a new start"}
        elif action == "sonar":
            status["ultrasonic_enabled"] = enabled
        status["message"] = f"Preview {action}: no robot commands or OS reboot sent"
        messages.put_nowait(status["message"])
        return dict(status)

    gui = LidarGUI(root, scans, messages, read_sensors=lambda: {} if unavailable else sensors,
                   read_health=lambda: health, allow_safety_disable=True, request_start=preview_start,
                   request_emergency_stop=lambda: preview_operator("stop"),
                   request_release_stop=lambda: preview_operator("release"),
                   request_continue_run=lambda: preview_operator("continue"),
                   request_ultrasonic=lambda enabled: preview_operator("sonar", enabled),
                   request_system_reset=lambda: preview_operator("restart"))
    root.title("GARBY - UI preview (sample data)")
    root.geometry(args.size)
    gui.subtitle_lbl.configure(text="Preview: sample data")
    gui.map_caption_lbl.configure(text="Sample data · front up · metres")
    gui.set_ble_status("Preview only - no connection", "#d29922")
    messages.put_nowait("Sample messages only. No robot hardware is connected.")
    distances = {"FRONT_LEFT": 1.65, "FRONT": 2.1, "FRONT_RIGHT": 1.8,
                 "LEFT": 0.8, "RIGHT": 0.85, "BACK_LEFT": 1.5, "BACK": 1.8, "BACK_RIGHT": 1.6}
    if args.scenario == "obstacle":
        distances["FRONT"] = 0.6

    def feed():
        data = {side: (distances[side], distances[side], 5) for side, *_ in DIRECTIONS}
        try:
            snapshot = LidarMapSnapshot(
                synthetic_scan_points(args.scenario == "obstacle"), time.monotonic(), 0)
            # The synthetic room's side walls as a wall line fit would report them
            # (hallway_geometry.py); the obstacle scenario also names its zone.
            sides = "SIDES:WL=80.0|WR=85.0|H=0.00" + (
                "|Z=FRONT_BOX,TURN_FRONT" if args.scenario == "obstacle" else "")
            scans.put_nowait((data, 0.1, 8.0, 22.0, sides, snapshot))
        except queue.Full:
            pass
        if args.scenario != "stale":
            root.after(250, feed)

    # Loading and unavailable show the map without any scan.
    if args.scenario not in ("loading", "unavailable"):
        feed()
    root.mainloop()


if __name__ == "__main__":
    main()
