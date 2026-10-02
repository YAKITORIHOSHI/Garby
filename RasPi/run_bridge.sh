#!/usr/bin/env bash
set -euo pipefail

# ROS and the locally built YDLIDAR overlay must both be sourced. Merely
# sourcing /opt/ros/humble does not make a workspace-built driver discoverable.
source "${GARBY_ROS_SETUP:-/opt/ros/humble/setup.bash}"
source "${GARBY_LIDAR_WORKSPACE_SETUP:-/home/garby/ros2_ws/install/setup.bash}"
bridge_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "${GARBY_PYTHON_BIN:-/usr/bin/python3}" "$bridge_dir/final_w_serial.py" "$@"
