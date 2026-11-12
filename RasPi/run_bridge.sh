#!/usr/bin/env bash
set -eo pipefail

# ROS-generated setup scripts may legitimately read unset variables. Enable
# nounset only after sourcing them. Never rely on interactive .bashrc at boot.
bridge_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
default_ros_setup=/opt/ros/jazzy/setup.bash
if [[ ! -r "$default_ros_setup" ]]; then
    default_ros_setup=/opt/ros/humble/setup.bash
fi
ros_setup="${GARBY_ROS_SETUP:-$default_ros_setup}"
if [[ ! -r "$ros_setup" ]]; then
    echo "GARBY: ROS setup missing: $ros_setup; set GARBY_ROS_SETUP." >&2
    exit 1
fi
source "$ros_setup"
# An explicitly configured overlay must exist. A system-installed driver can
# work without an overlay; preflight below verifies package discovery.
overlay="${GARBY_LIDAR_WORKSPACE_SETUP-$HOME/ros2_ws/install/setup.bash}"
if [[ -n "$overlay" ]]; then
    if [[ ! -r "$overlay" ]]; then
        echo "GARBY: LiDAR overlay missing: $overlay; configure GARBY_LIDAR_WORKSPACE_SETUP." >&2
        exit 1
    fi
    source "$overlay"
fi
set -u
# Optional user-local dependencies allow system Python + ROS without a venv.
local_packages="${GARBY_PYTHON_PACKAGES:-$HOME/.local/share/garby/python-packages}"
if [[ -d "$local_packages" ]]; then
    export PYTHONPATH="$local_packages${PYTHONPATH:+:$PYTHONPATH}"
fi
if [[ "${1:-}" == --check-runtime ]]; then
    shift
    exec "${GARBY_PYTHON_BIN:-/usr/bin/python3}" "$bridge_dir/check_runtime.py" "$@"
fi
# Scan only by default; add --connect only with the wheels lifted.
if [[ "${1:-}" == --check-ble ]]; then
    shift
    exec "${GARBY_PYTHON_BIN:-/usr/bin/python3}" "$bridge_dir/check_ble.py" "$@"
fi
exec "${GARBY_PYTHON_BIN:-/usr/bin/python3}" "$bridge_dir/final_w_serial.py" "$@"
