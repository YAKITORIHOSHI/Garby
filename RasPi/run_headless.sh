#!/usr/bin/env bash
# GARBY without a screen: the phone app is the monitor and the controls.
#
#   bash RasPi/run_headless.sh start    start the bridge in the background, log to a file
#   bash RasPi/run_headless.sh stop     stop it (SIGTERM; it shuts down within 5 s)
#   bash RasPi/run_headless.sh status   is it running, and what does it report
#   bash RasPi/run_headless.sh logs     follow the log (Ctrl-C leaves the bridge running)
#   bash RasPi/run_headless.sh doctor   check the Pi, the BLE scan and the cloud key
#   bash RasPi/run_headless.sh run      run in the foreground (what the boot service uses)
#
# Starting the bridge never starts a route: it comes up with every check enabled
# and waits for the controllers and a deliberate start from the app.
set -uo pipefail

bridge_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
state_dir="${GARBY_STATE_DIR:-$HOME/.local/state/garby}"
log_dir="${GARBY_LOG_DIR:-$HOME/garby-logs}"
pid_file="$state_dir/bridge.pid"
status_file="${GARBY_STATUS_FILE:-$state_dir/bridge-status.json}"
python_bin="${GARBY_PYTHON_BIN:-/usr/bin/python3}"

running_pid() {
    local pid
    [[ -r "$pid_file" ]] || return 1
    pid="$(<"$pid_file")"
    [[ "$pid" =~ ^[0-9]+$ ]] || return 1
    # The PID must still be this bridge, not a recycled number.
    [[ -r "/proc/$pid/cmdline" ]] && tr '\0' ' ' <"/proc/$pid/cmdline" | grep -q 'final_w_serial.py' || return 1
    echo "$pid"
}

case "${1:-}" in
start)
    if pid="$(running_pid)"; then
        echo "GARBY is already running (PID $pid). Use: $0 status"
        exit 0
    fi
    mkdir -p "$state_dir" "$log_dir"
    if ! bash "$bridge_dir/run_bridge.sh" --check-runtime >"$log_dir/last-preflight.txt" 2>&1; then
        echo "Runtime check failed; not started. See $log_dir/last-preflight.txt" >&2
        tail -n 15 "$log_dir/last-preflight.txt" >&2
        exit 1
    fi
    export GARBY_STATUS_FILE="$status_file"
    # Its own session: closing the terminal or the SSH connection leaves it running.
    setsid nohup bash "$bridge_dir/run_bridge.sh" --headless --log-dir "$log_dir" \
        >"$log_dir/console.txt" 2>&1 </dev/null &
    launcher=$!
    # run_bridge.sh ends in "exec python": the launcher PID becomes the bridge.
    for _ in $(seq 1 50); do
        if [[ -r "/proc/$launcher/cmdline" ]] &&
           tr '\0' ' ' <"/proc/$launcher/cmdline" | grep -q 'final_w_serial.py'; then
            echo "$launcher" >"$pid_file"
            echo "GARBY started headless (PID $launcher)."
            echo "Log: $log_dir/bridge.log    Status: $0 status"
            exit 0
        fi
        kill -0 "$launcher" 2>/dev/null || break
        sleep 0.2
    done
    echo "GARBY did not start. Last console lines:" >&2
    tail -n 20 "$log_dir/console.txt" >&2
    exit 1
    ;;
stop)
    if ! pid="$(running_pid)"; then
        echo "GARBY is not running."
        rm -f "$pid_file"
        exit 0
    fi
    kill -TERM "$pid"
    for _ in $(seq 1 40); do
        kill -0 "$pid" 2>/dev/null || { rm -f "$pid_file"; echo "GARBY stopped."; exit 0; }
        sleep 0.2
    done
    echo "GARBY (PID $pid) did not stop within 8 s. It is still running." >&2
    echo "The controllers stop the wheels by themselves when the Pi's path packets end." >&2
    exit 1
    ;;
status)
    if pid="$(running_pid)"; then
        echo "GARBY is running (PID $pid)."
    else
        echo "GARBY is not running."
    fi
    if [[ -r "$status_file" ]]; then
        "$python_bin" -I - "$status_file" <<'PY'
import json, os, sys, time
path = sys.argv[1]
try:
    with open(path, encoding="utf-8") as handle:
        status = json.load(handle)
except (OSError, ValueError) as exc:
    print(f"Status file unreadable: {exc}")
    raise SystemExit(0)
age = time.time() - os.path.getmtime(path)
stopped = status.get("emergencyStopped")
print(f"Reported {age:.0f} s ago" + ("" if age < 20 else "  (old: the bridge is not updating it)"))
print(f"  Robot state     {status.get('robotState', 'UNKNOWN')}")
print(f"  Controllers     {'connected' if status.get('bleConnected') else 'not connected'}")
print(f"  LiDAR           {'ok' if status.get('lidarHealthy') else 'not ready'}")
print(f"  Cloud           {status.get('cloud', 'unknown')}")
print("  Emergency stop  " + ("LATCHED" if stopped is True else "released" if stopped is False
                              else "unknown"))
for label, key in (("Held by", "motionHold"), ("Zones", "zones"), ("Route step", "routeLeg")):
    if status.get(key):
        print(f"  {label:<15} {status[key]}")
print(f"  In view         {status.get('people', 0)} likely person(s), {status.get('objects', 0)} object(s)")
print(f"  Message         {status.get('message', '')}")
PY
    else
        echo "No status yet ($status_file)."
    fi
    ;;
logs)
    [[ -r "$log_dir/bridge.log" ]] || { echo "No log yet in $log_dir" >&2; exit 1; }
    exec tail -n 60 -F "$log_dir/bridge.log"
    ;;
doctor)
    failed=0
    echo "== Runtime"; bash "$bridge_dir/run_bridge.sh" --check-runtime || failed=1
    echo "== BLE (scan only, nothing is connected or moved)"
    bash "$bridge_dir/run_bridge.sh" --check-ble || failed=1
    echo "== Cloud key"
    "$python_bin" "$bridge_dir/check_firebase.py" || failed=1
    [[ $failed -eq 0 ]] && echo "All checks passed." || echo "At least one check failed (see above)."
    exit $failed
    ;;
run)
    mkdir -p "$state_dir" "$log_dir"
    export GARBY_STATUS_FILE="$status_file"
    exec bash "$bridge_dir/run_bridge.sh" --headless --log-dir "$log_dir"
    ;;
*)
    sed -n '2,12p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    exit 2
    ;;
esac
