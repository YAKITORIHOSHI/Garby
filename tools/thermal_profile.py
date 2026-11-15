#!/usr/bin/env python3
"""Where the Pi's heat comes from: temperature, clock, fan and CPU use per process.

    python3 tools/thermal_profile.py --seconds 600 --out ~/garby-logs/thermal-headless.csv

Run it beside the bridge, once for each way of running GARBY (idle, monitor over
ssh -Y, monitor on a local display, headless), and compare the summaries. It
only reads /proc and /sys: it starts, stops and changes nothing.

Per sample (default every 5 s): CPU temperature, CPU frequency, the under-voltage
alarm, the fan step, total CPU use, and the CPU use of every process whose
command line matches one of the watched names (the bridge, the LiDAR driver, the
display server, ssh). For the bridge, the busiest threads are named in the summary.
"""
import argparse
import csv
import glob
import os
import sys
import time

WATCHED = {
    "bridge": ("final_w_serial.py",),
    "lidar_driver": ("ydlidar",),
    "display": ("Xorg", "Xwayland", "Xvfb", "gnome-shell", "labwc", "wayfire"),
    "ssh": ("sshd",),
}
TICKS = os.sysconf("SC_CLK_TCK")


def read(path, default=None):
    try:
        with open(path, encoding="ascii", errors="replace") as handle:
            return handle.read().strip()
    except OSError:
        return default


def temperature_c():
    raw = read("/sys/class/thermal/thermal_zone0/temp")
    try:
        value = float(raw)
    except (TypeError, ValueError):
        return None
    return value / 1000.0 if value > 200.0 else value


def frequency_mhz():
    raw = read("/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq")
    return None if raw is None or not raw.isdigit() else int(raw) / 1000.0


def fan_step():
    for device in sorted(glob.glob("/sys/class/thermal/cooling_device*")):
        if "fan" in (read(os.path.join(device, "type")) or ""):
            raw = read(os.path.join(device, "cur_state"))
            return int(raw) if raw and raw.isdigit() else None
    return None


def undervoltage():
    # The hwmon alarm, not soc:firmware/get_throttled (that file logs a kernel warning).
    for sensor in glob.glob("/sys/class/hwmon/hwmon*"):
        if read(os.path.join(sensor, "name")) == "rpi_volt":
            raw = read(os.path.join(sensor, "in0_lcrit_alarm"))
            return int(raw) if raw and raw.isdigit() else None
    return None


def total_jiffies():
    fields = (read("/proc/stat") or "").splitlines()[0].split()[1:]
    values = [int(value) for value in fields]
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    return sum(values), idle


def stat_jiffies(path):
    """utime + stime of a /proc/<pid>/stat or task stat file, or None."""
    raw = read(path)
    if not raw:
        return None
    fields = raw[raw.rfind(")") + 2:].split()
    try:
        return int(fields[11]) + int(fields[12])
    except (IndexError, ValueError):
        return None


def watched_processes():
    found = {}
    for entry in os.listdir("/proc"):
        if not entry.isdigit() or int(entry) == os.getpid():
            continue
        command = (read(f"/proc/{entry}/cmdline") or "").replace("\0", " ")
        if not command or "thermal_profile.py" in command:
            continue
        for name, needles in WATCHED.items():
            if any(needle in command for needle in needles):
                found.setdefault(name, []).append(int(entry))
    return found


def bridge_threads(pid):
    threads = {}
    for task in glob.glob(f"/proc/{pid}/task/*"):
        jiffies = stat_jiffies(os.path.join(task, "stat"))
        if jiffies is not None:
            threads[os.path.basename(task)] = (read(os.path.join(task, "comm")) or "?", jiffies)
    return threads


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--seconds", type=float, default=600.0)
    parser.add_argument("--interval", type=float, default=5.0)
    parser.add_argument("--out", help="CSV file (default: print the summary only)")
    args = parser.parse_args()
    cores = os.cpu_count() or 1
    names = list(WATCHED)
    rows = []
    last_total, last_idle = total_jiffies()
    last_proc = {}
    first_threads, last_threads, bridge_pid = {}, {}, None
    started = time.monotonic()
    while time.monotonic() - started < args.seconds:
        time.sleep(max(0.2, args.interval))
        total, idle = total_jiffies()
        span = max(1, total - last_total)
        row = {"t_s": round(time.monotonic() - started, 1), "temp_c": temperature_c(),
               "freq_mhz": frequency_mhz(), "fan_step": fan_step(), "undervoltage": undervoltage(),
               # 100 % = all cores busy.
               "cpu_total_pct": round(100.0 * (span - (idle - last_idle)) / span, 1)}
        processes = watched_processes()
        now_proc = {}
        for name in names:
            used = 0
            for pid in processes.get(name, ()):
                jiffies = stat_jiffies(f"/proc/{pid}/stat")
                if jiffies is None:
                    continue
                now_proc[pid] = jiffies
                if pid in last_proc:
                    used += jiffies - last_proc[pid]
            # 100 % = one core busy.
            row[f"{name}_pct"] = round(100.0 * used / (span / cores), 1) if processes.get(name) else None
        pids = processes.get("bridge") or []
        if pids:
            if bridge_pid != pids[0]:
                bridge_pid, first_threads = pids[0], bridge_threads(pids[0])
            last_threads = bridge_threads(pids[0])
        last_total, last_idle, last_proc = total, idle, now_proc
        rows.append(row)
    if not rows:
        print("No samples.")
        return 1
    if args.out:
        path = os.path.expanduser(args.out)
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        print(f"Wrote {len(rows)} samples to {path}")

    def mean(key):
        values = [row[key] for row in rows if row[key] is not None]
        return sum(values) / len(values) if values else None

    def show(label, value, unit="", digits=1):
        print(f"  {label:<26} " + ("n/a" if value is None else f"{value:.{digits}f}{unit}"))
    temps = [row["temp_c"] for row in rows if row["temp_c"] is not None]
    print(f"Summary over {rows[-1]['t_s']:.0f} s ({len(rows)} samples, {cores} cores)")
    show("Temperature, mean", mean("temp_c"), " C")
    show("Temperature, highest", max(temps) if temps else None, " C")
    show("Temperature, last - first", temps[-1] - temps[0] if len(temps) > 1 else None, " C")
    show("CPU clock, mean", mean("freq_mhz"), " MHz", 0)
    show("Fan step, mean", mean("fan_step"))
    show("All cores busy", mean("cpu_total_pct"), " %")
    for name in names:
        show(f"{name} (100 % = one core)", mean(f"{name}_pct"), " %")
    if any(row["undervoltage"] for row in rows):
        print("  UNDER-VOLTAGE was reported during the run: check the 5 V supply first.")
    if bridge_pid and first_threads and last_threads:
        elapsed = max(1e-6, rows[-1]["t_s"] - rows[0]["t_s"] + args.interval)
        used = sorted(((last_threads[tid][1] - first_threads.get(tid, (None, last_threads[tid][1]))[1],
                        last_threads[tid][0]) for tid in last_threads), reverse=True)[:8]
        print("  Busiest bridge threads (100 % = one core):")
        for jiffies, name in used:
            if jiffies > 0:
                print(f"    {name:<24} {100.0 * jiffies / TICKS / elapsed:5.1f} %")
    return 0


if __name__ == "__main__":
    sys.exit(main())
