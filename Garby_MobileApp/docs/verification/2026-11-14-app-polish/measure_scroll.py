"""User-authorized, repeatable app scrolling. Never taps or sends a robot command."""
import json
import hashlib
from pathlib import Path
import re
import subprocess
import sys
import time

ADB = r"C:\Users\napht\AppData\Local\Android\Sdk\platform-tools\adb.exe"
PACKAGE = "com.thesis.garby"
OUTPUT = Path(__file__).resolve().parent


def adb(*args):
    return subprocess.check_output([ADB, *args], text=True, encoding="utf-8", errors="replace")


def foreground():
    activity = adb("shell", "dumpsys", "activity", "activities")
    resumed = [line for line in activity.splitlines() if "mResumedActivity:" in line]
    if not resumed or PACKAGE not in resumed[0]:
        raise RuntimeError("GARBY is not foreground; no gesture was sent")


def swipe(start, end):
    foreground()
    adb("shell", "input", "swipe", "360", str(start), "360", str(end), "400")
    time.sleep(0.25)


def verify_optimized(apk):
    package = adb("shell", "dumpsys", "package", PACKAGE)
    if "DEBUGGABLE" in package or "TEST_ONLY" in package:
        raise RuntimeError("A debug/test-only build is installed; optimized measurement rejected")
    installed = adb("shell", "pm", "path", PACKAGE).strip().removeprefix("package:")
    actual = adb("shell", "sha256sum", installed).split()[0]
    expected = hashlib.sha256(Path(apk).read_bytes()).hexdigest()
    if actual != expected:
        raise RuntimeError("Installed APK differs from the verified optimized APK")
    return actual


def main():
    label = sys.argv[1]
    if label not in {"debug", "optimized"}:
        raise ValueError("Expected debug or optimized")
    expected_apk = sys.argv[2] if label == "optimized" else None
    initial_hash = verify_optimized(expected_apk) if expected_apk else None
    # Return to the top without touching the bottom action dock. Excluded from the sample.
    for _ in range(6):
        swipe(450, 1170)
    time.sleep(0.5)
    adb("shell", "dumpsys", "gfxinfo", PACKAGE, "reset")
    for start, end in [(1170, 450)] * 3 + [(450, 1170)] * 3:
        swipe(start, end)
    time.sleep(0.5)
    graphics = adb("shell", "dumpsys", "gfxinfo", PACKAGE)
    memory = adb("shell", "dumpsys", "meminfo", PACKAGE)
    final_hash = verify_optimized(expected_apk) if expected_apk else None
    (OUTPUT / f"{label}-scroll-gfxinfo.txt").write_text(graphics, encoding="utf-8")
    (OUTPUT / f"{label}-scroll-meminfo.txt").write_text(memory, encoding="utf-8")
    fields = {
        "Frames": r"Total frames rendered: (\d+)",
        "JankyFrames": r"Janky frames: (\d+)",
        "MedianMs": r"50th percentile: (\d+)ms",
        "P90Ms": r"90th percentile: (\d+)ms",
        "P99Ms": r"99th percentile: (\d+)ms",
        "SlowUiThreadFrames": r"Number Slow UI thread: (\d+)",
    }
    summary = {key: int(re.search(pattern, graphics)[1]) for key, pattern in fields.items()}
    summary["PssKiB"] = int(re.search(r"TOTAL PSS:\s+(\d+)", memory)[1])
    summary["Sequence"] = "Three upward and three downward swipes; 400 ms each, 250 ms pauses"
    summary["Coordinates"] = "x=360, y=1170 to 450 and back; 720x1600 screen"
    summary["InstalledApkHashBefore"] = initial_hash
    summary["InstalledApkHashAfter"] = final_hash
    (OUTPUT / f"{label}-scroll-summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
