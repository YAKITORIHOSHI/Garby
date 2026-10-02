"""One repeatable software validation command; never flashes hardware."""
import argparse
import os
from pathlib import Path
import py_compile
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parent.parent

def run(command, cwd=ROOT):
    print("RUN " + " ".join(map(str, command)), flush=True)
    environment = os.environ.copy()
    environment.update(PYTHONUTF8="1", PYTHONIOENCODING="utf-8")
    result = subprocess.run(command, cwd=cwd, env=environment, capture_output=True,
                            text=True, encoding="utf-8", errors="replace")
    print(result.stdout + result.stderr, flush=True)
    result.check_returncode()

def main():
    # Windows consoles may otherwise fail while printing Unicode audit output.
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(encoding="utf-8", errors="replace")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host-only", action="store_true", help="Skip ESP32 target and Android builds; report those as unverified")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix="garby-syntax-") as directory:
        sources = list((ROOT / "RasPi").glob("*.py")) + list((ROOT / "tools").glob("*.py")) + [ROOT / "Garby_MobileApp/final_w_serial.py"]
        for index, source in enumerate(sources):
            py_compile.compile(str(source), cfile=str(Path(directory) / f"{index}.pyc"), doraise=True)
    run([sys.executable, "-B", "-m", "unittest", "discover", "-s", "RasPi", "-p", "test_*.py"])
    run([sys.executable, "-B", "tools/run_firmware_tests.py"])
    run([sys.executable, "-B", "tools/source_sanity.py", "."])
    run([sys.executable, "-B", ".github/skills/garby-robot-maintainer/scripts/audit_project.py", "."])
    run([sys.executable, "-B", "Garby_MobileApp/tools/static_quality_check.py"])
    if args.host_only:
        print("HOST CHECKS PASS; ESP32 target builds and Android checks SKIPPED.")
        return
    cli = shutil.which("arduino-cli")
    if not cli:
        raise SystemExit("arduino-cli is required for full validation")
    for sketch in ("BLE_Receiver-Final", "NAPHTALI_CODE_V2"):
        run([cli, "compile", "--fqbn", "esp32:esp32:esp32", "--output-dir", str(ROOT / "artifacts/firmware" / sketch), sketch])
    wrapper = ROOT / "Garby_MobileApp" / ("gradlew.bat" if sys.platform == "win32" else "gradlew")
    run([str(wrapper), "testDebugUnitTest", "assembleDebug", "assembleRelease", "lintDebug", "--console=plain"], cwd=wrapper.parent)
    print("ALL SOFTWARE CHECKS PASS. Hardware and live Firebase acceptance remain required.")

if __name__ == "__main__":
    main()
