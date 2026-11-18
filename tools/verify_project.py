"""Verify the robot repository; optionally check a separately cloned Android app."""
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
    parser.add_argument("--host-only", action="store_true", help="Skip target builds; report them as unverified")
    parser.add_argument("--mobile-repo", type=Path, help="Opt in to checks of a separate Garby-Mobile-App checkout")
    args = parser.parse_args()
    mobile = args.mobile_repo.resolve() if args.mobile_repo else None
    if mobile is not None:
        if not (mobile / "app/build.gradle.kts").is_file():
            parser.error("--mobile-repo must point to a separate Android project")
        os.environ["GARBY_MOBILE_APP_ROOT"] = str(mobile)
    with tempfile.TemporaryDirectory(prefix="garby-syntax-") as directory:
        sources = (list((ROOT / "RasPi").glob("*.py")) + list((ROOT / "tools").glob("*.py"))
                   + list((ROOT / "tests/pi").glob("*.py")))
        if mobile is not None:
            sources += list((mobile / "tools").glob("*.py"))
        for index, source in enumerate(sources):
            py_compile.compile(str(source), cfile=str(Path(directory) / f"{index}.pyc"), doraise=True)
    # Tests live in tests/, outside the deployable RasPi/ folder (owner, 2026-11-16).
    run([sys.executable, "-B", "-m", "unittest", "discover", "-s", "tests/pi", "-p", "test_*.py"])
    run([sys.executable, "-B", "tools/run_firmware_tests.py"])
    run([sys.executable, "-B", "tools/source_sanity.py", "."])
    legacy_audit = ROOT / ".github/skills/garby-robot-maintainer/scripts/audit_project.py"
    if legacy_audit.is_file():
        run([sys.executable, "-B", str(legacy_audit), "."])
    else:
        print("SKIP legacy audit: script is not supplied in this workspace; audit coverage is unverified.", flush=True)
    if mobile is not None:
        run([sys.executable, "-B", str(mobile / "tools/static_quality_check.py")], cwd=mobile)
    if args.host_only:
        print("HOST CHECKS PASS; ESP32 target builds and Android checks SKIPPED.")
        return
    cli = shutil.which("arduino-cli")
    if not cli:
        raise SystemExit("arduino-cli is required for full validation")
    for sketch in ("BLE_Receiver-Final", "MAIN_EXECUTOR_CODE"):
        fqbn = "esp32:esp32:esp32s3" if sketch == "BLE_Receiver-Final" else "esp32:esp32:esp32"
        run([cli, "compile", "--fqbn", fqbn, "--output-dir", str(ROOT / "artifacts/firmware" / sketch), sketch])
    if mobile is not None:
        wrapper = mobile / ("gradlew.bat" if sys.platform == "win32" else "gradlew")
        run([str(wrapper), "testDebugUnitTest", "assembleDebug", "assembleRelease", "lintDebug", "--console=plain"], cwd=mobile)
    print("ALL SOFTWARE CHECKS PASS. Hardware and live Firebase acceptance remain required.")

if __name__ == "__main__":
    main()
