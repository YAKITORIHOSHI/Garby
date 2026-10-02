"""Compile and execute the actual firmware against deterministic host peripherals.

Needs MSVC Build Tools on Windows, or g++/clang++ on Unix. No robot is touched.
"""
from pathlib import Path
import os
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parent.parent
TESTS = ROOT / "tools" / "firmware_tests"

def main():
    with tempfile.TemporaryDirectory(prefix="garby-firmware-tests-") as directory:
        build = Path(directory)
        # Thin includes map the firmware's library headers to host substitutes.
        for name in ("ESP32Servo.h", "FastAccelStepper.h", "HardwareSerial.h", "HX711.h", "nvs_flash.h", "esp_bt_main.h", "freertos/FreeRTOS.h", "freertos/queue.h", "freertos/semphr.h", "soc/soc.h", "soc/rtc_cntl_reg.h"):
            path = build / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text('#pragma once\n#include "Arduino.h"\n')
        environment = os.environ.copy()
        if os.name == "nt":
            vswhere = Path(os.environ.get("ProgramFiles(x86)", "C:/Program Files (x86)")) / "Microsoft Visual Studio/Installer/vswhere.exe"
            installation = subprocess.check_output([str(vswhere), "-latest", "-products", "*", "-requires", "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-property", "installationPath"], text=True).strip()
            if not installation:
                raise SystemExit("MSVC C++ Build Tools are required for host firmware tests")
            vcvars = Path(installation) / "VC/Auxiliary/Build/vcvars64.bat"
            batch = build / "compiler-environment.cmd"
            batch.write_text(f'@echo off\ncall "{vcvars}" >nul\nif errorlevel 1 exit /b 1\nset\n')
            configured = subprocess.check_output(["cmd", "/d", "/c", str(batch)], text=True)
            for line in configured.splitlines():
                if "=" in line:
                    key, value = line.split("=", 1)
                    environment[key] = value
            compiler = shutil.which("cl.exe", path=environment.get("Path", environment.get("PATH")))
        else:
            compiler = shutil.which("g++") or shutil.which("clang++")
        if not compiler:
            raise SystemExit("No host C++ compiler available")
        for source in sorted(TESTS.glob("test_*.cpp")):
            executable = build / (source.stem + (".exe" if os.name == "nt" else ""))
            if os.name == "nt":
                command = [compiler, "/nologo", "/std:c++17", "/EHsc", "/utf-8", "/W3", f"/I{TESTS / 'stubs'}", f"/I{build}", str(source), f"/Fe:{executable}", f"/Fo:{build / (source.stem + '.obj')}"]
            else:
                command = [compiler, "-std=c++17", "-I", str(TESTS / "stubs"), "-I", str(build), str(source), "-o", str(executable)]
            result = subprocess.run(command, cwd=build, env=environment, capture_output=True, text=True)
            print(result.stdout + result.stderr)
            result.check_returncode()
            result = subprocess.run([str(executable)], env=environment, capture_output=True, text=True, timeout=15)
            print(result.stdout + result.stderr)
            result.check_returncode()
    print("Firmware behavioral checks PASS; hardware timing/electrical acceptance remains required.")

if __name__ == "__main__":
    main()
