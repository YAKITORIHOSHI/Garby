#!/usr/bin/env python3
"""Compatibility launcher; the maintained Pi runtime is ../RasPi/final_w_serial.py.

This file formerly contained a divergent copy. Deploy the complete RasPi/
directory to the robot, not this Android project directory.
"""
from pathlib import Path
import runpy
import sys

if __name__ == "__main__":
    runtime = Path(__file__).resolve().parent.parent / "RasPi" / "final_w_serial.py"
    if not runtime.is_file():
        raise SystemExit("Deploy RasPi/final_w_serial.py together with bridge_core.py.")
    sys.path.insert(0, str(runtime.parent))
    runpy.run_path(str(runtime), run_name="__main__")
