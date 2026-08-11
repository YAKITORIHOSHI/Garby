#!/usr/bin/env python3
"""Compatibility launcher for the single reviewed GARBY Raspberry Pi bridge.

The Android project previously carried an independent snapshot of the Pi
bridge. Keeping two implementations allowed safety and Firebase fixes to drift.
This entry point now forwards to ../RasPi/final_w_serial.py so there is only one
production implementation.
"""

from __future__ import annotations

import os
from pathlib import Path
import sys


def main() -> None:
    production = (
        Path(__file__).resolve().parent.parent / "RasPi" / "final_w_serial.py"
    )
    if not production.is_file():
        raise SystemExit(
            "Reviewed GARBY bridge not found at "
            f"{production}. Deploy and run the RasPi directory instead."
        )

    os.chdir(production.parent)
    os.execv(
        sys.executable,
        [sys.executable, str(production), *sys.argv[1:]],
    )


if __name__ == "__main__":
    main()
