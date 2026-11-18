"""The Pi tests live outside the deployable RasPi/ folder (owner, 2026-11-16).

Importing this module puts RasPi/ first on sys.path, so the tests import the same modules
the robot runs. ROOT and RASPI locate project files without depending on the test folder.
"""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RASPI = ROOT / "RasPi"
if str(RASPI) not in sys.path:
    sys.path.insert(0, str(RASPI))
