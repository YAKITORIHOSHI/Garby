"""Locate Firebase credentials without depending on usernames or the launch CWD."""
import json
import os
from pathlib import Path


def resolve_firebase_credentials(credential_path=None):
    """Prefer an explicit file; otherwise find exactly one local service account.

    Relative overrides and automatic discovery are based on this module's
    directory, which is deployed beside the runtime and its Firebase folder.
    Client configuration files are never selected as Admin SDK credentials.
    """
    base = Path(__file__).resolve().parent
    configured = credential_path or os.environ.get("GARBY_FIREBASE_CREDENTIALS")
    if configured:
        path = Path(configured).expanduser()
        if not path.is_absolute():
            path = base / path
        if not path.is_file():
            raise FileNotFoundError(f"Configured Firebase credential file not found: {path}")
        return str(path)

    folder = base / "Firebase"
    matches = []
    for path in sorted(folder.glob("*.json")):
        try:
            config = json.loads(path.read_text(encoding="utf-8-sig"))
        except (OSError, ValueError):
            continue
        if isinstance(config, dict) and config.get("type") == "service_account":
            matches.append(path)
    if not matches:
        raise FileNotFoundError(
            f"No service-account JSON found in {folder}; "
            "set GARBY_FIREBASE_CREDENTIALS to use another location."
        )
    if len(matches) != 1:
        raise ValueError(
            f"Multiple service-account JSON files found in {folder}; "
            "set GARBY_FIREBASE_CREDENTIALS to select one explicitly."
        )
    return str(matches[0])
