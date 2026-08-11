#!/usr/bin/env python3
"""Static safety audit for a GARBY project tree.

This script checks project structure and high-value safety invariants. It is not
an Arduino compiler, hardware test, or proof of safe operation.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Optional


@dataclass
class CheckResult:
    severity: str
    check: str
    detail: str
    file: str = ""


class Audit:
    def __init__(self, root: Path) -> None:
        self.root = root.resolve()
        self.results: list[CheckResult] = []

    def add(self, severity: str, check: str, detail: str, path: Optional[Path] = None) -> None:
        rel = ""
        if path is not None:
            try:
                rel = str(path.resolve().relative_to(self.root))
            except ValueError:
                rel = str(path)
        self.results.append(CheckResult(severity, check, detail, rel))

    def passed(self, check: str, detail: str, path: Optional[Path] = None) -> None:
        self.add("PASS", check, detail, path)

    def warn(self, check: str, detail: str, path: Optional[Path] = None) -> None:
        self.add("WARN", check, detail, path)

    def fail(self, check: str, detail: str, path: Optional[Path] = None) -> None:
        self.add("FAIL", check, detail, path)


def _candidate_score(path: Path, preferred_parts: Iterable[str]) -> tuple[int, int, str]:
    text = str(path).lower()
    penalty = 0
    if "garby-robot-maintainer" in text or "/skills/" in text or "/skill/" in text:
        penalty += 100
    for part in preferred_parts:
        if part.lower() in text:
            penalty -= 10
    return penalty, len(path.parts), text


def find_file(root: Path, patterns: Iterable[str], preferred_parts: Iterable[str] = ()) -> Optional[Path]:
    candidates: list[Path] = []
    for pattern in patterns:
        candidates.extend(path for path in root.rglob(pattern) if path.is_file())
    if not candidates:
        return None
    unique = sorted(set(candidates), key=lambda p: _candidate_score(p, preferred_parts))
    return unique[0]


def read_text(path: Optional[Path]) -> str:
    if path is None:
        return ""
    return path.read_text(encoding="utf-8", errors="replace")


def parse_number(text: str, name: str) -> Optional[float]:
    patterns = [
        rf"#define\s+{re.escape(name)}\s+([0-9]+(?:\.[0-9]+)?)",
        rf"\b{re.escape(name)}\s*=\s*([0-9]+(?:\.[0-9]+)?)",
    ]
    for pattern in patterns:
        match = re.search(pattern, text)
        if match:
            return float(match.group(1))
    return None


def get_block(text: str, start_marker: str, end_marker: str) -> str:
    start = text.find(start_marker)
    if start < 0:
        return ""
    end = text.find(end_marker, start + len(start_marker))
    if end < 0:
        end = len(text)
    return text[start:end]


def check_required_files(audit: Audit) -> dict[str, Optional[Path]]:
    files = {
        "architecture": find_file(
            audit.root,
            ["SYSTEM_ARCHITECTURE.md", "SYSTEM_ARCHITECTURE_REVISED.md", "SYSTEM_ARCHITECTURE*.md"],
            ["docs"],
        ),
        "pi": find_file(audit.root, ["final_w_serial.py", "final_w_serial*.py"], ["RaspberryPi", "RasPi"]),
        "bridge": find_file(audit.root, ["BLE_Receiver-Final.ino", "BLE_Receiver-Final*.ino"], ["BLE_Receiver-Final"]),
        "mcu_cpp": find_file(audit.root, ["NAPHTALI_CODE_V2.cpp", "NAPHTALI_CODE_V2*.cpp"], ["NAPHTALI_CODE_V2"]),
        "mcu_h": find_file(audit.root, ["NAPHTALI_CODE_V2.h", "NAPHTALI_CODE_V2*.h"], ["NAPHTALI_CODE_V2"]),
        "mcu_ino": find_file(audit.root, ["NAPHTALI_CODE_V2.ino", "NAPHTALI_CODE_V2*.ino"], ["NAPHTALI_CODE_V2"]),
        "points": find_file(audit.root, ["pointsRun.ino", "pointsRun*.ino"], ["NAPHTALI_CODE_V2"]),
    }
    labels = {
        "architecture": "SYSTEM_ARCHITECTURE.md",
        "pi": "Raspberry Pi program",
        "bridge": "BLE bridge sketch",
        "mcu_cpp": "main-controller implementation",
        "mcu_h": "main-controller header",
        "mcu_ino": "main-controller sketch",
        "points": "pointsRun route file",
    }
    for key, path in files.items():
        if path is None:
            audit.fail("required file", f"Missing {labels[key]}")
        else:
            audit.passed("required file", f"Found {labels[key]}", path)
    return files


def audit_pi(audit: Audit, path: Optional[Path]) -> None:
    if path is None:
        return
    text = read_text(path)

    stale = parse_number(text, "LIDAR_STALE_TIMEOUT_S")
    if stale is None:
        audit.fail("Pi LiDAR stale watchdog", "LIDAR_STALE_TIMEOUT_S was not found", path)
    elif stale <= 1.0:
        audit.passed("Pi LiDAR stale watchdog", f"Fail-closed stale timeout is {stale:g} s", path)
    elif stale <= 2.0:
        audit.warn("Pi LiDAR stale watchdog", f"Timeout is {stale:g} s; verify stopping margin", path)
    else:
        audit.fail("Pi LiDAR stale watchdog", f"Timeout is too slow for motion safety: {stale:g} s", path)

    if "CoalescingBleQueue" in text and "maxlen" in text:
        audit.passed("Pi BLE queue", "Uses a bounded latest-value coalescing queue", path)
    else:
        audit.fail("Pi BLE queue", "No bounded coalescing BLE queue was detected", path)

    if 'f"P:{seq}' in text and 'f"S:{seq}' in text:
        audit.passed("Pi sequenced protocol", "Produces matching sequenced path and steering packets", path)
    else:
        audit.fail("Pi sequenced protocol", "Sequenced P:/S: packet production was not detected", path)

    if "F=S|B=S" in text:
        audit.passed("Pi stale output", "LiDAR unavailability is converted to an explicit stale path packet", path)
    else:
        audit.fail("Pi stale output", "Stale LiDAR does not appear to emit F=S and B=S", path)

    if "asyncio.Lock" in text and "response=acknowledged" in text:
        audit.passed("Pi GATT serialization", "Uses one write lock and acknowledged safety/control writes", path)
    else:
        audit.fail("Pi GATT serialization", "Serialized acknowledged safety writes were not detected", path)

    sensor_period = parse_number(text, "SENSOR_TX_PERIOD_S")
    if sensor_period is None:
        audit.warn("Pi telemetry rate", "SENSOR_TX_PERIOD_S was not found", path)
    elif sensor_period >= 0.5:
        audit.passed("Pi telemetry rate", f"Telemetry period is throttled to {sensor_period:g} s", path)
    else:
        audit.warn("Pi telemetry rate", f"Telemetry period is high rate at {sensor_period:g} s", path)

    if re.search(r"ALLOW_RUNTIME_SAFETY_DISABLE\s*=\s*False", text):
        audit.passed("Pi runtime bypass", "Active-operation safety bypass is disabled by default", path)
    else:
        audit.warn("Pi runtime bypass", "Runtime safety-disable default was not confirmed as False", path)


def audit_bridge(audit: Audit, path: Optional[Path]) -> None:
    if path is None:
        return
    text = read_text(path)

    sensor_marker = 'if (raw.startsWith("SENSOR:"))'
    sensor_block = get_block(text, sensor_marker, 'if (raw == "[RESET]")')
    if sensor_block and "ESP_Serial.println(raw);" in sensor_block and "return;" in sensor_block:
        audit.passed("bridge telemetry isolation", "SENSOR telemetry returns before path logic", path)
    else:
        audit.fail("bridge telemetry isolation", "SENSOR telemetry may fall through into movement logic", path)

    sensor_index = text.find(sensor_marker)
    path_index = text.find('if (raw.startsWith("P:"))')
    if sensor_index >= 0 and path_index >= 0 and sensor_index < path_index:
        audit.passed("bridge strict dispatch order", "Telemetry dispatch occurs before path dispatch", path)
    else:
        audit.warn("bridge strict dispatch order", "Could not confirm strict telemetry-before-path dispatch", path)

    path_timeout = parse_number(text, "PATH_DATA_TIMEOUT_MS")
    if path_timeout is None:
        audit.fail("bridge path watchdog", "PATH_DATA_TIMEOUT_MS was not found", path)
    elif path_timeout <= 2000:
        audit.passed("bridge path watchdog", f"Path timeout is {path_timeout:g} ms", path)
    else:
        audit.fail("bridge path watchdog", f"Path timeout is too slow: {path_timeout:g} ms", path)

    link_timeout = parse_number(text, "LINK_DATA_TIMEOUT_MS")
    if link_timeout is None:
        audit.warn("bridge link watchdog", "LINK_DATA_TIMEOUT_MS was not found", path)
    elif link_timeout <= 10000:
        audit.passed("bridge link watchdog", f"Connection-recovery timeout is {link_timeout:g} ms", path)
    else:
        audit.warn("bridge link watchdog", f"Connection-recovery timeout is long: {link_timeout:g} ms", path)

    clear_count = parse_number(text, "CLEAR_CONFIRM_PACKETS")
    if clear_count is not None and clear_count >= 2:
        audit.passed("bridge STOP clearance", f"Requires {int(clear_count)} clear path packets", path)
    else:
        audit.fail("bridge STOP clearance", "Repeated clear confirmation was not detected", path)

    if re.search(r"stopLatched\s*=\s*true", text):
        audit.passed("bridge fail-closed default", "STOP latch defaults to true", path)
    else:
        audit.fail("bridge fail-closed default", "STOP latch does not clearly default to true", path)

    if "parseSequencedPacket" in text and "sequenceIsNewer" in text:
        audit.passed("bridge sequence freshness", "Parses and rejects stale path sequence numbers", path)
    else:
        audit.fail("bridge sequence freshness", "Freshness checking for path sequences was not detected", path)

    if "seq != lastPathSeq" in text:
        audit.passed("bridge steering binding", "Steering must match the newest path sequence", path)
    else:
        audit.fail("bridge steering binding", "Steering is not visibly bound to the newest path sequence", path)

    disconnect_block = get_block(text, "void onDisconnect", "void onConnParamsUpdate")
    if "STOP:LINK" in disconnect_block and "[RESET]" not in disconnect_block:
        audit.passed("bridge disconnect behavior", "Disconnect holds STOP without automatic reset/return", path)
    else:
        audit.fail("bridge disconnect behavior", "Disconnect STOP behavior is missing or may trigger reset", path)

    if "ESP_Serial.begin(115200" in text:
        audit.passed("bridge UART baud", "Bridge-to-controller UART is 115200 baud", path)
    else:
        audit.fail("bridge UART baud", "Bridge UART is not configured at 115200 baud", path)


def audit_mcu(audit: Audit, cpp_path: Optional[Path], h_path: Optional[Path], ino_path: Optional[Path], points_path: Optional[Path]) -> None:
    cpp = read_text(cpp_path)
    header = read_text(h_path)
    ino = read_text(ino_path)

    if cpp_path is not None:
        if re.search(r"bool\s+shouldStop\s*=\s*true", cpp):
            audit.passed("MCU fail-closed default", "Controller starts with shouldStop=true", cpp_path)
        else:
            audit.fail("MCU fail-closed default", "Controller does not clearly start stopped", cpp_path)

        nudge_block = get_block(cpp, 'if (espMsg.startsWith("N:"))', 'if (espMsg.startsWith("SENSOR:"))')
        if "!shouldStop && pathCommandFresh()" in nudge_block:
            audit.passed("MCU nudge guard", "Nudges require fresh clear path state", cpp_path)
        else:
            audit.fail("MCU nudge guard", "Nudge parsing is not visibly guarded by STOP and freshness", cpp_path)

        stop_block = get_block(cpp, 'if (espMsg.startsWith("STOP"))', 'if (espMsg == "GO")')
        if "resetQueued = true" not in stop_block:
            audit.passed("MCU STOP semantics", "STOP does not queue automatic return", cpp_path)
        else:
            audit.fail("MCU STOP semantics", "STOP appears to queue reset/return", cpp_path)

        if "controlledStopMotors" in cpp and "SAFETY_STOP_DECEL" in cpp:
            audit.passed("MCU controlled braking", "Uses a controlled safety deceleration path", cpp_path)
        else:
            audit.fail("MCU controlled braking", "Controlled safety deceleration was not detected", cpp_path)

        brake_block = get_block(cpp, "void activeBrakeStopMotors", "void startStraight")
        if re.search(r"\bmove\s*\(\s*-", brake_block):
            audit.warn("MCU reverse brake pulse", "A reverse move was detected inside the safety-brake function", cpp_path)
        else:
            audit.passed("MCU reverse brake pulse", "No reverse braking pulse was detected", cpp_path)

        expected_pattern = "{DEFAULT_VIEW, SCAN_RIGHT, DEFAULT_VIEW, SCAN_LEFT}"
        if expected_pattern in cpp:
            audit.passed("MCU servo pattern", "Uses center-right-center-left scanning", cpp_path)
        else:
            audit.warn("MCU servo pattern", "Expected center-right-center-left pattern was not found", cpp_path)

        if "queueSMSAlert" in cpp:
            audit.passed("MCU SMS isolation", "Operational SMS alerts use a queued worker interface", cpp_path)
        else:
            audit.warn("MCU SMS isolation", "Queued SMS worker interface was not detected", cpp_path)

    if h_path is not None:
        path_timeout = parse_number(header, "PATH_COMMAND_TIMEOUT_MS")
        if path_timeout is not None and path_timeout <= 2000:
            audit.passed("MCU path watchdog", f"Independent path timeout is {path_timeout:g} ms", h_path)
        else:
            audit.fail("MCU path watchdog", "Independent path timeout is missing or too slow", h_path)

        go_count = parse_number(header, "MCU_GO_CONFIRM_PACKETS")
        if go_count is not None and go_count >= 2:
            audit.passed("MCU GO confirmation", f"Requires {int(go_count)} GO packets", h_path)
        else:
            audit.fail("MCU GO confirmation", "Repeated GO confirmation was not detected", h_path)

        ultrasonic_timeout = parse_number(header, "ULTRASONIC_TIMEOUT_US")
        if ultrasonic_timeout is not None and ultrasonic_timeout <= 20000:
            audit.passed("MCU sonar timeout", f"Echo timeout is bounded at {ultrasonic_timeout:g} us", h_path)
        else:
            audit.fail("MCU sonar timeout", "Echo timeout is missing or exceeds 20000 us", h_path)

        side_nudge = parse_number(header, "ENABLE_ULTRASONIC_SIDE_NUDGE")
        if side_nudge == 0:
            audit.passed("MCU steering authority", "Ultrasonic side steering is disabled", h_path)
        else:
            audit.warn("MCU steering authority", "Ultrasonic side steering is enabled or undefined", h_path)

        scan_left = parse_number(header, "SCAN_LEFT")
        scan_right = parse_number(header, "SCAN_RIGHT")
        if scan_left is not None and scan_right is not None and scan_left <= 150 and scan_right >= 20:
            audit.passed("MCU servo endpoints", f"Servo endpoints are bounded at {scan_right:g} and {scan_left:g} degrees", h_path)
        else:
            audit.warn("MCU servo endpoints", "Servo endpoints may approach mechanical limits", h_path)

    if ino_path is not None:
        if "ESP_Serial.begin(115200" in ino:
            audit.passed("MCU UART baud", "Controller UART matches 115200 baud", ino_path)
        else:
            audit.fail("MCU UART baud", "Controller UART is not configured at 115200 baud", ino_path)

        if "runStart();" in ino and "outboundComplete" in ino:
            audit.passed("MCU outbound execution", "Outbound route is guarded against repeated execution", ino_path)
        else:
            audit.warn("MCU outbound execution", "One-shot outbound execution guard was not confirmed", ino_path)

    if points_path is not None:
        points = read_text(points_path)
        run_match = re.search(r"runStart\(\).*?safeMoveDistance\(([-0-9]+)", points, re.DOTALL)
        return_match = re.search(r"returnToPointB\(\).*?safeMoveDistance\(([-0-9]+)", points, re.DOTALL)
        if run_match and return_match:
            run_steps = int(run_match.group(1))
            return_steps = int(return_match.group(1))
            if run_steps > 0 and return_steps > 0:
                audit.warn(
                    "route direction",
                    "Outbound and return both use positive motion; verify physical direction with wheels lifted",
                    points_path,
                )
            else:
                audit.passed("route direction", "Outbound and return signs differ or include explicit reverse", points_path)
        else:
            audit.warn("route direction", "Could not extract outbound and return step values", points_path)


def audit_architecture(audit: Audit, path: Optional[Path]) -> None:
    if path is None:
        return
    text = read_text(path)
    required_topics = {
        "three-node ownership": ["Raspberry Pi", "BLE bridge", "Main-controller"],
        "sequenced protocol": ["P:<seq>", "S:<seq>"],
        "safety invariants": ["missing", "stale", "STOP"],
        "watchdog table": ["0.8", "1200", "1500"],
        "servo state machine": ["command-settle-ping", "12000"],
        "controlled braking": ["controlled deceleration", "reverse pulse"],
        "route limitation": ["pointsRun.ino", "wheels lifted"],
    }
    for name, terms in required_topics.items():
        if all(term.lower() in text.lower() for term in terms):
            audit.passed("architecture coverage", f"Documents {name}", path)
        else:
            audit.warn("architecture coverage", f"Architecture may not fully document {name}", path)


def main() -> int:
    parser = argparse.ArgumentParser(description="Audit GARBY project safety invariants")
    parser.add_argument("project_root", type=Path, help="Root directory of the GARBY project")
    parser.add_argument("--json", action="store_true", help="Emit machine-readable JSON")
    args = parser.parse_args()

    if not args.project_root.exists() or not args.project_root.is_dir():
        print(f"error: project root is not a directory: {args.project_root}", file=sys.stderr)
        return 2

    audit = Audit(args.project_root)
    files = check_required_files(audit)
    audit_pi(audit, files["pi"])
    audit_bridge(audit, files["bridge"])
    audit_mcu(audit, files["mcu_cpp"], files["mcu_h"], files["mcu_ino"], files["points"])
    audit_architecture(audit, files["architecture"])

    counts = {
        severity: sum(result.severity == severity for result in audit.results)
        for severity in ("PASS", "WARN", "FAIL")
    }

    if args.json:
        print(json.dumps({
            "root": str(audit.root),
            "summary": counts,
            "results": [asdict(result) for result in audit.results],
        }, indent=2))
    else:
        for result in audit.results:
            location = f" [{result.file}]" if result.file else ""
            print(f"{result.severity:4} {result.check}: {result.detail}{location}")
        print()
        print(f"Summary: {counts['PASS']} passed, {counts['WARN']} warnings, {counts['FAIL']} failed")
        print("This static audit does not replace compilation or physical safety testing.")

    return 1 if counts["FAIL"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
