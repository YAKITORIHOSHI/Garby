#!/usr/bin/env python3
"""Lightweight host-side sanity checks for the GARBY release.

Checks structural delimiter/preprocessor balance in Arduino/C++ source and JSON
parseability. This is deliberately not presented as an Arduino compiler.
"""
from __future__ import annotations
import json
import sys
from pathlib import Path

ROOT = Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
CPP_EXT = {".ino", ".cpp", ".h", ".hpp", ".c"}


def strip_cpp_noise(text: str) -> str:
    out = []
    i = 0
    state = "code"
    while i < len(text):
        c = text[i]
        n = text[i + 1] if i + 1 < len(text) else ""
        if state == "code":
            if c == "/" and n == "/":
                state = "line"; out.extend("  "); i += 2; continue
            if c == "/" and n == "*":
                state = "block"; out.extend("  "); i += 2; continue
            if c == '"':
                state = "string"; out.append(" "); i += 1; continue
            if c == "'":
                state = "char"; out.append(" "); i += 1; continue
            out.append(c); i += 1; continue
        if state == "line":
            if c == "\n": state = "code"; out.append("\n")
            else: out.append(" ")
            i += 1; continue
        if state == "block":
            if c == "*" and n == "/":
                state = "code"; out.extend("  "); i += 2
            else:
                out.append("\n" if c == "\n" else " "); i += 1
            continue
        if state in ("string", "char"):
            quote = '"' if state == "string" else "'"
            if c == "\\" and i + 1 < len(text):
                out.extend("  "); i += 2; continue
            if c == quote:
                state = "code"; out.append(" "); i += 1; continue
            out.append("\n" if c == "\n" else " "); i += 1
    return "".join(out)


def check_delimiters(path: Path) -> list[str]:
    text = strip_cpp_noise(path.read_text(encoding="utf-8", errors="replace"))
    pairs = {")": "(", "]": "[", "}": "{"}
    opens = set(pairs.values())
    stack: list[tuple[str, int]] = []
    line = 1
    errors = []
    for c in text:
        if c == "\n": line += 1; continue
        if c in opens: stack.append((c, line))
        elif c in pairs:
            if not stack or stack[-1][0] != pairs[c]:
                errors.append(f"line {line}: unmatched {c}")
            else: stack.pop()
    for c, ln in stack[-10:]:
        errors.append(f"line {ln}: unclosed {c}")

    pp_stack = []
    for ln, raw in enumerate(text.splitlines(), 1):
        stripped = raw.lstrip()
        if stripped.startswith(("#if ", "#ifdef ", "#ifndef ")):
            pp_stack.append(ln)
        elif stripped.startswith("#endif"):
            if pp_stack: pp_stack.pop()
            else: errors.append(f"line {ln}: #endif without #if")
    for ln in pp_stack:
        errors.append(f"line {ln}: preprocessor conditional not closed")
    return errors


def main() -> int:
    failures = []
    cpp_files = sorted(p for p in ROOT.rglob("*") if p.is_file() and p.suffix.lower() in CPP_EXT)
    json_files = sorted(ROOT.rglob("*.json"))

    for path in cpp_files:
        errs = check_delimiters(path)
        if errs:
            failures.append((path, "; ".join(errs)))
        else:
            print(f"PASS C++ structure  {path.relative_to(ROOT)}")

    for path in json_files:
        try:
            json.loads(path.read_text(encoding="utf-8"))
            print(f"PASS JSON parse     {path.relative_to(ROOT)}")
        except Exception as exc:
            failures.append((path, f"JSON parse error: {exc}"))

    required = [
        ROOT / "RasPi/bridge_core.py",
        ROOT / "RasPi/final_w_serial.py",
        ROOT / "BLE_Receiver-Final/BLE_Receiver-Final.ino",
        ROOT / "NAPHTALI_CODE_V2/NAPHTALI_CODE_V2.ino",
        ROOT / "NAPHTALI_CODE_V2/NAPHTALI_CODE_V2.cpp",
        ROOT / "NAPHTALI_CODE_V2/NAPHTALI_CODE_V2.h",
        ROOT / "NAPHTALI_CODE_V2/pointsRun.ino",
    ]
    for path in required:
        if path.is_file(): print(f"PASS required file  {path.relative_to(ROOT)}")
        else: failures.append((path, "missing required file"))

    if failures:
        print("\nFAILURES:")
        for path, detail in failures:
            try: label = path.relative_to(ROOT)
            except ValueError: label = path
            print(f"FAIL {label}: {detail}")
        print(f"\nSummary: FAIL ({len(failures)} issue(s))")
        return 1
    print(f"\nSummary: PASS ({len(cpp_files)} C/C++ files, {len(json_files)} JSON files)")
    print("Structural host check only; target Arduino compilation is still required.")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
