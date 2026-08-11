#!/usr/bin/env python3
"""Fast source-only release invariants for the GARBY Android client.

This is intentionally independent of Gradle/Android SDK so high-value security
and control-path regressions can be caught even on a minimal host.
"""
from __future__ import annotations

import re
import sys
import tomllib
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "app/src/main"
JAVA = SRC / "java"
MANIFEST = SRC / "AndroidManifest.xml"
BUILD = ROOT / "app/build.gradle.kts"
VERSIONS = ROOT / "gradle/libs.versions.toml"

failures: list[str] = []


def fail(message: str) -> None:
    failures.append(message)


# Parse configuration/resources.
with VERSIONS.open("rb") as handle:
    tomllib.load(handle)
for path in [MANIFEST, *(SRC / "res").rglob("*.xml")]:
    try:
        ET.parse(path)
    except Exception as exc:  # pragma: no cover - host validation path
        fail(f"XML parse failed: {path.relative_to(ROOT)}: {exc}")

manifest_text = MANIFEST.read_text(encoding="utf-8")
if 'android:allowBackup="false"' not in manifest_text:
    fail("Application backup must remain disabled")
if 'android:usesCleartextTraffic="false"' not in manifest_text:
    fail("Cleartext traffic must remain disabled")
if "CHANGE_NETWORK_STATE" in manifest_text:
    fail("CHANGE_NETWORK_STATE should not be requested")

source_files = list(JAVA.rglob("*.kt"))
source_text = "\n".join(path.read_text(encoding="utf-8") for path in source_files)
build_text = BUILD.read_text(encoding="utf-8")

for label, pattern in {
    "trust-all TLS": r"NoopHostnameVerifier|X509TrustManager|TrustAll",
    "embedded credential constants": r"const\s+val\s+(?:PASSWORD|USERNAME|EMAIL)\s*=",
    "manual RTDB disk persistence": r"setPersistenceEnabled\s*\(",
    "legacy direct HTTP stack": r"org\.apache\.hc|SensorAPIHelper|BasicCredentialsProvider",
    "unbounded/dead custom executor": r"LinkedBlockingQueue|AdaptiveThreadPool|ExecutorServices",
    "FCM registration token value logging": r"Log\.[A-Za-z]+\([^\n]*(?:\$token|\+\s*token)",
}.items():
    if re.search(pattern, source_text, re.IGNORECASE):
        fail(f"Forbidden pattern detected: {label}")

if re.search(r"(?:reading\?\.value|batteryPercent)\s*\?:\s*0", source_text):
    fail("Unavailable telemetry must not silently default to zero")

for required in [
    "ServerValue.TIMESTAMP",
    "purgeOutstandingWrites()",
    "waitForFirebaseConnection()",
    "requestedBy = uid",
    "else -> Unknown",
]:
    if required not in source_text:
        fail(f"Reset safety invariant missing: {required}")

for forbidden_dependency in [
    "httpclient",
    "gson",
    "kotlinx.serialization",
    "firebase.analytics",
    "firebase.crashlytics",
    "datastore",
    "constraintlayout",
    "androidx.window",
]:
    if forbidden_dependency in build_text:
        fail(f"Unused/legacy dependency returned: {forbidden_dependency}")

# Local resource references must resolve by resource stem.
resource_stems: dict[str, set[str]] = {}
for resource_type in ("drawable", "font"):
    folder = SRC / "res" / resource_type
    resource_stems[resource_type] = {p.stem for p in folder.iterdir() if p.is_file()}
for resource_type, name in re.findall(r"R\.(drawable|font)\.([A-Za-z0-9_]+)", source_text):
    if name not in resource_stems[resource_type]:
        fail(f"Missing R.{resource_type}.{name}")

if len(resource_stems["font"]) > 3:
    fail("Unexpected Montserrat font payload growth")

if failures:
    print("STATIC_QUALITY_CHECK: FAIL")
    for item in failures:
        print(f" - {item}")
    sys.exit(1)

print("STATIC_QUALITY_CHECK: PASS")
print(f"Kotlin files checked: {len(source_files)}")
print(f"Bundled font files: {len(resource_stems['font'])}")
