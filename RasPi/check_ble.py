#!/usr/bin/env python3
"""Diagnose GARBY BLE without ROS/Firebase. --connect writes readiness only, never GO."""
import argparse
import asyncio
import os
import shutil
import subprocess
import sys

from ble_support import (
    SERVICE_UUID, WRITE_CHAR_UUID, NOTIFY_CHAR_UUID, DEVICE_NAME,
    configured_uuid, adapter_options, discover_robot, robot_characteristics,
    subscribe, NotificationDecoder, DiscoveryDiagnostics,
)
from runtime_support import SingleInstanceGuard


def adapter_diagnostics():
    if sys.platform != "linux":
        return
    adapter = os.environ.get("GARBY_BLE_ADAPTER", "hci0").strip() or "hci0"
    for command in (
        ["systemctl", "is-active", "bluetooth"],
        ["rfkill", "list", "bluetooth"],
        ["busctl", "get-property", "org.bluez", f"/org/bluez/{adapter}",
         "org.bluez.Adapter1", "Powered"],
    ):
        if shutil.which(command[0]):
            try:
                result = subprocess.run(command, capture_output=True, text=True, timeout=5)
                print("ADAPTER " + " ".join(command) + ": " +
                      (result.stdout + result.stderr).strip())
            except (OSError, subprocess.TimeoutExpired) as error:
                print(f"ADAPTER diagnostic unavailable: {error}")


async def probe(connect=False, timeout=12.0):
    from bleak import BleakClient, BleakScanner
    service_uuid = configured_uuid("GARBY_BLE_SERVICE_UUID", SERVICE_UUID)
    write_uuid = configured_uuid("GARBY_BLE_WRITE_CHAR_UUID", WRITE_CHAR_UUID)
    notify_uuid = configured_uuid("GARBY_BLE_NOTIFY_CHAR_UUID", NOTIFY_CHAR_UUID)
    print("DISCOVERY scanning for GARBY service/name (or configured address)...", flush=True)
    discovery = DiscoveryDiagnostics()
    target = await asyncio.wait_for(discover_robot(
        BleakScanner, address=os.environ.get("GARBY_BLE_ADDRESS", "").strip() or None,
        name=os.environ.get("GARBY_BLE_DEVICE_NAME", DEVICE_NAME).strip() or DEVICE_NAME,
        service_uuid=service_uuid, timeout=timeout, diagnostics=discovery), timeout=timeout + 5.0)
    if target is None:
        raise RuntimeError(f"DISCOVERY FAIL: {discovery.failure_message()}; "
                           "check selection, adapter power/rfkill, ESP32 firmware, and other clients")
    print(f"DISCOVERY PASS: {target.name or '(name absent)'} [{target.address}]")
    if not connect:
        return
    client = BleakClient(target, timeout=30.0, **adapter_options())
    phase = "CONNECTION/GATT"
    try:
        await asyncio.wait_for(client.connect(), timeout=30.0)
        if not client.is_connected:
            raise RuntimeError("Disconnected during connection")
        write, notify = robot_characteristics(client, service_uuid, write_uuid, notify_uuid)
        print("CONNECTION/GATT PASS: expected service, write, notify")
        phase = "NOTIFY"
        decoder = NotificationDecoder()
        received = asyncio.Event()
        def notification(sender, data):
            for line in decoder.feed(data):
                print("RX " + line, flush=True)
                received.set()
        await asyncio.wait_for(subscribe(client, notify, notification), timeout=8.0)
        print("NOTIFY PASS: subscription accepted")
        phase = "READINESS WRITE"
        await asyncio.wait_for(client.write_gatt_char(write, b"[RASPI READY]", response=True), timeout=2.0)
        print("READINESS WRITE PASS: acknowledged; no GO sent")
        phase = "NOTIFICATION RECEIVE"
        await asyncio.wait_for(received.wait(), timeout=5.0)
        print("BLE ROUND TRIP PASS. LiDAR, UART, motors, and route remain separate acceptance items.")
    except Exception as error:
        raise RuntimeError(f"{phase} FAIL: {str(error) or type(error).__name__}") from error
    finally:
        try:
            await asyncio.wait_for(client.disconnect(), timeout=2.0)
        except Exception:
            pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connect", action="store_true", help="Connect/subscribe/write readiness; stop the production bridge first and lift wheels")
    parser.add_argument("--timeout", type=float, default=12.0)
    args = parser.parse_args()
    if not 1.0 <= args.timeout <= 60.0:
        parser.error("timeout must be between 1 and 60 seconds")
    adapter_diagnostics()
    try:
        if args.connect:
            with SingleInstanceGuard():
                asyncio.run(probe(True, args.timeout))
        else:
            asyncio.run(probe(False, args.timeout))
    except ImportError as error:
        raise SystemExit(f"FAIL: install RasPi/requirements.txt in this Python environment ({error})")
    except Exception as error:
        raise SystemExit(str(error))


if __name__ == "__main__":
    main()
