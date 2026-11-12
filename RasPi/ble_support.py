"""Shared GARBY BLE discovery/GATT helpers; no ROS or Firebase dependency."""
import os
import sys
from uuid import UUID

SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
WRITE_CHAR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a8"
NOTIFY_CHAR_UUID = "beb5483e-36e1-4688-b7f5-ea07361b26a9"
DEVICE_NAME = "GarbyESP32"


def configured_uuid(key, default):
    return str(UUID(os.environ.get(key, default).strip()))


def adapter_options():
    adapter = os.environ.get("GARBY_BLE_ADAPTER", "").strip()
    # The legacy keyword supports Bleak 0.20 through 3.x; BlueZ-specific
    # arguments support newer releases. Omit both outside Linux.
    if sys.platform == "linux" and adapter:
        return {"adapter": adapter, "bluez": {"adapter": adapter}}
    return {}


def matches_advertisement(device, advertisement, *, address, name, service_uuid):
    if address:
        return device.address.casefold() == address.casefold()
    services = {str(value).replace("-", "").casefold()
                for value in (advertisement.service_uuids or [])}
    if service_uuid.replace("-", "").casefold() in services:
        return True
    return any(value and value.strip().casefold() == name.casefold()
               for value in (device.name, advertisement.local_name))


async def discover_robot(scanner, *, address=None, name=DEVICE_NAME,
                         service_uuid=SERVICE_UUID, timeout=12.0, diagnostics=None):
    # Return the scanned BLEDevice. Passing an address to BleakClient makes it
    # scan again implicitly, which wasted most of the old 5/6-second deadline.
    options = adapter_options()
    if diagnostics is not None:
        diagnostics.reset(adapter=options.get("adapter"), address=address)

    def matches(device, advertisement):
        matched = matches_advertisement(
            device, advertisement, address=address, name=name, service_uuid=service_uuid)
        if diagnostics is not None:
            diagnostics.observe(device, advertisement, matched=matched,
                                address=address, name=name, service_uuid=service_uuid)
        return matched

    return await scanner.find_device_by_filter(
        matches, timeout=timeout, scanning_mode="active", **options)


class DiscoveryDiagnostics:
    """Evidence from the existing scan; no unrelated device identities retained.

    Counts advertisement callbacks, not unique devices or raw radio packets.
    Store at most three bounded GARBY candidate addresses rejected by a pin.
    """
    MAX_EVENTS = 2**31 - 1
    MAX_CANDIDATES = 3
    MAX_FIELD_LENGTH = 64

    def __init__(self):
        self.reset()

    @classmethod
    def _field(cls, value):
        return "".join(char if char.isprintable() else "?"
                       for char in str(value)[:cls.MAX_FIELD_LENGTH])

    def reset(self, *, adapter=None, address=None):
        self.event_count = 0
        self.adapter = self._field(adapter) if adapter else "platform default"
        self.pinned_address = self._field(address) if address else "unset"
        self.rejected_addresses = []

    def observe(self, device, advertisement, *, matched, address, name, service_uuid):
        self.event_count = min(self.event_count + 1, self.MAX_EVENTS)
        if (address and not matched and len(self.rejected_addresses) < self.MAX_CANDIDATES
                and matches_advertisement(device, advertisement, address=None,
                                          name=name, service_uuid=service_uuid)):
            candidate = self._field(device.address)
            if candidate.casefold() not in (value.casefold() for value in self.rejected_addresses):
                self.rejected_addresses.append(candidate)

    def failure_message(self):
        if not self.event_count:
            evidence = "no advertisement events received during scan"
        elif self.rejected_addresses:
            evidence = "GARBY-identifying advertisements rejected by pinned address"
        else:
            evidence = "advertisement events received, none matched configured GARBY identity"
        details = (f"events={self.event_count}; adapter={self.adapter}; "
                   f"pinned_address={self.pinned_address}")
        if self.rejected_addresses:
            details += "; rejected_candidates=" + ",".join(self.rejected_addresses)
        return f"{evidence}; {details}"


def robot_characteristics(client, service_uuid, write_uuid, notify_uuid):
    for service in client.services:
        if service.uuid.casefold() != service_uuid.casefold():
            continue
        write = next((c for c in service.characteristics
                      if c.uuid.casefold() == write_uuid.casefold()), None)
        notify = next((c for c in service.characteristics
                       if c.uuid.casefold() == notify_uuid.casefold()), None)
        if write is None or notify is None:
            break
        if "write" not in write.properties or "notify" not in notify.properties:
            raise RuntimeError("GARBY GATT properties do not support write/notify; check flashed firmware")
        return write, notify
    raise RuntimeError("GARBY service/write/notify UUIDs are missing; check firmware and UUID overrides")


async def subscribe(client, characteristic, callback):
    # Bleak 3.0.0/3.0.1 changed Linux to AcquireNotify, which fails with some
    # peripherals. Explicit StartNotify works with both older and newer Bleak.
    await client.start_notify(characteristic, callback,
                              bluez={"use_start_notify": True})


class NotificationDecoder:
    """Bounded reassembly: '~' starts a line, '+' continues it, newline ends it.

    Unframed notifications from older firmware remain readable.
    """
    def __init__(self):
        self.pending = ""

    def reset(self):
        self.pending = ""

    def feed(self, data):
        try:
            message = bytes(data).decode("ascii")
        except (UnicodeError, TypeError, ValueError):
            self.reset()
            return []
        if message.startswith("~"):
            self.reset()
        elif message.startswith("+"):
            if not self.pending:
                return []
        else:
            self.reset()
            return [line.strip() for line in message.splitlines() if line.strip()]
        self.pending += message[1:]
        if len(self.pending) > 256:
            self.reset()
            return []
        parts = self.pending.split("\n")
        self.pending = parts.pop()
        return [line.strip() for line in parts if line.strip()]
