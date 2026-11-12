"""Read-only Pi health snapshots for SMS; no network or device I/O at import."""
import math
import threading
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


def probe_internet(*, opener=None):
    """Verify HTTPS reachability independently of Firebase and captive portals.

    Each response is bounded. DNS/platform stalls remain in the dedicated daemon
    worker; a stale snapshot becomes unknown rather than refreshing safety data.
    """
    opener = urlopen if opener is None else opener
    targets = ("https://connectivitycheck.gstatic.com/generate_204",
               "https://www.cloudflare.com/cdn-cgi/trace")
    for index, target in enumerate(targets):
        try:
            request = Request(target, headers={"User-Agent": "GARBY-health/1"})
            with opener(request, timeout=2.0) as response:
                if response.geturl() != target:
                    continue
                body = response.read(1025)
                if index == 0 and response.status == 204 and not body:
                    return True
                if index == 1 and response.status == 200 and len(body) <= 1024:
                    lines = body.decode("ascii", errors="replace").splitlines()
                    if any(line.startswith("ip=") for line in lines) and any(
                            line.startswith("colo=") for line in lines):
                        return True
        except (OSError, HTTPError, URLError, ValueError):
            continue
    return False


class HealthReporter:
    STALE_S = 15.0
    CLOUD_STARTUP_GRACE_S = 60.0

    def __init__(self, *, cloud_expected, started_at=None):
        self._lock = threading.Lock()
        self._cloud_expected = cloud_expected
        self._started = time.monotonic() if started_at is None else started_at
        self._observed_at = None
        self._packet = "[HEALTH:1:XXXXX]"
        self._temperature_alarm = self._power_alarm = False

    @staticmethod
    def _state(value):
        return "U" if value is True else "D" if value is False else "X"

    def observe(self, internet, *, cloud, lidar, sensors, cpu_temperature_c=None,
                throttled_flags=None, now=None):
        now = time.monotonic() if now is None else now
        with self._lock:
            internet_state = self._state(internet)
            # A recent successful cloud operation is independent evidence when
            # a network filters the two public probe hosts. An old cloud badge
            # must not hide a real new outage.
            success_at = cloud.get("last_success_at") if cloud else None
            if (internet_state == "D" and cloud and cloud.get("online") is True
                    and isinstance(success_at, (int, float))
                    and 0 <= now - success_at <= self.STALE_S):
                internet_state = "U"
            cloud_state = "X"
            if self._cloud_expected and internet_state != "D":
                if cloud and cloud.get("online") is True:
                    cloud_state = "U"
                elif cloud and cloud.get("state") in ("Offline", "Stale"):
                    cloud_state = "D"
                elif now - self._started >= self.CLOUD_STARTUP_GRACE_S:
                    cloud_state = "D"
            try:
                temperature = float(cpu_temperature_c)
                temperature_valid = math.isfinite(temperature)
            except (TypeError, ValueError, OverflowError):
                temperature, temperature_valid = 0, False
            flags_valid = isinstance(throttled_flags, int) and throttled_flags >= 0
            active_flags = bool(throttled_flags & 0xF) if flags_valid else False
            if temperature_valid and temperature >= 80:
                self._temperature_alarm = True
            elif temperature_valid and temperature <= 75:
                self._temperature_alarm = False
            if flags_valid:
                self._power_alarm = active_flags
            observed_alarm = ((self._temperature_alarm and temperature_valid)
                              or (self._power_alarm and flags_valid))
            unresolved_alarm = ((self._temperature_alarm and not temperature_valid)
                                or (self._power_alarm and not flags_valid))
            thermal = ("D" if observed_alarm else "X" if unresolved_alarm else
                       "U" if temperature_valid or flags_valid else "X")
            self._packet = (f"[HEALTH:1:{internet_state}{cloud_state}{self._state(lidar)}"
                            f"{self._state(sensors)}{thermal}]")
            self._observed_at = now

    def packet(self, now=None):
        now = time.monotonic() if now is None else now
        with self._lock:
            if self._observed_at is None or not 0 <= now - self._observed_at <= self.STALE_S:
                return "[HEALTH:1:XXXXX]"
            return self._packet

    def invalidate(self):
        with self._lock:
            self._observed_at = None
