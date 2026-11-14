# Robot verification evidence — 2026-11-14

The retained suite passed eight firmware host programs and 596 assertions. Both actual
ESP32-S3 bridge and ESP32 executor targets compiled successfully, build RECONNECT-20261114.
See `firmware-host-final.log`, both target logs, `source-after.json`, `binary-artifacts.json`
and `verification-summary.json`. Original failing reproductions and the review patch are retained.

The Windows Pi suite completed 464 tests with 3 failures and 6 errors; see `pi-final.log`.
These results do not establish a green Linux target run or physical robot acceptance.
No upload, live robot command, database/rules write, SMS or service restart was performed.

Android source, APKs and Android check results belong to the separate app repository.
Current robot packages contain no embedded app. Production runtime source was unchanged
by this ownership correction, so MCU builds were not repeated.
