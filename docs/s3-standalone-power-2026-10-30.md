# S3 RGB LED and standalone-power investigation — 2026-10-30

Subsequent owner update: standalone operation works again. No electrical
measurement or confirmed root cause was supplied; the following remains the
dated investigation record. Work then moved to MPU/LiDAR steering stability.

## Reported configuration and symptom

The owner reports that the ESP32-S3 works when connected to a PC/laptop, but
does not work as intended on its standalone supply; its NeoPixel unexpectedly
lights. The reported power path is:

```text
52 V battery -> buck converter advertised 5 V / 10 A
            -> ESP32 5 V connection -> ESP32-S3 5Vin
```

The owner confirms that converter output negative, ESP32 GND and S3 GND are
wired together during standalone operation. Measured continuity and ground
voltage drop remain unknown. The converter model/input range, maximum charged
battery voltage, physical board models and voltage at the S3 are unverified.
The 10 A rating provides ample nominal capacity for an S3, but it is not a
measurement of voltage at the end of this path. The intermediate board's 5 V
pin, connectors, wires and ground return need checking. Daisy-chaining can work
on a suitable shared 5 V rail; this arrangement alone does not prove a defect.

## Verified software findings

The two production bridge sources contain no NeoPixel/RGB control or wait for
a USB host/Serial Monitor. Setup uses fixed delays, bounded MPU initialization
and calibration, then starts NimBLE advertising. MPU failure keeps heading
readiness invalid but still reaches BLE initialization. The application does
not disable brownout protection, the matching ELF includes brownout
initialization, and startup logs include the reset reason.

Both source hashes and all five recorded binary hashes match the successful
dated build in `artifacts/sms-notices-2026-10-30/BLE_Receiver-Final/`. That build
uses Arduino ESP32 core 3.3.11, USB CDC on boot disabled and UART0 debug Serial;
its ELF has `Serial0` and no matched HWCDC/USBCDC/USBSerial symbols. This does
not establish what is currently flashed on the physical board. The firmware ID
alone is also insufficient to distinguish all revisions built on this date.

The lit LED is therefore not an intentional GARBY status indication in the
audited application. It does not prove the CPU booted or BLE is running. The
exact physical board matters: Espressif's DevKitC-1 initial revision uses RGB
GPIO48, while v1.1 uses GPIO38. Other boards can differ. See the
[official revision details](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html#hardware-revision-details).

## Physical checks that distinguish the causes

These are proposed bench checks, not actions performed by this investigation.
Disconnect motor power and secure the robot. Change wiring only while unpowered,
keep the MPU level and still at boot, and observe BLE advertising rather than
starting a route. Use one documented board power input at a time.

1. Verify the converter's input range against the battery's maximum charged
   voltage and measure its output. A nominal 52 V battery label is insufficient
   to establish converter input compatibility. Confirm the S3's documented 5 V
   input; never apply this 5 V supply to its 3V3 pin.
2. The shared ground is confirmed by the owner's wiring description. Check
   the actual connections/continuity while unpowered and ground voltage drop
   under load. A PC connection may change both power and ground conditions.
3. Once the output is verified, test separate short power branches from the buck
   output to each board's documented 5 V input and GND. This bypasses the
   intermediate ESP32 power connection without changing firmware:

   ```text
   Buck +5 V --+--> ESP32 documented 5 V input
               +--> S3 documented 5 V input
   Buck GND ---+--> ESP32 GND
               +--> S3 GND
   ```

   If the S3 now boots and advertises reliably, investigate the bypassed wiring,
   connectors and first board's power path. The result would not identify which
   component caused a drop without measurements.
4. Compare S3 5 V, regulated 3.3 V and EN during cold startup, using the original
   path and the direct branch. A meter can find sustained drops; a scope is
   needed to characterize short dips/ripple and reset timing. Espressif requires
   the 3.3 V rail to stabilize before EN/CHIP_PU rises; see its
   [power-up/reset guidance](https://docs.espressif.com/projects/esp-hardware-design-guidelines/en/latest/esp32s3/schematic-checklist.html#chip-power-up-and-reset-timing).
5. A standalone USB charger/powerbank using the same S3 USB port/cable as the PC
   is another useful comparison. Successful standalone USB operation establishes
   that a PC host is unnecessary in that setup. Failure alone does not establish
   a USB software dependency. With the robot secured, a reset after the supply
   stabilizes can also distinguish cold-start behavior from continuing operation;
   it does not by itself identify an EN circuit fault.
6. Correlate the actual S3 boot/reset log with Pi BLE timestamps. The matching
   compiled artifact logs through UART0; native USB need not expose these logs.
   Use the exact board's documented debug interface, avoiding a second power
   source or a diagnostic adapter that changes reset/boot conditions. USB-UART
   DTR/RTS can affect GPIO0/EN; see Espressif's
   [boot-mode explanation](https://docs.espressif.com/projects/esptool/en/latest/esp32s3/advanced-topics/boot-mode-selection.html#automatic-bootloader).

Do not select RGB GPIOs, alter EN capacitors, disable brownout detection or relax
motion gates based only on this symptom. The board models and measured results
determine the appropriate repair. For the official DevKitC-1, USB power, 5 V/GND
header power and 3V3/GND header power are alternative supply methods; see its
[power options](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/user_guide_v1.1.html#power-supply-options).

## Evidence and acceptance boundary

The [source/build audit](../artifacts/s3-standalone-power-2026-10-30/source-build-audit.md)
records exact commands, hashes, USB options and ELF evidence. The associated
`audit_bridge.py` reproduces these artifact checks. This investigation required
no new compile or firmware change and performed no upload, hardware reset,
wiring change, SMS test or motion command. The electrical cause and installed
firmware remain unverified. Earlier Pi observations are recorded in the
[BLE investigation](ble-discovery-2026-10-30.md).
