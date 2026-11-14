# GARBY — autonomous waste-collection robot

GARBY is an indoor waste-collection robot built for a thesis project. It carries a trash
bin along a calibrated route through a school hallway to a collection point, waits there,
and returns to its base on request. On the way it keeps to the centre of the hallway,
stops for people and obstacles, and resumes on its own when the path is clear.

## What it does

- **Starts a run by itself** when the bin is heavy enough (load cell at 4 kg or more, confirmed
  twice) or the gas sensors read dangerous levels (confirmed three times), or when an operator
  confirms **Manual disposal** on the monitor or Android app, even before the bin,
  weight or gas thresholds are reached.
- **Follows a calibrated route.** Turns are measured with an MPU6050 gyro and checked against the
  hallway walls seen by the LiDAR. Long straights steer to the centre of the hallway, and before a
  long run the robot stands still and centres itself between the walls first.
- **Stops for anything in its way** and waits instead of trying to steer around it: the LiDAR
  watches its path and predicts people or objects moving toward it, and a low ultrasonic sensor
  on a sweeping servo watches for chairs, bins and other low objects. While held it beeps, drops
  to a low-power mode, and continues automatically once the path has been clear for a moment.
- **Lets people know it is coming** with a short beep while driving.
- **Reports its state** on the Raspberry Pi monitor, to an Android app through Firebase, and by
  SMS for important events (blocked for a long time, route fault, emergency stop, outages).
- **Fails safe.** Missing or stale path/heading data stops the wheels. App requests never
  bypass the robot's motion safety checks.

## How it is built

| Part | Hardware | Source | Role |
| --- | --- | --- | --- |
| Supervisor | Raspberry Pi 5, YDLIDAR X3, ROS 2 | `RasPi/` | Reads the LiDAR, decides whether the path is clear, fits the hallway walls, predicts moving objects, runs the monitor, talks to the bridge over Bluetooth LE, publishes to Firebase |
| Bridge | ESP32-S3 with an MPU6050 | `BLE_Receiver-Final/` | Receives path decisions over BLE, streams the robot's heading, forwards wall measurements, relays commands to the executor over UART |
| Executor | ESP32, two stepper drivers, sonar on a servo, buzzer, HX711 load cell, Air780E modem | `MAIN_EXECUTOR_CODE/` | Drives the routes and turns, steers, enforces every stop, starts runs, sends SMS |
| App | Android phone | `Garby_MobileApp/app/` | Dashboard, confirmed manual disposal and return-to-base requests |

A separate sensor board (bin fill ultrasonic, MQ4/MQ135/MQ137 gas sensors) connects to the Pi
by serial. Wiring for every pin: [`MAIN_EXECUTOR_CODE/PINOUTS.txt`](MAIN_EXECUTOR_CODE/PINOUTS.txt).

## Repository layout

| Folder | Contents |
| --- | --- |
| `RasPi/` | Pi runtime (`final_w_serial.py` is the entry point), monitor, launch scripts, tests |
| `BLE_Receiver-Final/` | ESP32-S3 bridge sketch |
| `MAIN_EXECUTOR_CODE/` | ESP32 executor sketch; route steps are in `pointsRun.ino`, tunables at the top of the headers |
| `Garby_MobileApp/` | Android app |
| `tools/` | Project verification and host tests of the real firmware source |
| `docs/` | Design and investigation reports |
| `Test/`, `MPU_and_Motor-Latest/` | Bench test sketches, not used on the robot |

## Requirements

- **Pi:** Raspberry Pi OS or Ubuntu with ROS 2 (Jazzy or Humble) and the YDLIDAR ROS 2 driver
  built in `~/ros2_ws`; Python 3 with `RasPi/requirements-firebase.txt` (or the lean
  `RasPi/requirements.txt` without cloud features); `python3-tk` for the monitor window.
- **Firmware:** Arduino IDE or `arduino-cli` with the ESP32 core 3.3, and the libraries
  ESP32Servo, FastAccelStepper, HX711 and NimBLE-Arduino.
- **App:** Android Studio with a JDK (see `Garby_MobileApp/`).
- **Firebase:** a project with a Realtime Database. The Pi needs a Firebase Admin service-account
  key; never commit it.

## Set up

### 1. Flash the two controllers

Flash both sketches from the same version of this repository; the messages between them must
match.

```bash
arduino-cli compile --upload -p <S3 port> --fqbn esp32:esp32:esp32s3:FlashSize=8M,PSRAM=disabled BLE_Receiver-Final
arduino-cli compile --upload -p <ESP32 port> --fqbn esp32:esp32:esp32:FlashSize=4M,PSRAM=disabled MAIN_EXECUTOR_CODE
```

Each board prints its build on its USB console at boot (`[EXECUTOR BUILD:...]`,
`[BRIDGE] Firmware ...`). Keep the robot still and level for the first seconds after power-on
while the gyro calibrates.

### 2. Install the Pi software

Copy the whole `RasPi/` directory to the Pi (it needs all of its helper files), then:

```bash
python3 -m pip install -r RasPi/requirements-firebase.txt
sudo install -m 600 RasPi/garby-bridge.env.example /etc/default/garby-bridge
sudoedit /etc/default/garby-bridge                        # paths and chassis settings
```

Put the Firebase Admin key (a `.json` file) in `RasPi/Firebase/`, or point
`GARBY_FIREBASE_CREDENTIALS` at it. The LiDAR and the sensor board are found automatically by
the data they send. The chassis size and LiDAR mounting are set in the environment file; see
[`RasPi/README.md`](RasPi/README.md) for every option. To start GARBY at boot, use
`RasPi/install_startup.py` or `RasPi/garby-bridge.service.example`.

### 3. Check, then run

```bash
set -a; source /etc/default/garby-bridge; set +a              # load the settings
bash RasPi/run_bridge.sh --check-runtime --require-firebase   # dependencies and devices
bash RasPi/run_bridge.sh --check-ble                          # finds the bridge (no motion)
bash RasPi/run_bridge.sh                                      # run with the monitor window
bash RasPi/run_bridge.sh --headless                           # run without a screen
```

Keep a log of every test run: `bash RasPi/run_bridge.sh 2>&1 | tee ~/garby-$(date +%F-%H%M).log`.
To look at the monitor without any robot hardware: `python3 RasPi/preview_ui.py`.

## Use

- **Monitor.** The *LiDAR readings* tab shows a radar map around the robot, front at the top:
  the distance in each direction, each direction's stop distance, the hallway walls and any
  zone or moving object that is holding the robot. The *BLE/messages* tab shows the link and the
  robot's reports. The *Controls* tab has **Manual disposal** (with confirmation), **Release stop**,
  front/rear LiDAR read-only, obstacle sonar on/off and **Restart Pi + bridge + executor**.
  **EMERGENCY STOP** is always visible.
- **Manual disposal after school.** With GARBY idle at base, press **Manual disposal** on
  the monitor or app, then confirm **Start manual disposal?**. Bin level, gas and weight
  thresholds may be below their automatic trigger values or unavailable. GARBY uses the
  usual outbound route; obstacles, stale path/heading evidence and emergency stop still
  hold the motors. The app shows whether GARBY accepted the request; acceptance is not
  arrival. At the collection point, dispose of the trash and request return as usual.
  App setup and Windows build steps: [`docs/manual-disposal-2026-11-13.md`](docs/manual-disposal-2026-11-13.md).
- **A run.** Once started, GARBY drives the outbound route and beeps every 1.2 s. If something
  blocks it, it stops, chirps twice every 2 s, and continues by itself when the path is clear; a
  long wait switches to low power. At the destination it stays put until a return is requested
  from the Android app.
- **Emergency stop** stops both controllers and stays latched through reconnects and restarts.
  **Release stop** returns GARBY to idle; it then needs a new Start.
- **Route fault.** If a route cannot be completed (for example a turn that cannot finish), GARBY
  stops where it is and sends an SMS. Move it back to its base by hand, then clear the fault with
  the supervised reset described in [`DEPLOYMENT_AND_ACCEPTANCE.md`](DEPLOYMENT_AND_ACCEPTANCE.md).

## Before running on the floor

Follow [`DEPLOYMENT_AND_ACCEPTANCE.md`](DEPLOYMENT_AND_ACCEPTANCE.md). With the wheels lifted,
check wheel direction, the gyro's turn direction, the LiDAR orientation and the sonar sweep;
then calibrate (gyro scale, steps per degree and per centimetre, LiDAR mounting) as described
in [`docs/navigation-2026-11-11.md`](docs/navigation-2026-11-11.md); then run short supervised
legs before the full route. Keep a hardwired emergency stop within reach.

## Verify the software

From the repository root:

```bash
python3 -B tools/verify_project.py --host-only            # all host checks
python3 -B -m unittest discover -s RasPi -p 'test_*.py'    # Pi tests
python3 -B tools/run_firmware_tests.py                     # firmware logic tests
```

`python3 -B tools/verify_project.py` without `--host-only` also builds both ESP32 sketches and
the Android app when their toolchains are installed. No verification command uploads firmware.

## Safety notes

- The LiDAR sits on top of the bin, so it sees what crosses its plane at about chest height.
  It cannot tell a person from an object, and it does not see children, chairs or bins below
  that plane; the low front sonar covers only the area ahead.
- The wheels are driven open loop (no encoders): slip is not detected, and stopping distance
  must be measured on the real floor with the real payload.
- Test with someone in charge of the emergency stop until the robot has been accepted on the floor.

## Documentation

- [`SYSTEM_ARCHITECTURE.md`](SYSTEM_ARCHITECTURE.md): protocols, timing, parameters, ownership
- [`DEPLOYMENT_AND_ACCEPTANCE.md`](DEPLOYMENT_AND_ACCEPTANCE.md): flashing, Pi configuration, acceptance tests
- [`RasPi/README.md`](RasPi/README.md): Pi runtime, monitor and environment variables
- [`BLE_CONNECTION_GUIDE.md`](BLE_CONNECTION_GUIDE.md): diagnosing the Pi-to-bridge Bluetooth link
- [`MAIN_EXECUTOR_CODE/PINOUTS.txt`](MAIN_EXECUTOR_CODE/PINOUTS.txt): wiring
- [`docs/`](docs/): design and investigation reports
