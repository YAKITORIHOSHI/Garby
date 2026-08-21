# GARBY Raspberry Pi Runtime

Production entry point: `final_w_serial.py`  
Required local helper: `bridge_core.py`

## Python packages

```bash
python3 -m pip install -r requirements.txt
```

ROS 2 (`rclpy`, `sensor_msgs`) and the YDLidar ROS 2 driver are system/ROS dependencies and are not installed by `requirements.txt`.

`firebase-admin` is listed for normal telemetry operation, but the safety bridge can continue without Firebase if the SDK, credentials, or network are unavailable.

## Start

```bash
python3 final_w_serial.py --headless
```

## Useful environment variables

- `GARBY_SENSOR_SERIAL_PORT` — sensor UART device; auto-detected when unset
- `GARBY_SENSOR_SERIAL_BAUD` — default `9600`
- `GARBY_BLE_ADDRESS` — optional fixed BLE MAC/address
- `GARBY_BLE_DEVICE_NAME` — default `GarbyESP32`
- `GARBY_LIDAR_YAW_OFFSET_DEG` — physical LiDAR yaw correction; default `0`, must be hardware-verified
- `GARBY_FIREBASE_CREDENTIALS` — Firebase service-account JSON path
- `GARBY_FIREBASE_DATABASE_URL` — override RTDB URL

Do not change UUIDs, path packet format, stale timeout, or LiDAR orientation without updating `../SYSTEM_ARCHITECTURE.md` and re-running the coordinated audit.
