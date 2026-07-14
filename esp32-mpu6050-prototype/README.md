# ESP32 + MPU6050 prototype

Early bring-up prototype for the tremor glove's IMU + BLE telemetry path,
built on an off-the-shelf ESP32 dev board instead of the project's STM32F401
target. This is **not** compatible with the main firmware in the repo root
(`../src`, `../webapp`) — different MCU, different BLE stack (native ESP32
BLE vs. HM-10 over UART), different packet format (quaternion + linear accel
vs. gyro-only). It exists purely to validate the sensor-fusion -> BLE ->
3D-visualization pipeline quickly before committing to the STM32 build.

## Firmware

`firmware/esp32_mpu6050_ble_gpio1314.ino` — reads fused orientation
(quaternion) and gravity-compensated linear acceleration from the MPU6050's
onboard DMP, and streams both over BLE as 7 little-endian floats
(w, x, y, z, ax, ay, az), 28 bytes per notification, ~50Hz.

Requires the Arduino IDE with the ESP32 board package, plus the
`I2Cdevlib-MPU6050` library (Library Manager). See the wiring/calibration
notes at the top of the `.ino` file.

## Web app

`webapp/index.html` — a Three.js 3D visualizer (orientation, acceleration
vector, dead-reckoned position trail) plus a togglable tremor-detection
overlay (bandpass + RMS envelope + hysteresis, adapted from the EMG-based
design to work on accel-only data).

Web Bluetooth requires a secure context, so `file://` won't work — serve it
locally, e.g.:

```bash
cd webapp
python3 -m http.server 8000
# open http://localhost:8000 in Chrome or Edge
```
