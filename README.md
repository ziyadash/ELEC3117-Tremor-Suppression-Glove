# Tremor Suppression Glove — ELEC3117

Closed-loop vibrotactile tremor suppression glove for Parkinson's disease.
ESP32-C5-DevKitC-1 + MPU-6050 (DMP) + 2× DRV2605L LRA actuators, both the
IMU and the actuators reached via a TCA9548A I2C mux + native BLE.

Tremor detection (dual-channel gyro+accel, adaptively calibrated, 3-gate)
runs on-device — the web app is a pure telemetry/status display, nothing
gets computed client-side.

---

## Hardware / wiring

Confirmed against real hardware via the sketches in `bringup/`
(`i2c_scanner`, `mux_channel_scanner`):

| Signal | Pin |
|---|---|
| I2C SDA | GPIO24 |
| I2C SCL | GPIO23 |
| TCA9548A mux RST | GPIO27 (held HIGH in software — also the board's onboard addressable RGB LED pin, not currently a conflict) |

Single shared I2C bus: TCA9548A mux (`0x70`) is the only thing directly
reachable. The MPU6050 (`0x68`) sits behind mux **channel 1**, and both
DRV2605L drivers (`0x5A`) sit behind mux **channels 2 and 3** — every
transaction to any of them selects its channel first (see `mux.ino`).

---

## Firmware — `tremor_glove/`

Plain Arduino sketch, no PlatformIO. Open `tremor_glove/tremor_glove.ino`
in the Arduino IDE or VS Code's Arduino extension, select board
**ESP32C5 Dev Module**, pick your serial port, verify/upload.

```
tremor_glove/
  tremor_glove.ino     — setup()/loop(): DMP read -> tremor_update -> haptic drive -> BLE notify, paced 50Hz
  mux.h / mux.ino       — shared TCA9548A channel select/deselect, used by both imu.ino and haptic.ino
  imu.h / imu.ino       — MPU6050 DMP init + per-sample read, behind mux channel 1
  tremor_detect.h/.ino  — dual-channel (gyro+accel) tremor detector, ported from the old client-side JS
  haptic.h / haptic.ino — 2x DRV2605L drive, behind mux channels 2 and 3
  ble_link.h / ble_link.ino — BLE GATT server, packet pack + notify
```

**Required library** (Arduino Library Manager): `I2Cdevlib-MPU6050`
(search "MPU6050"). BLE uses the ESP32 core's bundled library, no extra
install.

**MPU6050 calibration offsets**: every board's sensor has different
offsets; `imu.ino` defaults them to 0 (works, but drifts). Run the
`IMU_Zero`/`MPU6050_calibration` example sketch from i2cdevlib and paste
your six values into `imu.ino`.

**Tremor baseline calibration** happens automatically at boot — the glove
needs to be held still for ~2.6s after power-on while the detector
establishes its resting-noise threshold.

### BLE packet (55 bytes, little-endian)

| Offset | Size | Field |
|---|---|---|
| 0–39 | 40 | 10 floats: qw,qx,qy,qz, ax,ay,az, gx,gy,gz |
| 40 | 1 | tremor_active (uint8) |
| 41 | 1 | lead_channel (uint8, 0=gyro/1=accel) |
| 42–45 | 4 | rms (float32) |
| 46–49 | 4 | threshold (float32) |
| 50–53 | 4 | freq_hz (float32) |
| 54 | 1 | drive_intensity (uint8) |

---

## Web app — `webapp/`

No build step — runs directly in the browser as ES modules.

```bash
python3 -m http.server 8080 --directory webapp
```

Open `http://localhost:8080` in **Chrome or Edge** (required for Web
Bluetooth) and click "Connect device" — look for **TremorGlove**.

Shows live orientation (3D hand), acceleration, dead-reckoned position,
and tremor status (state/channel/frequency/RMS/threshold) exactly as the
firmware computed it.

---

## `esp32-mpu6050-prototype/` — standalone reference, kept on the side

An earlier bring-up (same MPU6050 DMP → BLE approach, no tremor detection
or haptics) with its own copy of the web app. Its `webapp/js/tremor.js`
still has the **original client-side** JS tremor detector. Its firmware
(`firmware/esp32_mpu6050_ble_gpio2423/`) talks to the MPU6050 **directly**
on GPIO24/23, with no mux channel selection — that only works if the IMU
is wired straight to the main bus. Now that the glove assembly routes the
IMU through mux channel 1 instead, this prototype needs the IMU
temporarily rewired off the mux to work standalone; it hasn't been
updated to select mux channel 1 the way `tremor_glove/imu.ino` does.

---

## `bringup/` — hardware diagnostic sketches (gitignored)

`i2c_scanner` (confirms the TCA9548A mux responds at `0x70`) and
`mux_channel_scanner` (confirms each mux channel routes to the right
DRV2605L, and that channels are actually isolated). Not part of the
tracked firmware — useful the next time something's wired up from
scratch.
