/* Tremor Suppression Glove — ESP32-C5-DevKitC-1
   ------------------------------------------------------------------------
   MPU6050 (DMP) -> on-device dual-channel (gyro+accel) tremor detection,
   ported from esp32-mpu6050-prototype/webapp/js/tremor.js (see
   tremor_detect.ino) -> 2x DRV2605L haptic drive -> BLE notify (raw
   telemetry + tremor status), paced 50Hz.

   Wiring (confirmed via bringup/i2c_scanner + bringup/mux_channel_scanner,
   both run against real hardware):
     Single shared I2C bus: SDA=GPIO24, SCL=GPIO23
       - TCA9548A mux @ 0x70, RST=GPIO27 (held HIGH in software - also the
         board's onboard addressable RGB LED pin, not currently a conflict
         since nothing else here drives GPIO27)
       - MPU6050 @ 0x68 behind mux channel 1 (see mux.ino - every IMU
         transaction selects this channel first, same as the haptic drivers)
       - 2x DRV2605L @ 0x5A behind mux channels 2, 3

   BLE packet (55 bytes, little-endian, see ble_link.ino):
     [0-39]  10 floats: qw,qx,qy,qz, ax,ay,az, gx,gy,gz  (unchanged from
             esp32-mpu6050-prototype's format)
     [40]    tremor_active (uint8)
     [41]    lead_channel (uint8, 0=gyro/1=accel)
     [42-45] rms (float32)
     [46-49] threshold (float32)
     [50-53] freq_hz (float32)
     [54]    drive_intensity (uint8)

   REQUIRED LIBRARIES (Arduino Library Manager): "I2Cdevlib-MPU6050"
   (search "MPU6050"). BLE is the ESP32 core's bundled library, no extra
   install. See esp32-mpu6050-prototype/README.md for background on the
   DMP calibration offsets in imu.ino.
*/
#include <Arduino.h>
#include "imu.h"
#include "tremor_detect.h"
#include "haptic.h"
#include "ble_link.h"

#define MUX_RST_PIN 27
#define SAMPLE_PERIOD_MS 20  /* 50 Hz - matches tremor_detect.ino's FS_HZ assumption */

static uint8_t s_hapticChannelOk = 0;
static uint32_t s_nextSampleDue = 0;

void setup() {
  Serial.begin(115200);

  pinMode(MUX_RST_PIN, OUTPUT);
  digitalWrite(MUX_RST_PIN, HIGH); /* hold TCA9548A out of reset */

  if (!imu_init()) Serial.println("MPU6050/DMP init failed");

  s_hapticChannelOk = haptic_init();
  if (s_hapticChannelOk != ((1 << HAPTIC_CHANNELS) - 1))
    Serial.printf("Haptic channel init mask: 0x%02X (expected 0x03)\n", s_hapticChannelOk);

  tremor_detect_init();

  ble_link_init("TremorGlove");
  Serial.println("BLE advertising as 'TremorGlove'");
  Serial.println("Calibrating tremor baseline (~2.6s) - keep the glove still...");
}

void loop() {
  /* Always attempt to drain the FIFO - this is what keeps it from backing
     up, independent of the paced processing/notify cadence below. */
  ImuSample imu;
  if (!imu_read(&imu)) return;

  uint32_t now = millis();
  if (now < s_nextSampleDue) return;
  s_nextSampleDue = now + SAMPLE_PERIOD_MS;

  TremorResult tr = tremor_detect_update(imu.gx, imu.gy, imu.gz,
                                          imu.ax, imu.ay, imu.az, now);

  /* Linear drive scaled by how far the leading channel's RMS sits above
     its threshold once active; 0 otherwise. No PI controller, no
     per-channel weighting - see tremor_glove's plan notes if you want to
     add differentiated intensity by motor placement later. */
  uint8_t drive = 0;
  if (tr.active && tr.threshold > 0.0f) {
    float ratio = tr.rms / tr.threshold;
    float scaled = (ratio - 1.0f) * 64.0f;
    if (scaled < 0.0f) scaled = 0.0f;
    if (scaled > 127.0f) scaled = 127.0f;
    drive = (uint8_t)scaled;
  }
  haptic_set_drive(s_hapticChannelOk, drive);

  ble_link_send(imu.qw, imu.qx, imu.qy, imu.qz,
                imu.ax, imu.ay, imu.az,
                imu.gx, imu.gy, imu.gz,
                tr.active, tr.lead_channel, tr.rms, tr.threshold, tr.freq_hz,
                drive);
}
