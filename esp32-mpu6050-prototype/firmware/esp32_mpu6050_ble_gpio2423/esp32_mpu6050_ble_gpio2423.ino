/*
  ESP32 + MPU6050 (DMP) -> BLE quaternion + linear accel + gyro telemetry
  ------------------------------------------------------------------------
  Reads fused orientation (quaternion), gravity-compensated linear
  acceleration, and raw angular velocity (gyro) from the MPU6050's onboard
  DMP and streams all three over BLE as 10 little-endian floats
  (w, x, y, z, ax, ay, az, gx, gy, gz), 40 bytes per notification.
  ax/ay/az are in g, gx/gy/gz are in deg/s, all in the sensor's body frame.

  The gyro channel exists because tremor detection (see the web app) uses
  angular velocity as its primary signal, not accel - the gyro is immune
  to the gravity-vector contamination that shows up in accel whenever the
  wrist/arm changes orientation during natural motion.

  WIRING (this version): SDA = GPIO24, SCL = GPIO23
  Same I2C bus as the mux bring-up (bringup/i2c_scanner,
  bringup/mux_channel_scanner) and the main tremor_glove firmware -
  confirmed working against real hardware. The MPU6050 sits behind the
  TCA9548A mux's channel 1 (not directly on the bus), same as
  tremor_glove/imu.ino, so this sketch also drives the mux's RST pin
  (GPIO27) and selects channel 1 before every MPU6050 transaction.

  REQUIRED LIBRARIES (Arduino Library Manager):
    - "I2Cdevlib-MPU6050" by Jeff Rowberg (search "MPU6050" in Library Manager,
      or install from https://github.com/jrowberg/i2cdevlib)
    - "ESP32 BLE Arduino" (bundled with the ESP32 board package)

  IMPORTANT: Every MPU6050 has slightly different sensor offsets.
  The DMP needs these to output a stable, drift-free quaternion.
  Run the "IMU_Zero" or "MPU6050_calibration" example sketch from the
  same i2cdevlib library first, and paste your six offset values into
  the CALIBRATION OFFSETS section below. Skipping this step still
  works, but the orientation will drift and feel "floaty".
*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ---------------- BLE UUIDs ----------------
// Randomly generated - keep these matching in the web app
#define SERVICE_UUID        "a1b2c3d4-0001-4a5b-8c6d-1234567890ab"
#define CHARACTERISTIC_UUID  "a1b2c3d4-0002-4a5b-8c6d-1234567890ab"

// ---------------- TCA9548A mux ----------------
// MPU6050 sits behind channel 1 - select it before every transaction,
// deselect after. Same protocol as bringup/mux_channel_scanner and
// tremor_glove/mux.ino, just inlined here since this sketch is meant to
// stay a single self-contained file.
#define MUX_ADDR 0x70
#define MUX_RST_PIN 27
#define MPU_MUX_CHANNEL 1

bool muxSelect(uint8_t channel) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write((uint8_t)(1 << channel));
  return Wire.endTransmission() == 0;
}

void muxDeselect() {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write((uint8_t)0x00);
  Wire.endTransmission();
}

BLEServer* pServer = nullptr;
BLECharacteristic* pCharacteristic = nullptr;
bool deviceConnected = false;

// ---------------- MPU6050 / DMP ----------------
MPU6050 mpu;

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;
VectorInt16 aa;       // raw accel, sensor frame
VectorInt16 aaReal;   // gravity-compensated linear accel, sensor frame
VectorInt16 gy;       // raw gyro, sensor frame
VectorFloat gravity;  // gravity vector, derived from the quaternion

// Paced BLE notify rate. The DMP fills its FIFO at ~100Hz regardless (see
// dmpInitialize()'s 200Hz sample rate / FIFO divisor of 2); notifying at a
// deliberate, steady 50Hz instead of "as fast as possible" turned out to
// matter in practice: an unthrottled loop only achieved a jittery ~62Hz in
// real BLE conditions (radio/stack overhead, not a clean multiple of
// anything), and that jitter was enough to keep re-triggering the web
// app's sample-rate auto-redesign, which wiped and restarted tremor
// detection's calibration before it could ever finish. 50Hz gives a 25Hz
// Nyquist margin against the detector's 12Hz upper cutoff - comfortably
// enough - while being a rate the loop can actually hold steady.
const uint32_t SAMPLE_PERIOD_MS = 20;
uint32_t nextSampleDue = 0;

// DMP-space accel is scaled to a fixed ~8192 LSB/g regardless of the
// MPU6050's AFS_SEL setting (an i2cdevlib/DMP firmware quirk) - divide by
// this to convert dmpGetLinearAccel()'s output into g.
const float DMP_ACCEL_LSB_PER_G = 8192.0f;

// dmpInitialize() hardcodes the gyro full-scale range to +-2000dps
// (see MPU6050_6Axis_MotionApps20::dmpInitialize(), which calls
// setFullScaleGyroRange(MPU6050_GYRO_FS_2000)), giving a fixed sensitivity
// of 16.4 LSB per deg/s - divide dmpGetGyro()'s raw output by this.
const float DMP_GYRO_LSB_PER_DPS = 16.4f;

// ---------------- CALIBRATION OFFSETS ----------------
// Replace these with YOUR board's values from the calibration sketch.
// Leaving them at 0 will work but the orientation will drift over time.
int16_t ax_offset = 0;
int16_t ay_offset = 0;
int16_t az_offset = 0;
int16_t gx_offset = 0;
int16_t gy_offset = 0;
int16_t gz_offset = 0;

// ---------------- BLE connection callbacks ----------------
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    deviceConnected = true;
    Serial.println("BLE client connected");
  }
  void onDisconnect(BLEServer* server) override {
    deviceConnected = false;
    Serial.println("BLE client disconnected, restarting advertising");
    server->getAdvertising()->start();
  }
};

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // ---- I2C + mux + MPU6050 init ----
  pinMode(MUX_RST_PIN, OUTPUT);
  digitalWrite(MUX_RST_PIN, HIGH);  // hold TCA9548A out of reset

  Wire.begin(24, 23);  // SDA = GPIO24, SCL = GPIO23
  Wire.setClock(400000);

  muxSelect(MPU_MUX_CHANNEL);

  Serial.println("Initializing MPU6050...");
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050 connected" : "MPU6050 connection FAILED");

  Serial.println("Initializing DMP...");
  devStatus = mpu.dmpInitialize();

  mpu.setXAccelOffset(ax_offset);
  mpu.setYAccelOffset(ay_offset);
  mpu.setZAccelOffset(az_offset);
  mpu.setXGyroOffset(gx_offset);
  mpu.setYGyroOffset(gy_offset);
  mpu.setZGyroOffset(gz_offset);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("DMP ready.");
  } else {
    Serial.print("DMP init failed, code: ");
    Serial.println(devStatus);
  }

  muxDeselect();

  // ---- BLE init ----
  BLEDevice::init("TremorGlove-IMU");
  // Default ATT MTU only allows a 20-byte notify payload; the 28-byte
  // quaternion+accel packet needs more room, so request a larger MTU.
  BLEDevice::setMTU(247);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.println("BLE advertising as 'TremorGlove-IMU'");
}

void loop() {
  if (!dmpReady) return;

  muxSelect(MPU_MUX_CHANNEL);
  fifoCount = mpu.getFIFOCount();

  if (fifoCount < packetSize) {
    muxDeselect();
    return;
  }

  // If FIFO overflowed, reset it and skip this cycle
  if (fifoCount >= 1024) {
    mpu.resetFIFO();
    muxDeselect();
    return;
  }

  // Always drain and decode the latest FIFO packet - this is what keeps
  // the FIFO from backing up - but only notify over BLE at the paced
  // SAMPLE_PERIOD_MS cadence below, so the rate the web app actually
  // sees stays steady regardless of the DMP's native output rate.
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  muxDeselect();  // done with the bus - dmpGet* below only parses fifoBuffer, no I2C

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetGyro(&gy, fifoBuffer);

  uint32_t now = millis();
  if (now < nextSampleDue) return;
  nextSampleDue = now + SAMPLE_PERIOD_MS;

  // DEBUG: dump decoded DMP output straight to Serial, independent of
  // BLE - isolates "is the DMP producing real data" from "is BLE/the
  // web app showing it correctly". Remove once data looks sane.
  Serial.printf("q(%.3f,%.3f,%.3f,%.3f) a(%.3f,%.3f,%.3f)g g(%.3f,%.3f,%.3f)dps\n",
                q.w, q.x, q.y, q.z,
                aaReal.x / DMP_ACCEL_LSB_PER_G,
                aaReal.y / DMP_ACCEL_LSB_PER_G,
                aaReal.z / DMP_ACCEL_LSB_PER_G,
                gy.x / DMP_GYRO_LSB_PER_DPS,
                gy.y / DMP_GYRO_LSB_PER_DPS,
                gy.z / DMP_GYRO_LSB_PER_DPS);

  if (deviceConnected) {
    // Pack w, x, y, z, ax, ay, az, gx, gy, gz as 10 little-endian
    // 32-bit floats = 40 bytes
    float payload[10] = {
      (float)q.w, (float)q.x, (float)q.y, (float)q.z,
      aaReal.x / DMP_ACCEL_LSB_PER_G,
      aaReal.y / DMP_ACCEL_LSB_PER_G,
      aaReal.z / DMP_ACCEL_LSB_PER_G,
      gy.x / DMP_GYRO_LSB_PER_DPS,
      gy.y / DMP_GYRO_LSB_PER_DPS,
      gy.z / DMP_GYRO_LSB_PER_DPS
    };
    pCharacteristic->setValue((uint8_t*)payload, sizeof(payload));
    pCharacteristic->notify();
  }
}
