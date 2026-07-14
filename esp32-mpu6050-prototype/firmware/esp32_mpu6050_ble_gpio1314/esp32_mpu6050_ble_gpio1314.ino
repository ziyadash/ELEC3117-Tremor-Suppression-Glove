/*
  ESP32 + MPU6050 (DMP) -> BLE quaternion + linear acceleration telemetry
  ------------------------------------------------------------------------
  Reads fused orientation (quaternion) and gravity-compensated linear
  acceleration from the MPU6050's onboard DMP and streams both over BLE
  as 7 little-endian floats (w, x, y, z, ax, ay, az), 28 bytes per
  notification. ax/ay/az are in g, expressed in the sensor's body frame.

  WIRING (this version): SDA = GPIO13, SCL = GPIO14
  Note: GPIO13/14 are shared with the native USB Serial/JTAG port (USB_D-/D+).
  If you need that native "USB" port free, flash/monitor via the "UART" port
  instead (the one behind the CP2102N chip) to avoid any pin contention.

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
VectorFloat gravity;  // gravity vector, derived from the quaternion

// DMP-space accel is scaled to a fixed ~8192 LSB/g regardless of the
// MPU6050's AFS_SEL setting (an i2cdevlib/DMP firmware quirk) - divide by
// this to convert dmpGetLinearAccel()'s output into g.
const float DMP_ACCEL_LSB_PER_G = 8192.0f;

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

  // ---- I2C + MPU6050 init ----
  Wire.begin(13, 14);  // SDA = GPIO13, SCL = GPIO14
  Wire.setClock(400000);

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

  fifoCount = mpu.getFIFOCount();

  if (fifoCount >= packetSize) {
    // If FIFO overflowed, reset it and skip this cycle
    if (fifoCount >= 1024) {
      mpu.resetFIFO();
      return;
    }

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

    if (deviceConnected) {
      // Pack w, x, y, z, ax, ay, az as 7 little-endian 32-bit floats = 28 bytes
      float payload[7] = {
        (float)q.w, (float)q.x, (float)q.y, (float)q.z,
        aaReal.x / DMP_ACCEL_LSB_PER_G,
        aaReal.y / DMP_ACCEL_LSB_PER_G,
        aaReal.z / DMP_ACCEL_LSB_PER_G
      };
      pCharacteristic->setValue((uint8_t*)payload, sizeof(payload));
      pCharacteristic->notify();
    }

    // Roughly 50 Hz update rate - fast enough to see tremor-frequency motion smoothly
    delay(20);
  }
}
