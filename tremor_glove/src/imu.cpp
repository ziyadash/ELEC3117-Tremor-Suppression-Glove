#include "../include/imu.h"
#include "../include/mux.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

/* Confirmed via bringup/i2c_scanner + bringup/mux_channel_scanner: single
   shared bus also carrying the TCA9548A mux. The MPU6050 itself sits
   behind mux channel 1 (drivers are on 2 and 3) - every transaction to
   it needs that channel selected first, same as the haptic drivers. */
#define IMU_SDA_PIN 24
#define IMU_SCL_PIN 23
#define IMU_MUX_CHANNEL 1

static MPU6050 mpu;
static bool dmpReady = false;
static uint16_t packetSize;
static uint8_t fifoBuffer[64];

static Quaternion q;
static VectorInt16 aa;      /* raw accel, sensor frame */
static VectorInt16 aaReal;  /* gravity-compensated linear accel, sensor frame */
static VectorInt16 gyroRaw; /* raw gyro, sensor frame */
static VectorFloat gravity;

/* DMP-space accel is scaled to a fixed ~8192 LSB/g regardless of AFS_SEL
   (i2cdevlib/DMP firmware quirk). dmpInitialize() hardcodes gyro FS to
   +-2000dps, giving a fixed 16.4 LSB/(deg/s). Both per esp32-mpu6050-prototype. */
static const float DMP_ACCEL_LSB_PER_G = 8192.0f;
static const float DMP_GYRO_LSB_PER_DPS = 16.4f;

/* Per-board calibration offsets. Replace with this board's values from the
   "IMU_Zero"/"MPU6050_calibration" i2cdevlib example sketch. Zero still
   works, the orientation just drifts more. */
static int16_t ax_offset = 0, ay_offset = 0, az_offset = 0;
static int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;

bool imu_init() {
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  Wire.setClock(400000);

  if (!mux_select(IMU_MUX_CHANNEL)) return false;

  mpu.initialize();
  if (!mpu.testConnection()) {
    mux_deselect();
    return false;
  }

  uint8_t devStatus = mpu.dmpInitialize();

  mpu.setXAccelOffset(ax_offset);
  mpu.setYAccelOffset(ay_offset);
  mpu.setZAccelOffset(az_offset);
  mpu.setXGyroOffset(gx_offset);
  mpu.setYGyroOffset(gy_offset);
  mpu.setZGyroOffset(gz_offset);

  if (devStatus != 0) {
    mux_deselect();
    return false;
  }

  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.setDMPEnabled(true);
  dmpReady = true;
  packetSize = mpu.dmpGetFIFOPacketSize();

  mux_deselect();
  return true;
}

bool imu_read(ImuSample *out) {
  if (!dmpReady) return false;
  if (!mux_select(IMU_MUX_CHANNEL)) return false;

  uint16_t fifoCount = mpu.getFIFOCount();
  if (fifoCount >= 1024) {
    mpu.resetFIFO();
    mux_deselect();
    return false;
  }
  if (fifoCount < packetSize) {
    mux_deselect();
    return false;
  }

  mpu.getFIFOBytes(fifoBuffer, packetSize);
  mux_deselect(); /* done with the bus - the dmpGet* calls below only parse fifoBuffer, no I2C */

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetGyro(&gyroRaw, fifoBuffer);

  out->qw = q.w; out->qx = q.x; out->qy = q.y; out->qz = q.z;
  out->ax = aaReal.x / DMP_ACCEL_LSB_PER_G;
  out->ay = aaReal.y / DMP_ACCEL_LSB_PER_G;
  out->az = aaReal.z / DMP_ACCEL_LSB_PER_G;
  out->gx = gyroRaw.x / DMP_GYRO_LSB_PER_DPS;
  out->gy = gyroRaw.y / DMP_GYRO_LSB_PER_DPS;
  out->gz = gyroRaw.z / DMP_GYRO_LSB_PER_DPS;
  return true;
}
