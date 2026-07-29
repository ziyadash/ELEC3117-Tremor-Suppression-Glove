#pragma once
#include <stdint.h>

/* Fused/derived per-sample IMU data from the MPU6050's onboard DMP. */
struct ImuSample {
  float qw, qx, qy, qz;  /* fused orientation quaternion */
  float ax, ay, az;      /* gravity-compensated linear accel, g, body frame */
  float gx, gy, gz;      /* raw angular rate, deg/s, body frame */
};

/* Wakes the MPU6050, initialises the DMP, runs its self-calibration.
   Returns false if the device doesn't respond or DMP init fails. */
bool imu_init();

/* Drains one DMP FIFO packet if available and decodes it into *out.
   Call every loop() iteration - this is what keeps the FIFO from
   backing up. Returns false (leaving *out untouched) if no full packet
   is available yet this call. */
bool imu_read(ImuSample *out);
