#pragma once
#include <stdint.h>
#include <stdbool.h>

/* Raw 6-axis data from IMU, straight from the sensor registers. */
typedef struct {
    int16_t accel_x;  /* raw register value */
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;   /* raw register value */
    int16_t gyro_y;
    int16_t gyro_z;
} ImuRawData;

/* Physical-unit data after scale conversion. */
typedef struct {
    float accel_x_g;    /* acceleration in g */
    float accel_y_g;
    float accel_z_g;
    float gyro_x_rads;  /* angular rate in rad/s */
    float gyro_y_rads;
    float gyro_z_rads;
} ImuData;

/* Initialise the IMU peripheral and configure for 200 Hz ODR.
   Returns true on success, false if the device does not respond. */
bool hal_imu_init(void);

/* Trigger a non-blocking burst read. Data will be ready at next
   hal_imu_read() call after the transfer completes. Call once per
   control cycle at the END of the cycle (pipelining). */
void hal_imu_trigger_read(void);

/* Copy the most recently completed DMA read into *out.
   If no read has completed yet, returns the last valid data (or zeros).
   Returns false if an I2C error occurred on the last transfer. */
bool hal_imu_read(ImuData *out);

/* Convert raw register values to physical units (inlined for performance). */
void hal_imu_raw_to_phys(const ImuRawData *raw, ImuData *phys);
