#ifndef NATIVE_TEST
#include "hal/hal_imu.h"
#include "stm32f4xx_hal.h"
#include <math.h>
#include <string.h>

/* MPU-6050 I2C address (AD0 pin low) */
#define MPU6050_ADDR       (0x68u << 1)

/* Register addresses */
#define REG_PWR_MGMT_1     0x6Bu
#define REG_SMPLRT_DIV     0x19u
#define REG_CONFIG         0x1Au
#define REG_GYRO_CONFIG    0x1Bu
#define REG_ACCEL_CONFIG   0x1Cu
#define REG_INT_ENABLE     0x38u
#define REG_ACCEL_XOUT_H   0x3Bu

/* Gyro full-scale: ±500 °/s → sensitivity 65.5 LSB/(°/s) */
#define GYRO_SCALE_RADS    (1.0f / 65.5f * 0.017453293f)  /* LSB → rad/s */
/* Accel full-scale: ±2 g → sensitivity 16384 LSB/g */
#define ACCEL_SCALE_G      (1.0f / 16384.0f)

extern I2C_HandleTypeDef hi2c1;

static uint8_t  s_dma_buf[14];     /* raw burst read buffer */
static ImuData  s_last_valid;      /* last successfully decoded reading */
static uint32_t s_error_count = 0;
static volatile bool s_transfer_done = false;

static bool write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDR, buf, 2, 10) == HAL_OK;
}

bool hal_imu_init(void)
{
    /* Wake from sleep: write 0x00 to PWR_MGMT_1 */
    if (!write_reg(REG_PWR_MGMT_1, 0x00)) return false;

    /* SMPLRT_DIV: ODR = Gyro_Output_Rate / (1 + SMPLRT_DIV)
       Gyro output rate with DLPF enabled = 1 kHz.
       For 200 Hz: SMPLRT_DIV = (1000/200) - 1 = 4 */
    if (!write_reg(REG_SMPLRT_DIV, 0x04)) return false;

    /* DLPF_CFG = 3 → 44 Hz bandwidth (filters mechanical noise above tremor band) */
    if (!write_reg(REG_CONFIG, 0x03)) return false;

    /* Gyro full-scale ±500 °/s (FS_SEL = 1) */
    if (!write_reg(REG_GYRO_CONFIG, 0x08)) return false;

    /* Accel full-scale ±2 g (AFS_SEL = 0, default) */
    if (!write_reg(REG_ACCEL_CONFIG, 0x00)) return false;

    /* Enable data-ready interrupt on INT pin */
    if (!write_reg(REG_INT_ENABLE, 0x01)) return false;

    memset(&s_last_valid, 0, sizeof(s_last_valid));
    return true;
}

void hal_imu_trigger_read(void)
{
    s_transfer_done = false;
    /* Non-blocking DMA read of 14 bytes starting at ACCEL_XOUT_H */
    if (HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_ADDR,
                              REG_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT,
                              s_dma_buf, 14) != HAL_OK) {
        s_error_count++;
    }
}

/* Called from HAL_I2C_MemRxCpltCallback in main.c */
void hal_imu_dma_complete_cb(void)
{
    s_transfer_done = true;
}

bool hal_imu_read(ImuData *out)
{
    if (!s_transfer_done) {
        *out = s_last_valid;
        return false;
    }

    ImuRawData raw;
    raw.accel_x = (int16_t)((s_dma_buf[0]  << 8) | s_dma_buf[1]);
    raw.accel_y = (int16_t)((s_dma_buf[2]  << 8) | s_dma_buf[3]);
    raw.accel_z = (int16_t)((s_dma_buf[4]  << 8) | s_dma_buf[5]);
    /* bytes 6-7 are temperature — skip */
    raw.gyro_x  = (int16_t)((s_dma_buf[8]  << 8) | s_dma_buf[9]);
    raw.gyro_y  = (int16_t)((s_dma_buf[10] << 8) | s_dma_buf[11]);
    raw.gyro_z  = (int16_t)((s_dma_buf[12] << 8) | s_dma_buf[13]);

    hal_imu_raw_to_phys(&raw, out);
    s_last_valid = *out;
    return true;
}

void hal_imu_raw_to_phys(const ImuRawData *raw, ImuData *phys)
{
    phys->accel_x_g   = raw->accel_x * ACCEL_SCALE_G;
    phys->accel_y_g   = raw->accel_y * ACCEL_SCALE_G;
    phys->accel_z_g   = raw->accel_z * ACCEL_SCALE_G;
    phys->gyro_x_rads = raw->gyro_x  * GYRO_SCALE_RADS;
    phys->gyro_y_rads = raw->gyro_y  * GYRO_SCALE_RADS;
    phys->gyro_z_rads = raw->gyro_z  * GYRO_SCALE_RADS;
}

#endif /* NATIVE_TEST */
