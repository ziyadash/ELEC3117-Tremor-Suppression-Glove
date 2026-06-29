#ifndef NATIVE_TEST
#ifdef HAPTIC_DRV2605L

#include "hal/hal_haptic.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* TCA9548A I2C address (all address pins low) */
#define TCA9548A_ADDR      (0x70u << 1)
/* DRV2605L I2C address (ADDR pin low) */
#define DRV2605L_ADDR      (0x5Au << 1)

/* DRV2605L register addresses */
#define DRV_REG_STATUS         0x00u
#define DRV_REG_MODE           0x01u
#define DRV_REG_RTP_INPUT      0x02u
#define DRV_REG_FEEDBACK_CTRL  0x1Au
#define DRV_REG_CONTROL3       0x1Du

/* MODE = 0x05 → RTP (Real-Time Playback) mode */
#define DRV_MODE_RTP           0x05u
/* FEEDBACK_CTRL bit 7: 1 = LRA mode */
#define DRV_FEEDBACK_LRA       0x80u

extern I2C_HandleTypeDef hi2c2;

static uint8_t s_channel_ok = 0x00u;  /* bitmask of successfully inited channels */

static bool mux_select(uint8_t ch)
{
    uint8_t cmd = (uint8_t)(1u << ch);
    return HAL_I2C_Master_Transmit(&hi2c2, TCA9548A_ADDR, &cmd, 1, 10) == HAL_OK;
}

static void mux_deselect(void)
{
    uint8_t cmd = 0x00u;
    HAL_I2C_Master_Transmit(&hi2c2, TCA9548A_ADDR, &cmd, 1, 10);
}

static bool drv_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return HAL_I2C_Master_Transmit(&hi2c2, DRV2605L_ADDR, buf, 2, 10) == HAL_OK;
}

uint8_t hal_haptic_init(void)
{
    s_channel_ok = 0x00u;

    for (uint8_t ch = 0; ch < HAL_HAPTIC_CHANNELS; ch++) {
        if (!mux_select(ch)) continue;

        bool ok = true;
        /* Put in RTP mode */
        ok &= drv_write_reg(DRV_REG_MODE, DRV_MODE_RTP);
        /* LRA mode (bit 7 of FEEDBACK_CTRL) */
        ok &= drv_write_reg(DRV_REG_FEEDBACK_CTRL, DRV_FEEDBACK_LRA);
        /* Zero drive on init */
        ok &= drv_write_reg(DRV_REG_RTP_INPUT, 0x00u);

        mux_deselect();

        if (ok)
            s_channel_ok |= (uint8_t)(1u << ch);
    }

    return s_channel_ok;
}

void hal_haptic_set_drive(const DriveCommand *cmd)
{
    for (uint8_t ch = 0; ch < HAL_HAPTIC_CHANNELS; ch++) {
        if (!(s_channel_ok & (1u << ch))) continue;
        if (!mux_select(ch)) continue;
        drv_write_reg(DRV_REG_RTP_INPUT, cmd->ch[ch]);
        mux_deselect();
    }
}

void hal_haptic_stop_all(void)
{
    DriveCommand zero;
    memset(&zero, 0, sizeof(zero));
    hal_haptic_set_drive(&zero);
}

#endif /* HAPTIC_DRV2605L */
#endif /* NATIVE_TEST */
