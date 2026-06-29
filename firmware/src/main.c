#ifndef NATIVE_TEST
#include "stm32f4xx_hal.h"
#include "hal/hal_imu.h"
#include "hal/hal_haptic.h"
#include "hal/hal_ble.h"
#include "ring_buffer.h"
#include "filters.h"
#include "tremor.h"
#include "control.h"
#include "telemetry.h"
#include <math.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* CubeMX-generated peripheral handles (defined in generated stm32 files) */
extern I2C_HandleTypeDef  hi2c1;
extern I2C_HandleTypeDef  hi2c2;
extern UART_HandleTypeDef huart1;

/* ------------------------------------------------------------------ */
/* Control loop state */
static volatile bool     s_loop_flag   = false;  /* set by SysTick every 5 ms */
static uint32_t          s_loop_count  = 0u;
static uint32_t          s_telem_count = 0u;

/* Algorithm state */
static RingBuf       s_rms_buf;
static BandpassFilter s_bp_filter;
static CompFilter    s_comp_filter;
static TremorDetector s_tremor;
static PIController  s_pi;

/* Telemetry TX buffer — must remain valid through DMA transfer */
static uint8_t s_telem_buf[TELEMETRY_PACKET_SIZE];

/* ------------------------------------------------------------------ */
/* SysTick fires every 1 ms (HAL default).
   We set the loop flag every 5 ms (200 Hz). */
void HAL_SYSTICK_Callback(void)
{
    static uint32_t tick = 0u;
    if (++tick >= 5u) {
        tick = 0u;
        s_loop_flag = true;
    }
}

/* Forward DMA completion callbacks to HAL driver modules */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
        hal_imu_dma_complete_cb();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
        hal_ble_tx_complete_cb();
}

/* ------------------------------------------------------------------ */
/* 2-second calibration: collect baseline RMS, set detection thresholds */
static void calibrate(void)
{
    const uint32_t CAL_DURATION_MS = 2000u;
    uint32_t start = HAL_GetTick();
    float rms_sum  = 0.0f;
    uint32_t n     = 0u;

    while ((HAL_GetTick() - start) < CAL_DURATION_MS) {
        if (!s_loop_flag) continue;
        s_loop_flag = false;

        ImuData imu;
        hal_imu_read(&imu);
        hal_imu_trigger_read();

        float gyro_mag = sqrtf(imu.gyro_x_rads * imu.gyro_x_rads
                             + imu.gyro_y_rads * imu.gyro_y_rads
                             + imu.gyro_z_rads * imu.gyro_z_rads);

        float filtered = filters_bandpass_apply(&s_bp_filter, gyro_mag);
        ringbuf_push(&s_rms_buf, filtered);
        rms_sum += ringbuf_rms(&s_rms_buf);
        n++;
    }

    float baseline_rms = (n > 0) ? (rms_sum / (float)n) : 0.0f;

    TremorConfig cfg = {
        .on_threshold  = baseline_rms * 3.0f,
        .off_threshold = baseline_rms * 2.0f,
        .on_hold_ms    = 300u,
        .off_hold_ms   = 500u,
    };
    tremor_set_config(&s_tremor, &cfg);
}

/* ------------------------------------------------------------------ */
int main(void)
{
    HAL_Init();
    /* SystemClock_Config() and peripheral MX_Init() calls are generated
       by CubeMX into separate files — call them here once generated. */

    /* Algorithm init */
    ringbuf_init(&s_rms_buf);
    filters_bandpass_init(&s_bp_filter);
    filters_comp_init(&s_comp_filter);
    tremor_init(&s_tremor);

    const PIConfig pi_cfg = {
        .kp    = 200.0f,   /* empirically tuned on hardware */
        .ki    = 0.0f,     /* P-only to start */
        .i_max = 60.0f,
        .dt    = 0.005f,
    };
    control_init(&s_pi, &pi_cfg);

    /* Peripheral init */
    hal_imu_init();
    hal_haptic_init();
    hal_ble_init();

    /* Prime the IMU read pipeline */
    hal_imu_trigger_read();

    /* Calibrate thresholds */
    calibrate();

    /* ---- 200 Hz control loop ---- */
    while (1) {
        if (!s_loop_flag) continue;
        s_loop_flag = false;
        s_loop_count++;

        /* 1. Read IMU (data from previous cycle's DMA) */
        ImuData imu;
        hal_imu_read(&imu);

        /* 2. Trigger next DMA read immediately (pipelined) */
        hal_imu_trigger_read();

        /* 3. Complementary filter → wrist angle (telemetry only) */
        float accel_angle = atan2f(imu.accel_y_g, imu.accel_z_g);
        filters_comp_update(&s_comp_filter, imu.gyro_y_rads, accel_angle, 0.005f);

        /* 4. Gyro vector magnitude → bandpass filter */
        float gyro_mag = sqrtf(imu.gyro_x_rads * imu.gyro_x_rads
                             + imu.gyro_y_rads * imu.gyro_y_rads
                             + imu.gyro_z_rads * imu.gyro_z_rads);
        float filtered = filters_bandpass_apply(&s_bp_filter, gyro_mag);

        /* 5. RMS envelope */
        ringbuf_push(&s_rms_buf, filtered);
        float rms = ringbuf_rms(&s_rms_buf);

        /* 6. Tremor state machine */
        TremorState state = tremor_update(&s_tremor, rms, HAL_GetTick());
        bool active = (state == TREMOR_STATE_ACTIVE);

        /* 7. PI controller */
        if (!active) control_reset(&s_pi);
        DriveCommand cmd = control_compute(&s_pi, active, rms);

        /* 8. Drive actuators */
        hal_haptic_set_drive(&cmd);

        /* 9. Telemetry at 20 Hz (every 10th loop iteration) */
        if (++s_telem_count >= 10u) {
            s_telem_count = 0u;
            if (hal_ble_tx_done()) {
                telemetry_pack(s_telem_buf,
                               (uint16_t)(HAL_GetTick() & 0xFFFFu),
                               gyro_mag,
                               imu.gyro_x_rads * 65.5f / 0.017453293f,  /* back to raw-ish */
                               imu.gyro_y_rads * 65.5f / 0.017453293f,
                               (uint8_t)active,
                               cmd.ch[0],
                               rms);
                hal_ble_transmit(s_telem_buf, TELEMETRY_PACKET_SIZE);
            }
        }

        /* 10. Pet the watchdog */
        /* HAL_IWDG_Refresh(&hiwdg); */  /* uncomment once IWDG configured in CubeMX */
    }
}

#endif /* NATIVE_TEST */
