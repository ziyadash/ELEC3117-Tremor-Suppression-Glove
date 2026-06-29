#ifndef NATIVE_TEST
#include "hal/hal_ble.h"
#include "stm32f4xx_hal.h"

extern UART_HandleTypeDef huart1;

static volatile bool s_tx_done = true;

bool hal_ble_init(void)
{
    /* UART1 at 9600 baud is configured by CubeMX-generated MX_USART1_UART_Init().
       Nothing additional needed — HM-10 is a transparent bridge. */
    s_tx_done = true;
    return true;
}

bool hal_ble_transmit(const uint8_t *buf, uint16_t len)
{
    if (!s_tx_done) return false;
    s_tx_done = false;
    if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len) != HAL_OK) {
        s_tx_done = true;
        return false;
    }
    return true;
}

bool hal_ble_tx_done(void)
{
    return s_tx_done;
}

/* Called from HAL_UART_TxCpltCallback in main.c */
void hal_ble_tx_complete_cb(void)
{
    s_tx_done = true;
}

#endif /* NATIVE_TEST */
