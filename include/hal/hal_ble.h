#pragma once
#include <stdint.h>
#include <stdbool.h>

/* Initialise UART1 at 9600 baud with DMA TX.
   Returns true on success. */
bool hal_ble_init(void);

/* Non-blocking DMA transmit.
   buf must remain valid until hal_ble_tx_done() returns true.
   Returns false if a previous transmit is still in progress. */
bool hal_ble_transmit(const uint8_t *buf, uint16_t len);

/* Returns true if the DMA transmit buffer is free (previous TX complete). */
bool hal_ble_tx_done(void);
