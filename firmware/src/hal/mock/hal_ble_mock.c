#ifdef NATIVE_TEST
#include "hal/hal_ble.h"
#include "telemetry.h"
#include <string.h>

#define MOCK_BLE_BUF_PACKETS 64u

static uint8_t  s_pkt_store[MOCK_BLE_BUF_PACKETS][TELEMETRY_PACKET_SIZE];
static uint32_t s_pkt_count   = 0u;
static uint32_t s_pkt_head    = 0u;  /* oldest unread packet index */

bool hal_ble_init(void)
{
    memset(s_pkt_store, 0, sizeof(s_pkt_store));
    s_pkt_count = 0u;
    s_pkt_head  = 0u;
    return true;
}

bool hal_ble_transmit(const uint8_t *buf, uint16_t len)
{
    if (len != TELEMETRY_PACKET_SIZE) return false;
    uint32_t idx = s_pkt_count % MOCK_BLE_BUF_PACKETS;
    memcpy(s_pkt_store[idx], buf, TELEMETRY_PACKET_SIZE);
    s_pkt_count++;
    return true;
}

bool hal_ble_tx_done(void) { return true; }

/* Retrieve the Nth most recently transmitted packet (0 = most recent).
   Returns false if fewer than n+1 packets have been transmitted. */
bool mock_ble_get_packet(uint32_t n, uint8_t out[TELEMETRY_PACKET_SIZE])
{
    if (n >= s_pkt_count || n >= MOCK_BLE_BUF_PACKETS) return false;
    uint32_t idx = (s_pkt_count - 1u - n) % MOCK_BLE_BUF_PACKETS;
    memcpy(out, s_pkt_store[idx], TELEMETRY_PACKET_SIZE);
    return true;
}

uint32_t mock_ble_get_tx_count(void) { return s_pkt_count; }
void     mock_ble_reset(void) { hal_ble_init(); }

#endif /* NATIVE_TEST */
