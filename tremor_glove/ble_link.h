#pragma once
#include <stdint.h>

#define BLE_PACKET_SIZE 55

/* Advertises as `device_name`, GATT service/characteristic UUIDs matching
   esp32-mpu6050-prototype (a1b2c3d4-...-0001 / -0002, NOTIFY) so the same
   web app connects to either firmware unmodified aside from packet size. */
void ble_link_init(const char *device_name);

bool ble_link_connected();

/* Packs and notifies one packet: bytes 0-39 are the same 10 little-endian
   floats (w,x,y,z,ax,ay,az,gx,gy,gz) esp32-mpu6050-prototype already
   sends, followed by tremor status (see tremor_glove.ino's packet layout
   comment for the byte offsets). No-op if nothing is connected. */
void ble_link_send(float qw, float qx, float qy, float qz,
                    float ax, float ay, float az,
                    float gx, float gy, float gz,
                    bool tremor_active, uint8_t lead_channel,
                    float rms, float threshold, float freq_hz,
                    uint8_t drive_intensity);
