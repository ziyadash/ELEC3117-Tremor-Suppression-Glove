#pragma once
#include <stdint.h>
#include <stdbool.h>

#define TELEMETRY_HEADER   0xAAu
#define TELEMETRY_PACKET_SIZE 13u

/* Packet layout (13 bytes, __packed):
   [0]     header          uint8   0xAA
   [1-2]   timestamp_ms    uint16  device uptime ms (wraps ~65 s)
   [3-4]   gyro_magnitude  uint16  gyro magnitude × 100 (0.01 rad/s LSB)
   [5-6]   gyro_x          int16   raw gyro X register value
   [7-8]   gyro_y          int16   raw gyro Y register value
   [9]     tremor_active   uint8   0=IDLE 1=ACTIVE
   [10]    drive_intensity uint8   CH0 drive byte 0–127
   [11]    rms_x100        uint8   RMS × 100 (saturates at 2.55 rad/s)
   [12]    checksum        uint8   XOR of bytes [1..11]
*/
typedef struct __attribute__((packed)) {
    uint8_t  header;
    uint16_t timestamp_ms;
    uint16_t gyro_magnitude;
    int16_t  gyro_x;
    int16_t  gyro_y;
    uint8_t  tremor_active;
    uint8_t  drive_intensity;
    uint8_t  rms_x100;
    uint8_t  checksum;
} TelemetryPacket;

/* Populate and serialise a packet into buf (must be TELEMETRY_PACKET_SIZE bytes).
   Computes and writes checksum. */
void telemetry_pack(uint8_t buf[TELEMETRY_PACKET_SIZE],
                    uint16_t timestamp_ms,
                    float    gyro_magnitude_rads,
                    int16_t  gyro_x_raw,
                    int16_t  gyro_y_raw,
                    uint8_t  tremor_active,
                    uint8_t  drive_intensity,
                    float    rms_rads);

/* Validate a received packet buffer. Returns false if header wrong or
   checksum mismatch — caller should discard silently. */
bool telemetry_validate(const uint8_t buf[TELEMETRY_PACKET_SIZE]);

/* Deserialise a validated buffer into a TelemetryPacket struct. */
void telemetry_unpack(const uint8_t buf[TELEMETRY_PACKET_SIZE],
                      TelemetryPacket *pkt);
