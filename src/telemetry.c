#include "telemetry.h"
#include <string.h>

static uint8_t compute_checksum(const uint8_t *buf)
{
    uint8_t cs = 0;
    /* XOR bytes [1..11] — excludes header (0) and checksum (12) */
    for (uint8_t i = 1; i < TELEMETRY_PACKET_SIZE - 1; i++)
        cs ^= buf[i];
    return cs;
}

void telemetry_pack(uint8_t buf[TELEMETRY_PACKET_SIZE],
                    uint16_t timestamp_ms,
                    float    gyro_magnitude_rads,
                    int16_t  gyro_x_raw,
                    int16_t  gyro_y_raw,
                    uint8_t  tremor_active,
                    uint8_t  drive_intensity,
                    float    rms_rads)
{
    /* gyro magnitude: store as uint16 in units of 0.01 rad/s */
    uint16_t gyro_mag_enc = (uint16_t)(gyro_magnitude_rads * 100.0f);

    /* RMS: store as uint8 in units of 0.01 rad/s, saturate at 2.55 */
    float rms_scaled = rms_rads * 100.0f;
    uint8_t rms_enc = (rms_scaled > 255.0f) ? 255u : (uint8_t)rms_scaled;

    buf[0]  = TELEMETRY_HEADER;
    buf[1]  = (uint8_t)(timestamp_ms & 0xFF);
    buf[2]  = (uint8_t)(timestamp_ms >> 8);
    buf[3]  = (uint8_t)(gyro_mag_enc & 0xFF);
    buf[4]  = (uint8_t)(gyro_mag_enc >> 8);
    buf[5]  = (uint8_t)((uint16_t)gyro_x_raw & 0xFF);
    buf[6]  = (uint8_t)((uint16_t)gyro_x_raw >> 8);
    buf[7]  = (uint8_t)((uint16_t)gyro_y_raw & 0xFF);
    buf[8]  = (uint8_t)((uint16_t)gyro_y_raw >> 8);
    buf[9]  = tremor_active;
    buf[10] = drive_intensity;
    buf[11] = rms_enc;
    buf[12] = compute_checksum(buf);
}

bool telemetry_validate(const uint8_t buf[TELEMETRY_PACKET_SIZE])
{
    if (buf[0] != TELEMETRY_HEADER)
        return false;
    return buf[TELEMETRY_PACKET_SIZE - 1] == compute_checksum(buf);
}

void telemetry_unpack(const uint8_t buf[TELEMETRY_PACKET_SIZE],
                      TelemetryPacket *pkt)
{
    pkt->header         = buf[0];
    pkt->timestamp_ms   = (uint16_t)(buf[1] | ((uint16_t)buf[2] << 8));
    pkt->gyro_magnitude = (uint16_t)(buf[3] | ((uint16_t)buf[4] << 8));
    pkt->gyro_x         = (int16_t) (buf[5] | ((uint16_t)buf[6] << 8));
    pkt->gyro_y         = (int16_t) (buf[7] | ((uint16_t)buf[8] << 8));
    pkt->tremor_active  = buf[9];
    pkt->drive_intensity = buf[10];
    pkt->rms_x100       = buf[11];
    pkt->checksum       = buf[12];
}
