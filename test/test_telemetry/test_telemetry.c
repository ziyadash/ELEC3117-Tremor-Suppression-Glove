#include <unity.h>
#include "telemetry.h"
#include <string.h>

static uint8_t buf[TELEMETRY_PACKET_SIZE];

void setUp(void)    { memset(buf, 0, sizeof(buf)); }
void tearDown(void) {}

static void pack_default(void)
{
    telemetry_pack(buf, 1234u, 0.5f, 100, -200, 1u, 63u, 0.12f);
}

void test_packet_size_is_13(void)
{
    TEST_ASSERT_EQUAL_UINT(13, TELEMETRY_PACKET_SIZE);
}

void test_struct_size_is_13(void)
{
    TEST_ASSERT_EQUAL_UINT(13, sizeof(TelemetryPacket));
}

void test_header_byte_is_0xAA(void)
{
    pack_default();
    TEST_ASSERT_EQUAL_HEX8(0xAAu, buf[0]);
}

void test_validate_accepts_valid_packet(void)
{
    pack_default();
    TEST_ASSERT_TRUE(telemetry_validate(buf));
}

void test_validate_rejects_wrong_header(void)
{
    pack_default();
    buf[0] = 0x00u;
    TEST_ASSERT_FALSE(telemetry_validate(buf));
}

void test_validate_rejects_corrupted_byte(void)
{
    pack_default();
    buf[5] ^= 0xFFu;  /* flip a data byte */
    TEST_ASSERT_FALSE(telemetry_validate(buf));
}

void test_checksum_is_xor_of_bytes_1_to_11(void)
{
    pack_default();
    uint8_t expected = 0u;
    for (int i = 1; i <= 11; i++) expected ^= buf[i];
    TEST_ASSERT_EQUAL_HEX8(expected, buf[12]);
}

void test_timestamp_roundtrip(void)
{
    telemetry_pack(buf, 0xABCDu, 0.0f, 0, 0, 0, 0, 0.0f);
    TelemetryPacket pkt;
    telemetry_unpack(buf, &pkt);
    TEST_ASSERT_EQUAL_UINT16(0xABCDu, pkt.timestamp_ms);
}

void test_gyro_x_signed_roundtrip(void)
{
    telemetry_pack(buf, 0u, 0.0f, -500, 0, 0, 0, 0.0f);
    TelemetryPacket pkt;
    telemetry_unpack(buf, &pkt);
    TEST_ASSERT_EQUAL_INT16(-500, pkt.gyro_x);
}

void test_tremor_active_flag(void)
{
    telemetry_pack(buf, 0u, 0.0f, 0, 0, 1u, 0, 0.0f);
    TelemetryPacket pkt;
    telemetry_unpack(buf, &pkt);
    TEST_ASSERT_EQUAL_UINT8(1u, pkt.tremor_active);
}

void test_rms_saturation(void)
{
    /* RMS of 10.0 rad/s → scaled by 100 = 1000, saturates at 255 */
    telemetry_pack(buf, 0u, 0.0f, 0, 0, 0, 0, 10.0f);
    TelemetryPacket pkt;
    telemetry_unpack(buf, &pkt);
    TEST_ASSERT_EQUAL_UINT8(255u, pkt.rms_x100);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_packet_size_is_13);
    RUN_TEST(test_struct_size_is_13);
    RUN_TEST(test_header_byte_is_0xAA);
    RUN_TEST(test_validate_accepts_valid_packet);
    RUN_TEST(test_validate_rejects_wrong_header);
    RUN_TEST(test_validate_rejects_corrupted_byte);
    RUN_TEST(test_checksum_is_xor_of_bytes_1_to_11);
    RUN_TEST(test_timestamp_roundtrip);
    RUN_TEST(test_gyro_x_signed_roundtrip);
    RUN_TEST(test_tremor_active_flag);
    RUN_TEST(test_rms_saturation);
    return UNITY_END();
}
