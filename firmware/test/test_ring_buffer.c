#include <unity.h>
#include "ring_buffer.h"
#include <math.h>

static RingBuf rb;

void setUp(void)    { ringbuf_init(&rb); }
void tearDown(void) {}

void test_initial_state_count_zero(void)
{
    TEST_ASSERT_EQUAL_UINT16(0, rb.count);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, ringbuf_rms(&rb));
}

void test_count_increments_to_capacity(void)
{
    for (int i = 0; i < RMS_WINDOW_SAMPLES + 5; i++)
        ringbuf_push(&rb, 1.0f);
    TEST_ASSERT_EQUAL_UINT16(RMS_WINDOW_SAMPLES, rb.count);
}

void test_rms_all_ones(void)
{
    for (int i = 0; i < RMS_WINDOW_SAMPLES; i++)
        ringbuf_push(&rb, 1.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, ringbuf_rms(&rb));
}

void test_rms_partial_fill(void)
{
    /* Push 5 known values: [3, 4, 0, 0, 0] → RMS = sqrt((9+16)/5) = sqrt(5) */
    ringbuf_push(&rb, 3.0f);
    ringbuf_push(&rb, 4.0f);
    ringbuf_push(&rb, 0.0f);
    ringbuf_push(&rb, 0.0f);
    ringbuf_push(&rb, 0.0f);
    TEST_ASSERT_EQUAL_UINT16(5, rb.count);
    float expected = sqrtf(25.0f / 5.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, expected, ringbuf_rms(&rb));
}

void test_head_wraps_correctly(void)
{
    /* Fill completely with 1.0, then push one 0.0 — head should wrap */
    for (int i = 0; i < RMS_WINDOW_SAMPLES; i++)
        ringbuf_push(&rb, 1.0f);
    ringbuf_push(&rb, 0.0f);
    TEST_ASSERT_EQUAL_UINT16(1, rb.head);
    TEST_ASSERT_EQUAL_UINT16(RMS_WINDOW_SAMPLES, rb.count);
    /* RMS should be slightly less than 1 now */
    float rms = ringbuf_rms(&rb);
    TEST_ASSERT_TRUE(rms < 1.0f);
    TEST_ASSERT_TRUE(rms > 0.9f);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_initial_state_count_zero);
    RUN_TEST(test_count_increments_to_capacity);
    RUN_TEST(test_rms_all_ones);
    RUN_TEST(test_rms_partial_fill);
    RUN_TEST(test_head_wraps_correctly);
    return UNITY_END();
}
