#include <unity.h>
#include "filters.h"
#include <math.h>
#include <string.h>

#define FS       200.0f
#define DT       (1.0f / FS)
#define N_CYCLES 400  /* 2 seconds of simulation to let filter settle */

static BandpassFilter bp;
static CompFilter     comp;

void setUp(void)
{
    filters_bandpass_init(&bp);
    filters_comp_init(&comp);
}
void tearDown(void) {}

/* Run a sine wave through the bandpass and return peak output amplitude
   (measured over the last 100 samples after settling). */
static float bandpass_response(float freq_hz)
{
    float peak = 0.0f;
    for (int i = 0; i < N_CYCLES; i++) {
        float t   = (float)i * DT;
        float in  = sinf(2.0f * 3.14159265f * freq_hz * t);
        float out = filters_bandpass_apply(&bp, in);
        if (i > N_CYCLES - 100) {
            float a = fabsf(out);
            if (a > peak) peak = a;
        }
    }
    return peak;
}

void test_bandpass_passes_5hz(void)
{
    /* 5 Hz is in the passband — output should be ≥ 70% of input amplitude */
    float resp = bandpass_response(5.0f);
    TEST_ASSERT_TRUE_MESSAGE(resp >= 0.70f, "5 Hz should pass with >70% amplitude");
}

void test_bandpass_rejects_1hz(void)
{
    /* 1 Hz is below stopband — output should be <10% of input */
    float resp = bandpass_response(1.0f);
    TEST_ASSERT_TRUE_MESSAGE(resp < 0.10f, "1 Hz should be attenuated >90%");
}

void test_bandpass_rejects_20hz(void)
{
    /* 20 Hz is above stopband */
    float resp = bandpass_response(20.0f);
    TEST_ASSERT_TRUE_MESSAGE(resp < 0.10f, "20 Hz should be attenuated >90%");
}

void test_bandpass_state_reset(void)
{
    /* Drive filter with a large signal, then reinit and check output is zero */
    for (int i = 0; i < 50; i++)
        filters_bandpass_apply(&bp, 1.0f);
    filters_bandpass_init(&bp);
    float out = filters_bandpass_apply(&bp, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, out);
}

void test_comp_filter_tracks_static_tilt(void)
{
    /* Static tilt: accel says 0.5 rad, gyro says 0. Filter should converge to 0.5. */
    float angle = 0.0f;
    for (int i = 0; i < 500; i++)
        angle = filters_comp_update(&comp, 0.0f, 0.5f, DT);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.5f, angle);
}

void test_comp_filter_integrates_gyro(void)
{
    /* Constant gyro rate of 1 rad/s for 0.1 s with zero accel reference.
       Angle should advance roughly 0.1 * ALPHA * 1.0 ≈ 0.096 rad
       (not exact because complementary filter blends both). */
    float angle = 0.0f;
    for (int i = 0; i < 20; i++)
        angle = filters_comp_update(&comp, 1.0f, 0.0f, DT);
    TEST_ASSERT_TRUE(angle > 0.05f);
    TEST_ASSERT_TRUE(angle < 0.12f);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_bandpass_passes_5hz);
    RUN_TEST(test_bandpass_rejects_1hz);
    RUN_TEST(test_bandpass_rejects_20hz);
    RUN_TEST(test_bandpass_state_reset);
    RUN_TEST(test_comp_filter_tracks_static_tilt);
    RUN_TEST(test_comp_filter_integrates_gyro);
    return UNITY_END();
}
