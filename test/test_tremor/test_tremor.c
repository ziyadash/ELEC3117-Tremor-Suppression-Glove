#include <unity.h>
#include "tremor.h"

static TremorDetector td;

static const TremorConfig TEST_CFG = {
    .on_threshold  = 0.10f,
    .off_threshold = 0.05f,
    .on_hold_ms    = 300u,
    .off_hold_ms   = 500u,
};

void setUp(void)
{
    tremor_init(&td);
    tremor_set_config(&td, &TEST_CFG);
}
void tearDown(void) {}

/* Simulate time advancing: call tremor_update repeatedly for duration_ms */
static TremorState run_for_ms(float rms, uint32_t duration_ms, uint32_t *now)
{
    TremorState s = TREMOR_STATE_IDLE;
    /* Step in 5 ms increments (200 Hz) */
    for (uint32_t t = 0; t < duration_ms; t += 5) {
        *now += 5;
        s = tremor_update(&td, rms, *now);
    }
    return s;
}

void test_stays_idle_below_threshold(void)
{
    uint32_t now = 0;
    TremorState s = run_for_ms(0.05f, 1000u, &now);
    TEST_ASSERT_EQUAL_INT(TREMOR_STATE_IDLE, s);
}

void test_idle_to_active_after_hold(void)
{
    uint32_t now = 0;
    /* Below threshold first */
    run_for_ms(0.05f, 100u, &now);
    /* Above ON threshold for on_hold_ms + 1 step (timer starts on first detection tick) */
    TremorState s = run_for_ms(0.20f, 305u, &now);
    TEST_ASSERT_EQUAL_INT(TREMOR_STATE_ACTIVE, s);
}

void test_brief_spike_does_not_activate(void)
{
    uint32_t now = 0;
    /* Above threshold but only for 100 ms — less than on_hold_ms */
    run_for_ms(0.20f, 100u, &now);
    /* Drop back below */
    TremorState s = run_for_ms(0.02f, 50u, &now);
    TEST_ASSERT_EQUAL_INT(TREMOR_STATE_IDLE, s);
}

void test_active_holds_through_brief_drop(void)
{
    uint32_t now = 0;
    /* Activate */
    run_for_ms(0.20f, 400u, &now);
    TEST_ASSERT_EQUAL_INT(TREMOR_STATE_ACTIVE, tremor_update(&td, 0.20f, now));
    /* Drop below OFF threshold for only 200 ms — less than off_hold_ms */
    TremorState s = run_for_ms(0.02f, 200u, &now);
    TEST_ASSERT_EQUAL_INT(TREMOR_STATE_ACTIVE, s);
}

void test_active_to_idle_after_off_hold(void)
{
    uint32_t now = 0;
    run_for_ms(0.20f, 400u, &now);  /* activate */
    TremorState s = run_for_ms(0.02f, 550u, &now);  /* hold off for 550 ms */
    TEST_ASSERT_EQUAL_INT(TREMOR_STATE_IDLE, s);
}

void test_transition_time_uses_ms_not_samples(void)
{
    /* Simulate at 100 Hz (10 ms steps) — hold time should still be ~300 ms */
    tremor_init(&td);
    tremor_set_config(&td, &TEST_CFG);

    uint32_t now = 1000u;
    /* 25 steps × 10 ms = 250 ms — not enough */
    for (int i = 0; i < 25; i++) {
        now += 10;
        tremor_update(&td, 0.20f, now);
    }
    TEST_ASSERT_EQUAL_INT(TREMOR_STATE_IDLE, td.state);

    /* 6 more steps × 10 ms = 60 ms more → total 310 ms → should activate
       (timer starts on first detection tick, so need on_hold_ms + 1 step) */
    for (int i = 0; i < 6; i++) {
        now += 10;
        tremor_update(&td, 0.20f, now);
    }
    TEST_ASSERT_EQUAL_INT(TREMOR_STATE_ACTIVE, td.state);
}

void test_is_active_helper(void)
{
    uint32_t now = 0;
    TEST_ASSERT_FALSE(tremor_is_active(&td));
    run_for_ms(0.20f, 400u, &now);
    TEST_ASSERT_TRUE(tremor_is_active(&td));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_stays_idle_below_threshold);
    RUN_TEST(test_idle_to_active_after_hold);
    RUN_TEST(test_brief_spike_does_not_activate);
    RUN_TEST(test_active_holds_through_brief_drop);
    RUN_TEST(test_active_to_idle_after_off_hold);
    RUN_TEST(test_transition_time_uses_ms_not_samples);
    RUN_TEST(test_is_active_helper);
    return UNITY_END();
}
