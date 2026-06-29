#include <unity.h>
#include "control.h"

static PIController pi;

static const PIConfig P_ONLY = {
    .kp = 1.0f, .ki = 0.0f, .i_max = 60.0f, .dt = 0.005f
};
static const PIConfig PI_CFG = {
    .kp = 1.0f, .ki = 2.0f, .i_max = 60.0f, .dt = 0.005f
};

void setUp(void)    { control_init(&pi, &P_ONLY); }
void tearDown(void) {}

void test_zero_output_when_inactive(void)
{
    DriveCommand cmd = control_compute(&pi, false, 100.0f);
    for (int i = 0; i < HAPTIC_CHANNELS; i++)
        TEST_ASSERT_EQUAL_UINT8(0, cmd.ch[i]);
}

void test_proportional_scaling_ch0(void)
{
    /* Kp=1, amplitude=64 → base drive=64, ch0 weight=1.0 → drive=64 */
    DriveCommand cmd = control_compute(&pi, true, 64.0f);
    TEST_ASSERT_EQUAL_UINT8(64, cmd.ch[0]);
}

void test_channel_weights_applied(void)
{
    /* Kp=1, amplitude=100 → base=100; ch0=100, ch1=70, ch2=50 */
    DriveCommand cmd = control_compute(&pi, true, 100.0f);
    TEST_ASSERT_EQUAL_UINT8(100, cmd.ch[0]);
    TEST_ASSERT_EQUAL_UINT8(70,  cmd.ch[1]);
    TEST_ASSERT_EQUAL_UINT8(50,  cmd.ch[2]);
}

void test_output_clamps_at_127(void)
{
    PIConfig high_kp = {.kp=10.0f, .ki=0.0f, .i_max=60.0f, .dt=0.005f};
    control_init(&pi, &high_kp);
    DriveCommand cmd = control_compute(&pi, true, 100.0f);
    TEST_ASSERT_EQUAL_UINT8(127, cmd.ch[0]);
}

void test_ki_zero_no_integral_accumulation(void)
{
    DriveCommand first = control_compute(&pi, true, 50.0f);
    for (int i = 0; i < 99; i++)
        control_compute(&pi, true, 50.0f);
    DriveCommand last = control_compute(&pi, true, 50.0f);
    TEST_ASSERT_EQUAL_UINT8(first.ch[0], last.ch[0]);
}

void test_integral_increases_drive_over_time(void)
{
    control_init(&pi, &PI_CFG);
    DriveCommand first = control_compute(&pi, true, 20.0f);
    for (int i = 0; i < 50; i++)
        control_compute(&pi, true, 20.0f);
    DriveCommand later = control_compute(&pi, true, 20.0f);
    TEST_ASSERT_TRUE(later.ch[0] >= first.ch[0]);
}

void test_antiwindup_clamps_integral(void)
{
    PIConfig big_ki = {.kp=0.0f, .ki=100.0f, .i_max=60.0f, .dt=0.005f};
    control_init(&pi, &big_ki);
    for (int i = 0; i < 1000; i++)
        control_compute(&pi, true, 1.0f);
    DriveCommand cmd = control_compute(&pi, true, 1.0f);
    /* i_max=60 so max drive from integral alone = clamp(60, 0, 127) = 60 */
    TEST_ASSERT_TRUE(cmd.ch[0] <= 60);
}

void test_reset_clears_integral(void)
{
    control_init(&pi, &PI_CFG);
    for (int i = 0; i < 50; i++)
        control_compute(&pi, true, 20.0f);
    control_reset(&pi);
    DriveCommand cmd = control_compute(&pi, true, 20.0f);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 20.0f, (float)cmd.ch[0]);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_zero_output_when_inactive);
    RUN_TEST(test_proportional_scaling_ch0);
    RUN_TEST(test_channel_weights_applied);
    RUN_TEST(test_output_clamps_at_127);
    RUN_TEST(test_ki_zero_no_integral_accumulation);
    RUN_TEST(test_integral_increases_drive_over_time);
    RUN_TEST(test_antiwindup_clamps_integral);
    RUN_TEST(test_reset_clears_integral);
    return UNITY_END();
}
