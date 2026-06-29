#ifdef NATIVE_TEST
#include "hal/hal_imu.h"
#include <math.h>
#include <string.h>

/* Composite mock signal: tremor (5 Hz) + voluntary movement (0.8 Hz) + noise.
   Noise is approximated with a simple LCG — no stdlib rand() dependency. */

static float  s_tremor_amp    = 0.25f;   /* rad/s, above default ON_THRESHOLD */
static float  s_tremor_hz     = 5.0f;
static bool   s_tremor_on     = true;
static float  s_voluntary_amp = 0.05f;
static float  s_t             = 0.0f;    /* simulated time (seconds) */
static uint32_t s_tick_ms     = 0u;
static ImuData  s_last;

/* Simple LCG for repeatable noise */
static uint32_t s_rng = 12345u;
static float lcg_noise(float sigma)
{
    s_rng = s_rng * 1664525u + 1013904223u;
    /* Map to [-1,1] then scale */
    float n = (float)(int32_t)s_rng / 2147483648.0f;
    return n * sigma;
}

void mock_imu_set_tremor(float amp_rads, float freq_hz, bool on)
{
    s_tremor_amp = amp_rads;
    s_tremor_hz  = freq_hz;
    s_tremor_on  = on;
}

bool hal_imu_init(void)
{
    s_t      = 0.0f;
    s_tick_ms = 0u;
    memset(&s_last, 0, sizeof(s_last));
    return true;
}

void hal_imu_trigger_read(void) { /* no-op for mock */ }

bool hal_imu_read(ImuData *out)
{
    float dt = 0.005f;  /* 200 Hz */
    s_t += dt;
    s_tick_ms += 5u;

    float tremor_component = s_tremor_on
        ? s_tremor_amp * sinf(2.0f * 3.14159265f * s_tremor_hz * s_t)
        : 0.0f;
    float voluntary_component =
        s_voluntary_amp * sinf(2.0f * 3.14159265f * 0.8f * s_t);
    float noise = lcg_noise(0.02f);

    float gyro_y = tremor_component + voluntary_component + noise;

    out->gyro_x_rads  = noise * 0.3f;
    out->gyro_y_rads  = gyro_y;
    out->gyro_z_rads  = noise * 0.2f;
    out->accel_x_g    = 0.0f;
    out->accel_y_g    = sinf(s_t * 0.1f) * 0.1f;  /* slow tilt */
    out->accel_z_g    = 1.0f;                       /* gravity */

    s_last = *out;
    return true;
}

void hal_imu_raw_to_phys(const ImuRawData *raw, ImuData *phys)
{
    /* Not used in mock path */
    (void)raw; (void)phys;
}

#endif /* NATIVE_TEST */
