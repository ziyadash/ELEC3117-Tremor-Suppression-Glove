#pragma once
#include <stdint.h>

/* Number of biquad stages for 4th-order Butterworth bandpass.
   A 4th-order bandpass = 4 biquad stages (2 LP poles + 2 HP poles). */
#define BANDPASS_NUM_STAGES 4

typedef struct {
    /* CMSIS-DSP Direct Form II Transposed state: 2 values per stage */
    float state[BANDPASS_NUM_STAGES * 2];
} BandpassFilter;

typedef struct {
    float angle;  /* current angle estimate (radians) */
} CompFilter;

/* Call once before use. Zeroes filter state. */
void filters_bandpass_init(BandpassFilter *f);
void filters_comp_init(CompFilter *f);

/* Apply one sample through the 3–8 Hz IIR biquad bandpass.
   fs = 200 Hz; input is gyro vector magnitude in rad/s. */
float filters_bandpass_apply(BandpassFilter *f, float sample);

/* Complementary filter update.
   gyro_rate_rads: angular rate around flexion-extension axis (rad/s)
   accel_angle:    pitch angle from accelerometer = atan2f(ay, az) (radians)
   dt:             sample period in seconds (0.005 at 200 Hz) */
float filters_comp_update(CompFilter *f, float gyro_rate_rads,
                          float accel_angle, float dt);
