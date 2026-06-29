#pragma once
#include <stdint.h>
#include <stdbool.h>

#define HAPTIC_CHANNELS 3

/* Per-actuator spatial weighting factors (applied to base drive). */
#define WEIGHT_CH0  1.00f   /* volar wrist  — primary */
#define WEIGHT_CH1  0.70f   /* volar forearm */
#define WEIGHT_CH2  0.50f   /* dorsal wrist */

typedef struct {
    float    kp;        /* proportional gain */
    float    ki;        /* integral gain (default 0 = P-only) */
    float    i_max;     /* anti-windup clamp on integral term */
    float    dt;        /* sample period (seconds) */
} PIConfig;

typedef struct {
    PIConfig cfg;
    float    i_term;    /* accumulated integral */
} PIController;

typedef struct {
    uint8_t ch[HAPTIC_CHANNELS];  /* drive byte per channel, 0–127 */
} DriveCommand;

/* Zero integral, load config. */
void control_init(PIController *pi, const PIConfig *cfg);

/* Compute drive for one control cycle.
   tremor_active: if false, returns all-zero DriveCommand immediately.
   amplitude:     RMS tremor amplitude (rad/s) — setpoint is zero.
   Returns computed DriveCommand with per-channel weights applied. */
DriveCommand control_compute(PIController *pi, bool tremor_active,
                             float amplitude);

/* Reset integral (e.g. when entering IDLE). */
void control_reset(PIController *pi);
