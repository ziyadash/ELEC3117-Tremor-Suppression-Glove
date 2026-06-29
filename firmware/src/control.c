#include "control.h"
#include <string.h>
#include <math.h>

static const float CH_WEIGHTS[HAPTIC_CHANNELS] = {
    WEIGHT_CH0, WEIGHT_CH1, WEIGHT_CH2
};

static uint8_t clamp_drive(float v)
{
    if (v < 0.0f)   return 0u;
    if (v > 127.0f) return 127u;
    return (uint8_t)v;
}

void control_init(PIController *pi, const PIConfig *cfg)
{
    pi->cfg    = *cfg;
    pi->i_term = 0.0f;
}

DriveCommand control_compute(PIController *pi, bool tremor_active,
                             float amplitude)
{
    DriveCommand cmd;
    memset(&cmd, 0, sizeof(cmd));

    if (!tremor_active)
        return cmd;

    /* PI: error = amplitude (setpoint is zero tremor) */
    float error = amplitude;

    pi->i_term += pi->cfg.ki * error * pi->cfg.dt;
    /* Anti-windup: clamp integral independently before summing */
    if (pi->i_term >  pi->cfg.i_max) pi->i_term =  pi->cfg.i_max;
    if (pi->i_term < -pi->cfg.i_max) pi->i_term = -pi->cfg.i_max;

    float base_drive = pi->cfg.kp * error + pi->i_term;

    for (int ch = 0; ch < HAPTIC_CHANNELS; ch++)
        cmd.ch[ch] = clamp_drive(base_drive * CH_WEIGHTS[ch]);

    return cmd;
}

void control_reset(PIController *pi)
{
    pi->i_term = 0.0f;
}
