#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    TREMOR_STATE_IDLE   = 0,
    TREMOR_STATE_ACTIVE = 1,
} TremorState;

typedef struct {
    float on_threshold;   /* RMS level to enter ACTIVE (rad/s) */
    float off_threshold;  /* RMS level to leave ACTIVE (rad/s) */
    uint32_t on_hold_ms;  /* sustained above on_threshold before activating (ms) */
    uint32_t off_hold_ms; /* sustained below off_threshold before deactivating (ms) */
} TremorConfig;

typedef struct {
    TremorState  state;
    TremorConfig cfg;
    uint32_t     threshold_entered_ms; /* tick when level first crossed threshold */
} TremorDetector;

/* Default config. Call tremor_set_config() to override after calibration. */
void tremor_init(TremorDetector *td);

/* Override thresholds at runtime (called after calibration). */
void tremor_set_config(TremorDetector *td, const TremorConfig *cfg);

/* Update state machine.
   rms:        current RMS envelope value (rad/s)
   now_ms:     HAL_GetTick() or equivalent millisecond timestamp
   Returns current state after update. */
TremorState tremor_update(TremorDetector *td, float rms, uint32_t now_ms);

bool tremor_is_active(const TremorDetector *td);
