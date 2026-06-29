#include "tremor.h"
#include <string.h>

#define DEFAULT_ON_THRESHOLD   0.15f  /* rad/s RMS — set properly after calibration */
#define DEFAULT_OFF_THRESHOLD  0.08f
#define DEFAULT_ON_HOLD_MS     300u
#define DEFAULT_OFF_HOLD_MS    500u

void tremor_init(TremorDetector *td)
{
    memset(td, 0, sizeof(*td));
    td->state = TREMOR_STATE_IDLE;
    td->cfg.on_threshold  = DEFAULT_ON_THRESHOLD;
    td->cfg.off_threshold = DEFAULT_OFF_THRESHOLD;
    td->cfg.on_hold_ms    = DEFAULT_ON_HOLD_MS;
    td->cfg.off_hold_ms   = DEFAULT_OFF_HOLD_MS;
    td->threshold_entered_ms = 0;
}

void tremor_set_config(TremorDetector *td, const TremorConfig *cfg)
{
    td->cfg = *cfg;
}

TremorState tremor_update(TremorDetector *td, float rms, uint32_t now_ms)
{
    switch (td->state) {
    case TREMOR_STATE_IDLE:
        if (rms >= td->cfg.on_threshold) {
            if (td->threshold_entered_ms == 0)
                td->threshold_entered_ms = now_ms;
            else if ((now_ms - td->threshold_entered_ms) >= td->cfg.on_hold_ms)
                td->state = TREMOR_STATE_ACTIVE;
        } else {
            td->threshold_entered_ms = 0;
        }
        break;

    case TREMOR_STATE_ACTIVE:
        if (rms < td->cfg.off_threshold) {
            if (td->threshold_entered_ms == 0)
                td->threshold_entered_ms = now_ms;
            else if ((now_ms - td->threshold_entered_ms) >= td->cfg.off_hold_ms) {
                td->state = TREMOR_STATE_IDLE;
                td->threshold_entered_ms = 0;
            }
        } else {
            td->threshold_entered_ms = 0;
        }
        break;
    }

    return td->state;
}

bool tremor_is_active(const TremorDetector *td)
{
    return td->state == TREMOR_STATE_ACTIVE;
}
