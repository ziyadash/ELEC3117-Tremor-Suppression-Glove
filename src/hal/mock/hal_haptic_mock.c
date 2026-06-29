#ifdef NATIVE_TEST
#include "hal/hal_haptic.h"
#include <string.h>

static uint8_t s_last_drive[HAL_HAPTIC_CHANNELS];
static uint32_t s_set_drive_call_count = 0u;

uint8_t hal_haptic_init(void)
{
    memset(s_last_drive, 0, sizeof(s_last_drive));
    s_set_drive_call_count = 0u;
    return 0x07u;  /* all channels ok */
}

void hal_haptic_set_drive(const DriveCommand *cmd)
{
    for (int i = 0; i < HAL_HAPTIC_CHANNELS; i++)
        s_last_drive[i] = cmd->ch[i];
    s_set_drive_call_count++;
}

void hal_haptic_stop_all(void)
{
    memset(s_last_drive, 0, sizeof(s_last_drive));
}

/* Test accessors */
uint8_t mock_haptic_get_drive(int channel)
{
    if (channel < 0 || channel >= HAL_HAPTIC_CHANNELS) return 0u;
    return s_last_drive[channel];
}

uint32_t mock_haptic_get_call_count(void) { return s_set_drive_call_count; }
void     mock_haptic_reset(void) { hal_haptic_init(); }

#endif /* NATIVE_TEST */
