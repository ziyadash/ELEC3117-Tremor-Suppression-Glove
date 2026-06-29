#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "control.h"

#define HAL_HAPTIC_CHANNELS 3

/* Initialise all haptic channels. For DRV2605L: wakes each driver via
   TCA9548A mux, writes MODE and FEEDBACK_CTRL registers, leaves in RTP mode.
   For ERM placeholder: configures PWM timers.
   Returns bitmask of successfully initialised channels (bit N = channel N). */
uint8_t hal_haptic_init(void);

/* Write drive intensities for all channels in one call.
   Channels with failed init (not set in init return mask) are silently skipped.
   intensity: 0–127 for DRV2605L RTP; mapped to PWM duty for ERM. */
void hal_haptic_set_drive(const DriveCommand *cmd);

/* Set a single channel to zero drive. */
void hal_haptic_stop_all(void);
