#pragma once
#include <stdint.h>

#define HAPTIC_CHANNELS 2

/* Mux channel indices carrying the two DRV2605L drivers - the IMU now
   occupies mux channel 1, so the drivers are on 2 and 3. */
extern const uint8_t HAPTIC_MUX_CHANNELS[HAPTIC_CHANNELS];

/* Initialises both DRV2605L drivers through the TCA9548A mux (RTP mode,
   ERM feedback, zero drive). Returns a bitmask (bit N corresponds to
   HAPTIC_MUX_CHANNELS[N]) of channels that initialised successfully;
   failed channels are silently skipped by haptic_set_drive() for the
   rest of the session rather than retried mid-loop. */
uint8_t haptic_init();

/* Same intensity (0-127) to every successfully-initialised channel.
   0 stops all motors. */
void haptic_set_drive(uint8_t channel_ok, uint8_t intensity);
