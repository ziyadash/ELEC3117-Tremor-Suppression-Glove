#pragma once
#include <stdint.h>

#define MUX_ADDR 0x70

/* Selects exactly one TCA9548A channel (0-7) - this replaces whichever
   channel was selected before (plain register write, not additive), so
   imu.ino and haptic.ino can freely interleave calls without needing to
   coordinate with each other, as long as each selects its own channel
   right before it touches the bus. Returns false if the mux doesn't ack. */
bool mux_select(uint8_t channel);

/* Deselects all channels (control register = 0x00). Call after finishing
   a burst of transactions on the selected channel - leaving a channel
   selected makes every device behind it respond to the next write meant
   for someone else. */
void mux_deselect();
