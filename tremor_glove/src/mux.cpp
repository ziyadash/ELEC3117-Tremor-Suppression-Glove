#include "../include/mux.h"
#include <Wire.h>

bool mux_select(uint8_t channel) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write((uint8_t)(1 << channel));
  return Wire.endTransmission() == 0;
}

void mux_deselect() {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write((uint8_t)0x00);
  Wire.endTransmission();
}
