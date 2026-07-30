#include "../include/haptic.h"
#include "../include/mux.h"
#include <Wire.h>

#define DRV2605L_ADDR 0x5A

#define DRV_REG_MODE          0x01
#define DRV_REG_RTP_INPUT     0x02
#define DRV_REG_FEEDBACK_CTRL 0x1A

#define DRV_MODE_RTP     0x05  /* Real-Time Playback mode */
#define DRV_FEEDBACK_LRA 0x80  /* FEEDBACK_CTRL bit 7: LRA mode */

const uint8_t HAPTIC_MUX_CHANNELS[HAPTIC_CHANNELS] = {2, 3};

static bool drv_write_reg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(DRV2605L_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

uint8_t haptic_init() {
  uint8_t channel_ok = 0;

  for (uint8_t i = 0; i < HAPTIC_CHANNELS; i++) {
    if (!mux_select(HAPTIC_MUX_CHANNELS[i])) continue;

    bool ok = true;
    ok &= drv_write_reg(DRV_REG_MODE, DRV_MODE_RTP);
    ok &= drv_write_reg(DRV_REG_FEEDBACK_CTRL, DRV_FEEDBACK_LRA);
    ok &= drv_write_reg(DRV_REG_RTP_INPUT, 0x00);

    mux_deselect();
    if (ok) channel_ok |= (uint8_t)(1 << i);
  }
  return channel_ok;
}

void haptic_set_drive(uint8_t channel_ok, uint8_t intensity) {
  for (uint8_t i = 0; i < HAPTIC_CHANNELS; i++) {
    if (!(channel_ok & (1 << i))) continue;
    if (!mux_select(HAPTIC_MUX_CHANNELS[i])) continue;
    drv_write_reg(DRV_REG_RTP_INPUT, intensity);
    mux_deselect();
  }
}
