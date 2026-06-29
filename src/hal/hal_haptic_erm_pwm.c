#ifndef NATIVE_TEST
#ifdef HAPTIC_ERM_PLACEHOLDER

/* ERM coin motor placeholder driver via PWM through BC547 transistors.
   PIN ASSIGNMENTS ARE NOT FINALISED — change before PCB layout.
   Current placeholder: TIM3 CH1=PA6, CH2=PA7, CH3=PB0. */

#include "hal/hal_haptic.h"
#include "stm32f4xx_hal.h"
#include <string.h>

#define PWM_FREQ_HZ     200u   /* arbitrary, ERM doesn't care about frequency */
#define PWM_PERIOD      (84000000u / PWM_FREQ_HZ)  /* APB1 = 84 MHz on F401 */

extern TIM_HandleTypeDef htim3;

static uint8_t s_channel_ok = 0x07u;  /* all channels assumed ok for ERM */

static void set_pwm(uint32_t channel, uint8_t intensity)
{
    /* Map 0–127 drive byte to 0–100% duty cycle */
    uint32_t pulse = ((uint32_t)intensity * PWM_PERIOD) / 127u;
    __HAL_TIM_SET_COMPARE(&htim3, channel, pulse);
}

uint8_t hal_haptic_init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    set_pwm(TIM_CHANNEL_1, 0);
    set_pwm(TIM_CHANNEL_2, 0);
    set_pwm(TIM_CHANNEL_3, 0);
    return s_channel_ok;
}

void hal_haptic_set_drive(const DriveCommand *cmd)
{
    set_pwm(TIM_CHANNEL_1, cmd->ch[0]);
    set_pwm(TIM_CHANNEL_2, cmd->ch[1]);
    set_pwm(TIM_CHANNEL_3, cmd->ch[2]);
}

void hal_haptic_stop_all(void)
{
    set_pwm(TIM_CHANNEL_1, 0);
    set_pwm(TIM_CHANNEL_2, 0);
    set_pwm(TIM_CHANNEL_3, 0);
}

#endif /* HAPTIC_ERM_PLACEHOLDER */
#endif /* NATIVE_TEST */
