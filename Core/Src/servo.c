#include "servo.h"
#include "tim.h"

void Servo_Init(void) {
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    Servo_SetPulse(1500); /* Force mechanical center position on startup */
}

void Servo_SetPulse(uint16_t pulse_us) {
    int32_t deviation = (int32_t)pulse_us - 1500;
    int32_t scaled_pulse = 1500 + ((deviation * 14) / 10);

    if (scaled_pulse < 800)  scaled_pulse = 800;
    if (scaled_pulse > 2200) scaled_pulse = 2200;

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (uint16_t)scaled_pulse);
}
