#include "servo.h"
#include "tim.h"

void Servo_Init(void) {
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    Servo_SetPulse(1500); /* Force mechanical center position on startup */
}

void Servo_SetPulse(uint16_t pulse_us) {
    /* Restrict pulse values to protect physical steering linkage from binding */
    if (pulse_us < 1000) pulse_us = 1000;
    if (pulse_us > 2000) pulse_us = 2000;

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_us);
}
