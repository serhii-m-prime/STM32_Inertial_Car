#include "motor.h"
#include "tim.h"

#define MOTOR_MAX_PWM 4999

void Motor_Init(void) {
    /* Start hardware PWM channels for Left Motor */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    /* Start hardware PWM channels for Right Motor */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

    /* Ensure vehicle stays still on power-up */
    Motor_SetPower(0, 0);
}

void Motor_SetPower(int16_t left_power, int16_t right_power) {
    /* Hard safety bounding limits checking */
    if (left_power > MOTOR_MAX_PWM)  left_power = MOTOR_MAX_PWM;
    if (left_power < -MOTOR_MAX_PWM) left_power = -MOTOR_MAX_PWM;
    if (right_power > MOTOR_MAX_PWM)  right_power = MOTOR_MAX_PWM;
    if (right_power < -MOTOR_MAX_PWM) right_power = -MOTOR_MAX_PWM;

    /* Left Motor Control: Channel 1 (IN1), Channel 2 (IN2) */
    if (left_power >= 0) {
        /* Forward rotation using Slow Decay (Brake) configuration */
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, MOTOR_MAX_PWM);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, MOTOR_MAX_PWM - left_power);
    } else {
        /* Reverse rotation using Slow Decay (Brake) configuration */
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, MOTOR_MAX_PWM + left_power);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, MOTOR_MAX_PWM);
    }

    /* Right Motor Control: Channel 3 (IN1), Channel 4 (IN2) */
    if (right_power >= 0) {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, MOTOR_MAX_PWM);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, MOTOR_MAX_PWM - right_power);
    } else {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, MOTOR_MAX_PWM + right_power);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, MOTOR_MAX_PWM);
    }
}
