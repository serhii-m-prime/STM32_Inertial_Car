#include "motor.h"
#include "tim.h"

#define MOTOR_MAX_PWM 4999

void Motor_Init(void) {
    /* Start hardware PWM channels for Left Motor (TIM3 CH1/CH2) */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

    /* Start hardware PWM channels for Right Motor (TIM3 CH3/CH4) */
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

    /* --- Left Motor Control (Motor A) --- */
    /* AIN1 = TIM3_CH1, AIN2 = TIM3_CH2 */
    if (left_power >= 0) {
        /* Forward (Slow Decay): AIN1 = HIGH, AIN2 = Inverted PWM */
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, MOTOR_MAX_PWM);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, MOTOR_MAX_PWM - left_power);
    } else {
        /* Reverse (Slow Decay): AIN1 = Inverted PWM, AIN2 = HIGH */
        /* Notice the minus sign: -left_power makes the negative number positive */
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, MOTOR_MAX_PWM - (-left_power));
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, MOTOR_MAX_PWM);
    }

    /* --- Right Motor Control (Motor B) --- */
    /* BIN1 = TIM3_CH3, BIN2 = TIM3_CH4 */
    if (right_power >= 0) {
        /* Forward (Slow Decay): BIN1 = HIGH, BIN2 = Inverted PWM */
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, MOTOR_MAX_PWM);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, MOTOR_MAX_PWM - right_power);
    } else {
        /* Reverse (Slow Decay): BIN1 = Inverted PWM, BIN2 = HIGH */
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, MOTOR_MAX_PWM - (-right_power));
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, MOTOR_MAX_PWM);
    }
}
