#include "servo.h"
#include "tim.h"
#include "vehicle_config.h"

void Servo_Init(void) {
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    Servo_Center();
}

void Servo_Center(void) {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, vehicle_config.servo_center_us);
}

void Servo_SetPulse(uint16_t pulse_us) {
    int32_t deviation = (int32_t)pulse_us - 1500;
    if (vehicle_config.steering_reverse) deviation = -deviation;
    int32_t scaled_pulse = vehicle_config.servo_center_us +
            ((deviation * vehicle_config.steering_rate_pct) / 100);

    if (scaled_pulse < vehicle_config.servo_min_us) scaled_pulse = vehicle_config.servo_min_us;
    if (scaled_pulse > vehicle_config.servo_max_us) scaled_pulse = vehicle_config.servo_max_us;

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, (uint16_t)scaled_pulse);
}
