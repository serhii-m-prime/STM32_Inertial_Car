#include "odometry.h"
#include "tim.h"

/* * Hardware Constants
 * Motor Gearbox: 1:150
 * Hall Sensor Base: 7 PPR
 * Quadrature Mode X4: 7 * 4 * 150 = 4200 ticks per wheel revolution
 * Wheel Diameter: 44mm -> Circumference = 0.13823 meters
 */
#define TICKS_PER_REV         4200.0f
#define WHEEL_CIRCUMFERENCE_M 0.13823f
#define METERS_PER_TICK       (WHEEL_CIRCUMFERENCE_M / TICKS_PER_REV) // ~0.0000329119f

/* Internal state for free-running counters */
static uint16_t prev_cnt_left = 0;   /* TIM1 is 16-bit */
static uint32_t prev_cnt_right = 0;  /* TIM2 is 32-bit */

/* Physical kinematic state */
static float speed_left_mps = 0.0f;
static float speed_right_mps = 0.0f;
static float distance_left_m = 0.0f;
static float distance_right_m = 0.0f;

void Odometry_Init(void) {
    /* Enable hardware quadrature decoding on both timers */
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);

    /* Latch initial positions to avoid startup speed spikes */
    prev_cnt_left = __HAL_TIM_GET_COUNTER(&htim1);
    prev_cnt_right = __HAL_TIM_GET_COUNTER(&htim2);
}

void Odometry_Update(float dt_seconds) {
    if (dt_seconds <= 0.0f) return;

    /* Read absolute hardware registers */
    uint16_t current_cnt_left = __HAL_TIM_GET_COUNTER(&htim1);
    uint32_t current_cnt_right = __HAL_TIM_GET_COUNTER(&htim2);

    /* * Calculate delta ticks
     * Magic of 2's complement: Casting unsigned difference to signed integer
     * automatically handles physical timer overflow/underflow perfectly!
     */
    int16_t delta_ticks_left = (int16_t)(current_cnt_left - prev_cnt_left);
    int32_t delta_ticks_right = (int32_t)(current_cnt_right - prev_cnt_right);

    /* Update history buffers for next iteration */
    prev_cnt_left = current_cnt_left;
    prev_cnt_right = current_cnt_right;

    /* * Hardware direction correction
     * Depending on physical wiring of Phase A/Phase B, the encoder might count backwards
     * when the wheel drives forward. We will invert the sign here if testing shows reversed logic.
     * For now, we assume positive delta means forward.
     */
    // delta_ticks_left = -delta_ticks_left;   // Uncomment if left wheel speed is negative when moving forward
    delta_ticks_right = -delta_ticks_right;

    /* 4. Convert ticks to physical units (Meters) */
    float delta_m_left = (float)delta_ticks_left * METERS_PER_TICK;
    float delta_m_right = (float)delta_ticks_right * METERS_PER_TICK;

    /* 5. Calculate linear velocity (m/s) */
    speed_left_mps = delta_m_left / dt_seconds;
    speed_right_mps = delta_m_right / dt_seconds;

    /* 6. Accumulate global traveled distance */
    distance_left_m += delta_m_left;
    distance_right_m += delta_m_right;
}

/* Pure Getters */
float Odometry_GetSpeedLeft(void)   { return speed_left_mps; }
float Odometry_GetSpeedRight(void)  { return speed_right_mps; }
float Odometry_GetDistanceLeft(void){ return distance_left_m; }
float Odometry_GetDistanceRight(void){ return distance_right_m; }
