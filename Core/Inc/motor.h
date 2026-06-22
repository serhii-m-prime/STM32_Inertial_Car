#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

/* Initialize PWM channels for rear DC motors and set initial duty to 0 */
void Motor_Init(void);

/* Set raw PWM power for left and right motors: range -4999 to 4999 */
void Motor_SetPower(int16_t left_power, int16_t right_power);

#endif /* MOTOR_H */
