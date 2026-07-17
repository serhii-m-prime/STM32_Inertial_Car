#ifndef SERVO_H
#define SERVO_H

#include "main.h"

/* Start PWM generation for the steering servo on TIM4 Channel 1 */
void Servo_Init(void);
void Servo_Center(void);

/* Set front wheel angle using standard pulse duration: range 1000us to 2000us */
void Servo_SetPulse(uint16_t pulse_us);

#endif /* SERVO_H */
