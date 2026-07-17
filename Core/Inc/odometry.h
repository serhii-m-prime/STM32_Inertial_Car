#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "main.h"

/*
 * Start hardware encoder timers and initialize baseline variables
 */
void Odometry_Init(void);

/*
 * Calculate current speed and distance.
 * Must be called periodically with a known time delta (e.g. 0.02f for 20ms)
 */
void Odometry_Update(float dt_seconds);

/* Getters for mathematical Control Loop (returned in meters and meters/second) */
float Odometry_GetSpeedLeft(void);
float Odometry_GetSpeedRight(void);
float Odometry_GetDistanceLeft(void);
float Odometry_GetDistanceRight(void);

#endif /* ODOMETRY_H */
