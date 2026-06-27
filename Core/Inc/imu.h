#ifndef IMU_H
#define IMU_H

#include "main.h"
#include <stdbool.h>

bool IMU_Init(void);
uint8_t IMU_GetWhoAmI(void);

#endif /* IMU_H */
