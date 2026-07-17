#ifndef AT24C256_H
#define AT24C256_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define AT24C256_I2C_ADDRESS (0x50U << 1)

bool AT24C256_IsReady(void);
bool AT24C256_Read(uint16_t address, void *data, uint16_t length);
bool AT24C256_Write(uint16_t address, const void *data, uint16_t length);
const char *AT24C256_GetLastStage(void);
HAL_StatusTypeDef AT24C256_GetLastHALStatus(void);
uint32_t AT24C256_GetLastI2CError(void);
HAL_I2C_StateTypeDef AT24C256_GetI2CState(void);

#endif
