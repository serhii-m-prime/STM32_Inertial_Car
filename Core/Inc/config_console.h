#ifndef CONFIG_CONSOLE_H
#define CONFIG_CONSOLE_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>

void ConfigConsole_Init(UART_HandleTypeDef *uart);
void ConfigConsole_Update(void);
const char *ConfigConsole_GetDisplayTitle(void);
const char *ConfigConsole_GetDisplayLine(void);
bool ConfigConsole_HasError(void);

#endif
