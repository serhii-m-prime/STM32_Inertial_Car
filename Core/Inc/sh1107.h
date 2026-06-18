#ifndef SH1107_H
#define SH1107_H

#include "stm32f4xx_hal.h"

// Display resolution
#define SH1107_WIDTH  128
#define SH1107_HEIGHT 128

// Buffer size: 128 pixels * 128 pixels / 8 bits = 2048 bytes
#define SH1107_BUFFER_SIZE (SH1107_WIDTH * SH1107_HEIGHT / 8)

// Initialize display hardware
void SH1107_Init(void);

// Update the physical screen with RAM buffer content using DMA
void SH1107_UpdateScreen(void);

// Low-level pixel manipulation in the RAM buffer
void SH1107_DrawPixel(uint8_t x, uint8_t y, uint8_t color);

#endif // SH1107_H
