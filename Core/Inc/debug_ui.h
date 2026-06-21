#ifndef DEBUG_UI_H
#define DEBUG_UI_H

#include <stdint.h>

void DebugUI_Init(void);
void DebugUI_Clear(void);
void DebugUI_Show(void);

/* Print formatted text. 'line' is from 0 to 15 (for 128px height / 8px font) */
void DebugUI_PrintLine(uint8_t line, const char* format, ...);

/* Draw a progress bar */
void DebugUI_ProgressBar(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t percentage);

/* Draw controller signal position*/
void DebugUI_StickSlider(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint16_t value);

#endif // DEBUG_UI_H
