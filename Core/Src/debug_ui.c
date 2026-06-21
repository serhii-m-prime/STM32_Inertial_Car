#include "debug_ui.h"
#include "sh1107.h"
#include "font5x7.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* You need to define this array in a separate font.c file.
   It contains 95 characters (from space to '~'), 5 bytes each. */
extern const uint8_t font5x7[95][5];

extern uint8_t sh1107_buffer[SH1107_BUFFER_SIZE];

void DebugUI_Init(void) {
    DebugUI_Clear();
    DebugUI_Show();
}

void DebugUI_Clear(void) {
    memset(sh1107_buffer, 0x00, SH1107_BUFFER_SIZE);
}

void DebugUI_Show(void) {
	SH1107_UpdateScreen_NonBlocking();
}

static void Font_DrawChar(uint8_t x, uint8_t y, char c) {
    // If char not in table
    if (c < 0 || c > 127) {
        c = '?';
    }

    // Find standard index in array
    uint16_t font_index = c * 5;

    // each 5 vertical symbols
    for (uint8_t col = 0; col < 5; col++) {
        uint8_t column_data = font_data[font_index + col];

        for (uint8_t row = 0; row < 7; row++) {
            if (column_data & (1 << row)) {
                SH1107_DrawPixel(x + col, y + row, 1); // white pixel
            } else {
                SH1107_DrawPixel(x + col, y + row, 0); // Black background
            }
        }
    }
}

void DebugUI_PrintLine(uint8_t line, const char* format, ...) {
    char buffer[22]; // Max ~21 chars for 128px width (5px char + 1px space)

    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    uint8_t y_pos = line * 8;
    uint8_t current_x = 0;

    for (uint8_t i = 0; buffer[i] != '\0'; i++) {
        Font_DrawChar(current_x, y_pos, buffer[i]);
        current_x += 6;
        if (current_x >= SH1107_WIDTH - 5) break; // Prevent text wrapping overflow
    }
}

void DebugUI_ProgressBar(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t percentage) {
    if (percentage > 100) percentage = 100;

    // Draw Frame
    for (uint8_t i = 0; i < width; i++) {
        SH1107_DrawPixel(x + i, y, 1);
        SH1107_DrawPixel(x + i, y + height - 1, 1);
    }
    for (uint8_t j = 0; j < height; j++) {
        SH1107_DrawPixel(x, y + j, 1);
        SH1107_DrawPixel(x + width - 1, y + j, 1);
    }

    // Draw Fill
    uint8_t inner_width = width > 4 ? width - 4 : 0;
    uint8_t fill_width = (inner_width * percentage) / 100;
    uint8_t inner_height = height > 4 ? height - 4 : 0;

    for (uint8_t i = 0; i < fill_width; i++) {
        for (uint8_t j = 0; j < inner_height; j++) {
            SH1107_DrawPixel(x + 2 + i, y + 2 + j, 1);
        }
    }
}

void DebugUI_StickSlider(uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint16_t value) {
    if (value < 1000) value = 1000;
    if (value > 2000) value = 2000;

    for (uint8_t i = 0; i < width; i++) {
        SH1107_DrawPixel(x + i, y, 1);
        SH1107_DrawPixel(x + i, y + height - 1, 1);
    }
    for (uint8_t j = 0; j < height; j++) {
        SH1107_DrawPixel(x, y + j, 1);
        SH1107_DrawPixel(x + width - 1, y + j, 1);
    }

    uint8_t center_x = x + (width / 2);
    for (uint8_t j = 0; j < height; j++) {
        if (j % 2 == 0) SH1107_DrawPixel(center_x, y + j, 1);
    }

    uint8_t inner_width = width - 2;
    uint8_t stick_px = x + 1 + ((uint32_t)(value - 1000) * inner_width) / 1000;

    for (int8_t i = -1; i <= 1; i++) {
        int16_t draw_x = stick_px + i;
        if (draw_x > x && draw_x < x + width - 1) {
            for (uint8_t j = 1; j < height - 1; j++) {
                SH1107_DrawPixel((uint8_t)draw_x, y + j, 1);
            }
        }
    }
}
