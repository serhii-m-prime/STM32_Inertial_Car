#include "sh1107.h"
#include "spi.h"   // hspi1
#include "gpio.h"  // pin definitions

/* Hardware control macros based on pinout */
#define SH1107_CS_LOW()   HAL_GPIO_WritePin(SCREEN_CS_GPIO_Port, SCREEN_CS_Pin, GPIO_PIN_RESET)
#define SH1107_CS_HIGH()  HAL_GPIO_WritePin(SCREEN_CS_GPIO_Port, SCREEN_CS_Pin, GPIO_PIN_SET)
#define SH1107_DC_CMD()   HAL_GPIO_WritePin(SCREEN_DC_GPIO_Port, SCREEN_DC_Pin, GPIO_PIN_RESET)
#define SH1107_DC_DATA()  HAL_GPIO_WritePin(SCREEN_DC_GPIO_Port, SCREEN_DC_Pin, GPIO_PIN_SET)
#define SH1107_RST_LOW()  HAL_GPIO_WritePin(SCREEN_RST_GPIO_Port, SCREEN_RST_Pin, GPIO_PIN_RESET)
#define SH1107_RST_HIGH() HAL_GPIO_WritePin(SCREEN_RST_GPIO_Port, SCREEN_RST_Pin, GPIO_PIN_SET)

/* The shadow buffer in MCU RAM */
uint8_t sh1107_buffer[SH1107_BUFFER_SIZE];

/* Send a single command byte  */
static void SH1107_WriteCommand(uint8_t cmd) {
    SH1107_CS_LOW();
    SH1107_DC_CMD();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    SH1107_CS_HIGH();
}

void SH1107_Init(void) {
    // 1. Hardware Reset
    SH1107_RST_LOW();
    HAL_Delay(50);
    SH1107_RST_HIGH();
    HAL_Delay(50);

    // 2. Init sequence for 128x128 SH1107
    SH1107_WriteCommand(0xAE); // Display OFF
    SH1107_WriteCommand(0x00); // Set lower column address
    SH1107_WriteCommand(0x10); // Set higher column address
    SH1107_WriteCommand(0x20); // Set memory addressing mode
    SH1107_WriteCommand(0x81); // Contrast control
    SH1107_WriteCommand(0x80); // Default contrast
    SH1107_WriteCommand(0xA0); // Segment remap
    SH1107_WriteCommand(0xC0); // Common output scan direction
    SH1107_WriteCommand(0xA8); // Multiplex ratio
    SH1107_WriteCommand(0x7F); // 128 lines
    SH1107_WriteCommand(0xD3); // Display offset
    SH1107_WriteCommand(0x00); // No offset
    SH1107_WriteCommand(0xD5); // Display clock divide ratio
    SH1107_WriteCommand(0x50);
    SH1107_WriteCommand(0xD9); // Pre-charge period
    SH1107_WriteCommand(0x22);
    SH1107_WriteCommand(0xDB); // VCOMH deselect level
    SH1107_WriteCommand(0x35);
    SH1107_WriteCommand(0xAF); // Display ON
}

void SH1107_DrawPixel(uint8_t x, uint8_t y, uint8_t color) {
    if (x >= SH1107_WIDTH || y >= SH1107_HEIGHT) return;

    // Page addressing: y / 8 gives the page, x gives the column
    uint16_t index = x + (y / 8) * SH1107_WIDTH;
    uint8_t bit = y % 8;

    if (color) sh1107_buffer[index] |= (1 << bit);
    else       sh1107_buffer[index] &= ~(1 << bit);
}

void SH1107_UpdateScreen(void) {
    // SH1107 uses page addressing. We must send 16 pages, 128 bytes each.
    for (uint8_t page = 0; page < 16; page++) {
        SH1107_WriteCommand(0xB0 + page); // Set page address
        SH1107_WriteCommand(0x00);        // Lower col
        SH1107_WriteCommand(0x10);        // Higher col

        SH1107_CS_LOW();
        SH1107_DC_DATA();

        // Transmit 1 page (128 bytes) via DMA
        HAL_SPI_Transmit_DMA(&hspi1, &sh1107_buffer[page * 128], 128);

        // Wait for DMA to complete this page before sending next command
        while (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {}

        SH1107_CS_HIGH();
    }
}
