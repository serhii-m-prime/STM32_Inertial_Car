#include "sh1107.h"
#include "spi.h"   // Access to hspi1
#include "gpio.h"  // Access to pin macros


// Macro optimization / get read HAL for pin interaction
//#define SH1107_CS_LOW()   HAL_GPIO_WritePin(SCREEN_CS_GPIO_Port, SCREEN_CS_Pin, GPIO_PIN_RESET)
//#define SH1107_CS_HIGH()  HAL_GPIO_WritePin(SCREEN_CS_GPIO_Port, SCREEN_CS_Pin, GPIO_PIN_SET)
//#define SH1107_DC_CMD()   HAL_GPIO_WritePin(SCREEN_DC_GPIO_Port, SCREEN_DC_Pin, GPIO_PIN_RESET)
//#define SH1107_DC_DATA()  HAL_GPIO_WritePin(SCREEN_DC_GPIO_Port, SCREEN_DC_Pin, GPIO_PIN_SET)
//#define SH1107_RST_LOW()  HAL_GPIO_WritePin(SCREEN_RST_GPIO_Port, SCREEN_RST_Pin, GPIO_PIN_RESET)
//#define SH1107_RST_HIGH() HAL_GPIO_WritePin(SCREEN_RST_GPIO_Port, SCREEN_RST_Pin, GPIO_PIN_SET)

#define SH1107_CS_LOW()   (SCREEN_CS_GPIO_Port->BSRR = (uint32_t)SCREEN_CS_Pin << 16)
#define SH1107_CS_HIGH()  (SCREEN_CS_GPIO_Port->BSRR = SCREEN_CS_Pin)
#define SH1107_DC_CMD()   (SCREEN_DC_GPIO_Port->BSRR = (uint32_t)SCREEN_DC_Pin << 16)
#define SH1107_DC_DATA()  (SCREEN_DC_GPIO_Port->BSRR = SCREEN_DC_Pin)
#define SH1107_RST_LOW()  (SCREEN_RST_GPIO_Port->BSRR = (uint32_t)SCREEN_RST_Pin << 16)
#define SH1107_RST_HIGH() (SCREEN_RST_GPIO_Port->BSRR = SCREEN_RST_Pin)

uint8_t sh1107_buffer[SH1107_BUFFER_SIZE];

/* FSM Internal variables */
static volatile SH1107_State_t oled_state = SH1107_STATE_IDLE;
static volatile uint8_t oled_current_page = 0;
static uint8_t oled_cmd_buffer[3]; // Temporary buffer for non-blocking command transmission

/* Blocking write for initialization only */
static void SH1107_WriteCommand_Blocking(uint8_t cmd) {
    SH1107_CS_LOW();
    SH1107_DC_CMD();
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);
    SH1107_CS_HIGH();
}

void SH1107_Init(void) {
    SH1107_RST_LOW();
    HAL_Delay(50);
    SH1107_RST_HIGH();
    HAL_Delay(50);

    // Initial hardware config
    SH1107_WriteCommand_Blocking(0xAE); // Display OFF
    SH1107_WriteCommand_Blocking(0x00); // Set lower column address
    SH1107_WriteCommand_Blocking(0x10); // Set higher column address
    SH1107_WriteCommand_Blocking(0x20); // Set memory addressing mode
    SH1107_WriteCommand_Blocking(0x81); // Contrast control
    SH1107_WriteCommand_Blocking(0x80); // Default
    SH1107_WriteCommand_Blocking(0xA0); // Segment remap
    SH1107_WriteCommand_Blocking(0xC0); // Common output scan direction
    SH1107_WriteCommand_Blocking(0xA8); // Multiplex ratio
    SH1107_WriteCommand_Blocking(0x7F); // 128 lines
    SH1107_WriteCommand_Blocking(0xD3); // Display offset
    SH1107_WriteCommand_Blocking(0x00);
    SH1107_WriteCommand_Blocking(0xD5); // Clock divide
    SH1107_WriteCommand_Blocking(0x50);
    SH1107_WriteCommand_Blocking(0xD9); // Pre-charge
    SH1107_WriteCommand_Blocking(0x22);
    SH1107_WriteCommand_Blocking(0xDB); // VCOMH
    SH1107_WriteCommand_Blocking(0x35);
    SH1107_WriteCommand_Blocking(0xAF); // Display ON
}

void SH1107_DrawPixel(uint8_t x, uint8_t y, uint8_t color) {
    if (x >= SH1107_WIDTH || y >= SH1107_HEIGHT) return; // over limits
//  uint16_t index = x + (y / 8) * SH1107_WIDTH;
//  Macro optimization only for  SH1107_WIDTH === 128
    uint16_t index = x + ((y >> 3) << 7);
    uint8_t bit = y & 7;
    if (color) sh1107_buffer[index] |= (1 << bit);
    else       sh1107_buffer[index] &= ~(1 << bit);
}

uint8_t SH1107_IsBusy(void) {
    return oled_state != SH1107_STATE_IDLE;
}

/* Kicks off the non-blocking rendering engine */
void SH1107_UpdateScreen_NonBlocking(void) {
    if (oled_state != SH1107_STATE_IDLE) {
        return;
    }

    oled_current_page = 0;
    oled_state = SH1107_STATE_SEND_CMD;

    SH1107_CS_LOW(); // Keep CS low for the duration of the entire frame transmission

    // Pack 3 bytes of command to tell SH1107 which page we are updating
    oled_cmd_buffer[0] = 0xB0 + oled_current_page; // Set Page Address
    oled_cmd_buffer[1] = 0x00;                     // Set Lower Column Address
    oled_cmd_buffer[2] = 0x10;                     // Set Higher Column Address

    SH1107_DC_CMD(); // Pin low -> Command mode

    // Send commands asynchronously. When done, HAL will trigger TxCpltCallback
    HAL_SPI_Transmit_DMA(&hspi1, oled_cmd_buffer, 3);
}

/* The actual engine room of the FSM, driven entirely by hardware interrupts */
void SH1107_FSM_Step(void) {
    if (oled_state == SH1107_STATE_SEND_CMD) {
        // Step 1 done: Commands sent successfully n switch to data mode and send the page's 128 bytes
        oled_state = SH1107_STATE_SEND_DATA;
        SH1107_DC_DATA(); // Pin high -> Data mode

        // Asynchronously dump 128 vertical pixel slices of the current page
        HAL_SPI_Transmit_DMA(&hspi1, &sh1107_buffer[oled_current_page * 128], 128);
    }
    else if (oled_state == SH1107_STATE_SEND_DATA) {
        // Step 2 done: Data for the current page sent move to the next page index
        oled_current_page++;

        if (oled_current_page < 16) {
            // We have more pages to process. Loop back to sending commands for the new page index
            oled_state = SH1107_STATE_SEND_CMD;
            oled_cmd_buffer[0] = 0xB0 + oled_current_page; // Set Page Address
            oled_cmd_buffer[1] = 0x00;                     // Set Lower Column Address
            oled_cmd_buffer[2] = 0x10;                     // Set Higher Column Address

            SH1107_DC_CMD();
            HAL_SPI_Transmit_DMA(&hspi1, oled_cmd_buffer, 3);
        } else {
            // All 16 pages (128x128 pixels total) have been transmitted. Clean up and close the transaction
            SH1107_CS_HIGH();
            oled_state = SH1107_STATE_IDLE; // FSM is now free for next render request
        }
    }
}
