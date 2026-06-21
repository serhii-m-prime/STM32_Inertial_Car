#ifndef SH1107_H
#define SH1107_H

#include "stm32f4xx_hal.h"

#define SH1107_WIDTH  128
#define SH1107_HEIGHT 128
#define SH1107_BUFFER_SIZE (SH1107_WIDTH * SH1107_HEIGHT / 8)

/* Display FSM States */
typedef enum {
    SH1107_STATE_IDLE,       // Screen is free, ready for next frame
    SH1107_STATE_SEND_CMD,   // DMA is currently sending page addresses
    SH1107_STATE_SEND_DATA   // DMA is currently dumping 128 bytes of pixel data
} SH1107_State_t;

void SH1107_Init(void);
void SH1107_DrawPixel(uint8_t x, uint8_t y, uint8_t color);

/* non-blocking trigger */
void SH1107_UpdateScreen_NonBlocking(void);

/* Check if the FSM is busy rendering */
uint8_t SH1107_IsBusy(void);

/* This must be called from SPI DMA Transfer Complete Callback */
void SH1107_FSM_Step(void);

#endif // SH1107_H
