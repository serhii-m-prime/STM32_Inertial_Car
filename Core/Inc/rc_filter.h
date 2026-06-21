#ifndef RC_FILTER_H
#define RC_FILTER_H

#include <stdint.h>
#include <stdbool.h>

/* FSM States */
typedef enum {
    RC_STATE_WAITING,
    RC_STATE_CALIBRATING,
    RC_STATE_ERROR,
    RC_STATE_READY
} RC_SystemState_t;

typedef struct {
    uint16_t center;
    uint16_t deadband;
    float alpha;            // EMA coefficient (0.01 - 0.99)
    float filtered_val;
} RC_Channel_Config_t;

extern RC_SystemState_t rc_sys_state;

void RC_Filter_Init(void);
void RC_ProcessCalibration(void);
uint16_t RC_GetProcessed(uint8_t channel_index);

#endif // RC_FILTER_H
