#ifndef CRSF_H
#define CRSF_H

#include <stdint.h>
#include <stdbool.h>

// Channel map
#define CRSF_MAP_STEERING    0
#define CRSF_MAP_THROTTLE    2
#define CRSF_MAP_DIR_SWITCH  6
#define CRSF_MAP_RTH_SWITCH  7

/* CRSF packs types */
#define CRSF_START_BYTE                0xC8
#define CRSF_FRAMETYPE_RC              0x16
#define CRSF_FRAMETYPE_LINK_STATISTICS 0x14

#define CRSF_FAILSAFE_TIME             10000

typedef struct {
    uint16_t raw_channels[16];
    uint32_t last_valid_frame_time;
    bool is_connected;

    uint8_t link_quality; // LQ
    int8_t  rssi_dbm;     // RSSI
} CRSF_State_t;

extern CRSF_State_t crsf;

void CRSF_Init(void);
bool CRSF_Update(void);
void CRSF_UpdateFailsafe(uint32_t current_time_ms);
uint16_t CRSF_GetPWM(uint8_t channel_index);

#endif // CRSF_H
