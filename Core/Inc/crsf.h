#ifndef CRSF_H
#define CRSF_H

#include <stdint.h>
#include <stdbool.h>

/* Мапінг каналів */
#define CRSF_MAP_STEERING    0
#define CRSF_MAP_THROTTLE    1
#define CRSF_MAP_DIR_SWITCH  4
#define CRSF_MAP_RTH_SWITCH  5

/* Типи пакетів протоколу CRSF */
#define CRSF_SYNC_BYTE                 0xC8
#define CRSF_FRAMETYPE_RC              0x16
#define CRSF_FRAMETYPE_LINK_STATISTICS 0x14

#define CRSF_FAILSAFE_TIME             300 // мс до повної зупинки

typedef struct {
    uint16_t raw_channels[16];
    uint32_t last_packet_time;
    bool is_connected;           // Залишаємо як прапорець апаратного Failsafe

    // --- НОВІ МЕТРИКИ ЗВ'ЯЗКУ ---
    uint8_t link_quality;        // Відсоток успішних пакетів (0 - 100%)
    int8_t  rssi_dbm;            // Потужність сигналу (наприклад, -85 dBm)
} CRSF_State_t;

extern CRSF_State_t crsf;

void CRSF_Init(void);
void CRSF_Update(void);
void CRSF_UpdateFailsafe(uint32_t current_time_ms);
uint16_t CRSF_GetPWM(uint8_t channel_index);

#endif // CRSF_H
