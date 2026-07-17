#ifndef VEHICLE_CONFIG_H
#define VEHICLE_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#define VEHICLE_CONFIG_MAGIC   0x43415231UL
#define VEHICLE_CONFIG_VERSION 2U

typedef struct {
    uint32_t magic; // narker to identify valid config data
    uint16_t version;
    uint16_t size;

    uint16_t servo_center_us;
    uint16_t servo_min_us;
    uint16_t servo_max_us;
    uint8_t steering_rate_pct;
    uint8_t steering_reverse;

    uint8_t forward_power_pct;
    uint8_t reverse_power_pct;
    uint8_t left_motor_pct;
    uint8_t right_motor_pct;

    uint16_t steering_deadband_us;
    uint16_t throttle_deadband_us;
    uint8_t steering_filter_pct;
    uint8_t throttle_filter_pct;

    uint16_t calibration_max_jitter_us;

    uint16_t switch_low_us;
    uint16_t switch_high_us;
    uint16_t failsafe_timeout_ms;

    uint32_t crc32;
} VehicleConfig_t;

typedef enum {
    CONFIG_STATUS_OK = 0,
    CONFIG_STATUS_EEPROM_NOT_FOUND,
    CONFIG_STATUS_READ_ERROR,
    CONFIG_STATUS_INVALID_DATA,
    CONFIG_STATUS_WRITE_ERROR,
    CONFIG_STATUS_VERIFY_ERROR
} ConfigStatus_t;

extern VehicleConfig_t vehicle_config;

void VehicleConfig_SetDefaults(void);
ConfigStatus_t VehicleConfig_Load(void);
ConfigStatus_t VehicleConfig_Save(void);
ConfigStatus_t VehicleConfig_GetLastStatus(void);
const char *VehicleConfig_StatusText(ConfigStatus_t status);
bool VehicleConfig_SetValue(const char *key, int32_t value,
                            char *error, uint16_t error_size);
void VehicleConfig_Format(char *buffer, uint16_t size);

#endif
