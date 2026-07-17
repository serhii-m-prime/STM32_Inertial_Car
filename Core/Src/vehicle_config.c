#include "vehicle_config.h"
#include "at24c256.h"
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#define CONFIG_EEPROM_ADDRESS 0x0000U

VehicleConfig_t vehicle_config;
static ConfigStatus_t last_status = CONFIG_STATUS_INVALID_DATA;

static uint32_t CRC32_Calculate(const uint8_t *data, uint16_t length)
{
    uint32_t crc = 0xFFFFFFFFUL;
    while (length--) {
        crc ^= *data++;
        for (uint8_t bit = 0; bit < 8U; bit++) {
            crc = (crc >> 1) ^ ((crc & 1U) ? 0xEDB88320UL : 0U);
        }
    }
    return ~crc;
}

static uint32_t ConfigCRC(const VehicleConfig_t *config)
{
    return CRC32_Calculate((const uint8_t *)config,
                           (uint16_t)offsetof(VehicleConfig_t, crc32));
}

static bool InRangeU16(uint16_t value, uint16_t min, uint16_t max)
{
    return value >= min && value <= max;
}

static bool ConfigIsValid(const VehicleConfig_t *c)
{
    return c->magic == VEHICLE_CONFIG_MAGIC &&
           c->version == VEHICLE_CONFIG_VERSION &&
           c->size == sizeof(VehicleConfig_t) &&
           c->crc32 == ConfigCRC(c) &&
           InRangeU16(c->servo_center_us, 1000, 2000) &&
           InRangeU16(c->servo_min_us, 500, 1500) &&
           InRangeU16(c->servo_max_us, 1500, 2500) &&
           c->servo_min_us < c->servo_center_us &&
           c->servo_center_us < c->servo_max_us &&
           c->steering_rate_pct >= 20U && c->steering_rate_pct <= 150U &&
           c->steering_reverse <= 1U &&
           c->forward_power_pct <= 100U && c->reverse_power_pct <= 100U &&
           c->left_motor_pct >= 50U && c->left_motor_pct <= 100U &&
           c->right_motor_pct >= 50U && c->right_motor_pct <= 100U &&
           c->steering_deadband_us <= 100U && c->throttle_deadband_us <= 100U &&
           c->steering_filter_pct >= 1U && c->steering_filter_pct <= 100U &&
           c->throttle_filter_pct >= 1U && c->throttle_filter_pct <= 100U &&
           c->calibration_max_jitter_us <= 100U &&
           InRangeU16(c->switch_low_us, 1000, 1499) &&
           InRangeU16(c->switch_high_us, 1501, 2000) &&
           InRangeU16(c->failsafe_timeout_ms, 100, 3000);
}

void VehicleConfig_SetDefaults(void)
{
    memset(&vehicle_config, 0, sizeof(vehicle_config));
    vehicle_config.magic = VEHICLE_CONFIG_MAGIC;
    vehicle_config.version = VEHICLE_CONFIG_VERSION;
    vehicle_config.size = sizeof(VehicleConfig_t);
    vehicle_config.servo_center_us = 1500;
    vehicle_config.servo_min_us = 800;
    vehicle_config.servo_max_us = 2200;
    vehicle_config.steering_rate_pct = 140;
    vehicle_config.steering_reverse = 0;
    vehicle_config.forward_power_pct = 100;
    vehicle_config.reverse_power_pct = 100;
    vehicle_config.left_motor_pct = 100;
    vehicle_config.right_motor_pct = 100;
    vehicle_config.steering_deadband_us = 15;
    vehicle_config.throttle_deadband_us = 15;
    vehicle_config.steering_filter_pct = 15;
    vehicle_config.throttle_filter_pct = 25;
    vehicle_config.calibration_max_jitter_us = 30;
    vehicle_config.switch_low_us = 1300;
    vehicle_config.switch_high_us = 1700;
    vehicle_config.failsafe_timeout_ms = 500;
    vehicle_config.crc32 = ConfigCRC(&vehicle_config);
}

ConfigStatus_t VehicleConfig_Load(void)
{
    VehicleConfig_t loaded;
    if (!AT24C256_IsReady()) last_status = CONFIG_STATUS_EEPROM_NOT_FOUND;
    else if (!AT24C256_Read(CONFIG_EEPROM_ADDRESS, &loaded, sizeof(loaded)))
        last_status = CONFIG_STATUS_READ_ERROR;
    else if (!ConfigIsValid(&loaded)) last_status = CONFIG_STATUS_INVALID_DATA;
    else {
        vehicle_config = loaded;
        last_status = CONFIG_STATUS_OK;
        return last_status;
    }
    VehicleConfig_SetDefaults();
    return last_status;
}

ConfigStatus_t VehicleConfig_Save(void)
{
    VehicleConfig_t verify;
    vehicle_config.magic = VEHICLE_CONFIG_MAGIC;
    vehicle_config.version = VEHICLE_CONFIG_VERSION;
    vehicle_config.size = sizeof(VehicleConfig_t);
    vehicle_config.crc32 = ConfigCRC(&vehicle_config);

    if (!ConfigIsValid(&vehicle_config)) return last_status = CONFIG_STATUS_INVALID_DATA;
    if (!AT24C256_IsReady()) return last_status = CONFIG_STATUS_EEPROM_NOT_FOUND;
    if (!AT24C256_Write(CONFIG_EEPROM_ADDRESS, &vehicle_config, sizeof(vehicle_config)))
        return last_status = CONFIG_STATUS_WRITE_ERROR;
    if (!AT24C256_Read(CONFIG_EEPROM_ADDRESS, &verify, sizeof(verify)) ||
        memcmp(&verify, &vehicle_config, sizeof(verify)) != 0)
        return last_status = CONFIG_STATUS_VERIFY_ERROR;
    return last_status = CONFIG_STATUS_OK;
}

ConfigStatus_t VehicleConfig_GetLastStatus(void) { return last_status; }

const char *VehicleConfig_StatusText(ConfigStatus_t status)
{
    switch (status) {
        case CONFIG_STATUS_OK: return "OK";
        case CONFIG_STATUS_EEPROM_NOT_FOUND: return "EEPROM_NOT_FOUND";
        case CONFIG_STATUS_READ_ERROR: return "EEPROM_READ_ERROR";
        case CONFIG_STATUS_INVALID_DATA: return "INVALID_CONFIG_OR_CRC";
        case CONFIG_STATUS_WRITE_ERROR: return "EEPROM_WRITE_ERROR";
        case CONFIG_STATUS_VERIFY_ERROR: return "EEPROM_VERIFY_ERROR";
        default: return "UNKNOWN_ERROR";
    }
}

static bool SetChecked(const char *key, int32_t value, int32_t min, int32_t max,
                       void *field, uint8_t bytes, char *error, uint16_t error_size)
{
    if (value < min || value > max) {
        snprintf(error, error_size, "%s range %ld..%ld", key, (long)min, (long)max);
        return false;
    }
    if (bytes == 1U) *(uint8_t *)field = (uint8_t)value;
    else *(uint16_t *)field = (uint16_t)value;
    return true;
}

#define SET_U8(name, field, min, max) if (strcmp(key, name) == 0) return SetChecked(key, value, min, max, &vehicle_config.field, 1, error, error_size)
#define SET_U16(name, field, min, max) if (strcmp(key, name) == 0) return SetChecked(key, value, min, max, &vehicle_config.field, 2, error, error_size)

bool VehicleConfig_SetValue(const char *key, int32_t value, char *error, uint16_t error_size)
{
    SET_U16("servo_center", servo_center_us, 1000, 2000);
    SET_U16("servo_min", servo_min_us, 500, 1500);
    SET_U16("servo_max", servo_max_us, 1500, 2500);
    SET_U8("steering_rate", steering_rate_pct, 20, 150);
    SET_U8("steering_reverse", steering_reverse, 0, 1);
    SET_U8("forward_power", forward_power_pct, 0, 100);
    SET_U8("reverse_power", reverse_power_pct, 0, 100);
    SET_U8("left_motor", left_motor_pct, 50, 100);
    SET_U8("right_motor", right_motor_pct, 50, 100);
    SET_U16("steering_deadband", steering_deadband_us, 0, 100);
    SET_U16("throttle_deadband", throttle_deadband_us, 0, 100);
    SET_U8("steering_filter", steering_filter_pct, 1, 100);
    SET_U8("throttle_filter", throttle_filter_pct, 1, 100);
    SET_U16("calibration_jitter", calibration_max_jitter_us, 0, 100);
    SET_U16("switch_low", switch_low_us, 1000, 1499);
    SET_U16("switch_high", switch_high_us, 1501, 2000);
    SET_U16("failsafe_ms", failsafe_timeout_ms, 100, 3000);
    snprintf(error, error_size, "unknown key: %s", key);
    return false;
}

void VehicleConfig_Format(char *b, uint16_t n)
{
    snprintf(b, n,
      "servo_center=%u\r\nservo_min=%u\r\nservo_max=%u\r\nsteering_rate=%u\r\nsteering_reverse=%u\r\n"
      "forward_power=%u\r\nreverse_power=%u\r\nleft_motor=%u\r\nright_motor=%u\r\n"
      "steering_deadband=%u\r\nthrottle_deadband=%u\r\nsteering_filter=%u\r\nthrottle_filter=%u\r\n"
      "calibration_jitter=%u\r\n"
      "switch_low=%u\r\nswitch_high=%u\r\nfailsafe_ms=%u\r\n",
      vehicle_config.servo_center_us, vehicle_config.servo_min_us, vehicle_config.servo_max_us,
      vehicle_config.steering_rate_pct, vehicle_config.steering_reverse,
      vehicle_config.forward_power_pct, vehicle_config.reverse_power_pct,
      vehicle_config.left_motor_pct, vehicle_config.right_motor_pct,
      vehicle_config.steering_deadband_us, vehicle_config.throttle_deadband_us,
      vehicle_config.steering_filter_pct, vehicle_config.throttle_filter_pct,
      vehicle_config.calibration_max_jitter_us, vehicle_config.switch_low_us,
      vehicle_config.switch_high_us, vehicle_config.failsafe_timeout_ms);
}
