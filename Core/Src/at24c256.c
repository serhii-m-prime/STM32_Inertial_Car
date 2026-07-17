#include "at24c256.h"
#include "i2c.h"

#define AT24C256_PAGE_SIZE       64U
#define AT24C256_IO_TIMEOUT_MS   100U
#define AT24C256_WRITE_CYCLE_TIMEOUT_MS 20U

static const char *last_stage = "idle";
static HAL_StatusTypeDef last_hal_status = HAL_OK;
static uint32_t last_i2c_error = HAL_I2C_ERROR_NONE;

static bool CaptureResult(const char *stage, HAL_StatusTypeDef status)
{
    last_stage = stage;
    last_hal_status = status;
    last_i2c_error = HAL_I2C_GetError(&hi2c1);
    return status == HAL_OK;
}

bool AT24C256_IsReady(void)
{
    uint32_t started = HAL_GetTick();
    HAL_StatusTypeDef status;

    do {
        status = HAL_I2C_IsDeviceReady(&hi2c1, AT24C256_I2C_ADDRESS,
                                       1U, AT24C256_IO_TIMEOUT_MS);
        if (status == HAL_OK) return CaptureResult("ready", HAL_OK);
        HAL_Delay(1U);
    } while ((HAL_GetTick() - started) < AT24C256_WRITE_CYCLE_TIMEOUT_MS);

    return CaptureResult("ready", status);
}

bool AT24C256_Read(uint16_t address, void *data, uint16_t length)
{
    if (data == NULL || length == 0U) return false;
    return CaptureResult("read", HAL_I2C_Mem_Read(
            &hi2c1, AT24C256_I2C_ADDRESS, address, I2C_MEMADD_SIZE_16BIT,
            (uint8_t *)data, length, AT24C256_IO_TIMEOUT_MS));
}

bool AT24C256_Write(uint16_t address, const void *data, uint16_t length)
{
    const uint8_t *source = (const uint8_t *)data;
    if (source == NULL || length == 0U) return false;

    while (length > 0U) {
        uint16_t page_space = AT24C256_PAGE_SIZE - (address % AT24C256_PAGE_SIZE);
        uint16_t chunk = (length < page_space) ? length : page_space;

        if (!CaptureResult("write", HAL_I2C_Mem_Write(
                &hi2c1, AT24C256_I2C_ADDRESS, address,
                I2C_MEMADD_SIZE_16BIT, (uint8_t *)source, chunk,
                AT24C256_IO_TIMEOUT_MS))) return false;

        if (!AT24C256_IsReady()) return false;
        address += chunk;
        source += chunk;
        length -= chunk;
    }
    return true;
}

const char *AT24C256_GetLastStage(void) { return last_stage; }
HAL_StatusTypeDef AT24C256_GetLastHALStatus(void) { return last_hal_status; }
uint32_t AT24C256_GetLastI2CError(void) { return last_i2c_error; }
HAL_I2C_StateTypeDef AT24C256_GetI2CState(void) { return HAL_I2C_GetState(&hi2c1); }
