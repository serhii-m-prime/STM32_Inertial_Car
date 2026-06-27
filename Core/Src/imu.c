#include "imu.h"
#include "spi.h"

#define ICM20948_WHO_AM_I_REG 0x00
#define ICM20948_EXPECTED_ID  0xEA

/* Bit 7 is 1 for READ operations in SPI mode */
#define SPI_READ_FLAG         0x80

/* Select IMU by pulling PA4 LOW */
static void CS_Enable(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
}

/* Deselect IMU by pulling PA4 HIGH */
static void CS_Disable(void) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

uint8_t IMU_ReadReg(uint8_t reg_addr) {
    uint8_t tx_data[2];
    uint8_t rx_data[2] = {0, 0};

    /* Byte 0: Register address | READ flag */
    tx_data[0] = reg_addr | SPI_READ_FLAG;
    /* Byte 1: Dummy byte to clock the data in */
    tx_data[1] = 0x00;

    CS_Enable();
    /* Transmit address and dummy, simultaneously receive status and data */
    HAL_SPI_TransmitReceive(&hspi2, tx_data, rx_data, 2, 10);
    CS_Disable();

    /* rx_data[0] is the status during the address phase, rx_data[1] is the actual register value */
    return rx_data[1];
}

bool IMU_Init(void) {
    /* Ensure CS is high before starting */
    CS_Disable();
    HAL_Delay(100); /* Let the sensor power up completely */

    uint8_t id = IMU_ReadReg(ICM20948_WHO_AM_I_REG);

    if (id == ICM20948_EXPECTED_ID) {
        return true;
    }
    return false;
}

uint8_t IMU_GetWhoAmI(void) {
    return IMU_ReadReg(ICM20948_WHO_AM_I_REG);
}
