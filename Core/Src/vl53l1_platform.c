#include "vl53l1_platform.h"
#include "main.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c2;

#define VL53L1X_I2C_HANDLE hi2c2

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    HAL_StatusTypeDef hal_status;
    uint8_t buffer[258];  /* 2 bytes register + up to 256 bytes data */

    if (count > 256)
    {
        return 1;
    }

    buffer[0] = (uint8_t)(index >> 8);
    buffer[1] = (uint8_t)(index & 0xFF);
    memcpy(&buffer[2], pdata, count);

    hal_status = HAL_I2C_Master_Transmit(&VL53L1X_I2C_HANDLE,
                                          dev,
                                          buffer,
                                          count + 2,
                                          HAL_MAX_DELAY);

    return (hal_status == HAL_OK) ? 0 : 1;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count)
{
    HAL_StatusTypeDef hal_status;
    uint8_t reg[2];

    reg[0] = (uint8_t)(index >> 8);
    reg[1] = (uint8_t)(index & 0xFF);

    hal_status = HAL_I2C_Master_Transmit(&VL53L1X_I2C_HANDLE,
                                          dev,
                                          reg,
                                          2,
                                          HAL_MAX_DELAY);

    if (hal_status != HAL_OK)
    {
        return 1;
    }

    hal_status = HAL_I2C_Master_Receive(&VL53L1X_I2C_HANDLE,
                                         dev,
                                         pdata,
                                         count,
                                         HAL_MAX_DELAY);

    return (hal_status == HAL_OK) ? 0 : 1;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data)
{
    uint8_t buffer[3];

    buffer[0] = (uint8_t)(index >> 8);
    buffer[1] = (uint8_t)(index & 0xFF);
    buffer[2] = data;

    return (HAL_I2C_Master_Transmit(&VL53L1X_I2C_HANDLE,
                                    dev,
                                    buffer,
                                    3,
                                    HAL_MAX_DELAY) == HAL_OK) ? 0 : 1;
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data)
{
    uint8_t buffer[4];

    buffer[0] = (uint8_t)(index >> 8);
    buffer[1] = (uint8_t)(index & 0xFF);
    buffer[2] = (uint8_t)(data >> 8);
    buffer[3] = (uint8_t)(data & 0xFF);

    return (HAL_I2C_Master_Transmit(&VL53L1X_I2C_HANDLE,
                                    dev,
                                    buffer,
                                    4,
                                    HAL_MAX_DELAY) == HAL_OK) ? 0 : 1;
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data)
{
    uint8_t buffer[6];

    buffer[0] = (uint8_t)(index >> 8);
    buffer[1] = (uint8_t)(index & 0xFF);
    buffer[2] = (uint8_t)(data >> 24);
    buffer[3] = (uint8_t)(data >> 16);
    buffer[4] = (uint8_t)(data >> 8);
    buffer[5] = (uint8_t)(data & 0xFF);

    return (HAL_I2C_Master_Transmit(&VL53L1X_I2C_HANDLE,
                                    dev,
                                    buffer,
                                    6,
                                    HAL_MAX_DELAY) == HAL_OK) ? 0 : 1;
}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data)
{
    return VL53L1_ReadMulti(dev, index, data, 1);
}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data)
{
    uint8_t buffer[2];
    int8_t status = VL53L1_ReadMulti(dev, index, buffer, 2);

    if (status == 0)
    {
        *data = ((uint16_t)buffer[0] << 8) | buffer[1];
    }

    return status;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data)
{
    uint8_t buffer[4];
    int8_t status = VL53L1_ReadMulti(dev, index, buffer, 4);

    if (status == 0)
    {
        *data = ((uint32_t)buffer[0] << 24) |
                ((uint32_t)buffer[1] << 16) |
                ((uint32_t)buffer[2] << 8)  |
                buffer[3];
    }

    return status;
}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms)
{
    (void)dev;
    HAL_Delay((uint32_t)wait_ms);
    return 0;
}
