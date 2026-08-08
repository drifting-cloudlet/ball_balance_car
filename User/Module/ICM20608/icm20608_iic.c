#include "icm20608_iic.h"

#include <string.h>

#define ICM20608_I2C_TIMEOUT_MS 5U

extern I2C_HandleTypeDef hi2c1;

void Init_I2C(void)
{
}

void i2cWriteData(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length)
{
    (void)HAL_I2C_Mem_Write(&hi2c1,
                            addr << 1,
                            regAddr,
                            I2C_MEMADD_SIZE_8BIT,
                            data,
                            length,
                            ICM20608_I2C_TIMEOUT_MS);
}

uint8_t i2cRead(uint8_t addr, uint8_t regAddr)
{
    uint8_t data = 0U;

    (void)HAL_I2C_Mem_Read(&hi2c1,
                           addr << 1,
                           regAddr,
                           I2C_MEMADD_SIZE_8BIT,
                           &data,
                           1,
                           ICM20608_I2C_TIMEOUT_MS);

    return data;
}

void i2cWrite(uint8_t addr, uint8_t regAddr, uint8_t data)
{
    (void)HAL_I2C_Mem_Write(&hi2c1,
                            addr << 1,
                            regAddr,
                            I2C_MEMADD_SIZE_8BIT,
                            &data,
                            1,
                            ICM20608_I2C_TIMEOUT_MS);
}

void i2cReadData(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length)
{
    if (HAL_I2C_Mem_Read(&hi2c1,
                         addr << 1,
                         regAddr,
                         I2C_MEMADD_SIZE_8BIT,
                         data,
                         length,
                         ICM20608_I2C_TIMEOUT_MS) != HAL_OK)
    {
        memset(data, 0, length);
    }
}

void Single_WriteI2C(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t REG_data)
{
    i2cWrite(SlaveAddress, REG_Address, REG_data);
}

uint8_t Single_ReadI2C(uint8_t SlaveAddress, uint8_t REG_Address)
{
    return i2cRead(SlaveAddress, REG_Address);
}

int16_t Double_ReadI2C(uint8_t SlaveAddress, uint8_t REG_Address)
{
    uint8_t data[2] = {0U, 0U};

    i2cReadData(SlaveAddress, REG_Address, data, 2);
    return (int16_t)(((uint16_t)data[0] << 8) | data[1]);
}
