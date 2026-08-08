#ifndef __ICM20608_IIC_H__
#define __ICM20608_IIC_H__

#include "stm32f4xx_hal.h"

void i2cWriteData(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length);
uint8_t i2cRead(uint8_t addr, uint8_t regAddr);
void i2cWrite(uint8_t addr, uint8_t regAddr, uint8_t data);
void i2cReadData(uint8_t addr, uint8_t regAddr, uint8_t *data, uint8_t length);
void Single_WriteI2C(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t REG_data);
uint8_t Single_ReadI2C(uint8_t SlaveAddress, uint8_t REG_Address);
int16_t Double_ReadI2C(uint8_t SlaveAddress, uint8_t REG_Address);

#endif

