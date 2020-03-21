/*
 * I2C_driver.h
 *
 *  Created on: Mar 10, 2020
 *      Author: Dave
 */

#ifndef I2C_DRIVER_H_
#define I2C_DRIVER_H_
#include <F2837xD_Device.h>

void I2C_Master_Init(uint16_t slaveAddress, float sysClkMhz, float I2CClkKHz);

void I2C_SendByte(char slaveAddr, char regAddr, char dataVal);
void I2C_SendBytes(char slaveAddr, char regAddr, char *dataVals, uint16_t bytesToWrite);
uint32_t I2C_ReadByte(char slaveAddr, char regAddr);
uint32_t I2C_ReadBytes(char slaveAddr, char regAddr, char *dataVals, uint16_t bytesToRead);

#endif /* I2C_DRIVER_H_ */
