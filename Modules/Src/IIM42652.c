/*
 * IIM42652.c
 *
 * 	Driver for I2C 6 axis IMU sensor (IIM-42652)
 *
 *  Created on: Oct 20, 2021
 *      Author: Santiago Macario
 */


#include "IIM42652.h"
#include <stdint.h>





HAL_StatusTypeDef IIM42652_ReadRegister( IIM42652 *dev, uint8_t reg, uint8_t *data )
{
	return HAL_I2C_Mem_Read( dev->i2cHandle, IIM42652_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}


HAL_StatusTypeDef IIM42652_ReadMultipleRegisters( IIM42652 *dev, uint8_t reg, uint8_t *data, uint8_t length )
{
	return HAL_I2C_Mem_Read( dev->i2cHandle, IIM42652_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}


HAL_StatusTypeDef IIM42652_WriteRegister( IIM42652 *dev, uint8_t reg, uint8_t *data )
{
	return HAL_I2C_Mem_Write( dev->i2cHandle, IIM42652_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}
