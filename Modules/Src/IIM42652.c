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


uint8_t IIM42652_Init( IIM42652 *dev, I2C_HandleTypeDef *i2cHandle )
{

	/* Initialise structure parameters */
	dev->i2cHandle = i2cHandle;

	dev->acc[0] = 0.0f;
	dev->acc[1] = 0.0f;
	dev->acc[2] = 0.0f;

	dev->gyr[0] = 0.0f;
	dev->gyr[1] = 0.0f;
	dev->gyr[2] = 0.0f;

	dev->temp_c = 0.0f;

	/* I2C transaction errors */
	uint8_t errNum = 0;
	HAL_StatusTypeDef status;

	/* Check for device ID */
	uint8_t regDataID;

	status = IIM42652_ReadRegister(dev, WHO_AM_I_ADD, &regDataID);
	errNum += ( status != HAL_OK );

	/* If ID not equal to IIM42652_I2C_ID, Init returns with an error code */
	if( regDataID != IIM42652_I2C_ID ) return HAL_ERROR;


	return 0;
}











