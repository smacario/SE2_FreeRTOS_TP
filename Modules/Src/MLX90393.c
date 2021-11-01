/*
 * MLX90393.c
 *
 * 	Driver for I2C 3 axis Magnetometer (MLX90393)
 *
 *  Created on: Oct 26, 2021
 *      Author: Santiago Macario
 */

#include "MLX90393.h"
#include <stdint.h>



HAL_StatusTypeDef MLX90393_ReadRegisterWord( MLX90393 *dev, uint8_t reg, uint8_t *data )
{
	return HAL_I2C_Mem_Read( dev->i2cHandle, MLX90393_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY );
}


HAL_StatusTypeDef MLX90393_ReadMultipleRegisters( MLX90393 *dev, uint8_t reg, uint8_t *data, uint8_t length )
{
	return HAL_I2C_Mem_Read( dev->i2cHandle, MLX90393_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY );
}


HAL_StatusTypeDef MLX90393_WriteRegister( MLX90393 *dev, uint8_t reg, uint8_t *data )
{
	return HAL_I2C_Mem_Write( dev->i2cHandle, MLX90393_I2C_ADDRESS, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY );
}


/* EXit command, returns the status */
uint8_t MLX90393_EX	( MLX90393 *dev)
{
	uint8_t status;
	HAL_StatusTypeDef i2c_status = HAL_OK;

	i2c_status = sendI2C( dev->i2cHandle, &status, (uint8_t*)MLX90393_REG_EX, 1, 1 );

	/* Checks the HAL status of i2c transaction */
	if(i2c_status != HAL_OK) return MLX90393_STATUS_ERROR;
	else{
		return status;
	}
}


/* Start Burst command, zyxt determines which axis to measure */
uint8_t MLX90393_SB	( MLX90393 *dev, uint8_t zyxt )
{
	uint8_t status;
	HAL_StatusTypeDef i2c_status = HAL_OK;

	/* Construct the SB command with axis from function parameters */
	uint8_t command = MLX90393_REG_SB | ( zyxt & 0x0F );

	i2c_status = sendI2C( dev->i2cHandle, &status, &command, 1, 1 );

	/* Checks the HAL status of i2c transaction */
	if(i2c_status != HAL_OK) return MLX90393_STATUS_ERROR;
	else{
		return status;
	}
}


/* Start Wake-up On Change command, zyxt determines which axis to measure */
uint8_t MLX90393_SWOC ( MLX90393 *dev, uint8_t zyxt )
{
	uint8_t status;
	HAL_StatusTypeDef i2c_status = HAL_OK;

	/* Construct the SB command with axis from function parameters */
	uint8_t command = MLX90393_REG_SW | ( zyxt & 0x0F );

	i2c_status = sendI2C( dev->i2cHandle, &status, &command, 1, 1 );

	/* Checks the HAL status of i2c transaction */
	if(i2c_status != HAL_OK) return MLX90393_STATUS_ERROR;
	else{
		return status;
	}
}


/* Start Measurement (polling), zyxt determines which axis to measure */
uint8_t MLX90393_SM	( MLX90393 *dev, uint8_t zyxt )
{
	uint8_t status;
	HAL_StatusTypeDef i2c_status = HAL_OK;

	/* Construct the SB command with axis from function parameters */
	uint8_t command = MLX90393_REG_SM | ( zyxt & 0x0F );

	i2c_status = sendI2C( dev->i2cHandle, &status, &command, 1, 1 );

	/* Checks the HAL status of i2c transaction */
	if(i2c_status != HAL_OK) return MLX90393_STATUS_ERROR;
	else{
		return status;
	}
}


/*
 * Read Measurement command, zyxt determines which axis to measure.
 * returns a pointer to an array of read data from device, passed as a parameter.
 *
 * readData needs to be a pointer to an array of size RM_DATA_LENGHT
 *
 */
uint8_t MLX90393_RM	( MLX90393 *dev, uint8_t zyxt, uint8_t* readData )
{
	uint8_t status;
	HAL_StatusTypeDef i2c_status = HAL_OK;

	/* Construct the SB command with axis from function parameters */
	uint8_t command = MLX90393_REG_RM | ( zyxt & 0x0F );

	i2c_status = sendI2C( dev->i2cHandle, readData, &command, 1, RM_DATA_LENGHT );

	/* First byte in data buffer is status */
	status = readData[0];

	/* Checks the HAL status of i2c transaction */
	if(i2c_status != HAL_OK) return MLX90393_STATUS_ERROR;
	else{
		return status;
	}
}


/*
 * Read Register command is used to read content from a specific register in volatile RAM
 * The address to read out is passed as a parameter
 *
 * readData needs to be a pointer to an array of size RR_DATA_LENGHT
 *
 */
uint8_t MLX90393_RR	( MLX90393 *dev, uint8_t regAddress, uint8_t *readData )
{
	uint8_t status;
	uint8_t sendBuffer[RR_ADDR_LENGHT];
	HAL_StatusTypeDef i2c_status = HAL_OK;

	/* Send buffer with command and register to read */
	sendBuffer[0] = MLX90393_REG_RR;
	sendBuffer[1] = regAddress;

	i2c_status = sendI2C( dev->i2cHandle, readData, sendBuffer, RR_ADDR_LENGHT, RR_DATA_LENGHT );

	/* First byte in data buffer is status */
	status = readData[0];

	/* Checks the HAL status of i2c transaction */
	if(i2c_status != HAL_OK) return MLX90393_STATUS_ERROR;
	else{
		return status;
	}
}


/* Write Register command, writes a word size register directly into volatile RAM */
uint8_t MLX90393_WR	( MLX90393 *dev, uint16_t *data, uint8_t regAddress )
{
	uint8_t status;
	uint8_t sendBuffer[4];
	HAL_StatusTypeDef i2c_status = HAL_OK;

	/* Constructs send buffer with Command + dataHI + dataLO + address */
	sendBuffer[0] = MLX90393_REG_WR;
	sendBuffer[1] = (*data >> 8);
	sendBuffer[2] = (*data & 0xFF);
	sendBuffer[3] = regAddress;

	i2c_status = sendI2C( dev->i2cHandle, &status, sendBuffer, WR_DATA_LENGHT, 1 );

	/* Checks the HAL status of i2c transaction */
	if(i2c_status != HAL_OK) return MLX90393_STATUS_ERROR;
	else{
		return status;
	}
}


/* Reset command, is used to reset the device, if device is runnning it will reset to idle mode */
uint8_t MLX90393_RT	( MLX90393 *dev )
{
	uint8_t status;
	HAL_StatusTypeDef i2c_status = HAL_OK;

	i2c_status = sendI2C( dev->i2cHandle, &status, (uint8_t*)MLX90393_REG_RT, 1, 1 );

	/* Checks the HAL status of i2c transaction */
	if(i2c_status != HAL_OK) return MLX90393_STATUS_ERROR;
	else{
		return status;
	}
}


/* Memmory Recall command, is used, the content from the non-volatile RAM is overwritten in the volatile RAM */
uint8_t MLX90393_HR	( MLX90393 *dev )
{
	uint8_t status;
	HAL_StatusTypeDef i2c_status = HAL_OK;

	i2c_status = sendI2C( dev->i2cHandle, &status, (uint8_t*)MLX90393_REG_HR, 1, 1 );

	/* Checks the HAL status of i2c transaction */
	if(i2c_status != HAL_OK) return MLX90393_STATUS_ERROR;
	else{
		return status;
	}
}


/* Memmory store command, the content from the volatile RAM is overwritten in the non-volatile RAM */
uint8_t MLX90393_HS	( MLX90393 *dev )
{
	uint8_t status;
	HAL_StatusTypeDef i2c_status = HAL_OK;

	i2c_status = sendI2C( dev->i2cHandle, &status, (uint8_t*)MLX90393_REG_HS, 1, 1 );

	/* Checks the HAL status of i2c transaction */
	if(i2c_status != HAL_OK) return MLX90393_STATUS_ERROR;
	else{
		return status;
	}
}



/* No Operation command, returns the status */
uint8_t MLX90393_NOP ( MLX90393 *dev )
{
	uint8_t status;
	HAL_StatusTypeDef i2c_status = HAL_OK;

	i2c_status = sendI2C( dev->i2cHandle, &status, (uint8_t*)MLX90393_REG_NOP, 1, 1 );

	/* Checks the HAL status of i2c transaction */
	if(i2c_status != HAL_OK) return MLX90393_STATUS_ERROR;
	else{
		return status;
	}
}


/* Function that communicates with MLX90393, first sending the command and waiting for the response */
HAL_StatusTypeDef sendI2C( I2C_HandleTypeDef *hi2c, uint8_t *receiveBuffer, uint8_t *sendBuffer, uint8_t sendMessageLength, uint8_t receiveMessageLength )
{
    HAL_StatusTypeDef i2c_status = HAL_OK;

	i2c_status |= HAL_I2C_Master_Transmit( hi2c, MLX90393_I2C_ADDRESS, sendBuffer, sendMessageLength, HAL_MAX_DELAY );
	i2c_status |= HAL_I2C_Master_Receive ( hi2c, MLX90393_I2C_ADDRESS, receiveBuffer, receiveMessageLength, HAL_MAX_DELAY );

    return i2c_status;
}


/* Initialization of sensor */
uint8_t MLX90393_Init ( MLX90393 *dev, I2C_HandleTypeDef *i2cHandle )
{
	/* Initialize structure parameters */
	dev->i2cHandle = i2cHandle;

	dev->mag[0] = 0.0f;
	dev->mag[1] = 0.0f;
	dev->mag[2] = 0.0f;

	dev->temp_c = 0.0f;

	/* Performs a NOP command to get the status of the device, returns HAL status */
	uint8_t status = 0;
	status = MLX90393_RT( dev );

	if( status == MLX90393_STATUS_ERROR ) return HAL_ERROR;
	else{

		/* Configure registers,  */
		MLX90393_CONF_1 configWord1;
		MLX90393_CONF_2 configWord2;
		MLX90393_CONF_3 configWord3;

		configWord1.data = 0x0000;
		configWord2.data = 0x0000;
		configWord3.data = 0x0000;

		configWord1.GAINSEL = 0x07;		/* Gain Select 7 */

		configWord3.RESX 	= 0x00; 	/* X resolution 0 */
		configWord3.RESY 	= 0x00; 	/* Y resolution 0 */
		configWord3.RESZ 	= 0x00; 	/* Z resolution 0 */
		configWord3.OSR 	= 0x03; 	/* OSR to 3 */
		configWord3.DIGFIL 	= 0x07; 	/* Digital filter to 3 */


		/* Write registers and returns status */
		uint8_t status1 = MLX90393_WR ( dev, &configWord1.data, MLX90393_CONF1 );
		uint8_t status2 = MLX90393_WR ( dev, &configWord2.data, MLX90393_CONF2 );
		uint8_t status3 = MLX90393_WR ( dev, &configWord3.data, MLX90393_CONF3 );

		status = status1 | status2 | status3;

		return status;
	}

}

