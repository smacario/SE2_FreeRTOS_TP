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
#include "cmsis_os.h"
#include "semphr.h"
#include "gpio.h"

/* Config bit masks */
#define BIT_MASK_BIT_0    0x01
#define BIT_MASK_BIT_1    0x02
#define BIT_MASK_BIT_2    0x04
#define BIT_MASK_BIT_3    0x08
#define BIT_MASK_BIT_4    0x10
#define BIT_MASK_BIT_5    0x20
#define BIT_MASK_BIT_6    0x40
#define BIT_MASK_BIT_7    0x80


SemaphoreHandle_t ImuIntSemaphore;	// Handler of semaphore that blocks the sensor reading function until the DRDY interrupt arrives.
uint8_t DRDY_IIMFlag = 0x00;			// Flags the occurrence of the DRDY interrupt from IIM42652.

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


/* Function that performs a soft reset of device */
HAL_StatusTypeDef IIM42652_SoftReset( IIM42652 *dev )
{

	HAL_StatusTypeDef status;
	uint8_t config;

	/* Reads config register, enables reset bit and writes changes */
	status =  IIM42652_ReadRegister( dev, DEVICE_CONFIG_ADD, &config );

	config |= 0x01;

	status |= IIM42652_WriteRegister( dev, DEVICE_CONFIG_ADD, &config );

	/* Waits for reset */
	vTaskDelay( 5000 / portTICK_PERIOD_MS );

	return status;
}


/* Function enables gyroscope */
HAL_StatusTypeDef IIM42652_EnableGyro( IIM42652 *dev )
{

	HAL_StatusTypeDef status;
	uint8_t config;

	/* Reads config register, enables reset bit and writes changes */
	status =  IIM42652_ReadRegister( dev, PWR_MGMT0_ADD, &config );

	config |= IIM42652_SET_GYRO_TLOW_NOISE_MODE;

	status |= IIM42652_WriteRegister( dev, PWR_MGMT0_ADD, &config );

	/* Waits for gyroscope power on */
	vTaskDelay( 5000 / portTICK_PERIOD_MS );

	return status;
}


/* Function enables gyroscope */
HAL_StatusTypeDef IIM42652_EnableAccel( IIM42652 *dev )
{

	HAL_StatusTypeDef status;
	uint8_t config;

	/* Reads config register, enables reset bit and writes changes */
	status =  IIM42652_ReadRegister( dev, PWR_MGMT0_ADD, &config );

	config |= IIM42652_SET_ACCEL_LOW_NOISE_MODE;

	status |= IIM42652_WriteRegister( dev, PWR_MGMT0_ADD, &config );

	/* Waits for accelerometer power on */
	vTaskDelay( 5000 / portTICK_PERIOD_MS );

	return status;
}


/* Configures gyroscope parameters */
HAL_StatusTypeDef IIM42652_setConfigGyro( IIM42652 *dev, IIM42652_GYRO_CFG_t gyrCfg )
{
	uint8_t config = 0x00;
	HAL_StatusTypeDef status = HAL_OK;

	/* Writes gyroscope selected registers */

	config  = ( gyrCfg.gyro_fs_sel & ( BIT_MASK_BIT_2 | BIT_MASK_BIT_1 | BIT_MASK_BIT_0 ) ) << 5;
	config |=   gyrCfg.gyro_odr    & ( BIT_MASK_BIT_3 | BIT_MASK_BIT_2 | BIT_MASK_BIT_1 | BIT_MASK_BIT_0 );

	status = IIM42652_WriteRegister( dev, GYRO_CONFIG0_ADD, &config );

	config  =  ( gyrCfg.gyro_ui_filt_ord  & ( BIT_MASK_BIT_1 | BIT_MASK_BIT_0 ) ) << 2;
	config |=    gyrCfg.gyro_dec2_m2_ord  & ( BIT_MASK_BIT_1 | BIT_MASK_BIT_0 );

	status |= IIM42652_WriteRegister( dev, GYRO_CONFIG1_ADD, &config );


	status |=  IIM42652_ReadRegister( dev, GYR_ACC_CONFIG0_ADD, &config );

	config |= gyrCfg.gyro_ui_filt_bw & ( BIT_MASK_BIT_3 | BIT_MASK_BIT_2 | BIT_MASK_BIT_1 | BIT_MASK_BIT_0 );

	status |= IIM42652_WriteRegister( dev, GYRO_CONFIG1_ADD, &config );


	return status;
}


/* Configure accelerometer parameters */
HAL_StatusTypeDef IIM42652_setConfigAccel( IIM42652 *dev, IIM42652_ACCL_CFG_t accCfg )
{
	uint8_t config = 0x00;
	HAL_StatusTypeDef status = HAL_OK;

	/* Writes gyroscope selected registers */

	config  = ( accCfg.accel_fs_sel & ( BIT_MASK_BIT_2 | BIT_MASK_BIT_1 | BIT_MASK_BIT_0 ) ) << 5;
	config |=   accCfg.accel_odr    & ( BIT_MASK_BIT_3 | BIT_MASK_BIT_2 | BIT_MASK_BIT_1 | BIT_MASK_BIT_0 );

	status |= IIM42652_WriteRegister( dev, ACCL_CONFIG0_ADD, &config );

	status |=  IIM42652_ReadRegister( dev, GYR_ACC_CONFIG0_ADD, &config );

	config |= ( accCfg.accel_ui_filt_bw & ( BIT_MASK_BIT_3 | BIT_MASK_BIT_2 | BIT_MASK_BIT_1 | BIT_MASK_BIT_0 ) ) << 4;

	status |= IIM42652_WriteRegister( dev, GYR_ACC_CONFIG0_ADD, &config );

	config  =  ( accCfg.accel_ui_filt_ord  & ( BIT_MASK_BIT_1 | BIT_MASK_BIT_0 ) ) << 3;
	config |=  ( accCfg.accel_dec2_m2_ord  & ( BIT_MASK_BIT_1 | BIT_MASK_BIT_0 ) ) << 1;

	status |= IIM42652_WriteRegister( dev, ACCL_CONFIG1_ADD, &config );

	return status;
}


HAL_StatusTypeDef IIM42652_ConfigInterrupt( IIM42652 *dev )
{
	uint8_t config = 0x00;
	HAL_StatusTypeDef status = HAL_OK;

	/* Implement interrupt configuration */
	config = IIM42654_SET_INT1 | IIM42652_SET_INT_ACTIVE_HI;
	status = IIM42652_WriteRegister( dev, INT_CONFIG_ADD, &config );

	config = IIM42652_SET_UI_DRDY_INT_CLEAR;
	status = IIM42652_WriteRegister( dev, INT_CONFIG0_ADD, &config );

	config = IIM42652_SET_UI_DRDY_INT1_EN;
	status = IIM42652_WriteRegister( dev, INT_SOURCE0_ADD, &config );

	return status;
}


uint8_t IIM42652_Init( IIM42652 *dev, I2C_HandleTypeDef *i2cHandle )
{

	IIM42652_ACCL_CFG_t ACC_CFG;
	IIM42652_GYRO_CFG_t GYR_CFG;

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
	else {

		/* Initialize routine */
		status |= IIM42652_SoftReset  ( dev );
		status |= IIM42652_EnableGyro ( dev );
		status |= IIM42652_EnableAccel( dev );


		GYR_CFG.gyro_fs_sel      = IIM42652_SET_GYRO_FS_SEL_2000_dps;
		GYR_CFG.gyro_odr         = IIM42652_SET_GYRO_ODR_1kHz;
		GYR_CFG.gyro_ui_filt_ord = IIM42652_SET_GYRO_UI_FILT_ORD_2st;
		GYR_CFG.gyro_dec2_m2_ord = IIM42652_SET_GYRO_DEC2_M2_ORD_3st;
		GYR_CFG.gyro_ui_filt_bw  = IIM42652_SET_GYRO_UI_FILT_BW_ODR_4;
	    IIM42652_setConfigGyro( dev, GYR_CFG );

	    ACC_CFG.accel_fs_sel      = IIM42652_SET_ACCEL_FS_SEL_16g;
	    ACC_CFG.accel_odr         = IIM42652_SET_ACCEL_ODR_1kHz;
	    ACC_CFG.accel_ui_filt_bw  = IIM42652_SET_ACCEL_UI_FILT_BW_ODR_4;
	    ACC_CFG.accel_ui_filt_ord = IIM42652_SET_ACCEL_UI_FILT_ORD_2st;
	    ACC_CFG.accel_dec2_m2_ord = IIM42652_SET_ACCEL_DEC2_M2_ORD_3st;
	    IIM42652_setConfigAccel( dev, ACC_CFG );

	    IIM42652_ConfigInterrupt( dev );

	    /* Takes semaphore until DRDY is ready */
	    xSemaphoreTake( ImuIntSemaphore, portMAX_DELAY );
	}


	return 0;
}



HAL_StatusTypeDef IIM42652_ReadMeasurementAxisAll( IIM42652 *dev )
{

	HAL_StatusTypeDef readStatus;

	/* Takes semaphore until DRDY is ready again */
	xSemaphoreTake( ImuIntSemaphore, portMAX_DELAY );
	DRDY_IIMFlag = 0x00;

	return readStatus;
}


void IIM42652_DRDYCallback( void )
{
	/* Gives semaphore and yields */

	if(DRDY_IIMFlag == 0x00)
	{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR( ImuIntSemaphore, &xHigherPriorityTaskWoken );
		portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

		DRDY_IIMFlag = 0x01;
	}
}









