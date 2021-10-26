/*
 * IIM42652.h
 *
 * 	Driver for I2C 6 axis IMU sensor (IIM-42652)
 *
 *  Created on: Oct 20, 2021
 *      Author: Santiago Macario
 */

#ifndef IIM42652_H_
#define IIM42652_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

/* Device i2c bus address with AP pin high on PCB (0b01101001) */
#define IIM42652_I2C_ADDRESS	(0x69 << 1)

/* Who am i register, expected return value is 0x6F */
#define WHO_AM_I_ADD			0x75
#define IIM42652_I2C_ID			0x6F


/* Bank select register, accesible from all banks
 *
 * 	[2:0]	- 000: Bank 0 (default)
 * 			- 001: Bank 1
 * 			- 010: Bank 2
 * 			- 011: Bank 3
 * 			- 100: Bank 4
 */
#define BANK_SEL_ADD			0x76


/*
 *	No register modifications should be done while the sensor is running
 *
 *	The correct order of operations is
 *		- Turn Accel and Gyro off
 *		- Modify registers
 *		- Turn Accel and Gyro on
 */


/* Bank 0 (p.57) */
#define DEVICE_CONFIG_ADD		0x11
#define DRIVE_CONFIG_ADD		0x13
#define INT_CONFIG_ADD			0x14
#define FIFO_CONFIG_ADD			0X16

#define TEMP_DATA_HI_ADD		0x1D
#define TEMP_DATA_LO_ADD		0x1E

#define ACCL_DATA_XHI_ADD		0x1F
#define ACCL_DATA_XLO_ADD		0x20
#define ACCL_DATA_YHI_ADD		0x21
#define ACCL_DATA_YLO_ADD		0x22
#define ACCL_DATA_ZHI_ADD		0x23
#define ACCL_DATA_ZLO_ADD		0x24

#define GYRO_DATA_XHI_ADD		0x25
#define GYRO_DATA_XLO_ADD		0x26
#define GYRO_DATA_YHI_ADD		0x27
#define GYRO_DATA_YLO_ADD		0x28
#define GYRO_DATA_ZHI_ADD		0x29
#define GYRO_DATA_ZLO_ADD		0x2A

#define INT_STATUS_ADD			0x2D
#define PWR_MGMT0_ADD			0x4E

#define GYRO_CONFIG0_ADD		0x4F
#define ACCL_CONFIG0_ADD		0x50
#define GYRO_CONFIG1_ADD		0x51
#define GYR_ACC_CONFIG0_ADD		0X52
#define ACCL_CONFIG1_ADD		0X53

#define INT_CONFIG0_ADD			0x63
#define INT_CONFIG1_ADD			0x64

#define INT_SOURCE0_ADD			0x65
#define INT_SOURCE1_ADD			0x66
#define INT_SOURCE2_ADD			0x68
#define INT_SOURCE3_ADD			0x69

#define SELF_TEST_CONFIG_ADD	0x70


/* Bank 1 (p.58)*/
#define SENSOR_CONFIG0_ADD		0x03

#define GYRO_CONFIG2_ADD		0x0B
#define GYRO_CONFIG3_ADD		0x0C
#define GYRO_CONFIG4_ADD		0x0D
#define GYRO_CONFIG5_ADD		0x0E
#define GYRO_CONFIG6_ADD		0x0F
#define GYRO_CONFIG7_ADD		0x10
#define GYRO_CONFIG8_ADD		0x11
#define GYRO_CONFIG9_ADD		0x12
#define GYRO_CONFIG10_ADD		0x13

#define XG_ST_DATA_ADD			0x5F
#define YG_ST_DATA_ADD			0x60
#define ZG_ST_DATA_ADD			0x61

#define INTF_CONFIG4_ADD		0x7A
#define INTF_CONFIG5_ADD		0x7B
#define INTF_CONFIG6_ADD		0x7C


/* Bank 2 (p.59)*/
#define ACCL_CONFIG2_ADD		0x03
#define ACCL_CONFIG3_ADD		0x04
#define ACCL_CONFIG4_ADD		0x05

#define XA_ST_DATA_ADD			0x3B
#define YA_ST_DATA_ADD			0x3C
#define ZA_ST_DATA_ADD			0x3D


/* Bank 3 (p.59)*/
#define PU_PD_CONFIG1			0x06
#define PU_PD_CONFIG2			0x0E


/* Bank 4 (p.59)*/
#define INT_SOURCE6_ADD			0x4D
#define INT_SOURCE7_ADD			0x4E
#define INT_SOURCE8_ADD			0x4F
#define INT_SOURCE9_ADD			0x50
#define INT_SOURCE10_ADD		0x51

#define OFFSET_USER0_ADD		0x77
#define OFFSET_USER1_ADD		0x78
#define OFFSET_USER2_ADD		0x79
#define OFFSET_USER3_ADD		0x7A
#define OFFSET_USER4_ADD		0x7B
#define OFFSET_USER5_ADD		0x7C
#define OFFSET_USER6_ADD		0x7D
#define OFFSET_USER7_ADD		0x7E
#define OFFSET_USER8_ADD		0x7F


/* Sensor struct
 *
 * 	Stores the accelerometer, gyroscope, and temperature data
 * 	for the X, Y and Z axis, in their respective float array.
 *
 * */
typedef struct {
	I2C_HandleTypeDef *i2cHandle;

	float acc[3];
	float gyr[3];

	float temp_c;
}IIM42652;



/* Functions prototypes */

uint8_t IIM42652_Init( IIM42652 *dev, I2C_HandleTypeDef *i2cHandle );

HAL_StatusTypeDef IIM42652_ReadAcc( IIM42652 *dev );
HAL_StatusTypeDef IIM42652_ReadGyr( IIM42652 *dev );
HAL_StatusTypeDef IIM42652_ReadTmp( IIM42652 *dev );

HAL_StatusTypeDef IIM42652_ReadRegister( IIM42652 *dev, uint8_t reg, uint8_t *data );
HAL_StatusTypeDef IIM42652_ReadMultipleRegisters( IIM42652 *dev, uint8_t reg, uint8_t *data, uint8_t length );

HAL_StatusTypeDef IIM42652_WriteRegister( IIM42652 *dev, uint8_t reg, uint8_t *data );


#endif /* IIM42652_H_ */
