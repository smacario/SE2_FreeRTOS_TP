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
#define WHO_AM_I_ADD			(0x75)
#define IIM42652_I2C_ID			(0x6F)


/* Bank select register, accesible from all banks
 *
 * 	[2:0]	- 000: Bank 0 (default)
 * 			- 001: Bank 1
 * 			- 010: Bank 2
 * 			- 011: Bank 3
 * 			- 100: Bank 4
 */
#define BANK_SEL_ADD			(0x76)


/*
 *	No register modifications should be done while the sensor is running
 *
 *	The correct order of operations is
 *		- Turn Accel and Gyro off
 *		- Modify registers
 *		- Turn Accel and Gyro on
 */


/* Bank 0 (p.57) */
#define DEVICE_CONFIG_ADD		(0x11)
#define DRIVE_CONFIG_ADD		(0x13)
#define INT_CONFIG_ADD			(0x14)
#define FIFO_CONFIG_ADD			(0X16)

#define TEMP_DATA_HI_ADD		(0x1D)
#define TEMP_DATA_LO_ADD		(0x1E)

#define ACCL_DATA_XHI_ADD		(0x1F)
#define ACCL_DATA_XLO_ADD		(0x20)
#define ACCL_DATA_YHI_ADD		(0x21)
#define ACCL_DATA_YLO_ADD		(0x22)
#define ACCL_DATA_ZHI_ADD		(0x23)
#define ACCL_DATA_ZLO_ADD		(0x24)

#define GYRO_DATA_XHI_ADD		(0x25)
#define GYRO_DATA_XLO_ADD		(0x26)
#define GYRO_DATA_YHI_ADD		(0x27)
#define GYRO_DATA_YLO_ADD		(0x28)
#define GYRO_DATA_ZHI_ADD		(0x29)
#define GYRO_DATA_ZLO_ADD		(0x2A)

#define INT_STATUS_ADD			(0x2D)
#define PWR_MGMT0_ADD			(0x4E)

#define GYRO_CONFIG0_ADD		(0x4F)
#define ACCL_CONFIG0_ADD		(0x50)
#define GYRO_CONFIG1_ADD		(0x51)
#define GYR_ACC_CONFIG0_ADD		(0X52)
#define ACCL_CONFIG1_ADD		(0X53)

#define INT_CONFIG0_ADD			(0x63)
#define INT_CONFIG1_ADD			(0x64)

#define INT_SOURCE0_ADD			(0x65)
#define INT_SOURCE1_ADD			(0x66)
#define INT_SOURCE2_ADD			(0x68)
#define INT_SOURCE3_ADD			(0x69)

#define SELF_TEST_CONFIG_ADD	(0x70)


/* Bank 1 (p.58)*/
#define SENSOR_CONFIG0_ADD		(0x03)

#define GYRO_CONFIG2_ADD		(0x0B)
#define GYRO_CONFIG3_ADD		(0x0C)
#define GYRO_CONFIG4_ADD		(0x0D)
#define GYRO_CONFIG5_ADD		(0x0E)
#define GYRO_CONFIG6_ADD		(0x0F)
#define GYRO_CONFIG7_ADD		(0x10)
#define GYRO_CONFIG8_ADD		(0x11)
#define GYRO_CONFIG9_ADD		(0x12)
#define GYRO_CONFIG10_ADD		(0x13)

#define XG_ST_DATA_ADD			(0x5F)
#define YG_ST_DATA_ADD			(0x60)
#define ZG_ST_DATA_ADD			(0x61)

#define INTF_CONFIG4_ADD		(0x7A)
#define INTF_CONFIG5_ADD		(0x7B)
#define INTF_CONFIG6_ADD		(0x7C)


/* Bank 2 (p.59)*/
#define ACCL_CONFIG2_ADD		(0x03)
#define ACCL_CONFIG3_ADD		(0x04)
#define ACCL_CONFIG4_ADD		(0x05)

#define XA_ST_DATA_ADD			(0x3B)
#define YA_ST_DATA_ADD			(0x3C)
#define ZA_ST_DATA_ADD			(0x3D)


/* Bank 3 (p.59)*/
#define PU_PD_CONFIG1			(0x06)
#define PU_PD_CONFIG2			(0x0E)


/* Bank 4 (p.59)*/
#define INT_SOURCE6_ADD			(0x4D)
#define INT_SOURCE7_ADD			(0x4E)
#define INT_SOURCE8_ADD			(0x4F)
#define INT_SOURCE9_ADD			(0x50)
#define INT_SOURCE10_ADD		(0x51)

#define OFFSET_USER0_ADD		(0x77)
#define OFFSET_USER1_ADD		(0x78)
#define OFFSET_USER2_ADD		(0x79)
#define OFFSET_USER3_ADD		(0x7A)
#define OFFSET_USER4_ADD		(0x7B)
#define OFFSET_USER5_ADD		(0x7C)
#define OFFSET_USER6_ADD		(0x7D)
#define OFFSET_USER7_ADD		(0x7E)
#define OFFSET_USER8_ADD		(0x7F)


/* Register configuration macros */
#define IIM42652_SET_TEMPERATURE_ENABLED                 0xDF
#define IIM42652_SET_TEMPERATURE_DISABLED                0x20

#define IIM42652_SET_GYRO_OFF_MODE                       0x00
#define IIM42652_SET_GYRO_STANDBY_MODE                   0x04
#define IIM42652_SET_GYRO_TLOW_NOISE_MODE                0x0C

#define IIM42652_SET_ACCEL_OFF_MODE                      0x00
#define IIM42652_SET_ACCEL_LOW_POWER_MODE                0x02
#define IIM42652_SET_ACCEL_LOW_NOISE_MODE                0x03

#define IIM42652_SET_GYRO_FS_SEL_2000_dps                0x00
#define IIM42652_SET_GYRO_FS_SEL_1000_dps                0x01
#define IIM42652_SET_GYRO_FS_SEL_500_dps                 0x02
#define IIM42652_SET_GYRO_FS_SEL_250_dps                 0x03
#define IIM42652_SET_GYRO_FS_SEL_125_dps                 0x04
#define IIM42652_SET_GYRO_FS_SEL_62_5_dps                0x05
#define IIM42652_SET_GYRO_FS_SEL_31_25_dps               0x06
#define IIM42652_SET_GYRO_FS_SEL_16_625_dps              0x07

#define IIM42652_SET_GYRO_ODR_32kHz                      0x01
#define IIM42652_SET_GYRO_ODR_16kHz                      0x02
#define IIM42652_SET_GYRO_ODR_8kHz                       0x03
#define IIM42652_SET_GYRO_ODR_4kHz                       0x04
#define IIM42652_SET_GYRO_ODR_2kHz                       0x05
#define IIM42652_SET_GYRO_ODR_1kHz                       0x06
#define IIM42652_SET_GYRO_ODR_200Hz                      0x07
#define IIM42652_SET_GYRO_ODR_100Hz                      0x08
#define IIM42652_SET_GYRO_ODR_50Hz                       0x09
#define IIM42652_SET_GYRO_ODR_25Hz                       0x0A
#define IIM42652_SET_GYRO_ODR_12_5Hz                     0x0B

#define IIM42652_SET_GYRO_UI_FILT_ORD_1st                0x00
#define IIM42652_SET_GYRO_UI_FILT_ORD_2st                0x01
#define IIM42652_SET_GYRO_UI_FILT_ORD_3st                0x02

#define IIM42652_SET_GYRO_DEC2_M2_ORD_3st                0x02

#define IIM42652_SET_GYRO_UI_FILT_BW_ODR_2               0x00
#define IIM42652_SET_GYRO_UI_FILT_BW_ODR_4               0x01
#define IIM42652_SET_GYRO_UI_FILT_BW_ODR_5               0x02
#define IIM42652_SET_GYRO_UI_FILT_BW_ODR_8               0x03
#define IIM42652_SET_GYRO_UI_FILT_BW_ODR_10              0x04
#define IIM42652_SET_GYRO_UI_FILT_BW_ODR_16              0x05
#define IIM42652_SET_GYRO_UI_FILT_BW_ODR_20              0x06
#define IIM42652_SET_GYRO_UI_FILT_BW_ODR_40              0x07
#define IIM42652_SET_GYRO_UI_FILT_BW_LOW_LATENCY_0       0x0E
#define IIM42652_SET_GYRO_UI_FILT_BW_LOW_LATENCY_1       0x0F

#define IIM42652_SET_ACCEL_FS_SEL_16g                    0x00
#define IIM42652_SET_ACCEL_FS_SEL_8g                     0x01
#define IIM42652_SET_ACCEL_FS_SEL_4g                     0x02
#define IIM42652_SET_ACCEL_FS_SEL_2g                     0x03

#define IIM42652_SET_ACCEL_ODR_32kHz                     0x01
#define IIM42652_SET_ACCEL_ODR_16kHz                     0x02
#define IIM42652_SET_ACCEL_ODR_8kHz                      0x03
#define IIM42652_SET_ACCEL_ODR_4kHz                      0x04
#define IIM42652_SET_ACCEL_ODR_2kHz                      0x05
#define IIM42652_SET_ACCEL_ODR_1kHz                      0x06
#define IIM42652_SET_ACCEL_ODR_200Hz                     0x07
#define IIM42652_SET_ACCEL_ODR_100Hz                     0x08
#define IIM42652_SET_ACCEL_ODR_50Hz                      0x09
#define IIM42652_SET_ACCEL_ODR_25Hz                      0x0A
#define IIM42652_SET_ACCEL_ODR_12_5Hz                    0x0B

#define IIM42652_SET_ACCEL_UI_FILT_BW_ODR_2              0x00
#define IIM42652_SET_ACCEL_UI_FILT_BW_ODR_4              0x01
#define IIM42652_SET_ACCEL_UI_FILT_BW_ODR_5              0x02
#define IIM42652_SET_ACCEL_UI_FILT_BW_ODR_8              0x03
#define IIM42652_SET_ACCEL_UI_FILT_BW_ODR_10             0x04
#define IIM42652_SET_ACCEL_UI_FILT_BW_ODR_16             0x05
#define IIM42652_SET_ACCEL_UI_FILT_BW_ODR_20             0x06
#define IIM42652_SET_ACCEL_UI_FILT_BW_ODR_40             0x07
#define IIM42652_SET_ACCEL_UI_FILT_BW_LOW_LATENCY_0      0x0E
#define IIM42652_SET_ACCEL_UI_FILT_BW_LOW_LATENCY_1      0x0F

#define IIM42652_SET_ACCEL_UI_FILT_ORD_1st               0x00
#define IIM42652_SET_ACCEL_UI_FILT_ORD_2st               0x01
#define IIM42652_SET_ACCEL_UI_FILT_ORD_3st               0x02

#define IIM42652_SET_ACCEL_DEC2_M2_ORD_3st               0x02

#define IIM42652_SET_BANK_0                              0x00
#define IIM42652_SET_BANK_1                              0x01
#define IIM42652_SET_BANK_2                              0x02
#define IIM42652_SET_BANK_3                              0x03
#define IIM42652_SET_BANK_4                              0x04

#define IIM42654_SET_INT1						         0x04
#define IIM42652_SET_INT_ACTIVE_HI						 0x01
#define IIM42652_SET_UI_DRDY_INT_CLEAR					 0x20
#define IIM42652_SET_UI_DRDY_INT1_EN					 0x08



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


typedef struct
{
    uint8_t gyro_fs_sel;
    uint8_t gyro_odr;
    uint8_t gyro_ui_filt_ord;
    uint8_t gyro_dec2_m2_ord;
    uint8_t gyro_ui_filt_bw;

} IIM42652_GYRO_CFG_t;



typedef struct
{
    uint8_t accel_fs_sel;
    uint8_t accel_odr;
    uint8_t accel_ui_filt_bw;
    uint8_t accel_ui_filt_ord;
    uint8_t accel_dec2_m2_ord;

} IIM42652_ACCL_CFG_t;



/* Function prototypes */

uint8_t IIM42652_Init( IIM42652 *dev, I2C_HandleTypeDef *i2cHandle );

HAL_StatusTypeDef IIM42652_ReadAcc( IIM42652 *dev );
HAL_StatusTypeDef IIM42652_ReadGyr( IIM42652 *dev );
HAL_StatusTypeDef IIM42652_ReadTmp( IIM42652 *dev );

HAL_StatusTypeDef IIM42652_SoftReset( IIM42652 *dev );
HAL_StatusTypeDef IIM42652_EnableGyro( IIM42652 *dev );
HAL_StatusTypeDef IIM42652_EnableAccel( IIM42652 *dev );
HAL_StatusTypeDef IIM42652_setConfigGyro( IIM42652 *dev, IIM42652_GYRO_CFG_t gyrCfg );
HAL_StatusTypeDef IIM42652_setConfigAccel( IIM42652 *dev, IIM42652_ACCL_CFG_t accCfg );
HAL_StatusTypeDef IIM42652_ConfigInterrupt( IIM42652 *dev );

HAL_StatusTypeDef IIM42652_ReadMeasurementAxisAll( IIM42652 *dev );

HAL_StatusTypeDef IIM42652_ReadRegister( IIM42652 *dev, uint8_t reg, uint8_t *data );
HAL_StatusTypeDef IIM42652_ReadMultipleRegisters( IIM42652 *dev, uint8_t reg, uint8_t *data, uint8_t length );

HAL_StatusTypeDef IIM42652_WriteRegister( IIM42652 *dev, uint8_t reg, uint8_t *data );


#endif /* IIM42652_H_ */
