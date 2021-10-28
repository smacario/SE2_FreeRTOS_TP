/*
 * MLX90393.h
 *
 * 	Driver for I2C 3 axis Magnetometer (MLX90393)
 *
 *  Created on: Oct 26, 2021
 *      Author: Santiago Macario
 */

#ifndef INC_MLX90393_H_
#define INC_MLX90393_H_

#include <stdint.h>
#include "stm32l4xx_hal.h"

/*
 * The MLX90393 only listens to a specific set of commands in a command / response architecture.
 * Apart from the Reset command, all commands generate a status byte that can be read out.
 * The MLX90393 will always acknowledge a command in I2C, even if the command is not a valid command
 *
 * */

/* Device i2c address with A1 pin shorted to supply and A0 shorted to GND */
#define MLX90393_I2C_ADDRESS	(0x0E << 1)


#define MLX90393_CONFIG_1_ADD		(0x00)
#define MLX90393_CONFIG_2_ADD		(0x01)
#define MLX90393_CONFIG_3_ADD		(0x02)
#define MLX90393_SENS_TC_ADD		(0x03)

#define MLX90393_OFFSET_X_ADD		(0x04)
#define MLX90393_OFFSET_Y_ADD		(0x05)
#define MLX90393_OFFSET_Z_ADD		(0x06)


#define MLX90393_AXIS_ALL 			(0x0E)    /* X+Y+Z axis bits for commands */
#define MLX90393_CONF1 				(0x00)    /* Gain */
#define MLX90393_CONF2 				(0x01)    /* Burst, common mode */
#define MLX90393_CONF3 				(0x02)    /* Oversampling, filter, res */
#define MLX90393_CONF4 				(0x03)    /* Sensitivty drift */
#define MLX90393_GAIN_SHIFT 		(4)       /* Left-shift for gain bits */
#define MLX90393_HALL_CONF 			(0x0C)    /* Hall plate spinning rate adj */

#define MLX90393_STATUS_OK 			(0x00)
#define MLX90393_STATUS_SMMODE 		(0x08)
#define MLX90393_STATUS_RESET 		(0x01)
#define MLX90393_STATUS_ERROR 		(0xFF)
#define MLX90393_STATUS_MASK 		(0xFC)


/* Sensor struct
 *
 * 	Stores the Magnetometer data
 * 	for the X, Y and Z axis, in its respective float array.
 *
 * */
typedef struct {
	I2C_HandleTypeDef *i2cHandle;

	float mag[3];

	float temp_c;
}MLX90393;


/* Register map */
enum {
  MLX90393_REG_SB = (0x10),  		/* Start burst mode */
  MLX90393_REG_SW = (0x20),  		/* Start wakeup on change mode */
  MLX90393_REG_SM = (0x30),  		/* Start single-meas mode */
  MLX90393_REG_RM = (0x40),  		/* Read measurement */
  MLX90393_REG_RR = (0x50),  		/* Read register */
  MLX90393_REG_WR = (0x60),  		/* Write register */
  MLX90393_REG_EX = (0x80),  		/* Exit mode */
  MLX90393_REG_HR = (0xD0),  		/* Memory recall */
  MLX90393_REG_HS = (0x70),  		/* Memory store */
  MLX90393_REG_RT = (0xF0),  		/* Reset */
  MLX90393_REG_NOP = (0x00), 		/* NOP */
};


/* Gain settings for CONF1 register */
typedef enum mlx90393_gain {
  MLX90393_GAIN_5X = (0x00),
  MLX90393_GAIN_4X,
  MLX90393_GAIN_3X,
  MLX90393_GAIN_2_5X,
  MLX90393_GAIN_2X,
  MLX90393_GAIN_1_67X,
  MLX90393_GAIN_1_33X,
  MLX90393_GAIN_1X
} mlx90393_gain_t;


/* Resolution settings for CONF3 register */
typedef enum mlx90393_resolution {
  MLX90393_RES_16,
  MLX90393_RES_17,
  MLX90393_RES_18,
  MLX90393_RES_19,
} mlx90393_resolution_t;


/* Axis designator */
typedef enum mlx90393_axis {
  MLX90393_X,
  MLX90393_Y,
  MLX90393_Z
} mlx90393_axis_t;


/* Digital filter settings for CONF3 register */
typedef enum mlx90393_filter {
  MLX90393_FILTER_0,
  MLX90393_FILTER_1,
  MLX90393_FILTER_2,
  MLX90393_FILTER_3,
  MLX90393_FILTER_4,
  MLX90393_FILTER_5,
  MLX90393_FILTER_6,
  MLX90393_FILTER_7,
} mlx90393_filter_t;


/* Oversampling settings for CONF3 register */
typedef enum mlx90393_oversampling {
  MLX90393_OSR_0,
  MLX90393_OSR_1,
  MLX90393_OSR_2,
  MLX90393_OSR_3,
} mlx90393_oversampling_t;


/* Function prototypes */

uint8_t MLX90393_Init( MLX90393 *dev, I2C_HandleTypeDef *i2cHandle );

HAL_StatusTypeDef sendI2C( I2C_HandleTypeDef *hi2c, uint8_t *receiveBuffer, uint8_t *sendBuffer, uint8_t sendMessageLength, uint8_t receiveMessageLength );

HAL_StatusTypeDef MLX90393_ReadRegister( MLX90393 *dev, uint8_t reg, uint8_t *data );
HAL_StatusTypeDef MLX90393_ReadMultipleRegisters( MLX90393 *dev, uint8_t reg, uint8_t *data, uint8_t length );

HAL_StatusTypeDef MLX90393_WriteRegister( MLX90393 *dev, uint8_t reg, uint8_t *data );


/* Command function prototypes */

uint8_t MLX90393_EX		( MLX90393 *dev );
uint8_t MLX90393_SB		( MLX90393 *dev, uint8_t zyxt );
uint8_t MLX90393_SWOC	( MLX90393 *dev, uint8_t zyxt );
uint8_t MLX90393_SM		( MLX90393 *dev, uint8_t zyxt );
uint8_t MLX90393_RM		( MLX90393 *dev );
uint8_t MLX90393_RR		( MLX90393 *dev, uint16_t *data );
uint8_t MLX90393_WR		( MLX90393 *dev, uint16_t *data, uint8_t add );
uint8_t MLX90393_RT		( MLX90393 *dev );
uint8_t MLX90393_HR		( MLX90393 *dev );
uint8_t MLX90393_HS		( MLX90393 *dev );
uint8_t MLX90393_NOP	( MLX90393 *dev );


#endif /* INC_MLX90393_H_ */
