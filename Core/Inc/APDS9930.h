/*
 * APDS9930.h
 *
 * Created on: October 25, 2024
 * Author: Josue Flores
 *
 * Last modified: October 25, 2024
 */

#ifndef INC_APDS9930_H_
#define INC_APDS9930_H_

#include "stm32f3xx_hal.h" // NEEDED FOR I2C

/*
 * DEFINES
 */

#define APDS9930_I2C_ADDR		(0x39 << 1)
#define APDS9930_ID_1			0x39
#define APDS9930_ID_2			0x12

/*
 * REGISTERS p.18
 */

#define APDS9930_REG_COMMAND_REPEAT		0x80
#define APDS9930_REG_COMMAND_AUTO_INC	0xA0
#define APDS9930_REG_ENABLE				0x00
#define APDS9930_REG_ATIME				0x01
#define APDS9930_REG_PTIME				0x02
#define APDS9930_REG_WTIME				0x03
#define APDS9930_REG_AILTL				0x04
#define APDS9930_REG_AILTH				0x05
#define APDS9930_REG_AIHTL				0x06
#define APDS9930_REG_AIHTH				0x07
#define APDS9930_REG_PILTL				0x08
#define APDS9930_REG_PILTH				0x09
#define APDS9930_REG_PIHTL				0x0A
#define APDS9930_REG_PIHTH				0x0B
#define APDS9930_REG_PERS				0x0C
#define APDS9930_REG_CONFIG				0x0D
#define APDS9930_REG_PPULSE				0x0E
#define APDS9930_REG_CONTROL			0x0F
#define APDS9930_REG_ID					0x12
#define APDS9930_REG_STATUS				0x13
#define APDS9930_REG_Ch0DATAL			0x14
#define APDS9930_REG_Ch0DATAH			0x15
#define APDS9930_REG_Ch1DATAL			0x16
#define APDS9930_REG_Ch1DATAH			0x17
#define APDS9930_REG_PDATAL				0x18
#define APDS9930_REG_PDATAH				0x19
#define APDS9930_REG_POFFSET			0x0E

/*
 * LUX COEFFICIENTS p.9
 */

#define DF		52
#define GA		0.49
#define B		1.862
#define C		0.746
#define D		1.291

/*
 * SENSOR STRUCT
 */

typedef struct {
	I2C_HandleTypeDef* i2c_handle;

	// processed lux and proximity (in mm) data
	float lux;
	float prox;
} APDS9930_t;

/*
 * INITIALIZATION
 */

// Outputs possible error codes
uint8_t APDS9930_Init (APDS9930_t* device, I2C_HandleTypeDef* i2c_handle);

/*
 * DATA AQUISITION
 */

HAL_StatusTypeDef APDS9930_ReadLux (APDS9930_t* device);
HAL_StatusTypeDef APDS9930_ReadProximity (APDS9930_t* device);

/*
 * LOW LEVEL FUNCTIONS
 */

HAL_StatusTypeDef APDS9930_ReadRegister (APDS9930_t* device, uint8_t reg, uint8_t* data);
HAL_StatusTypeDef APDS9930_ReadRegisters (APDS9930_t* device, uint8_t reg, uint8_t* data, uint8_t length);
HAL_StatusTypeDef APDS9930_WriteRegister (APDS9930_t* device, uint8_t reg, uint8_t* data);

HAL_StatusTypeDef APDS9930_CMD_ReadRegister (APDS9930_t* device, uint8_t reg, uint8_t* data);
HAL_StatusTypeDef APDS9930_CMD_ReadRegisters (APDS9930_t* device, uint8_t reg, uint8_t* data, uint8_t length);
HAL_StatusTypeDef APDS9930_CMD_WriteRegister (APDS9930_t* device, uint8_t reg, uint8_t* data);
//HAL_StatusTypeDef APDS9930_ReadRegisters (APDS9930_t* device, uint8_t reg, uint8_t* data, uint8_t length);
//
//HAL_StatusTypeDef APDS9930_WriteRegister (APDS9930_t* device, uint8_t reg, uint8_t* data);

#endif /* INC_APDS9930_H_ */
