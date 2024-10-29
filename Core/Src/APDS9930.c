/*
 * APDS9930.c
 *
 * Created on: October 25, 2024
 * Author: Josue Flores
 *
 * Last modified: October 25, 2024
 */

#include <stdarg.h>
#include <stdlib.h>

#include "APDS9930.h"

// PRIVATE VARIABLES
float ALSIT;
uint8_t AGAIN;

uint16_t Ch0_data;
uint16_t Ch1_data;

float IAC1;
float IAC2;

float IAC;
float LPC;

// PRIVATE FUNCTION PROTOTYPES
float Max (int num, ...);
uint8_t Pow (uint8_t base, uint8_t exp);

/*
 * INITIALIZATION
 */

uint8_t APDS9930_Init (APDS9930_t* device, I2C_HandleTypeDef* i2c_handle) {
	// Set struct parameters
	device->i2c_handle = i2c_handle;

	device->lux = 0.0f;
	device->prox = 0.0f;

	Ch0_data = 0;
	Ch1_data = 0;

	IAC1 = 0.0f;
	IAC2 = 0.0f;

	IAC = 0.0f;
	LPC = 0.0f;

	// Store number of transaction errors ( to be returned at the end of the function )
	uint8_t err_num = 0;
	HAL_StatusTypeDef status;
	uint8_t reg_data;

	// ENABLE : Disable and power down (p.14)
	reg_data = 0x00;
	status = APDS9930_CMD_WriteRegister (device, APDS9930_REG_ENABLE, &reg_data);
	err_num += (status != HAL_OK);

	// Check ID register (p.23)
	status = APDS9930_CMD_ReadRegister (device, APDS9930_REG_ID, &reg_data);
	err_num += (status != HAL_OK);

	if (reg_data != APDS9930_ID_1 && reg_data != APDS9930_ID_2)	return 255;

	// ENABLE : WEN, PEN, AEN, and PON are enabled. PIEN is enabled, AIEN is disabled
//	reg_data = 0x0F;
//	reg_data = 0x2F;
//	status = APDS9930_WriteRegister (device, APDS9930_REG_ENABLE, &reg_data);
//	err_num += (status != HAL_OK);

	// ALS Timing Register : 1 cycle (2.73ms)
	reg_data = 0xff;
	status = APDS9930_CMD_WriteRegister (device, APDS9930_REG_ATIME, &reg_data);
	err_num += (status != HAL_OK);

	ALSIT = 2.73 * (float) (256 - reg_data);

	// Proximity Time Control Register : 1 cycle (2.73ms)
	reg_data = 0xff;
	status = APDS9930_CMD_WriteRegister (device, APDS9930_REG_PTIME, &reg_data);
	err_num += (status != HAL_OK);

	// Wait Time Register : 1 cycle (2.73ms)
	reg_data = 0xff;
	status = APDS9930_CMD_WriteRegister (device, APDS9930_REG_WTIME, &reg_data);
	err_num += (status != HAL_OK);

	// Proximity Interrupt Threshold : low and high threshold of 0
	reg_data = 0x00;
	status = APDS9930_CMD_WriteRegister (device, APDS9930_REG_PILTL, &reg_data);
	err_num += (status != HAL_OK);

	status = APDS9930_CMD_WriteRegister (device, APDS9930_REG_PILTH, &reg_data);
	err_num += (status != HAL_OK);

	status = APDS9930_CMD_WriteRegister (device, APDS9930_REG_PIHTL, &reg_data);
	err_num += (status != HAL_OK);

	status = APDS9930_CMD_WriteRegister (device, APDS9930_REG_PIHTH, &reg_data);
	err_num += (status != HAL_OK);

	// Persistence Register : Every proximity cycle generates interrupt
	reg_data = 0x00;
	status = APDS9930_CMD_WriteRegister (device, APDS9930_REG_PERS, &reg_data);
	err_num += (status != HAL_OK);

	// Configuration Register : AGL, WLONG, and PDL are not asserted
	reg_data = 0x00;
	status = APDS9930_CMD_WriteRegister (device, APDS9930_REG_CONFIG, &reg_data);
	err_num += (status != HAL_OK);

	AGAIN = Pow (2, reg_data >> 2);

	// Proximity Pulse Count Register : 8 pulses (recommended p.22)
	reg_data = 0x08;
	status = APDS9930_CMD_WriteRegister (device, APDS9930_REG_PPULSE, &reg_data);
	err_num += (status != HAL_OK);

	// Control Register : 100mA LED strength, Proximity uses Ch1 diode, 1x Proximity gain, 1x ALS gain
	reg_data = 0x20;
	status = APDS9930_CMD_WriteRegister (device, APDS9930_REG_CONTROL, &reg_data);
	err_num += (status != HAL_OK);

	// ENABLE : WEN, PEN, AEN, and PON are enabled. PIEN is enabled, AIEN is disabled
	reg_data = 0x2F;
	status = APDS9930_CMD_WriteRegister (device, APDS9930_REG_ENABLE, &reg_data);
	err_num += (status != HAL_OK);
	HAL_Delay(12);

	return err_num;
}

/*
 * DATA AQUISITION
 */

HAL_StatusTypeDef APDS9930_ReadLux (APDS9930_t* device) {
	uint8_t raw_data[4];
	HAL_StatusTypeDef status;

	status = APDS9930_ReadRegisters (device, APDS9930_REG_Ch0DATAL, raw_data, 4);
	Ch0_data = 256 * raw_data[1] + raw_data[0];
	Ch1_data = 256 * raw_data[3] + raw_data[2];

	IAC1 = (float) Ch0_data - B * (float ) Ch1_data;
	IAC2 = C * (float) Ch0_data - D * (float) Ch1_data;

	IAC = Max (2, IAC1, IAC2);
	LPC = GA * DF / (ALSIT * (float) AGAIN);

	device->lux = IAC * LPC;

	return status;
}

HAL_StatusTypeDef APDS9930_ReadProximity (APDS9930_t* device) {
	uint8_t raw_data[2];
	HAL_StatusTypeDef status;

	status = APDS9930_ReadRegisters (device, APDS9930_REG_PDATAL, raw_data, 2);
	device->prox = 256 * raw_data[1] + raw_data[0];

	return status;
}

/*
 * LOW-LEVEL FUNCTIONS
 */

HAL_StatusTypeDef APDS9930_ReadRegister (APDS9930_t* device, uint8_t reg, uint8_t* data) {
	return HAL_I2C_Mem_Read(device->i2c_handle, APDS9930_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef APDS9930_ReadRegisters (APDS9930_t* device, uint8_t reg, uint8_t* data, uint8_t length) {
	return HAL_I2C_Mem_Read(device->i2c_handle, APDS9930_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY);
}

HAL_StatusTypeDef APDS9930_WriteRegister (APDS9930_t* device, uint8_t reg, uint8_t* data) {
	return HAL_I2C_Mem_Write(device->i2c_handle, APDS9930_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}



HAL_StatusTypeDef APDS9930_CMD_ReadRegister (APDS9930_t* device, uint8_t reg, uint8_t* data) {
	uint8_t cmd = APDS9930_REG_COMMAND_REPEAT + reg;

	if (HAL_I2C_Master_Transmit (device->i2c_handle, APDS9930_I2C_ADDR, &cmd, 1, HAL_MAX_DELAY)
			!= HAL_OK)	return HAL_ERROR;

	return HAL_I2C_Master_Receive (device->i2c_handle, APDS9930_I2C_ADDR, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef APDS9930_CMD_ReadRegisters (APDS9930_t* device, uint8_t reg, uint8_t* data, uint8_t length) {
	uint8_t cmd = APDS9930_REG_COMMAND_AUTO_INC + reg;

	if (HAL_I2C_Master_Transmit (device->i2c_handle, APDS9930_I2C_ADDR, &cmd, 1, HAL_MAX_DELAY)
			!= HAL_OK)	return HAL_ERROR;

	return HAL_I2C_Master_Receive (device->i2c_handle, APDS9930_I2C_ADDR, data, 1, HAL_MAX_DELAY);
}

HAL_StatusTypeDef APDS9930_CMD_WriteRegister (APDS9930_t* device, uint8_t reg, uint8_t* data) {
//	uint16_t cmd = ((APDS9930_REG_COMMAND_REPEAT + reg) << 8) + (*data);

	uint8_t cmd[2] = {
			APDS9930_REG_COMMAND_REPEAT + reg,
			(*data)
	};

	return HAL_I2C_Master_Transmit (device->i2c_handle, APDS9930_I2C_ADDR, cmd, 2, HAL_MAX_DELAY);
}

/*
 * PRIVATE FUNCTIONS
 */

float Max (int num, ...) {
	float max = 0.0f;
	float temp = 0.0f;
	va_list vl;
	va_start (vl, num);

	for (int i = 0; i < num; i++) {
		temp = (float) va_arg (vl, double);
		max = temp > max ? temp : max;
	}

	return max;
}

uint8_t Pow (uint8_t base, uint8_t exp) {
	uint8_t answer = 1;

	for (int i = 0; i < exp; i++) {
		answer *= base;
	}

	return answer;
}
