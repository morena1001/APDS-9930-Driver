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
#include <stdint.h>
#include <stdbool.h>

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
//	uint8_t reg_data_2;
	uint16_t reg_data_WORD;

	// ENABLE
	reg_data = APDS9930_DEFAULT_DISABLE;
	status = APDS9930_WriteRegister (device, APDS9930_REG_ENABLE, &reg_data);
	err_num += (status != HAL_OK);

	// Check ID register (p.23)
	status = APDS9930_ReadRegister (device, APDS9930_REG_ID, &reg_data);
	err_num += (status != HAL_OK);

	if (reg_data != APDS9930_ID)	return 255;

	// ALS Timing Register
	reg_data = APDS9930_DEFAULT_ATIME;
	status = APDS9930_WriteRegister (device, APDS9930_REG_ATIME, &reg_data);
	err_num += (status != HAL_OK);

	ALSIT = 2.73 * (float) (256 - reg_data);

	// Proximity Time Control Register
	reg_data = APDS9930_DEFAULT_PTIME;
	status = APDS9930_WriteRegister (device, APDS9930_REG_PTIME, &reg_data);
	err_num += (status != HAL_OK);

	// Wait Time Register
	reg_data = APDS9930_DEFAULT_WTIME;
	status = APDS9930_WriteRegister (device, APDS9930_REG_WTIME, &reg_data);
	err_num += (status != HAL_OK);

	// Proximity Interrupt Low Threshold
	// reg_data = 0x00;
	// reg_data_2 = 0x00;
	reg_data_WORD = APDS9930_DEFAULT_PILTx;
	// status = APDS9930_WORD_WriteRegister (device, APDS9930_REG_PILTL, &reg_data, &reg_data_2);
	status = APDS9930_WORD_WriteRegister_2 (device, APDS9930_REG_PILTL, &reg_data_WORD);
	err_num += (status != HAL_OK);

	// Proximity Interrupt High Threshold
	// reg_data = 0x01;
	reg_data_WORD = APDS9930_DEFAULT_PIHTx;
	// status = APDS9930_WORD_WriteRegister (device, APDS9930_REG_PIHTL, &reg_data, &reg_data_2);
	status = APDS9930_WORD_WriteRegister_2 (device, APDS9930_REG_PILTL, &reg_data_WORD);
	err_num += (status != HAL_OK);

	// Persistence Register
	reg_data = APDS9930_DEFAULT_PERS;
	status = APDS9930_WriteRegister (device, APDS9930_REG_PERS, &reg_data);
	err_num += (status != HAL_OK);

	// Configuration Register
	reg_data = APDS9930_DEFAULT_CONFIG;
	status = APDS9930_WriteRegister (device, APDS9930_REG_CONFIG, &reg_data);
	err_num += (status != HAL_OK);

	AGAIN = Pow (2, reg_data >> 2);
	LPC = GA * DF / (ALSIT * (float) AGAIN);

	// Proximity Pulse Count Register
	reg_data = APDS9930_DEFAULT_PPULSE;
	status = APDS9930_WriteRegister (device, APDS9930_REG_PPULSE, &reg_data);
	err_num += (status != HAL_OK);

	// Control Register
	reg_data = APDS9930_DEFAULT_CONTROL;
	status = APDS9930_WriteRegister (device, APDS9930_REG_CONTROL, &reg_data);
	err_num += (status != HAL_OK);

	// ENABLE
	reg_data = APDS9930_DEFAULT_ENABLE;
	status = APDS9930_WriteRegister (device, APDS9930_REG_ENABLE, &reg_data);
	err_num += (status != HAL_OK);
	HAL_Delay(12);

	return err_num;
}

/*
 * CONFIGURATION
 */
HAL_StatusTypeDef APDS9930_ENABLE_Write (APDS9930_t* device, bool SAI, bool PIEN, bool AIEN, bool WEN, bool PEN, bool AEN, bool PON) {
	uint8_t data = APDS9930_REG_ENABLE_SAI (SAI) + 
				   APDS9930_REG_ENABLE_PIEN (PIEN) + 
				   APDS9930_REG_ENABLE_AIEN (AIEN) + 
				   APDS9930_REG_ENABLE_WEN (WEN) + 
				   APDS9930_REG_ENABLE_PEN (PEN) + 
				   APDS9930_REG_ENABLE_AEN (AEN) + 
				   APDS9930_REG_ENABLE_PON (PON);
	
	return APDS9930_WriteRegister (device, APDS9930_REG_ENABLE, &data);
}

HAL_StatusTypeDef APDS9930_ATIME_Write (APDS9930_t* device, uint8_t cycles) {
	uint8_t data = APDS9930_REG_ATIME_CYCLES (cycles);

	ALSIT = 2.73 * (float) (256 - data);

	return APDS9930_WriteRegister (device, APDS9930_REG_ATIME, &data);
}

HAL_StatusTypeDef APDS9930_PTIME_Write (APDS9930_t* device, uint8_t cycles) {
	uint8_t data = APDS9930_REG_PTIME_CYCLES (cycles);

	return APDS9930_WriteRegister (device, APDS9930_REG_PTIME, &data);
}

HAL_StatusTypeDef APDS9930_WTIME_Write (APDS9930_t* device, uint8_t cycles) {
	uint8_t data = APDS9930_REG_WTIME_TIME (cycles);

	return APDS9930_WriteRegister (device, APDS9930_REG_WTIME, &data);
}

HAL_StatusTypeDef APDS9930_AILTx_Write (APDS9930_t* device, uint16_t threshold) {
	return APDS9930_WORD_WriteRegister_2 (device, APDS9930_REG_AILTL, &threshold);
}

HAL_StatusTypeDef APDS9930_AIHTx_Write (APDS9930_t* device, uint16_t threshold) {
	return APDS9930_WORD_WriteRegister_2 (device, APDS9930_REG_AIHTL, &threshold);
}

HAL_StatusTypeDef APDS9930_PILTx_Write (APDS9930_t* device, uint16_t threshold) {
	return APDS9930_WORD_WriteRegister_2 (device, APDS9930_REG_PILTL, &threshold);
}

HAL_StatusTypeDef APDS9930_PIHTx_Write (APDS9930_t* device, uint16_t threshold) {
	return APDS9930_WORD_WriteRegister_2 (device, APDS9930_REG_PIHTL, &threshold);
}

HAL_StatusTypeDef APDS9930_PERS_Write (APDS9930_t* device, uint8_t PPERS, uint8_t APERS) {
	uint8_t data = APDS9930_REG_PERS_PPERS (PPERS) + 
	   			   APDS9930_REG_PERS_APERS (APERS);

	return APDS9930_WriteRegister (device, APDS9930_REG_PERS, &data);
}

HAL_StatusTypeDef APDS9930_CONFIG_Write (APDS9930_t* device, bool AGL, bool WLONG, bool PDL) {
	uint8_t data = APDS9930_REG_CONFIG_AGL (AGL) +
				   APDS9930_REG_CONFIG_WLONG (WLONG) + 
				   APDS9930_REG_CONFIG_PDL (PDL);

	AGAIN = Pow (2, data >> 2);
	LPC = GA * DF / (ALSIT * (float) AGAIN);

	return APDS9930_WriteRegister (device, APDS9930_REG_CONFIG, &data);
}

HAL_StatusTypeDef APDS9930_PPULSE_Write (APDS9930_t* device, uint8_t PPULSE) {
	return APDS9930_WriteRegister (device, APDS9930_REG_PPULSE, &PPULSE);
}

HAL_StatusTypeDef APDS9930_CONTROL_Write (APDS9930_t* device, uint8_t PDRIVE, uint8_t PDIODE, uint8_t PGAIN, uint8_t AGAIN) {
	uint8_t data = APDS9930_REG_CONTROL_PDRIVE (PDRIVE) +
				   APDS9930_REG_CONTROL_PDIODE (PDIODE) + 
				   APDS9930_REG_CONTROL_PGAIN (PGAIN) +
				   APDS9930_REG_CONTROL_AGAIN (AGAIN);

	return APDS9930_WriteRegister (device, APDS9930_REG_CONTROL, &data);
}

HAL_StatusTypeDef APDS9930_POFFSET_Write (APDS9930_t* device, uint8_t SIGN, uint8_t MAGNITUDE) {
	uint8_t data = APDS9930_REG_POFFSET_SIGN (SIGN) + 
				   APDS9930_REG_POFFSET_MAGNITUDE (MAGNITUDE);

	return APDS9930_WriteRegister (device, APDS9930_REG_POFFSET, &data);
}

/*
 * DATA AQUISITION
 */

HAL_StatusTypeDef APDS9930_ReadLux (APDS9930_t* device) {
	uint8_t Ch0_raw[2];
	uint8_t Ch1_raw[2];
	HAL_StatusTypeDef status;

	status = APDS9930_WORD_ReadRegister (device, APDS9930_REG_Ch0DATAL, Ch0_raw);
	if (status != HAL_OK)	return status;
	status = APDS9930_WORD_ReadRegister (device, APDS9930_REG_Ch1DATAL, Ch1_raw);
	if (status != HAL_OK)	return status;

	Ch0_data = 256 * Ch0_raw[1] + Ch0_raw[0];
	Ch1_data = 256 * Ch1_raw[1] + Ch1_raw[0];

	IAC1 = (float) Ch0_data - B * (float) Ch1_data;
	IAC2 = C * (float) Ch0_data - D * (float) Ch1_data;

	IAC = Max (2, IAC1, IAC2);

	device->lux = IAC * LPC;

	return status;
}

HAL_StatusTypeDef APDS9930_ReadProximity (APDS9930_t* device) {
	uint8_t raw[2];
	HAL_StatusTypeDef status;

	status = APDS9930_WORD_ReadRegister (device, APDS9930_REG_PDATAL, raw);
	device->prox = 256 * raw[1] + raw[0];

	return status;
}

/*
 * LOW-LEVEL FUNCTIONS
 */

HAL_StatusTypeDef APDS9930_ReadRegister (APDS9930_t* device, uint8_t reg, uint8_t* data) {
	uint8_t cmd = APDS9930_REG_COMMAND_REPEAT + reg;

	if (HAL_I2C_Master_Transmit (device->i2c_handle, APDS9930_I2C_ADDR, &cmd, 1, HAL_MAX_DELAY)
			!= HAL_OK)	return HAL_ERROR;

	return HAL_I2C_Master_Receive (device->i2c_handle, APDS9930_I2C_ADDR, data, 1, HAL_MAX_DELAY);
}



HAL_StatusTypeDef APDS9930_WORD_ReadRegister (APDS9930_t* device, uint8_t reg, uint8_t* data) {
	uint8_t cmd = APDS9930_REG_COMMAND_REPEAT + reg;

	if (HAL_I2C_Master_Transmit (device->i2c_handle, APDS9930_I2C_ADDR, &cmd, 1, HAL_MAX_DELAY)
			!= HAL_OK)	return HAL_ERROR;

	return HAL_I2C_Master_Receive (device->i2c_handle, APDS9930_I2C_ADDR, data, 2, HAL_MAX_DELAY);
}



HAL_StatusTypeDef APDS9930_WriteRegister (APDS9930_t* device, uint8_t reg, uint8_t* data) {
	uint8_t cmd[2] = {
			APDS9930_REG_COMMAND_REPEAT + reg,
			(*data)
	};

	return HAL_I2C_Master_Transmit (device->i2c_handle, APDS9930_I2C_ADDR, cmd, 2, HAL_MAX_DELAY);
}



HAL_StatusTypeDef APDS9930_CLI_WriteRegister (APDS9930_t* device, uint8_t command) {
	// uint8_t cmd = command;

	return HAL_I2C_Master_Transmit (device->i2c_handle, APDS9930_I2C_ADDR, &command, 1, HAL_MAX_DELAY);
}



HAL_StatusTypeDef APDS9930_WORD_WriteRegister (APDS9930_t* device, uint8_t reg, uint8_t* data_low, uint8_t* data_high) {
	uint8_t cmd[3] = {
			APDS9930_REG_COMMAND_AUTO_INC + reg,
			(*data_low),
			(*data_high)
	};

	return HAL_I2C_Master_Transmit (device->i2c_handle, APDS9930_I2C_ADDR, cmd, 3, HAL_MAX_DELAY);
}



HAL_StatusTypeDef APDS9930_WORD_WriteRegister_2 (APDS9930_t* device, uint8_t reg, uint16_t* data) {
	uint8_t cmd[3] = {
			APDS9930_REG_COMMAND_AUTO_INC + reg,
			(*data) & 15,
			(*data) >> 4
	};

	return HAL_I2C_Master_Transmit (device->i2c_handle, APDS9930_I2C_ADDR, cmd, 3, HAL_MAX_DELAY);
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
