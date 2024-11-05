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
#define APDS9930_ID				0x39

#define APDS9930_ID_1			0x39
#define APDS9930_ID_2			0x12

/*
 * REGISTERS p.18
 */

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
 * REGISTER CONFIGURATION SHORTCUTS
 */

#define APDS9930_REG_COMMAND_REPEAT				0x80 // Use with APDS9930_CLI_WriteRegister ()
#define APDS9930_REG_COMMAND_AUTO_INC			0xA0 // Use with APDS9930_CLI_WriteRegister ()
#define APDS9930_REG_COMMAND_PROX_CLI			0XE5 // Use with APDS9930_CLI_WriteRegister ()
#define APDS9930_REG_COMMAND_ALS_CLI			0XE6 // Use with APDS9930_CLI_WriteRegister ()
#define APDS9930_REG_COMMAND_CLI				0xE7 // Use with APDS9930_CLI_WriteRegister ()

#define APDS9930_REG_ENABLE_SAI(val)			(val << 6) // Sleep after interrupt
#define APDS9930_REG_ENABLE_PIEN(val)			(val << 5) // Proximity interrupts
#define APDS9930_REG_ENABLE_AIEN(val)			(val << 4) // ALS interrupts
#define APDS9930_REG_ENABLE_WEN(val)			(val << 3) // Wait timer
#define APDS9930_REG_ENABLE_PEN(val)			(val << 2) // Proximity function
#define APDS9930_REG_ENABLE_AEN(val)			(val << 1) // ALS function
#define APDS9930_REG_ENABLE_PON(val)			(val) 	   // Power ON

// Control integration time of ALS Ch0 and Ch1 ADCs in 2.73 ms increments
#define APDS9930_REG_ATIME_CYCLES(val)			(val < 1 && val > 256 ? 255 : (val * -1) + 256)

// Control integration time of proxitmity ADS in 2.73 ms increments. 1 cycle is recommended
#define APDS9930_REG_PTIME_CYCLES(val)			(val < 1 && val > 256 ? 255 : (val * -1) + 256)

// Wait time set in 2.73 ms increments if WLONG is not asserted, else wait time is 12x longer. Programmed as a 2's complement number
// Should be configured before either PEN and AEN are asserted
#define APDS9930_REG_WTIME_TIME(val)			(val < 1 && val > 256 ? 255 : (val * -1) + 256)

#define APDS9930_REG_PERS_PPERS(val)			(val < 0 && val > 15 ? 0 : val << 4) // Proximity interrupt persistence
#define APDS9930_REG_PERS_APERS(val)			(val < 0 && val > 15 ? 0 : val) // ALS interrupt persistence

#define APDS9930_REG_CONFIG_AGL(val)			(val << 2) // ALS gain level. DO NOT use with AGAIN > 8x
#define APDS9930_REG_CONFIG_WLONG(val)			(val << 1) // Wait long. Increases wait cycle by 12x
#define APDS9930_REG_CONFIG_PDL(val)			(val) // Proximity driver level. Reduces current by 9

#define APDS9930_REG_CONTROL_PDRIVE(val)		(val < 0 && val > 3 ? 0 : val << 6) // LED Drive strength
#define APDS9930_REG_CONTROL_PDIODE(val)		(val < 0 && val > 3 ? 0 : val << 4) // Proximity Diode Select
#define APDS9930_REG_CONTROL_PGAIN(val)		(val < 0 && val > 3 ? 0 : val << 2) // Proximity Gain Control
#define APDS9930_REG_CONTROL_AGAIN(val)		(val < 0 && val > 3 ? 0 : val) // ALS Gain Control

#define APDS9930_REG_POFFSET_SIGN(val)			(val < 0 && val > 1 ? 0 : val << 7) // Proximity offset sign
#define APDS9930_REG_POFFSET_MAGNITUDE(val)	(val < 0 && val > 127 ? 0 : val) // Proximity offset magnitude

/*
 * DEFAULT VALUES 
 */

#define APDS9930_DEFAULT_DISABLE		0x00 // Disable and power down (p.14)
#define APDS9930_DEFAULT_ATIME			0xff // 1 cycle (2.73 ms)
#define APDS9930_DEFAULT_PTIME			0xff // 1 cycle (2.73 ms)
#define APDS9930_DEFAULT_WTIME			0xff // 1 cycle (2.73 ms)
#define APDS9930_DEFAULT_PILTx			0x0000 // Low threshold of 0
#define APDS9930_DEFAULT_PIHTx			0x0032 // High threshold of 1
#define APDS9930_DEFAULT_PERS			0x00 // Every proximity cycle generates an interrupt
#define APDS9930_DEFAULT_CONFIG			0x00 // AGL, WLONG, and PDL are not asserted
#define APDS9930_DEFAULT_PPULSE			0x08 // 8 pulses (recommended in p.22)
#define APDS9930_DEFAULT_CONTROL		0x20 // 100mA LED strength, proximity uses Ch1 diode, 1x proximity gain, 1x ALS gain
#define APDS9930_DEFAULT_ENABLE			0x2F // PIEN, WEN, PEN, AEN, and PON are enabled. SAI and AIEN are disabled


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

uint8_t APDS9930_Init (APDS9930_t* device, I2C_HandleTypeDef* i2c_handle); // Outputs possible error codes

/*
 * CONFIGURATION
 */

HAL_StatusTypeDef APDS9930_ENABLE_Write (APDS9930_t* device, bool SAI, bool PIEN, bool AIEN, bool WEN, bool PEN, bool AEN, bool PON);
HAL_StatusTypeDef APDS9930_ATIME_Write (APDS9930_t* device, uint8_t cycles);
HAL_StatusTypeDef APDS9930_PTIME_Write (APDS9930_t* device, uint8_t cycles);
HAL_StatusTypeDef APDS9930_WTIME_Write (APDS9930_t* device, uint8_t cycles);
HAL_StatusTypeDef APDS9930_AILTx_Write (APDS9930_t* device, uint16_t threshold);
HAL_StatusTypeDef APDS9930_AIHTx_Write (APDS9930_t* device, uint16_t threshold);
HAL_StatusTypeDef APDS9930_PILTx_Write (APDS9930_t* device, uint16_t threshold);
HAL_StatusTypeDef APDS9930_PIHTx_Write (APDS9930_t* device, uint16_t threshold);
HAL_StatusTypeDef APDS9930_PERS_Write (APDS9930_t* device, uint8_t PPERS, uint8_t APERS);
HAL_StatusTypeDef APDS9930_CONFIG_Write (APDS9930_t* device, bool AGL, bool WLONG, bool PDL);
HAL_StatusTypeDef APDS9930_PPULSE_Write (APDS9930_t* device, uint8_t PPULSE);
HAL_StatusTypeDef APDS9930_CONTROL_Write (APDS9930_t* device, uint8_t PDRIVE, uint8_t PDIODE, uint8_t PGAIN, uint8_t AGAIN);
HAL_StatusTypeDef APDS9930_POFFSET_Write (APDS9930_t* device, uint8_t SIGN, uint8_t MAGNITUDE);

/*
 * DATA AQUISITION
 */

HAL_StatusTypeDef APDS9930_ReadLux (APDS9930_t* device);
HAL_StatusTypeDef APDS9930_ReadProximity (APDS9930_t* device);

/*
 * LOW LEVEL FUNCTIONS
 */
HAL_StatusTypeDef APDS9930_ReadRegister (APDS9930_t* device, uint8_t reg, uint8_t* data);
HAL_StatusTypeDef APDS9930_WORD_ReadRegister(APDS9930_t* device, uint8_t reg, uint8_t* data);
HAL_StatusTypeDef APDS9930_WriteRegister (APDS9930_t* device, uint8_t reg, uint8_t* data);
HAL_StatusTypeDef APDS9930_CLI_WriteRegister (APDS9930_t* device, uint8_t command);
HAL_StatusTypeDef APDS9930_WORD_WriteRegister (APDS9930_t* device, uint8_t reg, uint8_t* data_low, uint8_t* data_high);
HAL_StatusTypeDef APDS9930_WORD_WriteRegister_2 (APDS9930_t* device, uint8_t reg, uint16_t* data);

#endif /* INC_APDS9930_H_ */
