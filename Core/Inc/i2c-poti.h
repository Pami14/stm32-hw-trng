//
// STM32 driver for I2C digital potentiometer
//
// Copyright (c) 2025 Pötscher Manuel.
//

#ifndef I2C_POTI_H
#define I2C_POTI_H


#include "stm32f4xx_hal.h" // Can be replaced by stm32xxxx_hal.h

typedef struct i2c_poti {
	I2C_HandleTypeDef* hi2c;	// I2C bus controller
	uint8_t addr_offset;		// Offset from base address (set using address pins)
} i2c_poti_t;

// Returns 0_ on success, 1 on error.
int i2c_poti_set(i2c_poti_t* poti, uint8_t RDAC, uint8_t* data);


// Returns 0 on success, 1 on error.
int i2c_poti_RDAC_to_eeprom(i2c_poti_t* poti, uint8_t RDAC);


// Returns 0 on success, 1 on error.
int i2c_poti_READ_EEPROM(i2c_poti_t* poti, uint8_t RDAC);

// Returns 0 on success, 1 on error.
int i2c_poti_READ_RDAC(i2c_poti_t* poti, uint8_t RDAC);

int i2c_poti_write(i2c_poti_t* poti, uint8_t RDAC, uint8_t* data, uint8_t CTRL);

int i2c_poti_read(i2c_poti_t* poti, uint8_t RDAC, uint8_t* data, uint8_t CTRL);

#endif
