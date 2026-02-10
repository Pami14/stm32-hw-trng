//
// STM32 driver for I2C digital potentiometer
//
// Copyright (c) 2025 Pötscher Manuel.
//

#include "i2c-poti.h"
#include "i2c.h"

#define I2C_POTI_BASE_ADDR 32

// All times are defined using milliseconds
#define I2C_POTI_TIMEOUT 1		// Depends on bit rate. At 400kHz, 1ms should be fine

#define CTRL_WRITE 1
#define CTRL_WRITE_SERIAL 2
#define CTRL_READ 3
#define CTRL_EEPROM 7
#define RDAC_to_EEPROM 1
#define EEPROM_to_RDAC 0
#define READ_EEPROM 1
#define READ_RDAC 3


int i2c_poti_set(i2c_poti_t* poti, uint8_t RDAC, uint8_t* data) {

	return i2c_poti_write( poti, RDAC, data, CTRL_WRITE);
}


int i2c_poti_RDAC_to_eeprom(i2c_poti_t* poti, uint8_t RDAC) {

	uint8_t data = RDAC_to_EEPROM;
	uint8_t eeprom;
	uint8_t current_RDAC;

	i2c_poti_write( poti, RDAC, &data, CTRL_EEPROM );

	HAL_Delay(15);

	data = READ_EEPROM;
	eeprom = i2c_poti_read( poti, RDAC, &data, CTRL_READ );

	data = READ_RDAC;
	current_RDAC = i2c_poti_read( poti, RDAC, &data, CTRL_READ );

	if (eeprom != current_RDAC) return 1;

	return 0;
}

int i2c_poti_READ_EEPROM(i2c_poti_t* poti, uint8_t RDAC) {

	uint8_t data = READ_EEPROM;
	uint8_t eepromvalue;

	eepromvalue = i2c_poti_read( poti, RDAC, &data, CTRL_READ );

	return eepromvalue;
}

int i2c_poti_READ_RDAC(i2c_poti_t* poti, uint8_t RDAC) {

	uint8_t data = READ_RDAC;
	uint8_t RDACvalue;

	RDACvalue = i2c_poti_read( poti, RDAC, &data, CTRL_READ );

	return RDACvalue;
}

int i2c_poti_read(i2c_poti_t* poti, uint8_t RDAC, uint8_t* data, uint8_t CTRL) {

	HAL_StatusTypeDef res;
	uint8_t ComAdr = (( CTRL<< 4) | RDAC);
	uint8_t addr = (I2C_POTI_BASE_ADDR + poti->addr_offset) << 1;
	uint8_t receive = 0;

	uint8_t transmit[2] = { ComAdr, *data };

	// Read back eeprom and RDAC
	res = HAL_I2C_Master_Transmit(poti->hi2c, addr, transmit, 2, I2C_POTI_TIMEOUT);
	if (res != HAL_OK) return 1;

	res = HAL_I2C_Master_Receive(poti->hi2c, addr, &receive, 1, I2C_POTI_TIMEOUT);
	if (res != HAL_OK) return 1;

	return receive;
}

int i2c_poti_write(i2c_poti_t* poti, uint8_t RDAC, uint8_t* data, uint8_t CTRL) {
	if (poti->hi2c == NULL) return 1;

	// Transmit bitmask to poti
	HAL_StatusTypeDef res;
	uint8_t ComAdr = (( CTRL<< 4) | RDAC);
	uint8_t addr = (I2C_POTI_BASE_ADDR + poti->addr_offset) << 1;

	uint8_t transmit[2] = { ComAdr, *data };
	res = HAL_I2C_Master_Transmit(poti->hi2c, addr, transmit, 2, I2C_POTI_TIMEOUT);
	if (res != HAL_OK) return 1;

	return 0;
}
