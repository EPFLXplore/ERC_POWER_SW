/*
 * Supplies.cpp
 *
 *  Created on: May 16, 2023
 *      Author: arion
 */

#include "Supplies.h"


struct SupplyConfig LVASupply {
	"LVAVSupply",
	LVA,
	0b11000010,		// I2C address
	MCP47_DAC0,		// I2C register
	MCP47_DAC0_NV,	// Non-volatile I2C register
	GPIOB,			// GPIO bank
	GPIO_PIN_15,	// CTRL pin
	GPIO_PIN_13,	// SHDN pin
	10.0f,			// Minimum input voltage
	30.0f,			// Maximum input voltage
	4.7f,			// Minimum output voltage
	5.3f,			// Maximum output voltage
	25.0f,			// Maximum power
	62				// Reference DAC voltage (3.3V/256)

};

struct SupplyConfig LVBSupply {
	"LVBSupply",
	LVB,
	0b11000010,		// I2C address
	MCP47_DAC1,		// I2C register
	MCP47_DAC1_NV,	// Non-volatile I2C register
	GPIOB,			// GPIO bank
	GPIO_PIN_14,	// CTRL pin
	GPIO_PIN_12,	// SHDN pin
	10.0f,			// Minimum input voltage
	30.0f,			// Maximum input voltage
	11.5f,			// Minimum output voltage
	12.5f,			// Maximum output voltage
	60.0f,			// Maximum power
	113				// Reference DAC voltage (3.3V/256)
};

struct SupplyConfig HVASupply {
	"HVASupply",
	HVA,
	0b11000000,		// I2C address
	MCP47_DAC0,		// I2C register
	MCP47_DAC0_NV,	// Non-volatile I2C register
	GPIOE,			// GPIO bank
	GPIO_PIN_5,		// CTRL pin
	GPIO_PIN_3,		// SHDN pin
	10.0f,			// Minimum input voltage
	30.0f,			// Maximum input voltage
	14.5f,			// Minimum output voltage
	15.5f,			// Maximum output voltage
	60.0f,			// Maximum power
	241				// Reference DAC voltage (3.3V/256)
};

struct SupplyConfig HVBSupply {
	"HVBSupply",
	HVB,
	0b11000000,		// I2C address
	MCP47_DAC1,		// I2C register
	MCP47_DAC1_NV,	// Non-volatile I2C register
	GPIOE,			// GPIO bank
	GPIO_PIN_4,		// CTRL pin
	GPIO_PIN_2,		// SHDN pin
	15.0f,			// Minimum input voltage
	30.0f,			// Maximum input voltage
	23.5f,			// Minimum output voltage
	24.5f,			// Maximum output voltage
	60.0f,			// Maximum power
	85				// Reference DAC voltage (3.3V/256)
};

