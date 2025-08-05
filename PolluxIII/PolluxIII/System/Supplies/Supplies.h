/*
 * Supplies.h
 *
 *  Created on: May 16, 2023
 *      Author: arion
 */

#ifndef SUPPLIES_SUPPLIES_H_
#define SUPPLIES_SUPPLIES_H_


#include "MCP47FEB/MCP47FEB.h"
#include "Sensors/PowerMonitors.h"

#include <gpio.h>


struct SupplyConfig {
	const char* name;
	PowerMonitor associated_monitor;
	uint8_t address;
	uint8_t dac_reg;
	uint8_t dac_reg_nv;
	GPIO_TypeDef* gpio;
	uint16_t ctrl;
	uint16_t shdn;
	float min_vin;
	float max_vin;
	float uvlo;
	float ovlo;
	float max_power;
	uint8_t reference_voltage;
};

extern struct SupplyConfig LVASupply;
extern struct SupplyConfig LVBSupply;
extern struct SupplyConfig HVASupply;
extern struct SupplyConfig HVBSupply;


#endif /* SUPPLIES_SUPPLIES_H_ */
