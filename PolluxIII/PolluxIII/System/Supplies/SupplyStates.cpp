/*
 * SupplyStates.cpp
 *
 *  Created on: 15 Jun 2023
 *      Author: arion
 */

#include "SupplyStates.h"


const char* toString(SupplyState state) {
	switch(state) {
	case OFF:
		return "Off";
	case SWITCHING:
		return "Switching";
	case SUPPLYING:
		return "Supplying";
	case UNDERVOLTAGE:
		return "Undervoltage";
	case OVERVOLTAGE:
		return "Overvoltage";
	case OVERCURRENT:
		return "Overcurrent";
	case VIN_TOO_LOW:
		return "Input too low";
	case VIN_TOO_HIGH:
		return "Input too high";
	case NO_DAC:
		return "DAC not found";
	case DAC_FAULT:
		return "DAC fault";
	}

	return "Unknown";
}
