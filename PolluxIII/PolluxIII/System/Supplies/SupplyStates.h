/*
 * SupplyStates.h
 *
 *  Created on: 15 Jun 2023
 *      Author: arion
 */

#ifndef SUPPLIES_SUPPLYSTATES_H_
#define SUPPLIES_SUPPLYSTATES_H_


enum SupplyState {
	OFF, SWITCHING, SUPPLYING, UNDERVOLTAGE, OVERVOLTAGE, OVERCURRENT, VIN_TOO_LOW, VIN_TOO_HIGH, NO_DAC, DAC_FAULT
};

const char* toString(SupplyState status);

#endif /* SUPPLIES_SUPPLYSTATES_H_ */
