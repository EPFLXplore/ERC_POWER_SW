/*
 * PowerMonitors.cpp
 *
 *  Created on: 14 Jun 2023
 *      Author: arion
 */

#include "PowerMonitors.h"

const char* toString(PowerMonitor monitor) {
	switch(monitor) {
	case BATTERY:
		return "Battery";
	case MOTORS:
		return "Motors";
	case LVA:
		return "LVA";
	case LVB:
		return "LVB";
	case HVA:
		return "HVA";
	case HVB:
		return "HVB";
	}

	return "Unknown";
}
