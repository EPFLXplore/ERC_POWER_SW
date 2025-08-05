/*
 * PowerMonitors.h
 *
 *  Created on: 12 Jun 2023
 *      Author: arion
 */

#ifndef SENSORS_POWERMONITORS_H_
#define SENSORS_POWERMONITORS_H_


typedef enum {
	BATTERY,
	MOTORS,
	LVA,
	LVB,
	HVA,
	HVB
} PowerMonitor;

const char* toString(PowerMonitor monitor);

#endif /* SENSORS_POWERMONITORS_H_ */
