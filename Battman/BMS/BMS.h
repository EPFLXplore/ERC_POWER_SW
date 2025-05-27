/*
 * BMS.h
 *
 *  Created on: May 15, 2025
 *      Author: otto
 */

#ifndef BMS_H_
#define BMS_H_
#include "stdint.h"
#include "stdbool.h"

#define TOTAL_IC 1	//total number of BMS ICs
extern uint32_t adcVal[2]; //Array that holds the ADC values (ADC1 and ADC2)

void readVoltages();	//reads the cell voltages
void readTemperatures();	//reads the temperatures

void LTC6811_init();	//Initializes the LTC and the SPI communication
void tempConvert();//convert ADC values into temperature
void currentConvert();//convert ADC values into current
void errorCheck();//check for overvoltage, undervoltage, overtemperature, undertemperature and communication errors
void outputControl(); //control the output based on the error flags and enable/disable charge/discharge
void LEDControl();//control the LED
void SetHardwareProtection();//set the hardware protection
bool resetOutputLatch(); //reset the output latch, returns true if successful
void balancingControl();

#endif /* BMS_H_ */
