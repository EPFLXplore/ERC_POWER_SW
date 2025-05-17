/*
 * BMS.c
 * Contains the BMs Logic
 *
 *  Created on: May 15, 2025
 *      Author: otto
 */

#include "BMS.h"
#include "LTC6811.h"
#include "LT_SPI.h"
#include "LTC681x.h"
#include "config.h"
#include <math.h>
#include <string.h>

cell_asic bms_ic[TOTAL_IC];	//the cell_asic struct objects

float voltages[TOTAL_IC][CellsNbS];	//holds the converted voltages of each cell
float temperatures[TOTAL_IC][NbTherm];	//holds the Temperatures
float currents[2]; //holds the current from both sensors
uint32_t adcVal[2]; //Array that holds the ADC values (ADC1 and ADC2)

//LTC CONFIGURATION VARIABLES
bool REFON = true; //!< Reference Powered Up Bit (true means Vref remains powered on between conversions)
bool ADCOPT = true; //!< ADC Mode option bit	(true chooses the second set of ADC frequencies)
bool gpioBits_a[5] = {false,false,false,false,false}; //!< GPIO Pin Control // Gpio 1,2,3,4,5 (false -> pull-down on)
bool dccBits_a[12] = {false,false,false,false,false,false,false,false,false,false,false,false}; //!< Discharge cell switch //Dcc 1,2,3,4,5,6,7,8,9,10,11,12 (all false -> no discharge enabled)
bool dctoBits_a[4] = {false, false, false, false}; //!< Discharge time value // Dcto 0,1,2,3	(all false -> discharge timer disabled)
uint16_t uv_a = 1000*MinDschgVolt; // The UV register
uint16_t  ov_a = 1000*ChgEndVolt;// The OV register
uint8_t N_Error = 10;	//number of allowed consecutive errors (To be Optimised)

//ERROR COUNTERS
int8_t cvError = 0,auxError = 0;	//hold if an error has occured while reading cell voltage and aux voltage values
int NOV [TOTAL_IC][CellsNbS];	//overvoltage error integrator per IC per cell
int NUV [TOTAL_IC][CellsNbS];	//undervoltage error integrator per IC per cell
int NOT [TOTAL_IC][NbTherm];	//overtemperature error integrator per IC per sensor
int NUT [TOTAL_IC][NbTherm];	//undertemperature error integrator per IC per sensor
int NC[TOTAL_IC];				//communication error integrator per IC
int NOC [2];					//over current error integrator per Sensor
//ERROR FLAGS
bool tempError = false;	        //holds if an error has occured while reading temperature values or out of range temperature(Yellow LED)
bool voltageError = false;		//holds if an error has occured while reading cell voltage values or out of range voltage(Red LED)
bool currentError = false;		//holds if an error has occured while reading current values or out of range current(Amber LED)
bool chargeEnable = false;		//holds if conditions are met to enable charge
bool dischargeEnable = false;	//holds if conditions are met to enable discharge
bool System_OK = true;			//holds if the MCU and BMS_ICs are OK, turned off by watchdog, Comm errors or failed BMS Selftests(Green LED)


//Initializes the LTC's registers and the SPI communication
void LTC6811_init(){
	//LTC6811_Initialize();	//Initializes the SPI communication at 1MHz
	LTC6811_init_cfg(TOTAL_IC, bms_ic);	//Initializes the confiugration registers to all 0s
	//This for loop initializes the configuration register variables
	for (uint8_t current_ic = 0; current_ic<TOTAL_IC;current_ic++){
		LTC6811_set_cfgr(current_ic,bms_ic,REFON,ADCOPT,gpioBits_a,dccBits_a,dctoBits_a,uv_a,ov_a); // write LTC config like defined above
    }
	LTC6811_reset_crc_count(TOTAL_IC,bms_ic);	//sets the CRC count to 0
	LTC6811_init_reg_limits(TOTAL_IC, bms_ic);	//Initializes the LTC's register limits for LTC6811 (because the generic LTC681x libraries can also be used for LTC6813 and others)
	wakeup_sleep(TOTAL_IC);
	LTC6811_wrcfg(TOTAL_IC,bms_ic);	//writes the configuration variables in the configuration registers via SPI
}
//convert ADC values into temperature
void tempConvert(){
	float innerlog;
	for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++){
		for(int sensor = 0; sensor < NbTherm; sensor++){
			if(bms_ic[current_ic].aux.a_codes[AUX_CH_VREF2-1] == 0x00){
				temperatures[current_ic][sensor] = 7;
				continue;
			}
			innerlog = (((float)bms_ic[current_ic].aux.a_codes[sensor]/(float)bms_ic[current_ic].aux.a_codes[AUX_CH_VREF2-1])-1.0)*ThermRs/ThermR25;
			temperatures[current_ic][sensor] = 1/((1/ThermB)*log(innerlog)+1/(298.15))-273.15;
		}
	}
}

void readVoltages(){
	cvError = LTC6811_rdcv(CELL_CH_ALL, TOTAL_IC, bms_ic); // Reads and parses the LTC6811 cell voltage registers.
//		uint8_t LTC6811_rdcv(uint8_t reg, // Controls which cell voltage register is read back.
//		                     uint8_t total_ic, // the number of ICs in the system
//		                     cell_asic ic[] // Array of the parsed cell codes
//		                    )
	for(int i=0; i<TOTAL_IC; i++){
		for(int j=0; j<CellsNbS; j++){
			voltages[i][j] = (float)bms_ic[i].cells.c_codes[j] * 0.0001; //convert the cell voltage codes to voltages
		}
	}
}
void readTemperatures(){
	auxError = LTC6811_rdaux(AUX_CH_ALL, TOTAL_IC, bms_ic);
}
//convert ADC values into current sensor skaling 19.8mV/A
void currentConvert(){
	for(int i=0;i<2;i++){
		currents[i] = adcVal[i]*0.0406901041667;	//19.8mV/A
	}
}

void errorCheck(){
	//Check for overvoltage or undervoltage and increase the counting arrays accordingly
	for (int i = 0; i < TOTAL_IC; i++){
		for (int j = 0; j < CellsNbS; j++){
			if(voltages[i][j] > ChgEndVolt){
				NOV[i][j]++;
			}else if(NOV[i][j]>0){
				NOV[i][j]--;
			}
			if(voltages[i][j] < MinDschgVolt){
				NUV[i][j]++;
			}else if(NUV[i][j]>0){
				NUV[i][j]--;
			}
		}
		//Check for overtemperature or undertemperature and increase the counting arrays accordingly
		for (int j = 0; j < NbTherm; j++){
			if(temperatures[i][j] > OverTemp){
				NOT[i][j]++;
			}else if(NOT[i][j]>0){
				NOT[i][j]--;
			}
			if(temperatures[i][j] < ChgUnderTemp){
				NUT[i][j]++;
			}else if(NUT[i][j]>0){
				NUT[i][j]--;
			}
		}

		//check for communication errors
		if(cvError>0 || auxError>0){
			NC[i]++;
		}
	}
	//Check for overcurrent and increase the counting arrays accordingly
	if(currents[0] > ChgOCP){
		NOC[0]++;
	}else if(NOC[0] > 0){
		NOC[0]--;
	}
	if(currents[1] > DschgOCP){
		NOC[1]++;
	}else if(NOC[1] > 0){
		NOC[1]--;
	}
	//Output control
	for (int i = 0; i < TOTAL_IC; i++){
		for (int j = 0; j < CellsNbS; j++){
			if(NOV[i][j] > N_Error){
				chargeEnable = false;
				voltageError = true;
			}
			if(NUV[i][j] > N_Error){
				dischargeEnable = false;
				voltageError = true;
			}
			if(NOT[i][j] > N_Error){
				chargeEnable = false;
				dischargeEnable = false;
				tempError = true;
			}
			if(NUT[i][j] > N_Error){
				chargeEnable = false;
				tempError = true;
			}
		}
	}
	if(NOC[0] > N_Error){
		chargeEnable = false;
		currentError = true;
	}
	if(NOC[1] > N_Error){
		dischargeEnable = false;
		currentError = true;
	}
}

void LEDControl(){
	//blink Red and Amber in case of hardware fault
	if(HAL_GPIO_ReadPin(HardwareFault_GPIO_Port, HardwareFault_Pin) == GPIO_PIN_RESET){
		HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
		HAL_GPIO_TogglePin(LED_A_GPIO_Port, LED_A_Pin);
	}else{
		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, voltageError);
		HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, currentError);
		HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, tempError);
	}

	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, System_OK);
	osDelay(800);
}

void SetHardwareProtection(){
	// HardwareOVP
    HAL_GPIO_WritePin(A1_1_GPIO_Port, A1_1_Pin, (HardwareOVP>>3)&1);
    HAL_GPIO_WritePin(A0_1_GPIO_Port, A0_1_Pin, (HardwareOVP>>2)&1);
    HAL_GPIO_WritePin(A1_2_GPIO_Port, A1_2_Pin, (HardwareOVP>>1)&1);
    HAL_GPIO_WritePin(A0_2_GPIO_Port, A0_2_Pin, HardwareOVP&1);
    //HardwareUVP
    HAL_GPIO_WritePin(A1_3_GPIO_Port, A1_3_Pin, (HardwareUVP>>3)&1);
    HAL_GPIO_WritePin(A0_3_GPIO_Port, A0_3_Pin, (HardwareUVP>>2)&1);
    HAL_GPIO_WritePin(A1_4_GPIO_Port, A1_4_Pin, (HardwareUVP>>1)&1);
    HAL_GPIO_WritePin(A0_4_GPIO_Port, A0_4_Pin, HardwareUVP&1);
    //Cell_CNT
    HAL_GPIO_WritePin(A1_5_GPIO_Port, A1_5_Pin, (HardwareCellCnt>>3)&1);
    HAL_GPIO_WritePin(A0_5_GPIO_Port, A0_5_Pin, (HardwareCellCnt>>2)&1);
    HAL_GPIO_WritePin(A1_6_GPIO_Port, A1_6_Pin, (HardwareCellCnt>>1)&1);
    HAL_GPIO_WritePin(A0_6_GPIO_Port, A0_6_Pin, HardwareCellCnt&1);
    //Hysterysis
    HAL_GPIO_WritePin(A0_7_GPIO_Port, A1_7_Pin, (HardwareHysterysis>>1)&1);
    HAL_GPIO_WritePin(A1_7_GPIO_Port, A0_7_Pin, (HardwareHysterysis)&1);
    //HardwareFault
    HAL_GPIO_WritePin(A1_8_GPIO_Port, A1_8_Pin, (HardwareCycleTime>>1)&1);
    HAL_GPIO_WritePin(A0_8_GPIO_Port, A0_8_Pin, (HardwareCycleTime)&1);
}

void balancingControl(){
	if(currents[0]>ChgEndCurr){
		//determine deltaV and highest cell
		float maxVoltage = 0;
		float minVoltage = 10;
		float maxCell = 0;
		float minCell = 0;
//		float deltaV = 0;
		for(int i=0;i<CellsNbS;i++){
			if(voltages[0][i]>maxVoltage){
				maxVoltage = voltages[0][i];
				maxCell = i;
			}
			if (voltages[0][i]<minVoltage){
				minVoltage = voltages[0][i];
				minCell = i;
			}
		}
		for(int i=0;i<CellsNbS;i++){
			if((voltages[0][i]-minVoltage)>MaxDisbal){
				dccBits_a[i] = true;	//enable balancing for this cell
			}
		}
		//write the balancing bits to the LTC6811
		LTC6811_set_cfgr_dis(TOTAL_IC, bms_ic, dccBits_a);
	}
}
