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
#include "gpio.h"
#include "adc.h"

cell_asic bms_ic[TOTAL_IC];	//the cell_asic struct objects

float voltages[TOTAL_IC][CellsNbS];	//holds the converted voltages of each cell
float temperatures[TOTAL_IC][NbTherm];	//holds the Temperatures
float currents[2]; //holds the current from both sensors

int32_t adc1Val, adc1Offset; //holds the ADC value for current sensor 1 and the offset
int32_t adc2Val, adc2Offset; //holds the ADC value for current sensor 2 and the offset

//LTC CONFIGURATION VARIABLES
bool REFON = true; //!< Reference Powered Up Bit (true means Vref remains powered on between conversions)
bool ADCOPT = true; //!< ADC Mode option bit	(true chooses the second set of ADC frequencies)
bool gpioBits_a[5] = {true,true,true,false,false}; //!< GPIO Pin Control // Gpio 1,2,3,4,5 (true -> pull-down off, false -> pull-down on)
bool dccBits_a[12] = {false,false,false,false,false,false,false,false,false,false,false,false}; //!< Discharge cell switch //Dcc 1,2,3,4,5,6,7,8,9,10,11,12 (all false -> no discharge enabled)
bool dctoBits_a[4] = {false, false, false, false}; //!< Discharge time value // Dcto 0,1,2,3	(all false -> discharge timer disabled)
uint16_t uv_a = 1000*MinDschgVolt; // The UV register
uint16_t  ov_a = 1000*ChgEndVolt;// The OV register


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
bool currentError = false;		//holds if an error has occurred while reading current values or out of range current(Amber LED)
bool chargeEnable = false;		//holds if conditions are met to enable charge
bool dischargeEnable = false;	//holds if conditions are met to enable discharge
bool system_OK = true;			//holds if the MCU and BMS_ICs are OK, turned off by watchdog, Comm errors or failed BMS Selftests(Green LED)


//Initializes the LTC's registers and the SPI communication, initialize STM SPI before
void LTC6811_init(){
	LTC6811_init_cfg(TOTAL_IC, bms_ic);	//Initializes the configuration registers to all 0s
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
	for (int current_ic = 0; current_ic < TOTAL_IC; current_ic++){
		for(int sensor = 0; sensor < NbTherm; sensor++){
			if(bms_ic[current_ic].aux.a_codes[AUX_CH_VREF2-1] == 0x00){
				temperatures[current_ic][sensor] = 7;
				continue;
			}
			// 1(1/B * ln(R/Ro) + (1/To))
		  	temperatures[current_ic][sensor] = 1.0 / (log((ThermRs*bms_ic[current_ic].aux.a_codes[sensor] / (bms_ic[current_ic].aux.a_codes[AUX_CH_VREF2-1] - bms_ic[current_ic].aux.a_codes[sensor])) / ThermR25) / ThermB + 1.0 / 298.15) - 273.15;
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


void adcOffsetZero(){
	//This function is used to zero the ADC offset, it is called once at the beginning of the program
	//It sets the adc1Val and adc2Val to 2048, which is the middle of the ADC range
	adc1Offset=adc2Offset=0;
	for(int i = 0; i < 10; i++){ // Take 10 samples to average the offset
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY); // Poll for conversion to complete
		adc1Offset += HAL_ADC_GetValue(&hadc1); // Read the ADC value for current sensor 2
		HAL_ADC_Start(&hadc2);
		HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY); // Poll for conversion to complete
		adc2Offset += HAL_ADC_GetValue(&hadc2); // Read the ADC value for current sensor 2
	}
	adc1Offset /= 10; // Average the offset
	adc2Offset /= 10; // Average the offset
}
//convert ADC values into current sensor skaling 19.8mV/A, ->1 LSB = 40.6901041667mA
void currentConvert(){
	currents[0] = (float)(adc1Val-adc1Offset)*0.0050863;///12.3;//*0.0406901041667;	//19.8mV/A
	currents[1] = (float)(adc2Val-adc2Offset)*0.0050863;///12.3;//*0.0406901041667;	//19.8mV/A
}

bool resetOutputLatch(){ //as this is a low active RS latch, the output is reset by setting the pin to low
	//check if error is still present, as in this case reseting the latch will put the system in undefined state
	HAL_GPIO_WritePin(ErrorReset_GPIO_Port, ErrorReset_Pin, GPIO_PIN_SET);
	if(HAL_GPIO_ReadPin(HardwareFault_GPIO_Port, HardwareFault_Pin) == GPIO_PIN_RESET){
//		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);	//turn on red LED
		return false;	//if the hardware fault pin is low, the latch cannot be reset
	}else{
		HAL_GPIO_WritePin(ErrorReset_GPIO_Port, ErrorReset_Pin, GPIO_PIN_RESET);
		HAL_Delay(10);	//wait for 10ms to ensure the latch is reset
		HAL_GPIO_WritePin(ErrorReset_GPIO_Port, ErrorReset_Pin, GPIO_PIN_SET);
		if(HAL_GPIO_ReadPin(OutputEnable_GPIO_Port, OutputEnable_Pin) == GPIO_PIN_SET){
//			HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, GPIO_PIN_SET);	//turn on yellow LED
			return true;	//output latch is reset
		}else{
//			HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);	//turn on green LED
			return false;	//output latch is not reset
		}
	}
}

void errorCheck(){
	//Reset all status variables, as they can only be flipped to the negative during check
	tempError = false;
	voltageError = false;
	currentError = false;
	chargeEnable = true;
	dischargeEnable = true;
	system_OK = true;
	//Check for overvoltage or undervoltage and increase the counting arrays accordingly
	for (int i = 0; i < TOTAL_IC; i++){
		for (int j = 0; j < CellsNbS; j++){
			if(voltages[i][j] > ChgEndVolt && NOV[i][j] < Max_Errors){
				NOV[i][j]++;
			}else if(NOV[i][j]>0){
				NOV[i][j]--;
			}
			if(voltages[i][j] < MinDschgVolt && NUV[i][j] < Max_Errors){
				NUV[i][j]++;
			}else if(NUV[i][j]>0){
				NUV[i][j]--;
			}
		}
		//Check for overtemperature or undertemperature and increase the counting arrays accordingly
		for (int j = 0; j < NbTherm; j++){
			if(temperatures[i][j] > OverTemp && NOT[i][j] < Max_Errors){
				NOT[i][j]++;
			}else if(NOT[i][j]>0){
				NOT[i][j]--;
			}
			if(temperatures[i][j] < ChgUnderTemp && NUT[i][j] < Max_Errors){
				NUT[i][j]++;
			}else if(NUT[i][j]>0){
				NUT[i][j]--;
			}
		}

		//check for communication errors
		if((cvError>0 || auxError>0) && NC[i] < Max_Errors){
			NC[i]++;
		}else if(NC[i]>0){
			NC[i]--;
		}
	}
	//Check for overcurrent and increase the counting arrays accordingly disabled as long as current sensor is not working properly
//	if(currents[0] > ChgOCP && NOC[0] < Max_Errors){
//		NOC[0]++;
//	}else if(NOC[0] > 0){
//		NOC[0]--;
//	}
//	if(currents[1] > DschgOCP && NOC[1] < Max_Errors){
//		NOC[1]++;
//	}else if(NOC[1] > 0){
//		NOC[1]--;
//	}
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
		}
		for (int j = 0; j < NbTherm; j++){
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
		if(NC[i] > N_Error){
			chargeEnable = false;
			dischargeEnable = false;
			system_OK = false;	//set system not OK
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
	if(HAL_GPIO_ReadPin(OutputEnable_GPIO_Port, OutputEnable_Pin) == GPIO_PIN_RESET){
		//if the hardware fault is present, turn off the outputs
		chargeEnable = false;
		dischargeEnable = false;
		system_OK = false;	//set system not OK
	}
}

void LEDControl(){
	//blink Red and Amber in case of hardware fault
	if(HAL_GPIO_ReadPin(OutputEnable_GPIO_Port, OutputEnable_Pin) == GPIO_PIN_RESET){
		HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
		HAL_GPIO_TogglePin(LED_A_GPIO_Port, LED_A_Pin);
	}else{
		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, voltageError);
		HAL_GPIO_WritePin(LED_A_GPIO_Port, LED_A_Pin, currentError);
	}
	HAL_GPIO_WritePin(LED_Y_GPIO_Port, LED_Y_Pin, tempError);

	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, system_OK);
}

void outputControl(){ //eventually integrate in errorcheck
	//autoreset, for later on
	/*
	if(HAL_GPIO_ReadPin(OutputEnable_GPIO_Port, OutputEnable_Pin) == GPIO_PIN_RESET){
		if(!resetOutputLatch()){
			return;	//if the output latch cannot be reset, return
		}
	}
	*/
	HAL_GPIO_WritePin(Enable1_GPIO_Port, Enable1_Pin, chargeEnable);
	HAL_GPIO_WritePin(Enable2_GPIO_Port, Enable2_Pin, dischargeEnable);

}
void SetHardwareProtection(){ //only set in idle mode stop TIM7 before calling this function
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
    HAL_GPIO_WritePin(A1_6_GPIO_Port, A1_6_Pin, (HardwareCellCnt>>3)&1);
    HAL_GPIO_WritePin(A0_6_GPIO_Port, A0_6_Pin, (HardwareCellCnt>>2)&1);
    HAL_GPIO_WritePin(A1_7_GPIO_Port, A1_7_Pin, (HardwareCellCnt>>1)&1);
    HAL_GPIO_WritePin(A0_7_GPIO_Port, A0_7_Pin, HardwareCellCnt&1);
    //Hysterysis
    HAL_GPIO_WritePin(A1_5_GPIO_Port, A1_5_Pin, (HardwareHysterysis>>1)&1);
    HAL_GPIO_WritePin(A0_5_GPIO_Port, A0_5_Pin, (HardwareHysterysis)&1);
    //Cycle Time
    HAL_GPIO_WritePin(A1_8_GPIO_Port, A1_8_Pin, (HardwareCycleTime>>1)&1);
    HAL_GPIO_WritePin(A0_8_GPIO_Port, A0_8_Pin, (HardwareCycleTime)&1);
}

void balancingControl(){
	if(currents[0]>ChgEndCurr){
		//determine deltaV and highest cell
		float maxVoltage = 0;
		float minVoltage = 10;
//		float maxCell = 0;
//		float minCell = 0;
//		float deltaV = 0;
		for(int i=0;i<CellsNbS;i++){
			if(voltages[0][i]>maxVoltage){
				maxVoltage = voltages[0][i];
//				maxCell = i;
			}
			if (voltages[0][i]<minVoltage){
				minVoltage = voltages[0][i];
//				minCell = i;
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
void LTC_Selftests(){
	HAL_GPIO_WritePin(LTCSelfTest_GPIO_Port, LTCSelfTest_Pin, GPIO_PIN_RESET);	//set the self test pin to low
}
