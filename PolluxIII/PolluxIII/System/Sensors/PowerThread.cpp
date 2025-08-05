/*
 * PowerThread.cpp
 *
 *  Created on: Apr 30, 2022
 *      Author: arion
 */


#include "PowerThread.h"
#include "Debug/Debug.h"

#define MAX_CURRENT 40   // A
#define SHUNT_VALUE	1000 // µΩ

#include <stdio.h>

void PowerThread::init() {

	osDelay(100 / portTICK_PERIOD_MS);

	if(device_id < 7) {
		uint8_t attempts = 3;
		bool init_success = false;

		while(attempts-- > 0) {
			if(get(INA239_DEVICE_ID_REG) == INA239_DEV_ID) {
				init_success = true;
				break;
			}
		}

		if(init_success) {

			bool status = set(INA239_CONFIG_REG, INA239_CONFIG_ADCRANGE);

			uint16_t adc_config = 0;
			adc_config |= INA239_ADCCONFIG_MODE & (0xF << INA239_ADCCONFIG_MODE_BIT);
			adc_config |= INA239_ADCCONFIG_VBUSCT & (0x2 << INA239_ADCCONFIG_VBUSCT_BIT);
			adc_config |= INA239_ADCCONFIG_VSHCT & (0x2 << INA239_ADCCONFIG_VSHCT_BIT);
			adc_config |= INA239_ADCCONFIG_VTCT & (0x7 << INA239_ADCCONFIG_VTCT_BIT);
			adc_config |= INA239_ADCCONFIG_AVG & (0x3 << INA239_ADCCONFIG_AVG_BIT);

			status |= set(INA239_ADC_CONFIG_REG, adc_config);

			status |= set(INA239_SHUNT_CAL_REG, MAX_CURRENT * SHUNT_VALUE / 5);

			if(status) {
				println("Device ID %d initialized successfully", device_id);
			} else {
				println("Failed to configure device ID %d", device_id);
				terminate();
			}
		} else {
			println("Failed to initialize device ID %d", device_id);
			terminate();
		}
	} else {
		println("Failed to initialize device ID %d (ID out of range)", device_id);
		terminate();
	}

	timer->Instance->CR1 |= TIM_CR1_CEN;
}


void PowerThread::loop() {
	float power = getPower();

	uint32_t current_t = timer->Instance->CNT;
	int32_t delta_t = current_t - last_time;
	this->last_time = current_t;

	if(delta_t > 0) {
		energy += delta_t * power;
	}

	Power_BusInfo info;
	info.bus_id = monitor_id;
	info.voltage = getVoltage();
	info.power = getPower();
	info.energy = getEnergy();
	info.temperature = getTemperature();

	MAKE_RELIABLE(info);

	bus->send(&info);

	if(monitor.enter(POWER_MONITORS | device_id)) {
		println("Energy: %.3fWh\x1b[K", getEnergy());
		println("Power: %.3fW\x1b[K", getPower());
		println("Voltage: %.3fV\x1b[K", getVoltage());
		println("Temperature: %.3f°C\x1b[K", getTemperature());

		monitor.exit(POWER_MONITORS | device_id);
	}
}


uint16_t PowerThread::get(uint8_t reg) {
	uint16_t value = 0;
	read(reg, (uint8_t*) &value, 2);
	return value;
}

float PowerThread::getVoltage() {
	return 1E-6f * 3125 * get(INA239_VBUS_REG);
}

float PowerThread::getTemperature() {
	return 1E-3f * 125 * ((int16_t) get(INA239_DIETEMP_REG) >> INA239_DIETEMP_BIT);
}

float PowerThread::getPower() {
	int32_t value = 0;
	read(INA239_POWER_REG, (uint8_t*) &value, 3);
	return 1E-3f * ((1000 * value * MAX_CURRENT / 5) >> 14);
}

float PowerThread::getEnergy() {
	return 1E-6f * energy / 3600;
}

bool PowerThread::set(uint8_t reg, uint16_t value) {
	return write(reg, (uint8_t*) &value, 2);
}

bool PowerThread::write(uint8_t reg, uint8_t *data, uint8_t size) {
	uint8_t tx_buffer[4];
	tx_buffer[0] = (reg << 2) | 0b00;

	for(uint8_t i = 0; i < size; i++) {
		tx_buffer[i+1] = data[size-i-1];
	}

	__disable_irq();
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, device_id << 1, GPIO_PIN_SET);
	HAL_StatusTypeDef status = HAL_SPI_Transmit(spi, tx_buffer, size+1, 10 / portTICK_PERIOD_MS);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_SET);
	__enable_irq();

	return status == HAL_OK;
}

bool PowerThread::read(uint8_t reg, uint8_t *data, uint8_t size) {
	uint8_t rx_buffer[4];
	rx_buffer[0] = (reg << 2) | 0b01;

	__disable_irq();
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, device_id << 1, GPIO_PIN_SET);
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(spi, rx_buffer, rx_buffer, size+1, 100 / portTICK_PERIOD_MS);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_SET);
	__enable_irq();

	for(uint8_t i = 0; i < size; i++) {
		data[i] = rx_buffer[size-i];
	}

	return status == HAL_OK;
}
