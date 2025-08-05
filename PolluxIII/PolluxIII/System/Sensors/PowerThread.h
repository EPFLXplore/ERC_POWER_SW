/*
 * PowerThread.h
 *
 *  Created on: Apr 30, 2022
 *      Author: arion
 */

#ifndef SENSORS_POWERTHREAD_H_
#define SENSORS_POWERTHREAD_H_


#include "Thread.h"

#include "PowerMonitors.h"
#include "INA239/INA239.h"
#include "RoCo/RoCo.h"

#include <spi.h>
#include <tim.h>

class PowerThread : public Thread {
public:
	PowerThread(SPI_HandleTypeDef* spi, TIM_HandleTypeDef* timer, PowerMonitor monitor_id, uint8_t device_id, MessageBus* bus) :
		Thread(toString(monitor_id), 2048),
		spi(spi),
		timer(timer),
		monitor_id(monitor_id),
		device_id(device_id),
		bus(bus) {}

	void init();
	void loop();
	float getPower();
	float getEnergy();
	float getVoltage();
	float getTemperature();

private:
	SPI_HandleTypeDef* spi;
	TIM_HandleTypeDef* timer;
	PowerMonitor monitor_id;
	uint8_t device_id;
	MessageBus* bus;
	uint32_t last_time;
	float energy;

	uint16_t get(uint8_t reg);
	bool set(uint8_t reg, uint16_t value);

	bool write(uint8_t reg, uint8_t *data, uint8_t size);
	bool read(uint8_t reg, uint8_t *data, uint8_t size);
};



#endif /* SENSORS_POWERTHREAD_H_ */
