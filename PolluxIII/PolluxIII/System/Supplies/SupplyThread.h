/*
 * SupplyThread.h
 *
 *  Created on: Apr 30, 2023
 *      Author: arion
 */

#ifndef SUPPLIES_SUPPLY_THREAD_H_
#define SUPPLIES_SUPPLY_THREAD_H_


#include "Thread.h"
#include "Supplies.h"
#include "SupplyStates.h"
#include "Sensors/PowerThread.h"

#include "RoCo/RoCo.h"

#include <i2c.h>


class SupplyThread : public Thread {
public:
	SupplyThread(I2C_HandleTypeDef* i2c, struct SupplyConfig device, PowerThread* input_sensor, PowerThread* output_sensor, MessageBus* bus) :
		Thread(device.name, 2048),
		i2c(i2c),
		device(device),
		input_sensor(input_sensor),
		output_sensor(output_sensor),
		bus(bus),
		switching(false),
		supplying(false),
		state(DAC_FAULT) {}

	void init();
	void loop();
	void setSwitching(bool switching);
	void setSupplying(bool supplying);
	void reprogram();
	const char* getStatus();

private:
	I2C_HandleTypeDef* i2c;
	struct SupplyConfig device;
	bool error;
	PowerThread* input_sensor;
	PowerThread* output_sensor;
	MessageBus* bus;
	bool switching;
	bool supplying;
	enum SupplyState state;

	uint16_t get(uint8_t reg);
	void set(uint8_t reg, uint16_t value);
	bool read(uint8_t reg, uint8_t* data, uint8_t size);
	bool write(uint8_t reg, uint8_t* data, uint8_t size);
	bool enable(uint8_t flag);
	bool disable(uint8_t flag);
	bool hasNewErrors();

	void handleControlCommands(uint8_t sender, Power_SupplyControl* packet);
};



#endif /* SUPPLIES_SUPPLY_THREAD_H_ */
