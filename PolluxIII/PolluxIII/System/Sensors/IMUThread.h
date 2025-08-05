/*
 * IMUThread.h
 *
 *  Created on: Apr 30, 2023
 *      Author: arion
 */

#ifndef SENSORS_IMUTHREAD_H_
#define SENSORS_IMUTHREAD_H_


#include "Thread.h"
#include "Libraries/INA239/INA239.h"

#include <i2c.h>

class IMUThread : public Thread {
public:
	IMUThread(I2C_HandleTypeDef* i2c, uint8_t device_id, const char* name) : Thread(name, 2048), i2c(i2c), device_id(device_id) {}
	void init();
	void loop();

private:
	I2C_HandleTypeDef* i2c;
	uint8_t device_id;
};



#endif /* SENSORS_IMUTHREAD_H_ */
