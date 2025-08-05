/*
 * IMUThread.cpp
 *
 *  Created on: Apr 30, 2022
 *      Author: arion
 */


#include "IMUThread.h"
#include "Debug/Debug.h"

void IMUThread::init() {

	osDelay(100 / portTICK_PERIOD_MS);

	if(HAL_I2C_IsDeviceReady(i2c, device_id, 3, 10 / portTICK_PERIOD_MS)) {
		bool status = true;

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
}


void IMUThread::loop() {

}
