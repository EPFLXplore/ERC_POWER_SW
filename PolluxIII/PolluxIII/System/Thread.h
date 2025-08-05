/*
 * Thread.hpp
 *
 *  Created on: 23 Oct 2020
 *      Author: AV Team 2020
 */

#ifndef THREAD_H_
#define THREAD_H_

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "stm32h7xx_hal.h"
#include "Libraries/Utils/Operators.h"
#include "Libraries/Utils/ExceptionTracker.h"

class Thread : public ExceptionTracker {
public:
	TRACK_EXCEPTIONS();

	Thread(const char* name);
	Thread(const char* name, osPriority priority);
	Thread(const char* name, uint32_t stackSize);
	Thread(const char* name, osPriority priority, uint32_t stackSize);
	virtual ~Thread() {};
	virtual void init() = 0;
	virtual void loop() = 0;
	osThreadId getHandle();
	bool isRunning() { return running; }
	void terminate();
	uint32_t getTickDelay();

protected:
	void setTickDelay(uint32_t ms);
	void println(const char* format, ...);

private:
	osThreadId handle;
	const char* name;
	bool running = true;
	uint32_t delay;

	void task();
};


#endif /* THREAD_H_ */
