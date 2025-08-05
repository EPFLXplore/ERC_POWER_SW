/*
 * Thread.cpp
 *
 *  Created on: 23 Oct 2020
 *      Author: AV Team 2020
 */

#include "Thread.h"

#include "Debug/Debug.h"
#include "Utils/Operators.h"

#include "iwdg.h"
#include "usart.h"

#include <stdarg.h>
#include <string.h>

#define DEFAULT_STACK_SIZE (2048) // Danger zone: changing the stack size might create very nasty bugs

static char buffer[128];

void __task_run(const void* arg) {
	Thread* thread = (Thread*) arg;

	osDelay(100 / portTICK_PERIOD_MS);

	thread->init();

	while(thread->isRunning()) {
		thread->loop();
		osDelay(thread->getTickDelay() / portTICK_PERIOD_MS);
	}

	vTaskDelete(nullptr);

	while(true) {
		osDelay(1000 / portTICK_PERIOD_MS);
	}
}

Thread::Thread(const char* name) : Thread(name, (osPriority) osPriorityNormal) {
	;
}

Thread::Thread(const char* name, osPriority priority) : Thread(name, priority, DEFAULT_STACK_SIZE) {
	;
}

Thread::Thread(const char* name, uint32_t stackSize) : Thread(name, (osPriority) osPriorityNormal, stackSize) {
	;
}

Thread::Thread(const char* name, osPriority priority, uint32_t stackSize) : delay(10) {
	osThreadDef_t thread = { (char*) name, &__task_run, priority, 0, stackSize};
	this->handle = osThreadCreate(&thread, this);
	this->name = name;
}

osThreadId Thread::getHandle() {
	return handle;
}

void Thread::terminate() {
	this->running = false;
}

void Thread::println(const char* format, ...) {
	va_list args;
	va_start(args, format);

	sprintf(buffer, "[%s] ", name);
	vsprintf(buffer + strlen(buffer), format, args);
	strcat(buffer, "\r\n");

	console.print(buffer);

	va_end(args);
}

void Thread::setTickDelay(uint32_t ms) {
	this->delay = ms;
}

uint32_t Thread::getTickDelay() {
	return this->delay;
}
