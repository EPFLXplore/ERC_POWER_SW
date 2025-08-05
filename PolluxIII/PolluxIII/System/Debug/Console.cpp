/*
 * console.c
 *
 *  Created on: Feb 17, 2020
 *      Author: AV Team 2020
 */

#include "Debug/Console.h"

#include <stdarg.h>
#include <string.h>
#include <stm32h7xx_hal.h>
#include <cmsis_os.h>



osSemaphoreDef(console_sem);

Console::Console(UART_HandleTypeDef* uart) : console_uart(uart) {
	console_uart = uart;
	console_semaphore = osSemaphoreCreate(osSemaphore(console_sem), 1);
}

void Console::lock() {
	osSemaphoreWait(console_semaphore, 100 * portTICK_PERIOD_MS);
}

void Console::unlock() {
	osSemaphoreRelease(console_semaphore);
}

void Console::transmit(uint8_t* buffer, uint32_t length) {
	HAL_UART_Transmit(console_uart, buffer, length, 0xFFFFFF);
}

void Console::print(const char* buffer) {
	vTaskSuspendAll();
	transmit((uint8_t*) buffer, strlen(buffer));
	xTaskResumeAll();
}

void Console::printf(const char *format, ...) {
	va_list args;
	va_start(args, format);

	if(vsnprintf(this->buffer, CONSOLE_BUFFER_SIZE, format, args) > 0) {
		print(this->buffer);
	}

	va_end(args);
}
