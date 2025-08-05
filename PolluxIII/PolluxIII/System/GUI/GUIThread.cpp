/*
 * GUIThread.cpp
 *
 *  Created on: 12 Jun 2023
 *      Author: arion
 */

#include "GUIThread.h"

#include "Debug/Debug.h"

GUIThread::GUIThread(SPI_HandleTypeDef* spi, MessageBus* bus) : Thread("GUI", 2048), wrapper(spi), screen(bus) {

}

GUIThread::~GUIThread() {
	delete context;
}

void GUIThread::init() {
	this->context = new FT81xContext(&wrapper, &console);

	context->trackExceptions(this);
	wrapper.trackExceptions(this);
	screen.trackExceptions(this);

	screen.setContext(context);
	context->update(&screen);

	setTickDelay(100);

	println("GUI ready");
}

void GUIThread::loop() {
	context->render(&screen);
}
