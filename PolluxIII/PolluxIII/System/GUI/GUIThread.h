/*
 * GUIThread.h
 *
 *  Created on: 12 Jun 2023
 *      Author: arion
 */

#ifndef GUI_GUITHREAD_H_
#define GUI_GUITHREAD_H_


#include "Thread.h"

#include "MainScreen.h"
#include "FT81x/FT81xContext.h"

class GUIThread : public Thread {
public:
	TRACK_EXCEPTIONS();

	GUIThread(SPI_HandleTypeDef* spi, MessageBus* bus);
	~GUIThread();

	void init();
	void loop();

private:
	QSPIWrapper wrapper;
	MainScreen screen;
	FT81xContext* context;
};



#endif /* GUI_GUITHREAD_H_ */
