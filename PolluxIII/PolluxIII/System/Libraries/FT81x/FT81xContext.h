/*
 * FT81xContext.h
 *
 *  Created on: Jun 10, 2023
 *      Author: arion
 */

#ifndef LIBRARIES_FT81X_FT81XCONTEXT_H_
#define LIBRARIES_FT81X_FT81XCONTEXT_H_

#include "EmbeddedGL/Context.h"
#include "FT81xRenderer.h"

#include "QSPIWrapper.h"
#include "Debug/Console.h"
#include "Utils/LinkedList.h"
#include "Utils/Operators.h"

struct AsyncTask {
	std::function<void()> task;
	osThreadId thread;
};

struct ColorSet {
	uint8_t alpha;
	uint32_t color;
	uint32_t background;
	uint32_t foreground;
};

class FT81xContext : public Context {
public:
	TRACK_EXCEPTIONS();

	FT81xContext(QSPIWrapper* qspi, Console* console);
	int32_t getWidth();
	int32_t getHeight();
	Renderer* getRenderer();

	void update(Component* screen);
	void render(Component* screen);
	bool async(std::function<void()> task);

	uint32_t allocateTouchTag();

	void setAlpha(uint8_t alpha);
	void setColor(uint32_t color);
	void setBackground(uint32_t color);
	void setForeground(uint32_t color);

	void pushColorContext();
	void popColorContext();

private:
	FT81xRenderer* renderer;
	LinkedList<struct ColorSet> colorStack;
	uint8_t currentAlpha;
	uint32_t currentColor;
	uint32_t currentBackground;
	uint32_t currentForeground;
	uint8_t currentTouchTag;
	uint32_t touchAntibouncingBuffer;
};


#endif /* LIBRARIES_FT81X_FT81XCONTEXT_H_ */
