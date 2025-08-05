/*
 * FT81xContext.cpp
 *
 *  Created on: Jun 8, 2023
 *      Author: arion
 */

#include "FT81xContext.h"
#include "EmbeddedGL/Theme.h"

FT81xContext::FT81xContext(QSPIWrapper* qspi, Console* console) {

	qspi->trackExceptions(this);
	colorStack.trackExceptions(this);

	this->renderer = new FT81xRenderer(qspi, console);

	if(renderer != nullptr) {
		renderer->init_gpu();
		renderer->wake(128);
	} else {
		throwException("MemoryAllocationFailure");
	}
}

void FT81xContext::update(Component* root) {
	root->updateLayout();
}

void FT81xContext::render(Component* root) {

	struct touch_input_t* touch = renderer->fetch_touch_inputs();
	Component* touchTarget = root->getComponentAt(touch->display_y, touch->display_x);

	if(touch->display_x >= 0 && touch->display_y >= 0 && touch->display_y < getWidth() && touch->display_x < getHeight()) {
		if(touchTarget != nullptr && touchAntibouncingBuffer >= 2) {
			touchTarget->triggerTouchEvent();
			this->touchAntibouncingBuffer = 0;
		}
	} else {
		this->touchAntibouncingBuffer++;
	}

	renderer->stream_start(); // Start streaming
	renderer->cmd_dlstart();  // Set REG_CMD_DL when done
	renderer->cmd_swap();     // Set AUTO swap at end of display list

	renderer->clear_color_rgb32(BACKGROUND_COLOR);
	renderer->clear();

	setAlpha(0xFF);
	setColor(OUTLINE_COLOR);
	setBackground(BACKGROUND_COLOR);
	setForeground(FOREGROUND_COLOR);

	root->renderAll();

	renderer->display();

	renderer->getfree(0);     // trigger FT81xRenderer to read the command buffer
	renderer->stream_stop();  // Finish streaming to command buffer

	//// Wait till the GPU is finished
	renderer->wait_finish();

}

static void __async(const void* arg) {
	struct AsyncTask* context = (struct AsyncTask*) arg;
	context->task();

	osThreadId thread_id = context->thread;
	vPortFree(context);

	vTaskDelete(thread_id);
}

bool FT81xContext::async(std::function<void()> task) {
	osThreadDef_t thread = { (char*) "AsyncTask", &__async, (osPriority) 3, 0, 2048};

	struct AsyncTask* context = new AsyncTask();

	if(context == nullptr) {
		return false;
	}

	context->task = task;
	context->thread = osThreadCreate(&thread, context);

	if(context->thread == nullptr) {
		return false;
	}

	return true;
}

uint32_t FT81xContext::allocateTouchTag() {
	if(currentTouchTag < 255) {
		currentTouchTag++;
		return currentTouchTag;
	} else {
		return 0;
	}
}

int32_t FT81xContext::getWidth() {
	return 480;
}

int32_t FT81xContext::getHeight() {
	return 272;
}

Renderer* FT81xContext::getRenderer() {
	return renderer;
}

void FT81xContext::setAlpha(uint8_t alpha) {
	this->currentAlpha = alpha;
	renderer->color_a(alpha);
}

void FT81xContext::setColor(uint32_t color) {
	this->currentColor = color;
	renderer->color_rgb32(color);
}

void FT81xContext::setBackground(uint32_t color) {
	this->currentBackground = color;
	renderer->bgcolor_rgb32(color);
}

void FT81xContext::setForeground(uint32_t color) {
	this->currentForeground = color;
	renderer->fgcolor_rgb32(color);
}

void FT81xContext::pushColorContext() {
	struct ColorSet colors = { currentAlpha, currentColor, currentBackground, currentForeground };

	colorStack.add(colors);
}

void FT81xContext::popColorContext() {
	struct ColorSet colors = colorStack.pop();

	setAlpha(colors.alpha);
	setColor(colors.color);
	setBackground(colors.background);
	setForeground(colors.foreground);
}

