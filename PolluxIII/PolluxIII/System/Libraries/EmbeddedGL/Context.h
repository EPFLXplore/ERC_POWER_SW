/*
 * Context.h
 *
 *  Created on: Jun 8, 2023
 *      Author: arion
 */

#ifndef LIBRARIES_EMBEDDEDGL_CONTEXT_H_
#define LIBRARIES_EMBEDDEDGL_CONTEXT_H_

#include "Renderer.h"
#include "Components/Component.h"
#include "Utils/ExceptionTracker.h"

#include <cstdint>
#include <functional>

class Component;

class Context : public ExceptionTracker {
public:
	TRACK_EXCEPTIONS();

	virtual ~Context() {}

	virtual int32_t getWidth() = 0;
	virtual int32_t getHeight() = 0;

	virtual Renderer* getRenderer() = 0;

	virtual void update(Component* root) = 0;
	virtual void render(Component* root) = 0;

	virtual bool async(std::function<void()> task) { return false; }

	virtual void setAlpha(uint8_t alpha) {}
	virtual void setColor(uint32_t color) {}
	virtual void setBackground(uint32_t color) {}
	virtual void setForeground(uint32_t color) {}

	virtual void pushColorContext() {}
	virtual void popColorContext() {}

	virtual uint32_t allocateTouchTag() { return 0; }
};


#endif /* LIBRARIES_EMBEDDEDGL_CONTEXT_H_ */
