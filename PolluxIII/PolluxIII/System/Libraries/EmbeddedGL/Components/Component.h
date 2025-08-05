/*
 * Component.h
 *
 *  Created on: Jun 8, 2023
 *      Author: arion
 */

#ifndef LIBRARIES_EMBEDDEDGL_COMPONENTS_COMPONENT_H_
#define LIBRARIES_EMBEDDEDGL_COMPONENTS_COMPONENT_H_


#include "EmbeddedGL/Context.h"

#include "Utils/ExceptionTracker.h"
#include "Utils/LinkedList.h"
#include "Utils/Operators.h"

#include <functional>


struct Bounds {
	float x;
	float y;
	float w;
	float h;
};

class Context;

class Component : public ExceptionTracker {
public:
	TRACK_EXCEPTIONS();

	Component();
	virtual ~Component() {};

	bool add(Component* comp, float x, float y, float width, float height);
	bool add(Component* comp, float x, float y);
	bool remove(Component* comp);

	void updateLayout();
	void renderAll();

	void setContext(Context* context);

	Component* getComponentAt(float x, float y);
	bool enableTouchEvents();
	void triggerTouchEvent();
	bool registerTouchHandler(std::function<void()> handler);
	bool unregisterTouchHandler(std::function<void()> handler);

	Component* setAlpha(uint8_t alpha);
	Component* setOutlineColor(uint32_t color);
	Component* setForegroundColor(uint32_t color);
	Component* setBackgroundColor(uint32_t color);
	Component* setVisible(bool visible);
	Component* setEnabled(bool enabled);

	uint8_t getAlpha();
	uint32_t getOutlineColor();
	uint32_t getBackgroundColor();
	uint32_t getForegroundColor();

	bool isAlphaSet();
	bool isOutlineSet();
	bool isBackgroundSet();
	bool isForegroundSet();

	bool isVisible();
	bool isEnabled();

protected:
	struct Bounds absoluteBounds;

	virtual void render(Renderer* renderer);
	virtual void onContextReady(Context* context);
	virtual void onTouchEvent();

	bool async(std::function<void()> task);

private:
	LinkedList<Component*> components;
	LinkedList<std::function<void()>> touchHandlers;
	struct Bounds relativeBounds;
	Context* context;

	uint8_t alpha;
	uint32_t color;
	uint32_t bgColor;
	uint32_t fgColor;

	bool alphaSet;
	bool colorSet;
	bool bgColorSet;
	bool fgColorSet;

	bool visible;
	bool enabled;
	uint8_t touchTag;

	bool hasRenderingContext();
	bool hasRenderer();
};


#endif /* LIBRARIES_EMBEDDEDGL_COMPONENTS_COMPONENT_H_ */
