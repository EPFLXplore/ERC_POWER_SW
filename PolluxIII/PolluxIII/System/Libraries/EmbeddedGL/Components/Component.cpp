/*
 * Component.cpp
 *
 *  Created on: Jun 8, 2023
 *      Author: arion
 */

#include "Component.h"
#include "EmbeddedGL/Theme.h"

Component::Component() : visible(true), enabled(true) {
	components.trackExceptions(this);
	// Undefined colors (only RGB values are expected)
	// Touch events are disabled by default
}

bool Component::add(Component* comp, float x, float y) {
	return add(comp, x, y, 0, 0);
}

bool Component::add(Component* comp, float x, float y, float width, float height) {
	if(components.add(comp)) {
		comp->trackExceptions(this);

		comp->relativeBounds = { x, y, width, height };

		return true;
	}

	return false;
}

bool Component::remove(Component* comp) {
	return components.remove(comp);
}

Component* Component::getComponentAt(float x, float y) {
	Component* bestCandidate = nullptr;

	if(visible) {
		Iterator<Component*> it = components.it();

		while(it) {
			Component* candidate = (*it++)->getComponentAt(x, y);

			if(candidate != nullptr) {
				bestCandidate = candidate;
			}
		}

		if(bestCandidate == nullptr && enabled) {
			// No subcomponent is in the bounding box, so check if the current component is.
			if(x >= absoluteBounds.x && y >= absoluteBounds.y && x < absoluteBounds.x + absoluteBounds.w && y < absoluteBounds.y + absoluteBounds.h) {
				bestCandidate = this;
			}
		}
	}

	return bestCandidate;
}

bool Component::enableTouchEvents() {
	this->touchTag = context->allocateTouchTag();
	return touchTag != 0;
}

void Component::triggerTouchEvent() {
	onTouchEvent();

	Iterator<std::function<void()>> it = touchHandlers.it();

	while(it) {
		(*it++)();
	}
}

void Component::onTouchEvent() {

}

void Component::onContextReady(Context* context) {

}

bool Component::registerTouchHandler(std::function<void()> handler) {
	return touchHandlers.add(handler);
}

bool Component::unregisterTouchHandler(std::function<void()> handler) {
	return false;
}

void Component::renderAll() {
	if(hasRenderer() && visible) {

		Renderer* renderer = context->getRenderer();

		context->pushColorContext();

		if(alphaSet) {
			if(enabled) {
				context->setAlpha(alpha);
			} else {
				context->setAlpha(alpha/2);
			}
		}

		if(!enabled) {
			context->setAlpha(0x7F);
		}

		if(colorSet) {
			context->setColor(color);
		}

		if(bgColorSet) {
			context->setBackground(bgColor);
		}

		if(fgColorSet) {
			context->setForeground(fgColor);
		}

		renderer->tag(touchTag);

		render(renderer);

		Iterator<Component*> it = components.it();

		while(it) {
			(*it++)->renderAll();
		}

		context->popColorContext();
	}
}

void Component::updateLayout() {
	Iterator<Component*> it = components.it();

	while(it) {
		struct Bounds* self = &this->absoluteBounds;
		struct Bounds* rel = &(*it)->relativeBounds;
		struct Bounds* abs = &(*it)->absoluteBounds;

		abs->x = self->x + self->w*rel->x;
		abs->y = self->y + self->h*rel->y;
		abs->w = self->w*rel->w;
		abs->h = self->h*rel->h;

		(*it++)->updateLayout();
	}
}

void Component::render(Renderer* gpu) {

}

void Component::setContext(Context* context) {
	this->context = context;

	this->absoluteBounds.x = 0.0f;
	this->absoluteBounds.y = 0.0f;
	this->absoluteBounds.w = context->getWidth();
	this->absoluteBounds.h = context->getHeight();

	Iterator<Component*> it = components.it();

	onContextReady(context);

	while(it) {
		(*it++)->setContext(context);
	}
}

bool Component::async(std::function<void()> task) {
	return context->async(task);
}

bool Component::hasRenderingContext() {
	return this->context != nullptr;
}

bool Component::hasRenderer() {
	return hasRenderingContext() && context->getRenderer() != nullptr;
}

Component* Component::setAlpha(uint8_t alpha) {
	this->alpha = alpha;
	this->alphaSet = true;
	return this;
}

Component* Component::setOutlineColor(uint32_t color) {
	this->color = color;
	this->colorSet = true;
	return this;
}

Component* Component::setForegroundColor(uint32_t color) {
	this->fgColor = color;
	this->fgColorSet = true;
	return this;
}

Component* Component::setBackgroundColor(uint32_t color) {
	this->bgColor = color;
	this->bgColorSet = true;
	return this;
}


Component* Component::setVisible(bool visible) {
	this->visible = visible;
	return this;
}

Component* Component::setEnabled(bool enabled) {
	this->enabled = enabled;
	return this;
}

uint8_t Component::getAlpha() {
	return alpha;
}

uint32_t Component::getOutlineColor() {
	return color;
}

uint32_t Component::getForegroundColor() {
	return fgColor;
}

uint32_t Component::getBackgroundColor() {
	return bgColor;
}

bool Component::isAlphaSet() {
	return alphaSet;
}

bool Component::isOutlineSet() {
	return colorSet;
}

bool Component::isBackgroundSet() {
	return bgColorSet;
}

bool Component::isForegroundSet() {
	return fgColorSet;
}

bool Component::isVisible() {
	return visible;
}

bool Component::isEnabled() {
	return enabled;
}

