/*
 * Gauge.cpp
 *
 *  Created on: 12 Jun 2023
 *      Author: arion
 */

#include "Gauge.h"


Gauge::Gauge() {
	this->radius = 50.0f;
}

void Gauge::render(Renderer* renderer) {
	float x = absoluteBounds.x;
	float y = absoluteBounds.y;

	renderer->begin(POINTS);
	renderer->point_size(radius * 16);
	renderer->vertex2f(x * 16, y * 16);

	renderer->cmd_gauge(x, y, radius-1, OPT_FLAT, 10, 1, this->level*100, 100);
}

void Gauge::onContextReady(Context* context) {
	enableTouchEvents();
}

Gauge* Gauge::setRadius(float radius) {
	this->radius = radius;
	return this;
}

Gauge* Gauge::setLevel(float level) {
	this->level = level;
	return this;
}
