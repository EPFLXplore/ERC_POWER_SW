/*
 * Box.cpp
 *
 *  Created on: Jun 10, 2023
 *      Author: arion
 */


#include "Box.h"


Box::Box() : Component(), radius(5.0f) {

}

void Box::render(Renderer* renderer) {
	float x1 = absoluteBounds.x + 1;
	float y1 = absoluteBounds.y + radius;
	float x2 = absoluteBounds.x + absoluteBounds.w - 2;
	float y2 = absoluteBounds.y + absoluteBounds.h - radius - 1;

	renderer->clearCST(0, 1, 0);

	renderer->line_width(radius * 16);
	renderer->begin(RECTS);

	// Draw stencil
	renderer->color_mask(0, 0, 0, 0);
	renderer->stencil_op(INCR, INCR);
	renderer->vertex2f(x1 * 16, y1 * 16);
	renderer->vertex2f(x2 * 16, y2 * 16);
	renderer->vertex2f((x1+1) * 16, (y1+1) * 16);
	renderer->vertex2f((x2-1) * 16, (y2-1) * 16);
	renderer->stencil_op(KEEP, KEEP);
	renderer->color_mask(0xFF, 0xFF, 0xFF, 0xFF);

	// Draw background
	if(isBackgroundSet()) {
		renderer->stencil_func(EQUAL, 2, 255);
		renderer->color_rgb32(getBackgroundColor());
		renderer->vertex2f(x1 * 16, y1 * 16);
		renderer->vertex2f(x2 * 16, y2 * 16);
	}

	// Draw outline
	if(isOutlineSet()) {
		renderer->color_rgb32(getOutlineColor());
	}

	renderer->stencil_func(EQUAL, 1, 255);
	renderer->vertex2f((x1-1) * 16, (y1-1) * 16);
	renderer->vertex2f((x2+1) * 16, (y2+1) * 16);

	renderer->end();

	renderer->stencil_func(ALWAYS, 0, 255);
}

Box* Box::setBorderRadius(float radius) {
	this->radius = radius;
	return this;
}
