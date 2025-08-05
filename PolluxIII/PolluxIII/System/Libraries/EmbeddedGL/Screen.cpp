/*
 * Screen.cpp
 *
 *  Created on: 14 Jun 2023
 *      Author: arion
 */

#include "Screen.h"

void Screen::render(Renderer* renderer) {
	if(isBackgroundSet()) {
		float x1 = absoluteBounds.x;
		float y1 = absoluteBounds.y;
		float x2 = absoluteBounds.x + absoluteBounds.w;
		float y2 = absoluteBounds.y + absoluteBounds.h;

		renderer->begin(RECTS);

		renderer->color_rgb32(getBackgroundColor());
		renderer->vertex2f(x1 * 16, y1 * 16);
		renderer->vertex2f(x2 * 16, y2 * 16);

		renderer->end();
	}
}
