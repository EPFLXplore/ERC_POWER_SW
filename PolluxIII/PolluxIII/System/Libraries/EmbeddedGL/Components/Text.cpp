/*
 * Text.cpp
 *
 *  Created on: 12 Jun 2023
 *      Author: arion
 */


#include "Text.h"

#include <cstring>

Text::Text() : font(30), anchor(Center) {

}

void Text::render(Renderer* renderer) {
	float x = absoluteBounds.x + absoluteBounds.w/2;
	float y = absoluteBounds.y + absoluteBounds.h/2;

	uint16_t alignement = 0;

	switch(anchor) {
	case NorthWest:
		alignement = 0			 | 0;
		break;
	case North:
		alignement = OPT_CENTERX | 0;
		break;
	case NorthEast:
		alignement = OPT_RIGHTX  | 0;
		break;
	case West:
		alignement = 0 			 | OPT_CENTERY;
		break;
	case Center:
		alignement = OPT_CENTERX | OPT_CENTERY;
		break;
	case East:
		alignement = OPT_RIGHTX  | OPT_CENTERY;
		break;
	}

	renderer->cmd_text(x, y, font, alignement, text);
}

Text* Text::setText(const char* text) {
	taskENTER_CRITICAL(); // Non-atomic operations must be executed when we are sure that the text is not being rendered
	strncpy(this->text, text, TEXT_MAX_LENGTH);
	taskEXIT_CRITICAL();

	return this;
}

Text* Text::setFontSize(uint8_t font) {
	this->font = font;
	return this;
}

Text* Text::setAnchor(enum Anchor anchor) {
	this->anchor = anchor;
	return this;
}
