/*
 * Button.h
 *
 *  Created on: 14 Jun 2023
 *      Author: arion
 */

#ifndef LIBRARIES_EMBEDDEDGL_COMPONENTS_BUTTON_H_
#define LIBRARIES_EMBEDDEDGL_COMPONENTS_BUTTON_H_

#include "Box.h"
#include "Text.h"

class Button : public Box, public Text {
	virtual void render(Renderer* renderer) {
		Box::render(renderer);
		Text::render(renderer);
	}
};


#endif /* LIBRARIES_EMBEDDEDGL_COMPONENTS_BUTTON_H_ */
