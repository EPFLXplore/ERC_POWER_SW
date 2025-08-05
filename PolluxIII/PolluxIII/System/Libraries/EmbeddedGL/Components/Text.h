/*
 * Text.h
 *
 *  Created on: 12 Jun 2023
 *      Author: arion
 */

#ifndef LIBRARIES_EMBEDDEDGL_COMPONENTS_TEXT_H_
#define LIBRARIES_EMBEDDEDGL_COMPONENTS_TEXT_H_


#include "Component.h"


#define TEXT_MAX_LENGTH 64

enum Anchor {
	NorthWest,
	North,
	NorthEast,
	West,
	Center,
	East
};

class Text : virtual public Component {
public:
	TRACK_EXCEPTIONS();
	Text();
	Text* setText(const char* text);
	Text* setAnchor(enum Anchor anchor);
	Text* setFontSize(uint8_t font);

protected:
	virtual void render(Renderer* renderer);

private:
	char text[TEXT_MAX_LENGTH];
	uint8_t font;
	enum Anchor anchor;
};



#endif /* LIBRARIES_EMBEDDEDGL_COMPONENTS_TEXT_H_ */
