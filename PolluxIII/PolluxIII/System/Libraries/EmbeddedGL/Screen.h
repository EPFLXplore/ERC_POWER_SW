/*
 * Screen.h
 *
 *  Created on: Jun 10, 2023
 *      Author: arion
 */

#ifndef LIBRARIES_EMBEDDEDGL_SCREEN_H_
#define LIBRARIES_EMBEDDEDGL_SCREEN_H_

#include "Components/Component.h"

class Screen : public Component {
protected:
	virtual void render(Renderer* renderer);
};


#endif /* LIBRARIES_EMBEDDEDGL_SCREEN_H_ */
