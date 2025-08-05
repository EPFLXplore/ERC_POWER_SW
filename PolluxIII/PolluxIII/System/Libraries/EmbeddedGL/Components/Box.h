/*
 * Box.h
 *
 *  Created on: Jun 10, 2023
 *      Author: arion
 */

#ifndef LIBRARIES_EMBEDDEDGL_COMPONENTS_BOX_H_
#define LIBRARIES_EMBEDDEDGL_COMPONENTS_BOX_H_

#include "Component.h"

class Box : virtual public Component {
public:
	Box();
	Box* setBorderRadius(float radius);

protected:
	virtual void render(Renderer* renderer);

private:
	float radius;
};


#endif /* LIBRARIES_EMBEDDEDGL_COMPONENTS_BOX_H_ */
