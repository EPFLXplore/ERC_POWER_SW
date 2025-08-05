/*
 * Gauge.h
 *
 *  Created on: 12 Jun 2023
 *      Author: arion
 */

#ifndef LIBRARIES_EMBEDDEDGL_COMPONENTS_GAUGE_H_
#define LIBRARIES_EMBEDDEDGL_COMPONENTS_GAUGE_H_



#include "Component.h"

class Gauge : public Component {
public:
	Gauge();
	Gauge* setRadius(float radius);
	Gauge* setLevel(float level);

protected:
	virtual void render(Renderer* renderer);
	void onContextReady(Context* context);

private:
	float radius;
	float level;
};


#endif /* LIBRARIES_EMBEDDEDGL_COMPONENTS_GAUGE_H_ */
