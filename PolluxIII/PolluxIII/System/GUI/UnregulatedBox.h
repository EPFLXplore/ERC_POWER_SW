/*
 * UnregulatedBox.h
 *
 *  Created on: 12 Jun 2023
 *      Author: arion
 */

#ifndef GUI_UNREGULATEDBOX_H_
#define GUI_UNREGULATEDBOX_H_


#include "PowerBox.h"

class UnregulatedBox : public PowerBox {
public:
	TRACK_EXCEPTIONS();
	UnregulatedBox(MessageBus* bus, PowerMonitor monitorID);
};

#endif /* GUI_UNREGULATEDBOX_H_ */
