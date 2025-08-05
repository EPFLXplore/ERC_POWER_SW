/*
 * SupplyBox.h
 *
 *  Created on: 12 Jun 2023
 *      Author: arion
 */

#ifndef GUI_SUPPLYBOX_H_
#define GUI_SUPPLYBOX_H_


#include "PowerBox.h"

class SupplyBox : public PowerBox {
public:
	TRACK_EXCEPTIONS();
	SupplyBox(enum Anchor align, MessageBus* bus, PowerMonitor monitorID);
};

#endif /* GUI_SUPPLYBOX_H_ */
