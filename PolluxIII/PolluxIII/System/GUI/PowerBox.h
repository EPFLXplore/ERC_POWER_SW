/*
 * PowerBox.h
 *
 *  Created on: 13 Jun 2023
 *      Author: arion
 */

#ifndef GUI_POWERBOX_H_
#define GUI_POWERBOX_H_

#include "EmbeddedGL/Components/Box.h"
#include "EmbeddedGL/Components/Text.h"

#include "Sensors/PowerMonitors.h"
#include "RoCo/RoCo.h"

class PowerBox : public Box {
public:
	TRACK_EXCEPTIONS();

	PowerBox(MessageBus* bus, PowerMonitor monitorID);
	~PowerBox();

	PowerBox* setError(bool error);

protected:
	Text* title;
	Text* voltage;
	Text* power;

private:
	PowerMonitor monitorID;
	void handleBusInfo(uint8_t sender, Power_BusInfo* packet);
	void handleSupplyInfo(uint8_t sender, Power_SupplyInfo* packet);
};


#endif /* GUI_POWERBOX_H_ */
