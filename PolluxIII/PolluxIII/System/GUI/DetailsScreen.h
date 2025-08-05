/*
 * DetailsScreen.h
 *
 *  Created on: 14 Jun 2023
 *      Author: arion
 */

#ifndef GUI_DETAILSSCREEN_H_
#define GUI_DETAILSSCREEN_H_

#include "EmbeddedGL/Screen.h"
#include "EmbeddedGL/Components/Box.h"
#include "EmbeddedGL/Components/Button.h"

#include "RoCo/RoCo.h"
#include "Sensors/PowerMonitors.h"

class DetailsScreen : public Screen {
public:
	TRACK_EXCEPTIONS();

	DetailsScreen(MessageBus* bus);
	~DetailsScreen();

	void setTargetMonitor(PowerMonitor monitorID);

private:
	MessageBus* bus;
	PowerMonitor monitorID;
	Box* mainFrame;
	Button* restartButton;
	Button* closeButton;
	Text* title;
	Text* status;
	Text* voltage;
	Text* power;
	Text* energy;
	Text* temperature;

	void restartSupply();
	void handleBusInfo(uint8_t sender, Power_BusInfo* packet);
	void handleSupplyInfo(uint8_t sender, Power_SupplyInfo* packet);
};


#endif /* GUI_DETAILSSCREEN_H_ */
