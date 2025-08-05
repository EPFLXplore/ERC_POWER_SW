/*
 * MainScreen.h
 *
 *  Created on: 7 Jun 2023
 *      Author: arion
 */

#ifndef GUI_MAINSCREEN_H_
#define GUI_MAINSCREEN_H_


#include "EmbeddedGL/Screen.h"

#include "SupplyBox.h"
#include "UnregulatedBox.h"
#include "ChargeGauge.h"
#include "DetailsScreen.h"

#include "RoCo/RoCo.h"

class MainScreen : public Screen {
public:
	TRACK_EXCEPTIONS();

	MainScreen(MessageBus* bus);
	~MainScreen();

private:
	SupplyBox* lvaBox;
	SupplyBox* lvbBox;
	SupplyBox* hvaBox;
	SupplyBox* hvbBox;
	UnregulatedBox* batteryBox;
	UnregulatedBox* motorsBox;
	ChargeGauge* chargeGauge;
	DetailsScreen* detailsScreen;

	void handleBatteryInfo(uint8_t sender, Power_BatteryInfo* packet);
	void showDetailedStats(PowerMonitor monitor);
};


#endif /* GUI_MAINSCREEN_H_ */
