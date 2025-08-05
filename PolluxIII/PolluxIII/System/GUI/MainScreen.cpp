/*
 * MainScreen.cpp
 *
 *  Created on: 7 Jun 2023
 *      Author: arion
 */


#include "MainScreen.h"
#include "EmbeddedGL/Components/Text.h"

MainScreen::MainScreen(MessageBus* bus) {
	this->lvaBox = new SupplyBox(West, bus, LVA);
	this->lvbBox = new SupplyBox(East, bus, LVB);
	this->hvaBox = new SupplyBox(West, bus, HVA);
	this->hvbBox = new SupplyBox(East, bus, HVB);
	this->batteryBox = new UnregulatedBox(bus, BATTERY);
	this->motorsBox = new UnregulatedBox(bus, MOTORS);
	this->chargeGauge = new ChargeGauge(bus);
	this->detailsScreen = new DetailsScreen(bus);

	if(lvaBox == nullptr || lvbBox == nullptr || hvaBox == nullptr || hvbBox == nullptr || batteryBox == nullptr
			|| motorsBox == nullptr || chargeGauge == nullptr || detailsScreen == nullptr) {
		throwException("MemoryAllocationFailure");
	}

	// One-pixel-wide epsilon value to remove borders of the following boxes
	float ex = 2.0f/480;
	float ey = 2.0f/272;

	add(detailsScreen, 0.0f, 0.0f, 1.0f, 1.0f);
	add(chargeGauge, 0.5f, 0.5f, 0.0f, 0.12f);
	add(batteryBox, 0.425f, 0.0f-2*ey, 0.15f, 0.22f);
	add(motorsBox, 0.425f, 0.78f, 0.15f, 0.22f+2*ey);
	add(lvaBox, 0.0f-ex, 0.0f-ey, 0.5f+ex, 0.5f+ey);
	add(lvbBox, 0.5f, 0.0f-ey, 0.5f+ex, 0.5f+ey);
	add(hvaBox, 0.0f-ex, 0.5f, 0.5f+ex, 0.5f+ey);
	add(hvbBox, 0.5f, 0.5f, 0.5f+ex, 0.5f+ey);

	detailsScreen->setVisible(false); // Overlay should not be visible by default

	lvaBox->registerTouchHandler(std::bind(&MainScreen::showDetailedStats, this, LVA));
	lvbBox->registerTouchHandler(std::bind(&MainScreen::showDetailedStats, this, LVB));
	hvaBox->registerTouchHandler(std::bind(&MainScreen::showDetailedStats, this, HVA));
	hvbBox->registerTouchHandler(std::bind(&MainScreen::showDetailedStats, this, HVB));
	batteryBox->registerTouchHandler(std::bind(&MainScreen::showDetailedStats, this, BATTERY));
	motorsBox->registerTouchHandler(std::bind(&MainScreen::showDetailedStats, this, MOTORS));
}

MainScreen::~MainScreen() {
	delete lvaBox;
	delete lvbBox;
	delete hvaBox;
	delete hvbBox;
	delete batteryBox;
	delete motorsBox;
	delete chargeGauge;
	delete detailsScreen;
}

void MainScreen::showDetailedStats(PowerMonitor monitor) {
	detailsScreen->setTargetMonitor(monitor);
	detailsScreen->setVisible(true);
}
