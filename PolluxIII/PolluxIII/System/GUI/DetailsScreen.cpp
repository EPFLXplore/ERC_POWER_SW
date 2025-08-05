/*
 * DetailsScreen.cpp
 *
 *  Created on: 14 Jun 2023
 *      Author: arion
 */

#include "DetailsScreen.h"
#include "EmbeddedGL/Theme.h"
#include "Supplies/SupplyStates.h"

DetailsScreen::DetailsScreen(MessageBus* bus) : bus(bus) {
	setBackgroundColor(0);
	setAlpha(0xBF);

	this->mainFrame = new Box();
	this->restartButton = new Button();
	this->closeButton = new Button();
	this->title = new Text();
	this->status = new Text();
	this->voltage = new Text();
	this->power = new Text();
	this->energy = new Text();
	this->temperature = new Text();

	if(mainFrame == nullptr || restartButton == nullptr || closeButton == nullptr || title == nullptr || status == nullptr
			|| voltage == nullptr || power == nullptr || energy == nullptr || temperature == nullptr) {
		throwException("MemoryAllocationFailure");
	}

	mainFrame->setAlpha(0xFF);
	mainFrame->setBackgroundColor(BACKGROUND_COLOR);
	mainFrame->setOutlineColor(OUTLINE_COLOR);

	title->setText("Title")->setAnchor(NorthEast);
	status->setText("Status")->setFontSize(28)->setAnchor(NorthEast);
	voltage->setText("Voltage: 0.0V")->setAnchor(NorthWest)->setFontSize(27);
	power->setText("Power: 0.0W")->setAnchor(NorthWest)->setFontSize(27);
	energy->setText("Energy: 0.0Wh")->setAnchor(NorthWest)->setFontSize(27);
	temperature->setText("Temperature: 0.0C")->setAnchor(NorthWest)->setFontSize(27);
	restartButton->setText("Restart")->setFontSize(28)->setOutlineColor(0xFF0000);
	closeButton->setText("Close")->setFontSize(28);

	restartButton->registerTouchHandler(std::bind(&DetailsScreen::restartSupply, this));
	closeButton->registerTouchHandler(std::bind(&Component::setVisible, this, false));

	add(mainFrame, 0.1f, 0.1f, 0.8f, 0.8f);

	mainFrame->add(title, 0.9f, 0.15f);
	mainFrame->add(status, 0.9f, 0.325f);
	mainFrame->add(voltage, 0.1f, 0.15f);
	mainFrame->add(power, 0.1f, 0.275f);
	mainFrame->add(energy, 0.1f, 0.4f);
	mainFrame->add(temperature, 0.1f, 0.525f);
	mainFrame->add(restartButton, 0.1f, 0.75f, 0.35f, 0.15f);
	mainFrame->add(closeButton, 0.55f, 0.75f, 0.35f, 0.15f);

	bus->handle<Power_BusInfo>(std::bind(&DetailsScreen::handleBusInfo, this, std::placeholders::_1, std::placeholders::_2));
	bus->handle<Power_SupplyInfo>(std::bind(&DetailsScreen::handleSupplyInfo, this, std::placeholders::_1, std::placeholders::_2));
}

DetailsScreen::~DetailsScreen() {
	delete mainFrame;
	delete restartButton;
	delete closeButton;
	delete title;
	delete status;
	delete voltage;
	delete power;
	delete energy;
	delete temperature;
}

void DetailsScreen::restartSupply() {

	const PowerMonitor busID = monitorID;

	async([this, busID]() {
		restartButton->setEnabled(false);

		char buffer[20];

		snprintf(buffer, 20, "%s 1/4...", toString(busID));
		restartButton->setText(buffer);

		Power_SupplyControl packet;
		packet.bus_id = monitorID;

		// Disable supply
		packet.command_mask = 0b01;
		packet.command_val = 0b00;
		MAKE_RELIABLE(packet);
		bus->send(&packet);

		osDelay(500);

		snprintf(buffer, 20, "%s 2/4...", toString(busID));
		restartButton->setText(buffer);

		// Disable controller
		packet.command_mask = 0b10;
		packet.command_val = 0b00;
		MAKE_RELIABLE(packet);
		bus->send(&packet);

		osDelay(1000);

		snprintf(buffer, 20, "%s 3/4...", toString(busID));
		restartButton->setText(buffer);

		// Enable controller
		packet.command_mask = 0b10;
		packet.command_val = 0b10;
		MAKE_RELIABLE(packet);
		bus->send(&packet);

		osDelay(500);

		snprintf(buffer, 20, "%s 4/4...", toString(busID));
		restartButton->setText(buffer);

		// Enable supply
		packet.command_mask = 0b01;
		packet.command_val = 0b01;
		MAKE_RELIABLE(packet);
		bus->send(&packet);

		restartButton->setText("Restart");
		restartButton->setEnabled(true);
	});
}

void DetailsScreen::handleBusInfo(uint8_t sender, Power_BusInfo* packet) {
	if(packet->bus_id == monitorID) {
		char buffer[20];

		snprintf(buffer, 20, "Voltage: %.1fV", packet->voltage);
		voltage->setText(buffer);

		snprintf(buffer, 20, "Power: %.1fW", packet->power);
		power->setText(buffer);

		snprintf(buffer, 20, "Energy: %.1fWh", packet->energy);
		energy->setText(buffer);

		snprintf(buffer, 20, "Temperature: %.1fC", packet->temperature);
		temperature->setText(buffer);
	}
}

void DetailsScreen::handleSupplyInfo(uint8_t sender, Power_SupplyInfo* packet) {
	if(packet->bus_id == monitorID) {
		enum SupplyState state = (enum SupplyState) packet->state;

		status->setText(toString(state));

		if(state == SUPPLYING) {
			title->setOutlineColor(0x00FF00);
			status->setOutlineColor(0x00FF00);
		} else if(state == SWITCHING || state == OFF || state == NO_DAC) {
			title->setOutlineColor(0xFFAA00);
			status->setOutlineColor(0xFFAA00);
		} else {
			title->setOutlineColor(0xFF0000);
			status->setOutlineColor(0xFF0000);
		}
	}
}

void DetailsScreen::setTargetMonitor(PowerMonitor monitorID) {
	this->monitorID = monitorID;
	title->setText(toString(monitorID));
}
