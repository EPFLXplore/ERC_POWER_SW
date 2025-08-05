/*
 * PowerBox.cpp
 *
 *  Created on: 12 Jun 2023
 *      Author: arion
 */

#include "PowerBox.h"
#include "Supplies/SupplyStates.h"

PowerBox::PowerBox(MessageBus* bus, PowerMonitor monitorID) : Box(), monitorID(monitorID) {
	this->title = new Text();
	this->voltage = new Text();
	this->power = new Text();

	if(title == nullptr || voltage == nullptr || power == nullptr) {
		throwException("MemoryAllocationFailure");
	}

	bus->handle<Power_BusInfo>(std::bind(&PowerBox::handleBusInfo, this, std::placeholders::_1, std::placeholders::_2));
	bus->handle<Power_SupplyInfo>(std::bind(&PowerBox::handleSupplyInfo, this, std::placeholders::_1, std::placeholders::_2));
}

PowerBox::~PowerBox() {
	delete title;
	delete voltage;
	delete power;
}

PowerBox* PowerBox::setError(bool error) {
	if(error) {
		setOutlineColor(0xFF0000);
	} else {
		setOutlineColor(0xFFFFFF);
	}

	return this;
}

void PowerBox::handleBusInfo(uint8_t sender, Power_BusInfo* packet) {
	if(packet->bus_id == monitorID) {
		char buffer[6];

		snprintf(buffer, 6, "%.1fW", packet->power);
		power->setText(buffer);

		snprintf(buffer, 6, "%.1fV", packet->voltage);
		voltage->setText(buffer);
	}
}

void PowerBox::handleSupplyInfo(uint8_t sender, Power_SupplyInfo* packet) {
	if(packet->bus_id == monitorID) {
		enum SupplyState state = (enum SupplyState) packet->state;

		if(state == SUPPLYING) {
			setOutlineColor(0xFFFFFF);
		} else if(state == SWITCHING || state == OFF || state == NO_DAC) {
			setOutlineColor(0xFFAA00);
		} else {
			setOutlineColor(0xFF0000);
		}
	}
}
