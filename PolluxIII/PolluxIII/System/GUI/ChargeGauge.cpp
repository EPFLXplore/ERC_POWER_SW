/*
 * ChargeGauge.cpp
 *
 *  Created on: 12 Jun 2023
 *      Author: arion
 */

#include "ChargeGauge.h"

ChargeGauge::ChargeGauge(MessageBus* bus) {
	this->remainingTime = new Text();

	if(remainingTime == nullptr) {
		throwException("MemoryAllocationFailure");
	}

	add(remainingTime, 0.5f, 1.0f);
	remainingTime->setText("...")->setAnchor(Center)->setFontSize(26);

	bus->handle<Power_BatteryInfo>(std::bind(&ChargeGauge::handleBatteryInfo, this, std::placeholders::_1, std::placeholders::_2));
}

ChargeGauge::~ChargeGauge() {

}

void ChargeGauge::handleBatteryInfo(uint8_t sender, Power_BatteryInfo* packet) {
	if(packet->estimated_runtime >= 0 && packet->estimated_runtime < 1000) {
		char buffer[5];
		snprintf(buffer, 5, "%dm", (int) packet->estimated_runtime);
		remainingTime->setText(buffer);
	}

	setLevel(packet->charge);
}
