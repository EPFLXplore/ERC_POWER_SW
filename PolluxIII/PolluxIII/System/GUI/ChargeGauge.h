/*
 * ChargeGauge.h
 *
 *  Created on: 12 Jun 2023
 *      Author: arion
 */

#ifndef GUI_CHARGEGAUGE_H_
#define GUI_CHARGEGAUGE_H_

#include "EmbeddedGL/Components/Gauge.h"
#include "EmbeddedGL/Components/Text.h"

#include "RoCo/RoCo.h"

class ChargeGauge : public Gauge {
public:
	TRACK_EXCEPTIONS();

	ChargeGauge(MessageBus* bus);
	~ChargeGauge();

private:
	Text* remainingTime;
	void handleBatteryInfo(uint8_t sender, Power_BatteryInfo* packet);
};

#endif /* GUI_CHARGEGAUGE_H_ */
