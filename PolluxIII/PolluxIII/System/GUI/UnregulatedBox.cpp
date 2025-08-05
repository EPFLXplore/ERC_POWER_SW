/*
 * UnregulatedBox.cpp
 *
 *  Created on: 12 Jun 2023
 *      Author: arion
 */

#include "UnregulatedBox.h"
#include "EmbeddedGL/Theme.h"

UnregulatedBox::UnregulatedBox(MessageBus* bus, PowerMonitor monitorID) : PowerBox(bus, monitorID) {
	setBorderRadius(4.0f);
	setBackgroundColor(BACKGROUND_COLOR);
	setOutlineColor(OUTLINE_COLOR);

	title->setText(toString(monitorID))->setFontSize(27);
	voltage->setText("0.0V")->setFontSize(26);
	power->setText("0.0W")->setFontSize(26);

	add(title, 0.5f, 0.25f);
	add(voltage, 0.5f, 0.55f);
	add(power, 0.5f, 0.8f);
}
