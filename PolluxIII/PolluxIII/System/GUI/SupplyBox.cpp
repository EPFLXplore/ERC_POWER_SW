/*
 * SupplyBox.cpp
 *
 *  Created on: 12 Jun 2023
 *      Author: arion
 */

#include "SupplyBox.h"

SupplyBox::SupplyBox(enum Anchor align, MessageBus* bus, PowerMonitor monitorID) : PowerBox(bus, monitorID) {
	setBorderRadius(1.0f);

	title->setAnchor(align)->setText(toString(monitorID))->setFontSize(29);
	voltage->setText("0.0V")->setFontSize(31);
	power->setText("0.0W")->setFontSize(27);

	if(align == West) {
		add(title, 0.05f, 0.15f);
	} else if(align == East) {
		add(title, 0.95f, 0.15f);
	} else {
		throwException("Not implemented");
	}

	add(voltage, 0.5f, 0.5f);
	add(power, 0.5f, 0.75f);
}
