/*
 * ExceptionTracker.cpp
 *
 *  Created on: 12 Jun 2023
 *      Author: arion
 */

#include "ExceptionTracker.h"

#include "Debug/Debug.h"


void ExceptionTracker::trackExceptions(ExceptionTracker* parent) {
	this->parent = parent;
}

void ExceptionTracker::__throwExc(const char* name, uint32_t line) {
	console.printf("%s occurred at line %d of %s: \r\n", name, line, __getSource());

	ExceptionTracker* current = this->parent;

	while(current != nullptr) {
		console.printf("\tUsed by %s\r\n", current->__getSource());
		current = current->parent;
	}
}
