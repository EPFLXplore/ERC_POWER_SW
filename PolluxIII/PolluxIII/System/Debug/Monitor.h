/*
 * monitor.h
 *
 *  Created on: 5 Sep 2020
 *      Author: AV Team 2020
 */

#ifndef APPLICATION_HOSTBOARD_INC_DEBUG_MONITOR_H_
#define APPLICATION_HOSTBOARD_INC_DEBUG_MONITOR_H_

#include <stdint.h>
#include <stdbool.h>

#define MAX_MONITORS 16

#define SUMMARY_MONITOR		 0b0001
#define POWER_MONITORS 		 0b1000


class Monitor {
public:
	bool enter(uint8_t id);
	bool exit(uint8_t id);

	void enable(uint8_t id, uint8_t location, uint8_t refresh_rate);
	void disable(uint8_t id);

private:
	uint8_t locations[MAX_MONITORS];
	uint8_t monitor_refresh_rate[MAX_MONITORS];
	uint32_t last_update[MAX_MONITORS];
};


#endif /* APPLICATION_HOSTBOARD_INC_DEBUG_MONITOR_H_ */
