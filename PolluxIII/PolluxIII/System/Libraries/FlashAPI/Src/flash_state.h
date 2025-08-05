/*
 * flash_status.h
 *
 *  Created on: 5 Oct 2019
 *      Author: Arion
 */

#ifndef FLASH_STATE_H_
#define FLASH_STATE_H_

#include <stdint.h>

void flash_ready();
void flash_success();
void flash_error();
void flash_fatal(uint32_t error_code);

#endif /* FLASH_STATE_H_ */
