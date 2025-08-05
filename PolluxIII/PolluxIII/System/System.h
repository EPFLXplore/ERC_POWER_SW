/*
 * System.h
 *
 *  Created on: 23 Oct 2020
 *      Author: AV Team 2020
 */

#ifndef SYSTEM_H_
#define SYSTEM_H_

#ifdef __cplusplus

#include "Thread.h"
#include "Utils/Operators.h"

extern "C" {
#endif

void systemd_init();

#ifdef __cplusplus
}
#endif


#endif /* SYSTEM_H_ */
