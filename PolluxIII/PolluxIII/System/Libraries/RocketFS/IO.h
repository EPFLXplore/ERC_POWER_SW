/*
 * IO.h
 *
 *  Created on: 9 Aug 2022
 *      Author: arion
 */

#ifndef INC_IO_H_
#define INC_IO_H_

#include <stdint.h>

typedef enum AccessType { READ, WRITE } AccessType;

class IO {
public:
	IO() {}
	virtual ~IO() {}

	virtual int32_t access(uint32_t* address, uint32_t length, AccessType access_type) = 0;

	virtual void read(uint32_t address, uint8_t* buffer, uint32_t length) = 0;
	virtual void write(uint32_t address, uint8_t* buffer, uint32_t length) = 0;

	virtual void flush() = 0;
};


#endif /* INC_IO_H_ */
