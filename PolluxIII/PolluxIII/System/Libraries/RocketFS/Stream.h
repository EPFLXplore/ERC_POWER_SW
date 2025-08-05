   /*
 * stream.h
 *
 *  Created on: 17 Oct 2019
 *      Author: Arion
 */

#ifndef INC_STREAM_H_
#define INC_STREAM_H_


#include "IO.h"
#include "File.h"

#include <stdint.h>
#include <stdbool.h>


class Stream {
public:
	Stream(IO* io, uint32_t base_address, FileType type);

	void close();

	int32_t  read(uint8_t* buffer, uint32_t length);
	uint8_t  read8();
	uint16_t read16();
	uint32_t read32();
	uint64_t read64();

	void write(uint8_t* buffer, uint32_t length);
	void write8(uint8_t data);
	void write16(uint16_t data);
	void write32(uint32_t data);
	void write64(uint64_t data);

	void seek(uint32_t index);

	bool isEOF();
	FileType getFileType();

private:
	IO* io;
	FileType type;
	uint32_t read_address;
	uint32_t write_address;
	bool eof;
};

#endif /* INC_STREAM_H_ */
