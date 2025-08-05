/*
 * stream.c
 *
 *  Created on: 17 Oct 2019
 *      Author: Arion
 */


#include "Stream.h"


Stream::Stream(IO* io, uint32_t base_address, FileType type) :
		io(io),
		type(type),
		read_address(base_address),
		write_address(base_address),
		eof(false) {
	;
}

void Stream::close() {
	if(type == EMPTY) {
		return;
	}

	io->flush();
}

int32_t Stream::read(uint8_t* buffer, uint32_t length) {
	if(type == EMPTY) {
		return -1;
	}

	uint32_t index = 0;
	int32_t readable_length = 0;

	if(length <= 0) {
		return 0;
	}

	do {
		readable_length = io->access(&read_address, length - index, READ); // Transforms the write address (or fails if end of file) if we are at the end of a readable section

		if(readable_length <= 0) {
			eof = true;
			return index;
		} else {
			eof = false;
		}

		io->read(read_address, buffer + index, readable_length);

		index += readable_length;
		read_address += readable_length;
	} while(index < length);

	return length;
}

uint8_t Stream::read8() {
	uint8_t coder; // Used as encoder and decoder
	read(&coder, 1);
	return coder;
}

uint16_t Stream::read16() {
	uint8_t coder[2]; // Used as encoder and decoder
	read(coder, 2);

	uint64_t composition = 0ULL;

	composition |= (uint16_t) coder[1] << 8;
	composition |= (uint16_t) coder[0];

	return composition;
}

uint32_t Stream::read32() {
	uint8_t coder[4]; // Used as encoder and decoder
	read(coder, 4);

	uint64_t composition = 0ULL;

	composition |= (uint64_t) coder[3] << 24;
	composition |= (uint64_t) coder[2] << 16;
	composition |= (uint64_t) coder[1] << 8;
	composition |= (uint64_t) coder[0];

	return composition;
}

uint64_t Stream::read64() {
	uint8_t coder[8]; // Used as encoder and decoder
	read(coder, 8);

	uint64_t composition = 0ULL;

	composition |= (uint64_t) coder[7] << 56;
	composition |= (uint64_t) coder[6] << 48;
	composition |= (uint64_t) coder[5] << 40;
	composition |= (uint64_t) coder[4] << 32;
	composition |= (uint64_t) coder[3] << 24;
	composition |= (uint64_t) coder[2] << 16;
	composition |= (uint64_t) coder[1] << 8;
	composition |= (uint64_t) coder[0];

	return composition;
}

void Stream::write(uint8_t* buffer, uint32_t length) {
	if(type == EMPTY) {
		return;
	}

   uint32_t index = 0;
   int32_t writable_length = 0;

   if(length <= 0) {
   		return;
   	}

   do {
      writable_length = io->access(&write_address, length - index, WRITE); // Transforms the write address (or fails if end of file) if we are at the end of a readable section

      if(writable_length <= 0) {
         eof = true;
         return;
      } else {
         eof = false;
      }

      io->write(write_address, buffer + index, writable_length);

      index += writable_length;
      write_address += writable_length;
   } while(index < length);
}

void Stream::write8(uint8_t data) {
	write(&data, 1);
}

void Stream::write16(uint16_t data) {
	uint8_t coder[2]; // Used as encoder and decoder

	coder[0] = data;
	coder[1] = data >> 8;

	write(coder, 2);
}

void Stream::write32(uint32_t data) {
	uint8_t coder[4]; // Used as encoder and decoder

	coder[0] = data;
	coder[1] = data >> 8;
	coder[2] = data >> 16;
	coder[3] = data >> 24;

	write(coder, 4);
}

void Stream::write64(uint64_t data) {
	uint8_t coder[8]; // Used as encoder and decoder

	coder[0] = data;
	coder[1] = data >> 8;
	coder[2] = data >> 16;
	coder[3] = data >> 24;
	coder[4] = data >> 32;
	coder[5] = data >> 40;
	coder[6] = data >> 48;
	coder[7] = data >> 56;

	write(coder, 8);
}

void Stream::seek(uint32_t index) {
	this->read_address += index;
	this->write_address += index;
}

bool Stream::isEOF() {
	return eof;
}

FileType Stream::getFileType() {
	return type;
}
