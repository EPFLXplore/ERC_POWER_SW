#ifndef PROTOCOL_MACROS_H
#define PROTOCOL_MACROS_H


#include <cstdint>

#include "CRC16.h"

#define STANDARD_PACKET(NAME, PACKET_DEF) struct NAME { PACKET_DEF } __attribute__((packed));
#define RELIABLE_PACKET(NAME, PACKET_DEF) struct NAME { PACKET_DEF uint16_t crc; } __attribute__((packed));
#define MAKE_RELIABLE(PACKET) PACKET.crc = crc16((uint8_t*) &PACKET, sizeof(PACKET) - 2)
#define IS_RELIABLE(PACKET) PACKET.crc == crc16((uint8_t*) &PACKET, sizeof(PACKET) - 2)

#endif /* PROTOCOL_MACROS_H */
