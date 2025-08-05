/*
 * CRC16.cpp
 *
 *  Created on: 15 Jun 2023
 *      Author: arion
 */

#include "CRC16.h"

#define CRC16 0x8005

/* Begin copy-paste from https://gist.github.com/2666368 */
uint16_t crc16(const uint8_t *data, uint16_t size) {
    uint16_t out = 0;
    int bits_read = 0, bit_flag;

    /* Sanity check: */
    if(data == nullptr)
        return 0;

    while(size > 0)
    {
        bit_flag = out >> 15;

        /* Get next bit: */
        out <<= 1;
        out |= (*data >> (7 - bits_read)) & 1;

        /* Increment bit counter: */
        bits_read++;
        if(bits_read > 7)
        {
            bits_read = 0;
            data++;
            size--;
        }

        /* Cycle check: */
        if(bit_flag)
            out ^= CRC16;
    }

    return out;
}
