/*
 * Copyright (c) 2025 Théo Heng
 *
 * This file is part of the bq25756e_multiplatform library.
 *
 * Licensed under the MIT License. See the LICENSE file in the project root for full license information.
 */

#include "I2C.h"

// Write a value to a specific DRV8214 register for a given driver address
void writeRegister(uint8_t address, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

void writeRegister16(uint8_t address, uint8_t reg, uint16_t value) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value & 0xFF);          // Write LSB
    Wire.write((value >> 8) & 0xFF);   // Write MSB first
    Wire.endTransmission();
}

uint8_t readRegister(uint8_t address, uint8_t reg) {
    Wire.beginTransmission(address);
    Wire.write(reg);
    // End the transmission with a STOP condition instead of a repeated start
    // True = send STOP condition after transmission
    // False = send a repeated start after transmission
    // If Wire.endTransmission(false) fails but Wire.endTransmission(true) works, the device might not support a repeated start condition.
    int errorCode = Wire.endTransmission(false);
    if (errorCode != 0) {
        return errorCode;  // Return the actual error code
    }
    
    // Request a single byte from the device
    Wire.requestFrom(address, (uint8_t)1);
    if (Wire.available()) {
        return Wire.read();
    }
    return 0; // Return 0 if no data was received
}

uint16_t readRegister16(uint8_t address, uint8_t reg) { 
    // 0) Send register address
    Wire.beginTransmission(address);
    Wire.write(reg);  
    // 1) End the transmission with a repeated start
    Wire.endTransmission(false);

    // 2) Request 2 bytes
    Wire.requestFrom((int)address, 2);

    // 3) Read LSB then MSB (typical for TI registers)
    uint8_t low_bits  = Wire.read();
    uint8_t high_bits = Wire.read();

    // 4) Combine into 16-bit
    return (static_cast<uint16_t>(high_bits) << 8) | low_bits;
}

// Modify specific bits in a specific DRV8214 register
void modifyRegister(uint8_t address, uint8_t reg, uint8_t mask, bool enable) {
    uint8_t value = readRegister(address, reg);
    if (enable) {
        value |= mask;  // Set bits
    } else {
        value &= ~mask; // Clear bits
    }
    writeRegister(address, reg, value);
}

// Modify specific bits in a specific DRV8214 register using a new value
void modifyRegisterBits(uint8_t address, uint8_t reg, uint8_t mask, uint8_t newValue) {
    uint8_t current = readRegister(address, reg);
    current = (current & ~mask) | (newValue & mask);
    writeRegister(address, reg, current);
}
