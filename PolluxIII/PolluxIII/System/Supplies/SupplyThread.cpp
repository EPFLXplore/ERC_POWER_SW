/*
 * SupplyThread.cpp
 *
 *  Created on: Apr 30, 2022
 *      Author: arion
 */


#include "SupplyThread.h"
#include "Debug/Debug.h"


#include "Libraries/MCP47FEB/MCP47FEB.h"


void SupplyThread::init() {

	osDelay(100 / portTICK_PERIOD_MS);

	uint16_t value = get(device.dac_reg);

	if(!hasNewErrors()) {
		if(value == device.reference_voltage) {
			println("Device ID %d initialized successfully", device.address);

			this->state = OFF;
			this->switching = true;
			this->supplying = true;

			println("Device ID %d started", device.address);
		} else {
			println("Device ID %d incorrectly configured. Please reflash device.", device.address);
			this->state = DAC_FAULT;
		}
	} else {
		println("Failed to initialize device ID %d", device.address);
		this->state = NO_DAC;
	}

	bus->handle<Power_SupplyControl>(std::bind(&SupplyThread::handleControlCommands, this, std::placeholders::_1, std::placeholders::_2));
}


void SupplyThread::loop() {
	float input_voltage = input_sensor->getVoltage();
	float output_voltage = output_sensor->getVoltage();
	float output_power = output_sensor->getPower();

	if(switching) {
		this->state = SWITCHING;
	}

	if(supplying) {
		this->state = SUPPLYING;
	}

	if(output_voltage < device.uvlo && supplying && switching) {
		this->state = UNDERVOLTAGE;
	}

	if(output_voltage > device.ovlo) {
		this->state = OVERVOLTAGE;
	}

	if(output_power > device.max_power) {
		this->state = OVERCURRENT;
	}

	if(input_voltage < device.min_vin) {
		this->state = VIN_TOO_LOW;
	}

	if(input_voltage > device.max_vin) {
		this->state = VIN_TOO_HIGH;
	}

	if(state == SWITCHING || state == SUPPLYING || state == UNDERVOLTAGE) {
		HAL_GPIO_WritePin(device.gpio, device.ctrl, switching ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(device.gpio, device.shdn, supplying ? GPIO_PIN_RESET : GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(device.gpio, device.shdn, GPIO_PIN_SET);
		this->supplying = false;
		HAL_GPIO_WritePin(device.gpio, device.ctrl, GPIO_PIN_RESET);
		this->switching = false;
	}

	Power_SupplyInfo packet;
	packet.bus_id = device.associated_monitor;
	packet.state = state;

	MAKE_RELIABLE(packet);

	bus->send(&packet);
}

void SupplyThread::reprogram() {

	println("Disabling supply...");

	bool switching_prev = switching;
	bool supplying_prev = supplying;

	this->switching = false;
	this->supplying = false;

	HAL_GPIO_WritePin(device.gpio, device.shdn, GPIO_PIN_SET);
	HAL_GPIO_WritePin(device.gpio, device.ctrl, GPIO_PIN_RESET);

	osDelay(1000 / portTICK_PERIOD_MS);

	println("Reprogramming output voltage...");

	while(get(MCP47_STATUS) & (1 << 7));// Wait before writing to EEPROM
	set(device.dac_reg_nv, device.reference_voltage);
	set(device.dac_reg, device.reference_voltage);

	osDelay(1000 / portTICK_PERIOD_MS);

	println("Enabling supply...");

	this->switching = switching_prev;
	this->supplying = supplying_prev;
}

void SupplyThread::setSwitching(bool switching) {
	uint8_t value = get(device.dac_reg);

	if(!hasNewErrors()) {
		if(value == device.reference_voltage) {
			this->switching = switching;
		} else {
			this->state = DAC_FAULT;
		}
	} else {
		this->state = NO_DAC;
	}
}

void SupplyThread::setSupplying(bool supplying) {
	uint8_t value = get(device.dac_reg);

	if(!hasNewErrors()) {
		if(value == device.reference_voltage) {
			this->supplying = supplying;
		} else {
			this->state = DAC_FAULT;
		}
	} else {
		this->state = NO_DAC;
	}
}

void SupplyThread::handleControlCommands(uint8_t sender, Power_SupplyControl* packet) {
	if(packet->bus_id == device.associated_monitor) {
		if(packet->command_mask & 0b10) { // Command switching state
			setSwitching(packet->command_val & 0b10);
		} else if(packet->command_mask & 0b01) { // Command supplying state
			setSupplying(packet->command_val & 0b01);
		}
	}
}

uint16_t SupplyThread::get(uint8_t reg) {
	uint8_t buffer[2];

	if(!read(reg, buffer, 2)) {
		error = true;
	}

	return (buffer[0] << 8) | buffer[1];
}

void SupplyThread::set(uint8_t reg, uint16_t value) {
	uint8_t buffer[2];

	buffer[0] = (value >> 8) & 0xFF;
	buffer[1] = value & 0xFF;

	if(!write(reg, buffer, 2)) {
		error = true;
	}
}

bool SupplyThread::read(uint8_t reg, uint8_t* data, uint8_t size) {
	vTaskSuspendAll();
	bool ok = HAL_I2C_Mem_Read(i2c, device.address, reg | MCP47_READ_CMD, 1, data, size, 10 / portTICK_PERIOD_MS) == HAL_OK;
	xTaskResumeAll();
	return ok;
}

bool SupplyThread::write(uint8_t reg, uint8_t* data, uint8_t size) {
	vTaskSuspendAll();
	bool ok = HAL_I2C_Mem_Write(i2c, device.address, reg | MCP47_WRITE_CMD, 1, data, size, 10 / portTICK_PERIOD_MS) == HAL_OK;
	xTaskResumeAll();
	return ok;
}

bool SupplyThread::enable(uint8_t reg) {
	uint8_t payload = reg | MCP47_CONF_EN;

	vTaskSuspendAll();
	bool ok = HAL_I2C_Mem_Write(i2c, device.address, payload, 1, &payload, 1, 10 / portTICK_PERIOD_MS) == HAL_OK;
	xTaskResumeAll();
	return ok;
}

bool SupplyThread::disable(uint8_t reg) {
	uint8_t payload = reg | MCP47_CONF_DIS;

	vTaskSuspendAll();
	bool ok = HAL_I2C_Mem_Write(i2c, device.address, payload, 1, &payload, 1, 10 / portTICK_PERIOD_MS) == HAL_OK;
	xTaskResumeAll();
	return ok;
}

bool SupplyThread::hasNewErrors() {
	bool output = error;
	error = false;
	return output;
}


const char* SupplyThread::getStatus() {
	return toString(state);
}


/*
 *
 * BEWARE: THE FOLLOWING CODE IS CURSED
 *
 * I'VE BURNT ONE OF ONE DAC WITH THIS PROCEDURE
 *
 */


/*
 * Procedure to flash a new I2C address to the DAC that changes the supply's voltage:
 * 1) Make sure the device.address field passed to the SupplyThread constructor is 0b11000000.
 * 2) Solder a thin wire that is mounted on the HVC pin of the MCP47FEB.
 * 3) Add hot glue around the wire to make sure it does not short other pins.
 * 4) Change the return value of this function to the desired I2C address.
 * 5) Apply a 10V/1mA signal to the HVC pin.
 * 6) Flash and run the new code.
 * 7) Disconnect to 10V supply to the HVC pin.
 * 8) Remove the thin wire from the HVC pin.
 * 9) Change the return value of this function to "return old_address".
 * 10) Change the device.address field passed to the SupplyThread constructor to the new address.
 */

/*


static uint8_t flash_new_address(uint8_t old_address) {
	return 0x42;
}

uint16_t address_mask = 0b1111111;

uint16_t address_reg = get(MCP47_ADDR_NV);
uint16_t address = address_reg & address_mask;
uint8_t new_address = flash_new_address(address);

println("Current address is %d", address_reg);

if(address != new_address) {
	bool initially_locked = false;

	println("Waiting for 10V on HVC pin for device ID %d...", device.address);

	while((get(MCP47_ADDR_NV) & (1 << 7)) != 0) {
		disable(MCP47_CONF_UNKNOWN); // Disable the address lock bit
		initially_locked = true;
	}

	println("Waiting for 0V on HVC pin for device ID %d...", device.address);

	println("New status: %d", get(MCP47_ADDR_NV));

	while(get(MCP47_ADDR_NV) == 0);

	println("Unlock procedure successful for device ID %d", device.address);

	if((get(MCP47_ADDR_NV) & (1 << 7)) == 0) {
		println("Address configuration register unlocked for device ID %d", device.address);

		while(get(MCP47_STATUS) & (1 << 7)); // Wait before writing to EEPROM
		set(MCP47_ADDR_NV, (address_reg & ~address_mask) | new_address); // Update address

		if((get(MCP47_ADDR_NV) & address_mask) == new_address) {
			println("Device ID %d has been updated to new ID %d", device.address, new_address);
		} else {
			println("Failed to update device ID %d", device.address);
		}

		if(initially_locked) {
			enable(MCP47_CONF_SALCK); // Enable the address lock bit

			if((get(MCP47_ADDR_NV) & (1 << 8)) == 0) {
				println("Failed to lock address register for device ID %d", device.address);
			} else {
				println("Address configuration register locked for device ID %d", device.address);
			}
		}
	} else {
		println("Failed to unlock address register for device ID %d", device.address);
	}
}

uint16_t lock_status = get(MCP47_LOCK_STATUS);

println("Lock status is %d", lock_status);

while(get(MCP47_STATUS) & (1 << 7)); // Wait before writing to EEPROM

*/
