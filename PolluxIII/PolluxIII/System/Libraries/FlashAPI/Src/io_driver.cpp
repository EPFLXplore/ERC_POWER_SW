/*
 * io_driver.c
 *
 *  Created on: 1 Oct 2019
 *      Author: Arion
 */

#include "io_driver.h"
#include "flash_state.h"
#include "MT25QL512ABB.h"


/*
 * Reads the flag status register and returns the value of the 8-bits register
 */
uint8_t __read_flags() {
	Command cmd = { 0 };
	load_default_command(&cmd);
	with_data(&cmd, 1);

	if(!qspi_run(&cmd, READ_FLAG_STATUS_REGISTER)) {
		flash_fatal(ERROR_READ | ERROR_RUN);
	}

	uint8_t flags;

	if(!qspi_receive(&flags)) {
		flash_fatal(ERROR_READ | ERROR_RECEIVE);
	}

	return flags;
}

uint8_t __read_status() {
	Command cmd = { 0 };
	load_default_command(&cmd);
	with_data(&cmd, 1);

	if(!qspi_run(&cmd, READ_STATUS_REGISTER)) {
		flash_fatal(ERROR_READ | ERROR_RUN);
	}

	uint8_t flags;

	if(!qspi_receive(&flags)) {
		flash_fatal(ERROR_READ | ERROR_RECEIVE);
	}

	return flags;
}



/*
 * Enables the write latch.
 * This function must be called before each PROGRAM or ERASE operation.
 */
bool __write_enable_latch() {
	Command cmd = { 0 };
	load_default_command(&cmd);

	if(qspi_run(&cmd, WRITE_ENABLE_LATCH)) {
		load_default_command(&cmd);
		with_data(&cmd, 1);

		if(qspi_poll(&cmd, READ_STATUS_REGISTER, 1, true)) {
			return true;
		}
	}

	while((__read_status() & (1 << 1)) == 0); // While is not latching

	return false;
}

/*
 * Call this function to prevent data corruption when a hardware fault (e.g. protection fault) occurs.
 * Please refer to the documentation for details.
 */
bool __write_disable_latch() {
	Command cmd = { 0 };
	load_default_command(&cmd);
	return qspi_run(&cmd, WRITE_DISABLE_LATCH);
}

/*
 * Initialises the flash driver
 */
void flash_init() {
	Command cmd = { 0 };
	load_default_command(&cmd);

	cmd.qspi_command.InstructionMode =	QSPI_INSTRUCTION_1_LINE;
	cmd.qspi_command.Instruction = 0x35;

	if(HAL_QSPI_Command(&hqspi, &(cmd.qspi_command), IO_TIMEOUT) != HAL_OK) {
		flash_fatal(ERROR_WRITE | ERROR_RUN);
	}

	uint16_t nv_configuration = 0b0000111001100111;

	load_default_command(&cmd);
	with_data(&cmd, 2);

	if(!qspi_run(&cmd, 0xB5)) { // Write non-volatile configuration register
		flash_fatal(ERROR_WRITE | ERROR_RUN);
	}

	uint16_t data;

	if(!qspi_receive((uint8_t*) &data)) {
		flash_fatal(ERROR_READ | ERROR_RECEIVE);
	}

	if(data != nv_configuration) {
		__write_enable_latch();

		load_default_command(&cmd);
		with_data(&cmd, 2);

		if(!__write_enable_latch()) {
			flash_fatal(ERROR_WRITE | ERROR_RUN);
		}

		if(!qspi_run(&cmd, 0xB1)) { // Write non-volatile configuration register
			flash_fatal(ERROR_WRITE | ERROR_RUN);
		}

		if(!qspi_transmit((uint8_t*) &nv_configuration)) {
			flash_fatal(ERROR_WRITE | ERROR_RUN);
		}

		while((__read_flags() & (1 << 7)) == 0);
		while((__read_status() & (1 << 1)) == (1 << 1)); // While is latching
	}
}

/*
 *
 * --- Read operations ---
 *
 * Test providers:
 * 	 - read_ut.c
 *
 */


void reset_memory() {

	Command cmd = { 0 };
	load_default_command(&cmd);

	if(!qspi_run(&cmd, 0x66)) { // Write volatile configuration register
		flash_fatal(ERROR_WRITE | ERROR_RUN);
	}

	if(!qspi_run(&cmd, 0x99)) { // Write volatile configuration register
		flash_fatal(ERROR_WRITE | ERROR_RUN);
	}
}

void flash_read(uint32_t address, uint8_t* buffer, uint32_t length) {
	Command cmd;
	load_default_command(&cmd);
	with_address(&cmd, address);
	with_data(&cmd, length);

	cmd.qspi_command.DummyCycles = 10;

	if(!qspi_run(&cmd, FREAD_QUAD)) {
		flash_fatal(ERROR_READ | ERROR_RUN);
	}

	if(!qspi_receive(buffer)) {
		flash_fatal(ERROR_READ | ERROR_RECEIVE);
	}
}



/*
 *
 * --- Write operations ---
 *
 * Test providers:
 * 	 - write_ut.c
 *
 */

void __flash_write_page(uint32_t address, uint8_t* buffer, uint32_t length) {
	__write_enable_latch();

	Command cmd;
	load_default_command(&cmd);

	with_address(&cmd, address);
	with_data(&cmd, length);

	if(!qspi_run(&cmd, FWRITE_QUAD)) {
		flash_fatal(ERROR_WRITE | ERROR_RUN);
	}

	if(!qspi_transmit(buffer)) {
		flash_fatal(ERROR_WRITE | ERROR_TRANSMIT);
	}

	/*
	 * Checks if the controller is ready to proceed to the next command
	 */


	while((__read_status() & 1) == 1); // While is writing
	while((__read_flags() & (1 << 7)) == 0); // While is not ready
	while((__read_status() & (1 << 1)) == (1 << 1)); // While is latching
}

void flash_write(uint32_t address, uint8_t* buffer, uint32_t length) {
	uint32_t internal_address = address % PAGE_SIZE;

	while(internal_address + length > PAGE_SIZE) {
		uint32_t write_length = PAGE_SIZE - internal_address;

		__flash_write_page(address, buffer, write_length);
		buffer += write_length;
		address += write_length;
		length -= write_length;

		internal_address = 0;
	}

	__flash_write_page(address, buffer, length);
}

/*
 *
 * --- Erase operations ---
 *
 * Test providers:
 * 	 - erase_ut.c
 *
 */
void flash_erase_all() {
	for(uint32_t i = 0; i < NUM_SECTORS; i++) {
		flash_erase_sector(i * SECTOR_SIZE);
	}
}

void __NOT_WORKING__flash_erase_all() {
   __write_enable_latch();

   Command cmd = { 0 };
   load_default_command(&cmd);

   if(!qspi_run(&cmd, ERASE_ALL)) {
      flash_fatal(ERROR_ERASE | ERROR_RUN);
   }

   /*
    * Checks if the controller is ready to proceed to the next command
    */

	while((__read_flags() & (1 << 7)) == 0); // Wait until controller is ready

   /*
    * Checks if the protection fault flag is set
    */
   uint8_t flags = __read_flags();

   if(flags & (1 << 5)) {
      flash_fatal(ERROR_ERASE | ERROR_STATE);
   }

   if((__read_status() & 0b01) == 0b01) {
   		__write_disable_latch();
   	}
}

void __flash_erase(uint32_t instruction, uint32_t address) {
	__write_enable_latch();


	Command cmd = { 0 };
	load_default_command(&cmd);
	with_address(&cmd, address);


	if(!qspi_run(&cmd, instruction)) {
		flash_fatal(ERROR_ERASE | ERROR_RUN);
	}

	/*
	 * Checks if the controller is ready to proceed to the next command
	 */

	while((__read_status() & 1) == 1); // While is writing
	while((__read_flags() & (1 << 7)) == 0); // Wait until controller is ready
	while((__read_status() & 0b01) == 0b01);
}

/*
 * Erases the whole sector represented by the provided address.
 * The address may be any of those within the sector.
 */
void flash_erase_sector(uint32_t address) {
	__flash_erase(ERASE_SECTOR, address);
}


/*
 * Erases the whole sub-sector represented by the provided address.
 * The address may be any of those within the sub-sector.
 */
void flash_erase_subsector(uint32_t address) {
	__flash_erase(ERASE_SUBSECTOR, address);
}
