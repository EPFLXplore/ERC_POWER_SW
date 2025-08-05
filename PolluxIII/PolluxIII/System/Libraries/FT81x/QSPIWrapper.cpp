/*
 * QSPIWrapper.cpp
 *
 *  Created on: May 29, 2023
 *      Author: arion
 */

#include "QSPIWrapper.h"

#include <cmsis_os.h>

static uint8_t buffer[256];
static uint16_t index = 0;
static uint16_t length;


HAL_StatusTypeDef QSPIWrapper::setup(uint32_t prescaler) {
	if(fake_qspi) {
		spi->Init.BaudRatePrescaler = prescaler;
		HAL_SPI_DeInit(spi);
		return HAL_SPI_Init(spi);
	} else {
		qspi->Init.ClockPrescaler = prescaler;
		HAL_QSPI_DeInit(qspi);
		HAL_QSPI_Init(qspi);
	}

	return HAL_OK;
}

HAL_StatusTypeDef QSPIWrapper::run(QSPI_CommandTypeDef *cmd, uint32_t Timeout) {
	if(fake_qspi) {
		length = 0;
		index = 0;

		if(cmd->InstructionMode == QSPI_INSTRUCTION_1_LINE) {
			buffer[index++] = cmd->Instruction;
			length++;
		} else if(cmd->InstructionMode == QSPI_INSTRUCTION_NONE) {
			// Do nothing
		} else {
			index = 0;
			length = 0;
			return HAL_ERROR;
		}

		if(cmd->AddressMode == QSPI_ADDRESS_1_LINE) {
			if(cmd->AddressSize == QSPI_ADDRESS_8_BITS) {
				buffer[index++] = (cmd->Address >> 0)  & 0xFF;
				length += 1;
			} else if(cmd->AddressSize == QSPI_ADDRESS_16_BITS) {
				buffer[index++] = (cmd->Address >> 8)  & 0xFF;
				buffer[index++] = (cmd->Address >> 0)  & 0xFF;
				length += 2;
			} else if(cmd->AddressSize == QSPI_ADDRESS_24_BITS) {
				buffer[index++] = (cmd->Address >> 16) & 0xFF;
				buffer[index++] = (cmd->Address >> 8)  & 0xFF;
				buffer[index++] = (cmd->Address >> 0)  & 0xFF;
				length += 3;
			} else if(cmd->AddressSize == QSPI_ADDRESS_32_BITS) {
				buffer[index++] = (cmd->Address >> 24) & 0xFF;
				buffer[index++] = (cmd->Address >> 16) & 0xFF;
				buffer[index++] = (cmd->Address >> 8)  & 0xFF;
				buffer[index++] = (cmd->Address >> 0)  & 0xFF;
				length += 4;
			}
		} else if(cmd->AddressMode == QSPI_ADDRESS_NONE) {
			// Do nothing
		} else {
			index = 0;
			length = 0;
			return HAL_ERROR;
		}

		index += cmd->DummyCycles;
		length += cmd->DummyCycles;

		if(cmd->DataMode == QSPI_DATA_1_LINE) {
			length += cmd->NbData;
		} else if(cmd->DataMode == QSPI_DATA_NONE) {
			// Do nothing
		} else {
			index = 0;
			length = 0;
			return HAL_ERROR;
		}

		if(index == length) {
			return flush(Timeout);
		}

		return HAL_OK;
	} else {
		return HAL_QSPI_Command(qspi, cmd, Timeout);
	}
}

HAL_StatusTypeDef QSPIWrapper::transmit(uint8_t *pData, uint32_t Timeout) {
	if(fake_qspi) {
		while(index < length) {
			buffer[index] = *(pData++);
			index++;
		}

		HAL_StatusTypeDef status = flush(Timeout);

		return status;
	} else {
		return HAL_QSPI_Transmit(qspi, pData, Timeout);
	}
}

HAL_StatusTypeDef QSPIWrapper::receive(uint8_t *pData, uint32_t Timeout) {
	if(fake_qspi) {
		taskENTER_CRITICAL();

		if(index > 0) {
			flush(Timeout);
		}

		uint32_t receive_size = length - index;
		HAL_StatusTypeDef out = HAL_SPI_Receive(spi, pData, receive_size, Timeout);

		taskEXIT_CRITICAL();

		if(out == HAL_OK) {
			return out;
		} else {
			return HAL_ERROR;
		}

		return out;
	} else {
		return HAL_QSPI_Receive(qspi, pData, Timeout);
	}
}

HAL_StatusTypeDef QSPIWrapper::flush(uint32_t Timeout) {
	return HAL_SPI_Transmit(spi, buffer, index, Timeout);
}
