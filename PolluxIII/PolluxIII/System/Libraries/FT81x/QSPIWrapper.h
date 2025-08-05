/*
 * QSPIWrapper.h
 *
 *  Created on: May 29, 2023
 *      Author: arion
 */

#ifndef LIBRARIES_FT81X_QSPIWRAPPER_H_
#define LIBRARIES_FT81X_QSPIWRAPPER_H_

#include "Libraries/Utils/ExceptionTracker.h"

#include <spi.h>
#include <quadspi.h>

class QSPIWrapper : public ExceptionTracker {
public:
	TRACK_EXCEPTIONS();

	QSPIWrapper(SPI_HandleTypeDef* spi) : spi(spi), qspi(nullptr), fake_qspi(true) {}
	QSPIWrapper(QSPI_HandleTypeDef* qspi) : spi(nullptr), qspi(qspi), fake_qspi(false) {}
	~QSPIWrapper() {}

	HAL_StatusTypeDef setup(uint32_t prescaler);
	HAL_StatusTypeDef run(QSPI_CommandTypeDef *cmd, uint32_t Timeout);
	HAL_StatusTypeDef transmit(uint8_t *pData, uint32_t Timeout);
	HAL_StatusTypeDef receive(uint8_t *pData, uint32_t Timeout);

private:
	SPI_HandleTypeDef* spi;
	QSPI_HandleTypeDef* qspi;
	bool fake_qspi;

	HAL_StatusTypeDef flush(uint32_t Timeout);
};


#endif /* LIBRARIES_FT81X_QSPIWRAPPER_H_ */
