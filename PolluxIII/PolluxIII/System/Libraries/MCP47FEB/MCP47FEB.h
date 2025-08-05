/*
 * MCP47FEB.h
 *
 *  Created on: May 16, 2023
 *      Author: arion
 */

#ifndef LIBRARIES_MCP47FEB_MCP47FEB_H_
#define LIBRARIES_MCP47FEB_MCP47FEB_H_


#define MCP47_DAC0			0x00 << 3
#define MCP47_DAC1			0x01 << 3
#define MCP47_VREF			0x08 << 3
#define MCP47_PD			0x09 << 3
#define MCP47_STATUS		0x0A << 3
#define MCP47_LOCK_STATUS	0x0B << 3

#define MCP47_DAC0_NV		0x10 << 3
#define MCP47_DAC1_NV		0x11 << 3
#define MCP47_VREF_NV		0x18 << 3
#define MCP47_PD_NV			0x19 << 3
#define MCP47_ADDR_NV		0x1A << 3

#define MCP47_CONF_CL0		0x00 << 3
#define MCP47_CONF_CL1		0x01 << 3
#define MCP47_CONF_DL0		0x10 << 3
#define MCP47_CONF_DL1		0x11 << 3
#define MCP47_CONF_SALCK	0x1A << 3
#define MCP47_CONF_UNKNOWN	0x1B << 3 // Some undocumented feature ???

#define MCP47_WRITE_CMD 	0b00 << 1
#define MCP47_READ_CMD  	0b11 << 1
#define MCP47_CONF_EN 		0b10 << 1
#define MCP47_CONF_DIS 		0b01 << 1


#endif /* LIBRARIES_MCP47FEB_MCP47FEB_H_ */
