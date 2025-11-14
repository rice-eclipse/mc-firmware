/*
 * mcp3208.h
 *
 *  Created on: Nov 7, 2025
 *      Author: Deepak
 */

#ifndef INC_MCP3208_H_
#define INC_MCP3208_H_
#include "stm32f4xx_hal.h"
#include "spi.h"
uint16_t MCP3208_GetAdcVal(int channel, int cs, SPI_HandleTypeDef* spiHandle);



#endif /* INC_MCP3208_H_ */
