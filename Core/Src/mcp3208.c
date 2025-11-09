/*
 * mcp3208.c
 *
 *  Created on: Nov 7, 2025
 *      Author: Deepak
 */
#include "mcp3208.h"
uint16_t MCP3208_GetAdcVal(int channel, int cs, SPI_HandleTypeDef* spiHandle){
	int status = 0;
	uint8_t tx[3];
	uint8_t rx[3];

	//start + single-ended + D2
	tx[0] = 0x06 | ((channel & 0x04) >> 2);
	//D1 + D0 shifted to B7 and B6
	tx[1] = (channel & 0x03) << 6;
	//don't care
	tx[2] = 0x00;

	HAL_GPIO_WritePin(GPIOC, cs, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(spiHandle, tx, rx, 3, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(GPIOC, cs, GPIO_PIN_SET);

	uint16_t dataBuff = ((rx[1] & 0x0F) << 8) | rx[2];

	return dataBuff;

}


