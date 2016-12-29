/*
 * MCP2515.h
 *
 *  Created on: Dec 28, 2016
 *      Author: jamesliu
 */

#ifndef MCP2515_H_
#define MCP2515_H_

int spi_notBusy();

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
void MCP2515_EXTICallback();


#endif /* MCP2515_H_ */
