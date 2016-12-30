/*
 * MCP2515.h
 *
 *  Created on: Dec 28, 2016
 *      Author: jamesliu
 */

#ifndef MCP2515_H_
#define MCP2515_H_

#include "cmsis_os.h"
#include "can.h"

void MCP2515_initMCP2515_init(SPI_HandleTypeDef *newhspi, osMessageQId tx, osMessageQId rx, GPIO_TypeDef *csPort, uint16_t csPin);
void MCP2515_reset(); //init calls reset
void MCP2515_write(Can_frame_t *frame); //sends to queue and initiates transaction
uint8_t MCP2515_getTxErrorCounter(); /*TODO*/
uint8_t MCP2515_getRxErrorCounter(); /*TODO*/

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
void MCP2515_EXTICallback();

#endif /* MCP2515_H_ */
