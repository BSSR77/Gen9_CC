/*
 * can.h
 *
 * Created on: Dec 4, 2016
 *     Author: jamesliu
 *       Note: the HAL CAN driver is a complete friggin hack job. No respect. Pisses me off.
 */

#ifndef CAN_H_
#define CAN_H_

#include "main.h"

#define CAN_BANKS 14
#define CAN_BUFFER_LENGTH 108 //This is approximately 2KByte

typedef struct
{
  uint32_t id;
  uint8_t dlc;
  uint8_t Data[8];
}Can_frame_core_t;

typedef struct
{
  Can_frame_core_t core;
  uint8_t isExt; //1 or 0
  uint8_t isRemote;
  int filterNum;
}Can_frame_t;

typedef struct
{
  uint32_t id;
  uint32_t mask;
  uint8_t isRemote;
  uint8_t maskRemote;
  uint8_t isExt;
  uint8_t isMasked;
  int filterNum;
}Can_filter_t; //only used for getFilter()

uint8_t Can_head;
uint8_t Can_tail;

Can_frame_t *Can_headAddr();
Can_frame_t *Can_tailAddr();
Can_frame_t *Can_getAddr(uint8_t index);

void Can_begin();

int Can_addMaskedFilterStd(uint16_t id, uint16_t mask, int isRemote/*-1 = don't care*/);
int Can_addMaskedFilterExt(uint32_t id, uint32_t mask, int isRemote/*-1 = don't care*/);
int Can_addFilterStd(uint16_t id, uint8_t isRemote);
int Can_addFilterExt(uint32_t id, uint8_t isRemote);
int Can_getFilter(Can_filter_t *target, int filterNum);
int Can_removeFilter(int filterNum);
int Can_getFilterNum(uint32_t fmi);

int Can_available();
int Can_read(Can_frame_t *target);

int Can_availableForTx();
int Can_sendStd(uint16_t id, uint8_t isRemote, uint8_t *data, uint8_t dlc);
int Can_sendExt(uint32_t id, uint8_t isRemote, uint8_t *data, uint8_t dlc);
int Can_resend();

void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan);
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);

void Can_setTxCallback(void(*pt)());
void Can_setRxCallback(void(*pt)());
void Can_setErrCallback(void(*pt)());

#endif /* CAN_H_ */
