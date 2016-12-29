/*
 * can.c
 *
 *  Created on: Dec 4, 2016
 *      Author: jamesliu
 */

#include "can.h"

extern CAN_HandleTypeDef hcan1;

/*
 * This is how HAL handles the filter values:
 *
 * 16 bit filters:
 * FR1 = ((0x0000FFFF & FilterMaskIdLow) << 16) | (0x0000FFFF & FilterIdLow);
 * FR2 = ((0x0000FFFF & FilterMaskIdHigh) << 16) | (0x0000FFFF & FilterIdHigh);
 *
 * 32 bit filters:
 * FR1 = ((0x0000FFFF & FilterIdHigh) << 16) | (0x0000FFFF & FilterIdLow);
 * FR2 = ((0x0000FFFF & FilterMaskIdHigh) << 16) | (0x0000FFFF & FilterMaskIdLow);
 *
 * A mask is also more significant than its corresponding ID, apparently.
 */

static CAN_FilterConfTypeDef Can_filters[CAN_BANKS];
static uint8_t Can_filterCapacity[CAN_BANKS];
static uint8_t Can_filterUsage[CAN_BANKS]; //one bit for each slot, msb is msb of reg1
static Can_frame_t Can_rxBuffer[CAN_BUFFER_LENGTH];

static CanTxMsgTypeDef txFrameBuf;
static CanRxMsgTypeDef rxFrameBuf;

uint8_t Can_head = 0, Can_tail = 0;
static uint8_t Can_ovf = 0;

static void (*Can_txcb)(); //don't touch. user callbacks.
static void (*Can_rxcb)();
static void (*Can_ercb)();

static void empty(){}


Can_frame_t *Can_headAddr(){
	return &Can_rxBuffer[Can_head];
}


Can_frame_t *Can_tailAddr(){
	return &Can_rxBuffer[Can_tail];
}


Can_frame_t *Can_getAddr(uint8_t index){
	return &Can_rxBuffer[index];
}

void Can_begin(){
	Can_txcb = empty;
	Can_rxcb = empty;
	Can_ercb = empty;
	hcan1.pRxMsg = &rxFrameBuf;
	HAL_CAN_Receive_IT(&hcan1,0);
}


int Can_addMaskedFilterStd(uint16_t id, uint16_t mask, int isRemote){ //2 slots per bank
	uint8_t rtr = (isRemote==0)?0:1;
	uint8_t rtrm = (isRemote<0)?0:1; //0 = don't care, 1 = must match
	for(int i=0; i<CAN_BANKS; i++){ //add to existing available bank
		uint8_t *usage = &Can_filterUsage[i];
		uint8_t *capacity = &Can_filterCapacity[i];
		if(*capacity==2 && *usage>0 && *usage<0x03 &&
				Can_filters[i].FilterScale==0){ //if in use but unfilled
			if((*usage&0x02) == 0){ //if slot 2 unused, so 3 must be in use
				Can_filters[i].FilterIdLow = id<<5 | rtr<<4 | 0<<3; //id|rtr|ide
				Can_filters[i].FilterMaskIdLow = mask<<5 | rtrm << 4 | 1<<3;
				*usage |= 0x02;
				HAL_CAN_ConfigFilter(&hcan1, &Can_filters[i]);
				return 4*i + 2; //"usage" index = xxxx0123
			}else{ //slot 2 in use, so slot 3 must be empty
				Can_filters[i].FilterIdHigh = id<<5 | rtr<<4 | 0<<3; //id|rtr|ide
				Can_filters[i].FilterMaskIdHigh = mask<<5 | rtrm << 4 | 1<<3;
				*usage |= 0x01;
				HAL_CAN_ConfigFilter(&hcan1, &Can_filters[i]);
				return 4*i + 3; //"usage" index = xxxx0123
			}
		}
	}
	for(int i=0; i<CAN_BANKS; i++){ //open up a fresh bank
		if(Can_filterUsage[i]==0){ //find unused bank
			Can_filterUsage[i] |= 0x02; //use highest slot first
			Can_filterCapacity[i] = 2;
			Can_filters[i].FilterIdLow = id<<5 | rtr<<4 | 0<<3; //id|rtr|ide
			Can_filters[i].FilterMaskIdLow = mask<<5 | rtrm << 4 | 1<<3;
			Can_filters[i].FilterIdHigh = 0; //0 should not be a valid addr
			Can_filters[i].FilterMaskIdHigh = ~0;
			Can_filters[i].FilterActivation = ENABLE;
			Can_filters[i].FilterFIFOAssignment = 0;
			Can_filters[i].BankNumber = i+1; //This one isn't even used us HAL...
			Can_filters[i].FilterNumber = i;
			Can_filters[i].FilterMode = 0; //0 for mask, 1 for list
			Can_filters[i].FilterScale = 0; //0 for 16, 1 for 32
			HAL_CAN_ConfigFilter(&hcan1, &Can_filters[i]);
			return 4*i + 2; //"usage" index = xxxx0123
		}
	}
	return -1;
}


int Can_addMaskedFilterExt(uint32_t id, uint32_t mask, int isRemote){ //1 slot per bank
	uint8_t rtr = (isRemote==0)?0:1;
	uint8_t rtrm = (isRemote<0)?0:1; //0 = don't care, 1 = must match
	for(int i=0; i<CAN_BANKS; i++){ //open up a fresh bank
		if(Can_filterUsage[i]==0){ //find unused bank
			Can_filterUsage[i] |= 0x01; //use highest slot first
			Can_filterCapacity[i] = 1;
			Can_filters[i].FilterIdHigh = id>>13; //id[28:13]
			Can_filters[i].FilterIdLow = ((id<<3)&0xffff) | 1<<2 | rtr<<1; //id[12:0]|ide|rtr|0
			Can_filters[i].FilterMaskIdHigh = mask>>13; //mask[28:13]
			Can_filters[i].FilterMaskIdLow = ((mask<<3)&0xffff) | 1<<2 | rtrm<<1; //mask[12:0]|1|rtrm|0
			Can_filters[i].FilterActivation = ENABLE;
			Can_filters[i].FilterFIFOAssignment = 0;
			Can_filters[i].BankNumber = i+1; //This one isn't even used us HAL...
			Can_filters[i].FilterNumber = i;
			Can_filters[i].FilterMode = 0; //0 for mask, 1 for list
			Can_filters[i].FilterScale = 1; //0 for 16, 1 for 32
			HAL_CAN_ConfigFilter(&hcan1, &Can_filters[i]);
			return 4*i + 3; //"usage" index = xxxx0123
		}
	}
	return -1;
}

int Can_addFilterStd(uint16_t id, uint8_t isRemote){ //4 slots per bank
	isRemote = (isRemote==0)?0:1;
	for(int i=0; i<CAN_BANKS; i++){ //add to existing available bank
		uint8_t *usage = &Can_filterUsage[i];
		uint8_t *capacity = &Can_filterCapacity[i];
		if(*capacity==4 && *usage>0 && *usage<0x0f){ //if in use but unfilled
			uint8_t openSlot;
			if((*usage&0x08)==0){
				openSlot = 0;
				Can_filters[i].FilterMaskIdLow = id<<5 | isRemote<<4 | 0<<3; //id|rtr|ide
			}else if((*usage&0x04)==0){
				openSlot = 1;
				Can_filters[i].FilterIdLow = id<<5 | isRemote<<4 | 0<<3; //id|rtr|ide
			}else if((*usage&0x02)==0){
				openSlot = 2;
				Can_filters[i].FilterMaskIdHigh = id<<5 | isRemote<<4 | 0<<3; //id|rtr|ide
			}else{
				openSlot = 3;
				Can_filters[i].FilterIdHigh = id<<5 | isRemote<<4 | 0<<3; //id|rtr|ide
			}
			*usage |= 0x08>>openSlot;
			HAL_CAN_ConfigFilter(&hcan1, &Can_filters[i]);
			return 4*i + openSlot;
		}
	}
	for(int i=0; i<CAN_BANKS; i++){ //open up a fresh bank
		if(Can_filterUsage[i]==0){ //find unused bank
			Can_filterUsage[i] |= 0x08; //use highest slot first
			Can_filterCapacity[i] = 4;
			Can_filters[i].FilterMaskIdLow = id<<5 | isRemote<<4 | 0<<3; //id|rtr|ide
			Can_filters[i].FilterIdLow = 0; //0 should not be a valid addr
			Can_filters[i].FilterMaskIdHigh = 0;
			Can_filters[i].FilterIdHigh = 0;
			Can_filters[i].FilterActivation = ENABLE;
			Can_filters[i].FilterFIFOAssignment = 0;
			Can_filters[i].BankNumber = i+1; //This one isn't even used us HAL...
			Can_filters[i].FilterNumber = i;
			Can_filters[i].FilterMode = 1; //0 for mask, 1 for list
			Can_filters[i].FilterScale = 0; //0 for 16, 1 for 32
			HAL_CAN_ConfigFilter(&hcan1, &Can_filters[i]);
			return 4*i + 0; //"usage" index = xxxx0123
		}
	}
	return -1;
}

int Can_addFilterExt(uint32_t id, uint8_t isRemote){ //2 slots per bank
	isRemote = (isRemote==0)?0:1;
	for(int i=0; i<CAN_BANKS; i++){ //add to existing available bank
		uint8_t *usage = &Can_filterUsage[i];
		uint8_t *capacity = &Can_filterCapacity[i];
		if(*capacity==2 && *usage>0 && *usage<0x03 &&
				Can_filters[i].FilterScale==1){ //if in use but unfilled
			if((*usage&0x02) == 0){ //if slot 2 unused, so 3 must be in use
				Can_filters[i].FilterIdHigh = id>>13; //id[28:13]
				Can_filters[i].FilterIdLow = ((id<<3)&0xffff) | 1<<2 | isRemote << 1; //id[12:0]|ide|rtr|0
				*usage |= 0x02;
				HAL_CAN_ConfigFilter(&hcan1, &Can_filters[i]);
				return 4*i + 2; //"usage" index = xxxx0123
			}else{ //slot 2 in use, so slot 3 must be empty
				Can_filters[i].FilterMaskIdHigh = id>>13; //id[28:13]
				Can_filters[i].FilterMaskIdLow = ((id<<3)&0xffff) | 1<<2 | isRemote << 1; //id[12:0]|ide|rtr|0
				*usage |= 0x01;
				HAL_CAN_ConfigFilter(&hcan1, &Can_filters[i]);
				return 4*i + 3; //"usage" index = xxxx0123
			}
		}
	}
	for(int i=0; i<CAN_BANKS; i++){ //open up a fresh bank
		if(Can_filterUsage[i]==0){ //find unused bank
			Can_filterUsage[i] |= 0x02; //use highest slot first
			Can_filterCapacity[i] = 2;
			Can_filters[i].FilterIdHigh = id>>13; //id[28:13]
			Can_filters[i].FilterIdLow = ((id<<3)&0xffff) | 1<<2 | isRemote << 1; //id[12:0]|ide|rtr|0
			Can_filters[i].FilterMaskIdHigh = 0; //0 should not be a valid addr
			Can_filters[i].FilterMaskIdLow = 0;
			Can_filters[i].FilterActivation = ENABLE;
			Can_filters[i].FilterFIFOAssignment = 0;
			Can_filters[i].BankNumber = i+1; //This one isn't even used us HAL...
			Can_filters[i].FilterNumber = i;
			Can_filters[i].FilterMode = 1; //0 for mask, 1 for list
			Can_filters[i].FilterScale = 1; //0 for 16, 1 for 32
			HAL_CAN_ConfigFilter(&hcan1, &Can_filters[i]);
			return 4*i + 2; //"usage" index = xxxx0123
		}
	}
	return -1;
}

int Can_getFilter(Can_filter_t *target, int filterNum){
	CAN_FilterConfTypeDef *bank = &Can_filters[filterNum/4];
	if(bank->FilterActivation == ENABLE){ //optimizing this by hand is just not worth it
		target->filterNum = filterNum;
		filterNum %= 4;
		if(bank->FilterScale == 0x00){ //16 bit
			target->isExt = 0;
			if(bank->FilterMode == 0x00){ //mask
				target->isMasked = 1;
				if(filterNum == 2){ //slot 2
					target->id = bank->FilterIdLow>>5;
					target->mask = bank->FilterMaskIdLow>>5;
					target->isRemote = (bank->FilterIdLow>>4)&1;
					target->maskRemote = (bank->FilterMaskIdLow>>4)&1;
				}else{ //slot 3
					target->id = bank->FilterIdHigh>>5;
					target->mask = bank->FilterMaskIdHigh>>5;
					target->isRemote = (bank->FilterIdHigh>>4)&1;
					target->maskRemote = (bank->FilterMaskIdHigh>>4)&1;
				}
			}else{ //list
				target->isMasked = 0;
				switch(filterNum){
				case 0:
					target->id = bank->FilterMaskIdLow>>5;
					target->isRemote = (bank->FilterMaskIdLow>>4)&1;
					break;
				case 1:
					target->id = bank->FilterIdLow>>5;
					target->isRemote = (bank->FilterIdLow>>4)&1;
					break;
				case 2:
					target->id = bank->FilterMaskIdHigh>>5;
					target->isRemote = (bank->FilterMaskIdHigh>>4)&1;
					break;
				case 3:
					target->id = bank->FilterIdHigh>>5;
					target->isRemote = (bank->FilterIdHigh>>4)&1;
					break;
				}
			}
		}else{ //32 bit
			target->isExt = 1;
			if(bank->FilterMode == 0x00){ //mask
				target->isMasked = 1;
				target->id = (bank->FilterIdLow>>3) | (bank->FilterIdHigh<<13);
				target->mask = (bank->FilterMaskIdLow>>3) | (bank->FilterMaskIdHigh<<13);
				target->isRemote = (bank->FilterIdLow>>1)&1;
				target->maskRemote = (bank->FilterMaskIdLow>>1)&1;
			}else{ //list
				target->isMasked = 0;
				if(filterNum == 2){ //slot 2
					target->id = (bank->FilterIdLow>>3) | (bank->FilterIdHigh<<13);
					target->isRemote = (bank->FilterIdLow>>1)&1;
				}else{ //slot 3
					target->id = (bank->FilterMaskIdLow>>3) | (bank->FilterMaskIdHigh<<13);
					target->isRemote = (bank->FilterMaskIdLow>>1)&1;
				}
			}
		}
		return 0;
	}
	return -1;
}

int Can_removeFilter(int filterNum){
	int bankNum = filterNum/4;
	CAN_FilterConfTypeDef *bank = &Can_filters[bankNum];
	filterNum %= 4;
	if(bank->FilterActivation == ENABLE){
		Can_filterUsage[bankNum] &= ~(1<<(3-filterNum));
		if(Can_filterUsage[bankNum] == 0x00){
			bank->FilterActivation = DISABLE;
			Can_filterCapacity[bankNum] = 0;
		}
		return 0;
	}
	return -1;
}

int Can_getFilterNum(uint32_t fmi){
	int result = 0; //fmi is 0 indexed
	for(int i=0; i<CAN_BANKS; i++){
		if(Can_filterCapacity[i] > fmi){ //if target is in ith bank
			result += 4-Can_filterCapacity[i]+fmi;
			return result;
		}
		result += 4;
		fmi -= Can_filterCapacity[i];
	}
	return -1;
}


int Can_available(){
	if(Can_ovf==0){
		return Can_head - Can_tail;
	}else if((Can_ovf==1) && (Can_head <= Can_tail)){
		return CAN_BUFFER_LENGTH - (Can_tail - Can_head);
	}else{
		Can_tail = Can_head;
		Can_ovf = 1;
		return CAN_BUFFER_LENGTH;
	}
}


int Can_read(Can_frame_t *target){
	if(Can_available()){
		Can_frame_t *tailItem = Can_tailAddr();
		target->core = tailItem->core;
		target->isExt = tailItem->isExt;
		target->isRemote = tailItem->isRemote;
		target->filterNum = tailItem->filterNum;
		Can_tail++;
		if(Can_tail >= CAN_BUFFER_LENGTH){
			Can_tail = 0;
			Can_ovf--;
		}
		return 0;
	}
	return -1;
}

int Can_availableForTx(){
	if((hcan1.State == HAL_CAN_STATE_READY) || (hcan1.State == HAL_CAN_STATE_BUSY_RX)){
		return 1;
	}
	return 0;
}


int Can_sendStd(uint16_t id, uint8_t isRemote, uint8_t *data, uint8_t dlc){
	if(Can_availableForTx()){
		txFrameBuf.IDE = CAN_ID_STD;
		txFrameBuf.RTR = isRemote ? CAN_RTR_REMOTE : CAN_RTR_DATA;
		txFrameBuf.StdId = id;
		txFrameBuf.DLC = dlc;
		for(int i=0; i<dlc; i++){
			txFrameBuf.Data[i] = data[i];
		}
		hcan1.pTxMsg = &txFrameBuf;
		if(HAL_CAN_Transmit_IT(&hcan1) == HAL_OK) return 0;
	}
	return -1;
}


int Can_sendExt(uint32_t id, uint8_t isRemote, uint8_t *data, uint8_t dlc){
	if(Can_availableForTx()){
		txFrameBuf.IDE = CAN_ID_EXT;
		txFrameBuf.RTR = isRemote ? CAN_RTR_REMOTE : CAN_RTR_DATA;
		txFrameBuf.ExtId = id;
		txFrameBuf.DLC = dlc;
		for(int i=0; i<dlc; i++){
			txFrameBuf.Data[i] = data[i];
		}
		hcan1.pTxMsg = &txFrameBuf;
		if(HAL_CAN_Transmit_IT(&hcan1) == HAL_OK) return 0;
	}
	return -1;
}

int Can_resend(){
	if(Can_availableForTx()){
		if(HAL_CAN_Transmit_IT(&hcan1) == HAL_OK) return 0;
	}
	return -1;
}

void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan){
	Can_txcb();
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){
	Can_frame_t *newFrame = Can_headAddr();
	newFrame->isExt = (hcan->pRxMsg->IDE) ? 1 : 0;
	newFrame->core.id = (hcan->pRxMsg->IDE) ? hcan->pRxMsg->ExtId : hcan->pRxMsg->StdId;
	newFrame->isRemote = (hcan->pRxMsg->RTR) ? 1 : 0;
	newFrame->core.dlc = hcan->pRxMsg->DLC;
	if(newFrame->isRemote == 0){
		for(int i=0; i<newFrame->core.dlc; i++){
			newFrame->core.Data[i] = hcan->pRxMsg->Data[i];
		}
	}
	newFrame->filterNum = Can_getFilterNum(hcan->pRxMsg->FMI);
	Can_head++;
	if(Can_head == CAN_BUFFER_LENGTH){
		Can_head = 0;
		if(Can_ovf < 3) Can_ovf++;
	}
	HAL_CAN_Receive_IT(&hcan1,0);
	Can_rxcb();
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan){
	Can_ercb();
}

void Can_setTxCallback(void(*pt)()){
	Can_txcb = pt;
}

void Can_setRxCallback(void(*pt)()){
	Can_rxcb = pt;
}

void Can_setErrCallback(void(*pt)()){
	Can_ercb = pt;
}
