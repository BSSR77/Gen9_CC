/*
 * MCP2515.c
 *
 *  Created on: Dec 28, 2016
 *      Author: jamesliu
 */

#include "MCP2515.h"

//ICOD (Interrupt COnDition) bits
#define	ICOD_CLEARED	0b000
#define	ICOD_ERROR		0b001
#define	ICOD_WAKE		0b010
#define	ICOD_TX0		0b011
#define	ICOD_TX1		0b100
#define	ICOD_TX2		0b101
#define	ICOD_RX0		0b110
#define	ICOD_RX1		0b111

//INTF (INTerrupt Flag) bit positions
#define	INTF_RX0	0
#define	INTF_RX1	1
#define	INTF_TX0	2
#define	INTF_TX1	3
#define	INTF_TX2	4
#define	INTF_ERR	5
#define	INTF_WAK	6
#define	INTF_MERR	7

//RX and TX buf registers
#define	BUF_CTRL	0
#define	BUF_SIDH	1
#define	BUF_SIDL	2
#define	BUF_EID8	3
#define	BUF_EID0	4
#define	BUF_DLC		5
#define	BUF_D0		6
#define	BUF_D1		7
#define	BUF_D2		8
#define	BUF_D3		9
#define	BUF_D4		10
#define	BUF_D5		11
#define	BUF_D6		12
#define	BUF_D7		13
#define	BUF_CANSTAT	14
#define	BUF_CANCTRL	15

//Misc numbers I'll definitely forget if I don't define
#define TXB0		0x30
#define TXB1		0x40
#define TXB2		0x50
#define RXB0		0x60
#define RXB1		0x70
#define INTE		0b00101011
#define RXF0		0x00
#define RXF1		0x04
#define RXF2		0x08
#define RXF3		0x10
#define RXF4		0x14
#define RXF5		0x18
#define RXM0		0x20
#define RXM1		0x24
#define BUKT		2
#define TXREQ 		3

static SPI_HandleTypeDef *hspi;
static GPIO_TypeDef *csport;
static uint16_t cspin;

static uint8_t spiTxBuf[18]; //these are enough to hold one whole buffer, plus a command and an address.
static uint8_t spiRxBuf[18];
static uint8_t CANINTF; //register mirror
static uint8_t txstat; //for each bit, 0 = available, 1 = occupied
static uint8_t not_in_use; //if the library isn't currently handling volatile data
static uint8_t int_trigd; //set by the INT pin
static uint8_t ext_act; //rtos called a blocking function

static osMessageQId osTxQueue, osRxQueue; //rtos queues so I don't have to write my own

static void (*spi_cb)();
static int cbparam;

static int spi_notBusy();
static void spiTransaction(uint8_t length);
static void spi_read(uint8_t addr, uint8_t length);
static void spi_write(uint8_t addr, uint8_t length);
static void readINTF_cb();
static void readINTF();
static void readRxBuf_cb();
static void spi_readRxBuf(uint8_t n, uint8_t m);
static void readRxBuf(uint8_t bufnum);
static void spi_loadTxBuf(uint8_t a, uint8_t b, uint8_t c);
static void requestToSend(uint8_t a);
static void sendFrame_cb();
static void spi_bitModify(uint8_t addr, uint8_t mask, uint8_t data);
static void clearINTF(uint8_t bit);
static void processINTF();
static void main_loop();
static void main_loop_isr();
static void enableInterrupts(uint8_t interrupts);
static void init_cb();
static void spi_reset();
static void empty();
static void setOverflow(uint8_t enable);
static void setFilter(uint8_t num, uint8_t id, uint8_t ide);

//Here starts the actual functions//

static int spi_notBusy(){
	if(hspi->State == HAL_SPI_STATE_READY){
		return 1;
	}else{
		return 0;
	}
}

static void spiTransaction(uint8_t length){
	HAL_GPIO_WritePin(csport, cspin, GPIO_PIN_SET);
	HAL_SPI_TransmitReceive_DMA(hspi, spiTxBuf, spiRxBuf, length);
	HAL_GPIO_WritePin(csport, cspin, GPIO_PIN_RESET);
}

static void spi_read(uint8_t addr, uint8_t length){ //read instruction
	not_in_use = 0;
	spiTxBuf[0] = 0b00000011;
	spiTxBuf[1] = addr;
	spiTransaction(length+2);
}

static void spi_write(uint8_t addr, uint8_t length){ //read instruction
	not_in_use = 0;
	spiTxBuf[0] = 0b00000010;
	spiTxBuf[1] = addr;
	spiTransaction(length+2);
}

static void readINTF_cb(){
	CANINTF = spiRxBuf[2];
	processINTF();
	not_in_use = 1;
}

static void readINTF(){
	not_in_use = 0;
	spi_cb = readINTF_cb;
	spi_read(0b00101100, 1);
}

static void readRxBuf_cb(){
	Can_frame_core_t newCore;
	if(spiRxBuf[BUF_SIDL] & (1 << 3)){ //Extended id
		newCore.id = spiRxBuf[BUF_SIDH] << 21;
		newCore.id |= (spiRxBuf[BUF_SIDL] & 0b11100000) << 13;
		newCore.id |= (spiRxBuf[BUF_SIDL] & 0b00000011) << 16;
		newCore.id |= spiRxBuf[BUF_EID8] << 8;
		newCore.id |= spiRxBuf[BUF_EID0] << 0;
	}else{ //standard id
		newCore.id = spiRxBuf[BUF_SIDH] << 3;
		newCore.id |= (spiRxBuf[BUF_SIDL] & 0b11100000) >> 5;
	}
	newCore.dlc = spiRxBuf[BUF_DLC] & 0b00001111;
	for(int i=0; i<newCore.dlc; i++){
		newCore.Data[i] = spiRxBuf[BUF_D0+i];
	}
	static BaseType_t pd;
	xQueueSendFromISR(osRxQueue, &newCore, &pd);
	not_in_use = 1;
}

static void spi_readRxBuf(uint8_t n, uint8_t m){ //readRxBuf instruction
	not_in_use = 0;
	n = n?1:0;
	m = m?1:0;
	spiTxBuf[0] = 0b10010000;
	spiTxBuf[0] |= n<<2;
	spiTxBuf[0] |= m<<1;
	spiTransaction(15);
}

static void readRxBuf(uint8_t bufnum){ //reads a whole buffer
	not_in_use = 0;
	bufnum = bufnum?1:0;
	spi_cb = readRxBuf_cb;
	spi_readRxBuf(bufnum, 0);
}

static void processINTF(){
	if(CANINTF & (1<<INTF_RX0)){
		readRxBuf(0); //automagically clears rxbuf interrupt
	}else if(CANINTF & (1<<INTF_RX1)){
		readRxBuf(1); //automagically clears rxbuf interrupt
	}else if(CANINTF & (1<<INTF_TX0)){
		txstat &= ~(1);
		clearINTF(INTF_TX0);
	}else if(CANINTF & (1<<INTF_TX1)){
		txstat &= ~(1<<1);
		clearINTF(INTF_TX1);
	}else if(CANINTF & (1<<INTF_TX2)){
		txstat &= ~(1<<2);
		clearINTF(INTF_TX2);
	}else if(CANINTF & (1<<INTF_ERR)){
		clearINTF(INTF_ERR); //WIPWIPWIP /*TODO*/
	}else if(CANINTF & (1<<INTF_WAK)){
		clearINTF(INTF_WAK); //WIPWIPWIP
	}else if(CANINTF & (1<<INTF_MERR)){
		clearINTF(INTF_MERR); //WIPWIPWIP
	}else{
		int_trigd = 0;
	}
}

static void spi_loadTxBuf(uint8_t a, uint8_t b, uint8_t c){
	not_in_use = 0;
	spiTxBuf[0] = 0b01000000;
	spiTxBuf[0] |= (a<<2) | (b<<1) | c;
	spiTransaction(14);
}

static void requestToSend(uint8_t a){
	not_in_use = 0;
	spi_cb = empty;
	cbparam = 0;
	spi_bitModify((a*0x10)+0x30, 0b00001000, 0b00001000);
}

static void sendFrame_cb(){
	requestToSend((uint8_t)cbparam);
}

static void sendFrame(Can_frame_t *newFrame){
	not_in_use = 0;
	spiTxBuf[BUF_SIDL] = (newFrame->isExt ? 1 : 0) << 3;
	spiTxBuf[BUF_DLC] = (newFrame->isRemote ? 1 : 0) << 6;
	spiTxBuf[BUF_DLC] |= newFrame->core.dlc & 0x0f;
	if(newFrame->isExt){
		spiTxBuf[BUF_SIDH] = (newFrame->core.id >> 21) & 0xff;
		spiTxBuf[BUF_SIDL] |= (newFrame->core.id >> 13) & 0b11100000;
		spiTxBuf[BUF_SIDL] |= (newFrame->core.id >> 16) & 0b00000011;
		spiTxBuf[BUF_EID8] = (newFrame->core.id >> 8) & 0xff;
		spiTxBuf[BUF_EID0] = newFrame->core.id & 0xff;
	}else{
		spiTxBuf[BUF_SIDH] = (newFrame->core.id >> 3) & 0xff;
		spiTxBuf[BUF_SIDL] |= newFrame->core.id << 5;
	}
	if(newFrame->isRemote == 0){
		for(int i=0; i<newFrame->core.dlc; i++){
			spiTxBuf[BUF_D0+i] = newFrame->core.Data[i];
		}
	}
	spi_cb = sendFrame_cb;
	if(txstat & 0x01){       //buf0 free
		spi_loadTxBuf(0,0,0);
		txstat |= 0x01;
		cbparam = 0;
	}else if(txstat & 0x02){ //buf1 free
		spi_loadTxBuf(0,1,0);
		txstat |= 0x02;
		cbparam = 1;
	}else if(txstat & 0x04){ //buf2 free
		spi_loadTxBuf(1,0,0);
		txstat |= 0x04;
		cbparam = 2;
	}
}

static void spi_bitModify(uint8_t addr, uint8_t mask, uint8_t data){
	not_in_use = 0;
	spiTxBuf[0] = 0b00000101;
	spiTxBuf[1] = addr;
	spiTxBuf[2] = mask;
	spiTxBuf[3] = data;
	spiTransaction(4);
}

static void clearINTF(uint8_t bit){
	not_in_use = 0;
	spi_cb = empty;
	spi_bitModify(0b00101100, 1<<bit, 0);
}

static void main_loop(){
	if(spi_notBusy() && not_in_use){
		if(int_trigd){
			readINTF();
		}else if(uxQueueMessagesWaiting(osTxQueue) && (txstat > 7)){
			static Can_frame_t newFrame;
			xQueueReceive(osTxQueue, &newFrame, 0);
			sendFrame(&newFrame);
		}
	}
}

static void main_loop_isr(){
	if(spi_notBusy() && not_in_use){
		if(ext_act) return;
		if(int_trigd){
			readINTF();
			processINTF();
		}else if(uxQueueMessagesWaitingFromISR(osTxQueue) && (txstat > 7)){
			static Can_frame_t newFrame;
			static BaseType_t woke;
			xQueueReceiveFromISR(osTxQueue, &newFrame, &woke);
			sendFrame(&newFrame);
		}
	}
}

static void enableInterrupts(uint8_t interrupts){
	not_in_use = 0;
	spiRxBuf[2] = interrupts;
	spi_bitModify(INTE, 0xff, interrupts);
	spi_cb = empty;
}

static void init_cb(){
	not_in_use = 0;
	switch(cbparam){ //steps to reset
	case 1:
		setOverflow(1); break; //enable overflow
	case 2:
		enableInterrupts(0b00011111); break; //enable rx and tx interrupts
	case 3:
		setFilter(0, 0, 0); break;
	case 4:
		setFilter(1, 0, 1); break;
	case 5:
		setFilter(6, 0, 0); break; //set mask: all pass-thru
	//case 6:
		/*TODO set operation modes*/
	default:
		not_in_use = 1;
		cbparam = 0;
		return;
	}
	spi_cb = init_cb;
	cbparam++;
}

static void spi_reset(){
	not_in_use = 0;
	spiTxBuf[0] = 0b11000000;
	spi_cb = empty;
	spiTransaction(1);
}

static void empty(){
	not_in_use = 1;
}

static void setOverflow(uint8_t enable){
	not_in_use = 0;
	enable = enable?1:0;
	spi_cb = empty;
	spi_bitModify(RXB0+BUF_CTRL, 1<<BUKT, 1<<BUKT);
}

static void setFilter(uint8_t num, uint8_t id, uint8_t ide){
	/*TODO buffer mode auto management*/
	not_in_use = 0;
	if(ide){
		spiTxBuf[BUF_SIDL+1] = 1 << 3;
		spiTxBuf[BUF_SIDH+1] = (id >> 21) & 0xff;
		spiTxBuf[BUF_SIDL+1] |= (id >> 13) & 0b11100000;
		spiTxBuf[BUF_SIDL+1] |= (id >> 16) & 0b00000011;
		spiTxBuf[BUF_EID8+1] = (id >> 8) & 0xff;
		spiTxBuf[BUF_EID0+1] = id & 0xff;
	}else{
		spiTxBuf[BUF_SIDH+1] = (id >> 3) & 0xff;
		spiTxBuf[BUF_SIDL+1] = id << 5;
	}
	switch(num){
	spi_cb = empty;
	case 0:
		spi_write(RXF0, 4); break;
	case 1:
		spi_write(RXF1, 4); break;
	case 2:
		spi_write(RXF2, 4); break;
	case 3:
		spi_write(RXF3, 4); break;
	case 4:
		spi_write(RXF4, 4); break;
	case 5:
		spi_write(RXF5, 4); break;
	case 6:
		spi_write(RXM0, 4); break;
	case 7:
		spi_write(RXM1, 4); break;
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi){
	spi_cb();
	main_loop_isr();
}

void MCP2515_EXTICallback(){
	int_trigd = 1;
	main_loop_isr();
}

void MCP2515_write(Can_frame_t *frame){
	xQueueSend(osTxQueue, frame, portMAX_DELAY);
	main_loop();
}

void MCP2515_init(SPI_HandleTypeDef *newhspi, osMessageQId tx, osMessageQId rx, GPIO_TypeDef *csPort, uint16_t csPin){
	not_in_use = 0;
	hspi = newhspi;
	osTxQueue = tx;
	osRxQueue = rx;
	csport = csPort;
	cspin = csPin;
	MCP2515_reset();
	cbparam = 1;
	spi_cb = init_cb;
}

void MCP2515_reset(){
	spi_cb = spi_reset;
	if(spi_notBusy() && not_in_use){
		spi_reset();
	}
}
