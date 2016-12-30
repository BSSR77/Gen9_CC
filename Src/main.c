/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "serial.h"
#include "can.h"
//#include "MCP2515.h"
#include "../../CAN_ID.h"	// Common CAN ID header 2 directories up from here
#include "firmwareTable.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_tx;
DMA_HandleTypeDef hdma_spi1_rx;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

WWDG_HandleTypeDef hwwdg;

osThreadId RT_HandlerHandle;
osThreadId mainCan_TxHandle;
osThreadId Can_ProcessorHandle;
osThreadId Node_ManagerHandle;
osThreadId motCan_TxHandle;
osThreadId kickWatchdogHandle;
osMessageQId mainCanTxBufHandle;
osMessageQId mainCanRxBufHandle;
osMessageQId motCanTxBufHandle;
osMessageQId badNodesHandle;
osTimerId HBTmrHandle;
osMutexId ctrlVarMtxHandle;
osSemaphoreId spiRxDirtyHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
osMutexId nodeEntryMtxHandle[MAX_NODE_NUM];		// Mutex for every node table entry
osTimerId nodeTmrHandle[MAX_NODE_NUM];			// Timer for each node's timeout timer
nodeEntry nodeTable[MAX_NODE_NUM];
controlVars userInput;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_WWDG_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
void doRealTime(void const * argument);
void doCanTx(void const * argument);
void doProcessCan(void const * argument);
void doNodeManager(void const * argument);
void doMotCanTx(void const * argument);
void doKickDog(void const * argument);
void TmrSendHB(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	switch(GPIO_Pin){
	case MCP_Int_Pin:
		//MCP2515_EXTICallback();
		break;
	}
}

static inline void shutdownMotor(){
	static Can_frame_t offFrame;
	// Switch positions to indicate motor neutral
	offFrame.isExt = 0;
	offFrame.isRemote = 0;
	offFrame.core.id = mc_P2P;
	offFrame.core.dlc = CMD_DLC;
	offFrame.core.Data[0] = NODE_SHUTDOWN;
	while(Can_availableForTx() == 0){	// Wait if bxCAN module is still busy
	  osDelay(motCanTxInterval);
	}
	Can_sendStd(offFrame.core.id,offFrame.isRemote,offFrame.core.Data,offFrame.core.dlc);
}

static inline void shutdown_routine(){
	// Don't care about locking the statusWord here since we are in Critical Area
	nodeTable[cc_nodeID].nodeStatusWord &= 0xfffffff8;	// Clear the node status field
	nodeTable[cc_nodeID].nodeStatusWord |= SHUTDOWN;		// Set status to shutdown

	// Put motor into SHUTDOWN state
	shutdownMotor();

	// Broadcast CC shutdown state to main CAN
	static Can_frame_t newFrame;
	newFrame.core.id = radio_SW;
	newFrame.core.dlc = CAN_HB_DLC;
	for(int i=0; i<4; i++){
		newFrame.core.Data[3-i] = (nodeTable[cc_nodeID].nodeStatusWord >> (8*i)) & 0xff;			// Convert uint32_t -> uint8_t
	}
	xQueueSendFromISR(mainCanTxBufHandle, &newFrame, pdFALSE);
	HAL_Delay(1);	// 1ms delay to ensure the queue is properly updated before proceeding

	// Loop through Main CAN Tx buffer to send any pending messages
	while(uxQueueMessagesWaitingFromISR(mainCanTxBufHandle) != 0){
		while(Can_availableForTx() == 0){	// Wait if bxCAN module is still busy
			HAL_WWDG_Refresh(&hwwdg);		// Since we are in the
			HAL_Delay(1);					// Hard delay to flush CAN tx buffers
		}
		xQueueReceiveFromISR(mainCanTxBufHandle, &newFrame, pdFALSE);
		// TODO: Send main CAN message buffers
	}
}

void node_hreset(){
#ifdef DEBUG
	static uint8_t msg[] = "Node hard reset issued\n";
	Serial2_writeBytes(msg, sizeof(msg)-1);
#endif
	NVIC_SystemReset();					// CMSIS System reset function
}

void node_reset(){
#ifdef DEBUG
	static uint8_t msg[] = "Node reset issued\n";
	Serial2_writeBytes(msg, sizeof(msg)-1);
#endif
	shutdown_routine();
	NVIC_SystemReset();					// CMSIS System reset function
}

void node_shutdown(){
	shutdown_routine();	// in shutdown state, CC will continue MC heartbeat
	// Shutdown all active timers
	for(uint8_t i = 0 ; i < MAX_NODE_NUM;i++){
		xTimerStop(nodeTmrHandle[i], portMAX_DELAY);
	}
}

void node_start(){
	// first send to all main nodes RESET command
	static Can_frame_t resetCmd;
	resetCmd.isExt = 0;
	resetCmd.isRemote = 0;
	resetCmd.core.dlc = CMD_DLC;
	for(uint8_t i = 0 ; i < MAX_NODE_NUM;i++){
		if(nodeTable[i].nodeFirmwareVersion != SW_Sentinel){
			resetCmd.core.id = i + p2pOffset;
			resetCmd.core.Data[0] = NODE_RESET;
			xQueueSend(mainCanTxBufHandle, &resetCmd, portMAX_DELAY);
		}
	}
	// Reset the badNodes queue since a fresh reset has been issued
	xQueueReset(badNodesHandle);
}

void executeCommand(nodeCommands cmd){
	taskENTER_CRITICAL();
	switch(cmd){
	case(NODE_HRESET):
		node_hreset();
		break;
	case(NODE_RESET):
		node_reset();
		break;

	case(NODE_SHUTDOWN):
		node_shutdown();
		break;

	case(NODE_START):
		node_start();
		break;

	default:
		break;
	}
	taskEXIT_CRITICAL();
}

// Handler for node HB timeout
void TmrHBTimeout(void const * argument){
 	uint8_t timerID = (uint8_t)pvTimerGetTimerID((TimerHandle_t)argument);
#ifdef DEBUG
 	Serial2_write(timerID);
 	static uint8_t msg[] = "Timer Triggered\n";
	Serial2_writeBytes(msg,sizeof(msg)-1);

#endif
	nodeTable[timerID].nodeConnectionState = UNRELIABLE;
	if((timerID) != mc_nodeID){
		xQueueSend(badNodesHandle, &timerID, portMAX_DELAY);
	}
}

void setupNodeTable(){
	for(uint8_t i = 0; i < MAX_NODE_NUM; i++){
		nodeTable[i].nodeStatusWord = SW_Sentinel;			// Initialize status word to SENTINEL
		nodeTable[i].nodeFirmwareVersion = SW_Sentinel;		// Initialize firm ware version to SENTINEL
	}

	#ifdef cc_nodeID
		nodeTable[cc_nodeID].nodeConnectionState = DISCONNECTED;
		nodeTable[cc_nodeID].nodeFirmwareVersion = cc_VERSION;
	#endif

	#ifdef mc_nodeID
		nodeTable[mc_nodeID].nodeConnectionState = DISCONNECTED;
		nodeTable[mc_nodeID].nodeFirmwareVersion = mc_VERSION;
	#endif

	#ifdef bps_nodeID
		nodeTable[bps_nodeID].nodeConnectionState = DISCONNECTED;
		nodeTable[bps_nodeID].nodeFirmwareVersion = bps_VERSION;
	#endif

	#ifdef ads_nodeID
		nodeTable[ads_nodeID].nodeConnectionState = DISCONNECTED;
		nodeTable[ads_nodeID].nodeFirmwareVersion = ads_VERSION;
	#endif

	#ifdef radio_nodeID
		nodeTable[radio_nodeID].nodeConnectionState = DISCONNECTED;
		nodeTable[radio_nodeID].nodeFirmwareVersion = radio_VERSION;
	#endif
}

/*
 * Main Can Receive Callback
 */
void mainCanRxCallback(){
	static Can_frame_core_t newCore;

	// TODO: Load newCore with received frame

	// Check if it's anything critical
	taskENTER_CRITICAL();
	switch(newCore.id){
	case (SysEMSD):
		// Ensure motor is shutdown, CC remains ACTIVE for diagnostics
		shutdownMotor();
		break;

	case (UsrEMSD):
		// Ensure motor is shutdown, CC remains ACTIVE for diagnostics
		shutdownMotor();
		break;

	case (p2pOffset):
		// CAN ID is a P2P broadcast -> command
		executeCommand((nodeCommands)newCore.Data);	// Byte length command, no need to put into uint32_t form
		break;

	case (cc_P2P):
		// CAN ID is a P2P ID -> command
		executeCommand((nodeCommands)newCore.Data);	// Byte length command, no need to put into uint32_t form
		break;
	}
	taskEXIT_CRITICAL();
	xQueueSend(mainCanRxBufHandle, &newCore, portMAX_DELAY);	// Send the new data to the mainCanRxBuf queue
}

/*
 * Motor CAN Receive callback
 * Currently implemented to dump all incoming frame cores into the mainCAN
 */
void motCanRxCallback(){
	static Can_frame_core_t newCore;

	// Parse the CAN frame
	// Map motor CAN EXT frame to main CAN Std frame
	newCore.id = (hcan1.pRxMsg->IDE) ? hcan1.pRxMsg->ExtId : hcan1.pRxMsg->StdId;
	switch(newCore.id){
	case mitsubaFr0:
		newCore.id = mcDiag0;
		break;

	case mitsubaFr1:
		newCore.id = mcDiag1;
		break;

	case mitsubaFr2:
		newCore.id = mcDiag2;
		break;
	}

	newCore.dlc = hcan1.pRxMsg->DLC;
	if(hcan1.pRxMsg->RTR == 0){
		for(int i=0; i<newCore.dlc; i++){
			newCore.Data[i] = hcan1.pRxMsg->Data[i];
		}
	}

	// XXX: Any data to additional application layer tasks should be buffered with Queues
	xQueueSend(mainCanTxBufHandle, &newCore, portMAX_DELAY);	// Push the data onto main CAN for radio
}

/*
 * Rest node given nodeID, the attempt number, and the last tick count for delay Until
 * Will ultimately destroy the task running it
 */
void resetNode(resetParams * passed){
	if(passed->attempts >= 0){
		static Can_frame_t newFrame;
		newFrame.isExt = 0;
		newFrame.isRemote = 0;
		newFrame.core.id = passed->nodeID + p2pOffset;
		newFrame.core.dlc = CMD_DLC;
		if(passed->attempts > 0){
			newFrame.core.Data[0] = NODE_RESET;	// First attempt - soft reset
		}
		else{
			newFrame.core.Data[0] = NODE_HRESET;	// Second attempt and beyond - hard reset
		}
		xQueueSend(mainCanTxBufHandle, &newFrame, portMAX_DELAY);
		osDelayUntil(&(passed->ticks), HB_Interval);
		if((nodeTable[passed->nodeID].nodeConnectionState & 0x07) == UNRELIABLE){
			passed->attempts = passed->attempts+1;
			resetNode(passed);	// Retry
		}
		else{
			// Recovery successful
#ifdef DEBUG
	Serial2_write(passed->nodeID);
	static uint8_t hbmsg[] = "Node reset success... \n";
	Serial2_writeBytes(hbmsg, sizeof(hbmsg)-1);
#endif
			vTaskDelete(NULL);
		}
	}
	else if(passed->attempts > MAX_RESET_ATTEMPTS){
		// Recovery failed, set node to Hard Error!
		xSemaphoreTake(nodeEntryMtxHandle[passed->nodeID], portMAX_DELAY);
		nodeTable[passed->nodeID].nodeConnectionState = HARD_ERROR;
		xSemaphoreGive(nodeEntryMtxHandle[passed->nodeID]);
#ifdef DEBUG
	Serial2_write(passed->nodeID);
	static uint8_t hbmsg[] = "Node reset failure... \n";
	Serial2_writeBytes(hbmsg, sizeof(hbmsg)-1);
#endif
		vTaskDelete(NULL);
	} else {
		passed->attempts = passed->attempts+1;
		resetNode(passed);	// Retry
	}
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_WWDG_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
#ifdef DEBUG
	static uint8_t hbmsg[] = "Command Center booting... \n";
	Serial2_writeBytes(hbmsg, sizeof(hbmsg)-1);
#endif
  Can_begin();
  Can_setRxCallback(motCanRxCallback);
  setupNodeTable();
  nodeTable[cc_nodeID].nodeStatusWord |= ACTIVE;		// Set initial status to ACTIVE

  // Set up CAN filter banks
  Can_addMaskedFilterStd(swOffset,0xFF0,0); // Filter: Status word group (ignore nodeID)
  Can_addMaskedFilterStd(fwOffset,0xFF0,0); // Filter: Firmware version group (ignore nodeID)

  Can_addMaskedFilterExt(mitsubaFr0,0xFFFFFF0F,0);	// Mitsuba Frame 0
  Can_addMaskedFilterExt(mitsubaFr1,0xFFFFFF0F,0);	// Mitsuba Frame 1
  Can_addMaskedFilterExt(mitsubaFr2,0xFFFFFF0F,0);	// Mitsuba Frame 2
  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of ctrlVarMtx */
  osMutexDef(ctrlVarMtx);
  ctrlVarMtxHandle = osMutexCreate(osMutex(ctrlVarMtx));

  /* USER CODE BEGIN RTOS_MUTEX */
  	// Node table entry mutex
	// Every entry has a mutex that is associated with the nodeID
	for(uint8_t i =0; i < MAX_NODE_NUM; i++){
	  osMutexDef(i);
	  nodeEntryMtxHandle[i] = osMutexCreate(osMutex(i));
	}
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of spiRxDirty */
  osSemaphoreDef(spiRxDirty);
  spiRxDirtyHandle = osSemaphoreCreate(osSemaphore(spiRxDirty), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of HBTmr */
  osTimerDef(HBTmr, TmrSendHB);
  HBTmrHandle = osTimerCreate(osTimer(HBTmr), osTimerPeriodic, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  // Node heartbeat timeout timers
  for(uint8_t TmrID = 0; TmrID < MAX_NODE_NUM; TmrID++){
	  osTimerDef(TmrID, TmrHBTimeout);
	  nodeTmrHandle[TmrID] = osTimerCreate(osTimer(TmrID), osTimerOnce, TmrID);	// TmrID here is stored directly as a variable
	  // One-shot timer since it should be refreshed by the Can Processor upon node HB reception
  }


  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of RT_Handler */
  osThreadDef(RT_Handler, doRealTime, osPriorityHigh, 0, 512);
  RT_HandlerHandle = osThreadCreate(osThread(RT_Handler), NULL);

  /* definition and creation of mainCan_Tx */
  osThreadDef(mainCan_Tx, doCanTx, osPriorityNormal, 0, 512);
  mainCan_TxHandle = osThreadCreate(osThread(mainCan_Tx), NULL);

  /* definition and creation of Can_Processor */
  osThreadDef(Can_Processor, doProcessCan, osPriorityBelowNormal, 0, 512);
  Can_ProcessorHandle = osThreadCreate(osThread(Can_Processor), NULL);

  /* definition and creation of Node_Manager */
  osThreadDef(Node_Manager, doNodeManager, osPriorityLow, 0, 512);
  Node_ManagerHandle = osThreadCreate(osThread(Node_Manager), NULL);

  /* definition and creation of motCan_Tx */
  osThreadDef(motCan_Tx, doMotCanTx, osPriorityHigh, 0, 512);
  motCan_TxHandle = osThreadCreate(osThread(motCan_Tx), NULL);

  /* definition and creation of kickWatchdog */
  osThreadDef(kickWatchdog, doKickDog, osPriorityRealtime, 0, 256);
  kickWatchdogHandle = osThreadCreate(osThread(kickWatchdog), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of mainCanTxBuf */
  osMessageQDef(mainCanTxBuf, 8, Can_frame_t);
  mainCanTxBufHandle = osMessageCreate(osMessageQ(mainCanTxBuf), NULL);

  /* definition and creation of mainCanRxBuf */
  osMessageQDef(mainCanRxBuf, 8, Can_frame_core_t);
  mainCanRxBufHandle = osMessageCreate(osMessageQ(mainCanRxBuf), NULL);

  /* definition and creation of motCanTxBuf */
  osMessageQDef(motCanTxBuf, 8, Can_frame_t);
  motCanTxBufHandle = osMessageCreate(osMessageQ(motCanTxBuf), NULL);

  /* definition and creation of badNodes */
  osMessageQDef(badNodes, 16, uint8_t);
  badNodesHandle = osMessageCreate(osMessageQ(badNodes), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the main internal regulator output voltage 
    */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.NbrOfDiscConversion = 1;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_16;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* CAN1 init function */
static void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 10;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_3TQ;
  hcan1.Init.BS1 = CAN_BS1_12TQ;
  hcan1.Init.BS2 = CAN_BS2_3TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = ENABLE;
  hcan1.Init.AWUM = ENABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* WWDG init function */
static void MX_WWDG_Init(void)
{

  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_8;
  hwwdg.Init.Window = 127;
  hwwdg.Init.Counter = 127;
  hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    Error_Handler();
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : MCP_Int_Pin */
  GPIO_InitStruct.Pin = MCP_Int_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(MCP_Int_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Driver_D0_Pin */
  GPIO_InitStruct.Pin = Driver_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Driver_D0_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* doRealTime function */
void doRealTime(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop
   * Take GPIO and ADC readings
   * Put into userInputs struct
   * Push variables to mainCanTxBuf
   * Push variables to motCanTxBuf
   * */
  uint32_t PreviousTickTime = osKernelSysTick();
  uint16_t delayTickMarker = osKernelSysTick();	// Counts how many ticks elapsed at least before sending message to main CAN

  for(;;)
  {
	// ALL CRITICAL MOTOR CONTROLS SHOULD BE TOGGLE SWITCHES; NO PUSH BUTTONS!
	xSemaphoreTake(ctrlVarMtxHandle,portMAX_DELAY);
	// TODO: Read ADC values, and update userInputs struct
	xSemaphoreGive(ctrlVarMtxHandle);

	uint32_t currentTickTime = osKernelSysTick();
	// Assemble Switch Position frame
	static Can_frame_t newFrame;
	newFrame.isExt = 0;
	newFrame.isRemote = 0;
	newFrame.core.id = swPos;
	newFrame.core.dlc = swPos_DLC;
	// TODO: assemble switch position data

	if(currentTickTime - delayTickMarker >= ctrlVarTxInterval){
		// TODO: Send to mainCAN
	}

	xQueueSend(motCanTxBufHandle, &newFrame, 0);

	// Assemble Brake Position frame
	newFrame.core.id = brakePos;
	newFrame.core.dlc = brakePos_DLC;
	for(int i=0; i<4; i++){
		newFrame.core.Data[3-i] = ((uint32_t)(userInput.brakePosition) >> (8*i)) & 0xff;			// Convert uint32_t -> uint8_t
	}

	// TODO: Send to mainCAN
	xQueueSend(motCanTxBufHandle, &newFrame, 0);

	// Assemble Accelerator Position frame
	newFrame.core.id = accelPos;
	newFrame.core.dlc = accelPos_DLC;
	for(int i=0; i<4; i++){
		newFrame.core.Data[3-i] = ((uint32_t)(userInput.accelPosition) >> (8*i)) & 0xff;			// Convert uint32_t -> uint8_t
	}

	if(currentTickTime - delayTickMarker >= ctrlVarTxInterval){
		// TODO: Send to mainCAN
	}
	xQueueSend(motCanTxBufHandle, &newFrame, 0);

	// Assemble Regen Position frame
	newFrame.core.id = regenPos;
	newFrame.core.dlc = regenPos_DLC;
	for(int i=0; i<4; i++){
		newFrame.core.Data[3-i] = ((uint32_t)(userInput.regenPosition) >> (8*i)) & 0xff;			// Convert uint32_t -> uint8_t
	}

	if(currentTickTime - delayTickMarker >= ctrlVarTxInterval){
		// TODO: Send to mainCAN
		delayTickMarker = osKernelSysTick();		// Only reset at the very end to ensure that all 3 variables are sent synchronously
	}
	xQueueSend(motCanTxBufHandle, &newFrame, 0);

	// Periodic function
	osDelayUntil(&PreviousTickTime, RT_Interval);
  }
  /* USER CODE END 5 */ 
}

/* doCanTx function
 * Main CAN transmission function via MCP2515
 * */
void doCanTx(void const * argument)
{
  /* USER CODE BEGIN doCanTx */
  /* Infinite loop */
  for(;;)
  {
	  static Can_frame_t newFrame;
	  xQueueReceive(mainCanTxBufHandle, &newFrame, portMAX_DELAY);	// Only process CAN Transmissions when the queue is non-empty

	  if (((nodeTable[cc_nodeID].nodeStatusWord & 0x07) == ACTIVE)){
		  // Only attempt transmission if CC is in ACTIVE mode
		  while(0/* TODO: mainCAN available */){	// Wait if MCP2515 module is still busy
			  osDelay(mainCanTxInterval);
		  }
		  // TODO: mainCAN send
	  }
  }
  /* USER CODE END doCanTx */
}

/* doProcessCan function */
void doProcessCan(void const * argument)
{
  /* USER CODE BEGIN doProcessCan */
  /* Infinite loop */
  for(;;)
  {
	static Can_frame_t newFrame;
	xQueueReceive(mainCanRxBufHandle, &newFrame, portMAX_DELAY);
	uint8_t canID = newFrame.core.id;	// canID container
	uint8_t nodeID = canID & 0x00F;		// nodeID container

	// CAN ID group processing
	if((nodeTable[cc_nodeID].nodeStatusWord & 0x07) == ACTIVE){
		// Only respond if node is active
		uint32_t temp_data = 0;				// Temporary data container
		for(int i=0; i<4; i++){
			temp_data |= (newFrame.core.Data[i] << (8*(3-i)));
		}

		if ((canID & 0xFF0) == swOffset){
			// Check if node is defined in nodeTable
			if(nodeTable[nodeID].nodeFirmwareVersion == SW_Sentinel)
			{
				// Node not in registered domain; Send SHUTDOWN command to node
				static Can_frame_t shutdownCMD;
				shutdownCMD.isExt = 0;
				shutdownCMD.isRemote = 0;
				shutdownCMD.core.id = nodeID + p2pOffset;
				shutdownCMD.core.dlc = CMD_DLC;
				shutdownCMD.core.Data[0] = NODE_SHUTDOWN;
				xQueueSend(mainCanTxBufHandle, &shutdownCMD, portMAX_DELAY);
				break;
			}
			// CAN ID is a status word address
			if(nodeTable[nodeID].nodeConnectionState == CONNECTED){
				xSemaphoreTake(nodeEntryMtxHandle[nodeID],portMAX_DELAY);
				nodeTable[nodeID].nodeStatusWord = temp_data;
				xSemaphoreGive(nodeEntryMtxHandle[nodeID]);
				if((temp_data & 0x07) == ACTIVE){
					xTimerReset(nodeTmrHandle[nodeID], portMAX_DELAY);	// Reset the node heartbeat count down
				}
				else if (((temp_data & 0x07) == SHUTDOWN)){
					xTimerStop(nodeTmrHandle[nodeID], portMAX_DELAY);	// Stop the heartbeat timer when the node is in SHUTDOWN state
				}
			}
			else if(nodeTable[nodeID].nodeConnectionState == CONNECTING){
				if((temp_data & 0x07) == ACTIVE){
					xSemaphoreTake(nodeEntryMtxHandle[nodeID],portMAX_DELAY);
					nodeTable[nodeID].nodeStatusWord = temp_data;
					nodeTable[nodeID].nodeConnectionState = CONNECTED;
					xSemaphoreGive(nodeEntryMtxHandle[nodeID]);

					if(nodeID == mc_nodeID){
						xTimerReset(HBTmrHandle, portMAX_DELAY);		// Start the timer for the motor heartbeat
					}
				}
				// If node not in ACTIVE state, will continue waiting until HB timeout and the node is flagged as unreliable
				xTimerReset(nodeTmrHandle[nodeID], portMAX_DELAY);	// Start the timer for the new node
			}
			else{
				xSemaphoreTake(nodeEntryMtxHandle[nodeID],portMAX_DELAY);
				nodeTable[nodeID].nodeStatusWord &= 0xFFFFFFF8;
				nodeTable[nodeID].nodeStatusWord |= UNRELIABLE;
				xSemaphoreGive(nodeEntryMtxHandle[nodeID]);
				xQueueSend(badNodesHandle, &nodeID, portMAX_DELAY);
			}
		}
		else if((canID & 0xFF0) == fwOffset){

			static Can_frame_t shutdownCMD;
			shutdownCMD.isExt = 0;
			shutdownCMD.isRemote = 0;
			shutdownCMD.core.id = nodeID + p2pOffset;
			shutdownCMD.core.dlc = CMD_DLC;

			// Check if node is in table
			if(nodeTable[nodeID].nodeFirmwareVersion == SW_Sentinel)
			{
				// Node not in registered domain; Send SHUTDOWN command to node
				shutdownCMD.core.Data[0] = NODE_SHUTDOWN;
			}
			else if(temp_data == nodeTable[nodeID].nodeFirmwareVersion){
				// Firmware version accepted
				shutdownCMD.core.Data[0] = CC_ACK;
				xSemaphoreTake(nodeEntryMtxHandle[nodeID],portMAX_DELAY);
				nodeTable[nodeID].nodeStatusWord &= 0xFFFFFFF8;
				nodeTable[nodeID].nodeStatusWord |= CONNECTING;
				xSemaphoreGive(nodeEntryMtxHandle[nodeID]);
			}
			else {
				// Incorrect firmware version
				shutdownCMD.core.Data[0] = CC_NACK;
				xSemaphoreTake(nodeEntryMtxHandle[nodeID],portMAX_DELAY);
				nodeTable[nodeID].nodeStatusWord &= 0xFFFFFFF8;
				nodeTable[nodeID].nodeStatusWord |= DISCONNECTED;
				xSemaphoreGive(nodeEntryMtxHandle[nodeID]);
			}
			xQueueSend(mainCanTxBufHandle, &shutdownCMD, portMAX_DELAY);
		}
		/*
		else {
			// Individual CAN ID Processing
			switch(canID){
				// Insert CAN IDs here
				default:
					// Ignore other incoming frames
					break;
			}
		}
		*/
	}
  }
  /* USER CODE END doProcessCan */
}

/* doNodeManager function */
void doNodeManager(void const * argument)
{
  /* USER CODE BEGIN doNodeManager */
  /* Infinite loop */
  for(;;)
  {
	uint8_t badNodeID = 0xFF;
	xQueueReceive(badNodesHandle, &badNodeID, portMAX_DELAY);
	resetParams toPass;
	toPass.nodeID = badNodeID;
	toPass.attempts = 0;
	toPass.ticks = osKernelSysTick();
	xTaskCreate(resetNode, (const char*)"", 512, (void *)(&toPass), uxTaskPriorityGet(NULL),NULL);
  }
  /* USER CODE END doNodeManager */
}

/* doMotCanTx function
 * send data to the motor CAN via STM32 bxCAN
 * */
void doMotCanTx(void const * argument)
{
  /* USER CODE BEGIN doMotCanTx */
  /* Infinite loop */
  for(;;)
  {
	  static Can_frame_t newFrame;
	  xQueueReceive(motCanTxBufHandle, &newFrame, portMAX_DELAY);	// Only process CAN Transmissions when the queue is non-empty
	  uint32_t motStatus = nodeTable[mc_nodeID].nodeStatusWord & 0x07;

	  // TODO: May have to check the tx states for motor controller
	  if (((motStatus & 0x07) == ACTIVE) || ((motStatus & 0x07) == SHUTDOWN)){
		  // Only send to motor controller if it is ACTIVE or in SHUTDOWN mode
		  while(Can_availableForTx() == 0){	// Wait if bxCAN module is still busy
			  osDelay(motCanTxInterval);
		  }

		  // Motor CAN diagnostic request is an extended frame
		  if (newFrame.isExt){
			  Can_sendExt(newFrame.core.id,newFrame.isRemote,newFrame.core.Data,newFrame.core.dlc);
		  } else{
			  Can_sendStd(newFrame.core.id,newFrame.isRemote,newFrame.core.Data,newFrame.core.dlc);
		  }
	  }
  }
  /* USER CODE END doMotCanTx */
}

/* doKickDog function */
void doKickDog(void const * argument)
{
  /* USER CODE BEGIN doKickDog */
  /* Infinite loop */
  uint32_t PreviousTickTime = osKernelSysTick();
  for(;;)
  {
	taskENTER_CRITICAL();
	HAL_WWDG_Refresh(&hwwdg);
	taskEXIT_CRITICAL();

	osDelayUntil(&PreviousTickTime, WD_Interval);
  }
  /* USER CODE END doKickDog */
}

/* TmrSendHB function */
void TmrSendHB(void const * argument)
{
  /* USER CODE BEGIN TmrSendHB */
	static Can_frame_t newFrame;
	uint32_t motStatus = nodeTable[mc_nodeID].nodeStatusWord & 0x07;

	// MC Diagnostic request
	if(motStatus == ACTIVE){
		// Only send HB to MC if MC is in ACTIVE or SHUTDOWN state
		newFrame.isExt = 1;
		newFrame.isRemote = 0;
		newFrame.core.id = mitsubaREQ;
		newFrame.core.dlc = Req_DLC;
		newFrame.core.Data[0] = Req_Frm0 | Req_Frm1 | Req_Frm2;
		// TODO: Check if the data above is in the correct form!
	#ifdef DEBUG
		static uint8_t hbmsg[] = "Motor diagnositc request issued\n";
		Serial2_writeBytes(hbmsg, sizeof(hbmsg)-1);
	#endif

		xQueueSendToFront(motCanTxBufHandle, &newFrame, 0);
	}

	// CC-MC Heartbeat
	if((motStatus == ACTIVE) || (motStatus == SHUTDOWN)){
		// Only send HB to MC if MC is in ACTIVE or SHUTDOWN state
		newFrame.isExt = 0;
		newFrame.isRemote = 0;
		newFrame.core.id = cc_SW;
		newFrame.core.dlc = CAN_HB_DLC;
		for(int i=0; i<4; i++){
			newFrame.core.Data[3-i] = (nodeTable[cc_nodeID].nodeStatusWord >> (8*i)) & 0xff;			// Convert uint32_t -> uint8_t
		}

	#ifdef DEBUG
		static uint8_t hbmsg[] = "CC->MC HB issued\n";
		Serial2_writeBytes(hbmsg, sizeof(hbmsg)-1);
	#endif

		xQueueSendToFront(motCanTxBufHandle, &newFrame, 0);
	}
  /* USER CODE END TmrSendHB */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
