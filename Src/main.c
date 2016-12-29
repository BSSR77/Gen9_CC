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

// Handler for node HB timeout
void TmrHBTimeout(void const * argument){
 	uint8_t timerID = (uint8_t)pvTimerGetTimerID((TimerHandle_t)argument);
#ifdef DEBUG
	Serial2_writeBytes("Timer ",6);
	Serial2_write(timerID);
	Serial2_writeBytes(" triggered!\n",12);
#endif
	nodeTable[timerID].nodeConnectionState = UNRELIABLE;
	if((timerID) != mc_nodeID){
		xQueueSend(badNodesHandle, argument,portMAX_DELAY);
	}
}

void setupNodeTable(){
	#ifdef cc_nodeID
		nodeTable[cc_nodeID].nodeConnectionState = DISCONNECTED;
		nodeTable[cc_nodeID].nodeFirmwareVersion = cc_VERSION;
		nodeTable[cc_nodeID].nodeStatusWord = SW_Sentinel;
	#endif

	#ifdef mc_nodeID
		nodeTable[mc_nodeID].nodeConnectionState = DISCONNECTED;
		nodeTable[mc_nodeID].nodeFirmwareVersion = mc_VERSION;
		nodeTable[mc_nodeID].nodeStatusWord = SW_Sentinel;
	#endif

	#ifdef bps_nodeID
		nodeTable[bps_nodeID].nodeConnectionState = DISCONNECTED;
		nodeTable[bps_nodeID].nodeFirmwareVersion = bps_VERSION;
		nodeTable[bps_nodeID].nodeStatusWord = SW_Sentinel;
	#endif

	#ifdef ads_nodeID
		nodeTable[ads_nodeID].nodeConnectionState = DISCONNECTED;
		nodeTable[ads_nodeID].nodeFirmwareVersion = ads_VERSION;
		nodeTable[ads_nodeID].nodeStatusWord = SW_Sentinel;
	#endif

	#ifdef radio_nodeID
		nodeTable[radio_nodeID].nodeConnectionState = DISCONNECTED;
		nodeTable[radio_nodeID].nodeFirmwareVersion = radio_VERSION;
		nodeTable[radio_nodeID].nodeStatusWord = SW_Sentinel;
	#endif
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
  setupNodeTable();
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
  //Start the watchdog and heartbeat timers
  osTimerStart(HBTmrHandle, CCMC_HB_Interval);
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
  /* Infinite loop */
  for(;;)
  {
	osDelay(1);
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
    osDelay(1);
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
    osDelay(1);
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

		  Can_sendStd(newFrame.core.id,newFrame.isRemote,newFrame.core.Data,newFrame.core.dlc);
	  }
  }
  /* USER CODE END doMotCanTx */
}

/* doKickDog function */
void doKickDog(void const * argument)
{
  /* USER CODE BEGIN doKickDog */
  /* Infinite loop */
  TickType_t xLastWakeTime = xTaskGetTickCount ();
  for(;;)
  {
	taskENTER_CRITICAL();
	HAL_WWDG_Refresh(&hwwdg);
	taskEXIT_CRITICAL();

	vTaskDelayUntil(&xLastWakeTime, WD_Interval);
  }
  /* USER CODE END doKickDog */
}

/* TmrSendHB function */
void TmrSendHB(void const * argument)
{
  /* USER CODE BEGIN TmrSendHB */
	static Can_frame_t newFrame;
	uint32_t motStatus = nodeTable[mc_nodeID].nodeStatusWord & 0x07;

	if((motStatus == ACTIVE) || (motStatus == SHUTDOWN)){
		// Only send HB to MC if MC is in ACTIVE or SHUTDOWN state
		newFrame.isExt = 0;
		newFrame.isRemote = 0;
		xSemaphoreTake(nodeEntryMtxHandle[cc_nodeID], portMAX_DELAY);
		newFrame.core.id = cc_SW;
		xSemaphoreGive(nodeEntryMtxHandle[cc_nodeID]);
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
