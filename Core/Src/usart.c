/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#define SystemFrequency SystemCoreClock
volatile uint8_t UARTTxEmpty0 = 1;
volatile uint8_t UARTTxEmpty1 = 1;
volatile uint8_t UARTTxEmpty2 = 1;
volatile uint8_t UARTBuffer0[BUFSIZE];
volatile uint8_t UARTBuffer0[BUFSIZE];
volatile uint8_t UARTBuffer1[BUFSIZE];
volatile uint8_t UARTBuffer2[BUFSIZE];
volatile int rxTimeout0;
volatile int rxTimeout1;
volatile int rxTimeout2;
volatile uint32_t UARTCount0 = 0;
volatile uint32_t UARTtxCount0;
volatile uint32_t UARTtxCount1;
volatile uint32_t UARTtxCount2;
         uint8_t *BufferTXPtr0;
volatile uint8_t *BufferTXPtr1;
volatile uint8_t *BufferTXPtr2;
volatile uint32_t UARTCount1 = 0;
volatile uint32_t UARTCount2 = 0;
volatile uint32_t UARTCount3 = 0;
volatile uint8_t ModbusState0;
volatile uint8_t ModbusState1;
volatile uint8_t ModbusState2;
uint16_t flow_ctrl_hangup_timer = 0;
unsigned char uartMode;

UART_HandleTypeDef *huart485;
DMA_HandleTypeDef *hdma_usart485_rx;
DMA_HandleTypeDef *hdma_usart485_tx;
/* USER CODE END 0 */

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT|UART_ADVFEATURE_DMADISABLEONERROR_INIT;
  huart2.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  huart2.AdvancedInit.DMADisableonRxError = UART_ADVFEATURE_DMA_DISABLEONRXERROR;
  if (HAL_RS485Ex_Init(&huart2, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

#if DEVICE == KVARK
  huart485 = &huart1;
  hdma_usart485_rx = &hdma_usart1_rx;
  hdma_usart485_tx = &hdma_usart1_tx;
#else
  huart485 = &huart2;
  hdma_usart485_rx = &hdma_usart2_rx;
  hdma_usart485_tx = &hdma_usart2_tx;
#endif

  if(HAL_UART_Receive_DMA(huart485, (uint8_t *)UARTBuffer0, BUFSIZE) != HAL_OK) //Start receiving
  {
    Error_Handler();
  }
  __HAL_UART_DISABLE_IT(huart485, UART_IT_PE);    //Disable Parity Error interrupt
  __HAL_UART_DISABLE_IT(huart485, UART_IT_FE);    //Disable Framing Error interrupt
  __HAL_UART_DISABLE_IT(huart485, UART_IT_ORE);    //Disable Overrun Error interrupt
  __HAL_UART_DISABLE_IT(huart485, UART_IT_ERR);
  __HAL_UART_ENABLE_IT(huart485, UART_IT_IDLE); //Enable UART Idle interrupt

  /* USER CODE END USART2_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA1     ------> USART2_DE
    PA2     ------> USART2_TX
    PA15 (JTDI)     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1|RS485_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = RS485_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_USART2;
    HAL_GPIO_Init(RS485_RX_GPIO_Port, &GPIO_InitStruct);

    /* USART2 DMA Init */
    /* USART2_RX Init */
    hdma_usart2_rx.Instance = DMA1_Channel6;
    hdma_usart2_rx.Init.Request = DMA_REQUEST_2;
    hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_rx.Init.Mode = DMA_NORMAL;
    hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart2_rx);

    /* USART2_TX Init */
    hdma_usart2_tx.Instance = DMA1_Channel7;
    hdma_usart2_tx.Init.Request = DMA_REQUEST_2;
    hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart2_tx.Init.Mode = DMA_NORMAL;
    hdma_usart2_tx.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart2_tx);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
    PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB13     ------> USART3_CTS
    PB14     ------> USART3_RTS
    PC10     ------> USART3_TX
    PC11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = CTS_OUT4_Pin|RTS_OUT3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = TX_OUT2_Pin|RX_OUT1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA1     ------> USART2_DE
    PA2     ------> USART2_TX
    PA15 (JTDI)     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|RS485_TX_Pin|RS485_RX_Pin);

    /* USART2 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB13     ------> USART3_CTS
    PB14     ------> USART3_RTS
    PC10     ------> USART3_TX
    PC11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, CTS_OUT4_Pin|RTS_OUT3_Pin);

    HAL_GPIO_DeInit(GPIOC, TX_OUT2_Pin|RX_OUT1_Pin);

  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *UartHandle)
{
  /* Set transmission flag: transfer complete */
	if(HAL_UART_DMAStop(huart485) != HAL_OK){ // Transfer complete
		Error_Handler();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	volatile uint32_t tmp;                  	// volatile to prevent optimizations
	tmp = huart485->Instance->ISR;                  // Read status register and data reg to clear RX flag
	tmp = huart485->Instance->RDR;
	(void) tmp;									// only to not have the compiler warning (variable not used)

	UARTCount0 = BUFSIZE - __HAL_DMA_GET_COUNTER(hdma_usart485_rx);
    ModbusState0 |= MODBUS_PACKET_RECIVED;

    if(HAL_UART_DMAStop(huart485) != HAL_OK ) //Stop receiving
    {
      Error_Handler();
    }
    //Reception is re-enabled in main

}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
    Error_Handler();
}



void UARTSend(uint8_t *BufferPtr, uint32_t Length)
{
	  if(HAL_UART_Transmit_DMA(huart485, (uint8_t*)BufferPtr, Length*2)!= HAL_OK) //Set length to double and treat HalfCplt interrupt as Transfer Complete
	  {
	    Error_Handler();
	  }
}

void UART1Send(uint8_t *BufferPtr, uint32_t Length)
{

}

int modbus_newRequest()
{
  if (uartMode == UART_MODE_RS485)
    return (ModbusState0 & MODBUS_PACKET_RECIVED) ? 1:0;
  else if (uartMode == UART_MODE_XBEE)
    return (ModbusState1 & MODBUS_PACKET_RECIVED) ? 1:0;
  else
	return 0;
}
int modbus_newRequest1()
{
  return (ModbusState1 & MODBUS_PACKET_RECIVED) ? 1:0;
}

int modbus_newRequest2()
{
  return (ModbusState2 & MODBUS_PACKET_RECIVED) ? 1:0;
}


int modbus_discard()
{
  if (uartMode == UART_MODE_RS485)
    return (ModbusState0 & MODBUS_DISCARD_PACKET) ? 1:0;
  else if (uartMode == UART_MODE_XBEE)
    return (ModbusState1 & MODBUS_DISCARD_PACKET) ? 1:0;
  else
	return 0;
}

int modbus_discard1()
{
  return (ModbusState1 & MODBUS_DISCARD_PACKET) ? 1:0;
}


int modbus_discard2()
{
  return (ModbusState2 & MODBUS_DISCARD_PACKET) ? 1:0;
}

void modbus_ReqProcessed()
{
  if (uartMode == UART_MODE_RS485) {
    ModbusState0 &= MODBUS_CLEAR_MASK;
    UARTCount0 = 0;
  }
  else if (uartMode == UART_MODE_XBEE) {
    ModbusState1 &= MODBUS_CLEAR_MASK;

    UARTCount1 = 0;
  }
  //uartMode = UART_MODE_NONE;
}

void reEnable_485_DMA_RX(void){

	uint32_t timeout_counter = 1000000;
	while (! (huart485->RxState == HAL_UART_STATE_READY && hdma_usart485_tx->State != HAL_DMA_STATE_BUSY)){	//Reenable reception, when DMA is stoped in HAL_UART_RxCpltCallback
		if(timeout_counter-- == 0){
			Error_Handler();
		}
	}
	if(HAL_UART_DMAStop(huart485) ) //stop again to prevent errors
	{
	  Error_Handler();
	}
	if(HAL_UART_Receive_DMA(huart485, (uint8_t *)UARTBuffer0, BUFSIZE) != HAL_OK) 	//Start receiving
	{
	  Error_Handler();
	}


}

void UART_ChangeBaudRate(int baud){
	HAL_UART_DMAStop(huart485);
    HAL_UART_DeInit(huart485);
    huart485->Init.BaudRate = baud;
    if (HAL_RS485Ex_Init(huart485, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
    {
      Error_Handler();
    }
	if(HAL_UART_Receive_DMA(huart485, (uint8_t *)UARTBuffer0, BUFSIZE) != HAL_OK) 	//Start receiving
	{
	  Error_Handler();
	}
}

void modbus_ReqProcessed1()
{
  ModbusState1 &= MODBUS_CLEAR_MASK;
  UARTCount1 = 0;
}

void modbus_ReqProcessed2()
{
  ModbusState2 &= MODBUS_CLEAR_MASK;
 // UARTCount2 = 0;
}


void UART0ClearStatus()
{
 // LPC_USART0->STAT &= ~(1 << 8) | ~(1 << 13) | ~(1 << 14) | ~(1 << 15) | ~(1 << 16); // clear status errors
}

void UART1ClearStatus()
{
 // LPC_USART1->STAT &= ~(1 << 8) | ~(1 << 13) | ~(1 << 14) | ~(1 << 15) | ~(1 << 16); // clear status errors
}


void UART2ClearStatus()
{
 // LPC_USART2->STAT &= ~(1 << 8) | ~(1 << 13) | ~(1 << 14) | ~(1 << 15) | ~(1 << 16); // clear status errors
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
