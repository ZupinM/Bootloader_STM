/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LoRa_RESET_GPIO_Port, LoRa_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_RD_Pin|LED_GR_Pin|LED_BL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LoRa_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LoRa_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LORA_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = BT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = BT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(BT2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = LORA_DIO6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LORA_DIO6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin */
  GPIO_InitStruct.Pin = LED_RD_Pin|LED_GR_Pin|LED_BL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = BOOT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

uint32_t LEDTimeouts[4];
uint32_t LEDStates;
uint32_t LEDStates_old;

void set_LED(uint8_t color, uint8_t state, uint16_t timeout){

	if(state == LED_ON){
		if(timeout && !(LEDStates & 1<<color)){ //set timeout only when LED was OFF before
			LEDTimeouts[color] = timeout;
		}else{
			LEDTimeouts[color] = 0;
		}
		LEDStates |= 1<<color;
	}else if(state == LED_OFF){
		LEDStates &= ~(1<<color);
	}
	else if(state == COUNTDOWN){
		for(int i=0 ; i<4 ; i++){
			if(LEDTimeouts[i]){
				LEDTimeouts[i]--;
			}
			else if(LEDTimeouts[i] == 1){
				LEDStates &= ~(1<<color);
			}
		}
	}

	if((LEDStates & 1<<RED) != (LEDStates_old & 1<<RED)){
		HAL_GPIO_WritePin(LED_RD_GPIO_Port, LED_RD_Pin, (LEDStates & 1<<RED ? GPIO_PIN_RESET : GPIO_PIN_SET) );
	}
	if((LEDStates & 1<<GREEN) != (LEDStates_old & 1<<GREEN)){
		HAL_GPIO_WritePin(LED_GR_GPIO_Port, LED_GR_Pin, (LEDStates & 1<<GREEN ? GPIO_PIN_RESET : GPIO_PIN_SET) );
	}
	if((LEDStates & 1<<BLUE) != (LEDStates_old & 1<<BLUE)){
		HAL_GPIO_WritePin(LED_BL_GPIO_Port, LED_BL_Pin, (LEDStates & 1<<BLUE ? GPIO_PIN_RESET : GPIO_PIN_SET) );
	}

	if((LEDStates & 1<<WHITE) != (LEDStates_old & 1<<WHITE)){
		HAL_GPIO_WritePin(LED_RD_GPIO_Port, LED_RD_Pin, (LEDStates & 1<<WHITE ? GPIO_PIN_RESET : GPIO_PIN_SET) );
		HAL_GPIO_WritePin(LED_GR_GPIO_Port, LED_GR_Pin, (LEDStates & 1<<WHITE ? GPIO_PIN_RESET : GPIO_PIN_SET) );
		HAL_GPIO_WritePin(LED_BL_GPIO_Port, LED_BL_Pin, (LEDStates & 1<<WHITE ? GPIO_PIN_RESET : GPIO_PIN_SET) );
	}

	LEDStates_old = LEDStates;

}



/* USER CODE END 2 */
