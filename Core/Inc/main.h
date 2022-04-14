/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Shared_Libraries/aes.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
  unsigned short int const sw_version;
  unsigned short int const boot_version;
  unsigned short int const blank2;
  unsigned short int const blank3;
  unsigned short int const blank4;
  unsigned short int const blank5;
  unsigned short int const blank6;
  unsigned short int const blank7;
  unsigned short int const blank8;
}Version;

typedef struct //mora biti 256 bytov velik
{
  unsigned short int LOADER_VER_1;
  unsigned short int LOADER_VER;
  unsigned int HW_REV;
  unsigned int DEV_TYPE;
  unsigned int DEV_MIN_VERSION;
  unsigned int FLASH_APP_SIZE;
  unsigned int FLASH_APP_START_ADDRESS;
  unsigned int reseved[58];
} system_defs_t;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM1_PRESCALER 0
#define CHARGE_PUMP_PERIOD 0
#define CHARGE_PUMP_PULSE 0
#define MOTOR_PWM_PERIOD 0
#define DEVICE 8
#define LED_RD_Pin GPIO_PIN_7
#define LED_RD_GPIO_Port GPIOB
#define LED_GR_Pin GPIO_PIN_8
#define LED_GR_GPIO_Port GPIOB
#define LED_BL_Pin GPIO_PIN_9
#define LED_BL_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
