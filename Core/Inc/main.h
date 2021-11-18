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
//#include "usart.h"
#include "Shared_Libraries/config.h"
#include "Shared_Libraries/aes.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct {
  unsigned short int const sw_version;
  unsigned short int const blank1;
  unsigned short int const blank2;
  unsigned short int const blank3;
  unsigned short int const blank4;
  unsigned short int const blank5;
  unsigned short int const blank6;
  unsigned short int const blank7;
  unsigned short int const blank8;
}Version;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define BUFSIZE		1040
extern uint8_t transceiver;
extern uint8_t transceiver_saved;
extern uint8_t LoRa_channel_received;
extern volatile uint8_t transmission;
extern volatile uint8_t lora_int_stat;
extern volatile uint8_t LoRa_bindMode;
extern  uint8_t LoRa_channel_received;
extern uint8_t LoRa_bindMode_slave;
extern int baudrate_timeout;
extern uint8_t checkRouting;
extern volatile int delay_reset;
extern unsigned int backup_timeout;
extern volatile unsigned int bldc_Speed;
extern unsigned int store_in_flash;
extern uint8_t init_main_state;
void save_to_buffer(char* input, uint32_t length);

extern volatile uint8_t UARTBuffer0[BUFSIZE];
extern unsigned char uartMode;
extern UART_HandleTypeDef huart2;
extern volatile uint32_t UARTCount0;

extern uint16_t online_timeouts[165];
extern uint8_t available_positioners[20];
extern unsigned char ButtonStatus;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void ClearStatus(uint8_t motor_id);
void update_flash_backup();
void read_SN(void);


void ModBus_SendInt(unsigned int val);
int write_SysVars(unsigned int addr);
int verify_SysVars(unsigned int addr);
void read_SysVars(unsigned int addr);

void modbus_cmd (void);
void modbus_crc(int length, int mode);
void flash_read_boot (unsigned int read_address, unsigned int size);
void flash_write_boot(unsigned int write_address, volatile uint8_t* flash_write_buffer, unsigned int size);
void flash_erase(unsigned int start_sector, unsigned int stop_sector);
void eraseApp();
void writeApp(unsigned int address, unsigned int size);
void flash_checksum (unsigned int start_address, unsigned int size);
void set_tx_flag(uint8_t* tx_buffer, uint8_t length);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TIM1_PRESCALER 0
#define CHARGE_PUMP_PERIOD 0
#define CHARGE_PUMP_PULSE 0
#define MOTOR_PWM_PERIOD 0
#define DEVICE 8
#define LoRa_RESET_Pin GPIO_PIN_13
#define LoRa_RESET_GPIO_Port GPIOC
#define RS485_TX_Pin GPIO_PIN_2
#define RS485_TX_GPIO_Port GPIOA
#define LORA_NSS_Pin GPIO_PIN_4
#define LORA_NSS_GPIO_Port GPIOA
#define CTS_OUT4_Pin GPIO_PIN_13
#define CTS_OUT4_GPIO_Port GPIOB
#define RTS_OUT3_Pin GPIO_PIN_14
#define RTS_OUT3_GPIO_Port GPIOB
#define RS485_RX_Pin GPIO_PIN_15
#define RS485_RX_GPIO_Port GPIOA
#define TX_OUT2_Pin GPIO_PIN_10
#define TX_OUT2_GPIO_Port GPIOC
#define RX_OUT1_Pin GPIO_PIN_11
#define RX_OUT1_GPIO_Port GPIOC
#define BT1_Pin GPIO_PIN_12
#define BT1_GPIO_Port GPIOC
#define BT2_Pin GPIO_PIN_2
#define BT2_GPIO_Port GPIOD
#define LORA_SCK_Pin GPIO_PIN_3
#define LORA_SCK_GPIO_Port GPIOB
#define LORA_MISO_Pin GPIO_PIN_4
#define LORA_MISO_GPIO_Port GPIOB
#define LORA_MOSI_Pin GPIO_PIN_5
#define LORA_MOSI_GPIO_Port GPIOB
#define LORA_DIO6_Pin GPIO_PIN_6
#define LORA_DIO6_GPIO_Port GPIOB
#define LED_RD_Pin GPIO_PIN_7
#define LED_RD_GPIO_Port GPIOB
#define BOOT_Pin GPIO_PIN_3
#define BOOT_GPIO_Port GPIOH
#define LED_GR_Pin GPIO_PIN_8
#define LED_GR_GPIO_Port GPIOB
#define LED_BL_Pin GPIO_PIN_9
#define LED_BL_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define LORA 1
#define XBEE 2
#define NONE 0

#define KVARK 5
#define XBEE2RS485 7
#define MICRO 8
#define PICO  9
//#define DEVICE KVARK

#define WAIT_TO_APPL 6000
#define WAIT_TO_APPL2 60000

#define set                 1
#define clear               0

//RS485 commands
#define	CMD_READ            0x01
#define CMD_ERASE           0x02
#define	CMD_WRITE           0x03
#define	CMD_GET_CS          0x04
#define	CMD_RESET           0x05
#define	CMD_GET_STATUS      0x06

#define CMD_GET_LOADER_VER  0x07
#define CMD_GET_HW_REV	    0x08
#define CMD_GET_DEVICE_TYPE 0x09
#define CMD_GET_MINVERSION  0x0A
#define CMD_SET_ID          0x0B
#define CMD_SET_SERIAL_ID   0x0C
#define CMD_GET_APP_SIZE    0x0D
#define CMD_GET_APP_ADDR    0x0E
#define CMD_GET_VERSION     0x0F

#define RX_MODE_NORMAL 0
#define RX_MODE_CRYPTED 1
#define CRC_MODE_NORMAL 1
#define CRC_MODE_UPGRADE 2

#define ACK_OK				0x00
#define ACK_ERROR           0xFF

#define CRC_BOOT 0x1234
#if (DEVICE == PICO)
  #define CRC_UPGRADE 0x468A
#elif (DEVICE == MICRO)
  #define CRC_UPGRADE 0x6942
#else
  #define CRC_UPGRADE CRC_BOOT
#endif

typedef struct //mora biti 256 bytov velik
{
  unsigned int LOADER_VER;
  unsigned int HW_REV;
  unsigned int DEV_TYPE;
  unsigned int DEV_MIN_VERSION;
  unsigned int FLASH_APP_SIZE;
  unsigned int FLASH_APP_START_ADDRESS;
  unsigned int reseved[58];
} system_defs_t;

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
