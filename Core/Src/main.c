/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "usb.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "Shared_Libraries/SX1278.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include "Shared_Libraries/config.h"
#include "RTT/SEGGER_RTT.h"
#include "flash.h"

//#define PRODUCTION_RELEASE

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
typedef void (*ptrF)(uint32_t dlyticks);
typedef void (*pFunction)(void);

#define FLASH_APP_ADDR 0x801f000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

//const unsigned char __attribute__ ((section(".bootSectionFLASH"))) boot_flash[50] = {};


uint32_t goref_Nday_cnt_B;
uint32_t goref_Nday_cnt_A;
uint8_t upgrExe_gl;

SEGGER_RTT_PRINTF_DESC BufferDesc;
char acBuffer[SEGGER_RTT_PRINTF_BUFFER_SIZE];
  unsigned NumDigits = 4;
  unsigned FormatFlags = 0;
  unsigned FieldWidth = 0;

/* parametri */
unsigned int events;
float err_currentA;
float err_positionA;
float err_voltageA;
float err_currentB;
float err_positionB;
float err_voltageB;

extern unsigned int Address_old;

unsigned int tracker_status;			//status kondicije, v kateri je tracker (napake, halli...)
unsigned int tracker_status2;			//Status for motors 1 & 3 when motor_count > 3
unsigned int tracker_exstatus;
unsigned int tracker_exstatus2;
uint8_t slave_addr;			//slave address on RS485

unsigned int SN[4];	   			//vsebujejo serijske stevilke

unsigned int modbus_timeout;                    //po tem casu gre v rest_position [ure]
unsigned int modbus_cnt;					//stevec
unsigned int modbus_cnt1;         //stevec
unsigned int modbus_cnt2;         //stevec
unsigned int modbus_timeout_delay;              //unsigned int modbus_timout_delay * MODBUS_ID  [s]

unsigned int crc_errors;

 extern unsigned char pcb_version1;                //TIV           27   27C1=0x1B 0x43 0x01 .... 0x1B=27 0x43='C' 0x01=1 .... vpisi 0x1B4301 oziroma 1786625
 extern char pcb_version2;                         //verzija TIVa  C
 extern unsigned char pcb_version3;                //TIV polaganje 1

uint32_t systick_count = 0;

/* system */
unsigned int  button_timeout;
unsigned int  OverUnderVoltageFlagClearTimer;

//unsigned char move_direction_B;

//unsigned int time_out;					//na silo ustavljen motor preden stop_motor()
unsigned int backup_timeout;			//zakasnjen vpis v flash - backup
unsigned int green_led;					//utripanje LED
unsigned int tick_1s;					//stevec za generiranj 1s intervala
unsigned int counter10s;                                //stevec za generiranj 10s intervala
unsigned int modbus_indicator;			//stevec dolzine utripa ob rs485 sprejetju stringa
extern volatile int slaveCommandTimeout;

/* flags registers */
unsigned int flags = 0;

/* rs485 */
extern volatile uint8_t UARTTxEmpty0;
extern volatile uint8_t UARTTxEmpty1;

unsigned int reset_status;
volatile unsigned int start_count = 0;
uint8_t init_main_state = 1;

extern volatile unsigned int number_of_poles;
extern int enableUpgrade;

//fixed loaction in flash for version and checksum
__attribute__ ((section (".appver")))
//fsta
const Version swVersion={.sw_version=8001,
                         .blank1=0xffff,
                         .blank2=0xffff,
                         .blank3=0xffff,
                         .blank4=0xffff,
                         .blank5=0xffff,
                         .blank6=0xffff,
                         .blank7=0xffff,
                         .blank8=0xffff
                         };

uint8_t usbcnt = 0;


uint8_t transceiver = NONE;
uint8_t transceiver_saved;
uint8_t tx_settings_flag;
uint8_t tx_setting_route;
uint8_t set_settings_flag;
uint8_t tx_buffered_flag;
uint8_t timeout_counter;
uint8_t bindMode_byChannel = 0;
uint8_t route_settings_delay = 0;

uint8_t tx_settings_flag;
uint8_t tx_setting_route;
uint8_t set_settings_flag;
uint8_t tx_buffered_flag;
uint8_t tx_packet_buffer[BUFSIZE];
uint32_t tx_packet_length;
uint8_t SPI_RxFifo_cmplt;
uint8_t SPI_TxFifo_cmplt;
uint8_t LoRa_firstVerPacket_received;

uint16_t online_timeouts[165];
uint8_t available_positioners[20];
int baudrate = 19200;
int baudrate_timeout = 1;

unsigned int crc_calc;
unsigned int number_TX_bytes;
unsigned char m_ack_state;				//vsebuje opis napake, za katero MODBUS ukaz NI bil izvrsen
unsigned int read_int_buf[4];
unsigned int tick_1ms_cnt;
unsigned int wait_appl_cnt;				//10 sekund caka, preden gre v applickacijo
float *f_flash_pointer;					//kazalec za branje po flash-u
unsigned int *i_flash_pointer;			//kazalec za branje po flash-u
unsigned int crypdedRx = RX_MODE_NORMAL;

#if (DEVICE == MICRO)
__attribute__ ((section (".sysdata")))
const  system_defs_t sys_defs =   {
	8001, //LOADER_VER;
	1,    //HW_REV;
	8,    //DEV_TYPE;        //ne spreminjaj
	8000,  //DEV_MIN_VERSION;
    0x61000,  //FLASH_APP_SIZE;
    0x1f000   // FLASH_APP_START_ADDRESS;
};
#endif

extern volatile uint8_t UARTBuffer0[BUFSIZE];
volatile uint8_t rs485_forward_enabled;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void ButtonProcess ();
void StatusUpdate();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

volatile int systick_ms;
void SysTick_Handler(void)
{
	  systick_count++;
	  systick_ms=1;

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

void go2APP(void)
{
	uint32_t JumpAddress;
	pFunction Jump_To_Application;
	printf("BOOTLOADER Start\r\n");

	//check if there is something "installed" in the app FLASH region
	if(((*(__IO uint32_t*) FLASH_APP_ADDR) & 0x2FFC0000) == 0x20000000)
	{
		printf("APP Start ...\r\n");
		HAL_Delay(100);
		//jump to the application
		JumpAddress = *(uint32_t *) (FLASH_APP_ADDR + 4);
		Jump_To_Application = (pFunction) JumpAddress;
		//initialize application's stack pointer
		__set_MSP(*(uint32_t *)FLASH_APP_ADDR);
		Jump_To_Application();
	}
	else
	{
		//No App found
		printf("No APP found\r\n");
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI3_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */

#ifdef PRODUCTION_RELEASE 					//Code Read Protection
  FLASH_OBProgramInitTypeDef CRP_settings;
  HAL_FLASH_Unlock();
  HAL_FLASH_OB_Unlock();
  HAL_FLASHEx_OBGetConfig(&CRP_settings);
  if(CRP_settings.RDPLevel == OB_RDP_LEVEL_0){
	  CRP_settings.RDPLevel = OB_RDP_LEVEL_1;
	  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
	  HAL_FLASHEx_OBProgram(&CRP_settings);
	  HAL_FLASH_OB_Launch();
  }
#endif

  //HAL_Delay(500);

  //sys_data_write();
  set_LED(GREEN, LED_ON, 0);
  read_SN();
  wait_appl_cnt=WAIT_TO_APPL;

  //dobi slave ID naslov
  read_SysVars(FLASH_ADDR_MAIN);
  if ((slave_addr==0)||(slave_addr>0xFE)) {				//ce niso shranjeni, bo branje 0xFF -> slave_addr=0 (float -1.#QNAN)
    read_SysVars(FLASH_ADDR_BACKUP);					//vzemi zadnje dobre iz backup-a
    if ((slave_addr==0)||(slave_addr>0xFE)) {
        slave_addr=255;		//ce se teh ni, vzemi default ID=255
    }
  }

  if(transceiver == LORA) {
    LoRa_config(module.channel, module.power, module.spFactor, module.LoRa_BW, LoRa_MAX_PACKET, RxMode);
  }
  else{
	transceiver = NONE;
	uartMode = UART_MODE_RS485;
	LoRa_id = 116;  // default
  }
  flags&=~(1<<reset_it);
  SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	     if (systick_ms != 1)
	       continue;
	     systick_ms=0;       // execute every 1ms

	     if (--wait_appl_cnt == 0) {
	    	 MX_USB_DEVICE_DeInit();
	    	 enableUpgrade = 0;
	    	 go2APP();
	    	 wait_appl_cnt = WAIT_TO_APPL2; // else wait
	     }

	     set_LED(0,COUNTDOWN,0); //LED timeouts processing

	     if (transceiver == LORA) {

	 		if(SPI_TxFifo_cmplt == 1){
		 		 uint8_t tmp = 0x8b;
		 		 LoRa_SPIWrite(LR_RegOpMode, &tmp, 1); //Tx Mode
		 		 SPI_TxFifo_cmplt = 0;
	 		}
			if(SPI_RxFifo_cmplt == 1){
				lora_int_stat = 0;
				checkRouting = 1;
				uint8_t tmp = 0x8D;
				LoRa_SPIWrite(LR_RegOpMode, &tmp, 1); //Rx Mode
				//rxOffline_counter = 10000;
				SPI_RxFifo_cmplt = 0;
			}
			get_LoRa_Status_DIO();
	       if(lora_int_stat == TRANSMISSION_FINISHED)
	         tx_finished();
	       if(lora_int_stat == PACKET_RECEIVED)
	         rx_finished();

	       if(checkRouting && !rs485_forward_enabled)
	         check_routing();
	       if(module.packetReady && (module.rxBuffer[0] == LoRa_id || module.rxBuffer[0] == slave_addr || module.rxBuffer[0] == 0 )){
	         modbus_cmd ();         //LoRa receive, KVARK response
	         modbus_ReqProcessed(); //re-enable reception
	       }
	       else if(module.packetReady){
	         //modbus_cmd1();              // LoRa received, forward through 485->nano
	         modbus_ReqProcessed1();      // re-enable reception
	       }
	       module.packetReady = 0;

	       if(transmission == 0){

	         if(tx_buffered_flag){
	           LoRa_TxPacket((uint8_t *)tx_packet_buffer, tx_packet_length, 8000);
	           tx_buffered_flag = 0;
	           if(set_settings_flag)   //exit sending settings, -> set settings
	             tx_settings_flag = 0;
	         }

	       }
	     }else if(modbus_newRequest() && (huart485->gState != HAL_UART_STATE_BUSY && huart485->RxState != HAL_UART_STATE_BUSY_RX ) ){
	    	 //save_to_buffer(UARTBuffer0, 8);
	         //SEGGER_RTT_printf(0, "id:%#02x  cmd:%#02x %#02x %#02x %#02x %#02x %#02x %#02x %#02x l:%d\n" , UARTBuffer0[0], UARTBuffer0[1], UARTBuffer0[2], UARTBuffer0[3], UARTBuffer0[4], UARTBuffer0[5], UARTBuffer0[6], UARTBuffer0[7], UARTBuffer0[8], UARTCount0);

		     // MODBUS RS485
		       modbus_cmd();
		       modbus_ReqProcessed();
		       reEnable_485_DMA_RX();
		 }

	     USB_write();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
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
  /** Initializes the CPU, AHB and APB buses clocks
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
  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */

char glSavingBuffer[50][8];
uint32_t glBuff_index;
void save_to_buffer(char* input, uint32_t length){
	if((glBuff_index+1)*length > sizeof(glSavingBuffer)){
		return;
	}
	glBuff_index ++;
	for(int i=0 ; i<=length ; i++){
		glSavingBuffer[glBuff_index][i] = *(input + i);
	}
}

/***********************************************************
  LED HANDLING & 1 second TICK
************************************************************/
void led_handling() {

  if ((tracker_status&SF_NO_MODBUS) || (modbus_indicator > 0)) {    // oranzna sveti, ce MODbus komunikacije ni - najbolj prednostna indikacija
    set_LED(RED, LED_ON, 0);
    set_LED(GREEN, LED_ON, 0);
    tick_1s = 1000;     // zaradi lepsega prehoda iz oranzne nazaj na zeleno (tick_1s)
  } else {
    set_LED(RED, LED_OFF, 0);
    if (green_led == 0)
      set_LED(GREEN, LED_OFF, 0);

  }

  if (modbus_indicator)
    modbus_indicator--;

   //1s TIME TICK
  if (green_led) {      // zeleno utripanje = delovanje ok
    set_LED(GREEN, 1, 0);
    green_led--;
  } else
    set_LED(GREEN, 0, 0);

  // zeleno normalno utripanje
  if (tick_1s == 0) {
    tick_1s = 1000;	//reload 1000 * 1ms = 1s   ... izbrana A os
    green_led = 30;
  }
  else
    tick_1s--;
}

/***********************************************************
  MODBUS COMMANDS
************************************************************/
void modbus_cmd () {

  unsigned int Utemp;

  if(transceiver == LORA){
    memcpy((uint8_t *)UARTBuffer0, (uint8_t *)module.rxBuffer, BUFSIZE);
    UARTCount0 = module.packetLength;
    module.packetReady = 0;
  }
  //0x43: LoRa write version, 73: 485 write version
  if(UARTCount0 == 0x13 || UARTCount0 == 0x93 || UARTCount0 == 0x73|| UARTCount0 == 0x43 || UARTCount0 == 0x113)
    crypdedRx = RX_MODE_CRYPTED;
  else
    crypdedRx = RX_MODE_NORMAL;

  unsigned int upgMode = CRC_MODE_NORMAL;
  if ((UARTCount0 > 0) && ((UARTBuffer0[0] == slave_addr) || (UARTBuffer0[0] == 0))) {
    modbus_crc (UARTCount0, upgMode);
    if (crc_calc != 0) {
      upgMode = CRC_MODE_UPGRADE;
      modbus_crc (UARTCount0, upgMode);
    }

    // disable reset, status (0x01, 0x06 lenght 4) and get all params (0x78) command - only from product
    if ((crc_calc == 0 && crypdedRx == RX_MODE_CRYPTED) || (crc_calc == 0 && UARTBuffer0[1] != 0x78 && crypdedRx == RX_MODE_NORMAL)) {
      if (crypdedRx == RX_MODE_CRYPTED || !((UARTBuffer0[1] == 0x01 || UARTBuffer0[1] == 0x06) && UARTCount0 == 4 && crypdedRx == RX_MODE_NORMAL)) {
        unsigned int addr = 0, size = 0;

        if (crypdedRx == RX_MODE_CRYPTED) {
          if (decryptData ((char *)&UARTBuffer0[1], UARTCount0 - 3) != 0) {
            UARTBuffer0[2] = ACK_ERROR;                       //ni poravnave na 16 byte-ov
            goto TX;
          }
        }

        addr+=(UARTBuffer0[2]*0x1000000); 	// ID[0], CMD[1]
        addr+=(UARTBuffer0[3]*0x10000);
        addr+=(UARTBuffer0[4]*0x100);
        addr+=UARTBuffer0[5];
        size+=(UARTBuffer0[6]*0x100);
        size+=UARTBuffer0[7];

        //broadcast commands
        if(UARTBuffer0[0] == 0){
          if(UARTBuffer0[1] == CMD_SET_SERIAL_ID) {
            Utemp = UARTBuffer0[2];
            Utemp |= ((unsigned int)UARTBuffer0[3]) << 8;
            Utemp |= ((unsigned int)UARTBuffer0[4]) << 16;
            Utemp |= ((unsigned int)UARTBuffer0[5]) << 24;

            if(SN[0] == Utemp) {
              if((UARTBuffer0[6] > 0) && (UARTBuffer0[6] <= 128)) {
                slave_addr = UARTBuffer0[6];
                write_SysVars(FLASH_ADDR_MAIN);
              }
            }
          }

          number_TX_bytes = 0;

          return;
        }

        //unicast commands
        number_TX_bytes = 3;
        switch (UARTBuffer0[1]) {   //CMD

          //READ
          case CMD_READ: {
            flash_read_boot (addr, size);
            break;
          }
          //ERASE
          case CMD_ERASE: {
            if ((UARTBuffer0[2] < 3) || (UARTBuffer0[3] < 3)) {         //ne sme iti pod 3 sektor - bootloader koda
            UARTBuffer0[2] = ACK_ERROR;
            }
            else
              if (upgMode == CRC_MODE_UPGRADE && crypdedRx == RX_MODE_CRYPTED)
                flash_erase(UARTBuffer0[2], UARTBuffer0[3]);   //100ms !!
            break;
          }
          //WRITE
          case CMD_WRITE: {
            if (addr < (FLASH_APP_START_ADDR&0xfffff)) {         					//ne sme iti pod 3 sektor - bootloader koda
              UARTBuffer0[2] = ACK_ERROR;
            }
            else
              if (upgMode == CRC_MODE_UPGRADE && crypdedRx == RX_MODE_CRYPTED){
            	if(LoRa_firstVerPacket_received && addr > FLASH_APP_VERSION_ADDR){
            		break; //Discard second packet of version write on LoRa
            	}
            	if(addr == FLASH_APP_VERSION_ADDR){	//Write version after upgrade
            		size = 0x10;
            		LoRa_firstVerPacket_received = 1;
            	}
            	UARTBuffer0[2] = flash_write_upgrade(addr, UARTBuffer0, size);
              }
            break;
          }
          //GET CHECKSUM
          case CMD_GET_CS: {
            flash_checksum (addr, size);
            break;
          }
          //RESET
          case CMD_RESET: {
            UARTBuffer0[2] = ACK_OK;
            flags |= (1 << reset_it);
            break;
          }
          //GET STATUS
          case CMD_GET_STATUS: {
            UARTBuffer0[2] = ACK_OK;
            break;
          }

          case CMD_GET_LOADER_VER: {
            ModBus_SendInt(sys_defs.LOADER_VER);
            break;
          }

          case CMD_GET_MINVERSION: {
            ModBus_SendInt(sys_defs.DEV_MIN_VERSION);
            break;
          }
          case CMD_GET_DEVICE_TYPE:{
            ModBus_SendInt(sys_defs.DEV_TYPE);
            break;
          }
          case CMD_GET_HW_REV:{
            ModBus_SendInt(sys_defs.HW_REV);
            break;
          }
          case CMD_GET_APP_SIZE: {
            ModBus_SendInt(sys_defs.FLASH_APP_SIZE);
            break;
          }
          case CMD_GET_APP_ADDR: {
            ModBus_SendInt(sys_defs.FLASH_APP_START_ADDRESS);
            break;
          }
          case CMD_GET_VERSION: {
            flash_read_boot (FLASH_APP_ADDR, 3);
            break;
          }

          case CMD_SET_ID :{
            slave_addr=UARTBuffer0[2];
            write_SysVars(FLASH_ADDR_MAIN);
            UARTBuffer0[2] = ACK_OK;
            break;
          }

          //NOT RECOGNIZED COMMAND
          default: {
            UARTBuffer0[2] = ACK_ERROR;   //ukaz ni prepoznan
            if(crypdedRx == RX_MODE_NORMAL) {
              UARTCount0 = 0;
              return;
            } else
              break;
          }
	}

TX:
        if (crypdedRx == RX_MODE_CRYPTED) {
          number_TX_bytes=encryptData ((char *)&UARTBuffer0[1],number_TX_bytes - 1);
          modbus_crc (++number_TX_bytes, CRC_MODE_NORMAL);
        } else{
          modbus_crc (number_TX_bytes, CRC_MODE_NORMAL);
        }
        UARTBuffer0[number_TX_bytes++] = crc_calc & 0xFF;
        UARTBuffer0[number_TX_bytes++] = crc_calc / 0x100;

        if(transceiver == LORA){
            set_tx_flag((uint8_t *)UARTBuffer0, number_TX_bytes);
        }
        else{
            UARTSend( (uint8_t *)UARTBuffer0, number_TX_bytes );
        }

        UARTCount0 = 0;
        number_TX_bytes = 0;
        wait_appl_cnt = WAIT_TO_APPL2;          //ponovno cakaj timeout preden gres v applikacijo
		//	bootled_cnt=0;                        //zacni utripanje oranzne LED
        if (flags&(1<<reset_it)){
        	HAL_NVIC_SystemReset();
        }
	}
      else UARTCount0 = 0;
      }
    else UARTCount0 = 0;
  }
  else UARTCount0 = 0;
}

/***********************************************************
  MODBUS CRC
************************************************************/

void modbus_crc(int length, int mode) {
unsigned int lsb;
unsigned int j, i;

        //izracunaj crc
  if(mode == CRC_MODE_NORMAL) {
    if (crypdedRx == RX_MODE_CRYPTED)
      crc_calc = CRC_BOOT;
    else
      crc_calc = 0xFFFF;
  }
  else
    crc_calc = CRC_UPGRADE; // ePico upgrade

  for (j = 0; j < length; j++)
  {
    crc_calc ^= UARTBuffer0[j];  //XOR

    for (i = 0; i < 8; i++) //ponavljamo 8x - bayt
    {
      lsb = crc_calc & 0x0001;

      if (lsb)
      {
        crc_calc >>= 1;  //shiftamo za en bit v levo
        crc_calc &= 0x7FFF;
        crc_calc ^= 0xA001;     // crc polinom = 0xA001
      }
      else
      {
        crc_calc >>= 1;  //shiftamo za en bit v levo
        crc_calc &= 0x7FFF;
      }
    }
  }

}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
