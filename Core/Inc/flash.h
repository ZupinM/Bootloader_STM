#ifndef FLASH_H
#define FLASH_H

#include <stdint.h>
#include "usart.h"
#ifdef APPLICATION
  #include "main.h"
#else
  //#include "bootloader LPC1549/main.h"
  //#define BOOTLOADER

#endif


#define FLASH_USER_SIZE 200     //size in floats
#define FLASH_USER_PAGE_1 250 //burned?
//#define FLASH_USER_PAGE_1 252
#define FLASH_ROW_SIZE 32  		// 32 Double Words
#define BANK1_WRITE_START_ADDR  ((uint32_t)0x08000000) //Main flash memory ; Currently used memory is aliased at 0x0000000
#define BANK1_WRITE_END_ADDR    BANK1_WRITE_START_ADDR + FLASH_BANK_SIZE //0x 0808 0000
//#define FLASH_USER_START_ADDR  (BANK1_WRITE_START_ADDR + FLASH_USER_PAGE_1*FLASH_PAGE_SIZE) //0x 0807 f000
#define FLASH_USER_SIZE_BYTES ( FLASH_USER_SIZE * sizeof(float) )

#define FLASH_ADDR_MAIN (BANK1_WRITE_END_ADDR - (FLASH_PAGE_SIZE*6)) //2-burned?
#define FLASH_ADDR_BACKUP (BANK1_WRITE_END_ADDR - (FLASH_PAGE_SIZE*5))
//#define FLASH_ADDR_MAIN (BANK1_WRITE_END_ADDR - (FLASH_PAGE_SIZE*4))
//#define FLASH_ADDR_BACKUP (BANK1_WRITE_END_ADDR - (FLASH_PAGE_SIZE*3))

#define FLASH_APP_START_ADDR 0x801f000
#define FLASH_BOOT_START_ADDR 0x8000000
#define FLASH_APP_SIZE_ 0x61000


#define BOOT_VERSION_ADDR         0x0801ef00
#define BOOT_HW_REV_ADDR          0x0801ef04
#define BOOT_DEVTYPE_ADDR         0x0801ef08
#define BOOT_APP_MINVERSION_ADDR  0x0801ef0C

#define BOOT_VERSION		*((unsigned int *)BOOT_VERSION_ADDR)
#define BOOT_HW_REV		*((unsigned int *)BOOT_HW_REV_ADDR)
#define BOOT_DEVTYPE		*((unsigned int *)BOOT_DEVTYPE_ADDR)
#define BOOT_APP_MINVERSION	*((unsigned int *)BOOT_APP_MINVERSION_ADDR)


extern unsigned int number_TX_bytes;
extern unsigned int tracker_status;			//status kondicije, v kateri je tracker (napake, halli...)
extern unsigned int tracker_status2;			//Status for motors 1 & 3 when motor_count > 3
extern unsigned int tracker_exstatus;
extern unsigned int tracker_exstatus2;
extern uint8_t slave_addr;
extern int baudrate;


#endif
