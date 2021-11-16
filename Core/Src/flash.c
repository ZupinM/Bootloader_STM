#include "flash.h"
//#include "../../Drivers/STM32L4xx_HAL_Driver/Inc/stm32l4xx_hal_flash.h"
#include "stm32l4xx_hal.h"
#include <string.h>

#define ACK_OK      0x00
#define ACK_ERROR   0xFF


/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;

uint32_t Address = 0, PAGEError = 0;
extern unsigned int SN[4];
extern unsigned int crypdedRx;

/* IAP - flash write (parameters backup) */
unsigned int command[5]; 	//IAP - spremenljivki
unsigned int result[5];
float        flash_backup[FLASH_USER_SIZE]; 	//polje spremenljivk, ki se zapisejo v flash
unsigned int FlashWriteCounter;
unsigned int crc_flash;
float sys_vars[256];
//int *lflags_p = (int*)&lflags;

char writeBuffer[BUFSIZE];
unsigned int addressToWrite;
unsigned int sizeFirst;
unsigned int sizeWrite;


void read_SN(){

  // read unique ID via IAP
  SN[0] = HAL_GetUIDw0();
  SN[1] = HAL_GetUIDw1();
  SN[2] = HAL_GetUIDw2();
  SN[3] = 0xaaaabbbb;
}

/***********************************************************
  FLASH READ/WRITE

************************************************************/

uint32_t flashError;

/***********************************************************
  BOOTLOADER COMMANDS

************************************************************/
void flash_read_boot (unsigned int read_address, unsigned int size) {

		unsigned int 	j=2;			//odgovor: ID, CMD, DATA....
    unsigned char *addr = (unsigned char *)(read_address);

    do {
		UARTBuffer0[j++]=*addr++;
    }
    while (--size);
	number_TX_bytes=(j-1);
}

void ModBus_SendInt(unsigned int val) //sent one int
{
 UARTBuffer0[2]=val>>24;
 UARTBuffer0[3]=val>>16;
 UARTBuffer0[4]=val>>8;
 UARTBuffer0[5]=val;
 number_TX_bytes =6;
}

unsigned int Address_old;
void flash_erase(unsigned int start_sector, unsigned int stop_sector) {

	  int start_page = start_sector * 2; //page_size: 0x800 (STM),  sector_size: 0x1000 (LPC)

	  /* Unlock the Flash to enable the flash control register access *************/
	  HAL_FLASH_Unlock();

	  /* Clear OPTVERR bit set on virgin samples */
	    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

	  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	  EraseInitStruct.Banks       = FLASH_BANK_1;
	  EraseInitStruct.Page        = start_page;
	  EraseInitStruct.NbPages     = (stop_sector * 2) - start_page;
	  Address_old = 0;

	  if( HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK){
	     tracker_status |= SYS_PARAM_FLASH_ERR;
	     flashError = HAL_FLASH_GetError();
	     UARTBuffer0[2] = ACK_ERROR;  		//ACK odgovor
	  }else{
		  UARTBuffer0[2] = ACK_OK;
	  }
	  return;
}


void eraseApp() {

	  /* Unlock the Flash to enable the flash control register access *************/
	  HAL_FLASH_Unlock();

	  /* Clear OPTVERR bit set on virgin samples */
	    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

	  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	  EraseInitStruct.Banks       = FLASH_BANK_1;
	  EraseInitStruct.Page        = FLASH_APP_START_ADDR / FLASH_PAGE_SIZE; //page_size: 0x800 (STM),  sector_size: 0x1000 (LPC)
	  EraseInitStruct.NbPages     = FLASH_APP_SIZE_ / FLASH_PAGE_SIZE;

	  if( HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK){
	     tracker_status |= SYS_PARAM_FLASH_ERR;
	     flashError = HAL_FLASH_GetError();
	     UARTBuffer0[2] = ACK_ERROR;  		//ACK odgovor
	  }else{
		  UARTBuffer0[2] = ACK_OK;
	  }
	  return;
}

void flash_write_boot(unsigned int write_address, unsigned int size) {


	if (Address_old >= write_address && write_address != FLASH_APP_VERSION_ADDR){ // Before retrying flash write, erase sector!
		//uint32_t start_sector = (write_address/FLASH_PAGE_SIZE) /2;
		//flash_erase(start_sector, start_sector+1 );
		Address_old = write_address;
		return;
	}
	Address_old = write_address;

    if(crypdedRx == RX_MODE_NORMAL) { // uncrypted mode - crypted file
      if (decryptData ((char *)&UARTBuffer0[8], size) != 0){
    	  UARTBuffer0[2] = ACK_ERROR;
    	  return;
      }
      else {
        if (size < 0x100) { // complete last write sector with 0xFF
          memset((char *)&UARTBuffer0[size + 8], 0xFF, 0x108 - size);
        }
      }
    }

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  /* Clear OPTVERR bit set on virgin samples */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

   uint64_t* flash_backup_dw = (uint64_t*)(UARTBuffer0+8); //Double word
   unsigned int Address = write_address;

  //Write flash row by row
  while (Address < (write_address + size) )
  {
	if(*(flash_backup_dw) != 0xffffffffffffffff){ // flash written by 0xff seems to be unable to over-writen (ECCR bits?)
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, *(flash_backup_dw++)) == HAL_OK)
		{
		  Address = Address + sizeof(uint64_t);
		}
	   else
		{
		  /* Error occurred while writing data in Flash memory.*/
		  tracker_status |= SYS_PARAM_FLASH_ERR;
		  flashError = HAL_FLASH_GetError();
		  UARTBuffer0[2] = ACK_ERROR;
		  Error_Handler();
		}
	}else{
		flash_backup_dw++;
		Address = Address + sizeof(uint64_t);
	}
  }

  if(memcmp((char*)write_address, (char *)&UARTBuffer0[8], size))
    UARTBuffer0[2] = ACK_ERROR;
  else
    UARTBuffer0[2] = ACK_OK;
  return;

}

/*void writeApp(unsigned int address, unsigned int size) {
  unsigned int write_address = sys_defs.FLASH_APP_START_ADDRESS + address;
  unsigned int sector = write_address / 0x1000;

  __disable_irq();

  //prepare sectors
  command[0] = 50;                                     // ukaz "prepare"
  command[1] = sector;                                 // zacetni sektor
  command[2] = sector;                                 // koncni sektor
  iap_entry (command, result);
  while (result[0])
  goto iap_error;

  //write
  command[0] = 51;                                    // ukaz "write"


  if(crypdedRx == RX_MODE_NORMAL) { // uncrypted mode - crypted file
    if (decryptData ((char *)&UARTBuffer0[8], size) != 0)
      goto iap_error;
    else {
      if (size < 0x100) { // complete last write sector with 0xFF
        memset((char *)&UARTBuffer0[size + 8], 0xFF, 0x108 - size);
        size = 0x100;
      }
    }
  }

  command[1] = write_address;                       // address
  command[3] = size;                                // dovoljene vrednosti 256, 512, 1024, (4096)

  command[2] = (unsigned int)&UARTBuffer[8];
  command[4] = 48000;                                   // 48.000kHz main clock
  iap_entry (command, result);
  while (result[0]) goto iap_error;

  __enable_irq();

  if(memcmp((char*)write_address, (char *)&UARTBuffer[8], size))
    UARTBuffer[2] = ACK_ERROR;
  else
    UARTBuffer[2] = ACK_OK;
  return;

iap_error:                                          // napaka pri zapisu v flash
  UARTBuffer[2] = ACK_ERROR;                        // ACK odgovor
  __enable_irq();
  return;
}*/

void flash_checksum (unsigned int start_address, unsigned int size) {

  unsigned char *addr = (unsigned char *)(start_address);
  unsigned short int flash_checksum = 0;

  do {
    flash_checksum += *addr++;  	//0x 0000 0000 checksum
    flash_checksum &= 0xFFFF;
  }
  while (--size);

  UARTBuffer0[2] = flash_checksum&0xFF;
  UARTBuffer0[3] = flash_checksum/0x100;
  number_TX_bytes=4;

}

void read_SysVars(unsigned int addr)
{
  memcpy((char *)&sys_vars,(char*)addr, sizeof(sys_vars));
  slave_addr = (unsigned int)sys_vars[0];
  LoRa_id = (unsigned int)sys_vars[96];
  module.channel = (uint8_t)sys_vars[64];
  module.power = (uint8_t)sys_vars[65];
  module.spFactor = (uint8_t)sys_vars[66];
  module.LoRa_BW = (uint8_t)sys_vars[67];
  //baudrate = sys_vars[151];
  transceiver = sys_vars[68];
}

int verify_SysVars(unsigned int addr)
{
  int i;

  for(i = 0 ; i < (sizeof(sys_vars) / sizeof(float)) ; i++)
    if( *((float*)(addr+(i*sizeof(float)))) != sys_vars[i])
      return 1;

  return 0;
}

int write_SysVars(unsigned int addr)
{
	sys_vars[0] = slave_addr;                    // address
	uint32_t SizeToWrite = sizeof(sys_vars);

	/* Unlock the Flash to enable the flash control register access *************/
	HAL_FLASH_Unlock();

	/* Clear OPTVERR bit set on virgin samples */
	  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

	  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	  EraseInitStruct.Banks       = FLASH_BANK_1;
	  EraseInitStruct.Page        = addr / FLASH_PAGE_SIZE;
	  EraseInitStruct.NbPages     = 1;

	  if( HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK){
	     tracker_status |= SYS_PARAM_FLASH_ERR;
	     flashError = HAL_FLASH_GetError();
	  }

	uint64_t* flash_backup_dw = (uint64_t*)sys_vars; //Double word

	//Write flash row by row
	while (Address < (addr + SizeToWrite) )
	{
	  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, *(flash_backup_dw++)) == HAL_OK)
	  {
		Address = Address + sizeof(uint64_t);
	  }
	 else
	  {
		/* Error occurred while writing data in Flash memory.*/
		tracker_status |= SYS_PARAM_FLASH_ERR;
		flashError = HAL_FLASH_GetError();
		UARTBuffer0[2] = ACK_ERROR;
		//Error_Handler();
		return 1;
	  }
	}
	return 0;
}


