/*
 * flash.c
 *
 *  Created on: Nov 18, 2021
 *      Author: development
 */
#include "../../../Micro/Core/Inc/flash.h"
#include "stm32l4xx_hal.h"

static FLASH_EraseInitTypeDef EraseInitStruct;
uint32_t Address = 0, PAGEError = 0;
extern const unsigned char _acMicro_bootloader[0xf800];
uint32_t flashError;

uint8_t eraseBoot(void) {

	  /* Unlock the Flash to enable the flash control register access *************/
	  HAL_FLASH_Unlock();

	  /* Clear OPTVERR bit set on virgin samples */
	    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

	  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	  EraseInitStruct.Banks       = FLASH_BANK_1;
	  EraseInitStruct.Page        = 0;
	  EraseInitStruct.NbPages     = (FLASH_APP_START_ADDR&0xfffff) / FLASH_PAGE_SIZE;

	  if( HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK){
		  return UPGRADE_ERROR;
	  }
	  return UPGRADE_OK;
}

void eraseApp(void) {

	  /* Unlock the Flash to enable the flash control register access *************/
	  HAL_FLASH_Unlock();

	  /* Clear OPTVERR bit set on virgin samples */
	    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

	  EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
	  EraseInitStruct.Banks       = FLASH_BANK_1;
	  EraseInitStruct.Page        = (FLASH_APP_START_ADDR&0xfffff) / FLASH_PAGE_SIZE; //page_size: 0x800 (STM),  sector_size: 0x1000 (LPC)
	  EraseInitStruct.NbPages     = FLASH_APP_SIZE_ACTUAL / FLASH_PAGE_SIZE;

	  HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError);
	  return;
}


uint8_t flash_write_boot(void) {

  /* Unlock the Flash to enable the flash control register access *************/
  HAL_FLASH_Unlock();

  /* Clear OPTVERR bit set on virgin samples */
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_ALL_ERRORS);

   uint64_t* flash_backup_dw = (uint64_t*)(_acMicro_bootloader); //Double word
   unsigned int Address = 0;

  //Write flash row by row
  while (Address < (FLASH_APP_START_ADDR&0xfffff) )
  {
		if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, Address, *(flash_backup_dw++)) == HAL_OK)
		{
		  Address = Address + sizeof(uint64_t);
		}
	   else
		{
		  /* Error occurred while writing data in Flash memory.*/
		  flashError = HAL_FLASH_GetError();
		  return UPGRADE_ERROR;
		}
  }
  return UPGRADE_OK;
}

int flash_verify(void)
{
  unsigned int  addr;
  for(addr = 0 ;addr < (FLASH_APP_START_ADDR & 0xfffff); addr++){
	  if( *((unsigned char *)addr) != _acMicro_bootloader[addr]){
		return UPGRADE_ERROR;
	  }
  }

 return UPGRADE_OK;
}
