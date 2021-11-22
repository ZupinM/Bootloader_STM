/*
 * usb.c
 *
 *  Created on: Nov 16, 2021
 *      Author: development
 */

#include "usb.h"
#include "RTT/SEGGER_RTT.h"

uint8_t bufGlbFirst[256];
uint8_t USB_rxBuff[100];
uint16_t USB_rxCount;
uint8_t USB_upgradeBuff[256];
uint8_t StartUpgrade = 0;
uint8_t receviedPartIndex = 0;
uint16_t LastPacketTimeout;
unsigned int addrToWrite = 0;
uint8_t send_back=USB_RESPONSE_OK;
extern unsigned int wait_appl_cnt;
extern const  system_defs_t sys_defs;
int enableUpgrade = 0;


void USART_To_USB_Send_Data(uint8_t* ascii_string, uint32_t count_in){

	if(count_in > 255)
		return;
	CDC_Transmit_FS(ascii_string, count_in);

}

/***********************************************************
  USB WRITE VALUES
************************************************************/
void USB_write(void) {

	if (USB_rxCount<255){	//else New data received
		if(USB_rxCount > 0 && enableUpgrade){
			if(LastPacketTimeout++ < 200){ //Last upgrade packed received (smaller than 255)
				return;
			}
		}else{
			return; // Packets smaller than upgrade packets rejected when not in upgrade mode
		}
	}
    if(addrToWrite == 0) {
      addrToWrite = sys_defs.FLASH_APP_START_ADDRESS;
      memcpy(bufGlbFirst, USB_upgradeBuff, 0x100);
    }
    else{
      addrToWrite += 0x100;
    }
    send_back = USB_RESPONSE_OK;
    if(addrToWrite == sys_defs.FLASH_APP_START_ADDRESS + 0x100) {
      // checking version type MICRO
      char bufGlbVCheck[16];
      memcpy(bufGlbVCheck, &USB_upgradeBuff[FLASH_APP_VERSION_ADDR & 0xff], 0x10);
      if (decryptData (bufGlbVCheck, 0x10) == 0) {
        int pVer = bufGlbVCheck[0] + 0x100 * bufGlbVCheck[1];
        if(pVer / 1000 == sys_defs.DEV_TYPE){
        	enableUpgrade = 1;
            eraseApp();	// erasing and writing 1st sector
            if(flash_write_upgrade(sys_defs.FLASH_APP_START_ADDRESS, bufGlbFirst, 0x100)){
            	send_back = USB_RESPONSE_ERROR;
            }
        }else{
        	send_back = USB_RESPONSE_ERROR;
        }
      }else{
    	  send_back = USB_RESPONSE_ERROR;
      }
    }
    // writing 2nd and other sectors
    if(addrToWrite > sys_defs.FLASH_APP_START_ADDRESS) {
      if(enableUpgrade){
    	  if(flash_write_upgrade(addrToWrite, USB_upgradeBuff, 0x100)){
    		  send_back = USB_RESPONSE_ERROR;
    	  }
      }
    }
    USB_rxCount = 0;
    wait_appl_cnt = WAIT_TO_APPL;
    USART_To_USB_Send_Data(&send_back, 1);

    if(send_back == USB_RESPONSE_ERROR){
    	addrToWrite = 0;
    	enableUpgrade = 0;
    }

}
