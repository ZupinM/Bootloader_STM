/*
 * usb.h
 *
 *  Created on: Nov 16, 2021
 *      Author: development
 */

#ifndef INC_USB_H_
#define INC_USB_H_

#include <stdint.h>
#include <string.h>
#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "flash.h"


#define USB_RESPONSE_OK 0x11
#define USB_RESPONSE_ERROR 0x00
#define ERROR_CRC 0x22

void USB_display(void);
void USB_write(void);
void USART_To_USB_Send_Data(uint8_t* ascii_string, uint32_t count_in);


#endif /* INC_USB_H_ */
