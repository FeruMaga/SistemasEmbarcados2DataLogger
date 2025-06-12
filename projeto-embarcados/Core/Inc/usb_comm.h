/*
 * usb.h
 *
 *  Created on: Jun 12, 2025
 *      Author: FeruMaga
 */

#ifndef INC_USB_COMM_H_
#define INC_USB_COMM_H_


#include "main.h"
#include "adc_sensor.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#define USB_RX_BUFFER_SIZE 256

void USB_Receive(uint8_t* Buf, uint32_t Len);
bool USB_ProcessCalibrationCommand(uint8_t* data, uint32_t len, CalibrationData_t* calData);
void USB_Send(char* str);




#endif /* INC_USB_COMM_H_ */
