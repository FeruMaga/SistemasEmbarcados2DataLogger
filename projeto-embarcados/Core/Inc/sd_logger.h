/*
 * sd_logger.h
 *
 *  Created on: Jun 12, 2025
 *      Author: FeruMaga
 */

#ifndef INC_SD_LOGGER_H_
#define INC_SD_LOGGER_H_


#include "main.h"
#include "fatfs.h"
#include "string.h"
#include "usb_comm.h" 

#define LOG_FILE_NAME "log.txt"

void SD_Logger_Init(SPI_HandleTypeDef* hspi);
void SD_Logger_WriteData(char* data);



#endif /* INC_SD_LOGGER_H_ */
