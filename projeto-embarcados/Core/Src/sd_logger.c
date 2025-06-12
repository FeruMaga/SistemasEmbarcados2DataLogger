/*
 * sd_logger.c
 *
 *  Created on: Jun 12, 2025
 *      Author: FeruMaga
 */



#include "sd_logger.h"

FATFS fs;
FIL file;
UINT bytesWritten;

void SD_Logger_Init(SPI_HandleTypeDef* hspi) {
    if (FATFS_LinkDriver(&USER_Driver, USERPath) != 0) {
        USB_Send("Erro ao linkar driver FATFS\r\n");
        Error_Handler();
    }

    if (f_mount(&fs, "", 1) != FR_OK) {
        USB_Send("Erro ao montar SD\r\n");
        Error_Handler();
    }
}

void SD_Logger_WriteData(char* data) {
    if (f_open(&file, LOG_FILE_NAME, FA_OPEN_APPEND | FA_WRITE) == FR_OK) {
        f_write(&file, data, strlen(data), &bytesWritten);
        f_close(&file);
    } else {
        USB_Send("Erro ao abrir arquivo de log no SD\r\n");
    }
}