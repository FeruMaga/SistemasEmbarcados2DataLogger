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


typedef enum {
    SD_OK       = 0,  // Status: Operação de cartão SD bem-sucedida.
    SD_OPEN_ERR = 1,  // Status: Erro ao abrir ou criar o arquivo no cartão SD.
    SD_WRITE_ERR = 2  // Status: Erro ao escrever dados no arquivo do cartão SD.
} SD_Status_t; // Definição de um tipo enumerado para representar o status das operações do cartão SD.

// Prototipo da função que escreve dados no cartão SD.
SD_Status_t SD_Logger_WriteData(char* data);

// Adicione uma variável global (extern) para o estado do SD
extern bool g_sd_card_full;

// Define uma macro para o nome do arquivo de log no cartão SD.
#define LOG_FILE_NAME "log.txt"

// Prototipo da função de inicialização do módulo de log no cartão SD.
void SD_Logger_Init(SPI_HandleTypeDef* hspi);

// Prototipo original da função de escrita de dados no cartão SD.
void SD_Logger_WriteData(char* data);

#endif /* INC_SD_LOGGER_H_ */
