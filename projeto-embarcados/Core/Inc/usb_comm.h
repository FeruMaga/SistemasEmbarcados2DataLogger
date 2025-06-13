/*
 * usb.h
 *
 *  Created on: Jun 12, 2025
 *      Author: FeruMaga
 */

#ifndef INC_USB_COMM_H_
#define INC_USB_COMM_H_

// Define o tamanho do buffer de recepção USB.
#define USB_RX_BUFFER_SIZE 256

void USB_Init(PCD_HandleTypeDef* hpcd);
// Inicializa o periférico USB Device (PCD) do STM32.

void USB_Receive(uint8_t* Buf, uint32_t Len);
// Função callback chamada quando dados são recebidos via USB.
// Ela copia e processa os dados de calibração.

bool USB_ProcessCalibrationCommand(uint8_t* data, uint32_t len, CalibrationData_t* calData);
// Analisa uma string de comando para extrair e aplicar parâmetros de calibração.

void USB_Send(char* str);
// Envia uma string de texto para o host via USB.



#endif /* INC_USB_COMM_H_ */
