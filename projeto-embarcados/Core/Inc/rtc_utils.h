/*
 * rtc_utils.h
 *
 *  Created on: Jun 1, 2025
 *      Author: FeruMaga
 */

#ifndef INC_RTC_UTILS_H_
#define INC_RTC_UTILS_H_


#include "main.h"
#include <stdint.h>
#include <stdio.h>

// Obtém a hora e a data atuais do RTC. Armazena os valores nas estruturas 'sTime' e 'sDate'.
void RTC_GetDateTime(RTC_HandleTypeDef* hrtc, RTC_TimeTypeDef* sTime, RTC_DateTypeDef* sDate);

// Define uma nova hora e data para o RTC. Permite configurar o RTC com um valor específico.
void RTC_SetDateTime(RTC_HandleTypeDef* hrtc, uint8_t hours, uint8_t minutes, uint8_t seconds, uint8_t day, uint8_t month, uint16_t year);

// Formata a hora e a data (obtidas das estruturas 'sTime' e 'sDate') em uma string de texto legível.
void RTC_FormatDateTime(RTC_TimeTypeDef* sTime, RTC_DateTypeDef* sDate, char* buffer, size_t bufferSize);


#endif /* INC_RTC_UTILS_H_ */
