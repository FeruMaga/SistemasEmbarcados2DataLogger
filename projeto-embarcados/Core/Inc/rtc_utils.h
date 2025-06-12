/*
 * rtc_utils.h
 *
 *  Created on: Jun 12, 2025
 *      Author: extre
 */

#ifndef INC_RTC_UTILS_H_
#define INC_RTC_UTILS_H_


#include "main.h"
#include <stdint.h>
#include <stdio.h>

void RTC_Init(RTC_HandleTypeDef* hrtc);
void RTC_GetDateTime(RTC_HandleTypeDef* hrtc, RTC_TimeTypeDef* sTime, RTC_DateTypeDef* sDate);
void RTC_SetDateTime(RTC_HandleTypeDef* hrtc, uint8_t hours, uint8_t minutes, uint8_t seconds, uint8_t day, uint8_t month, uint16_t year);
void RTC_FormatDateTime(RTC_TimeTypeDef* sTime, RTC_DateTypeDef* sDate, char* buffer, size_t bufferSize);


#endif /* INC_RTC_UTILS_H_ */
