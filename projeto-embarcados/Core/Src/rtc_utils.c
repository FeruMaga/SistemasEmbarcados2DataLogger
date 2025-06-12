/*
 * rtc_utils.c
 *
 *  Created on: Jun 12, 2025
 *      Author: FeruMaga
 */


#include "rtc_utils.h"

void RTC_Init(RTC_HandleTypeDef* hrtc) {

}

void RTC_GetDateTime(RTC_HandleTypeDef* hrtc, RTC_TimeTypeDef* sTime, RTC_DateTypeDef* sDate) {
    HAL_RTC_GetTime(hrtc, sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(hrtc, sDate, RTC_FORMAT_BIN);
}

void RTC_SetDateTime(RTC_HandleTypeDef* hrtc, uint8_t hours, uint8_t minutes, uint8_t seconds, uint8_t day, uint8_t month, uint16_t year) {
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    sTime.Hours = hours;
    sTime.Minutes = minutes;
    sTime.Seconds = seconds;
    sDate.Date = day;
    sDate.Month = month;
    sDate.Year = year - 2000;

    HAL_RTC_SetTime(hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_SetDate(hrtc, &sDate, RTC_FORMAT_BIN);
}

void RTC_FormatDateTime(RTC_TimeTypeDef* sTime, RTC_DateTypeDef* sDate, char* buffer, size_t bufferSize) {
    snprintf(buffer, bufferSize, "%02d/%02d/20%02d %02d:%02d:%02d",
             sDate->Date, sDate->Month, sDate->Year,
             sTime->Hours, sTime->Minutes, sTime->Seconds);
}
