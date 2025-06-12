/*
 * adc_sensor.c
 *
 *  Created on: Jun 12, 2025
 *      Author: FeruMaga
 */


#include "adc_sensor.h"
#include "filter.h"
#include "main.h"


CalibrationData_t gCalibrationData;


static uint16_t adcBuffer[ADC_BUFFER_SIZE];
static ADC_HandleTypeDef* pAdcHandle

static float tensao1Eng = 0.0f;
static float tensao2Eng = 0.0f;
static float corrente1Eng = 0.0f;
static float corrente2Eng = 0.0f;
static float temperatura1Eng = 0.0f;
static float temperatura2Eng = 0.0f;
static float massa1Eng = 0.0f;
static float massa2Eng = 0.0f;

void ADC_Sensor_Init(ADC_HandleTypeDef* hadc) {
    pAdcHandle = hadc;

    gCalibrationData.tensao1_offset = 2047.0f;
    gCalibrationData.tensao1_coeff = 0.1075f;
    gCalibrationData.tensao2_offset = 2047.0f;
    gCalibrationData.tensao2_coeff = 0.1075f;

    gCalibrationData.corrente1_offset = 2047.0f;
    gCalibrationData.corrente1_coeff = 0.002442f;
    gCalibrationData.corrente2_offset = 2047.0f;
    gCalibrationData.corrente2_coeff = 0.002442f;

    gCalibrationData.temperatura1_offset = 2047.0f;
    gCalibrationData.temperatura1_coeff = 0.04884f;
    gCalibrationData.temperatura2_offset = 2047.0f;
    gCalibrationData.temperatura2_coeff = 0.04884f;

    gCalibrationData.massa1_offset = 2047.0f;
    gCalibrationData.massa1_coeff = 0.24420f;
    gCalibrationData.massa2_offset = 2047.0f;
    gCalibrationData.massa2_coeff = 0.24420f;
}

void ADC_Sensor_StartConversion(void) {
    HAL_ADC_Start_DMA(pAdcHandle, (uint32_t*)adcBuffer, ADC_BUFFER_SIZE);
}

void ADC_Sensor_HandleDMA_Complete(void) {
    tensao1Eng = Filter_Apply(tensao1Eng, ADC_Sensor_Convert_Tensao(adcBuffer[ADC_TENSAO1], gCalibrationData.tensao1_offset, gCalibrationData.tensao1_coeff), FILTER_ALPHA_TENSAO);
    tensao2Eng = Filter_Apply(tensao2Eng, ADC_Sensor_Convert_Tensao(adcBuffer[ADC_TENSAO2], gCalibrationData.tensao2_offset, gCalibrationData.tensao2_coeff), FILTER_ALPHA_TENSAO);

    corrente1Eng = Filter_Apply(corrente1Eng, ADC_Sensor_Convert_Corrente(adcBuffer[ADC_CORRENTE1], gCalibrationData.corrente1_offset, gCalibrationData.corrente1_coeff), FILTER_ALPHA_CORRENTE);
    corrente2Eng = Filter_Apply(corrente2Eng, ADC_Sensor_Convert_Corrente(adcBuffer[ADC_CORRENTE2], gCalibrationData.corrente2_offset, gCalibrationData.corrente2_coeff), FILTER_ALPHA_CORRENTE);

    temperatura1Eng = Filter_Apply(temperatura1Eng, ADC_Sensor_Convert_Temperatura(adcBuffer[ADC_TEMPERATURA1], gCalibrationData.temperatura1_offset, gCalibrationData.temperatura1_coeff), FILTER_ALPHA_TEMPERATURA);
    temperatura2Eng = Filter_Apply(temperatura2Eng, ADC_Sensor_Convert_Temperatura(adcBuffer[ADC_TEMPERATURA2], gCalibrationData.temperatura2_offset, gCalibrationData.temperatura2_coeff), FILTER_ALPHA_TEMPERATURA);

    massa1Eng = Filter_Apply(massa1Eng, ADC_Sensor_Convert_Massa(adcBuffer[ADC_MASSA1], gCalibrationData.massa1_offset, gCalibrationData.massa1_coeff), FILTER_ALPHA_MASSA);
    massa2Eng = Filter_Apply(massa2Eng, ADC_Sensor_Convert_Massa(adcBuffer[ADC_MASSA2], gCalibrationData.massa2_offset, gCalibrationData.massa2_coeff), FILTER_ALPHA_MASSA);

    HAL_ADC_Start_DMA(pAdcHandle, (uint32_t*)adcBuffer, ADC_BUFFER_SIZE);
}

float ADC_Sensor_Convert_Tensao(uint16_t adc_raw, float offset, float coeff) {
    return ((float)adc_raw - offset) * coeff;
}

float ADC_Sensor_Convert_Corrente(uint16_t adc_raw, float offset, float coeff) {
    return ((float)adc_raw - offset) * coeff;
}

float ADC_Sensor_Convert_Temperatura(uint16_t adc_raw, float offset, float coeff) {
    return ((float)adc_raw - offset) * coeff;
}

float ADC_Sensor_Convert_Massa(uint16_t adc_raw, float offset, float coeff) {
    return ((float)adc_raw - offset) * coeff;
}

float ADC_Sensor_GetTensao1(void) { return tensao1Eng; }
float ADC_Sensor_GetTensao2(void) { return tensao2Eng; }
float ADC_Sensor_GetCorrente1(void) { return corrente1Eng; }
float ADC_Sensor_GetCorrente2(void) { return corrente2Eng; }
float ADC_Sensor_GetTemperatura1(void) { return temperatura1Eng; }
float ADC_Sensor_GetTemperatura2(void) { return temperatura2Eng; }
float ADC_Sensor_GetMassa1(void) { return massa1Eng; }
float ADC_Sensor_GetMassa2(void) { return massa2Eng; }
