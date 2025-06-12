/*
 * adc_sensor.c
 *
 *  Created on: Jun 12, 2025
 *      Author: FeruMaga
 */



#include "adc_sensor.h"
#include "filter.h"
#include "main.h" // Incluir main.h para HAL_ADC_Start_DMA, etc.

CalibrationData_t gCalibrationData;

// ALTERAÇÃO: Variável estática para o buffer do ADC
static uint16_t adcBuffer[NUM_ADC_CHANNELS]; // ALTERAÇÃO: Buffer agora com o tamanho dos canais,
                                             // assumindo que o DMA preenche as últimas 8 amostras ou que
                                             // você está interessado apenas nas 8 posições iniciais se o DMA for circular.
                                             // Se ADC_BUFFER_SIZE (512) for realmente necessário (e.g., média de 64 amostras por canal),
                                             // a lógica de acesso e o tamanho do buffer aqui precisarão ser ajustados.
                                             // Para o exemplo, estamos assumindo 8 amostras diretas.

// ALTERAÇÃO: Variável estática para o handle do ADC
static ADC_HandleTypeDef* pAdcHandle;

// Variáveis de engenharia para armazenar os valores filtrados
static float tensao1Eng = 0.0f;
static float tensao2Eng = 0.0f;
static float corrente1Eng = 0.0f;
static float corrente2Eng = 0.0f;
static float temperatura1Eng = 0.0f;
static float temperatura2Eng = 0.0f;
static float massa1Eng = 0.0f;
static float massa2Eng = 0.0f;

static bool firstAdcRead = true;

void ADC_Sensor_Init(ADC_HandleTypeDef* hadc) {
    pAdcHandle = hadc;

   gCalibrationData.tensao1_offset = 2047.0f;
    gCalibrationData.tensao1_coeff = 0.1075f;
    gCalibrationData.tensao2_offset = 2047.0f;
    gCalibrationData.tensao2_coeff = 0.1075f;

    // Corrente
    gCalibrationData.corrente1_offset = 2047.0f;
    gCalibrationData.corrente1_coeff = 0.002442f;
    gCalibrationData.corrente2_offset = 2047.0f;
    gCalibrationData.corrente2_coeff = 0.002442f;

    // Temperatura
    gCalibrationData.temperatura1_offset = 2047.0f;
    gCalibrationData.temperatura1_coeff = 0.04884f;
    gCalibrationData.temperatura2_offset = 2047.0f;
    gCalibrationData.temperatura2_coeff = 0.04884f;

    // Massa
    gCalibrationData.massa1_offset = 2047.0f;
    gCalibrationData.massa1_coeff = 0.24420f;
    gCalibrationData.massa2_offset = 2047.0f;
    gCalibrationData.massa2_coeff = 0.24420f;

    firstAdcRead = true;
}

void ADC_Sensor_StartConversion(void) {
    HAL_ADC_Start_DMA(pAdcHandle, (uint32_t*)adcBuffer, NUM_ADC_CHANNELS);
}


void ADC_Sensor_HandleDMA_Complete(void) {
    float newTensao1 = ADC_Sensor_Convert_Tensao(adcBuffer[ADC_TENSAO1], gCalibrationData.tensao1_offset, gCalibrationData.tensao1_coeff);
    float newTensao2 = ADC_Sensor_Convert_Tensao(adcBuffer[ADC_TENSAO2], gCalibrationData.tensao2_offset, gCalibrationData.tensao2_coeff);
    float newCorrente1 = ADC_Sensor_Convert_Corrente(adcBuffer[ADC_CORRENTE1], gCalibrationData.corrente1_offset, gCalibrationData.corrente1_coeff);
    float newCorrente2 = ADC_Sensor_Convert_Corrente(adcBuffer[ADC_CORRENTE2], gCalibrationData.corrente2_offset, gCalibrationData.corrente2_coeff);
    float newTemperatura1 = ADC_Sensor_Convert_Temperatura(adcBuffer[ADC_TEMPERATURA1], gCalibrationData.temperatura1_offset, gCalibrationData.temperatura1_coeff);
    float newTemperatura2 = ADC_Sensor_Convert_Temperatura(adcBuffer[ADC_TEMPERATURA2], gCalibrationData.temperatura2_offset, gCalibrationData.temperatura2_coeff);
    float newMassa1 = ADC_Sensor_Convert_Massa(adcBuffer[ADC_MASSA1], gCalibrationData.massa1_offset, gCalibrationData.massa1_coeff);
    float newMassa2 = ADC_Sensor_Convert_Massa(adcBuffer[ADC_MASSA2], gCalibrationData.massa2_offset, gCalibrationData.massa2_coeff);

    if (firstAdcRead) {
        tensao1Eng = newTensao1;
        tensao2Eng = newTensao2;
        corrente1Eng = newCorrente1;
        corrente2Eng = newCorrente2;
        temperatura1Eng = newTemperatura1;
        temperatura2Eng = newTemperatura2;
        massa1Eng = newMassa1;
        massa2Eng = newMassa2;
        firstAdcRead = false;
    } else {
        tensao1Eng = Filter_Lowpass(tensao1Eng, newTensao1, FILTER_ALPHA_TENSAO);
        tensao2Eng = Filter_Lowpass(tensao2Eng, newTensao2, FILTER_ALPHA_TENSAO);
        corrente1Eng = Filter_Lowpass(corrente1Eng, newCorrente1, FILTER_ALPHA_CORRENTE);
        corrente2Eng = Filter_Lowpass(corrente2Eng, newCorrente2, FILTER_ALPHA_CORRENTE);
        temperatura1Eng = Filter_Lowpass(temperatura1Eng, newTemperatura1, FILTER_ALPHA_TEMPERATURA);
        temperatura2Eng = Filter_Lowpass(temperatura2Eng, newTemperatura2, FILTER_ALPHA_TEMPERATURA);
        massa1Eng = Filter_Lowpass(massa1Eng, newMassa1, FILTER_ALPHA_MASSA);
        massa2Eng = Filter_Lowpass(massa2Eng, newMassa2, FILTER_ALPHA_MASSA);
    }

    HAL_ADC_Start_DMA(pAdcHandle, (uint32_t*)adcBuffer, NUM_ADC_CHANNELS);
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