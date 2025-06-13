/*
 * adc_sensor.c
 *
 *  Created on: Jun 12, 2025
 *      Author: FeruMaga
 */



#include "adc_sensor.h"
#include "filter.h"
#include "main.h"
// Definição da variável global de dados de calibração.
CalibrationData_t gCalibrationData;

static uint16_t adcBuffer[NUM_ADC_CHANNELS];
static ADC_HandleTypeDef* pAdcHandle;

// Variáveis estáticas para armazenar os valores dos sensores em unidades de engenharia.
static float tensao1Eng = 0.0f;
static float tensao2Eng = 0.0f;
static float corrente1Eng = 0.0f;
static float corrente2Eng = 0.0f;
static float temperatura1Eng = 0.0f;
static float temperatura2Eng = 0.0f;
static float massa1Eng = 0.0f;
static float massa2Eng = 0.0f;

// --- Instâncias de estado do filtro Butterworth para CADA SENSOR ---
static Butter2State_t tensao1FilterState;
static Butter2State_t tensao2FilterState;
static Butter2State_t corrente1FilterState;
static Butter2State_t corrente2FilterState;
static Butter2State_t temperatura1FilterState;
static Butter2State_t temperatura2FilterState;
static Butter2State_t massa1FilterState;
static Butter2State_t massa2FilterState;

// --- Instâncias de coeficientes de filtro para CADA SENSOR ---
static const FilterCoefficients_t tensaoFilterCoeffs = {
    .B0 = 1.95669352e-03, .B1 = 3.91338703e-03, .B2 = 1.95669352e-03,
    .A1 = -1.90562624e+00, .A2 = 9.13453273e-01
};
static const FilterCoefficients_t tensao2FilterCoeffs = {
    .B0 = 1.95669352e-03, .B1 = 3.91338703e-03, .B2 = 1.95669352e-03,
    .A1 = -1.90562624e+00, .A2 = 9.13453273e-01
};

static const FilterCoefficients_t correnteFilterCoeffs = {
    .B0 = 1.95669352e-03, .B1 = 3.91338703e-03, .B2 = 1.95669352e-03,
    .A1 = -1.90562624e+00, .A2 = 9.13453273e-01
};
static const FilterCoefficients_t corrente2FilterCoeffs = {
    .B0 = 1.95669352e-03, .B1 = 3.91338703e-03, .B2 = 1.95669352e-03,
    .A1 = -1.90562624e+00, .A2 = 9.13453273e-01
};

static const FilterCoefficients_t temperaturaFilterCoeffs = {
    .B0 = 1.57912440e-07, .B1 = 3.15824879e-07, .B2 = 1.57912440e-07,
    .A1 = -1.99842095e+00, .A2 = 9.98421045e-01
};
static const FilterCoefficients_t temperatura2FilterCoeffs = {
    .B0 = 1.57912440e-07, .B1 = 3.15824879e-07, .B2 = 1.57912440e-07,
    .A1 = -1.99842095e+00, .A2 = 9.98421045e-01
};

static const FilterCoefficients_t massaFilterCoeffs = {
    .B0 = 3.94553258e-06, .B1 = 7.89106516e-06, .B2 = 3.94553258e-06,
    .A1 = -1.98418047e+00, .A2 = 9.84206979e-01
};
static const FilterCoefficients_t massa2FilterCoeffs = {
    .B0 = 3.94553258e-06, .B1 = 7.89106516e-06, .B2 = 3.94553258e-06,
    .A1 = -1.98418047e+00, .A2 = 9.84206979e-01
};

static bool firstAdcRead = true;

// Inicializa o módulo ADC_Sensor.
void ADC_Sensor_Init(ADC_HandleTypeDef* hadc) {
    pAdcHandle = hadc;

    // --- Configurações de calibração (permanecem as mesmas) ---
    gCalibrationData.tensao1_offset = 2047.0f; gCalibrationData.tensao1_coeff = 0.1075f;
    gCalibrationData.tensao2_offset = 2047.0f; gCalibrationData.tensao2_coeff = 0.1075f;
    gCalibrationData.corrente1_offset = 2047.0f; gCalibrationData.corrente1_coeff = 0.002442f;
    gCalibrationData.corrente2_offset = 2047.0f; gCalibrationData.corrente2_coeff = 0.002442f;
    gCalibrationData.temperatura1_offset = 2047.0f; gCalibrationData.temperatura1_coeff = 0.04884f;
    gCalibrationData.temperatura2_offset = 2047.0f; gCalibrationData.temperatura2_coeff = 0.04884f;
    gCalibrationData.massa1_offset = 2047.0f; gCalibrationData.massa1_coeff = 0.24420f;
    gCalibrationData.massa2_offset = 2047.0f; gCalibrationData.massa2_coeff = 0.24420f;
    gCalibrationData.isCalibrated = false;

    // --- Inicializa os estados dos filtros Butterworth para TODOS os sensores ---
    butter2_init(&tensao1FilterState);
    butter2_init(&tensao2FilterState);
    butter2_init(&corrente1FilterState);
    butter2_init(&corrente2FilterState);
    butter2_init(&temperatura1FilterState);
    butter2_init(&temperatura2FilterState);
    butter2_init(&massa1FilterState);
    butter2_init(&massa2FilterState);

    firstAdcRead = true;
}

// Inicia a conversão do ADC usando DMA.
void ADC_Sensor_StartConversion(void) {
    HAL_ADC_Start_DMA(pAdcHandle, (uint32_t*)adcBuffer, NUM_ADC_CHANNELS);
}

// Função de callback (chamada de retorno) invocada quando a transferência DMA do ADC é concluída.
void ADC_Sensor_HandleDMA_Complete(void) {
    // Converte as leituras brutas do ADC para unidades de engenharia.
    float newTensao1 = ADC_Sensor_Convert_Tensao(adcBuffer[ADC_TENSAO1], gCalibrationData.tensao1_offset, gCalibrationData.tensao1_coeff);
    float newTensao2 = ADC_Sensor_Convert_Tensao(adcBuffer[ADC_TENSAO2], gCalibrationData.tensao2_offset, gCalibrationData.tensao2_coeff);
    float newCorrente1 = ADC_Sensor_Convert_Corrente(adcBuffer[ADC_CORRENTE1], gCalibrationData.corrente1_offset, gCalibrationData.corrente1_coeff);
    float newCorrente2 = ADC_Sensor_Convert_Corrente(adcBuffer[ADC_CORRENTE2], gCalibrationData.corrente2_offset, gCalibrationData.corrente2_coeff);
    float newTemperatura1 = ADC_Sensor_Convert_Temperatura(adcBuffer[ADC_TEMPERATURA1], gCalibrationData.temperatura1_offset, gCalibrationData.temperatura1_coeff);
    float newTemperatura2 = ADC_Sensor_Convert_Temperatura(adcBuffer[ADC_TEMPERATURA2], gCalibrationData.temperatura2_offset, gCalibrationData.temperatura2_coeff);
    float newMassa1 = ADC_Sensor_Convert_Massa(adcBuffer[ADC_MASSA1], gCalibrationData.massa1_offset, gCalibrationData.massa1_coeff);
    float newMassa2 = ADC_Sensor_Convert_Massa(adcBuffer[ADC_MASSA2], gCalibrationData.massa2_offset, gCalibrationData.massa2_coeff);

    // --- Aplica o filtro Butterworth de 2ª ordem a TODAS as leituras dos sensores ---
    // Usando os coeficientes específicos para cada tipo de sensor.
    tensao1Eng = (float)butter2_apply(&tensao1FilterState, (double)newTensao1, &tensaoFilterCoeffs);
    tensao2Eng = (float)butter2_apply(&tensao2FilterState, (double)newTensao2, &tensao2FilterCoeffs); // Note: Assuming tensao2 also uses tensaoFilterCoeffs, update if different
    corrente1Eng = (float)butter2_apply(&corrente1FilterState, (double)newCorrente1, &correnteFilterCoeffs);
    corrente2Eng = (float)butter2_apply(&corrente2FilterState, (double)newCorrente2, &corrente2FilterCoeffs); // Note: Assuming corrente2 also uses correnteFilterCoeffs, update if different
    temperatura1Eng = (float)butter2_apply(&temperatura1FilterState, (double)newTemperatura1, &temperaturaFilterCoeffs);
    temperatura2Eng = (float)butter2_apply(&temperatura2FilterState, (double)newTemperatura2, &temperatura2FilterCoeffs); // Note: Assuming temperatura2 also uses temperaturaFilterCoeffs, update if different
    massa1Eng = (float)butter2_apply(&massa1FilterState, (double)newMassa1, &massaFilterCoeffs);
    massa2Eng = (float)butter2_apply(&massa2FilterState, (double)newMassa2, &massa2FilterCoeffs); // Note: Assuming massa2 also uses massaFilterCoeffs, update if different

    firstAdcRead = false; // Se a flag for usada para outras finalidades, mantenha e atualize.


    // Reinicia a conversão do ADC via DMA imediatamente após o processamento.
    HAL_ADC_Start_DMA(pAdcHandle, (uint32_t*)adcBuffer, NUM_ADC_CHANNELS);
}

// --- Funções de Conversão (permanecem as mesmas) ---
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

// --- Funções Getters (permanecem as mesmas) ---
float ADC_Sensor_GetTensao1(void) { return tensao1Eng; }
float ADC_Sensor_GetTensao2(void) { return tensao2Eng; }
float ADC_Sensor_GetCorrente1(void) { return corrente1Eng; }
float ADC_Sensor_GetCorrente2(void) { return corrente2Eng; }
float ADC_Sensor_GetTemperatura1(void) { return temperatura1Eng; }
float ADC_Sensor_GetTemperatura2(void) { return temperatura2Eng; }
float ADC_Sensor_GetMassa1(void) { return massa1Eng; }
float ADC_Sensor_GetMassa2(void) { return massa2Eng; }
