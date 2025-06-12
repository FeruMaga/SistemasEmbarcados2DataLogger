/*
 * adc_sensors.h
 *
 *  Created on: Jun 12, 2025
 *      Author: FeruMaga
 */

#ifndef INC_ADC_SENSOR_H_
#define INC_ADC_SENSOR_H_


#include "main.h"
#include <stdbool.h>

#define NUM_ADC_CHANNELS 8
#define ADC_BUFFER_SIZE (NUM_ADC_CHANNELS * 64)

#define ADC_TENSAO1       0
#define ADC_TENSAO2       1
#define ADC_CORRENTE1     2
#define ADC_CORRENTE2     3
#define ADC_TEMPERATURA1  4
#define ADC_TEMPERATURA2  5
#define ADC_MASSA1        6
#define ADC_MASSA2        7

#define ADC_MAX_VALUE 4095.0f
#define ADC_VREF_ANALOG 3.3f

typedef struct {
    float tensao1_offset;
    float tensao1_coeff;
    float tensao2_offset;
    float tensao2_coeff;
    float corrente1_offset;
    float corrente1_coeff;
    float corrente2_offset;
    float corrente2_coeff;
    float temperatura1_offset;
    float temperatura1_coeff;
    float temperatura2_offset;
    float temperatura2_coeff;
    float massa1_offset;
    float massa1_coeff;
    float massa2_offset;
    float massa2_coeff;
} CalibrationData_t;

extern CalibrationData_t gCalibrationData;

void ADC_Sensor_Init(ADC_HandleTypeDef* hadc);
void ADC_Sensor_StartConversion(void);
void ADC_Sensor_HandleDMA_Complete(void);
float ADC_Sensor_Convert_Tensao(uint16_t adc_raw, float offset, float coeff);
float ADC_Sensor_Convert_Corrente(uint16_t adc_raw, float offset, float coeff);
float ADC_Sensor_Convert_Temperatura(uint16_t adc_raw, float offset, float coeff);
float ADC_Sensor_Convert_Massa(uint16_t adc_raw, float offset, float coeff);

float ADC_Sensor_GetTensao1(void);
float ADC_Sensor_GetTensao2(void);
float ADC_Sensor_GetCorrente1(void);
float ADC_Sensor_GetCorrente2(void);
float ADC_Sensor_GetTemperatura1(void);
float ADC_Sensor_GetTemperatura2(void);
float ADC_Sensor_GetMassa1(void);
float ADC_Sensor_GetMassa2(void);
#endif /* INC_ADC_SENSOR_H_ */
