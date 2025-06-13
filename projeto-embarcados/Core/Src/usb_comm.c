/*
 * usb.c
 *
 *  Created on: Jun 12, 2025
 *      Author: FeruMaga
 */


#include "usb_comm.h"
#include "usbd_cdc_if.h"

extern CalibrationData_t gCalibrationData;

void USB_Init(PCD_HandleTypeDef* hpcd) {
    // Inicia o periférico USB Device (PCD).
    // Prepara o hardware USB para comunicação.
    HAL_PCD_Start(hpcd);
}

void USB_Receive(uint8_t* Buf, uint32_t Len) {
    char rx_buffer_copy[USB_RX_BUFFER_SIZE]; // Buffer local para copiar os dados recebidos.

    // Garante que o comprimento dos dados não exceda o tamanho do buffer local.
    if (Len >= USB_RX_BUFFER_SIZE) {
        Len = USB_RX_BUFFER_SIZE - 1; // Deixa espaço para o terminador nulo.
    }
    // Copia os dados recebidos do buffer da USB para o buffer local.
    memcpy(rx_buffer_copy, Buf, Len);
    // Adiciona o terminador nulo para tratar a cópia como uma string C.
    rx_buffer_copy[Len] = '\0';

    // Tenta processar a string como um comando de calibração.
    if (USB_ProcessCalibrationCommand((uint8_t*)rx_buffer_copy, Len, &gCalibrationData)) {
        // Se a calibração foi bem-sucedida, marca o sistema como calibrado.
        gCalibrationData.isCalibrated = true;
        // Envia uma mensagem de sucesso de volta ao host.
        USB_Send("Calibracao aplicada com sucesso!\r\n");
    } else {
        // Se o comando de calibração for inválido.
        if (gCalibrationData.isCalibrated) {
            // Se já havia calibração, informa que o comando foi inválido, mas mantém os dados existentes.
            USB_Send("Comando de calibracao invalido. Usando valores ja calibrados.\r\n");
        } else {
            // Se não havia calibração e o comando foi inválido, informa que usará valores padrão.
            USB_Send("Calibracao invalida ou ausente. Usando valores mocados.\r\n");
        }
    }
}

bool USB_ProcessCalibrationCommand(uint8_t* data, uint32_t len, CalibrationData_t* calData) {
    char *token;       // Ponteiro para cada pedaço da string separada por vírgula.
    // Aloca memória dinamicamente para uma cópia da string de entrada,
    // pois strtok_r modifica a string original.
    char *data_copy = (char*)malloc(len + 1);
    if (data_copy == NULL) return false; // Retorna false se a alocação falhar.
    memcpy(data_copy, data, len);        // Copia os dados.
    data_copy[len] = '\0';               // Garante a terminação nula.

    char *rest = data_copy;              // Ponteiro para o restante da string a ser analisada.
    bool receivedAny = false;            // Flag para verificar se algum parâmetro foi lido.

    // Itera sobre a string, dividindo-a em tokens (partes separadas por vírgulas).
    while ((token = strtok_r(rest, ",", &rest)) != NULL) {
        // Tenta parsear cada token para um parâmetro de calibração específico usando sscanf.
        // O retorno '1' de sscanf indica que um valor foi lido e atribuído com sucesso.
        if (sscanf(token, "T1_OFF=%f", &calData->tensao1_offset) == 1) receivedAny = true;
        else if (sscanf(token, "T1_COEFF=%f", &calData->tensao1_coeff) == 1) receivedAny = true;
        else if (sscanf(token, "T2_OFF=%f", &calData->tensao2_offset) == 1) receivedAny = true;
        else if (sscanf(token, "T2_COEFF=%f", &calData->tensao2_coeff) == 1) receivedAny = true;
        else if (sscanf(token, "C1_OFF=%f", &calData->corrente1_offset) == 1) receivedAny = true;
        else if (sscanf(token, "C1_COEFF=%f", &calData->corrente1_coeff) == 1) receivedAny = true;
        else if (sscanf(token, "C2_OFF=%f", &calData->corrente2_offset) == 1) receivedAny = true;
        else if (sscanf(token, "C2_COEFF=%f", &calData->corrente2_coeff) == 1) receivedAny = true;
        else if (sscanf(token, "TEMP1_OFF=%f", &calData->temperatura1_offset) == 1) receivedAny = true;
        else if (sscanf(token, "TEMP1_COEFF=%f", &calData->temperatura1_coeff) == 1) receivedAny = true;
        else if (sscanf(token, "TEMP2_OFF=%f", &calData->temperatura2_offset) == 1) receivedAny = true;
        else if (sscanf(token, "TEMP2_COEFF=%f", &calData->temperatura2_coeff) == 1) receivedAny = true;
        else if (sscanf(token, "MASSA1_OFF=%f", &calData->massa1_offset) == 1) receivedAny = true;
        else if (sscanf(token, "MASSA1_COEFF=%f", &calData->massa1_coeff) == 1) receivedAny = true;
        else if (sscanf(token, "MASSA2_OFF=%f", &calData->massa2_offset) == 1) receivedAny = true;
        else if (sscanf(token, "MASSA2_COEFF=%f", &calData->massa2_coeff) == 1) receivedAny = true;
    }

    free(data_copy); // Libera a memória alocada.
    return receivedAny; // Retorna verdadeiro se algum parâmetro foi lido.
}

void USB_Send(char* str) {
    // Envia a string para o host via USB CDC (Virtual Serial Port).
    CDC_Transmit_FS((uint8_t*)str, strlen(str));
}
