/*
 * usb.c
 *
 *  Created on: Jun 12, 2025
 *      Author: FeruMaga
 */


#include "usb_comm.h"
#include "usbd_cdc_if.h"

void USB_Receive(uint8_t* Buf, uint32_t Len) {
    char rx_buffer_copy[USB_RX_BUFFER_SIZE];
    if (Len >= USB_RX_BUFFER_SIZE) {
        Len = USB_RX_BUFFER_SIZE - 1;
    }
    memcpy(rx_buffer_copy, Buf, Len);
    rx_buffer_copy[Len] = '\0'; 
    if (USB_ProcessCalibrationCommand((uint8_t*)rx_buffer_copy, Len, &gCalibrationData)) {
        USB_Send("Calibracao aplicada com sucesso!\r\n");
    } else {
        USB_Send("Calibracao invalida ou ausente. Usando valores mocados.\r\n");
    }

}

bool USB_ProcessCalibrationCommand(uint8_t* data, uint32_t len, CalibrationData_t* calData) {
    char *token;
    char *rest = (char*)data;
    bool receivedAny = false;

    while ((token = strtok_r(rest, ",", &rest)) != NULL) {
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

    return receivedAny;
}

void USB_Send(char* str) {
    CDC_Transmit_FS((uint8_t*)str, strlen(str));
}