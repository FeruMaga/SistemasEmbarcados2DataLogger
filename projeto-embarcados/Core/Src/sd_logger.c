/*
 * sd_logger.c
 *
 *  Created on: Jun 12, 2025
 *      Author: FeruMaga
 */



#include "sd_logger.h"
#include "ff.h"

 // Objeto de sistema de arquivos (File System Object) para a biblioteca FATFS.
FATFS fs;
// Objeto de arquivo (File Object) para a biblioteca FATFS.
FIL file;

// Variável para armazenar o número de bytes escritos em uma operação de escrita.
UINT bytesWritten;
// Inicializa a flag que indica se o cartão SD está cheio.
bool g_sd_card_full = false;

void SD_Logger_Init(SPI_HandleTypeDef* hspi) {
    // Redefine a flag de cartão cheio para falso na inicialização.
    g_sd_card_full = false;

    // Tenta vincular o driver de hardware do cartão SD (definido como USER_Driver)
    if (FATFS_LinkDriver(&USER_Driver, USERPath) != 0) {
        // Se houver um erro ao vincular o driver, envia uma mensagem via USB
        // e chama o Error_Handler do sistema.
        USB_Send("Erro ao linkar driver FATFS\r\n");
        Error_Handler();
    }

    // Tenta montar o sistema de arquivos FATFS no volume lógico.
    // O "" indica o volume padrão, e "1" indica que a montagem deve ser forçada (útil em alguns cenários).
    FRESULT fr_mount = f_mount(&fs, "", 1);
    if (fr_mount != FR_OK) {
        // Se a montagem falhar, o cartão SD pode estar ausente, não formatado ou corrompido.
        USB_Send("Erro ao montar SD\r\n");
        // Em caso de falha na montagem, assume-se que o cartão está inacessível para escrita.
        // A flag 'g_sd_card_full' é definida como true para evitar tentativas futuras de escrita.
        g_sd_card_full = true;
        // Chama o Error_Handler do sistema para tratamento de erro crítico.
        Error_Handler(); // Dependendo da sua estratégia de erro, pode parar aqui.
    }
}

// Implementação da função de escrita de dados no cartão SD.
// Retorna um status indicando o resultado da operação.
SD_Status_t SD_Logger_WriteData(char* data) {
    FRESULT fr; // Variável para armazenar o resultado das operações da biblioteca FATFS.

    // Verifica se a flag de cartão cheio está ativa.
    // Se estiver, evita novas tentativas de escrita para otimizar o desempenho.
    if (g_sd_card_full) {
        return SD_WRITE_ERR; // Retorna um erro indicando que o cartão não pode ser escrito.
                              // Poderia ser um novo status como SD_ALREADY_FULL para maior clareza.
    }

    // Tenta abrir o arquivo de log (LOG_FILE_NAME).
    // FA_OPEN_APPEND: Abre o arquivo para escrita, posicionando o cursor no final.
    // Se o arquivo não existir, ele será criado.
    // FA_WRITE: Habilita a permissão de escrita.
    fr = f_open(&file, LOG_FILE_NAME, FA_OPEN_APPEND | FA_WRITE);

    // Verifica se a operação de abertura do arquivo foi bem-sucedida.
    if (fr == FR_OK) {
        // Se o arquivo foi aberto com sucesso, tenta escrever os dados.
        // 'data': Ponteiro para os dados a serem escritos.
        // 'strlen(data)': Número de bytes a serem escritos (tamanho da string).
        // '&bytesWritten': Ponteiro para a variável onde o número de bytes realmente escritos será armazenado.
        fr = f_write(&file, data, strlen(data), &bytesWritten);

        // Verifica se a escrita foi bem-sucedida (fr == FR_OK) E se todos os bytes foram escritos.
        if (fr == FR_OK && bytesWritten == strlen(data)) {
            f_close(&file); // Fecha o arquivo para salvar as alterações e liberar recursos.
            return SD_OK; // Retorna status de sucesso.
        } else {
            // Se houve um erro durante a escrita ou se nem todos os bytes foram escritos.
            // Isso pode ocorrer por disco cheio, erro físico, ou outros problemas.
            USB_Send("Erro de escrita no SD ou disco cheio.\r\n");
            // Define a flag de cartão cheio como true.
            // Isso é uma medida preventiva para evitar novas tentativas de escrita que falhariam.
            g_sd_card_full = true;
            // Tenta fechar o arquivo mesmo com erro, para liberar recursos.
            f_close(&file);
            return SD_WRITE_ERR; // Retorna status de erro de escrita.
        }
    } else {
        // Se houver um erro ao tentar abrir o arquivo.
        // Isso pode ser causado por: cartão removido, cartão corrompido,
        // ou o próprio cartão SD estar fisicamente cheio (FATFS pode retornar FR_DENIED ou FR_DISK_ERR).
        USB_Send("Erro ao abrir arquivo de log no SD. (Cartão cheio ou problema?)\r\n");
        // Assume que o cartão está cheio ou inacessível para escrita e define a flag.
        g_sd_card_full = true;
        return SD_OPEN_ERR; // Retorna status de erro de abertura.
    }
}
