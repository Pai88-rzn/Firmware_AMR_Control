/**
  ******************************************************************************
  * @file           : lora_module.h
  * @brief          : Header for LoRa Module Interface (USART1 on PA9/PA10)
  ******************************************************************************
  */

#ifndef LORA_MODULE_H
#define LORA_MODULE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

#define LORA_DEFAULT_BAUD       9600
#define LORA_RX_BUFFER_SIZE     256

HAL_StatusTypeDef LoRa_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef LoRa_Transmit(const uint8_t *data, uint16_t len);
uint16_t LoRa_ReceivePacket(uint8_t *buf, uint16_t max_len);

#ifdef __cplusplus
}
#endif

#endif /* LORA_MODULE_H */
