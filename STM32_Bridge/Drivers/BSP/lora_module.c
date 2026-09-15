/**
  ******************************************************************************
  * @file           : lora_module.c
  * @brief          : Implementation for LoRa Module Interface (USART1)
  ******************************************************************************
  */

#include "lora_module.h"

static UART_HandleTypeDef *g_lora_huart = NULL;

HAL_StatusTypeDef LoRa_Init(UART_HandleTypeDef *huart)
{
  g_lora_huart = huart;
  return HAL_OK;
}

HAL_StatusTypeDef LoRa_Transmit(const uint8_t *data, uint16_t len)
{
  if (!g_lora_huart || !data || len == 0) return HAL_ERROR;
  return HAL_UART_Transmit(g_lora_huart, (uint8_t *)data, len, 100);
}

uint16_t LoRa_ReceivePacket(uint8_t *buf, uint16_t max_len)
{
  if (!g_lora_huart || !buf || max_len == 0) return 0;

  uint16_t bytes_read = 0;
  HAL_StatusTypeDef status = HAL_UART_Receive(g_lora_huart, buf, max_len, 10);
  if (status == HAL_OK)
  {
    bytes_read = max_len;
  }
  return bytes_read;
}
