/**
  ******************************************************************************
  * @file           : custom_transport.c
  * @brief          : Custom UART transport implementation for micro-ROS on STM32F103
  ******************************************************************************
  */

#include "custom_transport.h"
#include <string.h>

#define RX_RING_BUFFER_SIZE 512

static UART_HandleTypeDef *g_huart = NULL;
static volatile uint8_t g_rx_buffer[RX_RING_BUFFER_SIZE];
static volatile uint16_t g_rx_head = 0;
static volatile uint16_t g_rx_tail = 0;

void custom_transport_init(UART_HandleTypeDef *huart)
{
  g_huart = huart;
  g_rx_head = 0;
  g_rx_tail = 0;

  if (g_huart != NULL)
  {
    /* Enable RX Not Empty interrupt on USART1 */
    __HAL_UART_ENABLE_IT(g_huart, UART_IT_RXNE);
  }
}

void custom_transport_rx_push(uint8_t byte)
{
  uint16_t next_head = (uint16_t)((g_rx_head + 1) % RX_RING_BUFFER_SIZE);
  if (next_head != g_rx_tail)
  {
    g_rx_buffer[g_rx_head] = byte;
    g_rx_head = next_head;
  }
}

bool custom_transport_open(struct uxrCustomTransport * transport)
{
  (void)transport;
  g_rx_head = 0;
  g_rx_tail = 0;
  if (g_huart != NULL)
  {
    __HAL_UART_ENABLE_IT(g_huart, UART_IT_RXNE);
  }
  return true;
}

bool custom_transport_close(struct uxrCustomTransport * transport)
{
  (void)transport;
  if (g_huart != NULL)
  {
    __HAL_UART_DISABLE_IT(g_huart, UART_IT_RXNE);
  }
  return true;
}

size_t custom_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err)
{
  (void)transport;
  if (g_huart == NULL || buf == NULL || len == 0)
  {
    if (err) *err = 1;
    return 0;
  }

  HAL_StatusTypeDef status = HAL_UART_Transmit(g_huart, (uint8_t *)buf, (uint16_t)len, 100);
  if (status == HAL_OK)
  {
    if (err) *err = 0;
    return len;
  }
  else
  {
    if (err) *err = 1;
    return 0;
  }
}

size_t custom_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err)
{
  (void)transport;
  if (buf == NULL || len == 0)
  {
    if (err) *err = 1;
    return 0;
  }

  uint32_t start = HAL_GetTick();
  size_t bytes_read = 0;

  while (bytes_read < len)
  {
    if (g_rx_tail != g_rx_head)
    {
      buf[bytes_read++] = g_rx_buffer[g_rx_tail];
      g_rx_tail = (uint16_t)((g_rx_tail + 1) % RX_RING_BUFFER_SIZE);
    }
    else
    {
      if ((HAL_GetTick() - start) >= (uint32_t)timeout)
      {
        break;
      }
    }
  }

  if (err) *err = (bytes_read > 0) ? 0 : 1;
  return bytes_read;
}
