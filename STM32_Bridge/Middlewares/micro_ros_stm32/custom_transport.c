/**
  ******************************************************************************
  * @file           : custom_transport.c
  * @brief          : Implementation of custom transport for micro-ROS on STM32F411
  ******************************************************************************
  */

#include "custom_transport.h"
#include <string.h>

#define RX_BUFFER_SIZE 2048

static volatile uint8_t  g_rx_ring_buffer[RX_BUFFER_SIZE];
static volatile uint16_t g_rx_head = 0;
static volatile uint16_t g_rx_tail = 0;

/* Weak reference to USB CDC Transmit function */
__attribute__((weak)) uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  (void)Buf;
  (void)Len;
  return 0; /* USBD_OK */
}

void custom_transport_rx_push(uint8_t byte)
{
  uint16_t next_head = (uint16_t)((g_rx_head + 1) % RX_BUFFER_SIZE);
  if (next_head != g_rx_tail)
  {
    g_rx_ring_buffer[g_rx_head] = byte;
    g_rx_head = next_head;
  }
}

void custom_transport_rx_push_buffer(const uint8_t *buffer, size_t len)
{
  if (!buffer || len == 0) return;
  for (size_t i = 0; i < len; i++)
  {
    custom_transport_rx_push(buffer[i]);
  }
}

bool custom_transport_open(struct uxrCustomTransport * transport)
{
  (void)transport;
  g_rx_head = 0;
  g_rx_tail = 0;
  return true;
}

bool custom_transport_close(struct uxrCustomTransport * transport)
{
  (void)transport;
  return true;
}

size_t custom_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err)
{
  (void)transport;
  if (!buf || len == 0)
  {
    if (err) *err = 1;
    return 0;
  }

  uint32_t start_tick = HAL_GetTick();
  uint8_t result = 1; /* USBD_BUSY */

  while ((HAL_GetTick() - start_tick) < 100)
  {
    result = CDC_Transmit_FS((uint8_t *)buf, (uint16_t)len);
    if (result == 0) /* USBD_OK */
    {
      break;
    }
  }

  if (result == 0) /* USBD_OK */
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
  if (!buf || len == 0)
  {
    if (err) *err = 1;
    return 0;
  }

  uint32_t start_tick = HAL_GetTick();
  size_t bytes_read = 0;

  while (bytes_read < len)
  {
    if (g_rx_tail != g_rx_head)
    {
      buf[bytes_read++] = g_rx_ring_buffer[g_rx_tail];
      g_rx_tail = (uint16_t)((g_rx_tail + 1) % RX_BUFFER_SIZE);
    }
    else
    {
      if ((HAL_GetTick() - start_tick) >= (uint32_t)timeout)
      {
        break;
      }
    }
  }

  if (err) *err = (bytes_read > 0) ? 0 : 1;
  return bytes_read;
}
