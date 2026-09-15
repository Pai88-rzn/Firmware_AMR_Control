/**
  ******************************************************************************
  * @file           : custom_transport.h
  * @brief          : Header for custom USB / UART transport for micro-ROS on STM32F411
  ******************************************************************************
  */

#ifndef CUSTOM_TRANSPORT_H
#define CUSTOM_TRANSPORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <uxr/client/transport.h>
#include <stdbool.h>
#include <stddef.h>

void custom_transport_rx_push(uint8_t byte);
void custom_transport_rx_push_buffer(const uint8_t *buffer, size_t len);

bool custom_transport_open(struct uxrCustomTransport * transport);
bool custom_transport_close(struct uxrCustomTransport * transport);
size_t custom_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t custom_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

#ifdef __cplusplus
}
#endif

#endif /* CUSTOM_TRANSPORT_H */
