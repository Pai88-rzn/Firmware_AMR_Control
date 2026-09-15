/**
  ******************************************************************************
  * @file           : microros_interface.h
  * @brief          : Header for micro-ROS Client interface on STM32_Bridge
  ******************************************************************************
  */

#ifndef MICROROS_INTERFACE_H
#define MICROROS_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

void MicroRos_Init(void);
void MicroRos_SpinOnce(void);
bool MicroRos_IsConnected(void);

#ifdef __cplusplus
}
#endif

#endif /* MICROROS_INTERFACE_H */
