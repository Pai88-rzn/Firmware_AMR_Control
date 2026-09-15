/**
  ******************************************************************************
  * @file           : rtos_queues.c
  * @brief          : Implementation of Thread-Safe Inter-Task Queues
  ******************************************************************************
  */

#include "rtos_queues.h"

QueueHandle_t xTelemetryQueue = NULL;
QueueHandle_t xRelayCmdQueue  = NULL;
QueueHandle_t xEmergencyQueue = NULL;

bool RTOS_Queues_Init(void)
{
  xTelemetryQueue = xQueueCreate(16, sizeof(TelemetryMsg_t));
  xRelayCmdQueue  = xQueueCreate(8,  sizeof(RelayCmd_t));
  xEmergencyQueue = xQueueCreate(4,  sizeof(EmergencyEvent_t));

  return (xTelemetryQueue != NULL && xRelayCmdQueue != NULL && xEmergencyQueue != NULL);
}
