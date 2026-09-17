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

SemaphoreHandle_t xI2C1Mutex = NULL;
SemaphoreHandle_t xI2C2Mutex = NULL;
SemaphoreHandle_t xI2C3Mutex = NULL;

bool RTOS_Queues_Init(void)
{
  xTelemetryQueue = xQueueCreate(32, sizeof(TelemetryMsg_t));
  xRelayCmdQueue  = xQueueCreate(8,  sizeof(RelayCmd_t));
  xEmergencyQueue = xQueueCreate(4,  sizeof(EmergencyEvent_t));

  xI2C1Mutex = xSemaphoreCreateMutex();
  xI2C2Mutex = xSemaphoreCreateMutex();
  xI2C3Mutex = xSemaphoreCreateMutex();

  return (xTelemetryQueue != NULL && xRelayCmdQueue != NULL && xEmergencyQueue != NULL &&
          xI2C1Mutex != NULL && xI2C2Mutex != NULL && xI2C3Mutex != NULL);
}
