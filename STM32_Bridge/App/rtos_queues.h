/**
  ******************************************************************************
  * @file           : rtos_queues.h
  * @brief          : Header for Thread-Safe Inter-Task Queues on STM32_Bridge
  ******************************************************************************
  */

#ifndef RTOS_QUEUES_H
#define RTOS_QUEUES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"

typedef enum {
  TELEMETRY_LIDAR_REAR_LEFT = 0,
  TELEMETRY_LIDAR_REAR_RIGHT,
  TELEMETRY_SONAR_LEFT,
  TELEMETRY_SONAR_RIGHT,
  TELEMETRY_SAFETY_STATUS
} TelemetryType_t;

typedef struct {
  TelemetryType_t type;
  float           range_m;
  uint8_t         status_byte;
  uint32_t        timestamp_ms;
} TelemetryMsg_t;

typedef struct {
  uint8_t  relay_mask;
  uint16_t pulse_duration_ms;
} RelayCmd_t;

typedef struct {
  uint8_t  emergency_flags; /* bit0: E-Stop, bits 2..5: Bumpers */
  uint32_t timestamp_ms;
} EmergencyEvent_t;

extern QueueHandle_t xTelemetryQueue;
extern QueueHandle_t xRelayCmdQueue;
extern QueueHandle_t xEmergencyQueue;

bool RTOS_Queues_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* RTOS_QUEUES_H */
