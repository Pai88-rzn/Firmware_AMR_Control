/**
  ******************************************************************************
  * @file           : safety_monitor.h
  * @brief          : Hard Real-Time (<10ms) Safety Monitor & Bumper Cutoff
  ******************************************************************************
  */

#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "pcf8574.h"
#include "rtos_queues.h"

#define WATCHDOG_TIMEOUT_MS     500  /* 500ms without host heartbeat triggers failsafe */

typedef enum {
  SAFETY_OK = 0,
  SAFETY_ESTOP_ACTIVE,
  SAFETY_COLLISION_ACTIVE,
  SAFETY_WATCHDOG_TIMEOUT
} SafetyState_t;

void SafetyMonitor_Init(I2C_HandleTypeDef *hi2c3);
void SafetyMonitor_Step(I2C_HandleTypeDef *hi2c3);
void SafetyMonitor_FeedWatchdog(void);
bool SafetyMonitor_ResetEmergency(I2C_HandleTypeDef *hi2c3);
SafetyState_t SafetyMonitor_GetState(void);
uint8_t SafetyMonitor_GetRawInputs(void);

#ifdef __cplusplus
}
#endif

#endif /* SAFETY_MONITOR_H */
