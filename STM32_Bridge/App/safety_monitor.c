/**
  ******************************************************************************
  * @file           : safety_monitor.c
  * @brief          : Hard Real-Time (<10ms) Safety Monitor Implementation
  ******************************************************************************
  */

#include "safety_monitor.h"

static SafetyState_t g_safety_state = SAFETY_OK;
static uint32_t g_last_watchdog_feed = 0;
static uint8_t  g_last_inputs_active_high = 0;

void SafetyMonitor_Init(I2C_HandleTypeDef *hi2c3)
{
  (void)hi2c3;
  g_safety_state = SAFETY_OK;
  g_last_watchdog_feed = HAL_GetTick();
  g_last_inputs_active_high = 0;
}

void SafetyMonitor_FeedWatchdog(void)
{
  g_last_watchdog_feed = HAL_GetTick();
  if (g_safety_state == SAFETY_WATCHDOG_TIMEOUT)
  {
    if ((g_last_inputs_active_high & (INPUT_BIT_ESTOP | BUMPER_MASK_ALL)) == 0)
    {
      g_safety_state = SAFETY_OK;
    }
  }
}

void SafetyMonitor_Step(I2C_HandleTypeDef *hi2c3)
{
  uint8_t raw_low = 0xFF;
  uint8_t active_high = 0x00;

  /* Sample PCF8574 Input Expander (IC3 @ 0x21) */
  if (PCF8574_ReadInputs(hi2c3, &raw_low, &active_high) == HAL_OK)
  {
    g_last_inputs_active_high = active_high;

    /* Check E-Stop button (DIN1) */
    if (active_high & INPUT_BIT_ESTOP)
    {
      if (g_safety_state != SAFETY_ESTOP_ACTIVE)
      {
        g_safety_state = SAFETY_ESTOP_ACTIVE;

        /* Hard Real-Time Cutoff: Immediately cut Motor Enable Relay (DOUT2) */
        PCF8574_SetRelay(hi2c3, 1, false);

        /* Send high-priority emergency event to front of queue */
        EmergencyEvent_t evt = {
          .emergency_flags = INPUT_BIT_ESTOP,
          .timestamp_ms = HAL_GetTick()
        };
        xQueueSendToFront(xEmergencyQueue, &evt, 0);
      }
      return;
    }

    /* Check any of the 4 bumper limit switches (DIN3..DIN6) */
    if (active_high & BUMPER_MASK_ALL)
    {
      if (g_safety_state != SAFETY_COLLISION_ACTIVE)
      {
        g_safety_state = SAFETY_COLLISION_ACTIVE;

        /* Hard Real-Time Cutoff: Immediately cut Motor Enable Relay (DOUT2) */
        PCF8574_SetRelay(hi2c3, 1, false);

        /* Send high-priority emergency event to front of queue */
        EmergencyEvent_t evt = {
          .emergency_flags = (active_high & BUMPER_MASK_ALL),
          .timestamp_ms = HAL_GetTick()
        };
        xQueueSendToFront(xEmergencyQueue, &evt, 0);
      }
      return;
    }
  }

  /* Check Host Communication Watchdog */
  uint32_t now = HAL_GetTick();
  if ((now - g_last_watchdog_feed) > WATCHDOG_TIMEOUT_MS)
  {
    if (g_safety_state == SAFETY_OK)
    {
      g_safety_state = SAFETY_WATCHDOG_TIMEOUT;
      /* Cut motor power relay on comm loss */
      PCF8574_SetRelay(hi2c3, 1, false);
    }
  }
}

bool SafetyMonitor_ResetEmergency(I2C_HandleTypeDef *hi2c3)
{
  /* Can only reset if bumpers and E-stop are physically clear */
  if ((g_last_inputs_active_high & (INPUT_BIT_ESTOP | BUMPER_MASK_ALL)) == 0)
  {
    g_safety_state = SAFETY_OK;
    g_last_watchdog_feed = HAL_GetTick();
    /* Re-enable motor relay */
    PCF8574_SetRelay(hi2c3, 1, true);
    return true;
  }
  return false;
}

SafetyState_t SafetyMonitor_GetState(void)
{
  return g_safety_state;
}

uint8_t SafetyMonitor_GetRawInputs(void)
{
  return g_last_inputs_active_high;
}
