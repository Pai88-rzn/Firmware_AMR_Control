/**
  ******************************************************************************
  * @file           : io_controller.c
  * @brief          : Actuator, Relay, Tower Light, and Buzzer Controller
  ******************************************************************************
  */

#include "io_controller.h"
#include "rtos_queues.h"

static TowerLightMode_t g_tower_mode = TOWER_LIGHT_GREEN;
static uint32_t g_buzzer_until_tick = 0;
static bool g_flash_state = false;
static uint32_t g_last_flash_toggle = 0;

void IoController_Init(I2C_HandleTypeDef *hi2c3)
{
  g_tower_mode = TOWER_LIGHT_GREEN;
  g_buzzer_until_tick = 0;
  g_flash_state = false;
  g_last_flash_toggle = HAL_GetTick();

  IoController_SetTowerLight(hi2c3, g_tower_mode);
}

void IoController_SetTowerLight(I2C_HandleTypeDef *hi2c3, TowerLightMode_t mode)
{
  g_tower_mode = mode;

  uint8_t current_mask = PCF8574_GetLastRelayState();

  /* Clear all 3 tower lights */
  current_mask &= (uint8_t)~(RELAY_BIT_LIGHT_GREEN | RELAY_BIT_LIGHT_YEL | RELAY_BIT_LIGHT_RED);

  switch (mode)
  {
    case TOWER_LIGHT_GREEN:
      current_mask |= RELAY_BIT_LIGHT_GREEN;
      break;
    case TOWER_LIGHT_YELLOW:
      current_mask |= RELAY_BIT_LIGHT_YEL;
      break;
    case TOWER_LIGHT_RED:
      current_mask |= RELAY_BIT_LIGHT_RED;
      break;
    default:
      break;
  }

  PCF8574_WriteRelays(hi2c3, current_mask);
}

void IoController_TriggerBuzzer(uint16_t duration_ms)
{
  g_buzzer_until_tick = HAL_GetTick() + duration_ms;
}

void IoController_Step(I2C_HandleTypeDef *hi2c3)
{
  uint32_t now = HAL_GetTick();
  uint8_t current_mask = PCF8574_GetLastRelayState();
  bool need_update = false;

  /* Check Buzzer duration */
  bool buzzer_should_be_on = (now < g_buzzer_until_tick);
  bool buzzer_is_on = (current_mask & RELAY_BIT_BUZZER) != 0;

  if (buzzer_should_be_on != buzzer_is_on)
  {
    if (buzzer_should_be_on)
    {
      current_mask |= RELAY_BIT_BUZZER;
    }
    else
    {
      current_mask &= (uint8_t)~RELAY_BIT_BUZZER;
    }
    need_update = true;
  }

  /* Handle Flashing Red mode */
  if (g_tower_mode == TOWER_LIGHT_FLASHING_RED)
  {
    if ((now - g_last_flash_toggle) >= 250) /* 2 Hz flash */
    {
      g_last_flash_toggle = now;
      g_flash_state = !g_flash_state;

      current_mask &= (uint8_t)~(RELAY_BIT_LIGHT_GREEN | RELAY_BIT_LIGHT_YEL);
      if (g_flash_state)
      {
        current_mask |= RELAY_BIT_LIGHT_RED;
      }
      else
      {
        current_mask &= (uint8_t)~RELAY_BIT_LIGHT_RED;
      }
      need_update = true;
    }
  }

  if (need_update)
  {
    PCF8574_WriteRelays(hi2c3, current_mask);
  }
}
