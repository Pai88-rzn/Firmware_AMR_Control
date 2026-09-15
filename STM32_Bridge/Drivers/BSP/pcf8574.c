/**
  ******************************************************************************
  * @file           : pcf8574.c
  * @brief          : Implementation of Dual PCF8574 I/O Expanders
  ******************************************************************************
  */

#include "pcf8574.h"

static uint8_t g_current_relay_state = 0x00; /* Logical state: 1=ON, 0=OFF */

HAL_StatusTypeDef PCF8574_Init(I2C_HandleTypeDef *hi2c)
{
  if (!hi2c) return HAL_ERROR;

  /* Rule 5.1: All relays MUST default to OFF (de-energized) on startup.
     Hardware circuit is Active LOW, so writing 0xFF de-energizes all relays. */
  uint8_t failsafe_byte = 0xFF;
  HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(
      hi2c,
      (uint16_t)(PCF8574_ADDR_OUTPUT << 1),
      &failsafe_byte,
      1,
      50
  );

  g_current_relay_state = 0x00;
  return status;
}

HAL_StatusTypeDef PCF8574_WriteRelays(I2C_HandleTypeDef *hi2c, uint8_t active_high_relay_mask)
{
  if (!hi2c) return HAL_ERROR;

  /* Invert bits because hardware drivers (BC848 + PC817) are Active LOW */
  uint8_t physical_byte = (uint8_t)(~active_high_relay_mask);

  HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(
      hi2c,
      (uint16_t)(PCF8574_ADDR_OUTPUT << 1),
      &physical_byte,
      1,
      20
  );

  if (status == HAL_OK)
  {
    g_current_relay_state = active_high_relay_mask;
  }
  return status;
}

HAL_StatusTypeDef PCF8574_SetRelay(I2C_HandleTypeDef *hi2c, uint8_t relay_index, bool state)
{
  if (relay_index > 7) return HAL_ERROR;

  uint8_t mask = g_current_relay_state;
  if (state)
  {
    mask |= (uint8_t)(1 << relay_index);
  }
  else
  {
    mask &= (uint8_t)~(1 << relay_index);
  }

  return PCF8574_WriteRelays(hi2c, mask);
}

HAL_StatusTypeDef PCF8574_ReadInputs(I2C_HandleTypeDef *hi2c, uint8_t *raw_active_low, uint8_t *active_high_flags)
{
  if (!hi2c) return HAL_ERROR;

  uint8_t data = 0xFF;
  HAL_StatusTypeDef status = HAL_I2C_Master_Receive(
      hi2c,
      (uint16_t)(PCF8574_ADDR_INPUT << 1),
      &data,
      1,
      20
  );

  if (status == HAL_OK)
  {
    if (raw_active_low)
    {
      *raw_active_low = data;
    }
    if (active_high_flags)
    {
      /* Active LOW inverted: bit=1 means switch is triggered/pressed */
      *active_high_flags = (uint8_t)(~data);
    }
  }

  return status;
}

uint8_t PCF8574_GetLastRelayState(void)
{
  return g_current_relay_state;
}
