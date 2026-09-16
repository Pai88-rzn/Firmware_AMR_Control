/**
  ******************************************************************************
  * @file           : onboard_leds.c
  * @brief          : Implementation of status LEDs for STM32_Bridge
  ******************************************************************************
  */

#include "onboard_leds.h"

void OnboardLEDs_Init(void)
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Default state: OFF (Active LOW -> Set High) */
  HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_SET);

  /* Configure LED1 (PC13) */
  GPIO_InitStruct.Pin = LED1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStruct);

  /* Configure LED2 (PB12) */
  GPIO_InitStruct.Pin = LED2_PIN;
  HAL_GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStruct);
}

void OnboardLEDs_Set(uint8_t led_id, bool on)
{
  GPIO_PinState pin_state = on ? GPIO_PIN_RESET : GPIO_PIN_SET; /* Active LOW */
  if (led_id == LED_SYS_STATUS)
  {
    HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, pin_state);
  }
  else if (led_id == LED_COMM_STATUS)
  {
    HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, pin_state);
  }
}

void OnboardLEDs_Toggle(uint8_t led_id)
{
  if (led_id == LED_SYS_STATUS)
  {
    HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);
  }
  else if (led_id == LED_COMM_STATUS)
  {
    HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);
  }
}
