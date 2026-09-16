/**
  ******************************************************************************
  * @file           : onboard_leds.h
  * @brief          : Header for status LEDs on STM32_Bridge
  *                   LED1: PC13 (Active LOW)
  *                   LED2: PB12 (Active LOW)
  ******************************************************************************
  */

#ifndef ONBOARD_LEDS_H
#define ONBOARD_LEDS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

#define LED_SYS_STATUS  0  /* LED1 - PC13 */
#define LED_COMM_STATUS 1  /* LED2 - PB12 */

void OnboardLEDs_Init(void);
void OnboardLEDs_Set(uint8_t led_id, bool on);
void OnboardLEDs_Toggle(uint8_t led_id);

#ifdef __cplusplus
}
#endif

#endif /* ONBOARD_LEDS_H */
