/**
  ******************************************************************************
  * @file           : pcf8574.h
  * @brief          : Header for Dual PCF8574 I/O Expanders on STM32_Bridge
  *                   IC2: Output Expander (0x20) -> 8x Relays (Active LOW)
  *                   IC3: Input Expander  (0x21) <- 8x Opto Inputs (Active LOW)
  ******************************************************************************
  */

#ifndef PCF8574_H
#define PCF8574_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

/* I2C 7-bit addresses */
#define PCF8574_ADDR_OUTPUT   0x20  /* IC2: 8x 24V Relays */
#define PCF8574_ADDR_INPUT    0x21  /* IC3: 8x Opto Digital Inputs */

/* Output bitmask definitions (Relays - Active LOW) */
#define RELAY_BIT_MAIN_PWR    (1 << 0)  /* DOUT1 */
#define RELAY_BIT_MOTOR_EN    (1 << 1)  /* DOUT2 */
#define RELAY_BIT_PAYLOAD_PWR (1 << 2)  /* DOUT3 */
#define RELAY_BIT_LIGHT_GREEN (1 << 3)  /* DOUT4 */
#define RELAY_BIT_LIGHT_YEL   (1 << 4)  /* DOUT5 */
#define RELAY_BIT_LIGHT_RED   (1 << 5)  /* DOUT6 */
#define RELAY_BIT_BUZZER      (1 << 6)  /* DOUT7 */
#define RELAY_BIT_HEADLIGHT   (1 << 7)  /* DOUT8 */

/* Input bitmask definitions (Sensors/Switches - Active LOW: 0 when pressed/tripped) */
#define INPUT_BIT_ESTOP       (1 << 0)  /* DIN1 */
#define INPUT_BIT_START_BTN   (1 << 1)  /* DIN2 */
#define INPUT_BIT_BUMPER_FL   (1 << 2)  /* DIN3 */
#define INPUT_BIT_BUMPER_FR   (1 << 3)  /* DIN4 */
#define INPUT_BIT_BUMPER_RL   (1 << 4)  /* DIN5 */
#define INPUT_BIT_BUMPER_RR   (1 << 5)  /* DIN6 */
#define INPUT_BIT_MODE_AUTO   (1 << 6)  /* DIN7 */
#define INPUT_BIT_DOCK_SENSE  (1 << 7)  /* DIN8 */

#define BUMPER_MASK_ALL       (INPUT_BIT_BUMPER_FL | INPUT_BIT_BUMPER_FR | INPUT_BIT_BUMPER_RL | INPUT_BIT_BUMPER_RR)

/* Driver Functions */
HAL_StatusTypeDef PCF8574_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef PCF8574_WriteRelays(I2C_HandleTypeDef *hi2c, uint8_t active_high_relay_mask);
HAL_StatusTypeDef PCF8574_SetRelay(I2C_HandleTypeDef *hi2c, uint8_t relay_index, bool state);
HAL_StatusTypeDef PCF8574_ReadInputs(I2C_HandleTypeDef *hi2c, uint8_t *raw_active_low, uint8_t *active_high_flags);
uint8_t PCF8574_GetLastRelayState(void);

#ifdef __cplusplus
}
#endif

#endif /* PCF8574_H */
