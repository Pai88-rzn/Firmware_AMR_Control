/**
  ******************************************************************************
  * @file           : pcf8574.h
  * @brief          : Header for Dual PCF8574 I/O Expanders on STM32_Bridge
  *                   Supports:
  *                   1. Dual-Bus Prototype Setup:
  *                      - DO (Output Relays) on I2C1 (PB6/PB7) @ 0x24
  *                      - DI (Input Sensors) on I2C2 (PB10/PB3) @ 0x24
  *                   2. Single-Bus Custom PCB Setup:
  *                      - DO on I2C3 (PA8/PB4) @ 0x20
  *                      - DI on I2C3 (PA8/PB4) @ 0x21
  *                   3. Dynamic Auto-Discovery at boot
  ******************************************************************************
  */

#ifndef PCF8574_H
#define PCF8574_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* I2C 7-bit addresses (Default Custom PCB) */
#define PCF8574_ADDR_OUTPUT         0x20  /* IC2: 8x 24V Relays (Active LOW) */
#define PCF8574_ADDR_INPUT          0x21  /* IC3: 8x Opto Digital Inputs (Active LOW) */

/* Prototype / Tested Breakout Addresses */
#define PCF8574_ADDR_PROTOTYPE      0x24  /* A0=GND, A1=GND, A2=VCC */

/* Output bitmask definitions (Relays - Active LOW) */
#define RELAY_BIT_MAIN_PWR          (1 << 0)  /* DOUT1 */
#define RELAY_BIT_MOTOR_EN          (1 << 1)  /* DOUT2 */
#define RELAY_BIT_PAYLOAD_PWR       (1 << 2)  /* DOUT3 */
#define RELAY_BIT_LIGHT_GREEN       (1 << 3)  /* DOUT4 */
#define RELAY_BIT_LIGHT_YEL         (1 << 4)  /* DOUT5 */
#define RELAY_BIT_LIGHT_RED         (1 << 5)  /* DOUT6 */
#define RELAY_BIT_BUZZER            (1 << 6)  /* DOUT7 */
#define RELAY_BIT_HEADLIGHT         (1 << 7)  /* DOUT8 */

/* Input bitmask definitions (Sensors/Switches - Active LOW: 0 when pressed/tripped) */
#define INPUT_BIT_ESTOP             (1 << 0)  /* DIN1 */
#define INPUT_BIT_START_BTN         (1 << 1)  /* DIN2 */
#define INPUT_BIT_BUMPER_FL         (1 << 2)  /* DIN3 */
#define INPUT_BIT_BUMPER_FR         (1 << 3)  /* DIN4 */
#define INPUT_BIT_BUMPER_RL         (1 << 4)  /* DIN5 */
#define INPUT_BIT_BUMPER_RR         (1 << 5)  /* DIN6 */
#define INPUT_BIT_MODE_AUTO         (1 << 6)  /* DIN7 */
#define INPUT_BIT_DOCK_SENSE        (1 << 7)  /* DIN8 */

#define BUMPER_MASK_ALL             (INPUT_BIT_BUMPER_FL | INPUT_BIT_BUMPER_FR | INPUT_BIT_BUMPER_RL | INPUT_BIT_BUMPER_RR)

/* Driver Functions */

/**
  * @brief Standard initialization (Backwards-compatible)
  *        If hi2c is non-NULL, checks hi2c first. If not found,
  *        auto-detects across active I2C peripherals (I2C1/I2C2).
  */
HAL_StatusTypeDef PCF8574_Init(I2C_HandleTypeDef *hi2c);

/**
  * @brief Auto-discovery initialization across all available I2C buses:
  *        1. Checks hi2c3 (PCB mode: 0x20 and 0x21).
  *        2. If not found, checks hi2c1 (DO @ 0x24) and hi2c2 (DI @ 0x24).
  *        3. Writes failsafe 0xFF to DO (all relays OFF) and 0xFF to DI (weak pull-up).
  */
HAL_StatusTypeDef PCF8574_InitAuto(I2C_HandleTypeDef *hi2c1_handle,
                                   I2C_HandleTypeDef *hi2c2_handle,
                                   I2C_HandleTypeDef *hi2c3_handle);

/**
  * @brief Explicitly configure separate I2C buses and addresses for DO and DI
  */
void PCF8574_Configure(I2C_HandleTypeDef *do_i2c, uint8_t do_addr,
                       I2C_HandleTypeDef *di_i2c, uint8_t di_addr);

/**
  * @brief Write active-high logical mask to 8 Relays (Inverted to active-LOW on hardware)
  * @param hi2c: If non-NULL, used as fallback if DO is not configured.
  */
HAL_StatusTypeDef PCF8574_WriteRelays(I2C_HandleTypeDef *hi2c, uint8_t active_high_relay_mask);

/**
  * @brief Set state of individual relay (0 to 7)
  */
HAL_StatusTypeDef PCF8574_SetRelay(I2C_HandleTypeDef *hi2c, uint8_t relay_index, bool state);

/**
  * @brief Read 8 inputs from Digital Input expander (Active-LOW inverted to active-HIGH flags)
  * @param hi2c: If non-NULL, used as fallback if DI is not configured.
  */
HAL_StatusTypeDef PCF8574_ReadInputs(I2C_HandleTypeDef *hi2c, uint8_t *raw_active_low, uint8_t *active_high_flags);

/**
  * @brief Get last commanded relay state bitmask (logical 1 = ON)
  */
uint8_t PCF8574_GetLastRelayState(void);

/**
  * @brief Status queries for diagnostics and telemetry
  */
I2C_HandleTypeDef* PCF8574_GetDoHandle(void);
uint8_t            PCF8574_GetDoAddress(void);
I2C_HandleTypeDef* PCF8574_GetDiHandle(void);
uint8_t            PCF8574_GetDiAddress(void);
bool               PCF8574_IsDoConnected(void);
bool               PCF8574_IsDiConnected(void);

#ifdef __cplusplus
}
#endif

#endif /* PCF8574_H */
