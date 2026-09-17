/**
  ******************************************************************************
  * @file           : pcf8574.h
  * @brief          : Driver & Diagnostic Engine for PCF8574 / PCF8574T / PCF8574A
  *                   8-Bit I2C Quasi-Bidirectional I/O Expander
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
#include <stddef.h>

/* Standard PCF8574 / PCF8574T Address Range: 0x20 - 0x27 (A0, A1, A2) */
#define PCF8574_BASE_ADDR           0x20
#define PCF8574_MAX_ADDR            0x27

/* PCF8574A (Alternate Address variant) Range: 0x38 - 0x3F (A0, A1, A2) */
#define PCF8574A_BASE_ADDR          0x38
#define PCF8574A_MAX_ADDR           0x3F

/* Pin masks */
#define PCF8574_PIN_0               (1 << 0)
#define PCF8574_PIN_1               (1 << 1)
#define PCF8574_PIN_2               (1 << 2)
#define PCF8574_PIN_3               (1 << 3)
#define PCF8574_PIN_4               (1 << 4)
#define PCF8574_PIN_5               (1 << 5)
#define PCF8574_PIN_6               (1 << 6)
#define PCF8574_PIN_7               (1 << 7)
#define PCF8574_ALL_PINS            0xFF

typedef struct {
  uint8_t addr;
  bool is_type_a;
  uint8_t a0;
  uint8_t a1;
  uint8_t a2;
  char desc[64];
} PCF8574_DeviceInfo_t;

/**
  * @brief Scan an I2C bus and return all detected PCF8574/PCF8574A devices
  * @param hi2c: Pointer to I2C handle
  * @param devices: Array to store detected devices
  * @param max_devs: Capacity of devices array
  * @return Number of detected devices
  */
uint8_t PCF8574_ScanAll(I2C_HandleTypeDef *hi2c, PCF8574_DeviceInfo_t *devices, uint8_t max_devs);

/**
  * @brief Scan an I2C bus for PCF8574 or PCF8574A chips (first match)
  * @param hi2c: Pointer to I2C handle (e.g. &hi2c1)
  * @param found_addr: Pointer to store first detected 7-bit address
  * @param addr_desc: Optional string buffer for descriptive name & jumper state
  * @param desc_size: Size of addr_desc buffer
  * @retval HAL_OK if at least one PCF8574 is found, HAL_ERROR otherwise
  */
HAL_StatusTypeDef PCF8574_Scan(I2C_HandleTypeDef *hi2c, uint8_t *found_addr, char *addr_desc, size_t desc_size);

/**
  * @brief Write a raw byte to PCF8574
  * @param hi2c: Pointer to I2C handle
  * @param addr_7bit: 7-bit I2C address (e.g. 0x20)
  * @param data: Byte to write (8-bit port value)
  */
HAL_StatusTypeDef PCF8574_Write(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit, uint8_t data);

/**
  * @brief Read a raw byte from PCF8574
  * @note  To read external inputs reliably, pins must first be set HIGH ('1')
  *        to turn on internal weak pull-up (~100 uA).
  * @param hi2c: Pointer to I2C handle
  * @param addr_7bit: 7-bit I2C address
  * @param data: Pointer to store read byte
  */
HAL_StatusTypeDef PCF8574_Read(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit, uint8_t *data);

/**
  * @brief Set all 8 pins to HIGH (0xFF) to configure as quasi-bidirectional inputs
  */
HAL_StatusTypeDef PCF8574_ConfigureAsInputs(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit);

/**
  * @brief Set or reset a single pin on PCF8574
  * @param hi2c: Pointer to I2C handle
  * @param addr_7bit: 7-bit I2C address
  * @param pin_index: 0 to 7
  * @param state: true (HIGH/1) or false (LOW/0)
  * @param port_shadow: Pointer to current 8-bit shadow state
  */
HAL_StatusTypeDef PCF8574_WritePin(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit, uint8_t pin_index, bool state, uint8_t *port_shadow);

/**
  * @brief Format 8-bit byte as bit string "[P7][P6][P5][P4][P3][P2][P1][P0]"
  */
void PCF8574_FormatBits(uint8_t byte_val, char *out_str, size_t max_len);

#ifdef __cplusplus
}
#endif

#endif /* PCF8574_H */
