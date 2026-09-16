/**
  ******************************************************************************
  * @file           : dyp_a22_ultrasonic.h
  * @brief          : Header for Dianyingpu DYP-A22 Ultrasonic Sensor (I2C)
  *                   Model: DYP-A22YYCW-V1.0 (IIC Variant)
  ******************************************************************************
  */

#ifndef DYP_A22_ULTRASONIC_H
#define DYP_A22_ULTRASONIC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

/* I2C 7-bit addresses (8-bit write address >> 1) */
#define DYP_A22_ADDR_LEFT       0x74  /* CN14: Default 8-bit 0xE8 -> 7-bit 0x74 */
#define DYP_A22_ADDR_RIGHT      0x75  /* CN16: Reconfigured 8-bit 0xEA -> 7-bit 0x75 */

/* Register Definitions from A22-Output-Interfaces Datasheet */
#define DYP_A22_REG_VER_ID      0x00  /* Read-only: Version ID (0x0001) */
#define DYP_A22_REG_DIST_VAL    0x02  /* Read-only: Distance MSB (0x02), LSB (0x03) in mm */
#define DYP_A22_REG_SLAVE_ADDR  0x05  /* Read/Write: Slave address */
#define DYP_A22_REG_NOISE_LVL   0x06  /* Read/Write: Power noise level (1..5) */
#define DYP_A22_REG_ANGLE_LVL   0x07  /* Read/Write: Angle level (1=30 deg .. 4=60 deg) */
#define DYP_A22_REG_CMD         0x10  /* Write-only: Ranging instruction control */

/* Command Codes */
#define DYP_A22_CMD_TRIGGER_L4  0xB4  /* Level 4: up to 350cm, returns mm */
#define DYP_A22_CMD_TRIGGER_L3  0xB8  /* Level 3: up to 250cm, returns mm */
#define DYP_A22_CMD_TRIGGER_L2  0xBC  /* Level 2: up to 150cm, returns mm */
#define DYP_A22_CMD_TRIGGER_L1  0xBD  /* Level 1: up to 50cm, returns mm */

#define DYP_A22_MIN_RANGE_M     0.02f /* 2 cm */
#define DYP_A22_MAX_RANGE_M     3.50f /* 350 cm */
#define DYP_A22_FOV_RAD         1.047f /* 60 degrees */

typedef struct {
  float    distance_m;
  uint16_t raw_mm;
  bool     valid;
} DYP_A22_Data_t;

HAL_StatusTypeDef DYP_A22_Init(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr);
HAL_StatusTypeDef DYP_A22_Trigger(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr);
HAL_StatusTypeDef DYP_A22_ReadDistance(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, DYP_A22_Data_t *out_data);

#ifdef __cplusplus
}
#endif

#endif /* DYP_A22_ULTRASONIC_H */
