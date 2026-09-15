/**
  ******************************************************************************
  * @file           : tfmini_s_lidar.h
  * @brief          : Header for Benewake TFmini-S Single-Point ToF LiDAR (I2C)
  *                   Mounted at Rear-Left and Rear-Right for AMR Docking Assist
  ******************************************************************************
  */

#ifndef TFMINI_S_LIDAR_H
#define TFMINI_S_LIDAR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>

#define TFMINI_S_ADDR_REAR_LEFT   0x10  /* CN15: Default 7-bit I2C Address */
#define TFMINI_S_ADDR_REAR_RIGHT  0x11  /* CN17: Reconfigured Address */

#define TFMINI_S_MIN_RANGE_M      0.10f
#define TFMINI_S_MAX_RANGE_M      12.00f
#define TFMINI_S_FOV_RAD          0.035f /* 2 degrees cone */

typedef struct {
  float    distance_m;
  uint16_t strength;
  float    temperature_c;
  bool     valid;
} TFminiS_Data_t;

HAL_StatusTypeDef TFminiS_Init(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr);
HAL_StatusTypeDef TFminiS_TriggerQuery(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr);
HAL_StatusTypeDef TFminiS_ReadData(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, TFminiS_Data_t *out_data);

#ifdef __cplusplus
}
#endif

#endif /* TFMINI_S_LIDAR_H */
