/**
  ******************************************************************************
  * @file           : tfmini_s_lidar.c
  * @brief          : Implementation for Benewake TFmini-S ToF LiDAR (I2C)
  ******************************************************************************
  */

#include "tfmini_s_lidar.h"

/* Standard Benewake obtain data frame: 0x5A, 0x05, 0x00, 0x01, 0x60 */
static const uint8_t g_tfmini_query_cmd[5] = { 0x5A, 0x05, 0x00, 0x01, 0x60 };

HAL_StatusTypeDef TFminiS_Init(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr)
{
  if (!hi2c) return HAL_ERROR;
  /* Test if device responds on I2C address */
  return HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(i2c_addr << 1), 2, 50);
}

HAL_StatusTypeDef TFminiS_TriggerQuery(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr)
{
  if (!hi2c) return HAL_ERROR;

  return HAL_I2C_Master_Transmit(
      hi2c,
      (uint16_t)(i2c_addr << 1),
      (uint8_t *)g_tfmini_query_cmd,
      sizeof(g_tfmini_query_cmd),
      20
  );
}

HAL_StatusTypeDef TFminiS_ReadData(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, TFminiS_Data_t *out_data)
{
  if (!hi2c || !out_data) return HAL_ERROR;

  out_data->valid = false;
  out_data->distance_m = 0.0f;
  out_data->strength = 0;
  out_data->temperature_c = 0.0f;

  /* Step 1: Send query command */
  HAL_StatusTypeDef status = TFminiS_TriggerQuery(hi2c, i2c_addr);
  if (status != HAL_OK)
  {
    return status;
  }

  /* Small inter-frame turn-around */
  for (volatile int i = 0; i < 500; ++i) { __NOP(); }

  /* Step 2: Read 9-byte data packet */
  uint8_t rx_buf[9] = {0};
  status = HAL_I2C_Master_Receive(
      hi2c,
      (uint16_t)(i2c_addr << 1),
      rx_buf,
      9,
      30
  );

  if (status != HAL_OK)
  {
    return status;
  }

  /* Validate frame header 0x59 0x59 */
  if (rx_buf[0] != 0x59 || rx_buf[1] != 0x59)
  {
    return HAL_ERROR;
  }

  /* Verify checksum (sum of bytes 0..7 matches byte 8) */
  uint8_t checksum = 0;
  for (int i = 0; i < 8; i++)
  {
    checksum = (uint8_t)(checksum + rx_buf[i]);
  }
  if (checksum != rx_buf[8])
  {
    return HAL_ERROR;
  }

  /* Parse distance in cm */
  uint16_t dist_cm = (uint16_t)((rx_buf[3] << 8) | rx_buf[2]);
  uint16_t strength = (uint16_t)((rx_buf[5] << 8) | rx_buf[4]);
  int16_t raw_temp = (int16_t)((rx_buf[7] << 8) | rx_buf[6]);

  out_data->distance_m = (float)dist_cm * 0.01f;
  out_data->strength = strength;
  out_data->temperature_c = (float)raw_temp * 0.1f;

  /* Validity check: Signal strength > 100 and distance within physical specs */
  if (out_data->distance_m >= TFMINI_S_MIN_RANGE_M &&
      out_data->distance_m <= TFMINI_S_MAX_RANGE_M &&
      strength >= 100)
  {
    out_data->valid = true;
  }

  return HAL_OK;
}
