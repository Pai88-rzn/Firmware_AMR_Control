/**
  ******************************************************************************
  * @file           : dyp_a22_ultrasonic.c
  * @brief          : Implementation for Dianyingpu DYP-A22 Ultrasonic Sensor (I2C)
  ******************************************************************************
  */

#include "dyp_a22_ultrasonic.h"

HAL_StatusTypeDef DYP_A22_Init(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr)
{
  if (!hi2c) return HAL_ERROR;
  /* Test if device responds on I2C address */
  return HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(i2c_addr << 1), 2, 50);
}

HAL_StatusTypeDef DYP_A22_Trigger(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr)
{
  if (!hi2c) return HAL_ERROR;

  uint8_t cmd = DYP_A22_CMD_TRIGGER_L4; /* Level 4: up to 350cm */
  return HAL_I2C_Mem_Write(
      hi2c,
      (uint16_t)(i2c_addr << 1),
      DYP_A22_REG_CMD,
      I2C_MEMADD_SIZE_8BIT,
      &cmd,
      1,
      20
  );
}

HAL_StatusTypeDef DYP_A22_ReadDistance(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, DYP_A22_Data_t *out_data)
{
  if (!hi2c || !out_data) return HAL_ERROR;

  out_data->valid = false;
  out_data->distance_m = 0.0f;

  /* Read 2 bytes from register 0x02 (Distance MSB, LSB in mm) */
  uint8_t rx_buf[2] = { 0xFF, 0xFF };
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
      hi2c,
      (uint16_t)(i2c_addr << 1),
      DYP_A22_REG_DIST_VAL,
      I2C_MEMADD_SIZE_8BIT,
      rx_buf,
      2,
      25
  );

  if (status != HAL_OK)
  {
    return status;
  }

  uint16_t raw_mm = (uint16_t)((rx_buf[0] << 8) | rx_buf[1]);

  /* Datasheet note: 0xFFFF indicates ranging in progress or invalid detection */
  if (raw_mm == 0xFFFF || raw_mm == 0x0000)
  {
    return HAL_ERROR;
  }

  out_data->distance_m = (float)raw_mm * 0.001f;

  /* Validate within operating range (0.02m to 3.50m) */
  if (out_data->distance_m >= DYP_A22_MIN_RANGE_M &&
      out_data->distance_m <= DYP_A22_MAX_RANGE_M)
  {
    out_data->valid = true;
  }

  return HAL_OK;
}
