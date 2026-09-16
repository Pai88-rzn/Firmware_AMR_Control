/**
  ******************************************************************************
  * @file           : tfmini_s_lidar.h
  * @brief          : Header for Benewake TFmini-S Single-Point ToF LiDAR Driver
  *                   Supports I2C and UART, adapted from Arduino/ESP32 TFminiS
  *                   library (dhrubasaha08) with complete error handling & accurate
  *                   temperature conversion.
  ******************************************************************************
  */

#ifndef TFMINI_S_LIDAR_H
#define TFMINI_S_LIDAR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

/* I2C Slave Addresses (7-bit) */
#define TFMINI_S_ADDR_REAR_LEFT   0x10  /* CN15: Default 7-bit I2C Address */
#define TFMINI_S_ADDR_REAR_RIGHT  0x11  /* CN17: Reconfigured Address */

/* Physical specifications */
#define TFMINI_S_MIN_RANGE_M      0.10f /* Blind zone: 10 cm */
#define TFMINI_S_MAX_RANGE_M      12.00f /* Max range @ 90% reflectivity */
#define TFMINI_S_FOV_RAD          0.035f /* 2 degrees cone angle (~0.035 rad) */

#define TFMINI_S_FRAME_HEADER     0x59  /* 9-byte packet sync header byte */
#define TFMINI_S_FRAME_LEN        9     /* Standard packet length */

/**
  * @brief Error codes aligned with Benewake TFmini-S specification
  *        and dhrubasaha08/TFminiS Arduino library.
  */
typedef enum {
  TFMINI_S_OK                 =  0,   /* Valid measurement, no error */
  TFMINI_S_ERR_WEAK_SIGNAL    = -1,   /* Signal strength is too low (Dist == -1 or Strength <= 100) */
  TFMINI_S_ERR_STRENGTH_SAT   = -2,   /* Signal strength saturation (Dist == -2 or 65532) */
  TFMINI_S_ERR_CHECKSUM       = -3,   /* Checksum error */
  TFMINI_S_ERR_AMBIENT_SAT    = -4,   /* Ambient light saturation (Dist == -4) */
  TFMINI_S_ERR_OUT_OF_RANGE   = -5,   /* Distance out of physical operating specs (<10cm or >12m) */
  TFMINI_S_ERR_I2C_NACK       = -6,   /* Device offline / I2C NACK / Bus Error */
  TFMINI_S_ERR_TIMEOUT        = -7,   /* Communication timeout */
  TFMINI_S_ERR_HEADER         = -8    /* Invalid frame header */
} TFminiS_ErrorCode_t;

/**
  * @brief Processed sensor data packet
  */
typedef struct {
  int16_t  distance_cm;     /* Distance in cm (negative if error code) */
  float    distance_m;      /* Distance in meters (for ROS 2 / robotics) */
  uint16_t strength;        /* Signal quality / strength flux */
  float    temperature_c;   /* Chip temperature in degrees Celsius */
  int8_t   error_code;      /* Error status (TFminiS_ErrorCode_t) */
  bool     valid;           /* True if measurement is valid and within range */
} TFminiS_Data_t;

/**
  * @brief Stream parser state machine for UART streams / RX ISR
  */
typedef struct {
  uint8_t  state;           /* 0: Header1, 1: Header2, 2: Payload */
  uint8_t  buf_idx;
  uint8_t  rx_buf[TFMINI_S_FRAME_LEN];
} TFminiS_Parser_t;

/* --- Core Decoding & Utility Functions (Transport Independent) --- */
bool TFminiS_ParseFrame(const uint8_t *frame_buf, TFminiS_Data_t *out_data);
void TFminiS_ParserInit(TFminiS_Parser_t *parser);
bool TFminiS_ProcessByte(TFminiS_Parser_t *parser, uint8_t byte, TFminiS_Data_t *out_data);
const char* TFminiS_GetErrorString(int8_t error_code);

/* --- I2C Interface Functions --- */
HAL_StatusTypeDef TFminiS_Init(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr);
HAL_StatusTypeDef TFminiS_TriggerQuery(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr);
HAL_StatusTypeDef TFminiS_ReceiveFrame(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, TFminiS_Data_t *out_data);
HAL_StatusTypeDef TFminiS_ReadData(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, TFminiS_Data_t *out_data);

/* --- UART Interface Functions --- */
HAL_StatusTypeDef TFminiS_ReadUART(UART_HandleTypeDef *huart, TFminiS_Data_t *out_data, uint32_t timeout_ms);

/* --- Configuration Commands --- */
HAL_StatusTypeDef TFminiS_SetI2CMode_UART(UART_HandleTypeDef *huart);
HAL_StatusTypeDef TFminiS_SetUARTMode_I2C(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr);
HAL_StatusTypeDef TFminiS_SetI2CAddress(I2C_HandleTypeDef *hi2c, uint8_t current_addr, uint8_t new_addr);
HAL_StatusTypeDef TFminiS_SaveSettings_I2C(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr);
HAL_StatusTypeDef TFminiS_SoftReset_I2C(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr);

#ifdef __cplusplus
}
#endif

#endif /* TFMINI_S_LIDAR_H */
