/**
  ******************************************************************************
  * @file           : tfmini_s_lidar.c
  * @brief          : Implementation for Benewake TFmini-S ToF LiDAR
  *                   Adapted from dhrubasaha08/TFminiS Arduino/ESP32 library.
  ******************************************************************************
  */

#include "tfmini_s_lidar.h"
#include <string.h>

/* Standard Benewake Commands */
static const uint8_t CMD_OBTAIN_DATA_CM[5] = { 0x5A, 0x05, 0x00, 0x01, 0x60 };
static const uint8_t CMD_SET_I2C[5]        = { 0x5A, 0x05, 0x0A, 0x01, 0x6A };
static const uint8_t CMD_SET_UART[5]       = { 0x5A, 0x05, 0x0A, 0x00, 0x69 };
static const uint8_t CMD_SAVE_SETTINGS[4]  = { 0x5A, 0x04, 0x11, 0x6F };
static const uint8_t CMD_SOFT_RESET[4]     = { 0x5A, 0x04, 0x02, 0x60 };

/* ========================================================================== */
/*                   CORE DECODING & PROTOCOL ENGINE                         */
/* ========================================================================== */

/**
  * @brief  Parse a standard 9-byte Benewake TFmini-S frame.
  * @param  frame_buf: Pointer to 9-byte raw packet.
  * @param  out_data:  Pointer to destination structure.
  * @return true if packet header & checksum are valid, false otherwise.
  */
bool TFminiS_ParseFrame(const uint8_t *frame_buf, TFminiS_Data_t *out_data)
{
  if (!frame_buf || !out_data) return false;

  out_data->valid = false;
  out_data->distance_cm = 0;
  out_data->distance_m = 0.0f;
  out_data->strength = 0;
  out_data->temperature_c = 0.0f;

  /* Step 1: Validate 2-byte header 0x59 0x59 */
  if (frame_buf[0] != TFMINI_S_FRAME_HEADER || frame_buf[1] != TFMINI_S_FRAME_HEADER)
  {
    out_data->error_code = TFMINI_S_ERR_HEADER;
    return false;
  }

  /* Step 2: Validate 8-bit checksum: sum(bytes 0..7) == byte 8 */
  uint8_t checksum = 0;
  for (int i = 0; i < 8; i++)
  {
    checksum = (uint8_t)(checksum + frame_buf[i]);
  }
  if (checksum != frame_buf[8])
  {
    out_data->error_code = TFMINI_S_ERR_CHECKSUM;
    return false;
  }

  /* Step 3: Extract distance, strength, and chip temperature */
  int16_t raw_dist = (int16_t)((frame_buf[3] << 8) | frame_buf[2]);
  uint16_t strength = (uint16_t)((frame_buf[5] << 8) | frame_buf[4]);
  int16_t raw_temp = (int16_t)((frame_buf[7] << 8) | frame_buf[6]);

  out_data->distance_cm = raw_dist;
  out_data->distance_m = (float)raw_dist * 0.01f;
  out_data->strength = strength;

  /* Formula from Benewake manual & TFminiS Arduino library:
   * Temperature = raw / 8.0 - 256 (°C) */
  out_data->temperature_c = ((float)raw_temp / 8.0f) - 256.0f;

  /* Step 4: Map status / error conditions as per Benewake spec & TFminiS lib */
  if (raw_dist == -1 || (uint16_t)raw_dist == 65535 || strength <= 100)
  {
    out_data->error_code = TFMINI_S_ERR_WEAK_SIGNAL;
    out_data->valid = false;
  }
  else if (raw_dist == -2 || (uint16_t)raw_dist == 65534 || (uint16_t)raw_dist == 65532)
  {
    out_data->error_code = TFMINI_S_ERR_STRENGTH_SAT;
    out_data->valid = false;
  }
  else if (raw_dist == -4 || (uint16_t)raw_dist == 65532)
  {
    out_data->error_code = TFMINI_S_ERR_AMBIENT_SAT;
    out_data->valid = false;
  }
  else if (out_data->distance_m < TFMINI_S_MIN_RANGE_M || out_data->distance_m > TFMINI_S_MAX_RANGE_M)
  {
    out_data->error_code = TFMINI_S_ERR_OUT_OF_RANGE;
    out_data->valid = false;
  }
  else
  {
    out_data->error_code = TFMINI_S_OK;
    out_data->valid = true;
  }

  return true;
}

/**
  * @brief  Initialize the byte stream parser.
  */
void TFminiS_ParserInit(TFminiS_Parser_t *parser)
{
  if (!parser) return;
  parser->state = 0;
  parser->buf_idx = 0;
  memset(parser->rx_buf, 0, sizeof(parser->rx_buf));
}

/**
  * @brief  Feed one byte into the stream parser state machine (for UART RX / ISR).
  * @return true when a complete valid packet is decoded, false otherwise.
  */
bool TFminiS_ProcessByte(TFminiS_Parser_t *parser, uint8_t byte, TFminiS_Data_t *out_data)
{
  if (!parser) return false;

  switch (parser->state)
  {
    case 0: /* Expect 1st sync header byte: 0x59 */
      if (byte == TFMINI_S_FRAME_HEADER)
      {
        parser->rx_buf[0] = byte;
        parser->state = 1;
      }
      break;

    case 1: /* Expect 2nd sync header byte: 0x59 */
      if (byte == TFMINI_S_FRAME_HEADER)
      {
        parser->rx_buf[1] = byte;
        parser->buf_idx = 2;
        parser->state = 2;
      }
      else
      {
        parser->state = 0;
      }
      break;

    case 2: /* Accumulate payload bytes (dist, strength, temp, checksum) */
      parser->rx_buf[parser->buf_idx++] = byte;
      if (parser->buf_idx >= TFMINI_S_FRAME_LEN)
      {
        parser->state = 0; /* Reset state for subsequent frame */
        return TFminiS_ParseFrame(parser->rx_buf, out_data);
      }
      break;

    default:
      parser->state = 0;
      break;
  }

  return false;
}

/**
  * @brief  Translate error codes into human-readable strings.
  *         Aligned with TFminiS::getErrorString from dhrubasaha08/TFminiS.
  */
const char* TFminiS_GetErrorString(int8_t error_code)
{
  switch (error_code)
  {
    case TFMINI_S_OK:
      return "No error.";
    case TFMINI_S_ERR_WEAK_SIGNAL:
      return "Signal strength is too low.";
    case TFMINI_S_ERR_STRENGTH_SAT:
      return "Signal strength saturation.";
    case TFMINI_S_ERR_CHECKSUM:
      return "Checksum error.";
    case TFMINI_S_ERR_AMBIENT_SAT:
      return "Ambient light saturation.";
    case TFMINI_S_ERR_OUT_OF_RANGE:
      return "Out of measurement range.";
    case TFMINI_S_ERR_I2C_NACK:
      return "I2C NACK / Device offline.";
    case TFMINI_S_ERR_TIMEOUT:
      return "Communication timeout.";
    case TFMINI_S_ERR_HEADER:
      return "Invalid frame header.";
    default:
      return "Unknown error.";
  }
}

/* ========================================================================== */
/*                             I2C INTERFACE                                  */
/* ========================================================================== */

HAL_StatusTypeDef TFminiS_Init(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr)
{
  if (!hi2c) return HAL_ERROR;
  /* Test if device responds on 7-bit I2C address */
  return HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(i2c_addr << 1), 3, 50);
}

/**
  * @brief  Send 5-byte obtain data frame trigger command (0x5A 0x05 0x00 0x01 0x60).
  */
HAL_StatusTypeDef TFminiS_TriggerQuery(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr)
{
  if (!hi2c) return HAL_ERROR;

  return HAL_I2C_Master_Transmit(
      hi2c,
      (uint16_t)(i2c_addr << 1),
      (uint8_t *)CMD_OBTAIN_DATA_CM,
      sizeof(CMD_OBTAIN_DATA_CM),
      25
  );
}

/**
  * @brief  Read 9-byte data packet from sensor after trigger and parse it.
  */
HAL_StatusTypeDef TFminiS_ReceiveFrame(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, TFminiS_Data_t *out_data)
{
  if (!hi2c || !out_data) return HAL_ERROR;

  uint8_t rx_buf[TFMINI_S_FRAME_LEN] = {0};
  HAL_StatusTypeDef status = HAL_I2C_Master_Receive(
      hi2c,
      (uint16_t)(i2c_addr << 1),
      rx_buf,
      TFMINI_S_FRAME_LEN,
      35
  );

  if (status != HAL_OK)
  {
    out_data->error_code = TFMINI_S_ERR_I2C_NACK;
    out_data->valid = false;
    return status;
  }

  if (!TFminiS_ParseFrame(rx_buf, out_data))
  {
    return HAL_ERROR;
  }

  return HAL_OK;
}

/**
  * @brief  Trigger query, wait internal processing time, and read data packet.
  */
HAL_StatusTypeDef TFminiS_ReadData(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr, TFminiS_Data_t *out_data)
{
  if (!hi2c || !out_data) return HAL_ERROR;

  out_data->valid = false;
  out_data->distance_cm = 0;
  out_data->distance_m = 0.0f;
  out_data->strength = 0;
  out_data->temperature_c = 0.0f;
  out_data->error_code = TFMINI_S_ERR_I2C_NACK;

  /* Step 1: Send query command */
  HAL_StatusTypeDef status = TFminiS_TriggerQuery(hi2c, i2c_addr);
  if (status != HAL_OK)
  {
    out_data->error_code = TFMINI_S_ERR_I2C_NACK;
    return status;
  }

  /* Inter-frame processing delay: Benewake TFmini-S MCU needs ~2-3 ms
   * to compute distance and prepare output buffer for I2C master. */
  HAL_Delay(3);

  /* Step 2: Read response */
  return TFminiS_ReceiveFrame(hi2c, i2c_addr, out_data);
}

/* ========================================================================== */
/*                             UART INTERFACE                                 */
/* ========================================================================== */

/**
  * @brief  Synchronously read a valid TFmini-S frame over UART with timeout.
  */
HAL_StatusTypeDef TFminiS_ReadUART(UART_HandleTypeDef *huart, TFminiS_Data_t *out_data, uint32_t timeout_ms)
{
  if (!huart || !out_data) return HAL_ERROR;

  uint32_t start_tick = HAL_GetTick();
  uint8_t rx_byte = 0;
  TFminiS_Parser_t parser;
  TFminiS_ParserInit(&parser);

  while ((HAL_GetTick() - start_tick) < timeout_ms)
  {
    if (HAL_UART_Receive(huart, &rx_byte, 1, 10) == HAL_OK)
    {
      if (TFminiS_ProcessByte(&parser, rx_byte, out_data))
      {
        return HAL_OK;
      }
    }
  }

  out_data->valid = false;
  out_data->error_code = TFMINI_S_ERR_TIMEOUT;
  return HAL_TIMEOUT;
}

/* ========================================================================== */
/*                        CONFIGURATION COMMANDS                              */
/* ========================================================================== */

HAL_StatusTypeDef TFminiS_SetI2CMode_UART(UART_HandleTypeDef *huart)
{
  if (!huart) return HAL_ERROR;
  return HAL_UART_Transmit(huart, (uint8_t *)CMD_SET_I2C, sizeof(CMD_SET_I2C), 100);
}

HAL_StatusTypeDef TFminiS_SetUARTMode_I2C(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr)
{
  if (!hi2c) return HAL_ERROR;
  return HAL_I2C_Master_Transmit(hi2c, (uint16_t)(i2c_addr << 1), (uint8_t *)CMD_SET_UART, sizeof(CMD_SET_UART), 50);
}

HAL_StatusTypeDef TFminiS_SetI2CAddress(I2C_HandleTypeDef *hi2c, uint8_t current_addr, uint8_t new_addr)
{
  if (!hi2c) return HAL_ERROR;
  uint8_t cmd[5] = { 0x5A, 0x05, 0x0B, new_addr, 0 };
  uint8_t checksum = 0;
  for (int i = 0; i < 4; i++)
  {
    checksum = (uint8_t)(checksum + cmd[i]);
  }
  cmd[4] = checksum;
  return HAL_I2C_Master_Transmit(hi2c, (uint16_t)(current_addr << 1), cmd, sizeof(cmd), 50);
}

HAL_StatusTypeDef TFminiS_SaveSettings_I2C(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr)
{
  if (!hi2c) return HAL_ERROR;
  return HAL_I2C_Master_Transmit(hi2c, (uint16_t)(i2c_addr << 1), (uint8_t *)CMD_SAVE_SETTINGS, sizeof(CMD_SAVE_SETTINGS), 50);
}

HAL_StatusTypeDef TFminiS_SoftReset_I2C(I2C_HandleTypeDef *hi2c, uint8_t i2c_addr)
{
  if (!hi2c) return HAL_ERROR;
  return HAL_I2C_Master_Transmit(hi2c, (uint16_t)(i2c_addr << 1), (uint8_t *)CMD_SOFT_RESET, sizeof(CMD_SOFT_RESET), 50);
}
