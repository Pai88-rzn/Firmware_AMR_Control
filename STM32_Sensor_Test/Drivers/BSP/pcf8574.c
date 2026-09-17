/**
  ******************************************************************************
  * @file           : pcf8574.c
  * @brief          : Driver implementation for PCF8574 / PCF8574T / PCF8574A
  ******************************************************************************
  */

#include "pcf8574.h"
#include <stdio.h>
#include <string.h>

uint8_t PCF8574_ScanAll(I2C_HandleTypeDef *hi2c, PCF8574_DeviceInfo_t *devices, uint8_t max_devs)
{
  if (!hi2c || !devices || max_devs == 0) return 0;

  static const uint8_t candidate_addrs[] = {
    0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
    0x38, 0x39, 0x3A, 0x3B, 0x3C, 0x3D, 0x3E, 0x3F
  };

  uint8_t count = 0;
  for (size_t i = 0; i < sizeof(candidate_addrs) && count < max_devs; i++)
  {
    uint8_t addr = candidate_addrs[i];
    if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr << 1), 2, 5) == HAL_OK)
    {
      bool is_type_a = (addr >= 0x38);
      uint8_t a_offset = is_type_a ? (addr - 0x38) : (addr - 0x20);
      uint8_t a0 = (a_offset & 0x01) ? 1 : 0;
      uint8_t a1 = (a_offset & 0x02) ? 1 : 0;
      uint8_t a2 = (a_offset & 0x04) ? 1 : 0;

      devices[count].addr = addr;
      devices[count].is_type_a = is_type_a;
      devices[count].a0 = a0;
      devices[count].a1 = a1;
      devices[count].a2 = a2;

      snprintf(devices[count].desc, sizeof(devices[count].desc),
        "%s @ 0x%02X (A0=%s, A1=%s, A2=%s)",
        is_type_a ? "PCF8574A" : "PCF8574/T",
        addr,
        a0 ? "VCC" : "GND",
        a1 ? "VCC" : "GND",
        a2 ? "VCC" : "GND"
      );

      count++;
    }
    else
    {
      if (hi2c->State == HAL_I2C_STATE_BUSY || (hi2c->ErrorCode & (HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_ARLO | HAL_I2C_ERROR_TIMEOUT)))
      {
        __HAL_I2C_RESET_HANDLE_STATE(hi2c);
        HAL_I2C_DeInit(hi2c);
        HAL_I2C_Init(hi2c);
      }
      else
      {
        hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
      }
    }
  }
  return count;
}

HAL_StatusTypeDef PCF8574_Scan(I2C_HandleTypeDef *hi2c, uint8_t *found_addr, char *addr_desc, size_t desc_size)
{
  PCF8574_DeviceInfo_t devs[1];
  uint8_t count = PCF8574_ScanAll(hi2c, devs, 1);
  if (count > 0)
  {
    if (found_addr) *found_addr = devs[0].addr;
    if (addr_desc && desc_size > 0)
    {
      snprintf(addr_desc, desc_size, "%s", devs[0].desc);
    }
    return HAL_OK;
  }

  if (addr_desc && desc_size > 0)
  {
    snprintf(addr_desc, desc_size, "None detected in 0x20-0x27 / 0x38-0x3F");
  }
  return HAL_ERROR;
}

HAL_StatusTypeDef PCF8574_Write(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit, uint8_t data)
{
  if (!hi2c) return HAL_ERROR;

  return HAL_I2C_Master_Transmit(
      hi2c,
      (uint16_t)(addr_7bit << 1),
      &data,
      1,
      20
  );
}

HAL_StatusTypeDef PCF8574_Read(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit, uint8_t *data)
{
  if (!hi2c || !data) return HAL_ERROR;

  return HAL_I2C_Master_Receive(
      hi2c,
      (uint16_t)(addr_7bit << 1),
      data,
      1,
      20
  );
}

HAL_StatusTypeDef PCF8574_ConfigureAsInputs(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit)
{
  /* PCF8574 quasi-bidirectional input mode: write 0xFF to enable weak pull-ups */
  uint8_t all_high = 0xFF;
  return PCF8574_Write(hi2c, addr_7bit, all_high);
}

HAL_StatusTypeDef PCF8574_WritePin(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit, uint8_t pin_index, bool state, uint8_t *port_shadow)
{
  if (pin_index > 7 || !port_shadow) return HAL_ERROR;

  if (state)
  {
    *port_shadow |= (uint8_t)(1 << pin_index);
  }
  else
  {
    *port_shadow &= (uint8_t)~(1 << pin_index);
  }

  return PCF8574_Write(hi2c, addr_7bit, *port_shadow);
}

void PCF8574_FormatBits(uint8_t byte_val, char *out_str, size_t max_len)
{
  if (!out_str || max_len == 0) return;

  snprintf(out_str, max_len,
    "[%d %d %d %d %d %d %d %d]",
    (byte_val >> 7) & 1,
    (byte_val >> 6) & 1,
    (byte_val >> 5) & 1,
    (byte_val >> 4) & 1,
    (byte_val >> 3) & 1,
    (byte_val >> 2) & 1,
    (byte_val >> 1) & 1,
    (byte_val >> 0) & 1
  );
}
