/**
  ******************************************************************************
  * @file           : pcf8574.c
  * @brief          : Implementation of Dual PCF8574 I/O Expanders
  *                   Supports Dual Independent I2C Buses & Automatic Discovery
  ******************************************************************************
  */

#include "pcf8574.h"
#include "rtos_queues.h"
#include "task.h"

/* External handles from main.c if needed for auto-detect fallback */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;

/* Active bindings for DO (Output Relays) and DI (Input Sensors) */
static I2C_HandleTypeDef *s_pcf_do_i2c = NULL;
static uint8_t            s_pcf_do_addr = PCF8574_ADDR_OUTPUT;
static bool               s_pcf_do_connected = false;

static I2C_HandleTypeDef *s_pcf_di_i2c = NULL;
static uint8_t            s_pcf_di_addr = PCF8574_ADDR_INPUT;
static bool               s_pcf_di_connected = false;

static uint8_t            g_current_relay_state = 0x00; /* Logical state: 1=ON, 0=OFF */

static void I2C_SafeRecover(I2C_HandleTypeDef *hi2c)
{
  if (!hi2c) return;

  if ((hi2c->ErrorCode & (HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_ARLO | HAL_I2C_ERROR_TIMEOUT)) != 0 ||
      hi2c->State == HAL_I2C_STATE_BUSY)
  {
    if (hi2c->Instance == I2C1) {
      __HAL_RCC_I2C1_FORCE_RESET();
      __HAL_RCC_I2C1_RELEASE_RESET();
    } else if (hi2c->Instance == I2C2) {
      __HAL_RCC_I2C2_FORCE_RESET();
      __HAL_RCC_I2C2_RELEASE_RESET();
    } else if (hi2c->Instance == I2C3) {
      __HAL_RCC_I2C3_FORCE_RESET();
      __HAL_RCC_I2C3_RELEASE_RESET();
    }
    __HAL_I2C_RESET_HANDLE_STATE(hi2c);
    HAL_I2C_DeInit(hi2c);
    HAL_I2C_Init(hi2c);
  }
  else
  {
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
  }
}

static void I2C_BusLock(I2C_HandleTypeDef *hi2c)
{
  if (!hi2c) return;
  if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) return;

  SemaphoreHandle_t m = NULL;
  if (hi2c->Instance == I2C1) m = xI2C1Mutex;
  else if (hi2c->Instance == I2C2) m = xI2C2Mutex;
  else if (hi2c->Instance == I2C3) m = xI2C3Mutex;

  if (m != NULL)
  {
    xSemaphoreTake(m, pdMS_TO_TICKS(25));
  }
}

static void I2C_BusUnlock(I2C_HandleTypeDef *hi2c)
{
  if (!hi2c) return;
  if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) return;

  SemaphoreHandle_t m = NULL;
  if (hi2c->Instance == I2C1) m = xI2C1Mutex;
  else if (hi2c->Instance == I2C2) m = xI2C2Mutex;
  else if (hi2c->Instance == I2C3) m = xI2C3Mutex;

  if (m != NULL)
  {
    xSemaphoreGive(m);
  }
}

static bool CheckDevice(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit)
{
  if (!hi2c) return false;
  HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr_7bit << 1), 2, 5);
  I2C_SafeRecover(hi2c);
  return (status == HAL_OK);
}

void PCF8574_Configure(I2C_HandleTypeDef *do_i2c, uint8_t do_addr,
                       I2C_HandleTypeDef *di_i2c, uint8_t di_addr)
{
  s_pcf_do_i2c = do_i2c;
  s_pcf_do_addr = do_addr;
  s_pcf_do_connected = (do_i2c != NULL);

  s_pcf_di_i2c = di_i2c;
  s_pcf_di_addr = di_addr;
  s_pcf_di_connected = (di_i2c != NULL);
}

HAL_StatusTypeDef PCF8574_InitAuto(I2C_HandleTypeDef *hi2c1_handle,
                                   I2C_HandleTypeDef *hi2c2_handle,
                                   I2C_HandleTypeDef *hi2c3_handle)
{
  s_pcf_do_connected = false;
  s_pcf_di_connected = false;

  /* Priority 1: Check Dedicated I2C3 (Custom PCB Bridge Board) */
  if (hi2c3_handle != NULL)
  {
    if (CheckDevice(hi2c3_handle, PCF8574_ADDR_OUTPUT))
    {
      s_pcf_do_i2c = hi2c3_handle;
      s_pcf_do_addr = PCF8574_ADDR_OUTPUT;
      s_pcf_do_connected = true;
    }
    if (CheckDevice(hi2c3_handle, PCF8574_ADDR_INPUT))
    {
      s_pcf_di_i2c = hi2c3_handle;
      s_pcf_di_addr = PCF8574_ADDR_INPUT;
      s_pcf_di_connected = true;
    }
  }

  /* Priority 2: Check Dual-Bus Prototype Setup (I2C1 for DO, I2C2 for DI) */
  static const uint8_t search_addrs[] = {0x24, 0x20, 0x21, 0x22, 0x23, 0x25, 0x26, 0x27, 0x38, 0x3F};

  if (!s_pcf_do_connected && hi2c1_handle != NULL)
  {
    for (size_t i = 0; i < sizeof(search_addrs); i++)
    {
      if (CheckDevice(hi2c1_handle, search_addrs[i]))
      {
        s_pcf_do_i2c = hi2c1_handle;
        s_pcf_do_addr = search_addrs[i];
        s_pcf_do_connected = true;
        break;
      }
    }
  }

  if (!s_pcf_di_connected && hi2c2_handle != NULL)
  {
    for (size_t i = 0; i < sizeof(search_addrs); i++)
    {
      if (CheckDevice(hi2c2_handle, search_addrs[i]))
      {
        s_pcf_di_i2c = hi2c2_handle;
        s_pcf_di_addr = search_addrs[i];
        s_pcf_di_connected = true;
        break;
      }
    }
  }

  /* Fallback defaults if no physical response (safe simulation / disconnected state) */
  if (!s_pcf_do_i2c)
  {
    s_pcf_do_i2c = (hi2c1_handle != NULL) ? hi2c1_handle : hi2c3_handle;
    s_pcf_do_addr = PCF8574_ADDR_PROTOTYPE; /* 0x24 */
  }
  if (!s_pcf_di_i2c)
  {
    s_pcf_di_i2c = (hi2c2_handle != NULL) ? hi2c2_handle : hi2c3_handle;
    s_pcf_di_addr = PCF8574_ADDR_PROTOTYPE; /* 0x24 */
  }

  /* Rule 5.1 Failsafe: All relays MUST default to OFF (de-energized) on startup.
     Hardware circuit is Active LOW, so writing 0xFF de-energizes all relays. */
  uint8_t failsafe_byte = 0xFF;
  HAL_StatusTypeDef status_do = HAL_OK;
  if (s_pcf_do_i2c)
  {
    status_do = HAL_I2C_Master_Transmit(
        s_pcf_do_i2c,
        (uint16_t)(s_pcf_do_addr << 1),
        &failsafe_byte,
        1,
        30
    );
    I2C_SafeRecover(s_pcf_do_i2c);
  }

  /* Initialize DI with 0xFF to enable internal weak pull-up (~100 uA) for inputs */
  if (s_pcf_di_i2c)
  {
    uint8_t input_init_byte = 0xFF;
    HAL_I2C_Master_Transmit(
        s_pcf_di_i2c,
        (uint16_t)(s_pcf_di_addr << 1),
        &input_init_byte,
        1,
        30
    );
    I2C_SafeRecover(s_pcf_di_i2c);
  }

  g_current_relay_state = 0x00;
  return status_do;
}

HAL_StatusTypeDef PCF8574_Init(I2C_HandleTypeDef *hi2c)
{
  return PCF8574_InitAuto(&hi2c1, &hi2c2, hi2c);
}

HAL_StatusTypeDef PCF8574_WriteRelays(I2C_HandleTypeDef *hi2c, uint8_t active_high_relay_mask)
{
  I2C_HandleTypeDef *target_i2c = (s_pcf_do_i2c != NULL) ? s_pcf_do_i2c : hi2c;
  if (!target_i2c) return HAL_ERROR;

  uint8_t target_addr = (s_pcf_do_i2c != NULL) ? s_pcf_do_addr : PCF8574_ADDR_OUTPUT;

  /* Invert bits because hardware drivers (BC848 + PC817) are Active LOW */
  uint8_t physical_byte = (uint8_t)(~active_high_relay_mask);

  I2C_BusLock(target_i2c);
  HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(
      target_i2c,
      (uint16_t)(target_addr << 1),
      &physical_byte,
      1,
      20
  );

  if (status == HAL_OK)
  {
    g_current_relay_state = active_high_relay_mask;
  }
  else
  {
    I2C_SafeRecover(target_i2c);
  }
  I2C_BusUnlock(target_i2c);

  return status;
}

HAL_StatusTypeDef PCF8574_SetRelay(I2C_HandleTypeDef *hi2c, uint8_t relay_index, bool state)
{
  if (relay_index > 7) return HAL_ERROR;

  uint8_t mask = g_current_relay_state;
  if (state)
  {
    mask |= (uint8_t)(1 << relay_index);
  }
  else
  {
    mask &= (uint8_t)~(1 << relay_index);
  }

  return PCF8574_WriteRelays(hi2c, mask);
}

HAL_StatusTypeDef PCF8574_ReadInputs(I2C_HandleTypeDef *hi2c, uint8_t *raw_active_low, uint8_t *active_high_flags)
{
  I2C_HandleTypeDef *target_i2c = (s_pcf_di_i2c != NULL) ? s_pcf_di_i2c : hi2c;
  if (!target_i2c) return HAL_ERROR;

  uint8_t target_addr = (s_pcf_di_i2c != NULL) ? s_pcf_di_addr : PCF8574_ADDR_INPUT;

  uint8_t data = 0xFF;
  I2C_BusLock(target_i2c);
  HAL_StatusTypeDef status = HAL_I2C_Master_Receive(
      target_i2c,
      (uint16_t)(target_addr << 1),
      &data,
      1,
      20
  );

  if (status == HAL_OK)
  {
    if (raw_active_low)
    {
      *raw_active_low = data;
    }
    if (active_high_flags)
    {
      /* Active LOW inverted: bit=1 means switch is triggered/pressed */
      *active_high_flags = (uint8_t)(~data);
    }
  }
  else
  {
    I2C_SafeRecover(target_i2c);
  }
  I2C_BusUnlock(target_i2c);

  return status;
}

uint8_t PCF8574_GetLastRelayState(void)
{
  return g_current_relay_state;
}

I2C_HandleTypeDef* PCF8574_GetDoHandle(void) { return s_pcf_do_i2c; }
uint8_t            PCF8574_GetDoAddress(void) { return s_pcf_do_addr; }
I2C_HandleTypeDef* PCF8574_GetDiHandle(void) { return s_pcf_di_i2c; }
uint8_t            PCF8574_GetDiAddress(void) { return s_pcf_di_addr; }
bool               PCF8574_IsDoConnected(void) { return s_pcf_do_connected; }
bool               PCF8574_IsDiConnected(void) { return s_pcf_di_connected; }
