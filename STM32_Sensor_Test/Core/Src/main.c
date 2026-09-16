/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Standalone Sensor Diagnostics & Tester for AMR Control
  *                   Hardware: STM32F401CCU6 BlackPill @ 84 MHz
  *                   Outputs real-time ASCII diagnostics to USB CDC (/dev/ttyACM0)
  ******************************************************************************
  */

#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "dyp_a22_ultrasonic.h"
#include "tfmini_s_lidar.h"
#include "onboard_leds.h"
#include <stdio.h>
#include <string.h>

/* Peripheral Handles */
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

/* Diagnostic state */
static uint8_t g_i2c1_pin_scl = 1;
static uint8_t g_i2c1_pin_sda = 1;
static uint8_t g_i2c2_pin_scl = 1;
static uint8_t g_i2c2_pin_sda = 1;

/* Private function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void I2C_CheckPhysicalPins(void);
static void I2C_BusClear(void);
static void I2C_ScanBus(I2C_HandleTypeDef *hi2c, const char *bus_name, char *out_str, size_t max_len);
static void I2C_RecoverIfNeeded(I2C_HandleTypeDef *hi2c);

int main(void)
{
  /* Reset of all peripherals, Initializes Flash interface and Systick */
  HAL_Init();

  /* Configure system clock to 84 MHz, USB clock to 48 MHz */
  SystemClock_Config();

  /* Initialize GPIO clocks */
  MX_GPIO_Init();

  /* Initialize Status LED */
  OnboardLEDs_Init();
  OnboardLEDs_Set(LED_SYS_STATUS, true);

  /* Check physical line voltages on I2C pins before activating peripherals */
  I2C_CheckPhysicalPins();

  /* Clear any stuck I2C lines */
  I2C_BusClear();

  /* Initialize I2C Peripherals at 100 kHz Standard Mode */
  MX_I2C1_Init(); /* Ultrasonic Bus: PB6 (SCL), PB7 (SDA) */
  MX_I2C2_Init(); /* TFmini-S LiDAR Bus: PB10 (SCL), PB3 (SDA) */

  /* Initialize USB CDC */
  MX_USB_DEVICE_Init();

  /* Wait for USB host enumeration */
  uint32_t usb_wait = HAL_GetTick();
  while (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED && (HAL_GetTick() - usb_wait) < 3000)
  {
    HAL_Delay(50);
  }
  HAL_Delay(500);

  /* Buffer for consolidated USB transmissions */
  static char out_buf[1024];

  /* Print Welcome Banner & Physical Pin Diagnostic */
  snprintf(out_buf, sizeof(out_buf),
    "\r\n\r\n"
    "================================================================\r\n"
    "       AMR SENSOR HARDWARE DIAGNOSTICS & TESTER (v2.0)          \r\n"
    "       Target: STM32F401CCU6 @ 84 MHz (USB CDC @ 115200)        \r\n"
    "================================================================\r\n"
    "[PHYSICAL PIN VOLTAGE CHECK]\r\n"
    "  I2C1 (PB6-SCL, PB7-SDA)  : SCL=%s | SDA=%s\r\n"
    "  I2C2 (PB10-SCL, PB3-SDA) : SCL=%s | SDA=%s\r\n"
    "%s"
    "================================================================\r\n",
    g_i2c1_pin_scl ? "HIGH (3.3V OK)" : "LOW (WARNING: Missing Pull-up!)",
    g_i2c1_pin_sda ? "HIGH (3.3V OK)" : "LOW (WARNING: Missing Pull-up!)",
    g_i2c2_pin_scl ? "HIGH (3.3V OK)" : "LOW (WARNING: Missing Pull-up!)",
    g_i2c2_pin_sda ? "HIGH (3.3V OK)" : "LOW (WARNING: Missing Pull-up!)",
    (!g_i2c1_pin_scl || !g_i2c1_pin_sda || !g_i2c2_pin_scl || !g_i2c2_pin_sda)
      ? "  --> [ALERT] Satu atau lebih pin I2C terdeteksi LOW!\r\n"
        "      Pasang resistor 4.7k ohm dari pin tersebut ke 3.3V!\r\n"
      : "  --> [INFO] Semua jalur I2C bertegangan HIGH normal.\r\n"
  );
  printf("%s", out_buf);
  HAL_Delay(50);

  /* Run Initial I2C Bus Scanners */
  char scan_buf[512];
  I2C_ScanBus(&hi2c1, "I2C1 (Ultrasonic Bus)", scan_buf, sizeof(scan_buf));
  printf("%s", scan_buf);
  HAL_Delay(50);

  I2C_ScanBus(&hi2c2, "I2C2 (TFmini-S LiDAR Bus)", scan_buf, sizeof(scan_buf));
  printf("%s", scan_buf);
  HAL_Delay(50);

  printf("Starting real-time sensor loop (Update: 5 Hz)...\r\n\r\n");

  uint32_t loop_count = 0;

  while (1)
  {
    loop_count++;
    OnboardLEDs_Toggle(LED_SYS_STATUS);

    /* Run periodic I2C bus scan every ~5 seconds (25 loops) */
    if ((loop_count % 25) == 1)
    {
      char scan_buf[512];
      printf("\r\n================ [LIVE I2C BUS SCAN] ================\r\n");
      printf("[PIN VOLTAGE AT BOOT]\r\n");
      printf("  I2C1 (PB6-SCL, PB7-SDA)  : SCL=%s | SDA=%s\r\n",
             g_i2c1_pin_scl ? "HIGH (3.3V OK)" : "LOW (WARNING: Missing Pull-up!)",
             g_i2c1_pin_sda ? "HIGH (3.3V OK)" : "LOW (WARNING: Missing Pull-up!)");
      printf("  I2C2 (PB10-SCL, PB3-SDA) : SCL=%s | SDA=%s\r\n",
             g_i2c2_pin_scl ? "HIGH (3.3V OK)" : "LOW (WARNING: Missing Pull-up!)",
             g_i2c2_pin_sda ? "HIGH (3.3V OK)" : "LOW (WARNING: Missing Pull-up!)");
      HAL_Delay(20);
      I2C_ScanBus(&hi2c1, "I2C1 (Ultrasonic: PB6-SCL, PB7-SDA)", scan_buf, sizeof(scan_buf));
      printf("%s", scan_buf);
      HAL_Delay(20);
      I2C_ScanBus(&hi2c2, "I2C2 (TFmini-S: PB10-SCL, PB3-SDA)", scan_buf, sizeof(scan_buf));
      printf("%s=====================================================\r\n\r\n", scan_buf);
      HAL_Delay(20);
    }

    /* --- STEP 1: TRIGGER DYP-A22 ULTRASONIC SENSORS --- */
    DYP_A22_Trigger(&hi2c1, DYP_A22_ADDR_LEFT);
    I2C_RecoverIfNeeded(&hi2c1);

    DYP_A22_Trigger(&hi2c1, DYP_A22_ADDR_RIGHT);
    I2C_RecoverIfNeeded(&hi2c1);

    /* Wait 65 ms for acoustic pulse transmission and echo calculation */
    HAL_Delay(65);

    /* --- STEP 2: READ DYP-A22 ULTRASONIC SENSORS --- */
    DYP_A22_Data_t a22_left = {0};
    DYP_A22_Data_t a22_right = {0};

    HAL_StatusTypeDef a22_l_status = DYP_A22_ReadDistance(&hi2c1, DYP_A22_ADDR_LEFT, &a22_left);
    I2C_RecoverIfNeeded(&hi2c1);

    HAL_StatusTypeDef a22_r_status = DYP_A22_ReadDistance(&hi2c1, DYP_A22_ADDR_RIGHT, &a22_right);
    I2C_RecoverIfNeeded(&hi2c1);

    /* --- STEP 3: READ TFMINI-S LIDAR SENSORS --- */
    TFminiS_Data_t tf_left = {0};
    TFminiS_Data_t tf_right = {0};

    HAL_StatusTypeDef tf_l_status = TFminiS_ReadData(&hi2c2, TFMINI_S_ADDR_REAR_LEFT, &tf_left);
    I2C_RecoverIfNeeded(&hi2c2);

    HAL_StatusTypeDef tf_r_status = TFminiS_ReadData(&hi2c2, TFMINI_S_ADDR_REAR_RIGHT, &tf_right);
    I2C_RecoverIfNeeded(&hi2c2);

    /* --- STEP 4: FORMAT & TRANSMIT SINGLE UNIFIED CONSOLE BLOCK --- */
    snprintf(out_buf, sizeof(out_buf),
      "[#%05lu | %6lu ms]\r\n"
      "  [A22 Ultrasonic - I2C1]\r\n"
      "    Left  (0x74): %s\r\n"
      "    Right (0x75): %s\r\n"
      "  [TFmini-S LiDAR - I2C2]\r\n"
      "    Left  (0x10): %s\r\n"
      "    Right (0x11): %s\r\n\r\n",
      loop_count, HAL_GetTick(),
      (a22_l_status == HAL_OK && a22_left.valid)
        ? "VALID"
        : (a22_l_status == HAL_OK ? "OUT_OF_RANGE" : "NO_RESPONSE / NACK"),
      (a22_r_status == HAL_OK && a22_right.valid)
        ? "VALID"
        : (a22_r_status == HAL_OK ? "OUT_OF_RANGE" : "NO_RESPONSE / NACK"),
      (tf_l_status == HAL_OK && tf_left.valid)
        ? "VALID"
        : (tf_l_status == HAL_OK ? TFminiS_GetErrorString(tf_left.error_code) : "NO_RESPONSE / NACK"),
      (tf_r_status == HAL_OK && tf_right.valid)
        ? "VALID"
        : (tf_r_status == HAL_OK ? TFminiS_GetErrorString(tf_right.error_code) : "NO_RESPONSE / NACK")
    );

    /* If any sensor has valid data, format detailed numerical string */
    char num_buf[512] = {0};
    int num_offset = 0;

    if (a22_l_status == HAL_OK)
    {
      if (a22_left.valid)
      {
        num_offset += snprintf(num_buf + num_offset, sizeof(num_buf) - num_offset,
          "    >> A22 Left  : %6.2f cm (%4d mm)\r\n", a22_left.distance_m * 100.0f, (int)(a22_left.distance_m * 1000.0f));
      }
      else
      {
        num_offset += snprintf(num_buf + num_offset, sizeof(num_buf) - num_offset,
          "    >> A22 Left [TIDAK VALID]: Raw register = 0x%04X (%u mm)\r\n", a22_left.raw_mm, a22_left.raw_mm);
      }
    }
    if (a22_r_status == HAL_OK)
    {
      if (a22_right.valid)
      {
        num_offset += snprintf(num_buf + num_offset, sizeof(num_buf) - num_offset,
          "    >> A22 Right : %6.2f cm (%4d mm)\r\n", a22_right.distance_m * 100.0f, (int)(a22_right.distance_m * 1000.0f));
      }
      else
      {
        num_offset += snprintf(num_buf + num_offset, sizeof(num_buf) - num_offset,
          "    >> A22 Right [TIDAK VALID]: Raw register = 0x%04X (%u mm)\r\n", a22_right.raw_mm, a22_right.raw_mm);
      }
    }
    if (tf_l_status == HAL_OK)
    {
      if (tf_left.valid)
      {
        num_offset += snprintf(num_buf + num_offset, sizeof(num_buf) - num_offset,
          "    >> TFmini-S L: %6.2f cm (Strength: %5u, Temp: %4.1f C) [VALID]\r\n",
          tf_left.distance_m * 100.0f, tf_left.strength, tf_left.temperature_c);
      }
      else
      {
        num_offset += snprintf(num_buf + num_offset, sizeof(num_buf) - num_offset,
          "    >> TFmini-S L: %d cm (Strength: %5u, Temp: %4.1f C) [%s]\r\n",
          tf_left.distance_cm, tf_left.strength, tf_left.temperature_c,
          TFminiS_GetErrorString(tf_left.error_code));
      }
    }
    if (tf_r_status == HAL_OK)
    {
      if (tf_right.valid)
      {
        num_offset += snprintf(num_buf + num_offset, sizeof(num_buf) - num_offset,
          "    >> TFmini-S R: %6.2f cm (Strength: %5u, Temp: %4.1f C) [VALID]\r\n",
          tf_right.distance_m * 100.0f, tf_right.strength, tf_right.temperature_c);
      }
      else
      {
        num_offset += snprintf(num_buf + num_offset, sizeof(num_buf) - num_offset,
          "    >> TFmini-S R: %d cm (Strength: %5u, Temp: %4.1f C) [%s]\r\n",
          tf_right.distance_cm, tf_right.strength, tf_right.temperature_c,
          TFminiS_GetErrorString(tf_right.error_code));
      }
    }

    if (num_offset > 0)
    {
      printf("%s%s\r\n", out_buf, num_buf);
    }
    else
    {
      printf("%s", out_buf);
    }

    /* Update cycle = ~200 ms */
    HAL_Delay(165);
  }
}

/**
  * @brief Read raw digital levels on I2C pins before peripheral init
  */
static void I2C_CheckPhysicalPins(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL; /* Read true physical voltage without internal pull */
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_10 | GPIO_PIN_3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_Delay(10);

  g_i2c1_pin_scl = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET) ? 1 : 0;
  g_i2c1_pin_sda = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) ? 1 : 0;
  g_i2c2_pin_scl = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET) ? 1 : 0;
  g_i2c2_pin_sda = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET) ? 1 : 0;
}

/**
  * @brief System Clock Configuration for STM32F401CC @ 84 MHz
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Clock 9 pulses to release any slave device stuck holding SDA low
  */
static void I2C_BusClear(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_10 | GPIO_PIN_3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  for (int i = 0; i < 9; i++)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_Delay(1);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(1);
  }

  /* Generate STOP condition */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_Delay(1);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
  HAL_Delay(1);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
  HAL_Delay(1);
}

/**
  * @brief I2C1 Init: PB6 -> SCL (AF4), PB7 -> SDA (AF4), 100 kHz
  */
static void MX_I2C1_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C1_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c1);
}

/**
  * @brief I2C2 Init: PB10 -> SCL (AF4), PB3 -> SDA (AF9), 100 kHz
  */
static void MX_I2C2_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C2_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* PB10 -> SCL (AF4) */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PB3 -> SDA (AF9) */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Alternate = GPIO_AF9_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  HAL_I2C_Init(&hi2c2);
}

/**
  * @brief Scan all 7-bit addresses (0x01 to 0x7F) and format results
  */
static void I2C_ScanBus(I2C_HandleTypeDef *hi2c, const char *bus_name, char *out_str, size_t max_len)
{
  size_t offset = snprintf(out_str, max_len, "[SCAN %s]:\r\n", bus_name);
  uint8_t devices_found = 0;

  for (uint8_t addr = 1; addr < 128; addr++)
  {
    HAL_StatusTypeDef res = HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr << 1), 1, 5);
    if (res == HAL_OK)
    {
      devices_found++;
      const char *label = "Unknown";
      if (addr == DYP_A22_ADDR_LEFT)             label = "DYP-A22 Left (Default 0x74)";
      else if (addr == DYP_A22_ADDR_RIGHT)       label = "DYP-A22 Right (0x75)";
      else if (addr == TFMINI_S_ADDR_REAR_LEFT)  label = "TFmini-S Left (Default 0x10)";
      else if (addr == TFMINI_S_ADDR_REAR_RIGHT) label = "TFmini-S Right (0x11)";

      offset += snprintf(out_str + offset, max_len - offset,
        "  -> 0x%02X (8-bit: 0x%02X) [%s]\r\n", addr, (addr << 1), label);
    }
  }

  if (devices_found == 0)
  {
    snprintf(out_str + offset, max_len - offset,
      "  -> TIDAK ADA perangkat terdeteksi di bus ini!\r\n");
  }
}

static void I2C_RecoverIfNeeded(I2C_HandleTypeDef *hi2c)
{
  /* Only reset the peripheral on fatal hardware bus errors (BERR, ARLO, TIMEOUT)
   * or stuck BUSY state. Do NOT deinit on simple NACK (HAL_I2C_ERROR_AF) when a slave is absent! */
  if ((hi2c->ErrorCode & (HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_ARLO | HAL_I2C_ERROR_TIMEOUT)) != 0 ||
      hi2c->State == HAL_I2C_STATE_BUSY)
  {
    __HAL_I2C_RESET_HANDLE_STATE(hi2c);
    HAL_I2C_DeInit(hi2c);
    HAL_I2C_Init(hi2c);
  }
  else
  {
    /* Clear benign acknowledge failure flag so subsequent transfers proceed normally */
    hi2c->ErrorCode = HAL_I2C_ERROR_NONE;
  }
}

/**
  * @brief GPIO Clock Init
  */
static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

/**
  * @brief Error Handler
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
