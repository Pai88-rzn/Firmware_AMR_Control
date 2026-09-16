/**
  ******************************************************************************
  * @file           : main_converter.c
  * @brief          : Standalone Sensor Configurator & Address Utility
  *                   - Converts TFmini-S LiDAR from 0x10 -> 0x11 (Rear-Right)
  *                   - Monitors DYP-A22 Ultrasonic (0x74, 0x75)
  *                   Target: STM32F401CCU6 BlackPill
  ******************************************************************************
  */

#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "onboard_leds.h"
#include "dyp_a22_ultrasonic.h"
#include "tfmini_s_lidar.h"
#include <stdio.h>
#include <string.h>

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void I2C_BusClear(void);
static void I2C_RecoverIfNeeded(I2C_HandleTypeDef *hi2c);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  OnboardLEDs_Init();
  OnboardLEDs_Set(LED_SYS_STATUS, true);

  /* Clear I2C buses and initialize peripherals at 100 kHz */
  I2C_BusClear();
  MX_I2C1_Init(); /* Ultrasonic: PB6 (SCL), PB7 (SDA) */
  MX_I2C2_Init(); /* TFmini-S:   PB10 (SCL), PB3 (SDA) */

  /* Initialize USB CDC */
  MX_USB_DEVICE_Init();

  /* Wait 2 seconds for USB host enumeration */
  HAL_Delay(2000);

  printf("\r\n\r\n");
  printf("===============================================================\r\n");
  printf("     TFMINI-S LIDAR I2C ADDRESS CHANGER (0x10 -> 0x11)         \r\n");
  printf("     Target: STM32F401CCU6 BlackPill                           \r\n");
  printf("===============================================================\r\n");
  printf("PANDUAN KONEKSI SENSOR TFMINI-S KANAN:\r\n");
  printf("  1. Pastikan TFmini-S KIRI (0x10) SUDAH DICABUT sementara!\r\n");
  printf("  2. Hubungkan TFmini-S KEDUA (Kanan) ke pin:\r\n");
  printf("     - Kabel Hijau (SCL) -> Pin PB10\r\n");
  printf("     - Kabel Putih (SDA) -> Pin PB3\r\n");
  printf("     - Kabel Merah (VCC) -> Pin 5V (Pastikan ada cahaya ungu!)\r\n");
  printf("     - Kabel Hitam (GND) -> Pin GND\r\n");
  printf("===============================================================\r\n\r\n");

  uint8_t tf_converted = 0;
  uint32_t last_print = 0;

  while (1)
  {
    OnboardLEDs_Toggle(LED_SYS_STATUS);

    if ((HAL_GetTick() - last_print) >= 1200)
    {
      last_print = HAL_GetTick();

      /* 1. PHYSICAL PIN VOLTAGE CHECK */
      uint8_t scl1_lvl = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET) ? 1 : 0;
      uint8_t sda1_lvl = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) ? 1 : 0;
      uint8_t scl2_lvl = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET) ? 1 : 0;
      uint8_t sda2_lvl = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET) ? 1 : 0;

      printf("---------------------------------------------------\r\n");
      printf("[VOLTASE PIN] I2C1: SCL=%s | SDA=%s  ||  I2C2: SCL=%s | SDA=%s\r\n",
             scl1_lvl ? "HIGH(3.3V)" : "LOW(DROP!)",
             sda1_lvl ? "HIGH(3.3V)" : "LOW(DROP!)",
             scl2_lvl ? "HIGH(3.3V)" : "LOW(DROP!)",
             sda2_lvl ? "HIGH(3.3V)" : "LOW(DROP!)");

      /* 2. SCAN & CONFIGURE I2C2 (TFMINI-S) */
      printf("--- [SCAN I2C2 (PB10/PB3 - TFMINI-S)] ---\r\n");

      /* Check if sensor is at 0x10 */
      if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(0x10 << 1), 2, 25) == HAL_OK)
      {
        printf("  -> Terdeteksi TFmini-S di alamat 0x10 (Default)!\r\n");

        if (tf_converted == 0)
        {
          printf("\r\n  [MEMPROSES] Mengubah alamat TFmini-S dari 0x10 -> 0x11...\r\n");

          /* Step 1: Send Set I2C Address to 0x11 (Command: 5A 05 0B 11 7B) */
          HAL_StatusTypeDef st_addr = TFminiS_SetI2CAddress(&hi2c2, 0x10, 0x11);
          HAL_Delay(100);

          /* Step 2: Send Save Settings command to both 0x11 and 0x10 */
          TFminiS_SaveSettings_I2C(&hi2c2, 0x11);
          HAL_Delay(50);
          TFminiS_SaveSettings_I2C(&hi2c2, 0x10);
          HAL_Delay(100);

          /* Step 3: Send Soft Reset */
          TFminiS_SoftReset_I2C(&hi2c2, 0x11);
          HAL_Delay(300);

          if (st_addr == HAL_OK)
          {
            printf("  -> Perintah ganti alamat ke 0x11 dan SIMPAN berhasil dikirim!\r\n");
            tf_converted = 1;
          }
          else
          {
            printf("  -> [GAGAL] Gagal mengirim perintah ganti alamat (Status: %d)\r\n", st_addr);
          }
        }
      }
      I2C_RecoverIfNeeded(&hi2c2);

      /* Check if sensor is responding at 0x11 */
      if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(0x11 << 1), 2, 25) == HAL_OK)
      {
        TFminiS_Data_t tf11_data = {0};
        HAL_StatusTypeDef read_st = TFminiS_ReadData(&hi2c2, 0x11, &tf11_data);
        I2C_RecoverIfNeeded(&hi2c2);

        if (read_st == HAL_OK)
        {
          printf("\r\n===============================================================\r\n");
          printf("--> [SUKSES BESAR!] TFMINI-S KANAN AKTIF DI ALAMAT 0x11!\r\n");
          printf("    Jarak terukur   : %d cm (%.2f m)\r\n", tf11_data.distance_cm, tf11_data.distance_m);
          printf("    Kekuatan sinyal : %u\r\n", tf11_data.strength);
          printf("    Suhu chip       : %.1f C\r\n", tf11_data.temperature_c);
          printf("    Status          : %s\r\n", tf11_data.valid ? "VALID" : "OUT_OF_RANGE / BLIND_ZONE");
          printf("===============================================================\r\n");
          printf("LANGKAH SELANJUTNYA:\r\n");
          printf("  1. Alamat 0x11 sudah tersimpan PERMANEN di sensor kanan!\r\n");
          printf("  2. Sekarang pasang KEDUA sensor TFmini-S ke PB10 & PB3:\r\n");
          printf("     - TFmini-S Kiri  : 0x10\r\n");
          printf("     - TFmini-S Kanan : 0x11\r\n");
          printf("  3. Dan pasang KEDUA sensor Ultrasonik ke PB6 & PB7:\r\n");
          printf("     - Ultrasonik Kiri  : 0x74\r\n");
          printf("     - Ultrasonik Kanan : 0x75\r\n");
          printf("  4. Flash firmware utama: ./tools/flash_bridge_stlink.sh\r\n");
          printf("===============================================================\r\n\r\n");
        }
        else
        {
          printf("  -> Terdeteksi ACK di 0x11, membaca data...\r\n");
        }
      }
      else
      {
        if (tf_converted == 1)
        {
          printf("  -> Menunggu sensor 0x11 aktif (Jika belum terbaca, coba cabut & colok kabel 5V TFmini-S sekali lagi)\r\n");
        }
        else
        {
          printf("  -> Belum terdeteksi di 0x11 (Menunggu sensor di 0x10 untuk diubah)\r\n");
        }
      }
      I2C_RecoverIfNeeded(&hi2c2);

      /* 3. ALSO REPORT ULTRASONIC SENSORS ON I2C1 */
      printf("--- [SCAN I2C1 (PB6/PB7 - ULTRASONIC)] ---\r\n");
      DYP_A22_Data_t dyp_data = {0};
      if (DYP_A22_ReadDistance(&hi2c1, 0x74, &dyp_data) == HAL_OK && dyp_data.valid)
      {
        printf("  -> Sonar Kiri  (0x74): %.2f cm\r\n", dyp_data.distance_m * 100.0f);
      }
      I2C_RecoverIfNeeded(&hi2c1);

      if (DYP_A22_ReadDistance(&hi2c1, 0x75, &dyp_data) == HAL_OK && dyp_data.valid)
      {
        printf("  -> Sonar Kanan (0x75): %.2f cm\r\n", dyp_data.distance_m * 100.0f);
      }
      I2C_RecoverIfNeeded(&hi2c1);

      printf("---------------------------------------------------\r\n\r\n");
    }

    HAL_Delay(100);
  }
}

static void I2C_RecoverIfNeeded(I2C_HandleTypeDef *hi2c)
{
  if (!hi2c) return;
  if ((hi2c->ErrorCode & (HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_ARLO | HAL_I2C_ERROR_TIMEOUT)) != 0 ||
      hi2c->State == HAL_I2C_STATE_BUSY)
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

static void MX_I2C2_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_I2C2_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
