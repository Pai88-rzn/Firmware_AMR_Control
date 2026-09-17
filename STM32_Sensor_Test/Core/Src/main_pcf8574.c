/**
  ******************************************************************************
  * @file           : main_pcf8574.c
  * @brief          : Standalone Interactive Dual PCF8574 Tester & Diagnostics
  *                   Supports Dual Independent I2C Buses:
  *                   - Bus I2C1: PB6 (SCL), PB7 (SDA)
  *                   - Bus I2C2: PB10 (SCL), PB3 (SDA)
  *                   Outputs real-time ASCII diagnostics to USB CDC (/dev/ttyACM0)
  ******************************************************************************
  */

#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "onboard_leds.h"
#include "pcf8574.h"
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

/* Dual PCF8574 Target Config (can be on the SAME or DIFFERENT I2C buses!) */
static I2C_HandleTypeDef *g_pcf_do_i2c = &hi2c1;
static const char        *g_pcf_do_bus_name = "I2C1 (PB6/PB7)";
static uint8_t            g_pcf_do_addr = 0x20;
static bool               g_pcf_do_detected = false;
static uint8_t            g_pcf_do_shadow = 0xFF; /* DO: 0xFF = all relays OFF */

static I2C_HandleTypeDef *g_pcf_di_i2c = &hi2c2;
static const char        *g_pcf_di_bus_name = "I2C2 (PB10/PB3)";
static uint8_t            g_pcf_di_addr = 0x21;
static bool               g_pcf_di_detected = false;
static uint8_t            g_last_di_val = 0xFF;

/* Test modes */
typedef enum {
  TEST_MODE_DUAL_DEMO = 0,   /* Simultaneous DO Relay Chaser + DI Live Monitoring */
  TEST_MODE_INPUT_MONITOR,   /* High-speed DI monitoring (10 Hz) with edge detection */
  TEST_MODE_RELAY_CHASER,    /* DO Active-low walking zero (Relay 1..8) */
  TEST_MODE_LED_CHASER,      /* DO Active-high walking one (P0..P7) */
  TEST_MODE_ALL_TOGGLE,      /* DO Alternates 0x00 and 0xFF */
  TEST_MODE_MANUAL           /* DO Manual key toggle (0..7, a, f) */
} TestMode_t;

static TestMode_t g_current_mode = TEST_MODE_DUAL_DEMO;

/* Private function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void I2C_CheckPhysicalPins(void);
static void I2C_BusClear(void);
static void I2C_RecoverIfNeeded(I2C_HandleTypeDef *hi2c);
static void Run_Bus_Scan(void);
static void Print_Help_Menu(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();

  OnboardLEDs_Init();
  OnboardLEDs_Set(LED_SYS_STATUS, true);

  /* Check physical voltages and clear any hung bus */
  I2C_CheckPhysicalPins();
  I2C_BusClear();

  /* Initialize both I2C Peripherals at 100 kHz Standard Mode */
  MX_I2C1_Init(); /* Bus 1: PB6 (SCL), PB7 (SDA) */
  MX_I2C2_Init(); /* Bus 2: PB10 (SCL), PB3 (SDA) */

  /* Initialize USB CDC */
  MX_USB_DEVICE_Init();

  /* Wait for USB host enumeration */
  uint32_t usb_wait = HAL_GetTick();
  while (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED && (HAL_GetTick() - usb_wait) < 3000)
  {
    HAL_Delay(50);
  }
  HAL_Delay(500);

  /* Welcome Banner */
  printf("\r\n\r\n");
  printf("====================================================================\r\n");
  printf("     DUAL-BUS PCF8574 / PCF8574T INTERACTIVE DIAGNOSTICS            \r\n");
  printf("     Bus 1 (I2C1): PB6 (SCL) & PB7 (SDA)                            \r\n");
  printf("     Bus 2 (I2C2): PB10 (SCL) & PB3 (SDA)                           \r\n");
  printf("     Target MCU: STM32F401CCU6 @ 84 MHz (USB CDC @ 115200)          \r\n");
  printf("====================================================================\r\n");

  /* Initial full scan of both buses */
  Run_Bus_Scan();

  Print_Help_Menu();

  uint32_t last_display_time = 0;
  uint32_t step_counter = 0;

  while (1)
  {
    uint32_t now = HAL_GetTick();

    /* 1. Process USB CDC User Input */
    uint8_t rx_char = 0;
    while (CDC_GetChar(&rx_char))
    {
      if (rx_char == '\r' || rx_char == '\n') continue;

      /* Manual toggle keys for DO (0..7 in manual mode) */
      if (g_current_mode == TEST_MODE_MANUAL && rx_char >= '0' && rx_char <= '7')
      {
        uint8_t pin = rx_char - '0';
        g_pcf_do_shadow ^= (uint8_t)(1 << pin);
        if (g_pcf_do_detected)
        {
          PCF8574_Write(g_pcf_do_i2c, g_pcf_do_addr, g_pcf_do_shadow);
          I2C_RecoverIfNeeded(g_pcf_do_i2c);
        }
        printf("\r\n[MANUAL TOGGLE] DO Pin P%d toggled. DO Port State = 0x%02X\r\n", pin, g_pcf_do_shadow);
      }
      else if (rx_char == '0')
      {
        g_current_mode = TEST_MODE_DUAL_DEMO;
        printf("\r\n[MODE SWITCH] Switched to: MODE 0 (Dual Simultaneous Demo - DO Chaser + DI Monitor)\r\n");
      }
      else if (rx_char == '1')
      {
        g_current_mode = TEST_MODE_INPUT_MONITOR;
        if (g_pcf_di_detected)
        {
          PCF8574_ConfigureAsInputs(g_pcf_di_i2c, g_pcf_di_addr);
          I2C_RecoverIfNeeded(g_pcf_di_i2c);
        }
        printf("\r\n[MODE SWITCH] Switched to: MODE 1 (Real-Time DI Monitor on %s @ 0x%02X)\r\n"
               "  -> Ground any DI pin (P0..P7) to test!\r\n", g_pcf_di_bus_name, g_pcf_di_addr);
      }
      else if (rx_char == '2')
      {
        g_current_mode = TEST_MODE_RELAY_CHASER;
        step_counter = 0;
        printf("\r\n[MODE SWITCH] Switched to: MODE 2 (DO Active-LOW Relay Chaser RY1..RY8 on %s @ 0x%02X)\r\n",
               g_pcf_do_bus_name, g_pcf_do_addr);
      }
      else if (rx_char == '3')
      {
        g_current_mode = TEST_MODE_LED_CHASER;
        step_counter = 0;
        printf("\r\n[MODE SWITCH] Switched to: MODE 3 (DO Active-HIGH LED Chaser P0..P7 on %s @ 0x%02X)\r\n",
               g_pcf_do_bus_name, g_pcf_do_addr);
      }
      else if (rx_char == '4')
      {
        g_current_mode = TEST_MODE_ALL_TOGGLE;
        step_counter = 0;
        printf("\r\n[MODE SWITCH] Switched to: MODE 4 (DO All ON / All OFF Toggle on %s @ 0x%02X)\r\n",
               g_pcf_do_bus_name, g_pcf_do_addr);
      }
      else if (rx_char == 'm' || rx_char == 'M')
      {
        g_current_mode = TEST_MODE_MANUAL;
        printf("\r\n[MODE SWITCH] Switched to: MODE MANUAL (DO Control on %s @ 0x%02X)\r\n"
               "  -> Press '0'..'7' to toggle pins, 'a' for All-ON (0x00), 'f' for All-OFF (0xFF)\r\n",
               g_pcf_do_bus_name, g_pcf_do_addr);
      }
      else if (rx_char == 'x' || rx_char == 'X')
      {
        /* Swap DO and DI (handles, bus names, addresses, and detection status) */
        I2C_HandleTypeDef *tmp_i2c = g_pcf_do_i2c;
        g_pcf_do_i2c = g_pcf_di_i2c;
        g_pcf_di_i2c = tmp_i2c;

        const char *tmp_name = g_pcf_do_bus_name;
        g_pcf_do_bus_name = g_pcf_di_bus_name;
        g_pcf_di_bus_name = tmp_name;

        uint8_t tmp_addr = g_pcf_do_addr;
        g_pcf_do_addr = g_pcf_di_addr;
        g_pcf_di_addr = tmp_addr;

        bool tmp_det = g_pcf_do_detected;
        g_pcf_do_detected = g_pcf_di_detected;
        g_pcf_di_detected = tmp_det;

        /* Re-initialize both */
        if (g_pcf_do_detected)
        {
          PCF8574_Write(g_pcf_do_i2c, g_pcf_do_addr, 0xFF);
          I2C_RecoverIfNeeded(g_pcf_do_i2c);
          g_pcf_do_shadow = 0xFF;
        }
        if (g_pcf_di_detected)
        {
          PCF8574_ConfigureAsInputs(g_pcf_di_i2c, g_pcf_di_addr);
          I2C_RecoverIfNeeded(g_pcf_di_i2c);
          g_last_di_val = 0xFF;
        }

        printf("\r\n[ROLES SWAPPED]\r\n"
               "  >> DO (Output Relays) : on %s @ 0x%02X (Detected: %s)\r\n"
               "  >> DI (Input Sensors) : on %s @ 0x%02X (Detected: %s)\r\n\r\n",
               g_pcf_do_bus_name, g_pcf_do_addr, g_pcf_do_detected ? "YES" : "NO",
               g_pcf_di_bus_name, g_pcf_di_addr, g_pcf_di_detected ? "YES" : "NO");
      }
      else if (rx_char == 'a' || rx_char == 'A')
      {
        g_current_mode = TEST_MODE_MANUAL;
        g_pcf_do_shadow = 0x00; /* All LOW (All relays ON if active-low) */
        if (g_pcf_do_detected)
        {
          PCF8574_Write(g_pcf_do_i2c, g_pcf_do_addr, g_pcf_do_shadow);
          I2C_RecoverIfNeeded(g_pcf_do_i2c);
        }
        printf("\r\n[MANUAL] DO Pins set to 0x00 (LOW / All Relays ON)\r\n");
      }
      else if (rx_char == 'f' || rx_char == 'F')
      {
        g_current_mode = TEST_MODE_MANUAL;
        g_pcf_do_shadow = 0xFF; /* All HIGH (All relays OFF if active-low) */
        if (g_pcf_do_detected)
        {
          PCF8574_Write(g_pcf_do_i2c, g_pcf_do_addr, g_pcf_do_shadow);
          I2C_RecoverIfNeeded(g_pcf_do_i2c);
        }
        printf("\r\n[MANUAL] DO Pins set to 0xFF (HIGH / All Relays OFF)\r\n");
      }
      else if (rx_char == 's' || rx_char == 'S')
      {
        printf("\r\n--- RE-SCANNING BOTH I2C BUSES ---\r\n");
        Run_Bus_Scan();
      }
      else if (rx_char == 'h' || rx_char == 'H' || rx_char == '?')
      {
        Print_Help_Menu();
      }
      else
      {
        printf("\r\n[Unknown command '%c'. Type 'h' for help.]\r\n", rx_char);
      }
    }

    /* 2. Execute Periodic Diagnostics based on Mode */
    uint32_t interval = 500;
    if (g_current_mode == TEST_MODE_INPUT_MONITOR)
    {
      interval = 100; /* 10 Hz for fast input response */
    }
    else if (g_current_mode == TEST_MODE_RELAY_CHASER || g_current_mode == TEST_MODE_LED_CHASER)
    {
      interval = 400;
    }
    else if (g_current_mode == TEST_MODE_ALL_TOGGLE)
    {
      interval = 1000;
    }

    if (now - last_display_time >= interval)
    {
      last_display_time = now;
      OnboardLEDs_Toggle(LED_SYS_STATUS);

      if (!g_pcf_do_detected && !g_pcf_di_detected)
      {
        printf("[%06lu ms] Belum ada PCF8574 terdeteksi! Retrying scan...\r\n", now);
        Run_Bus_Scan();
        HAL_Delay(300);
        continue;
      }

      switch (g_current_mode)
      {
        case TEST_MODE_INPUT_MONITOR:
        {
          if (g_pcf_di_detected)
          {
            uint8_t read_val = 0xFF;
            HAL_StatusTypeDef status = PCF8574_Read(g_pcf_di_i2c, g_pcf_di_addr, &read_val);
            I2C_RecoverIfNeeded(g_pcf_di_i2c);

            if (status == HAL_OK)
            {
              char bits_str[32];
              PCF8574_FormatBits(read_val, bits_str, sizeof(bits_str));

              bool changed = (read_val != g_last_di_val);
              g_last_di_val = read_val;

              if (changed)
              {
                printf("\r\n>>> [DI INPUT EVENT @ %lu ms] <<<\r\n", now);
                printf("    DI on %s (0x%02X) [P7..P0]: %s  Hex: 0x%02X\r\n",
                       g_pcf_di_bus_name, g_pcf_di_addr, bits_str, read_val);
                for (int p = 0; p < 8; p++)
                {
                  bool is_low = !((read_val >> p) & 1);
                  if (is_low)
                  {
                    printf("    -> Pin P%d is LOW  (PRESSED / GROUNDED)\r\n", p);
                  }
                }
                printf("\r\n");
              }
              else
              {
                static uint32_t s_heartbeat = 0;
                if (++s_heartbeat % 10 == 0)
                {
                  printf("[DI MONITOR on %s @ 0x%02X] Live: %s (0x%02X) | Ground any pin to test\r\n",
                         g_pcf_di_bus_name, g_pcf_di_addr, bits_str, read_val);
                }
              }
            }
            else
            {
              printf("[ERROR] Failed to read DI on %s @ 0x%02X (NACK)\r\n", g_pcf_di_bus_name, g_pcf_di_addr);
            }
          }
          else
          {
            printf("[DI NOT DETECTED] Target DI on %s @ 0x%02X tidak merespons.\r\n",
                   g_pcf_di_bus_name, g_pcf_di_addr);
          }
          break;
        }

        case TEST_MODE_RELAY_CHASER:
        {
          if (g_pcf_do_detected)
          {
            uint8_t active_relay = (step_counter % 8);
            g_pcf_do_shadow = (uint8_t)~(1 << active_relay);
            PCF8574_Write(g_pcf_do_i2c, g_pcf_do_addr, g_pcf_do_shadow);
            I2C_RecoverIfNeeded(g_pcf_do_i2c);

            char bits_str[32];
            PCF8574_FormatBits(g_pcf_do_shadow, bits_str, sizeof(bits_str));
            printf("[DO RELAY CHASER on %s @ 0x%02X] Relay #%d ON (P%d=LOW) | Port: %s (0x%02X)\r\n",
                   g_pcf_do_bus_name, g_pcf_do_addr, active_relay + 1, active_relay, bits_str, g_pcf_do_shadow);

            step_counter++;
          }
          break;
        }

        case TEST_MODE_LED_CHASER:
        {
          if (g_pcf_do_detected)
          {
            uint8_t active_pin = (step_counter % 8);
            g_pcf_do_shadow = (uint8_t)(1 << active_pin);
            PCF8574_Write(g_pcf_do_i2c, g_pcf_do_addr, g_pcf_do_shadow);
            I2C_RecoverIfNeeded(g_pcf_do_i2c);

            char bits_str[32];
            PCF8574_FormatBits(g_pcf_do_shadow, bits_str, sizeof(bits_str));
            printf("[DO LED CHASER on %s @ 0x%02X] Pin P%d=HIGH | Port: %s (0x%02X)\r\n",
                   g_pcf_do_bus_name, g_pcf_do_addr, active_pin, bits_str, g_pcf_do_shadow);

            step_counter++;
          }
          break;
        }

        case TEST_MODE_ALL_TOGGLE:
        {
          if (g_pcf_do_detected)
          {
            bool is_on = (step_counter % 2) == 0;
            g_pcf_do_shadow = is_on ? 0x00 : 0xFF;
            PCF8574_Write(g_pcf_do_i2c, g_pcf_do_addr, g_pcf_do_shadow);
            I2C_RecoverIfNeeded(g_pcf_do_i2c);

            printf("[DO ALL TOGGLE on %s @ 0x%02X] All Pins = %s (0x%02X)\r\n",
                   g_pcf_do_bus_name, g_pcf_do_addr,
                   is_on ? "LOW (0x00 - All Relays ON)" : "HIGH (0xFF - All Relays OFF)",
                   g_pcf_do_shadow);

            step_counter++;
          }
          break;
        }

        case TEST_MODE_MANUAL:
        {
          char bits_str[32];
          PCF8574_FormatBits(g_pcf_do_shadow, bits_str, sizeof(bits_str));

          printf("[DO MANUAL on %s @ 0x%02X] Written: %s (0x%02X) | Press 0-7, a, f\r\n",
                 g_pcf_do_bus_name, g_pcf_do_addr, bits_str, g_pcf_do_shadow);
          break;
        }

        case TEST_MODE_DUAL_DEMO:
        default:
        {
          /* Step DO through relays */
          uint8_t phase = (step_counter % 10);
          if (g_pcf_do_detected)
          {
            if (phase < 8)
            {
              g_pcf_do_shadow = (uint8_t)~(1 << phase);
            }
            else
            {
              g_pcf_do_shadow = 0xFF; /* All relays off */
            }
            PCF8574_Write(g_pcf_do_i2c, g_pcf_do_addr, g_pcf_do_shadow);
            I2C_RecoverIfNeeded(g_pcf_do_i2c);
          }

          /* Read DI in real time */
          uint8_t di_val = 0xFF;
          if (g_pcf_di_detected)
          {
            PCF8574_Read(g_pcf_di_i2c, g_pcf_di_addr, &di_val);
            I2C_RecoverIfNeeded(g_pcf_di_i2c);
          }

          char do_str[32];
          PCF8574_FormatBits(g_pcf_do_shadow, do_str, sizeof(do_str));

          char di_str[32];
          PCF8574_FormatBits(di_val, di_str, sizeof(di_str));

          printf("[DUAL-BUS #%04lu]\r\n", step_counter);
          if (g_pcf_do_detected)
          {
            if (phase < 8)
            {
              printf("  DO on %s (@ 0x%02X): Relay #%d ON (P%d=LOW) -> [P7..P0]: %s (0x%02X)\r\n",
                     g_pcf_do_bus_name, g_pcf_do_addr, phase + 1, phase, do_str, g_pcf_do_shadow);
            }
            else
            {
              printf("  DO on %s (@ 0x%02X): All Relays OFF        -> [P7..P0]: %s (0x%02X)\r\n",
                     g_pcf_do_bus_name, g_pcf_do_addr, do_str, g_pcf_do_shadow);
            }
          }
          else
          {
            printf("  DO on %s (@ 0x%02X): [TIDAK TERDETEKSI]\r\n", g_pcf_do_bus_name, g_pcf_do_addr);
          }

          if (g_pcf_di_detected)
          {
            printf("  DI on %s (@ 0x%02X): Inputs Live Sample     -> [P7..P0]: %s (0x%02X)%s\r\n",
                   g_pcf_di_bus_name, g_pcf_di_addr, di_str, di_val,
                   (di_val != 0xFF) ? " <--- PIN GROUNDED!" : "");
          }
          else
          {
            printf("  DI on %s (@ 0x%02X): [TIDAK TERDETEKSI - Cek kabel/pull-up]\r\n",
                   g_pcf_di_bus_name, g_pcf_di_addr);
          }
          printf("\r\n");

          step_counter++;
          break;
        }
      }
    }
  }
}

static void Run_Bus_Scan(void)
{
  printf("\r\n====================================================================\r\n");
  printf("             SCANNING DUAL I2C BUSES (I2C1 & I2C2)                  \r\n");
  printf("====================================================================\r\n");

  /* Check live physical pin voltages directly via GPIOB->IDR */
  uint8_t pb6_val  = (GPIOB->IDR & GPIO_PIN_6)  ? 1 : 0;
  uint8_t pb7_val  = (GPIOB->IDR & GPIO_PIN_7)  ? 1 : 0;
  uint8_t pb10_val = (GPIOB->IDR & GPIO_PIN_10) ? 1 : 0;
  uint8_t pb3_val  = (GPIOB->IDR & GPIO_PIN_3)  ? 1 : 0;

  printf("[LIVE PIN VOLTAGE CHECK]\r\n");
  printf("  I2C1 SCL (PB6) : %s | SDA (PB7) : %s\r\n",
         pb6_val ? "HIGH (3.3V OK)" : "LOW (WARNING: Short or Missing Pull-up!)",
         pb7_val ? "HIGH (3.3V OK)" : "LOW (WARNING: Short or Missing Pull-up!)");
  printf("  I2C2 SCL (PB10): %s | SDA (PB3) : %s\r\n",
         pb10_val ? "HIGH (3.3V OK)" : "LOW (WARNING: Short or Missing Pull-up!)",
         pb3_val  ? "HIGH (3.3V OK)" : "LOW (WARNING: Short or Missing Pull-up!)");

  if (!pb10_val || !pb3_val)
  {
    printf("  --> [ALERT I2C2]: Pin PB10 atau PB3 terdeteksi LOW!\r\n"
           "      Pastikan VCC modul terhubung, dan pasang resistor pull-up 4.7k ke 3.3V\r\n"
           "      jika modul tidak memiliki resistor pull-up sendiri.\r\n");
  }

  /* 1. Scan I2C1 (PB6, PB7) */
  I2C_RecoverIfNeeded(&hi2c1);
  PCF8574_DeviceInfo_t devs1[8];
  uint8_t count1 = PCF8574_ScanAll(&hi2c1, devs1, 8);

  printf("\r\n1. Hasil Scan Bus I2C1 (PB6=SCL, PB7=SDA):\r\n");
  if (count1 > 0)
  {
    for (uint8_t i = 0; i < count1; i++)
    {
      printf("    -> [PCF #%d] %s\r\n", i + 1, devs1[i].desc);
    }
  }
  else
  {
    printf("    Tidak ada PCF8574 terdeteksi di I2C1.\r\n");
  }

  /* Full 7-bit scan on I2C1 for any other address */
  for (uint8_t a = 0x08; a <= 0x77; a++)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(a << 1), 1, 3) == HAL_OK)
    {
      bool is_pcf = false;
      for (uint8_t i = 0; i < count1; i++) {
        if (devs1[i].addr == a) is_pcf = true;
      }
      if (!is_pcf) {
        printf("    -> [Other I2C Device] Alamat: 0x%02X\r\n", a);
      }
    }
    I2C_RecoverIfNeeded(&hi2c1);
  }

  /* 2. Scan I2C2 (PB10, PB3) */
  I2C_RecoverIfNeeded(&hi2c2);
  PCF8574_DeviceInfo_t devs2[8];
  uint8_t count2 = PCF8574_ScanAll(&hi2c2, devs2, 8);

  printf("\r\n2. Hasil Scan Bus I2C2 (PB10=SCL, PB3=SDA):\r\n");
  if (count2 > 0)
  {
    for (uint8_t i = 0; i < count2; i++)
    {
      printf("    -> [PCF #%d] %s\r\n", i + 1, devs2[i].desc);
    }
  }
  else
  {
    printf("    Tidak ada PCF8574 terdeteksi di I2C2.\r\n");
  }

  /* Full 7-bit scan on I2C2 for any other address */
  for (uint8_t a = 0x08; a <= 0x77; a++)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(a << 1), 1, 3) == HAL_OK)
    {
      bool is_pcf = false;
      for (uint8_t i = 0; i < count2; i++) {
        if (devs2[i].addr == a) is_pcf = true;
      }
      if (!is_pcf) {
        printf("    -> [Other I2C Device] Alamat: 0x%02X\r\n", a);
      }
    }
    I2C_RecoverIfNeeded(&hi2c2);
  }

  /* Target Allocation */
  g_pcf_do_detected = false;
  g_pcf_di_detected = false;

  if (count1 >= 1 && count2 >= 1)
  {
    /* IDEAL DUAL-BUS SCENARIO: One on I2C1, one on I2C2 */
    printf("\r\n  [SUCCESS] DUAL-BUS TERDETEKSI LENGKAP!\r\n");
    g_pcf_do_i2c = &hi2c1;
    g_pcf_do_bus_name = "I2C1 (PB6/PB7)";
    g_pcf_do_addr = devs1[0].addr;
    g_pcf_do_detected = true;

    g_pcf_di_i2c = &hi2c2;
    g_pcf_di_bus_name = "I2C2 (PB10/PB3)";
    g_pcf_di_addr = devs2[0].addr;
    g_pcf_di_detected = true;
  }
  else if (count1 >= 2)
  {
    /* Both on I2C1 */
    printf("\r\n  [INFO] Kedua modul terdeteksi pada bus I2C1 yang sama.\r\n");
    g_pcf_do_i2c = &hi2c1;
    g_pcf_do_bus_name = "I2C1 (PB6/PB7)";
    g_pcf_do_addr = devs1[0].addr;
    g_pcf_do_detected = true;

    g_pcf_di_i2c = &hi2c1;
    g_pcf_di_bus_name = "I2C1 (PB6/PB7)";
    g_pcf_di_addr = devs1[1].addr;
    g_pcf_di_detected = true;
  }
  else if (count2 >= 2)
  {
    /* Both on I2C2 */
    printf("\r\n  [INFO] Kedua modul terdeteksi pada bus I2C2 yang sama.\r\n");
    g_pcf_do_i2c = &hi2c2;
    g_pcf_do_bus_name = "I2C2 (PB10/PB3)";
    g_pcf_do_addr = devs2[0].addr;
    g_pcf_do_detected = true;

    g_pcf_di_i2c = &hi2c2;
    g_pcf_di_bus_name = "I2C2 (PB10/PB3)";
    g_pcf_di_addr = devs2[1].addr;
    g_pcf_di_detected = true;
  }
  else if (count1 == 1)
  {
    /* Only I2C1 has a device */
    g_pcf_do_i2c = &hi2c1;
    g_pcf_do_bus_name = "I2C1 (PB6/PB7)";
    g_pcf_do_addr = devs1[0].addr;
    g_pcf_do_detected = true;

    g_pcf_di_i2c = &hi2c2;
    g_pcf_di_bus_name = "I2C2 (PB10/PB3)";
    g_pcf_di_detected = false;
  }
  else if (count2 == 1)
  {
    /* Only I2C2 has a device */
    g_pcf_do_i2c = &hi2c2;
    g_pcf_do_bus_name = "I2C2 (PB10/PB3)";
    g_pcf_do_addr = devs2[0].addr;
    g_pcf_do_detected = true;

    g_pcf_di_i2c = &hi2c1;
    g_pcf_di_bus_name = "I2C1 (PB6/PB7)";
    g_pcf_di_detected = false;
  }

  printf("\r\n>>> ALOKASI DUAL TARGET:\r\n");
  if (g_pcf_do_detected)
  {
    printf("  [DO - Output Relays] : on %s @ 0x%02X [TERHUBUNG OK]\r\n", g_pcf_do_bus_name, g_pcf_do_addr);
    PCF8574_Write(g_pcf_do_i2c, g_pcf_do_addr, 0xFF);
    I2C_RecoverIfNeeded(g_pcf_do_i2c);
    g_pcf_do_shadow = 0xFF;
  }
  else
  {
    printf("  [DO - Output Relays] : on %s @ 0x%02X [BELUM TERDETEKSI]\r\n", g_pcf_do_bus_name, g_pcf_do_addr);
  }

  if (g_pcf_di_detected)
  {
    printf("  [DI - Input Sensors] : on %s @ 0x%02X [TERHUBUNG OK]\r\n", g_pcf_di_bus_name, g_pcf_di_addr);
    PCF8574_ConfigureAsInputs(g_pcf_di_i2c, g_pcf_di_addr);
    I2C_RecoverIfNeeded(g_pcf_di_i2c);
    g_last_di_val = 0xFF;
  }
  else
  {
    printf("  [DI - Input Sensors] : on %s [BELUM TERDETEKSI]\r\n", g_pcf_di_bus_name);
  }

  printf("  -> Tip: Tekan 'x' untuk swap peran DO <-> DI | Tekan 's' untuk scan ulang.\r\n");
  printf("====================================================================\r\n\r\n");
}

static void Print_Help_Menu(void)
{
  printf("\r\n====================== DAFTAR PERINTAH TESTER ======================\r\n");
  printf(" Tekan tombol pada keyboard di terminal untuk kontrol instan:\r\n");
  printf("  [0] : Mode Dual Demo (DO Relay Chaser + DI Live Monitor serentak)\r\n");
  printf("  [1] : Mode Input Monitor DI (Fokus monitor input pada %s @ 0x%02X)\r\n", g_pcf_di_bus_name, g_pcf_di_addr);
  printf("  [2] : Mode Relay Chaser DO (Fokus uji relay pada %s @ 0x%02X)\r\n", g_pcf_do_bus_name, g_pcf_do_addr);
  printf("  [3] : Mode LED Chaser DO (Active-HIGH P0..P7 pada %s @ 0x%02X)\r\n", g_pcf_do_bus_name, g_pcf_do_addr);
  printf("  [4] : Mode All Toggle DO (Semua ON [0x00] <--> Semua OFF [0xFF])\r\n");
  printf("  [m] : Mode Manual DO (Tekan 0..7 untuk toggle pin individu)\r\n");
  printf("  [x] : TUKAR PERAN (Swap DO <-> DI antar bus/alamat)\r\n");
  printf("  [a] : Set Semua Pin DO LOW  (0x00 / Semua Relay Aktif)\r\n");
  printf("  [f] : Set Semua Pin DO HIGH (0xFF / Semua Relay Mati)\r\n");
  printf("  [s] : Re-scan Bus I2C sekarang (I2C1 dan I2C2)\r\n");
  printf("  [h] : Tampilkan kembali menu bantuan ini\r\n");
  printf("====================================================================\r\n\r\n");
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

static void I2C_CheckPhysicalPins(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;

  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_10 | GPIO_PIN_3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_Delay(10);

  g_i2c1_pin_scl = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6);
  g_i2c1_pin_sda = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7);
  g_i2c2_pin_scl = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
  g_i2c2_pin_sda = (uint8_t)HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);

  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_10 | GPIO_PIN_3);
}

static void I2C_BusClear(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_10;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_10, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7 | GPIO_PIN_3, GPIO_PIN_SET);
  HAL_Delay(5);

  for (int i = 0; i < 9; i++)
  {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_10, GPIO_PIN_SET);
    HAL_Delay(1);
  }

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7 | GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_10, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7 | GPIO_PIN_3, GPIO_PIN_SET);
  HAL_Delay(1);

  HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_10 | GPIO_PIN_3);
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

static void MX_GPIO_Init(void)
{
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
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
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4; /* 84 MHz CPU */
  RCC_OscInitStruct.PLL.PLLQ = 7;             /* 48 MHz USB */
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}
