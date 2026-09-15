/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file for STM32_Bridge
  *                   Device: STM32F411CEU6TR (Arm Cortex-M4F @ 100MHz)
  ******************************************************************************
  */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

/* Pin definitions for STM32_Bridge */
#define LED1_PIN                GPIO_PIN_13
#define LED1_GPIO_PORT          GPIOC
#define LED2_PIN                GPIO_PIN_12
#define LED2_GPIO_PORT          GPIOB

/* I2C1 Pins (Ultrasonic Sensors) */
#define I2C1_SCL_PIN            GPIO_PIN_6
#define I2C1_SDA_PIN            GPIO_PIN_7
#define I2C1_GPIO_PORT          GPIOB

/* I2C2 Pins (Single-point ToF LiDARs) */
#define I2C2_SCL_PIN            GPIO_PIN_10
#define I2C2_SDA_PIN            GPIO_PIN_3
#define I2C2_GPIO_PORT          GPIOB

/* I2C3 Pins (Dual PCF8574 I/O Expanders) */
#define I2C3_SCL_PIN            GPIO_PIN_8
#define I2C3_SCL_PORT           GPIOA
#define I2C3_SDA_PIN            GPIO_PIN_4
#define I2C3_SDA_PORT           GPIOB

/* USART1 Pins (LoRa Transceiver) */
#define LORA_TX_PIN             GPIO_PIN_9
#define LORA_RX_PIN             GPIO_PIN_10
#define LORA_GPIO_PORT          GPIOA

/* USB OTG FS Pins */
#define USB_DM_PIN              GPIO_PIN_11
#define USB_DP_PIN              GPIO_PIN_12
#define USB_GPIO_PORT           GPIOA

/* Exported functions */
void Error_Handler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
