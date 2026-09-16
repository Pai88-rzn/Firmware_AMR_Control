/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file in STM32_Sensor_Test
  *                   Target: STM32F401CCU6 (Arm Cortex-M4F @ 84MHz)
  ******************************************************************************
  */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"

/* Onboard User LED (BlackPill PC13, active LOW) */
#define LED1_PIN                GPIO_PIN_13
#define LED1_GPIO_PORT          GPIOC
#define LED2_PIN                GPIO_PIN_12
#define LED2_GPIO_PORT          GPIOB

/* I2C1 Pins (DYP-A22 Ultrasonic Sensors) */
#define I2C1_SCL_PIN            GPIO_PIN_6
#define I2C1_SDA_PIN            GPIO_PIN_7
#define I2C1_GPIO_PORT          GPIOB

/* I2C2 Pins (TFmini-S LiDAR Sensors) */
#define I2C2_SCL_PIN            GPIO_PIN_10
#define I2C2_SDA_PIN            GPIO_PIN_3
#define I2C2_GPIO_PORT          GPIOB

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
