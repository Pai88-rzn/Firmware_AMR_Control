/**
  ******************************************************************************
  * @file           : io_controller.h
  * @brief          : Actuator, Relay, Tower Light, and Buzzer Controller
  ******************************************************************************
  */

#ifndef IO_CONTROLLER_H
#define IO_CONTROLLER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "pcf8574.h"

typedef enum {
  TOWER_LIGHT_OFF = 0,
  TOWER_LIGHT_GREEN,
  TOWER_LIGHT_YELLOW,
  TOWER_LIGHT_RED,
  TOWER_LIGHT_FLASHING_RED
} TowerLightMode_t;

void IoController_Init(I2C_HandleTypeDef *hi2c3);
void IoController_SetTowerLight(I2C_HandleTypeDef *hi2c3, TowerLightMode_t mode);
void IoController_TriggerBuzzer(uint16_t duration_ms);
void IoController_Step(I2C_HandleTypeDef *hi2c3);

#ifdef __cplusplus
}
#endif

#endif /* IO_CONTROLLER_H */
