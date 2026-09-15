/**
  ******************************************************************************
  * @file           : imu_app.c
  * @brief          : Implementation of IMU supervisory state machine
  *                   (Board 4: STM32_IMU).
  ******************************************************************************
  */

#include "imu_app.h"
#include <string.h>

static ImuApp_t g_imu_app;

void IMU_App_Init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart)
{
  (void)hi2c;

  memset(&g_imu_app, 0, sizeof(ImuApp_t));
  g_imu_app.state = IMU_STATE_INIT;
  g_imu_app.sample_period_ms = 10; /* 10 ms = 100 Hz */

  /* Initialize micro-ROS subsystem on USART1 */
  MicroRosIMU_Init(huart);
}

void IMU_App_Process(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart)
{
  (void)huart;
  uint32_t now = HAL_GetTick();

  switch (g_imu_app.state)
  {
    case IMU_STATE_UNINITIALIZED:
    case IMU_STATE_INIT:
    {
      LED_Off();
      MicroRosIMU_Process(NULL);

      /* Initialize BNO055 in NDOF mode with external 32.768kHz crystal */
      if (BNO055_Init(hi2c, BNO055_OPR_MODE_NDOF) == HAL_OK)
      {
        g_imu_app.state = IMU_STATE_STREAMING;
        g_imu_app.error_count = 0;
        g_imu_app.last_sample_tick = now;
        g_imu_app.last_led_tick = now;
      }
      else
      {
        g_imu_app.state = IMU_STATE_ERROR;
        g_imu_app.last_sample_tick = now;
      }
      break;
    }

    case IMU_STATE_STREAMING:
    {
      /* Allow micro-ROS state machine to process connection/pings */
      MicroRosIMU_Process(NULL);

      /* Rate-limited sampling loop (100 Hz) */
      if ((now - g_imu_app.last_sample_tick) >= g_imu_app.sample_period_ms)
      {
        g_imu_app.last_sample_tick = now;

        /* Burst read all sensor parameters */
        if (BNO055_ReadAllData(hi2c, &g_imu_app.sensor_data) == HAL_OK)
        {
          /* Publish clean /imu message via micro-ROS to ROS 2 Jazzy */
          MicroRosIMU_Process(&g_imu_app.sensor_data);

          g_imu_app.packets_sent++;
          g_imu_app.error_count = 0;
        }
        else
        {
          g_imu_app.error_count++;
          if (g_imu_app.error_count >= 10)
          {
            g_imu_app.state = IMU_STATE_ERROR;
          }
        }
      }

      /* Healthy heartbeat LED (500 ms toggle if connected, 1000 ms if waiting for agent) */
      uint32_t led_period = (MicroRosIMU_GetState() == MICROROS_STATE_AGENT_CONNECTED) ? 500 : 1000;
      if ((now - g_imu_app.last_led_tick) >= led_period)
      {
        g_imu_app.last_led_tick = now;
        LED_Toggle();
      }
      break;
    }

    case IMU_STATE_ERROR:
    {
      /* Error indicator: rapid blink (100 ms) */
      if ((now - g_imu_app.last_led_tick) >= 100)
      {
        g_imu_app.last_led_tick = now;
        LED_Toggle();
      }

      /* Attempt bus recovery and re-init every 2000 ms */
      if ((now - g_imu_app.last_sample_tick) >= 2000)
      {
        g_imu_app.last_sample_tick = now;
        BNO055_RecoverBus();
        /* Reinitialize I2C peripheral */
        HAL_I2C_DeInit(hi2c);
        HAL_I2C_Init(hi2c);
        g_imu_app.state = IMU_STATE_INIT;
      }
      break;
    }

    default:
      g_imu_app.state = IMU_STATE_INIT;
      break;
  }
}

ImuAppState_t IMU_App_GetState(void)
{
  return g_imu_app.state;
}
