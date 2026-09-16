/**
  ******************************************************************************
  * @file           : rtos_tasks.c
  * @brief          : FreeRTOS Task Implementations & Orchestration for STM32_Bridge
  ******************************************************************************
  */

#include "rtos_tasks.h"
#include "rtos_queues.h"
#include "safety_monitor.h"
#include "io_controller.h"
#include "microros_interface.h"
#include "onboard_leds.h"
#include "tfmini_s_lidar.h"
#include "dyp_a22_ultrasonic.h"

#include "FreeRTOS.h"
#include "task.h"

/* Peripheral Handles declared in main.c */
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern I2C_HandleTypeDef hi2c3;

/* Task Handles */
static TaskHandle_t xSafetyTaskHandle = NULL;
static TaskHandle_t xMicroRosTaskHandle = NULL;
static TaskHandle_t xLidarTaskHandle = NULL;
static TaskHandle_t xUltrasonicTaskHandle = NULL;
static TaskHandle_t xIoAnimationTaskHandle = NULL;

/**
  * @brief  Hard Real-Time Safety Monitor Task (100 Hz / 10ms cycle)
  *         Monitors E-Stop & 4 Bumpers with <10ms latency cutoff of Motor Enable Relay.
  */
static void vSafetyTask(void *pvParameters)
{
  (void)pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(10);

  for (;;)
  {
    /* Hard real-time safety evaluation */
    SafetyMonitor_Step(&hi2c3);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/**
  * @brief  micro-ROS Client Task
  *         Handles DDS entity lifecycle, publish queues, and subscriber callbacks.
  */
static void vMicroRosTask(void *pvParameters)
{
  (void)pvParameters;

  /* Initialize micro-ROS client stack */
  MicroRos_Init();

  for (;;)
  {
    /* Step micro-ROS state machine and executor */
    MicroRos_SpinOnce();

    /* MicroRos_SpinOnce internally yields or has timeout, add short delay */
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

/**
  * @brief  Recover I2C peripheral on fatal bus hardware errors (BERR, ARLO, TIMEOUT)
  *         or stuck BUSY state. Benign NACKs (AF) from absent sensor are cleared.
  */
static void I2C_RecoverIfNeeded(I2C_HandleTypeDef *hi2c)
{
  if (!hi2c) return;

  if (hi2c->ErrorCode != HAL_I2C_ERROR_NONE && hi2c->ErrorCode != HAL_I2C_ERROR_AF)
  {
    if (hi2c->Instance == I2C1) {
      __HAL_RCC_I2C1_FORCE_RESET();
      __HAL_RCC_I2C1_RELEASE_RESET();
    } else if (hi2c->Instance == I2C2) {
      __HAL_RCC_I2C2_FORCE_RESET();
      __HAL_RCC_I2C2_RELEASE_RESET();
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

/**
  * @brief  Single-Point ToF LiDAR Task (50 Hz / 20ms cycle)
  *         Samples Rear-Left (0x10) and Rear-Right (0x11) TFmini-S for AMR auto-docking.
  */
static void vLidarTask(void *pvParameters)
{
  (void)pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(50); /* 20 Hz sample cycle */

  TFminiS_Data_t lidar_data;

  for (;;)
  {
    /* 1. Sample Rear-Left LiDAR (0x10) */
    TelemetryMsg_t msg_rl;
    msg_rl.type = TELEMETRY_LIDAR_REAR_LEFT;
    msg_rl.timestamp_ms = HAL_GetTick();
    if (TFminiS_ReadData(&hi2c2, TFMINI_S_ADDR_REAR_LEFT, &lidar_data) == HAL_OK)
    {
      msg_rl.range_cm = (float)lidar_data.distance_cm;
      msg_rl.status_byte = (uint8_t)(lidar_data.valid ? 0 : 1);
      xQueueSend(xTelemetryQueue, &msg_rl, 0);
    }
    I2C_RecoverIfNeeded(&hi2c2);

    /* 2. Sample Rear-Right LiDAR (0x11) */
    TelemetryMsg_t msg_rr;
    msg_rr.type = TELEMETRY_LIDAR_REAR_RIGHT;
    msg_rr.timestamp_ms = HAL_GetTick();
    if (TFminiS_ReadData(&hi2c2, TFMINI_S_ADDR_REAR_RIGHT, &lidar_data) == HAL_OK)
    {
      msg_rr.range_cm = (float)lidar_data.distance_cm;
      msg_rr.status_byte = (uint8_t)(lidar_data.valid ? 0 : 1);
      xQueueSend(xTelemetryQueue, &msg_rr, 0);
    }
    I2C_RecoverIfNeeded(&hi2c2);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/**
  * @brief  DYP-A22 Ultrasonic Task (~10 Hz / 100ms cycle)
  *         Samples Left (0x74) and Right (0x75) sonar sensors.
  *         Uses 65ms acoustic transit wait + internal driver retry loop.
  */
static void vUltrasonicTask(void *pvParameters)
{
  (void)pvParameters;
  DYP_A22_Data_t sonar_data;

  for (;;)
  {
    /* 1. Trigger Left Sonar */
    DYP_A22_Trigger(&hi2c1, DYP_A22_ADDR_LEFT);
    I2C_RecoverIfNeeded(&hi2c1);
    vTaskDelay(pdMS_TO_TICKS(5));

    /* 2. Trigger Right Sonar */
    DYP_A22_Trigger(&hi2c1, DYP_A22_ADDR_RIGHT);
    I2C_RecoverIfNeeded(&hi2c1);

    /* 3. Wait for acoustic echo measurement (typical 65 ms) */
    vTaskDelay(pdMS_TO_TICKS(65));

    /* 4. Read Left Ultrasonic Distance */
    TelemetryMsg_t msg_sl;
    msg_sl.type = TELEMETRY_SONAR_LEFT;
    msg_sl.timestamp_ms = HAL_GetTick();
    if (DYP_A22_ReadDistance(&hi2c1, DYP_A22_ADDR_LEFT, &sonar_data) == HAL_OK && sonar_data.valid)
    {
      msg_sl.range_cm = sonar_data.distance_cm;
      msg_sl.status_byte = 0;
      xQueueSend(xTelemetryQueue, &msg_sl, 0);
    }
    I2C_RecoverIfNeeded(&hi2c1);
    vTaskDelay(pdMS_TO_TICKS(5));

    /* 5. Read Right Ultrasonic Distance */
    TelemetryMsg_t msg_sr;
    msg_sr.type = TELEMETRY_SONAR_RIGHT;
    msg_sr.timestamp_ms = HAL_GetTick();
    if (DYP_A22_ReadDistance(&hi2c1, DYP_A22_ADDR_RIGHT, &sonar_data) == HAL_OK && sonar_data.valid)
    {
      msg_sr.range_cm = sonar_data.distance_cm;
      msg_sr.status_byte = 0;
      xQueueSend(xTelemetryQueue, &msg_sr, 0);
    }
    I2C_RecoverIfNeeded(&hi2c1);

    /* 10 Hz cycle padding */
    vTaskDelay(pdMS_TO_TICKS(25));
  }
}

/**
  * @brief  I/O Animation, Relay Control & Heartbeat Task (10 Hz / 100ms cycle)
  *         Processes relay queue, updates tower light / buzzer state, and blinks LED1.
  */
static void vIoAnimationTask(void *pvParameters)
{
  (void)pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(100);

  RelayCmd_t rcmd;

  for (;;)
  {
    /* 1. Drain incoming relay commands from ROS */
    while (xQueueReceive(xRelayCmdQueue, &rcmd, 0) == pdPASS)
    {
      PCF8574_WriteRelays(&hi2c3, rcmd.relay_mask);
    }

    /* 2. Step Tower Light animation & Buzzer pulse expiration */
    IoController_Step(&hi2c3);

    /* 3. Heartbeat blink on Board LED1 */
    OnboardLEDs_Toggle(LED_SYS_STATUS);

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/**
  * @brief  Initialize Inter-Task Queues, create FreeRTOS tasks, and start scheduler
  */
void RTOS_Tasks_Init(void)
{
  /* 1. Initialize Thread-Safe Queues */
  RTOS_Queues_Init();

  /* 2. Create FreeRTOS Tasks */
  xTaskCreate(vSafetyTask,
              "SafetyTask",
              256,
              NULL,
              4, /* Priority 4: Highest deterministic hard real-time safety task */
              &xSafetyTaskHandle);

  xTaskCreate(vMicroRosTask,
              "uRosTask",
              1536, /* Generous stack for DDS / micro-ROS serializer and network frames */
              NULL,
              3, /* Priority 3: High priority communication */
              &xMicroRosTaskHandle);

  xTaskCreate(vLidarTask,
              "LidarTask",
              256,
              NULL,
              2, /* Priority 2: 50 Hz LiDAR sampling */
              &xLidarTaskHandle);

  xTaskCreate(vUltrasonicTask,
              "SonarTask",
              256,
              NULL,
              1, /* Priority 1: 20 Hz Ultrasonic sampling */
              &xUltrasonicTaskHandle);

  xTaskCreate(vIoAnimationTask,
              "IoAnimTask",
              256,
              NULL,
              1, /* Priority 1: 10 Hz I/O animation and LED heartbeat */
              &xIoAnimationTaskHandle);

  /* 3. Start Scheduler */
  vTaskStartScheduler();

  /* Should never reach here */
  for (;;);
}
