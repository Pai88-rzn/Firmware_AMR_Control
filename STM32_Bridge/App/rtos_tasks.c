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
  uint32_t telemetry_counter = 0;

  for (;;)
  {
    /* Hard real-time safety evaluation */
    SafetyMonitor_Step(&hi2c3);

    /* Periodically (every 50ms / 20 Hz) publish safety input & state telemetry */
    telemetry_counter++;
    if (telemetry_counter >= 5)
    {
      telemetry_counter = 0;
      TelemetryMsg_t msg;
      msg.type = TELEMETRY_SAFETY_STATUS;
      msg.status_byte = SafetyMonitor_GetRawInputs();
      msg.range_m = (float)SafetyMonitor_GetState();
      msg.timestamp_ms = HAL_GetTick();

      xQueueSend(xTelemetryQueue, &msg, 0);
    }

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
  * @brief  Single-Point ToF LiDAR Task (50 Hz / 20ms cycle)
  *         Samples Rear-Left (0x10) and Rear-Right (0x11) TFmini-S for AMR auto-docking.
  */
static void vLidarTask(void *pvParameters)
{
  (void)pvParameters;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20);

  TFminiS_Data_t lidar_data;

  for (;;)
  {
    /* 1. Sample Rear-Left LiDAR (0x10) */
    if (TFminiS_ReadData(&hi2c2, TFMINI_S_ADDR_REAR_LEFT, &lidar_data) == HAL_OK && lidar_data.valid)
    {
      TelemetryMsg_t msg;
      msg.type = TELEMETRY_LIDAR_REAR_LEFT;
      msg.range_m = lidar_data.distance_m;
      msg.status_byte = (uint8_t)(lidar_data.strength > 100 ? 0 : 1);
      msg.timestamp_ms = HAL_GetTick();

      xQueueSend(xTelemetryQueue, &msg, 0);
    }

    /* 2. Sample Rear-Right LiDAR (0x11) */
    if (TFminiS_ReadData(&hi2c2, TFMINI_S_ADDR_REAR_RIGHT, &lidar_data) == HAL_OK && lidar_data.valid)
    {
      TelemetryMsg_t msg;
      msg.type = TELEMETRY_LIDAR_REAR_RIGHT;
      msg.range_m = lidar_data.distance_m;
      msg.status_byte = (uint8_t)(lidar_data.strength > 100 ? 0 : 1);
      msg.timestamp_ms = HAL_GetTick();

      xQueueSend(xTelemetryQueue, &msg, 0);
    }

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/**
  * @brief  DYP-A22 Ultrasonic Task (~20 Hz / 50ms cycle)
  *         Samples Left (0x74) and Right (0x75) sonar sensors.
  */
static void vUltrasonicTask(void *pvParameters)
{
  (void)pvParameters;
  DYP_A22_Data_t sonar_data;

  for (;;)
  {
    /* 1. Trigger ranging simultaneously on both sensors */
    DYP_A22_Trigger(&hi2c1, DYP_A22_ADDR_LEFT);
    DYP_A22_Trigger(&hi2c1, DYP_A22_ADDR_RIGHT);

    /* 2. Wait for acoustic echo measurement to complete (typical 30-40 ms) */
    vTaskDelay(pdMS_TO_TICKS(40));

    /* 3. Read Left Ultrasonic Distance */
    if (DYP_A22_ReadDistance(&hi2c1, DYP_A22_ADDR_LEFT, &sonar_data) == HAL_OK && sonar_data.valid)
    {
      TelemetryMsg_t msg;
      msg.type = TELEMETRY_SONAR_LEFT;
      msg.range_m = sonar_data.distance_m;
      msg.status_byte = 0;
      msg.timestamp_ms = HAL_GetTick();

      xQueueSend(xTelemetryQueue, &msg, 0);
    }

    /* 4. Read Right Ultrasonic Distance */
    if (DYP_A22_ReadDistance(&hi2c1, DYP_A22_ADDR_RIGHT, &sonar_data) == HAL_OK && sonar_data.valid)
    {
      TelemetryMsg_t msg;
      msg.type = TELEMETRY_SONAR_RIGHT;
      msg.range_m = sonar_data.distance_m;
      msg.status_byte = 0;
      msg.timestamp_ms = HAL_GetTick();

      xQueueSend(xTelemetryQueue, &msg, 0);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
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
