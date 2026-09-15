/**
  ******************************************************************************
  * @file           : microros_interface.c
  * @brief          : Implementation of micro-ROS Client on STM32_Bridge
  ******************************************************************************
  */

#include "microros_interface.h"
#include "custom_transport.h"
#include "rtos_queues.h"
#include "safety_monitor.h"
#include "io_controller.h"
#include "onboard_leds.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/u_int16.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/bool.h>
#include <std_srvs/srv/trigger.h>

#pragma GCC diagnostic ignored "-Wunused-result"

extern I2C_HandleTypeDef hi2c3;

typedef enum {
  AGENT_WAITING,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} AgentState_t;

static AgentState_t g_state = AGENT_WAITING;

/* micro-ROS core handles */
static rcl_allocator_t g_allocator;
static rclc_support_t   g_support;
static rcl_node_t       g_node;
static rclc_executor_t  g_executor;

/* Publishers */
static rcl_publisher_t g_pub_lidar_rl;
static rcl_publisher_t g_pub_lidar_rr;
static rcl_publisher_t g_pub_sonar_l;
static rcl_publisher_t g_pub_sonar_r;
static rcl_publisher_t g_pub_safety_status;
static rcl_publisher_t g_pub_safety_emergency;
static rcl_publisher_t g_pub_heartbeat;

/* Messages */
static sensor_msgs__msg__Range g_range_msg;
static std_msgs__msg__UInt8    g_status_msg;
static std_msgs__msg__Bool     g_emergency_msg;
static std_msgs__msg__UInt32   g_heartbeat_msg;

/* Subscribers & Service */
static rcl_subscription_t g_sub_relays;
static std_msgs__msg__UInt8 g_msg_sub_relays;

static rcl_subscription_t g_sub_tower_light;
static std_msgs__msg__UInt8 g_msg_sub_tower_light;

static rcl_subscription_t g_sub_buzzer;
static std_msgs__msg__UInt16 g_msg_sub_buzzer;

static rcl_service_t g_srv_safety_reset;
static std_srvs__srv__Trigger_Request  g_srv_req;
static std_srvs__srv__Trigger_Response g_srv_res;

static char g_frame_id_buffer[32] = {0};

/* Subscription Callbacks */
static void relays_callback(const void *msgin)
{
  const std_msgs__msg__UInt8 *msg = (const std_msgs__msg__UInt8 *)msgin;
  if (msg)
  {
    SafetyMonitor_FeedWatchdog();
    RelayCmd_t cmd = {
      .relay_mask = msg->data,
      .pulse_duration_ms = 0
    };
    xQueueSend(xRelayCmdQueue, &cmd, 0);
  }
}

static void tower_light_callback(const void *msgin)
{
  const std_msgs__msg__UInt8 *msg = (const std_msgs__msg__UInt8 *)msgin;
  if (msg)
  {
    IoController_SetTowerLight(&hi2c3, (TowerLightMode_t)msg->data);
  }
}

static void buzzer_callback(const void *msgin)
{
  const std_msgs__msg__UInt16 *msg = (const std_msgs__msg__UInt16 *)msgin;
  if (msg)
  {
    IoController_TriggerBuzzer(msg->data);
  }
}

static void safety_reset_callback(const void *req, void *res)
{
  (void)req;
  std_srvs__srv__Trigger_Response *response = (std_srvs__srv__Trigger_Response *)res;
  if (response)
  {
    bool success = SafetyMonitor_ResetEmergency(&hi2c3);
    response->success = success;
    if (success)
    {
      IoController_SetTowerLight(&hi2c3, TOWER_LIGHT_GREEN);
    }
  }
}

static bool CreateEntities(void)
{
  g_allocator = rcl_get_default_allocator();

  if (rclc_support_init(&g_support, 0, NULL, &g_allocator) != RCL_RET_OK)
  {
    return false;
  }

  if (rclc_node_init_default(&g_node, "stm32_bridge_node", "", &g_support) != RCL_RET_OK)
  {
    rclc_support_fini(&g_support);
    return false;
  }

  /* Publishers */
  rclc_publisher_init_default(&g_pub_lidar_rl, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
      "/sensor/range/lidar_rear_left");

  rclc_publisher_init_default(&g_pub_lidar_rr, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
      "/sensor/range/lidar_rear_right");

  rclc_publisher_init_default(&g_pub_sonar_l, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
      "/sensor/range/sonar_left");

  rclc_publisher_init_default(&g_pub_sonar_r, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
      "/sensor/range/sonar_right");

  rclc_publisher_init_default(&g_pub_safety_status, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
      "/amr/safety/status");

  rclc_publisher_init_default(&g_pub_safety_emergency, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
      "/amr/safety/emergency");

  rclc_publisher_init_default(&g_pub_heartbeat, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
      "/amr/system/heartbeat");

  /* Subscribers */
  rclc_subscription_init_default(&g_sub_relays, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
      "/amr/cmd/relays");

  rclc_subscription_init_default(&g_sub_tower_light, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
      "/amr/cmd/tower_light");

  rclc_subscription_init_default(&g_sub_buzzer, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16),
      "/amr/cmd/buzzer");

  /* Service Server */
  rclc_service_init_default(&g_srv_safety_reset, &g_node,
      ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
      "/amr/safety/reset");

  /* Executor for 3 subscriptions + 1 service = 4 handles */
  rclc_executor_init(&g_executor, &g_support.context, 4, &g_allocator);
  rclc_executor_add_subscription(&g_executor, &g_sub_relays, &g_msg_sub_relays, &relays_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&g_executor, &g_sub_tower_light, &g_msg_sub_tower_light, &tower_light_callback, ON_NEW_DATA);
  rclc_executor_add_subscription(&g_executor, &g_sub_buzzer, &g_msg_sub_buzzer, &buzzer_callback, ON_NEW_DATA);
  rclc_executor_add_service(&g_executor, &g_srv_safety_reset, &g_srv_req, &g_srv_res, &safety_reset_callback);

  /* Time synchronization with ROS 2 agent */
  rmw_uros_sync_session(100);

  /* Set Range static metadata */
  g_range_msg.header.frame_id.data = g_frame_id_buffer;
  g_range_msg.header.frame_id.capacity = sizeof(g_frame_id_buffer);

  return true;
}

static void DestroyEntities(void)
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&g_support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  (void)rcl_publisher_fini(&g_pub_lidar_rl, &g_node);
  (void)rcl_publisher_fini(&g_pub_lidar_rr, &g_node);
  (void)rcl_publisher_fini(&g_pub_sonar_l, &g_node);
  (void)rcl_publisher_fini(&g_pub_sonar_r, &g_node);
  (void)rcl_publisher_fini(&g_pub_safety_status, &g_node);
  (void)rcl_publisher_fini(&g_pub_safety_emergency, &g_node);
  (void)rcl_publisher_fini(&g_pub_heartbeat, &g_node);

  (void)rcl_subscription_fini(&g_sub_relays, &g_node);
  (void)rcl_subscription_fini(&g_sub_tower_light, &g_node);
  (void)rcl_subscription_fini(&g_sub_buzzer, &g_node);
  (void)rcl_service_fini(&g_srv_safety_reset, &g_node);

  (void)rclc_executor_fini(&g_executor);
  (void)rcl_node_fini(&g_node);
  (void)rclc_support_fini(&g_support);
}

void MicroRos_Init(void)
{
  rmw_uros_set_custom_transport(
      true,
      NULL,
      custom_transport_open,
      custom_transport_close,
      custom_transport_write,
      custom_transport_read
  );

  g_state = AGENT_WAITING;
}

void MicroRos_SpinOnce(void)
{
  uint32_t now = HAL_GetTick();

  switch (g_state)
  {
    case AGENT_WAITING:
      if (rmw_uros_ping_agent(50, 1) == RMW_RET_OK)
      {
        g_state = AGENT_AVAILABLE;
      }
      break;

    case AGENT_AVAILABLE:
      if (CreateEntities())
      {
        g_state = AGENT_CONNECTED;
        OnboardLEDs_Set(LED_COMM_STATUS, true);
      }
      else
      {
        DestroyEntities();
        g_state = AGENT_WAITING;
      }
      break;

    case AGENT_CONNECTED:
      if (rmw_uros_ping_agent(50, 1) != RMW_RET_OK)
      {
        g_state = AGENT_DISCONNECTED;
        break;
      }

      /* 1. Check emergency queue (High priority) */
      EmergencyEvent_t emergency_evt;
      if (xQueueReceive(xEmergencyQueue, &emergency_evt, 0) == pdTRUE)
      {
        g_emergency_msg.data = true;
        (void)rcl_publish(&g_pub_safety_emergency, &g_emergency_msg, NULL);
      }

      /* 2. Process telemetry messages from sensor tasks */
      TelemetryMsg_t msg;
      while (xQueueReceive(xTelemetryQueue, &msg, 0) == pdTRUE)
      {
        int64_t stamp_ns = rmw_uros_epoch_nanos();
        g_range_msg.header.stamp.sec = (int32_t)(stamp_ns / 1000000000LL);
        g_range_msg.header.stamp.nanosec = (uint32_t)(stamp_ns % 1000000000LL);

        switch (msg.type)
        {
          case TELEMETRY_LIDAR_REAR_LEFT:
            strncpy(g_frame_id_buffer, "lidar_rear_left_link", sizeof(g_frame_id_buffer) - 1);
            g_range_msg.header.frame_id.size = strlen(g_frame_id_buffer);
            g_range_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
            g_range_msg.field_of_view = 0.035f;
            g_range_msg.min_range = 0.10f;
            g_range_msg.max_range = 12.00f;
            g_range_msg.range = msg.range_m;
            (void)rcl_publish(&g_pub_lidar_rl, &g_range_msg, NULL);
            break;

          case TELEMETRY_LIDAR_REAR_RIGHT:
            strncpy(g_frame_id_buffer, "lidar_rear_right_link", sizeof(g_frame_id_buffer) - 1);
            g_range_msg.header.frame_id.size = strlen(g_frame_id_buffer);
            g_range_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
            g_range_msg.field_of_view = 0.035f;
            g_range_msg.min_range = 0.10f;
            g_range_msg.max_range = 12.00f;
            g_range_msg.range = msg.range_m;
            (void)rcl_publish(&g_pub_lidar_rr, &g_range_msg, NULL);
            break;

          case TELEMETRY_SONAR_LEFT:
            strncpy(g_frame_id_buffer, "sonar_left_link", sizeof(g_frame_id_buffer) - 1);
            g_range_msg.header.frame_id.size = strlen(g_frame_id_buffer);
            g_range_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
            g_range_msg.field_of_view = 1.047f;
            g_range_msg.min_range = 0.02f;
            g_range_msg.max_range = 3.50f;
            g_range_msg.range = msg.range_m;
            (void)rcl_publish(&g_pub_sonar_l, &g_range_msg, NULL);
            break;

          case TELEMETRY_SONAR_RIGHT:
            strncpy(g_frame_id_buffer, "sonar_right_link", sizeof(g_frame_id_buffer) - 1);
            g_range_msg.header.frame_id.size = strlen(g_frame_id_buffer);
            g_range_msg.radiation_type = sensor_msgs__msg__Range__ULTRASOUND;
            g_range_msg.field_of_view = 1.047f;
            g_range_msg.min_range = 0.02f;
            g_range_msg.max_range = 3.50f;
            g_range_msg.range = msg.range_m;
            (void)rcl_publish(&g_pub_sonar_r, &g_range_msg, NULL);
            break;

          case TELEMETRY_SAFETY_STATUS:
            g_status_msg.data = msg.status_byte;
            (void)rcl_publish(&g_pub_safety_status, &g_status_msg, NULL);
            break;
        }
      }

      /* 3. Publish periodic Heartbeat (2 Hz) */
      static uint32_t s_last_hb = 0;
      if ((now - s_last_hb) >= 500)
      {
        s_last_hb = now;
        g_heartbeat_msg.data = now;
        (void)rcl_publish(&g_pub_heartbeat, &g_heartbeat_msg, NULL);
      }

      /* 4. Spin executor for incoming commands */
      rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(10));
      break;

    case AGENT_DISCONNECTED:
      OnboardLEDs_Set(LED_COMM_STATUS, false);
      DestroyEntities();
      g_state = AGENT_WAITING;
      break;
  }
}

bool MicroRos_IsConnected(void)
{
  return (g_state == AGENT_CONNECTED);
}
