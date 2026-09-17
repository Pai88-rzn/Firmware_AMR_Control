/**
  ******************************************************************************
  * @file           : microros_interface.c
  * @brief          : Streamlined micro-ROS Client for pure distance telemetry
  ******************************************************************************
  */

#include "microros_interface.h"
#include "custom_transport.h"
#include "rtos_queues.h"
#include "onboard_leds.h"
#include "safety_monitor.h"
#include "pcf8574.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/u_int8.h>

#pragma GCC diagnostic ignored "-Wunused-result"

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

/* 4 Distance Sensor Publishers */
static rcl_publisher_t g_pub_lidar_rl;
static rcl_publisher_t g_pub_lidar_rr;
static rcl_publisher_t g_pub_sonar_l;
static rcl_publisher_t g_pub_sonar_r;

/* Digital Input Publisher (PCF8574 DI) */
static rcl_publisher_t g_pub_inputs;

/* Relay Output Subscriber (PCF8574 DO) */
static rcl_subscription_t g_sub_relays;

/* Message buffers */
static std_msgs__msg__Float32 g_dist_msg;
static std_msgs__msg__UInt8   g_inputs_msg;
static std_msgs__msg__UInt8   g_relay_sub_msg;

static void RelaySubCallback(const void * msgin)
{
  const std_msgs__msg__UInt8 * msg = (const std_msgs__msg__UInt8 *)msgin;
  if (msg != NULL)
  {
    RelayCmd_t rcmd;
    rcmd.relay_mask = msg->data;
    rcmd.pulse_duration_ms = 0;

    /* Feed safety watchdog on incoming command */
    SafetyMonitor_FeedWatchdog();

    /* Safety Cutoff: If emergency is active (E-Stop or Collision), do not allow enabling Motor Relay (DOUT2) */
    if (SafetyMonitor_GetState() != SAFETY_OK)
    {
      rcmd.relay_mask &= (uint8_t)~RELAY_BIT_MOTOR_EN;
    }

    xQueueSend(xRelayCmdQueue, &rcmd, 0);
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

  /* Publishers for the 4 distance sensors */
  rclc_publisher_init_default(&g_pub_lidar_rl, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/sensor/range/lidar_rear_left");

  rclc_publisher_init_default(&g_pub_lidar_rr, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/sensor/range/lidar_rear_right");

  rclc_publisher_init_default(&g_pub_sonar_l, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/sensor/range/sonar_left");

  rclc_publisher_init_default(&g_pub_sonar_r, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/sensor/range/sonar_right");

  /* Publisher for Digital Inputs (PCF8574 DI) */
  rclc_publisher_init_default(&g_pub_inputs, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
      "/io/inputs");

  /* Subscriber for Relay Outputs (PCF8574 DO) */
  rclc_subscription_init_default(&g_sub_relays, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
      "/cmd/relays");

  /* Executor for handling incoming subscriptions (1 handle for g_sub_relays) */
  if (rclc_executor_init(&g_executor, &g_support.context, 1, &g_allocator) != RCL_RET_OK)
  {
    return false;
  }

  if (rclc_executor_add_subscription(&g_executor, &g_sub_relays, &g_relay_sub_msg,
                                     RelaySubCallback, ON_NEW_DATA) != RCL_RET_OK)
  {
    return false;
  }

  return true;
}

static void DestroyEntities(void)
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&g_support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  (void)rclc_executor_fini(&g_executor);
  (void)rcl_subscription_fini(&g_sub_relays, &g_node);
  (void)rcl_publisher_fini(&g_pub_inputs, &g_node);

  (void)rcl_publisher_fini(&g_pub_lidar_rl, &g_node);
  (void)rcl_publisher_fini(&g_pub_lidar_rr, &g_node);
  (void)rcl_publisher_fini(&g_pub_sonar_l, &g_node);
  (void)rcl_publisher_fini(&g_pub_sonar_r, &g_node);

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
      {
        static uint32_t s_last_ping = 0;
        uint32_t now = HAL_GetTick();
        if ((now - s_last_ping) >= 1000)
        {
          s_last_ping = now;
          if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK)
          {
            g_state = AGENT_DISCONNECTED;
            break;
          }
        }
      }

      /* Feed safety watchdog while micro-ROS connection is active */
      SafetyMonitor_FeedWatchdog();

      /* 1. Spin executor to execute incoming subscriber callbacks (/cmd/relays) */
      rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(5));

      /* 2. Publish digital inputs (PCF8574 DI) - edge triggered + 20 Hz periodic */
      {
        static uint32_t s_last_input_pub = 0;
        static uint8_t  s_last_input_val = 0xFF;
        uint32_t now = HAL_GetTick();
        uint8_t current_inputs = SafetyMonitor_GetRawInputs();

        if ((current_inputs != s_last_input_val) || ((now - s_last_input_pub) >= 50))
        {
          s_last_input_pub = now;
          s_last_input_val = current_inputs;
          g_inputs_msg.data = current_inputs;
          (void)rcl_publish(&g_pub_inputs, &g_inputs_msg, NULL);
        }
      }

      /* 3. Process pure distance telemetry messages from sensor tasks */
      TelemetryMsg_t msg;
      while (xQueueReceive(xTelemetryQueue, &msg, 0) == pdTRUE)
      {
        g_dist_msg.data = msg.range_cm;

        switch (msg.type)
        {
          case TELEMETRY_LIDAR_REAR_LEFT:
            (void)rcl_publish(&g_pub_lidar_rl, &g_dist_msg, NULL);
            break;

          case TELEMETRY_LIDAR_REAR_RIGHT:
            (void)rcl_publish(&g_pub_lidar_rr, &g_dist_msg, NULL);
            break;

          case TELEMETRY_SONAR_LEFT:
            (void)rcl_publish(&g_pub_sonar_l, &g_dist_msg, NULL);
            break;

          case TELEMETRY_SONAR_RIGHT:
            (void)rcl_publish(&g_pub_sonar_r, &g_dist_msg, NULL);
            break;

          default:
            break;
        }
      }
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
