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

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/float32.h>

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

/* 4 Distance Sensor Publishers */
static rcl_publisher_t g_pub_lidar_rl;
static rcl_publisher_t g_pub_lidar_rr;
static rcl_publisher_t g_pub_sonar_l;
static rcl_publisher_t g_pub_sonar_r;

/* Single Float32 Message */
static std_msgs__msg__Float32 g_dist_msg;

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
      if (rmw_uros_ping_agent(50, 1) != RMW_RET_OK)
      {
        g_state = AGENT_DISCONNECTED;
        break;
      }

      /* Process pure distance telemetry messages from sensor tasks */
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
