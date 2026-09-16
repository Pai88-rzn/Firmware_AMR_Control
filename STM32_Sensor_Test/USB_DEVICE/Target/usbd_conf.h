/**
  ******************************************************************************
  * @file           : usbd_conf.h
  * @brief          : Header for usbd_conf.c - USB Device configuration file
  ******************************************************************************
  */

#ifndef __USBD_CONF_H__
#define __USBD_CONF_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

/* Common configurations */
#define USBD_MAX_NUM_INTERFACES     1U
#define USBD_MAX_NUM_CONFIGURATION  1U
#define USBD_MAX_STR_DESC_SIZ       512U
#define USBD_SUPPORT_USER_STRING_DESC 0U
#define USBD_SELF_POWERED           1U
#define USBD_DEBUG_LEVEL            0U

/* CDC Class Config */
#define USBD_CDC_INTERVAL           1000U

/* Memory management macros */
#define USBD_malloc               malloc
#define USBD_free                 free
#define USBD_memset               memset
#define USBD_memcpy               memcpy
#define USBD_Delay                HAL_Delay

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CONF_H__ */
