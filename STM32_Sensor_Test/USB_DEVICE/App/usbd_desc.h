/**
  ******************************************************************************
  * @file           : usbd_desc.h
  * @brief          : Header for usbd_desc.c - USB Device Descriptor
  ******************************************************************************
  */

#ifndef __USBD_DESC_H__
#define __USBD_DESC_H__

#ifdef __cplusplus
 extern "C" {
#endif

#include "usbd_def.h"

#define DEVICE_FS 0

#define USB_SIZ_STRING_SERIAL         0x1AU

#define USBD_VID                      1155  /* 0x0483: STMicroelectronics */
#define USBD_LANGID_STRING            1033  /* 0x0409: English (United States) */
#define USBD_MANUFACTURER_STRING      "STMicroelectronics"
#define USBD_PID_FS                   22336 /* 0x5740: Virtual COM Port */
#define USBD_PRODUCT_STRING_FS        "STM32 Bridge AMR Micro-ROS"
#define USBD_CONFIGURATION_STRING_FS  "CDC Config"
#define USBD_INTERFACE_STRING_FS      "CDC Interface"

extern USBD_DescriptorsTypeDef FS_Desc;

#ifdef __cplusplus
}
#endif

#endif /* __USBD_DESC_H__ */
