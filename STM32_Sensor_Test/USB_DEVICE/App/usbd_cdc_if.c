/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @brief          : Usb CDC interface for STM32_Sensor_Test
  ******************************************************************************
  */

#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

static USBD_CDC_LineCodingTypeDef LineCoding =
{
  115200, /* baud rate */
  0x00,   /* stop bits-1 */
  0x00,   /* parity - none */
  0x08    /* nb. of bits 8 */
};

static int8_t CDC_Init_FS(void)
{
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
}

static int8_t CDC_DeInit_FS(void)
{
  return (USBD_OK);
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:
      break;
    case CDC_GET_ENCAPSULATED_RESPONSE:
      break;
    case CDC_SET_COMM_FEATURE:
      break;
    case CDC_GET_COMM_FEATURE:
      break;
    case CDC_CLEAR_COMM_FEATURE:
      break;
    case CDC_SET_LINE_CODING:
      LineCoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |\
                              (pbuf[2] << 16) | (pbuf[3] << 24));
      LineCoding.format     = pbuf[4];
      LineCoding.paritytype = pbuf[5];
      LineCoding.datatype   = pbuf[6];
      break;
    case CDC_GET_LINE_CODING:
      pbuf[0] = (uint8_t)(LineCoding.bitrate);
      pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
      pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
      pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
      pbuf[4] = LineCoding.format;
      pbuf[5] = LineCoding.paritytype;
      pbuf[6] = LineCoding.datatype;
      break;
    case CDC_SET_CONTROL_LINE_STATE:
      break;
    case CDC_SEND_BREAK:
      break;
    default:
      break;
  }
  return (USBD_OK);
}

#define CDC_RX_RING_SIZE 256
static uint8_t s_cdc_rx_ring[CDC_RX_RING_SIZE];
static volatile uint16_t s_cdc_rx_head = 0;
static volatile uint16_t s_cdc_rx_tail = 0;

static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  if (Buf && Len && *Len > 0)
  {
    for (uint32_t i = 0; i < *Len; i++)
    {
      uint16_t next = (uint16_t)((s_cdc_rx_head + 1) % CDC_RX_RING_SIZE);
      if (next != s_cdc_rx_tail)
      {
        s_cdc_rx_ring[s_cdc_rx_head] = Buf[i];
        s_cdc_rx_head = next;
      }
    }
  }
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &UserRxBufferFS[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
}

int CDC_GetChar(uint8_t *ch)
{
  if (s_cdc_rx_head == s_cdc_rx_tail) return 0;
  if (ch)
  {
    *ch = s_cdc_rx_ring[s_cdc_rx_tail];
  }
  s_cdc_rx_tail = (uint16_t)((s_cdc_rx_tail + 1) % CDC_RX_RING_SIZE);
  return 1;
}

int CDC_Available(void)
{
  if (s_cdc_rx_head >= s_cdc_rx_tail)
    return (s_cdc_rx_head - s_cdc_rx_tail);
  return (CDC_RX_RING_SIZE - s_cdc_rx_tail + s_cdc_rx_head);
}

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

  if (hcdc == NULL)
  {
    return USBD_FAIL;
  }

  if (hcdc->TxState != 0)
  {
    return USBD_BUSY;
  }

  if (Len > APP_TX_DATA_SIZE)
  {
    Len = APP_TX_DATA_SIZE;
  }
  memcpy(UserTxBufferFS, Buf, Len);
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  return result;
}

static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  return (USBD_OK);
}
