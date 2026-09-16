/**
 ******************************************************************************
 * @file      syscalls.c
 * @brief     Minimal System Calls for Newlib with robust USB CDC routing
 ******************************************************************************
 */

#include <sys/stat.h>
#include <stdlib.h>
#include <errno.h>
#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/times.h>

#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include "usbd_def.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

int __io_putchar(int ch) __attribute__((weak));
int __io_getchar(void) __attribute__((weak));

int __io_putchar(int ch)
{
  return ch;
}

int __io_getchar(void)
{
  return 0;
}

int _getpid(void)
{
  return 1;
}

int _kill(int pid, int sig)
{
  (void)pid;
  (void)sig;
  errno = EINVAL;
  return -1;
}

void _exit (int status)
{
  _kill(status, -1);
  while (1) {}
}

__attribute__((weak)) int _read(int file, char *ptr, int len)
{
  (void)file;
  (void)ptr;
  (void)len;
  return 0;
}

int _write(int file, char *ptr, int len)
{
  (void)file;
  if (len <= 0) return 0;

  /* If USB is not fully enumerated and configured by host, drop safely */
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED)
  {
    return len;
  }

  uint16_t sent = 0;
  while (sent < len)
  {
    uint16_t chunk = (len - sent) > APP_TX_DATA_SIZE ? APP_TX_DATA_SIZE : (uint16_t)(len - sent);
    uint32_t start = HAL_GetTick();

    while (CDC_Transmit_FS((uint8_t*)(ptr + sent), chunk) == USBD_BUSY)
    {
      if ((HAL_GetTick() - start) > 50)
      {
        /* Host is not polling IN endpoint; drop packet without corrupting hardware state */
        return len;
      }
    }
    sent += chunk;
  }
  return len;
}

int _close(int file)
{
  (void)file;
  return -1;
}

int _fstat(int file, struct stat *st)
{
  (void)file;
  st->st_mode = S_IFCHR;
  return 0;
}

int _isatty(int file)
{
  (void)file;
  return 1;
}

int _lseek(int file, int ptr, int dir)
{
  (void)file;
  (void)ptr;
  (void)dir;
  return 0;
}

caddr_t _sbrk(int incr)
{
  extern char end asm("end");
  static char *heap_end;
  char *prev_heap_end;

  if (heap_end == 0)
    heap_end = &end;

  prev_heap_end = heap_end;
  heap_end += incr;

  return (caddr_t) prev_heap_end;
}
