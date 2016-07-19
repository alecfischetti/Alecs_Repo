/*************************************************************************************************/
/*!
 *  \file   board.c
 *
 *  \brief  Common Nordic board routines.
 *
 *          $Date: 2015-05-11 10:56:23 -0700 (Mon, 11 May 2015) $
 *          $Revision: 2815 $
 *
 *  Copyright (c) 2013 Wicentric, Inc., all rights reserved.
 *  Wicentric confidential and proprietary.
 *
 *  IMPORTANT.  Your use of this file is governed by a Software License Agreement
 *  ("Agreement") that must be accepted in order to download or otherwise receive a
 *  copy of this file.  You may not use or copy this file for any purpose other than
 *  as described in the Agreement.  If you do not agree to all of the terms of the
 *  Agreement do not use this file and delete all copies in your possession or control;
 *  if you do not have a copy of the Agreement, you must contact Wicentric, Inc. prior
 *  to any use, copying or further distribution of this software.
 */
/*************************************************************************************************/

#include "wsf_types.h"
#include "wsf_assert.h"
#include "wsf_buf.h"
#include "wsf_math.h"
#include "wsf_msg.h"
#include "wsf_sec.h"
#include "wsf_timer.h"

#include "apollo.h"
#include "am_bsp.h"
#include "am_util.h"

#include "am_devices_da14580.h"
#include "da14581_hci_image.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/* Timer ticks per WSF timer ticks */
#define CLK_TICKS_PER_WSF_TICKS     5       /* 9.8 ms */

/* Number of WSF buffer pools */
#define WSF_BUF_POOLS               4

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! Free memory for pool buffers. */
static uint32_t boardBufMem[1024/sizeof(uint32_t)];

/*! Default pool descriptor. */
static wsfBufPoolDesc_t boardPoolDesc[WSF_BUF_POOLS] =
{
  {  16,  8 },
  {  32,  4 },
  {  64,  6 },
  { 128,  2 }
};

bool_t boardTimerExp;

/**************************************************************************************************
  Functions
**************************************************************************************************/

static void BoardTimerInit(void);
#if WSF_TOKEN_ENABLED == TRUE
static void BoardTraceInit(void);
#endif

/*************************************************************************************************/
/*!
 *  \brief  Initialize hardware.
 *
 *  \return None.
 */
/*************************************************************************************************/
void BoardHwInit(void)
{
  // Initialize board-essential peripherals.
  am_bsp_init();

  // Enable the ITM.
  am_hal_itm_enable();

#ifdef HCI_TR_UART
  // Enable UART FIFO operation.
  am_hal_uart_fifo_config(AM_HAL_UART_TX_FIFO_1_2 | AM_HAL_UART_RX_FIFO_1_2);

  // Force RTS low
  am_hal_gpio_out_bit_clear(AM_BSP_GPIO_UART_RTS);
  am_util_delay_ms(10);

  // turn on radio power
  am_hal_gpio_out_bit_set(AM_BSP_GPIO_DIALOG_POWER);

  // Assert and Deassert RESET to the Dialog device.
  am_hal_gpio_out_bit_set(AM_BSP_GPIO_DIALOG_RESET);
  am_util_delay_ms(2);
  am_hal_gpio_out_bit_clear(AM_BSP_GPIO_DIALOG_RESET);

  // Transmit the Dialog firmware image across the PMOD UART port.
  am_devices_da14580_uart_boot(g_pui8Da14581HciImage, DA14581_HCI_IMAGE_LENGTH);

  // Clear any UART errors that may have come up in the reboot process.
  uint32_t ui32Status = AM_REG(UART, IES);
  AM_REG(UART, IES) = ui32Status;

  // Clean out the pesky zero byte in the UART Receive FIFO
  //uint8_t ui8TempChar;
  //am_hal_uart_char_receive_polled((char*)&ui8TempChar);

  // fixme figure out when we can start sending
  am_util_delay_ms(20);

  // Enable the UART and RX timeout interrupt.
  AM_REGn(UART, 0, IER) |= (AM_REG_UART_IES_RTRIS_M |
                            AM_REG_UART_IES_TXRIS_M);

  // Enable the UART interrupt handler.
  am_hal_interrupt_enable(AM_HAL_INTERRUPT_UART);
#endif

#ifdef HCI_TR_SPI

  NVIC_EnableIRQ(9);

  am_devices_dialog_spi_init(&g_am_bsp_iom_da14580, 40,
                             BoardSpiRecvComplete,
                             BoardSpiSendComplete);

#endif

  BoardTimerInit();

#if WSF_TOKEN_ENABLED == TRUE
  /* configure trace I/O */
  BoardTraceInit();
#endif
}

/*************************************************************************************************/
/*!
 *  \brief  Initialize the platform.
 *
 *  \return None.
 */
/*************************************************************************************************/
void BoardPlatformInit(void)
{
  WsfTimerInit();
  WsfBufInit(sizeof(boardBufMem), (uint8_t*)boardBufMem, WSF_BUF_POOLS, boardPoolDesc);
  WsfSecInit();
}

/*************************************************************************************************/
/*!
 *  \brief  Timer initialization.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void BoardTimerInit(void)
{
  am_hal_ctimer_config_t timer =
  {
      .ui32Link = 0,
      .ui32TimerAConfig = AM_HAL_CTIMER_INT_ENABLE |
                          AM_HAL_CTIMER_LFRC_512HZ |
                          AM_HAL_CTIMER_FN_ONCE
  };

  am_hal_ctimer_clear(0, AM_HAL_CTIMER_TIMERA);
  am_hal_ctimer_config(0, &timer);

  //
  // Enable the timer interrupt in the NVIC.
  //
  am_hal_interrupt_enable(AM_HAL_INTERRUPT_CTIMER);
}

/*************************************************************************************************/
/*!
 *  \brief  Timer service.
 *
 *  \return None.
 */
/*************************************************************************************************/
void BoardTimerService(void)
{
  uint16_t elapsedTime;

  if (boardTimerExp)
  {
    elapsedTime = AM_REGn(CTIMER, 0, CMPRA0);
  }
  else
  {
    elapsedTime = am_hal_ctimer_read(0, AM_HAL_CTIMER_TIMERA);
  }

  if (elapsedTime)
  {
    WsfTimerUpdate(elapsedTime / CLK_TICKS_PER_WSF_TICKS);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  Timer set wakeup.
 *
 *  \return TRUE if timer running, FALSE otherwise.
 */
/*************************************************************************************************/
bool_t BoardTimerSetWakeup(void)
{
  bool_t timerRunning;
  wsfTimerTicks_t nextExpTime;

  if ((nextExpTime = WsfTimerNextExpiration(&timerRunning)) != 0)
  {
    am_hal_ctimer_clear(0, AM_HAL_CTIMER_TIMERA);

    am_hal_ctimer_period_set(0, AM_HAL_CTIMER_TIMERA,
                             nextExpTime * CLK_TICKS_PER_WSF_TICKS,
                             0);

    boardTimerExp = FALSE;

    am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA0);

    am_hal_ctimer_start(0, AM_HAL_CTIMER_TIMERA);
  }
  else
  {
    am_hal_ctimer_int_disable(AM_HAL_CTIMER_INT_TIMERA0);
  }

  return timerRunning;
}

/*************************************************************************************************/
/*!
 *  \fn     am_ctimer_isr
 *
 *  \brief  Timer handler.
 *
 *  \return None.
 */
/*************************************************************************************************/
void am_ctimer_isr(void)
{
  boardTimerExp = TRUE;
  //
  // Clear TimerA0 Interrupt (write to clear).
  //
  am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA0);
}

#if WSF_TOKEN_ENABLED == TRUE

/*************************************************************************************************/
/*!
 *  \brief  Initialize the trace transport.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void BoardTraceInit(void)
{
}

/*************************************************************************************************/
/*!
 *  \brief  Write token data.
 *
 *  \param  pBuf    Buffer to write.
 *  \param  len     Length of buffer.
 *
 *  \return Number of actual bytes written.
 */
/*************************************************************************************************/
uint8_t WsfTokenIOWrite(uint8_t *pBuf, uint8_t len)
{
  return len;
}

#endif
