/*************************************************************************************************/
/*!
 *  \file   hci_drv.c
 *
 *  \brief  HCI driver interface.
 *
 *          $Date: 2014-12-19 15:05:45 -0800 (Fri, 19 Dec 2014) $
 *          $Revision: 2139 $
 *
 *  Copyright (c) 2012 Wicentric, Inc., all rights reserved.
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
#include "wsf_msg.h"
#include "hci_drv.h"
#include "hci_core.h"

#include "apollo.h"
#include "am_hal_uart.h"

#include <string.h>
#include "board_apollo.h"

#ifdef HCI_TR_UART
void hciTrSerialRxIncoming(uint8_t *pBuf, uint8_t len);
#endif

/**************************************************************************************************
  Data
**************************************************************************************************/

static uint8_t *pHciDrvWriteBuf;

/*************************************************************************************************/
/*!
 *  \brief  SPI receive completion callback.
 *
 *  \param  type    Packet type.
 *  \param  pBuf    Received buffer.
 *  \param  len     Buffer length.
 *
 *  \return None.
 */
/*************************************************************************************************/
void BoardSpiRecvComplete(uint8_t type, uint8_t *pBuf, uint8_t len)
{
  uint8_t *pPktRx;

  /* allocate data buffer to hold entire packet */
  if ((pPktRx = (uint8_t*)WsfMsgAlloc(len)) != NULL)
  {
    memcpy(pPktRx, pBuf, len);
    hciCoreRecv(type, pPktRx);
  }
}

/*************************************************************************************************/
/*!
 *  \brief  SPI send completion callback.
 *
 *  \return None.
 */
/*************************************************************************************************/
void BoardSpiSendComplete(void)
{
  WsfMsgFree(pHciDrvWriteBuf);
  pHciDrvWriteBuf = NULL;
}

/*************************************************************************************************/
/*!
 *  \fn     hciDrvWrite
 *
 *  \brief  Write data the driver.
 *
 *  \param  type     HCI packet type
 *  \param  len      Number of bytes to write.
 *  \param  pData    Byte array to write.
 *
 *  \return Return actual number of data bytes written.
 *
 *  \note   The type parameter allows the driver layer to prepend the data with a header on the
 *          same write transaction.
 */
/*************************************************************************************************/
uint16_t hciDrvWrite(uint8_t type, uint16_t len, uint8_t *pData)
{
  uint16_t count = 0;

  pHciDrvWriteBuf = pData;

#ifdef HCI_TR_UART
  am_hal_uart_char_transmit_polled(type);

  while (count < len)
  {
    am_hal_uart_char_transmit_polled(*pData++);
    count++;
  }
#endif

#ifdef HCI_TR_SPI
  am_devices_dialog_spi_send(type, pData, len);
#endif

  return count;
}

/*************************************************************************************************/
/*!
 *  \fn     hciDrvReadyToSleep
 *
 *  \brief  Returns TRUE if driver allows MCU to enter low power sleep mode.
 *
 *  \return TRUE if ready to sleep, FALSE otherwise.
 */
/*************************************************************************************************/
bool_t hciDrvReadyToSleep(void)
{
  return TRUE;
}

#ifdef HCI_TR_UART

/*************************************************************************************************/
/*!
 *  \fn     am_uart_isr
 *
 *  \brief  UART handler.
 *
 *  \return None.
 */
/*************************************************************************************************/
void am_uart_isr(void)
{
  static bool_t isFirstByte = TRUE;

  uint32_t status;

  //
  // Read and save the interrupt status, but clear out the status register.
  //
  status = AM_REGn(UART, 0, MIS);
  AM_REGn(UART, 0, IEC) = status;

  //
  // Check to see if we have filled the Rx FIFO past the configured limit, or
  // if we have an 'old' character or two sitting in the FIFO.
  //
  if (status & (AM_REG_UART_IES_RXRIS_M | AM_REG_UART_IES_RTRIS_M))
  {
    //
    // While there's stuff in the RX fifo....
    //
    while (!AM_BFRn(UART, 0, FR, RXFE))
    {
      uint8_t ch = AM_REGn(UART, 0, DR);

      if (!isFirstByte)
      {
        hciTrSerialRxIncoming(&ch, 1);
      }
      else
      {
        isFirstByte = FALSE;
      }
    }
  }
}

#endif
