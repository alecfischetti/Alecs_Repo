/*************************************************************************************************/
/*!
 *  \file   hci_core_ps.c
 *
 *  \brief  HCI core platform-specific module single-chip.
 *
 *          $Date: 2015-09-05 09:01:07 -0700 (Sat, 05 Sep 2015) $
 *          $Revision: 3793 $
 *
 *  Copyright (c) 2009 Wicentric, Inc., all rights reserved.
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

#include <string.h>
#include "wsf_types.h"
#include "wsf_msg.h"
#include "wsf_trace.h"
#include "bda.h"
#include "bstream.h"
#include "hci_core.h"
#include "hci_tr.h"
#include "hci_cmd.h"
#include "hci_evt.h"
#include "hci_api.h"
#include "hci_main.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \fn     hciCoreInit
 *
 *  \brief  HCI core initialization.
 *
 *  \return None.
 */
/*************************************************************************************************/
void hciCoreInit(void)
{
  LlEvtRegister(hciCoreEvtProcessLlEvt);
  LlAclRegister(hciCoreNumCmplPkts, hciCoreAclRecvPending);

  /* synchronize with LL */
  hciCoreCb.numBufs = LlGetAclTxBufs();
  hciCoreCb.availBufs = LlGetAclTxBufs();
  hciCoreCb.bufSize = LlGetAclMaxSize();
}

/*************************************************************************************************/
/*!
 *  \fn     hciCoreEvtProcessLlEvt
 *
 *  \brief  Process received HCI events.
 *
 *  \param  pEvt    Buffer containing LL event.
 *
 *  \return Always TRUE.
 */
/*************************************************************************************************/
bool_t hciCoreEvtProcessLlEvt(llEvt_t *pEvt)
{
  llEvt_t *pMsg;

  if ((pMsg = WsfMsgAlloc(sizeof(*pMsg))) != NULL)
  {
    *pMsg = *pEvt;      /* copy event to message buffer */
    WsfMsgEnq(&hciCb.rxQueue, HCI_EVT_TYPE, pMsg);
    WsfSetEvent(hciCb.handlerId, HCI_EVT_RX);
  }

  return TRUE;
}

/*************************************************************************************************/
/*!
 *  \fn     hciCoreAclRecvPending
 *
 *  \brief  LL ACL receive buffer pending handler.
 *
 *  \param  handle    Connection handle.
 *  \param  numBufs   Number of buffers completed.
 *
 *  \return None.
 */
/*************************************************************************************************/
void hciCoreAclRecvPending(uint16_t handle, uint8_t numBufs)
{
  uint8_t *pBuf;

  while ((pBuf = LlRecvAclData()) != NULL)
  {
    WsfMsgEnq(&hciCb.rxQueue, HCI_ACL_TYPE, pBuf);
  }

  WsfSetEvent(hciCb.handlerId, HCI_EVT_RX);
}

/*************************************************************************************************/
/*!
 *  \fn     hciCoreNumCmplPkts
 *
 *  \brief  Handle an HCI Number of Completed Packets event.
 *
 *  \param  pMsg    Message containing the HCI Number of Completed Packets event.
 *
 *  \return None.
 */
/*************************************************************************************************/
void hciCoreNumCmplPkts(uint16_t handle, uint8_t numBufs)
{
  hciCoreConn_t   *pConn;

  if ((pConn = hciCoreConnByHandle(handle)) != NULL)
  {
    /* decrement outstanding buffer count to controller */
    pConn->outBufs -= (uint8_t) numBufs;

    /* decrement queued buffer count for this connection */
    pConn->queuedBufs -= (uint8_t) numBufs;

    /* call flow control callback */
    if (pConn->flowDisabled && pConn->queuedBufs <=  hciCoreCb.aclQueueLo)
    {
      pConn->flowDisabled = FALSE;
      (*hciCb.flowCback)(handle, FALSE);
    }

    /* service TX data path */
    hciCoreTxReady(numBufs);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciCoreHandler
 *
 *  \brief  WSF event handler for core HCI.
 *
 *  \param  event   WSF event mask.
 *  \param  pMsg    WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciCoreHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{
  uint8_t         *pBuf;
  wsfHandlerId_t  handlerId;

  /* Handle events */
  if (event & HCI_EVT_RX)
  {
    /* Process rx queue */
    while ((pBuf = WsfMsgDeq(&hciCb.rxQueue, &handlerId)) != NULL)
    {
      /* Handle incoming HCI events */
      if (handlerId == HCI_EVT_TYPE)
      {
        /* Parse/process events */
        hciEvtProcessMsg(pBuf);

        /* Free buffer */
        WsfMsgFree(pBuf);
      }
      /* Handle ACL data */
      else
      {
        /* Reassemble */
        if ((pBuf = hciCoreAclReassembly(pBuf)) != NULL)
        {
          /* Call ACL callback; client will free buffer */
          hciCb.aclCback(pBuf);
        }
      }
    }
  }
}

/*************************************************************************************************/
/*!
 *  \fn     HciGetBdAddr
 *
 *  \brief  Return a pointer to the BD address of this device.
 *
 *  \return Pointer to the BD address.
 */
/*************************************************************************************************/
uint8_t *HciGetBdAddr(void)
{
  return hciCoreCb.bdAddr;
}

/*************************************************************************************************/
/*!
 *  \fn     HciGetWhiteListSize
 *
 *  \brief  Return the white list size.
 *
 *  \return White list size.
 */
/*************************************************************************************************/
uint8_t HciGetWhiteListSize(void)
{
  return LlGetWhitelistSize();
}

/*************************************************************************************************/
/*!
 *  \fn     HciGetAdvTxPwr
 *
 *  \brief  Return the advertising transmit power.
 *
 *  \return Advertising transmit power.
 */
/*************************************************************************************************/
int8_t HciGetAdvTxPwr(void)
{
  return LlGetAdvTxPwr();
}

/*************************************************************************************************/
/*!
 *  \fn     HciGetBufSize
 *
 *  \brief  Return the ACL buffer size supported by the controller.
 *
 *  \return ACL buffer size.
 */
/*************************************************************************************************/
uint16_t HciGetBufSize(void)
{
  return hciCoreCb.bufSize;
}

/*************************************************************************************************/
/*!
 *  \fn     HciGetNumBufs
 *
 *  \brief  Return the number of ACL buffers supported by the controller.
 *
 *  \return Number of ACL buffers.
 */
/*************************************************************************************************/
uint8_t HciGetNumBufs(void)
{
  return hciCoreCb.numBufs;
}

/*************************************************************************************************/
/*!
 *  \fn     HciGetSupStates
 *
 *  \brief  Return the states supported by the controller.
 *
 *  \return Pointer to the supported states array.
 */
/*************************************************************************************************/
uint8_t *HciGetSupStates(void)
{
  static uint8_t supStates[8];

  LlGetSupStates(supStates);

  return supStates;
}

/*************************************************************************************************/
/*!
 *  \fn     HciGetLeSupFeat
 *
 *  \brief  Return the LE supported features supported by the controller.
 *
 *  \return Supported features.
 */
/*************************************************************************************************/
uint8_t HciGetLeSupFeat(void)
{
  uint8_t feat[HCI_LE_STATES_LEN];

  LlGetFeatures(feat);

  return feat[0];
}
