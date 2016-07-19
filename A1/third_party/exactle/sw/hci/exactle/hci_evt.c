/*************************************************************************************************/
/*!
 *  \file   hci_evt.c
 *
 *  \brief  HCI event module.
 *
 *          $Date: 2015-06-12 04:19:18 -0700 (Fri, 12 Jun 2015) $
 *          $Revision: 3061 $
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
#include "wsf_buf.h"
#include "wsf_msg.h"
#include "wsf_trace.h"
#include "bstream.h"
#include "hci_api.h"
#include "hci_main.h"
#include "hci_evt.h"
#include "hci_cmd.h"
#include "hci_core.h"

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! LL event to HCI callback event lookup table. */
static const uint8_t hciEvtLookup[] =
{
  HCI_RESET_SEQ_CMPL_CBACK_EVT,                        /* LL_RESET_CNF */
  HCI_LE_ADV_REPORT_CBACK_EVT,                         /* LL_ADV_REPORT_IND */
  0,                                                   /* LL_ADV_ENABLE_CNF */
  0,                                                   /* LL_SCAN_ENABLE_CNF */
  HCI_LE_CONN_CMPL_CBACK_EVT,                          /* LL_CONN_IND */
  HCI_DISCONNECT_CMPL_CBACK_EVT,                       /* LL_DISCONNECT_IND */
  HCI_LE_CONN_UPDATE_CMPL_CBACK_EVT,                   /* LL_CONN_UPDATE_IND */
  0,                                                   /* LL_REM_CONN_PARAM_IND */
  HCI_LE_CREATE_CONN_CANCEL_CMD_CMPL_CBACK_EVT,        /* LL_CREATE_CONN_CANCEL_CNF */
  HCI_READ_REMOTE_VER_INFO_CMPL_CBACK_EVT,             /* LL_READ_REMOTE_VER_INFO_CNF */
  HCI_LE_READ_REMOTE_FEAT_CMPL_CBACK_EVT,              /* LL_READ_REMOTE_FEAT_CNF */
  HCI_ENC_CHANGE_CBACK_EVT,                            /* LL_ENC_CHANGE_IND */
  HCI_ENC_KEY_REFRESH_CMPL_CBACK_EVT,                  /* LL_ENC_KEY_REFRESH_IND */
  HCI_LE_LTK_REQ_CBACK_EVT,                            /* LL_LTK_REQ_IND */
  HCI_LE_LTK_REQ_NEG_REPL_CMD_CMPL_CBACK_EVT,          /* LL_LTK_REQ_NEG_REPLY_CNF */
  HCI_LE_LTK_REQ_REPL_CMD_CMPL_CBACK_EVT,              /* LL_LTK_REQ_REPLY_CNF */
  HCI_HW_ERROR_CBACK_EVT                               /* LL_ERROR_IND */
};

/*************************************************************************************************/
/*!
 *  \fn     hciEvtProcessMsg
 *
 *  \brief  Process received HCI events.
 *
 *  \param  pEvt    Buffer containing HCI event.
 *
 *  \return None.
 */
/*************************************************************************************************/
void hciEvtProcessMsg(uint8_t *pEvt)
{
  llEvt_t *pMsg = (llEvt_t *)pEvt;

  uint8_t event = pMsg->hdr.event;

  /* convert hci event code to internal event code and perform special handling */
  switch (event)
  {
    case LL_RESET_CNF:
      /* restore LL state */
      LlGetBdAddr(hciCoreCb.bdAddr);

      /* reset internals */
      hciCoreCb.availBufs = hciCoreCb.numBufs;
      if (hciCb.secCback)
      {
        HciLeRandCmd();
        HciLeRandCmd();
      }
      hciCb.resetting = FALSE;

      /* propagate reset */
      pMsg->hdr.event = HCI_RESET_SEQ_CMPL_CBACK_EVT;
      hciCb.evtCback((hciEvt_t *)pMsg);
      break;

    case LL_ADV_REPORT_IND:
      break;
    case LL_ADV_ENABLE_CNF:
      break;
    case LL_SCAN_ENABLE_CNF:
      break;

    case LL_CONN_IND:
      hciCoreConnOpen(pMsg->connInd.handle);
      /* fall through */
      
    case LL_DISCONNECT_IND:
    case LL_CONN_UPDATE_IND:
    case LL_CREATE_CONN_CANCEL_CNF:
    case LL_READ_REMOTE_VER_INFO_CNF:
    case LL_READ_REMOTE_FEAT_CNF:
    case LL_ENC_CHANGE_IND:
    case LL_LTK_REQ_IND:
    case LL_LTK_REQ_NEG_REPLY_CNF:
    case LL_LTK_REQ_REPLY_CNF:
    case LL_ERROR_IND:
      /* lookup HCI event callback code */
      pMsg->hdr.event = hciEvtLookup[pMsg->hdr.event];
      /* HCI and LL event structures identical, no translation needed */
      hciCb.evtCback((hciEvt_t *)pMsg);

      if (event == LL_DISCONNECT_IND)
      {
        hciCoreConnClose(pMsg->disconnectInd.handle);
      }

      break;

    default:
      break;
  }
}
