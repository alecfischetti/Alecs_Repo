/*************************************************************************************************/
/*!
 *  \file   wsf_sec_ecc.c
 *        
 *  \brief  WSF Security ECC implementation using uECC.
 *
 *          $Date: 2015-08-31 05:26:49 -0700 (Mon, 31 Aug 2015) $
 *          $Revision: 3760 $
 *  
 *  Copyright (c) 2010 Wicentric, Inc., all rights reserved.
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

#include <stdlib.h>
#include <time.h>
#include <string.h>
#include "wsf_types.h"
#include "wsf_queue.h"
#include "wsf_msg.h"
#include "wsf_trace.h"
#include "wsf_sec.h"
#include "wsf_sec_int.h"
#include "wsf_buf.h"
#include "hci_api.h"
#include "calc128.h"
#include "uECC.h"

#ifndef WSF_SEC_ECC_DEBUG
#define WSF_SEC_ECC_DEBUG FALSE
#endif

#if WSF_SEC_ECC_DEBUG == FALSE

/**************************************************************************************************
  External Variables
**************************************************************************************************/

extern wsfSecCb_t wsfSecCb;

/*************************************************************************************************/
/*!
 *  \fn     wsfSecEccRng
 *        
 *  \brief  Random number generator used by uECC.
 *
 *  \param  p_dest      Buffer to hold random number
 *  \param  p_size      Size of p_dest in bytes .
 *
 *  \return TRUE if successful.
 */
/*************************************************************************************************/
static int wsfSecEccRng(uint8_t *p_dest, unsigned p_size)
{
  WsfSecRand(p_dest, p_size);
  return TRUE;
}

/*************************************************************************************************/
/*!
 *  \fn     WsfSecEccHciCback
 *
 *  \brief  Callback for HCI encryption for ECC operations.
 *
 *  \param  pBuf        Pointer to sec queue element.
 *  \param  pEvent      Pointer to HCI event.
 *  \param  handlerId   WSF handler ID.
 *
 *  \return none.
 */
/*************************************************************************************************/
void WsfSecEccHciCback(wsfSecQueueBuf_t *pBuf, hciEvt_t *pEvent, wsfHandlerId_t handlerId)
{
  /* TBD */
}

/*************************************************************************************************/
/*!
 *  \fn     WsfSecEccGenKey
 *        
 *  \brief  Generate an ECC key.
 *
 *  \param  handlerId   WSF handler ID for client.
 *  \param  param       Optional parameter sent to client's WSF handler.
 *  \param  event       Event for client's WSF handler.
 *
 *  \return None.
 */
/*************************************************************************************************/
uint8_t WsfSecEccGenKey(wsfHandlerId_t handlerId, uint16_t param, uint8_t event)
{
  wsfSecEccMsg_t *pMsg = WsfMsgAlloc(sizeof(wsfSecEccMsg_t));

  if (pMsg)
  {
    /* Generate the keys */ 
    uECC_make_key(pMsg->data.key.pubKey_x, pMsg->data.key.privKey);

    /* Send shared secret to handler */ 
    pMsg->hdr.event = event;
    pMsg->hdr.param = param;
    pMsg->hdr.status = HCI_SUCCESS;
    WsfMsgSend(handlerId, pMsg);

    return TRUE;
  }

  return FALSE;
}

/*************************************************************************************************/
/*!
 *  \fn     WsfSecEccGenSharedSecret
 *        
 *  \brief  Generate an ECC key.
 *
 *  \param  pKey        ECC Key structure.
 *  \param  handlerId   WSF handler ID for client.
 *  \param  param       Optional parameter sent to client's WSF handler.
 *  \param  event       Event for client's WSF handler.
 *
 *  \return None.
 */
/*************************************************************************************************/
uint8_t WsfSecEccGenSharedSecret(wsfSecEccKey_t *pKey, wsfHandlerId_t handlerId, uint16_t param, uint8_t event)
{
  wsfSecEccMsg_t *pMsg = WsfMsgAlloc(sizeof(wsfSecEccMsg_t));

  if (pMsg)
  {
    uECC_shared_secret(pKey->pubKey_x, pKey->privKey, pMsg->data.sharedSecret.secret);

    /* Send shared secret to handler */  
    pMsg->hdr.event = event;
    pMsg->hdr.param = param;
    pMsg->hdr.status = HCI_SUCCESS;
    WsfMsgSend(handlerId, pMsg);

    return TRUE;
  }

  return FALSE;
}

/*************************************************************************************************/
/*!
 *  \fn     WsfSecEccInit
 *        
 *  \brief  Called to initialize ECC security.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WsfSecEccInit()
{
  srand((unsigned int)time(NULL));
  uECC_set_rng(wsfSecEccRng);
}

#endif /* WSF_SEC_ECC_DEBUG */
