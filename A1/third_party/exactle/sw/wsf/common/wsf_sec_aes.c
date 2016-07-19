/*************************************************************************************************/
/*!
 *  \file   wsf_sec_aes.c
 *        
 *  \brief  AES and random number security service implemented using HCI.
 *
 *          $Date: 2015-06-12 04:19:18 -0700 (Fri, 12 Jun 2015) $
 *          $Revision: 3061 $
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

#include <string.h>
#include "wsf_types.h"
#include "wsf_queue.h"
#include "wsf_msg.h"
#include "wsf_trace.h"
#include "wsf_sec.h"
#include "wsf_sec_int.h"
#include "hci_api.h"
#include "calc128.h"

/**************************************************************************************************
  External Variables
**************************************************************************************************/

extern wsfSecCb_t wsfSecCb;

/*************************************************************************************************/
/*!
 *  \fn     WsfSecAes
 *        
 *  \brief  Execute an AES calculation.  When the calculation completes, a WSF message will be
 *          sent to the specified handler.  This function returns a token value that
 *          the client can use to match calls to this function with messages.
 *
 *  \param  pKey        Pointer to 16 byte key.
 *  \param  pPlaintext  Pointer to 16 byte plaintext.
 *  \param  handlerId   WSF handler ID.
 *  \param  param       Client-defined parameter returned in message.
 *  \param  event       Event for client's WSF handler.
 *
 *  \return Token value.
 */
/*************************************************************************************************/
uint8_t WsfSecAes(uint8_t *pKey, uint8_t *pPlaintext, wsfHandlerId_t handlerId,
                  uint16_t param, uint8_t event)
{
  wsfSecQueueBuf_t  *pBuf;
  
  /* allocate a buffer */
  if ((pBuf = WsfMsgAlloc(sizeof(wsfSecQueueBuf_t))) != NULL)
  {
    pBuf->msg.hdr.status = wsfSecCb.token++;
    pBuf->msg.hdr.param = param;
    pBuf->msg.hdr.event = event;
   
    pBuf->type = WSF_SEC_TYPE_AES;

    /* queue buffer */
    WsfMsgEnq(&wsfSecCb.queue, handlerId, pBuf);
    
    /* call HCI encrypt function */
    HciLeEncryptCmd(pKey, pPlaintext);
  }
  
  return pBuf->msg.hdr.status;
}

/*************************************************************************************************/
/*!
 *  \fn     WsfSecAesHciCback
 *        
 *  \brief  Callback for HCI encryption for AES operations.
 *
 *  \param  pBuf        Pointer to sec queue element.
 *  \param  pEvent      Pointer to HCI event.
 *  \param  handlerId   WSF handler ID.
 *
 *  \return none.
 */
/*************************************************************************************************/
void WsfSecAesHciCback(wsfSecQueueBuf_t *pBuf, hciEvt_t *pEvent, wsfHandlerId_t handlerId)
{
  wsfSecAes_t *pAes = (wsfSecAes_t *) &pBuf->msg;
  
  /* set encrypted data pointer and copy */
  pAes->pCiphertext = pBuf->ciphertext;
  Calc128Cpy(pAes->pCiphertext, pEvent->leEncryptCmdCmpl.data);

  /* send message */
  WsfMsgSend(handlerId, pAes);
}

/*************************************************************************************************/
/*!
 *  \fn     WsfSecAesInit
 *        
 *  \brief  Called to initialize AES secuirity.
 *
 *  \param  none.
 *
 *  \return none.
 */
/*************************************************************************************************/
void WsfSecAesInit()
{
  wsfSecCb.hciCbackTbl[WSF_SEC_TYPE_AES] = WsfSecAesHciCback;
}
