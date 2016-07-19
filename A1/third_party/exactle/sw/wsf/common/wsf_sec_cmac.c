/*************************************************************************************************/
/*!
 *  \file   wsf_sec_cmac.c
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
#include "wsf_buf.h"
#include "hci_api.h"
#include "calc128.h"

enum
{
  WSF_SEC_CMAC_STATE_SUBKEY,
  WSF_SEC_CMAC_STATE_BLOCK,  
  WSF_SEC_CMAC_STATE_COMPLETE,  
};

/**************************************************************************************************
  External Variables
**************************************************************************************************/

extern wsfSecCb_t wsfSecCb;

static void wsfCmacReverseCpy(uint8_t *pBuf1, const uint8_t *pBuf2, uint8_t len)
{ 
  int8_t i;

  for (i=0; i<len; i++)
  {
    pBuf1[WSF_SEC_BLOCK_LEN-1-i] = pBuf2[i];
  }
}

static void wsfCmacReverse(uint8_t *pBuf, uint8_t len)
{
  uint8_t i, temp;
  
  for (i=0; i<len/2; i++)
  {
    temp = pBuf[len-i-1];
    pBuf[len-i-1] = pBuf[i];
    pBuf[i] = temp;
  }
}

/*************************************************************************************************/
/*!
 *  \fn     wsfSecCmacProcessBlock
 *        
 *  \brief  Continue the execution of the CMAC algorithm over the next message block.
 *
 *  \param  pBuf    Security queue buffer containing CMAC algorithm control block. 
 *
 *  \return None.
 */
/*************************************************************************************************/
static void wsfSecCmacProcessBlock(wsfSecQueueBuf_t *pBuf)
{
  wsfSecCmacSecCb_t *pCmac = (wsfSecCmacSecCb_t*) pBuf->pCb;
  uint8_t buf[WSF_SEC_BLOCK_LEN];
  uint8_t *pMn = pCmac->pPlainText + pCmac->position;
  int8_t remaining = (int16_t) pCmac->len - pCmac->position;
    
  /* Check for Last Block */
  if (remaining <= WSF_SEC_BLOCK_LEN)
  {
    wsfCmacReverseCpy(buf, pMn, remaining);

    /* Pad the message if necessary */
    if (remaining != WSF_SEC_BLOCK_LEN)
    {
      memset(buf, 0, WSF_SEC_BLOCK_LEN - remaining);
      buf[WSF_SEC_BLOCK_LEN-remaining-1] = 0x80;
    }
    
    /* XOr the subkey */
    Calc128Xor(buf, pCmac->subkey);
    pCmac->state = WSF_SEC_CMAC_STATE_COMPLETE;
  }
  else
  {
    /* Copy the block to the buffer */
    wsfCmacReverseCpy(buf, pMn, WSF_SEC_BLOCK_LEN);
  }

  if (pCmac->position != 0)
  {
    /* Except for first block, XOr the previous AES calculation */
    Calc128Xor(buf, pBuf->ciphertext);
  }

  pCmac->position += WSF_SEC_BLOCK_LEN;

  /* Enqueue and perform AES operation */
  WsfMsgEnq(&wsfSecCb.queue, pCmac->handlerId, pBuf);
  HciLeEncryptCmd(pCmac->key, buf);
}

/*************************************************************************************************/
/*!
 *  \fn     wsfSecCmacGenSubkey1
 *        
 *  \brief  Step 1 to generate the subkey used in the CMAC algorithm.
 *
 *  \param  pBuf    Security queue buffer containing CMAC algorithm control block. 
 *
 *  \return None.
 */
/*************************************************************************************************/
static void wsfSecCmacGenSubkey1(wsfSecQueueBuf_t *pBuf)
{
  wsfSecCmacSecCb_t *pCmac = (wsfSecCmacSecCb_t*) pBuf->pCb;
  uint8_t buf[WSF_SEC_BLOCK_LEN];
  
  /* Perform aes on the key with a constant zero */
  memset(buf, 0, WSF_SEC_BLOCK_LEN);
  
  WsfMsgEnq(&wsfSecCb.queue, pCmac->handlerId, pBuf);
  HciLeEncryptCmd(pCmac->key, buf);
}

/*************************************************************************************************/
/*!
 *  \fn     wsfSecCmacKeyShift
 *        
 *  \brief  Left shift a buffer of WSF_CMAC_KEY_LEN bytes by N bits.
 *
 *  \param  pBuf    Buffer to left shift. 
 *  \param  shift   Number of bits to shift. 
 *
 *  \return None.
 */
/*************************************************************************************************/
static uint8_t wsfSecCmacKeyShift(uint8_t *pBuf, uint8_t shift)
{
  uint8_t bits, i, prevBits = 0;
  
  for (i = 0; i < WSF_CMAC_KEY_LEN; i++)
  {
    /* store shifted bits for next byte */
    bits = pBuf[i] >> (8 - shift);

    /* shift byte and OR in shifted bits from previous byte */
    pBuf[i] = (pBuf[i] << shift) | prevBits;

    prevBits = bits;
  }
  
  return prevBits;
}

/*************************************************************************************************/
/*!
 *  \fn     wsfSecCmacGenSubkey2
 *        
 *  \brief  Complete generation of the subkey used in the CMAC algorithm.
 *
 *  \param  pBuf    Security queue buffer containing CMAC algorithm control block. 
 *
 *  \return None.
 */
/*************************************************************************************************/
static void wsfSecCmacGenSubkey2(wsfSecQueueBuf_t *pBuf)
{
  wsfSecCmacSecCb_t *pCmac = (wsfSecCmacSecCb_t*) pBuf->pCb;
  uint8_t overflow;
  
  /* Copy the result of the AES oepration */
  Calc128Cpy(pCmac->subkey, pBuf->ciphertext);
  
  /* Calculate the K1 subkey */
  overflow = wsfSecCmacKeyShift(pCmac->subkey, 1);
  
  if (overflow)
  {
    pCmac->subkey[0] ^= WSF_SEC_CMAC_RB;
  }
      
  if (pCmac->len % WSF_SEC_BLOCK_LEN != 0)
  {
    /* If the message len is not a multiple of WSF_SEC_BLOCK_LEN */
    /* Continue with generation of the K2 subkey based on the K1 key */
    overflow = wsfSecCmacKeyShift(pCmac->subkey, 1);
    
    if (overflow)
    {
      pCmac->subkey[0] ^= WSF_SEC_CMAC_RB;
    }
  }
    
  /* Begin CMAC calculation */
  pCmac->state = WSF_SEC_CMAC_STATE_BLOCK;
  wsfSecCmacProcessBlock(pBuf);
}

/*************************************************************************************************/
/*!
 *  \fn     wsfSecCmacComplete
 *        
 *  \brief  Send a message to the handler with CMAC result.
 *
 *  \param  pBuf    Security queue buffer containing CMAC algorithm control block. 
 *
 *  \return None.
 */
/*************************************************************************************************/
static void wsfSecCmacComplete(wsfSecQueueBuf_t *pBuf)
{
  /* CMAC is complete, copy and send result to handler */
  wsfSecCmacMsg_t *pMsg = (wsfSecCmacMsg_t *) &pBuf->msg;
  wsfSecCmacSecCb_t *pCmac = (wsfSecCmacSecCb_t*) pBuf->pCb;

  wsfCmacReverse(pBuf->ciphertext, WSF_CMAC_KEY_LEN);
  pMsg->pCiphertext = pBuf->ciphertext;
  WsfBufFree(pCmac->pPlainText);
  
  WsfMsgSend(pCmac->handlerId, pMsg);
}

/*************************************************************************************************/
/*!
 *  \fn     WsfSecCmacHciCback
 *        
 *  \brief  Callback for HCI encryption for CMAC operations.
 *
 *  \param  pBuf        Pointer to sec queue element.
 *  \param  pEvent      Pointer to HCI event.
 *  \param  handlerId   WSF handler ID.
 *
 *  \return none.
 */
/*************************************************************************************************/
void WsfSecCmacHciCback(wsfSecQueueBuf_t *pBuf, hciEvt_t *pEvent, wsfHandlerId_t handlerId)
{
  wsfSecCmacSecCb_t *pCmac = (wsfSecCmacSecCb_t *) pBuf->pCb;

  if (pCmac)
  {
    Calc128Cpy(pBuf->ciphertext, pEvent->leEncryptCmdCmpl.data);

    switch (pCmac->state)
    {
    case WSF_SEC_CMAC_STATE_SUBKEY:
      wsfSecCmacGenSubkey2(pBuf);
      break;
      
    case WSF_SEC_CMAC_STATE_BLOCK:
      wsfSecCmacProcessBlock(pBuf);
      break;
      
    case WSF_SEC_CMAC_STATE_COMPLETE:
      wsfSecCmacComplete(pBuf);
      break;
    }
  }
}

/*************************************************************************************************/
/*!
 *  \fn     WsfSecAesCmac
 *        
 *  \brief  Execute the CMAC algorithm.
 *
 *  \param  pKey        Key used in CMAC operation.
 *  \param  pPlainText  Data to perform CMAC operation over
 *  \param  len         Size of pPlaintext in bytes.
 *  \param  handlerId   WSF handler ID for client.
 *  \param  param       Optional parameter sent to client's WSF handler.
 *  \param  event       Event for client's WSF handler.
 *
 *  \return TRUE if an error, else FALSE.
 */
/*************************************************************************************************/
uint8_t WsfSecCmac(const uint8_t *pKey, uint8_t *pPlainText, uint8_t textLen, wsfHandlerId_t handlerId, uint16_t param, uint8_t event)
{
  wsfSecQueueBuf_t *pBuf;
  uint16_t bufSize = sizeof(wsfSecQueueBuf_t) + sizeof(wsfSecCmacSecCb_t);
  
  if ((pBuf = WsfMsgAlloc(bufSize)) != NULL)
  {
    wsfSecCmacSecCb_t *pCmacCb = (wsfSecCmacSecCb_t *) (pBuf + 1);
    
    /* Setup queue buffer */    
    pBuf->pCb = pCmacCb;
    pBuf->type = WSF_SEC_TYPE_CMAC;

    pBuf->msg.hdr.status = wsfSecCb.token++;
    pBuf->msg.hdr.param = param;
    pBuf->msg.hdr.event = event;

    pCmacCb->pPlainText = pPlainText;
    
    pCmacCb->len = textLen;
    pCmacCb->position = 0;
    pCmacCb->handlerId = handlerId;
    pCmacCb->state = WSF_SEC_CMAC_STATE_SUBKEY;
    
    /* Copy key */
    wsfCmacReverseCpy(pCmacCb->key, pKey, WSF_CMAC_KEY_LEN);
    
    /* Start the CMAC process by calculating the subkey */
    wsfSecCmacGenSubkey1(pBuf);
    
    return FALSE;
  }
  
  return TRUE;
}

/*************************************************************************************************/
/*!
 *  \fn     WsfSecCmacInit
 *        
 *  \brief  Called to initialize CMAC secuirity.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WsfSecCmacInit()
{
  wsfSecCb.hciCbackTbl[WSF_SEC_TYPE_CMAC] = WsfSecCmacHciCback;
}