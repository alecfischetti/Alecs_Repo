/*************************************************************************************************/
/*!
 *  \file   wsf_sec.c
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
#include "wsf_assert.h"
#include "wsf_sec.h"
#include "wsf_sec_int.h"
#include "hci_api.h"
#include "calc128.h"

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/* WSF Security Control block */
wsfSecCb_t wsfSecCb;

/*************************************************************************************************/
/*!
 *  \fn     wsfSecHciCback
 *        
 *  \brief  Callback for HCI encryption and random number events.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void wsfSecHciCback(hciEvt_t *pEvent)
{
  wsfSecQueueBuf_t  *pBuf;
  wsfHandlerId_t    handlerId;
  
  /* handle random number event */
  if (pEvent->hdr.event == HCI_LE_RAND_CMD_CMPL_CBACK_EVT)
  {
    /* move up data by eight bytes */    
    memcpy(&wsfSecCb.rand[HCI_RAND_LEN], wsfSecCb.rand, HCI_RAND_LEN * (WSF_HCI_RAND_MULT-1));
    
    /* copy new data to random data buffer */
    memcpy(wsfSecCb.rand, pEvent->leRandCmdCmpl.randNum, HCI_RAND_LEN);
  }
  /* handle encryption event */
  else if (pEvent->hdr.event == HCI_LE_ENCRYPT_CMD_CMPL_CBACK_EVT)
  {
    if ((pBuf = WsfMsgDeq(&wsfSecCb.queue, &handlerId)) != NULL)
    {
      /* Process HCI callback based on type of encryption */ 
      if (wsfSecCb.hciCbackTbl[pBuf->type])
      {
        wsfSecCb.hciCbackTbl[pBuf->type](pBuf, pEvent, handlerId);
      }
      else
      {
        WSF_TRACE_WARN0("WSF sec not registered");
      }
    }
    else
    {
      WSF_TRACE_WARN0("WSF sec queue empty!");
    }
  }
}

/*************************************************************************************************/
/*!
 *  \fn     WsfSecInit
 *        
 *  \brief  Initialize the security service.  This function should only be called once
 *          upon system initialization.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WsfSecInit(void)
{
  WSF_QUEUE_INIT(&wsfSecCb.queue);
  wsfSecCb.token = 0;
  
  /* Register callback with HCI */
  HciSecRegister(wsfSecHciCback);
}

/*************************************************************************************************/
/*!
 *  \fn     WsfSecRandInit
 *        
 *  \brief  Initialize the random number service.  This function should only be called once
 *          upon system initialization.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WsfSecRandInit(void)
{
  int8_t i;

  /* get new random numbers */
  for (i=0; i<WSF_HCI_RAND_MULT; i++)
  {
    HciLeRandCmd();
  }
}

/*************************************************************************************************/
/*!
 *  \fn     WsfSecRand
 *        
 *  \brief  This function returns up to 16 bytes of random data to a buffer provided by the
 *          client.
 *
 *  \param  pRand       Pointer to returned random data.
 *  \param  randLen     Length of random data.
 *
 *  \return None.
 */
/*************************************************************************************************/
void WsfSecRand(uint8_t *pRand, uint8_t randLen)
{
  int8_t count = (randLen + HCI_RAND_LEN - 1) / HCI_RAND_LEN;

  WSF_ASSERT(randLen <= HCI_RAND_LEN * WSF_HCI_RAND_MULT);

  /* copy data */
  memcpy(pRand, wsfSecCb.rand, randLen);
  
  /* get new random numbers */
  while (count--)
  {
    HciLeRandCmd();
  }
}
