/*************************************************************************************************/
/*!
 *  \file   wsf_sec_ecc.c
 *
 *  \brief  WSF Security ECC implementation using debug keys.
 *
 *          $Date: 2015-09-05 09:01:07 -0700 (Sat, 05 Sep 2015) $
 *          $Revision: 3793 $
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

#ifndef WSF_SEC_ECC_DEBUG
#define WSF_SEC_ECC_DEBUG TRUE
#endif

#if WSF_SEC_ECC_DEBUG == TRUE

/* Debug Keys */
static const uint8_t debugPrivateKey[] = {0x3f, 0x49, 0xf6, 0xd4,  0xa3, 0xc5, 0x5f, 0x38,
                                          0x74, 0xc9, 0xb3, 0xe3,  0xd2, 0x10, 0x3f, 0x50,
                                          0x4a, 0xff, 0x60, 0x7b,  0xeb, 0x40, 0xb7, 0x99,
                                          0x58, 0x99, 0xb8, 0xa6,  0xcd, 0x3c, 0x1a, 0xbd};

static const uint8_t debugPublicKeyX[] = {0x20, 0xb0, 0x03, 0xd2,  0xf2, 0x97, 0xbe, 0x2c,
                                          0x5e, 0x2c, 0x83, 0xa7,  0xe9, 0xf9, 0xa5, 0xb9,
                                          0xef, 0xf4, 0x91, 0x11,  0xac, 0xf4, 0xfd, 0xdb,
                                          0xcc, 0x03, 0x01, 0x48,  0x0e, 0x35, 0x9d, 0xe6};

static const uint8_t debugPublicKeyY[] = {0xdc, 0x80, 0x9c, 0x49,  0x65, 0x2a, 0xeb, 0x6d,
                                          0x63, 0x32, 0x9a, 0xbf,  0x5a, 0x52, 0x15, 0x5c,
                                          0x76, 0x63, 0x45, 0xc2,  0x8f, 0xed, 0x30, 0x24,
                                          0x74, 0x1c, 0x8e, 0xd0,  0x15, 0x89, 0xd2, 0x8b};

static const uint8_t debugSharedSecret[] = {0x49, 0x4c, 0xfd, 0x99, 0x6f, 0x40, 0x17, 0xf5,
                                            0xb5, 0x48, 0xba, 0x66, 0x99, 0x60, 0x64, 0x08,
                                            0x62, 0x75, 0xd5, 0x1f, 0xe0, 0x8e, 0x56, 0x36,
                                            0xb9, 0x36, 0xd1, 0xe4, 0x57, 0x46, 0x4b, 0xed};

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
    memcpy(pMsg->data.key.pubKey_x, debugPublicKeyX, WSF_ECC_KEY_LEN);
    memcpy(pMsg->data.key.pubKey_y, debugPublicKeyY, WSF_ECC_KEY_LEN);
    memcpy(pMsg->data.key.privKey, debugPrivateKey, WSF_ECC_KEY_LEN);

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
    memcpy(pMsg->data.sharedSecret.secret, debugSharedSecret, WSF_ECC_KEY_LEN);

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

}

#endif /* WSF_SEC_ECC_DEBUG */
