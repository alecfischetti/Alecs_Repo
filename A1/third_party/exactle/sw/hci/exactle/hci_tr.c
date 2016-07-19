/*************************************************************************************************/
/*!
 *  \file   hci_tr.c
 *
 *  \brief  HCI transport module.
 *
 *          $Date: 2015-06-12 04:19:18 -0700 (Fri, 12 Jun 2015) $
 *          $Revision: 3061 $
 *
 *  Copyright (c) 2011 Wicentric, Inc., all rights reserved.
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
#include "bstream.h"
#include "hci_api.h"
#include "hci_core.h"
#include "hci_tr.h"
#include "ll_api.h"

/*************************************************************************************************/
/*!
 *  \fn     hciTrSendAclData
 *        
 *  \brief  Send a complete HCI ACL packet to the transport. 
 *
 *  \param  pContext Connection context.
 *  \param  pData    WSF msg buffer containing an ACL packet.
 *
 *  \return None.
 */
/*************************************************************************************************/
void hciTrSendAclData(void *pContext, uint8_t *pData)
{
  uint16_t len;
  uint8_t  *p;

  /* get 16-bit length */
  BYTES_TO_UINT16(len, (pData + 2))
  len += HCI_ACL_HDR_LEN;
  
  /* allocate LL buffer */
  if ((p = WsfMsgAlloc(len + BB_PDU_TAILROOM)) != NULL)
  {
    /* copy data */
    memcpy(p, pData, len);

    /* send to LL  */
    LlSendAclData(p);
    
    /* free HCI buffer */
    hciCoreTxAclComplete(pContext, pData);
  }
}
