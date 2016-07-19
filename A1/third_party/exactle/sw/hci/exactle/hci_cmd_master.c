/*************************************************************************************************/
/*!
 *  \file   hci_cmd.c
 *
 *  \brief  HCI command module.
 *
 *          $Date: 2013-08-25 14:47:01 -0700 (Sun, 25 Aug 2013) $
 *          $Revision: 906 $
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
#include "wsf_assert.h"
#include "wsf_queue.h"
#include "wsf_timer.h"
#include "wsf_msg.h"
#include "wsf_trace.h"
#include "bstream.h"
#include "hci_cmd.h"
#include "hci_tr.h"
#include "hci_api.h"
#include "hci_main.h"

#include "ll_api.h"

/*************************************************************************************************/
/*!
 *  \fn     HciLeSetScanEnableCmd
 *
 *  \brief  HCI LE set scan enable command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeSetScanEnableCmd(uint8_t enable, uint8_t filterDup)
{
  LlScanEnable(enable, filterDup);
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeSetScanParamCmd
 *
 *  \brief  HCI set scan parameters command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeSetScanParamCmd(uint8_t scanType, uint16_t scanInterval, uint16_t scanWindow,
                          uint8_t ownAddrType, uint8_t scanFiltPolicy)
{
  uint8_t status;
  (void)status;

  status = LlSetScanParam(scanType,
                          scanInterval,
                          scanWindow,
                          ownAddrType,
                          scanFiltPolicy);

  WSF_ASSERT(status == LL_SUCCESS);
}
