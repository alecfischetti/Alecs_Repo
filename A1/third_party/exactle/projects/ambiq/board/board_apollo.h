/*************************************************************************************************/
/*!
 *  \file   board.h
 *
 *  \brief  Common Ambiq board definitions.
 *
 *          $Date: 2015-02-19 21:09:31 -0800 (Thu, 19 Feb 2015) $
 *          $Revision: 2343 $
 *
 *  Copyright (c) 2013 Wicentric, Inc., all rights reserved.
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

#ifndef BOARD_BRAZO_H
#define BOARD_BRAZO_H

/**************************************************************************************************
  Functions
**************************************************************************************************/

void BoardHwInit(void);
void BoardPlatformInit(void);
void BoardStackInit(void);

void BoardTimerService(void);
bool_t BoardTimerSetWakeup(void);

void BoardSpiRecvComplete(uint8_t type, uint8_t *pBuf, uint8_t len);
void BoardSpiSendComplete(void);

#endif  /* BOARD_BRAZO_H */
