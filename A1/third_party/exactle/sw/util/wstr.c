/*************************************************************************************************/
/*!
 *  \file   wstring.h
 *        
 *  \brief  Wicentric string manipulation functions.
 *
 *          $Date: 2014-03-29 06:30:22 -0700 (Sat, 29 Mar 2014) $
 *          $Revision: 1275 $
 *  
 *  Copyright (c) 2014 Wicentric, Inc., all rights reserved.
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
#include "wstr.h"

/*************************************************************************************************/
/*!
 *  \fn     WstrnCpy
 *        
 *  \brief  Copy a string and zero out space after the string length.
 *
 *  \return none.
 */
/*************************************************************************************************/
void WstrnCpy(uint8_t *pBuf, uint8_t *pData, uint8_t n)
{
  uint8_t i;
  uint8_t zeroing = FALSE;
  
  for (i=0; i<n; i++)
  {
    if (!zeroing && pData[i] == '\0')
      zeroing = TRUE;
    
    if (zeroing)
      *pBuf++ = 0;
    else
      *pBuf++ = pData[i];
  }
}