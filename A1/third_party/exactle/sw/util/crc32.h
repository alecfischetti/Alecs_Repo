/*************************************************************************************************/
/*!
 *  \file   crc32.h
 *
 *  \brief  CRC-32 utilities.
 *
 *          $Date: 2014-03-29 06:30:22 -0700 (Sat, 29 Mar 2014) $
 *          $Revision: 1275 $
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
#ifndef CRC32_H
#define CRC32_H

#ifdef __cplusplus
extern "C" {
#endif

/*************************************************************************************************/
/*!
 *  \fn     CalcCrc32
 *        
 *  \brief  Calculate the CRC-32 of the given buffer.
 *
 *  \param  len   Length fo the buffer.
 *  \param  pBuf  Buffer to compute the CRC.
 *
 *  \return None.
 *
 *  This routine was originally generated with crcmod.py using the following parameters:
 *    - polynomial 0x104C11DB7
 *    - bit reverse algorithm
 */
/*************************************************************************************************/
uint32_t CalcCrc32(int len, uint8_t *pBuf);

#ifdef __cplusplus
};
#endif

#endif /* CRC32_H */
