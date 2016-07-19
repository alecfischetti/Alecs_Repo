/*************************************************************************************************/
/*!
 *  \file   wsf_math.h
 *
 *  \brief  Common math utilities.
 *
 *          $Date: 2015-08-27 05:31:35 -0700 (Thu, 27 Aug 2015) $
 *          $Revision: 3731 $
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
#ifndef WSF_MATH_H
#define WSF_MATH_H

#include "wsf_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! \brief      Returns the minimum of two values. */
#define WSF_MIN(a,b)        ((a) < (b) ? (a) : (b))

/*! \brief      Returns the maximum of two values. */
#define WSF_MAX(a,b)        ((a) > (b) ? (a) : (b))

/*! \brief      Binary divide with 1,000 divisor. */
#define WSF_MATH_DIV_10E3(n)    (((n) * UINT32_C(1048)) >> 20)

/*! \brief      Binary divide with 1,000,000 divisor. */
#define WSF_MATH_DIV_10E6(n)    ((uint32_t)(((uint64_t)(n) * UINT64_C(4295)) >> 32))

/*! \brief      Binary divide with 10 divisor. */
#define WSF_MATH_DIV_10(n)      ((uint32_t)(((uint64_t)(n) * UINT64_C(419431)) >> 22))

/*! \brief      Binary divide with 37 divisor. */
#define WSF_MATH_DIV_37(n)      (((n) * UINT32_C(56680)) >> 21)

/*! \brief      Binary modulo 37. */
#define WSF_MATH_MOD_37(n)      ((n) - (WSF_MATH_DIV_37(n) * 37))

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

void WsfMathInit(void);
uint32_t WsfRandNum(void);
void WsfAesEcb(const uint8_t *pKey, uint8_t *pOut, const uint8_t *pIn);

#ifdef __cplusplus
};
#endif

#endif /* WSF_MATH_H */
