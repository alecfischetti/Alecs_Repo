/*************************************************************************************************/
/*!
 *  \file   meds_main.h
 *        
 *  \brief  Health/medical sensor sample application interface file.
 *
 *          $Date: 2012-12-27 12:15:33 -0800 (Thu, 27 Dec 2012) $
 *          $Revision: 403 $
 *  
 *  Copyright (c) 2012 Wicentric, Inc., all rights reserved.
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

#ifndef MEDS_MAIN_H
#define MEDS_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! WSF message event starting value */
#define MEDS_MSG_START             0xA0

/*! WSF message event enumeration */
enum
{
  MEDS_TIMER_IND = MEDS_MSG_START,    /*! Timer expired */
};

/**************************************************************************************************
  Data Types
**************************************************************************************************/

/*! profile interface callback functions */
typedef void (*medsStartCback_t)(void);
typedef void (*medsProcMsgCback_t)(wsfMsgHdr_t *pMsg);
typedef void (*medsBtnCback_t)(dmConnId_t connId, uint8_t btn);

/*! profile interface structure */
typedef struct
{
  medsStartCback_t      start;
  medsProcMsgCback_t    procMsg;
  medsBtnCback_t        btn;
} medsIf_t;

/*! application control block */
typedef struct
{
  medsIf_t          *pIf;                           /*! Profile interface */
  wsfHandlerId_t    handlerId;                      /*! WSF hander ID */
} medsCb_t;

/**************************************************************************************************
  Global Variables
**************************************************************************************************/

/*! application control block */
extern medsCb_t medsCb;

/*! profile interface pointers */
extern medsIf_t medsBlpIf;          /* blood pressure profile */
extern medsIf_t medsWspIf;          /* weight scale profile */
extern medsIf_t medsHtpIf;          /* health thermometer profile */

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/

void medsCccCback(attsCccEvt_t *pEvt);

#ifdef __cplusplus
};
#endif

#endif /* MEDS_MAIN_H */

