/*************************************************************************************************/
/*!
 *  \file   mouse_api.h
 *
 *  \brief  HID Mouse sample application.
 *
 *          $Date: 2015-09-14 07:24:29 -0700 (Mon, 14 Sep 2015) $
 *          $Revision: 3894 $
 *
 *  Copyright (c) 2015 Wicentric, Inc., all rights reserved.
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
#ifndef MOUSE_API_H
#define MOUSE_API_H

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/
/*************************************************************************************************/
/*!
 *  \fn     MouseStart
 *        
 *  \brief  Start the application.
 *
 *  \return None.
 */
/*************************************************************************************************/
void MouseStart(void);

/*************************************************************************************************/
/*!
 *  \fn     MouseHandlerInit
 *        
 *  \brief  Application handler init function called during system initialization.
 *
 *  \param  handlerID  WSF handler ID for App.
 *
 *  \return None.
 */
/*************************************************************************************************/
void MouseHandlerInit(wsfHandlerId_t handlerId);


/*************************************************************************************************/
/*!
 *  \fn     MouseHandler
 *        
 *  \brief  WSF event handler for the application.
 *
 *  \param  event   WSF event mask.
 *  \param  pMsg    WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void MouseHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg);

#ifdef __cplusplus
};
#endif

#endif /* MOUSE_API_H */
