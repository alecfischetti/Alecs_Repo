/*************************************************************************************************/
/*!
 *  \file   meds_api.h
 *
 *  \brief  Medical sensor sample application interface.
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
#ifndef MEDS_API_H
#define MEDS_API_H

#include "wsf_os.h"

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Profile identifier used for MedsSetProfile() */
enum
{
  MEDS_ID_BLP,      /*! Blood pressure profile */
  MEDS_ID_WSP,      /*! Weight scale profile */
  MEDS_ID_HTP       /*! Health thermometer profile */
};

/**************************************************************************************************
  Function Declarations
**************************************************************************************************/
/*************************************************************************************************/
/*!
 *  \fn     MedsStart
 *        
 *  \brief  Start the application.
 *
 *  \return None.
 */
/*************************************************************************************************/
void MedsStart(void);

/*************************************************************************************************/
/*!
 *  \fn     MedsHandlerInit
 *        
 *  \brief  Application handler init function called during system initialization.
 *
 *  \param  handlerID  WSF handler ID for App.
 *
 *  \return None.
 */
/*************************************************************************************************/
void MedsHandlerInit(wsfHandlerId_t handlerId);


/*************************************************************************************************/
/*!
 *  \fn     MedsHandler
 *        
 *  \brief  WSF event handler for the application.
 *
 *  \param  event   WSF event mask.
 *  \param  pMsg    WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void MedsHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg);

/*************************************************************************************************/
/*!
 *  \fn     MedsSetProfile
 *        
 *  \brief  Set the profile to be used by the application.  This function is called internally
 *          by MedsHandlerInit() with a default value.  It may also be called by the system
 *          to configure the profile after executing MedsHandlerInit() and before executing
 *          MedsStart().
 *
 *  \param  profile  Profile identifier.
 *
 *  \return None.
 */
/*************************************************************************************************/
void MedsSetProfile(uint8_t profile);

#ifdef __cplusplus
};
#endif

#endif /* MEDS_API_H */
