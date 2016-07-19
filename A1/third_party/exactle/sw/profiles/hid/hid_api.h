/*************************************************************************************************/
/*!
 *  \file   hid_api.h
 *
 *  \brief  Himan Interface Device Profile.
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
#ifndef HID_API_H
#define HID_API_H

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
  Constant Values
**************************************************************************************************/

/* Type of HID device */
#define HID_DEVICE_TYPE_KEYBOARD                0
#define HID_DEVICE_TYPE_MOUSE                   1
#define HID_DEVICE_TYPE_GENERIC                 2

/* Type of HID Information */
#define HID_INFO_CONTROL_POINT                  0
#define HID_INFO_PROTOCOL_MODE                  1

/**************************************************************************************************
  Callback Function Types
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \fn     hidOutputReportCback_t
 *        
 *  \brief  This callback function sends a received Output Report to the application.
 *
 *  \param  connId    The connection identifier.
 *  \param  id        The ID of the report.
 *  \param  len       The length of the report data in pReport.
 *  \param  pReport   A buffer containing the report.
 *
 *  \return None.
 */
/*************************************************************************************************/
typedef void (*hidOutputReportCback_t)(dmConnId_t connId, uint8_t id, uint16_t len, uint8_t *pReport);

/*************************************************************************************************/
/*!
 *  \fn     hidFeatureReportCback_t
 *        
 *  \brief  This callback function sends a received Feature Report to the application.
 *
 *  \param  connId    The connection identifier.
 *  \param  id        The ID of the report.
 *  \param  len       The length of the report data in pReport.
 *  \param  pReport   A buffer containing the report.
 *
 *  \return None.
 */
/*************************************************************************************************/
typedef void (*hidFeatureReportCback_t)(dmConnId_t connId, uint8_t id, uint16_t len, uint8_t *pReport);

/*************************************************************************************************/
/*!
 *  \fn     hidInfoCback_t
 *        
 *  \brief  This callback function notifies the application of a change in the protocol mode or 
 *          control point from the host.
 *
 *  \param  connId    The connection identifier.
 *  \param  mode      The type of information (HID_INFO_CONTROL_POINT or HID_INFO_PROTOCOL_MODE)
 *  \param  value     The value of the information
 *
 *  \return None.
 */
/*************************************************************************************************/
typedef void (*hidInfoCback_t)(dmConnId_t connId, uint8_t type, uint8_t value);

/* HID Report Type/ID to Attribute handle map item */
typedef struct
{
  uint8_t type;
  uint8_t id;
  uint16_t handle;
} hidReportIdMap_t;

/* HID Profile Configuration */
typedef struct
{
  uint8_t                     deviceType;           /* The type of HID device (keyboard, mouse, or generic) */
  uint8_t                     *pReportMap;          /* The report map, aslo called report descriptor */
  uint16_t                    reportMapSize;        /* The size of the report map in bytes (pReportMap) */
  hidReportIdMap_t            *pReportIdMap;        /* A map between report Type/ID and Attribute handle */
  uint8_t                     reportIdMapSize;      /* The number of Reports in the ID map (pReportIdMap) */
  hidOutputReportCback_t      outputCback;          /* Callback called on receipt of an Output Report */
  hidFeatureReportCback_t     featureCback;         /* Callback called on receipt of a Feature Report */
  hidInfoCback_t              infoCback;            /* Callback called on receipt of protocol mode or control point */
} hidConfig_t;

/**************************************************************************************************
  API Functions
**************************************************************************************************/

/*************************************************************************************************/
/*!
 *  \fn     HidSendInputReport
 *        
 *  \brief  Sends an input report to the host
 *
 *  \param  connId      The connection ID
 *  \param  reportId    The Report ID
 *  \param  len         The length of the report in bytes
 *  \param  pValue      A buffer containing the report
 *
 *  \return none.
 */
/*************************************************************************************************/
void HidSendInputReport(dmConnId_t connId, uint8_t reportId, uint16_t len, uint8_t *pValue);

/*************************************************************************************************/
/*!
 *  \fn     HidSetProtocolMode
 *        
 *  \brief  Sets the HID protocol mode for keyboard and mouse devices that support Boot Mode.
 *
 *  \param  protocolMode    The protocol mode (HID_PROTOCOL_MODE_REPORT or HID_PROTOCOL_MODE_BOOT)
 *
 *  \return None.
 */
/*************************************************************************************************/
void HidSetProtocolMode(uint8_t protocolMode);

/*************************************************************************************************/
/*!
 *  \fn     HidGetProtocolMode
 *        
 *  \brief  Gets the HID protocol mode value.
 *
 *  \param  None.
 *
 *  \return The protocol mode value (HID_PROTOCOL_MODE_REPORT or HID_PROTOCOL_MODE_BOOT).
 */
/*************************************************************************************************/
uint8_t HidGetProtocolMode();

/*************************************************************************************************/
/*!
 *  \fn     HidGetControlPoint
 *        
 *  \brief  Gets the HID control point value.
 *
 *  \param  None.
 *
 *  \return The control point value (HID_CONTROL_POINT_SUSPEND or HID_CONTROL_POINT_RESUME).
 */
/*************************************************************************************************/
uint8_t HidGetControlPoint();

/*************************************************************************************************/
/*!
 *  \fn     HidInit
 *        
 *  \brief  Initialize the HID profile.
 *
 *  \param  pConfig     HID Configuration structure
 *
 *  \return None.
 */
/*************************************************************************************************/
void HidInit(const hidConfig_t *pConfig);

#ifdef __cplusplus
};
#endif

#endif /* HID_API_H */