/*************************************************************************************************/
/*!
 *  \file   svc_hid.h
 *        
 *  \brief  Human Interface Device service implementation.
 *
 *          $Date: 2015-09-24 15:22:42 -0700 (Thu, 24 Sep 2015) $
 *          $Revision: 4017 $
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

#ifndef SVC_HID_H
#define SVC_HID_H

#ifdef __cplusplus
extern "C" {
#endif

/**************************************************************************************************
Macros
**************************************************************************************************/

/* HID Service */
#define HID_SVC_UUID                  ATT_UUID_HID_SERVICE

/* HID Spec Version: 1.11 */
#define HID_VERSION                   0x0111

/* HID Report Types */
#define HID_REPORT_TYPE_INPUT         0x01
#define HID_REPORT_TYPE_OUTPUT        0x02
#define HID_REPORT_TYPE_FEATURE       0x03

/* HID Boot Report ID */
#define HID_BOOT_ID                   0xFF

/* HID Protocol Mode Types */
#define HID_PROTOCOL_MODE_BOOT        0x00
#define HID_PROTOCOL_MODE_REPORT      0x01

/* HID Control Point Values */
#define HID_CONTROL_POINT_SUSPEND     0x00
#define HID_CONTROL_POINT_RESUME      0x01

/* Max length of the report map value */
#define HID_MAX_REPORT_MAP_LEN        512

/* Max length of an output report value */
#define HID_MAX_REPORT_LEN            32

/* Initial length of the report map value */
#define HID_INIT_REPORT_MAP_LEN       1

/**************************************************************************************************
 Handle Ranges
**************************************************************************************************/

/* Proprietary Service */
#define HID_START_HDL                 0x100
#define HIDKB_END_HDL                 (HIDKB_MAX_HDL - 1)
#define HIDM_END_HDL                  (HIDM_MAX_HDL - 1)
#define HIDG_END_HDL                  (HIDG_MAX_HDL - 1)

/**************************************************************************************************
 Handles
**************************************************************************************************/
/* Proprietary Service Handles Common to HID Devices */
enum
{
  HID_SVC_HDL = HID_START_HDL,        /* Proprietary Service Declaration */
  HID_INFO_CH_HDL,                    /* HID Information Characteristic Declaration */ 
  HID_INFO_HDL,                       /* HID Information Value */
  HID_REPORT_MAP_CH_HDL,              /* HID Report Map Characteristic Declaration */ 
  HID_REPORT_MAP_HDL,                 /* HID Report Map Value */ 
  HID_EXTERNAL_REPORT_HDL,            /* HID External Report Descriptor */ 
  HID_CONTROL_POINT_CH_HDL,           /* HID Control Point Characteristic Declaration */ 
  HID_CONTROL_POINT_HDL,              /* HID Control Point Value */ 
  HID_COMMON_MAX_HDL
};

/* Proprietary Service Handles for a HID Keyboard Device */
enum
{
  HIDKB_KEYBOARD_BOOT_IN_CH_HDL = HID_COMMON_MAX_HDL,        /* HID Keyboard Boot Input Characteristic Declaration */ 
  HIDKB_KEYBOARD_BOOT_IN_HDL,           /* HID Keyboard Boot Input Value */
  HIDKB_KEYBOARD_BOOT_IN_CH_CCC_HDL,    /* HID Keyboard Boot Input CCC Descriptor */
  HIDKB_KEYBOARD_BOOT_OUT_CH_HDL,       /* HID Keyboard Boot Output Characteristic Declaration */ 
  HIDKB_KEYBOARD_BOOT_OUT_HDL,          /* HID Keyboard Boot Output Value */
  HIDKB_INPUT_REPORT_CH_HDL,            /* HID Keyboard Input Report Characteristic Declaration */ 
  HIDKB_INPUT_REPORT_HDL,               /* HID Keyboard Input Report Value */ 
  HIDKB_INPUT_REPORT_CH_CCC_HDL,        /* HID Keyboard Input Report CCC Descriptor */
  HIDKB_INPUT_REPORT_REFERENCE_HDL,     /* HID Keyboard Input Report Reference Descriptor */ 
  HIDKB_OUTPUT_REPORT_CH_HDL,           /* HID Keyboard Output Report Characteristic Declaration */ 
  HIDKB_OUTPUT_REPORT_HDL,              /* HID Keyboard Output Report Value */ 
  HIDKB_OUTPUT_REPORT_REFERENCE_HDL,    /* HID Keyboard Output Report Reference Descriptor */ 
  HIDKB_FEATURE_REPORT_CH_HDL,          /* HID Keyboard Feature Report Characteristic Declaration */ 
  HIDKB_FEATURE_REPORT_HDL,             /* HID Keyboard Feature Report Value */ 
  HIDKB_FEATURE_REPORT_REFERENCE_HDL,   /* HID Keyboard Feature Report Reference Descriptor */ 
  HIDKB_PROTOCOL_MODE_CH_HDL,           /* HID Keyboard Protocol Mode Characteristic Declaration */ 
  HIDKB_PROTOCOL_MODE_HDL,              /* HID Keyboard Protocol Mode Value */ 
  HIDKB_MAX_HDL
};

/* Proprietary Service Handles for a HID Mouse Device */
enum
{
  HIDM_MOUSE_BOOT_IN_CH_HDL = HID_COMMON_MAX_HDL,           /* HID Mouse Boot Input Characteristic Declaration */ 
  HIDM_MOUSE_BOOT_IN_HDL,              /* HID Mouse Boot Input Value */
  HIDM_MOUSE_BOOT_IN_CH_CCC_HDL,       /* HID Mouse Boot Input CCC Descriptor */
  HIDM_INPUT_REPORT_CH_HDL,            /* HID Mouse Input Report Characteristic Declaration */ 
  HIDM_INPUT_REPORT_HDL,               /* HID Mouse Input Report Value */ 
  HIDM_INPUT_REPORT_CH_CCC_HDL,        /* HID Mouse Input Report CCC Descriptor */
  HIDM_INPUT_REPORT_REFERENCE_HDL,     /* HID Mouse Input Report Reference Descriptor */ 
  HIDM_PROTOCOL_MODE_CH_HDL,           /* HID Mouse Protocol Mode Characteristic Declaration */ 
  HIDM_PROTOCOL_MODE_HDL,              /* HID Mouse Protocol Mode Value */ 
  HIDM_MAX_HDL
};

/* Proprietary Service Handles For a Generic HID Device */
enum
{
  HIDG_INPUT_REPORT_CH_HDL = HID_COMMON_MAX_HDL,            /* HID Generic Input Report Characteristic Declaration */ 
  HIDG_INPUT_REPORT_HDL,               /* HID Generic Input Report Value */ 
  HIDG_INPUT_REPORT_CH_CCC_HDL,        /* HID Generic Input Report CCC Descriptor */
  HIDG_INPUT_REPORT_REFERENCE_HDL,     /* HID Generic Input Report Reference Descriptor */ 
  HIDG_OUTPUT_REPORT_CH_HDL,           /* HID Generic Output Report Characteristic Declaration */ 
  HIDG_OUTPUT_REPORT_HDL,              /* HID Generic Output Report Value */ 
  HIDG_OUTPUT_REPORT_REFERENCE_HDL,    /* HID Generic Output Report Reference Descriptor */ 
  HIDG_FEATURE_REPORT_CH_HDL,          /* HID Generic Feature Report Characteristic Declaration */ 
  HIDG_FEATURE_REPORT_HDL,             /* HID Generic Feature Report Value */ 
  HIDG_FEATURE_REPORT_REFERENCE_HDL,   /* HID Generic Feature Report Reference Descriptor */ 
  HIDG_MAX_HDL
};

/*************************************************************************************************/
/*!
 *  \fn     SvcHidMouseAddGroup
 *        
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcHidMouseAddGroup(void);

/*************************************************************************************************/
/*!
 *  \fn     SvcHidMouseRemoveGroup
 *        
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcHidMouseRemoveGroup(void);

/*************************************************************************************************/
/*!
 *  \fn     SvcHidMouseRegister
 *        
 *  \brief  Register a read and write callback functions for the ATT Group.
 *
 *  \param  writeCb   Write callback function
 *  \param  readCb    Read callback function
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcHidMouseRegister(attsWriteCback_t writeCb, attsReadCback_t readCb);

/*************************************************************************************************/
/*!
 *  \fn     SvcHidKeyboardAddGroup
 *        
 *  \brief  Add the services to the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcHidKeyboardAddGroup(void);

/*************************************************************************************************/
/*!
 *  \fn     SvcHidKeyboardRemoveGroup
 *        
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcHidKeyboardRemoveGroup(void);

/*************************************************************************************************/
/*!
 *  \fn     SvcHidKeyboardRegister
 *        
 *  \brief  Register a read and write callback functions for the ATT Group.
 *
 *  \param  writeCb   Write callback function
 *  \param  readCb    Read callback function
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcHidKeyboardRegister(attsWriteCback_t writeCb, attsReadCback_t readCb);

/*************************************************************************************************/
/*!
 *  \fn     SvcHidGenericAddGroup
 *        
 *  \brief  Add the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcHidGenericAddGroup(void);

/*************************************************************************************************/
/*!
 *  \fn     SvcHidGenericRemoveGroup
 *        
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcHidGenericRemoveGroup(void);

/*************************************************************************************************/
/*!
 *  \fn     SvcHidGenericRegister
 *        
 *  \brief  Register a read and write callback functions for the ATT Group.
 *
 *  \param  writeCb   Write callback function
 *  \param  readCb    Read callback function
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcHidGenericRegister(attsWriteCback_t writeCb, attsReadCback_t readCb);

#ifdef __cplusplus
}
#endif
 
#endif /* SVC_HID_H */