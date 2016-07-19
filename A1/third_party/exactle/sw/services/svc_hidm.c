/*************************************************************************************************/
/*!
 *  \file   svc_hidm.c
 *        
 *  \brief  Human Interface Device service implementation for Mouse devices.
 *
 *          $Date: 2015-09-30 13:33:59 -0700 (Wed, 30 Sep 2015) $
 *          $Revision: 4057 $
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

#include <string.h>
#include "wsf_types.h"
#include "bstream.h"
#include "att_api.h"
#include "svc_hid.h"
#include "wsf_trace.h"
#include "svc_ch.h"
#include "svc_cfg.h"
#include "svc_batt.h"

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! Characteristic read permissions */
#ifndef HIDM_SEC_PERMIT_READ
#define HIDM_SEC_PERMIT_READ (ATTS_PERMIT_READ | ATTS_PERMIT_READ_ENC)
#endif

/*! Characteristic write permissions */
#ifndef HIDM_SEC_PERMIT_WRITE
#define HIDM_SEC_PERMIT_WRITE (ATTS_PERMIT_WRITE | ATTS_PERMIT_WRITE_ENC)
#endif

/**************************************************************************************************
 Static Variables
**************************************************************************************************/

/* UUIDs */

/**************************************************************************************************
 Service variables
**************************************************************************************************/

/* Common HID Service Attributes */

/* Proprietary Service Declaration */
static const uint8_t hidmValSvc[] = {UINT16_TO_BYTES(ATT_UUID_HID_SERVICE)};
static const uint16_t hidmLenSvc = sizeof(hidmValSvc);


/* HID Info Characteristic */
static const uint8_t hidmInfoCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(HID_INFO_HDL), UINT16_TO_BYTES(ATT_UUID_HID_INFORMATION)};
static const uint16_t hidmLenInfoCh = sizeof(hidmInfoCh);

/* HID Info Value: HID Spec version, country code, flags */
static const uint8_t hidmInfoVal[] = {UINT16_TO_BYTES(HID_VERSION), 0x00, 0x00};
static const uint16_t hidmLenInfoVal = sizeof(hidmInfoVal);


/* HID Report Map Characteristic */
static const uint8_t hidmRmCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(HID_REPORT_MAP_HDL), UINT16_TO_BYTES(ATT_UUID_HID_REPORT_MAP)};
static const uint16_t hidmLenRmCh = sizeof(hidmRmCh);

/* HID Report Map Value */
static uint8_t hidmRmVal[HID_MAX_REPORT_MAP_LEN];
static uint16_t hidmLenRmVal = HID_INIT_REPORT_MAP_LEN;

/* HID External Report Reference Descriptor */
static const uint8_t hidmExtReport[] = {UINT16_TO_BYTES(ATT_UUID_BATTERY_LEVEL)};
static const uint16_t hidmLenExtReport = sizeof(hidmExtReport);


/* HID Control Point Characteristic */
static const uint8_t hidmCpCh[] = {ATT_PROP_WRITE_NO_RSP, UINT16_TO_BYTES(HID_CONTROL_POINT_HDL), UINT16_TO_BYTES(ATT_UUID_HID_CONTROL_POINT)};
static const uint16_t hidmLenCpCh = sizeof(hidmCpCh);

/* HID Control Point Value */
static uint8_t hidmCpVal[] = {0};
static const uint16_t hidmLenCpVal = sizeof(hidmCpVal);

/* HID Service Attributes for Mouse Devices */

/* HID Boot Mouse In Characteristic */
static const uint8_t hidmBmiCh[] = {ATT_PROP_READ | ATT_PROP_NOTIFY, UINT16_TO_BYTES(HIDM_MOUSE_BOOT_IN_HDL), UINT16_TO_BYTES(ATT_UUID_HID_BOOT_MOUSE_IN)};
static const uint16_t hidmLenBmiCh = sizeof(hidmBmiCh);

/* HID Boot Mouse In Value */
static uint8_t hidmBmiVal[HID_MAX_REPORT_LEN];
static uint16_t hidmLenBmiVal = sizeof(hidmBmiVal);

/* HID Boot Mouse In client characteristic configuration */
static uint8_t hidmValBmiChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t hidmLenBmiChCcc = sizeof(hidmValBmiChCcc);


/* HID Input Report Characteristic */
static const uint8_t hidmIRepCh[] = {ATT_PROP_READ | ATT_PROP_NOTIFY, UINT16_TO_BYTES(HIDM_INPUT_REPORT_HDL), UINT16_TO_BYTES(ATT_UUID_HID_REPORT)};
static const uint16_t hidmLenIRepCh = sizeof(hidmIRepCh);

/* HID Input Report Value */
static uint8_t hidmIRepVal[HID_MAX_REPORT_LEN];
static uint16_t hidmLenIRepVal = sizeof(hidmIRepVal);

/* HID Input Report client characteristic configuration */
static uint8_t hidmValIRepChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t hidmLenIRepChCcc = sizeof(hidmValIRepChCcc);

/* HID Input Report Reference - ID, Type */
static const uint8_t hidmValIRepIdMap[] = {0x00, HID_REPORT_TYPE_INPUT};
static const uint16_t hidmLenIRepIdMap = sizeof(hidmValIRepIdMap);


/* HID Protocol Mode Characteristic */
static const uint8_t hidmPmCh[] = {ATT_PROP_READ | ATT_PROP_WRITE_NO_RSP, UINT16_TO_BYTES(HIDM_PROTOCOL_MODE_HDL), UINT16_TO_BYTES(ATT_UUID_HID_PROTOCOL_MODE)};
static const uint16_t hidmLenPmCh = sizeof(hidmPmCh);

/* HID Protocol Mode Value */
static uint8_t hidmPmVal[] = {HID_PROTOCOL_MODE_REPORT};
static const uint16_t hidmLenPmVal = sizeof(hidmPmVal);


/* Attribute list for HID group */
static const attsAttr_t hidmList[] =
{
  /* Service Delcaration */
  {
    attPrimSvcUuid, 
    (uint8_t *) hidmValSvc,
    (uint16_t *) &hidmLenSvc, 
    sizeof(hidmValSvc),
    0,
    HIDM_SEC_PERMIT_READ
  },

  {
    attChUuid,
    (uint8_t *) hidmInfoCh,
    (uint16_t *) &hidmLenInfoCh,
    sizeof(hidmInfoCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidiChUuid,
    (uint8_t *) hidmInfoVal,
    (uint16_t *) &hidmLenInfoVal,
    sizeof(hidmInfoVal),
    0,
    HIDM_SEC_PERMIT_READ
  },

  {
    attChUuid,
    (uint8_t *) hidmRmCh,
    (uint16_t *) &hidmLenRmCh,
    sizeof(hidmRmCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidRmChUuid,
    (uint8_t *) hidmRmVal,
    (uint16_t *) &hidmLenRmVal,
    HID_MAX_REPORT_MAP_LEN,
    ATTS_SET_VARIABLE_LEN,
    HIDM_SEC_PERMIT_READ
  },
  {
    attHidErmUuid,
    (uint8_t *) hidmExtReport,
    (uint16_t *) &hidmLenExtReport,
    sizeof(hidmExtReport),
    0,
    HIDM_SEC_PERMIT_READ
  },

  {
    attChUuid,
    (uint8_t *) hidmCpCh,
    (uint16_t *) &hidmLenCpCh,
    sizeof(hidmCpCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidRepChUuid,
    (uint8_t *) hidmCpVal,
    (uint16_t *) &hidmLenCpVal,
    sizeof(hidmCpVal),
    ATTS_SET_WRITE_CBACK,
    HIDM_SEC_PERMIT_WRITE
  },

  {
    attChUuid,
    (uint8_t *) hidmBmiCh,
    (uint16_t *) &hidmLenBmiCh,
    sizeof(hidmBmiCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidBmiChUuid,
    (uint8_t *) hidmBmiVal,
    (uint16_t *) &hidmLenBmiVal,
    sizeof(hidmBmiVal),
    ATTS_SET_VARIABLE_LEN,
    HIDM_SEC_PERMIT_READ
  },
  {
    attCliChCfgUuid,
    (uint8_t *) hidmValBmiChCcc,
    (uint16_t *) &hidmLenBmiChCcc,
    sizeof(hidmValBmiChCcc),
    ATTS_SET_CCC,
    (HIDM_SEC_PERMIT_READ | HIDM_SEC_PERMIT_WRITE)
  },

  {
    attChUuid,
    (uint8_t *) hidmIRepCh,
    (uint16_t *) &hidmLenIRepCh,
    sizeof(hidmIRepCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidRepChUuid,
    (uint8_t *) hidmIRepVal,
    (uint16_t *) &hidmLenIRepVal,
    sizeof(hidmIRepVal),
    ATTS_SET_VARIABLE_LEN,
    HIDM_SEC_PERMIT_READ
  },
  {
    attCliChCfgUuid,
    (uint8_t *) hidmValIRepChCcc,
    (uint16_t *) &hidmLenIRepChCcc,
    sizeof(hidmValIRepChCcc),
    ATTS_SET_CCC,
    (HIDM_SEC_PERMIT_READ | HIDM_SEC_PERMIT_WRITE)
  },
  {
    attHidRimUuid,
    (uint8_t *) hidmValIRepIdMap,
    (uint16_t *) &hidmLenIRepIdMap,
    sizeof(hidmValIRepIdMap),
    0,
    HIDM_SEC_PERMIT_READ
  },

  {
    attChUuid,
    (uint8_t *) hidmPmCh,
    (uint16_t *) &hidmLenPmCh,
    sizeof(hidmPmCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidRepChUuid,
    (uint8_t *) hidmPmVal,
    (uint16_t *) &hidmLenPmVal,
    sizeof(hidmPmVal),
    ATTS_SET_WRITE_CBACK,
    (HIDM_SEC_PERMIT_READ | HIDM_SEC_PERMIT_WRITE)
  }
};

/* HID group structure */
static attsGroup_t svcHidmGroup =
{
  NULL,
  (attsAttr_t *) hidmList,
  NULL,
  NULL,
  HID_START_HDL,
  HIDM_END_HDL
};

/*************************************************************************************************/
/*!
 *  \fn     SvcHidMouseAddGroup
 *        
 *  \brief  Add the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcHidMouseAddGroup(void)
{
  memset(hidmRmVal, 0, sizeof(hidmRmVal));
  AttsAddGroup(&svcHidmGroup);
}

/*************************************************************************************************/
/*!
 *  \fn     SvcHidMouseRemoveGroup
 *        
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcHidMouseRemoveGroup(void)
{
  AttsRemoveGroup(HID_START_HDL);
}

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
void SvcHidMouseRegister(attsWriteCback_t writeCb, attsReadCback_t readCb)
{
  svcHidmGroup.writeCback = writeCb;
  svcHidmGroup.readCback = readCb;
}