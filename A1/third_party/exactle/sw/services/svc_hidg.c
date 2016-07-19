/*************************************************************************************************/
/*!
 *  \file   svc_hidg.c
 *        
 *  \brief  Human Interface Device service implementation for Generic devices.
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
#ifndef HIDG_SEC_PERMIT_READ
#define HIDG_SEC_PERMIT_READ (ATTS_PERMIT_READ | ATTS_PERMIT_READ_ENC)
#endif

/*! Characteristic write permissions */
#ifndef HIDG_SEC_PERMIT_WRITE
#define HIDG_SEC_PERMIT_WRITE (ATTS_PERMIT_WRITE | ATTS_PERMIT_WRITE_ENC)
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
static const uint8_t hidgValSvc[] = {UINT16_TO_BYTES(ATT_UUID_HID_SERVICE)};
static const uint16_t hidgLenSvc = sizeof(hidgValSvc);


/* HID Info Characteristic */
static const uint8_t hidgInfoCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(HID_INFO_HDL), UINT16_TO_BYTES(ATT_UUID_HID_INFORMATION)};
static const uint16_t hidgLenInfoCh = sizeof(hidgInfoCh);

/* HID Info Value: HID Spec version, country code, flags */
static const uint8_t hidgInfoVal[] = {UINT16_TO_BYTES(HID_VERSION), 0x00, 0x00};
static const uint16_t hidgLenInfoVal = sizeof(hidgInfoVal);


/* HID Report Map Characteristic */
static const uint8_t hidgRmCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(HID_REPORT_MAP_HDL), UINT16_TO_BYTES(ATT_UUID_HID_REPORT_MAP)};
static const uint16_t hidgLenRmCh = sizeof(hidgRmCh);

/* HID Report Map Value */
static uint8_t hidgRmVal[HID_MAX_REPORT_MAP_LEN];
static uint16_t hidgLenRmVal = HID_INIT_REPORT_MAP_LEN;

/* HID External Report Reference Descriptor */
static const uint8_t hidgExtReport[] = {UINT16_TO_BYTES(ATT_UUID_BATTERY_LEVEL)};
static const uint16_t hidgLenExtReport = sizeof(hidgExtReport);


/* HID Control Point Characteristic */
static const uint8_t hidgCpCh[] = {ATT_PROP_WRITE_NO_RSP, UINT16_TO_BYTES(HID_CONTROL_POINT_HDL), UINT16_TO_BYTES(ATT_UUID_HID_CONTROL_POINT)};
static const uint16_t hidgLenCpCh = sizeof(hidgCpCh);

/* HID Control Point Value */
static const uint8_t hidgCpVal[] = {0};
static const uint16_t hidgLenCpVal = sizeof(hidgCpVal);


/* HID Service Attributes for Generic Devices */

/* HID Input Report Characteristic */
static const uint8_t hidgIRepCh[] = {ATT_PROP_READ | ATT_PROP_NOTIFY, UINT16_TO_BYTES(HIDG_INPUT_REPORT_HDL), UINT16_TO_BYTES(ATT_UUID_HID_REPORT)};
static const uint16_t hidgLenIRepCh = sizeof(hidgIRepCh);

/* HID Input Report Value */
static uint8_t hidgIRepVal[HID_MAX_REPORT_LEN];
static uint16_t hidgLenIRepVal = sizeof(hidgIRepVal);

/* HID Input Report client characteristic configuration */
static uint8_t hidgValIRepChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t hidgLenIRepChCcc = sizeof(hidgValIRepChCcc);

/* HID Input Report Reference - ID, Type */
static const uint8_t hidgValIRepIdMap[] = {0x00, HID_REPORT_TYPE_INPUT};
static const uint16_t hidgLenIRepIdMap = sizeof(hidgValIRepIdMap);


/* HID Output Report Characteristic */
static const uint8_t hidgORepCh[] = {ATT_PROP_READ | ATT_PROP_WRITE | ATT_PROP_WRITE_NO_RSP, UINT16_TO_BYTES(HIDG_OUTPUT_REPORT_HDL), UINT16_TO_BYTES(ATT_UUID_HID_REPORT)};
static const uint16_t hidgLenORepCh = sizeof(hidgORepCh);

/* HID Output Report Value */
static uint8_t hidgORepVal[HID_MAX_REPORT_LEN];
static uint16_t hidgLenORepVal = sizeof(hidgORepVal);

/* HID Output Report Reference - ID, Type */
static const uint8_t hidgValORepIdMap[] = {0x00, HID_REPORT_TYPE_OUTPUT};
static const uint16_t hidgLenORepIdMap = sizeof(hidgValORepIdMap);


/* HID Feature Report Characteristic */
static const uint8_t hidgFRepCh[] = {ATT_PROP_READ | ATT_PROP_WRITE, UINT16_TO_BYTES(HIDG_FEATURE_REPORT_HDL), UINT16_TO_BYTES(ATT_UUID_HID_REPORT)};
static const uint16_t hidgLenFRepCh = sizeof(hidgFRepCh);

/* HID Feature Report Value */
static uint8_t hidgFRepVal[HID_MAX_REPORT_LEN];
static uint16_t hidgLenFRepVal = sizeof(hidgFRepVal);

/* HID Feature Report Reference - ID, Type */
static const uint8_t hidgValFRepIdMap[] = {0x00, HID_REPORT_TYPE_FEATURE};
static const uint16_t hidgLenFRepIdMap = sizeof(hidgValFRepIdMap);


/* Attribute list for HID group */
static const attsAttr_t hidgList[] =
{
  /* Service Delcaration */
  {
    attPrimSvcUuid, 
    (uint8_t *) hidgValSvc,
    (uint16_t *) &hidgLenSvc, 
    sizeof(hidgValSvc),
    0,
    HIDG_SEC_PERMIT_READ
  },

  {
    attChUuid,
    (uint8_t *) hidgInfoCh,
    (uint16_t *) &hidgLenInfoCh,
    sizeof(hidgInfoCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidiChUuid,
    (uint8_t *) hidgInfoVal,
    (uint16_t *) &hidgLenInfoVal,
    sizeof(hidgInfoVal),
    0,
    HIDG_SEC_PERMIT_READ
  },

  {
    attChUuid,
    (uint8_t *) hidgRmCh,
    (uint16_t *) &hidgLenRmCh,
    sizeof(hidgRmCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidRmChUuid,
    (uint8_t *) hidgRmVal,
    (uint16_t *) &hidgLenRmVal,
    HID_MAX_REPORT_MAP_LEN,
    ATTS_SET_VARIABLE_LEN,
    HIDG_SEC_PERMIT_READ
  },
  {
    attHidErmUuid,
    (uint8_t *) hidgExtReport,
    (uint16_t *) &hidgLenExtReport,
    sizeof(hidgExtReport),
    0,
    HIDG_SEC_PERMIT_READ
  },

  {
    attChUuid,
    (uint8_t *) hidgCpCh,
    (uint16_t *) &hidgLenCpCh,
    sizeof(hidgCpCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidRepChUuid,
    (uint8_t *) hidgCpVal,
    (uint16_t *) &hidgLenCpVal,
    sizeof(hidgCpVal),
    ATTS_SET_WRITE_CBACK,
    HIDG_SEC_PERMIT_WRITE
  },

  {
    attChUuid,
    (uint8_t *) hidgIRepCh,
    (uint16_t *) &hidgLenIRepCh,
    sizeof(hidgIRepCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidRepChUuid,
    (uint8_t *) hidgIRepVal,
    (uint16_t *) &hidgLenIRepVal,
    sizeof(hidgIRepVal),
    ATTS_SET_VARIABLE_LEN,
    HIDG_SEC_PERMIT_READ
  },
  {
    attCliChCfgUuid,
    (uint8_t *) hidgValIRepChCcc,
    (uint16_t *) &hidgLenIRepChCcc,
    sizeof(hidgValIRepChCcc),
    ATTS_SET_CCC,
    (HIDG_SEC_PERMIT_READ | HIDG_SEC_PERMIT_WRITE)
  },
  {
    attHidRimUuid,
    (uint8_t *) hidgValIRepIdMap,
    (uint16_t *) &hidgLenIRepIdMap,
    sizeof(hidgValIRepIdMap),
    0,
    HIDG_SEC_PERMIT_READ
  },

  {
    attChUuid,
    (uint8_t *) hidgORepCh,
    (uint16_t *) &hidgLenORepCh,
    sizeof(hidgORepCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidRepChUuid,
    (uint8_t *) hidgORepVal,
    (uint16_t *) &hidgLenORepVal,
    sizeof(hidgORepVal),
    (ATTS_SET_WRITE_CBACK | ATTS_SET_VARIABLE_LEN),
    (HIDG_SEC_PERMIT_READ | HIDG_SEC_PERMIT_WRITE)
  },
  {
    attHidRimUuid,
    (uint8_t *) hidgValORepIdMap,
    (uint16_t *) &hidgLenORepIdMap,
    sizeof(hidgValORepIdMap),
    0,
    HIDG_SEC_PERMIT_READ
  },

  {
    attChUuid,
    (uint8_t *) hidgFRepCh,
    (uint16_t *) &hidgLenFRepCh,
    sizeof(hidgFRepCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidRepChUuid,
    (uint8_t *) hidgFRepVal,
    (uint16_t *) &hidgLenFRepVal,
    sizeof(hidgFRepVal),
    (ATTS_SET_WRITE_CBACK | ATTS_SET_VARIABLE_LEN),
    (HIDG_SEC_PERMIT_READ | HIDG_SEC_PERMIT_WRITE)
  },
  {
    attHidRimUuid,
    (uint8_t *) hidgValFRepIdMap,
    (uint16_t *) &hidgLenFRepIdMap,
    sizeof(hidgValFRepIdMap),
    0,
    HIDG_SEC_PERMIT_READ
  }
};

/* HID group structure */
static attsGroup_t svcHidgGroup =
{
  NULL,
  (attsAttr_t *) hidgList,
  NULL,
  NULL,
  HID_START_HDL,
  HIDG_END_HDL
};

/*************************************************************************************************/
/*!
 *  \fn     SvcHidGenericAddGroup
 *        
 *  \brief  Add the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcHidGenericAddGroup(void)
{
  memset(hidgRmVal, 0, sizeof(hidgRmVal));
  AttsAddGroup(&svcHidgGroup);
}

/*************************************************************************************************/
/*!
 *  \fn     SvcHidGenericRemoveGroup
 *        
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcHidGenericRemoveGroup(void)
{
  AttsRemoveGroup(HID_START_HDL);
}

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
void SvcHidGenericRegister(attsWriteCback_t writeCb, attsReadCback_t readCb)
{
  svcHidgGroup.writeCback = writeCb;
  svcHidgGroup.readCback = readCb;
}