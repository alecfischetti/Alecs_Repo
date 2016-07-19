/*************************************************************************************************/
/*!
 *  \file   svc_hidkbkb.c
 *        
 *  \brief  Human Interface Device service implementation for Keyboard devices.
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
#ifndef HIDKB_SEC_PERMIT_READ
#define HIDKB_SEC_PERMIT_READ (ATTS_PERMIT_READ | ATTS_PERMIT_READ_ENC)
#endif

/*! Characteristic write permissions */
#ifndef HIDKB_SEC_PERMIT_WRITE
#define HIDKB_SEC_PERMIT_WRITE (ATTS_PERMIT_WRITE | ATTS_PERMIT_WRITE_ENC)
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
static const uint8_t hidkbValSvc[] = {UINT16_TO_BYTES(ATT_UUID_HID_SERVICE)};
static const uint16_t hidkbLenSvc = sizeof(hidkbValSvc);


/* HID Info Characteristic */
static const uint8_t hidkbInfoCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(HID_INFO_HDL), UINT16_TO_BYTES(ATT_UUID_HID_INFORMATION)};
static const uint16_t hidkbLenInfoCh = sizeof(hidkbInfoCh);

/* HID Info Value: HID Spec version, country code, flags */
static const uint8_t hidkbInfoVal[] = {UINT16_TO_BYTES(HID_VERSION), 0x00, 0x00};
static const uint16_t hidkbLenInfoVal = sizeof(hidkbInfoVal);


/* HID Report Map Characteristic */
static const uint8_t hidkbRmCh[] = {ATT_PROP_READ, UINT16_TO_BYTES(HID_REPORT_MAP_HDL), UINT16_TO_BYTES(ATT_UUID_HID_REPORT_MAP)};
static const uint16_t hidkbLenRmCh = sizeof(hidkbRmCh);

/* HID Report Map Value */
static uint8_t hidkbRmVal[HID_MAX_REPORT_MAP_LEN];
static uint16_t hidkbLenRmVal = HID_INIT_REPORT_MAP_LEN;

/* HID External Report Reference Descriptor */
static const uint8_t hidkbExtReport[] = {UINT16_TO_BYTES(ATT_UUID_BATTERY_LEVEL)};
static const uint16_t hidkbLenExtReport = sizeof(hidkbExtReport);


/* HID Control Point Characteristic */
static const uint8_t hidkbCpCh[] = {ATT_PROP_WRITE_NO_RSP, UINT16_TO_BYTES(HID_CONTROL_POINT_HDL), UINT16_TO_BYTES(ATT_UUID_HID_CONTROL_POINT)};
static const uint16_t hidkbLenCpCh = sizeof(hidkbCpCh);

/* HID Control Point Value */
static uint8_t hidkbCpVal[] = {0};
static const uint16_t hidkbLenCpVal = sizeof(hidkbCpVal);


/* HID Service Attributes for Keyboard Devices */

/* HID Boot Keyboard In Characteristic */
static const uint8_t hidkbBkiCh[] = {ATT_PROP_READ | ATT_PROP_NOTIFY, UINT16_TO_BYTES(HIDKB_KEYBOARD_BOOT_IN_HDL), UINT16_TO_BYTES(ATT_UUID_HID_BOOT_KEYBOARD_IN)};
static const uint16_t hidkbLenBkiCh = sizeof(hidkbBkiCh);

/* HID Boot Keyboard In Value */
static uint8_t hidkbBkiVal[HID_MAX_REPORT_LEN];
static uint16_t hidkbLenBkiVal = sizeof(hidkbBkiVal);

/* HID Boot Keyboard In client characteristic configuration */
static uint8_t hidkbValBkiChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t hidkbLenBkiChCcc = sizeof(hidkbValBkiChCcc);


/* HID Boot Keyboard Out Characteristic */
static const uint8_t hidkbBkoCh[] = {ATT_PROP_READ | ATT_PROP_WRITE | ATT_PROP_WRITE_NO_RSP, UINT16_TO_BYTES(HIDKB_KEYBOARD_BOOT_OUT_HDL), UINT16_TO_BYTES(ATT_UUID_HID_BOOT_KEYBOARD_OUT)};
static const uint16_t hidkbLenBkoCh = sizeof(hidkbBkoCh);

/* HID Boot Keyboard Out Value */
static uint8_t hidkbBkoVal[HID_MAX_REPORT_LEN];
static uint16_t hidkbLenBkoVal = sizeof(hidkbBkoVal);


/* HID Input Report Characteristic */
static const uint8_t hidkbIRepCh[] = {ATT_PROP_READ | ATT_PROP_NOTIFY, UINT16_TO_BYTES(HIDKB_INPUT_REPORT_HDL), UINT16_TO_BYTES(ATT_UUID_HID_REPORT)};
static const uint16_t hidkbLenIRepCh = sizeof(hidkbIRepCh);

/* HID Input Report Value */
static uint8_t hidkbIRepVal[HID_MAX_REPORT_LEN];
static uint16_t hidkbLenIRepVal = sizeof(hidkbIRepVal);

/* HID Input Report client characteristic configuration */
static uint8_t hidkbValIRepChCcc[] = {UINT16_TO_BYTES(0x0000)};
static const uint16_t hidkbLenIRepChCcc = sizeof(hidkbValIRepChCcc);

/* HID Input Report Reference - ID, Type */
static const uint8_t hidkbValIRepIdMap[] = {0x00, HID_REPORT_TYPE_INPUT};
static const uint16_t hidkbLenIRepIdMap = sizeof(hidkbValIRepIdMap);


/* HID Output Report Characteristic */
static const uint8_t hidkbORepCh[] = {ATT_PROP_READ | ATT_PROP_WRITE | ATT_PROP_WRITE_NO_RSP, UINT16_TO_BYTES(HIDKB_OUTPUT_REPORT_HDL), UINT16_TO_BYTES(ATT_UUID_HID_REPORT)};
static const uint16_t hidkbLenORepCh = sizeof(hidkbORepCh);

/* HID Output Report Value */
static uint8_t hidkbORepVal[HID_MAX_REPORT_LEN];
static uint16_t hidkbLenORepVal = sizeof(hidkbORepVal);

/* HID Output Report Reference - ID, Type */
static const uint8_t hidkbValORepIdMap[] = {0x00, HID_REPORT_TYPE_OUTPUT};
static const uint16_t hidkbLenORepIdMap = sizeof(hidkbValORepIdMap);


/* HID Feature Report Characteristic */
static const uint8_t hidkbFRepCh[] = {ATT_PROP_READ | ATT_PROP_WRITE, UINT16_TO_BYTES(HIDKB_FEATURE_REPORT_HDL), UINT16_TO_BYTES(ATT_UUID_HID_REPORT)};
static const uint16_t hidkbLenFRepCh = sizeof(hidkbFRepCh);

/* HID Feature Report Value */
static uint8_t hidkbFRepVal[HID_MAX_REPORT_LEN];
static uint16_t hidkbLenFRepVal = sizeof(hidkbFRepVal);

/* HID Feature Report Reference - ID, Type */
static const uint8_t hidkbValFRepIdMap[] = {0x00, HID_REPORT_TYPE_FEATURE};
static const uint16_t hidkbLenFRepIdMap = sizeof(hidkbValFRepIdMap);


/* HID Protocol Mode Characteristic */
static const uint8_t hidkbPmCh[] = {ATT_PROP_READ | ATT_PROP_WRITE_NO_RSP, UINT16_TO_BYTES(HIDKB_PROTOCOL_MODE_HDL), UINT16_TO_BYTES(ATT_UUID_HID_PROTOCOL_MODE)};
static const uint16_t hidkbLenPmCh = sizeof(hidkbPmCh);

/* HID Protocol Mode Value */
static uint8_t hidkbPmVal[] = {HID_PROTOCOL_MODE_REPORT};
static const uint16_t hidkbLenPmVal = sizeof(hidkbPmVal);


/* Attribute list for HID group */
static const attsAttr_t hidkbList[] =
{
  /* Service Delcaration */
  {
    attPrimSvcUuid, 
    (uint8_t *) hidkbValSvc,
    (uint16_t *) &hidkbLenSvc, 
    sizeof(hidkbValSvc),
    0,
    HIDKB_SEC_PERMIT_READ
  },

  {
    attChUuid,
    (uint8_t *) hidkbInfoCh,
    (uint16_t *) &hidkbLenInfoCh,
    sizeof(hidkbInfoCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidiChUuid,
    (uint8_t *) hidkbInfoVal,
    (uint16_t *) &hidkbLenInfoVal,
    sizeof(hidkbInfoVal),
    0,
    HIDKB_SEC_PERMIT_READ
  },

  {
    attChUuid,
    (uint8_t *) hidkbRmCh,
    (uint16_t *) &hidkbLenRmCh,
    sizeof(hidkbRmCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidRmChUuid,
    (uint8_t *) hidkbRmVal,
    (uint16_t *) &hidkbLenRmVal,
    HID_MAX_REPORT_MAP_LEN,
    ATTS_SET_VARIABLE_LEN,
    HIDKB_SEC_PERMIT_READ
  },
  {
    attHidErmUuid,
    (uint8_t *) hidkbExtReport,
    (uint16_t *) &hidkbLenExtReport,
    sizeof(hidkbExtReport),
    0,
    HIDKB_SEC_PERMIT_READ
  },

  {
    attChUuid,
    (uint8_t *) hidkbCpCh,
    (uint16_t *) &hidkbLenCpCh,
    sizeof(hidkbCpCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidRepChUuid,
    (uint8_t *) hidkbCpVal,
    (uint16_t *) &hidkbLenCpVal,
    sizeof(hidkbCpVal),
    ATTS_SET_WRITE_CBACK,
    HIDKB_SEC_PERMIT_WRITE
  },

  {
    attChUuid,
    (uint8_t *) hidkbBkiCh,
    (uint16_t *) &hidkbLenBkiCh,
    sizeof(hidkbBkiCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidBkiChUuid,
    (uint8_t *) hidkbBkiVal,
    (uint16_t *) &hidkbLenBkiVal,
    sizeof(hidkbBkiVal),
    ATTS_SET_VARIABLE_LEN,
    HIDKB_SEC_PERMIT_READ
  },
  {
    attCliChCfgUuid,
    (uint8_t *) hidkbValBkiChCcc,
    (uint16_t *) &hidkbLenBkiChCcc,
    sizeof(hidkbValBkiChCcc),
    ATTS_SET_CCC,
    (HIDKB_SEC_PERMIT_READ | HIDKB_SEC_PERMIT_WRITE)
  },

  {
    attChUuid,
    (uint8_t *) hidkbBkoCh,
    (uint16_t *) &hidkbLenBkoCh,
    sizeof(hidkbBkoCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidBkoChUuid,
    (uint8_t *) hidkbBkoVal,
    (uint16_t *) &hidkbLenBkoVal,
    sizeof(hidkbBkoVal),
    (ATTS_SET_WRITE_CBACK | ATTS_SET_VARIABLE_LEN),
    (HIDKB_SEC_PERMIT_READ | HIDKB_SEC_PERMIT_WRITE)
  },

  {
    attChUuid,
    (uint8_t *) hidkbIRepCh,
    (uint16_t *) &hidkbLenIRepCh,
    sizeof(hidkbIRepCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidRepChUuid,
    (uint8_t *) hidkbIRepVal,
    (uint16_t *) &hidkbLenIRepVal,
    sizeof(hidkbIRepVal),
    ATTS_SET_VARIABLE_LEN,
    HIDKB_SEC_PERMIT_READ
  },
  {
    attCliChCfgUuid,
    (uint8_t *) hidkbValIRepChCcc,
    (uint16_t *) &hidkbLenIRepChCcc,
    sizeof(hidkbValIRepChCcc),
    ATTS_SET_CCC,
    (HIDKB_SEC_PERMIT_READ | HIDKB_SEC_PERMIT_WRITE)
  },
  {
    attHidRimUuid,
    (uint8_t *) hidkbValIRepIdMap,
    (uint16_t *) &hidkbLenIRepIdMap,
    sizeof(hidkbValIRepIdMap),
    0,
    HIDKB_SEC_PERMIT_READ
  },

  {
    attChUuid,
    (uint8_t *) hidkbORepCh,
    (uint16_t *) &hidkbLenORepCh,
    sizeof(hidkbORepCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidRepChUuid,
    (uint8_t *) hidkbORepVal,
    (uint16_t *) &hidkbLenORepVal,
    sizeof(hidkbORepVal),
    (ATTS_SET_WRITE_CBACK | ATTS_SET_VARIABLE_LEN),
    (HIDKB_SEC_PERMIT_READ | HIDKB_SEC_PERMIT_WRITE)
  },
  {
    attHidRimUuid,
    (uint8_t *) hidkbValORepIdMap,
    (uint16_t *) &hidkbLenORepIdMap,
    sizeof(hidkbValORepIdMap),
    0,
    HIDKB_SEC_PERMIT_READ
  },

  {
    attChUuid,
    (uint8_t *) hidkbFRepCh,
    (uint16_t *) &hidkbLenFRepCh,
    sizeof(hidkbFRepCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidRepChUuid,
    (uint8_t *) hidkbFRepVal,
    (uint16_t *) &hidkbLenFRepVal,
    sizeof(hidkbFRepVal),
    (ATTS_SET_WRITE_CBACK | ATTS_SET_VARIABLE_LEN),
    (HIDKB_SEC_PERMIT_READ | HIDKB_SEC_PERMIT_WRITE)
  },
  {
    attHidRimUuid,
    (uint8_t *) hidkbValFRepIdMap,
    (uint16_t *) &hidkbLenFRepIdMap,
    sizeof(hidkbValFRepIdMap),
    0,
    HIDKB_SEC_PERMIT_READ
  },

  {
    attChUuid,
    (uint8_t *) hidkbPmCh,
    (uint16_t *) &hidkbLenPmCh,
    sizeof(hidkbPmCh),
    0,
    ATTS_PERMIT_READ
  },
  {
    attHidRepChUuid,
    (uint8_t *) hidkbPmVal,
    (uint16_t *) &hidkbLenPmVal,
    sizeof(hidkbPmVal),
    ATTS_SET_WRITE_CBACK,
    (HIDKB_SEC_PERMIT_READ | HIDKB_SEC_PERMIT_WRITE)
  }
};

/* HID group structure */
static attsGroup_t svcHidkbGroup =
{
  NULL,
  (attsAttr_t *) hidkbList,
  NULL,
  NULL,
  HID_START_HDL,
  HIDKB_END_HDL
};

/*************************************************************************************************/
/*!
 *  \fn     SvcHidKeyboardAddGroup
 *        
 *  \brief  Add the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcHidKeyboardAddGroup(void)
{
  memset(hidkbRmVal, 0, sizeof(hidkbRmVal));
  AttsAddGroup(&svcHidkbGroup);
}

/*************************************************************************************************/
/*!
 *  \fn     SvcHidKeyboardRemoveGroup
 *        
 *  \brief  Remove the services from the attribute server.
 *
 *  \return None.
 */
/*************************************************************************************************/
void SvcHidKeyboardRemoveGroup(void)
{
  AttsRemoveGroup(HID_START_HDL);
}

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
void SvcHidKeyboardRegister(attsWriteCback_t writeCb, attsReadCback_t readCb)
{
  svcHidkbGroup.writeCback = writeCb;
  svcHidkbGroup.readCback = readCb;
}