/*************************************************************************************************/
/*!
 *  \file   keyboard_main.c
 *
 *  \brief  HID Keyboard sample application.
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
#include "wsf_msg.h"
#include "wsf_trace.h"
#include "wsf_assert.h"
#include "wsf_os.h"
#include "hci_api.h"
#include "dm_api.h"
#include "att_api.h"
#include "smp_api.h"
#include "app_cfg.h"
#include "app_api.h"
#include "app_db.h"
#include "app_ui.h"
#include "svc_ch.h"
#include "svc_hid.h"
#include "svc_core.h"
#include "svc_dis.h"
#include "svc_batt.h"
#include "bas_api.h"
#include "gatt_api.h"
#include "hid_api.h"
#include "keyboard_api.h"

/**************************************************************************************************
  Configurable Parameters
**************************************************************************************************/

/*! configurable parameters for advertising */
static const appAdvCfg_t keyboardAdvCfg =
{
  {60000,     0,     0},                  /*! Advertising durations in ms */
  {   64,  1600,     0}                   /*! Advertising intervals in 0.625 ms units */
};

/*! configurable parameters for slave */
static const appSlaveCfg_t keyboardSlaveCfg =
{
  1,                                      /*! Maximum connections */
};

/*! configurable parameters for security */
static const appSecCfg_t keyboardSecCfg =
{
  DM_AUTH_BOND_FLAG | DM_AUTH_SC_FLAG,    /*! Authentication and bonding flags */
  0,                                      /*! Initiator key distribution flags */
  DM_KEY_DIST_LTK,                        /*! Responder key distribution flags */
  FALSE,                                  /*! TRUE if Out-of-band pairing data is present */
  TRUE                                    /*! TRUE to initiate security upon connection */
};

/*! configurable parameters for connection parameter update */
static const appUpdateCfg_t keyboardUpdateCfg =
{
  0,                                      /*! Connection idle period in ms before attempting
                                              connection parameter update; set to zero to disable */
  640,                                    /*! Minimum connection interval in 1.25ms units */
  800,                                    /*! Maximum connection interval in 1.25ms units */
  3,                                      /*! Connection latency */
  600,                                    /*! Supervision timeout in 10ms units */
  5                                       /*! Number of update attempts before giving up */
};

/*! battery measurement configuration */
static const basCfg_t keyboardBasCfg =
{
  30,       /*! Battery measurement timer expiration period in seconds */
  1,        /*! Perform battery measurement after this many timer periods */
  100       /*! Send battery level notification to peer when below this level. */
};

/*! SMP security parameter configuration */
static const smpCfg_t keyboardSmpCfg =
{
  3000,                                   /*! 'Repeated attempts' timeout in msec */
  SMP_IO_NO_IN_NO_OUT,                    /*! I/O Capability */
  7,                                      /*! Minimum encryption key length */
  16,                                     /*! Maximum encryption key length */
  3,                                      /*! Attempts to trigger 'repeated attempts' timeout */
  0                                       /*! Device authentication requirements */
};

/**************************************************************************************************
  Advertising Data
**************************************************************************************************/

/*! advertising data, discoverable mode */
static const uint8_t keyboardAdvDataDisc[] =
{
  /*! flags */
  2,                                      /*! length */
  DM_ADV_TYPE_FLAGS,                      /*! AD type */
  DM_FLAG_LE_GENERAL_DISC |               /*! flags */
  DM_FLAG_LE_BREDR_NOT_SUP,
  
  /*! manufacturer specific data */
  3,                                      /*! length */
  DM_ADV_TYPE_MANUFACTURER,               /*! AD type */
  UINT16_TO_BYTES(HCI_ID_WICENTRIC),      /*! company ID */

  /*! service UUID list */
  3,                                      /*! length */
  DM_ADV_TYPE_16_UUID,                    /*! AD type */
  UINT16_TO_BYTES(ATT_UUID_HID_SERVICE)
};

/*! scan data, discoverable mode */
static const uint8_t keyboardScanDataDisc[] =
{
  /*! device name */
  13,                                     /*! length */
  DM_ADV_TYPE_LOCAL_NAME,                /*! AD type */
  'H',
  'I',
  'D',
  ' ',
  'K',
  'e',
  'y',
  'b',
  'o',
  'a',
  'r',
  'd'
};

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! WSF message event starting value */
#define KEYBOARD_MSG_START               0xA0

/*! WSF message event enumeration */
enum
{
  KEYBOARD_BATT_TIMER_IND = KEYBOARD_MSG_START,       /*! Battery timer expired */
};

/* Keyboard input record message format */
#define KEYBOARD_IR_MODIFIER_POS          0
#define KEYBOARD_IR_RESERVED_POS          1
#define KEYBOARD_IR_KEY_POS               2
#define KEYBOARD_IR_MAX_KEYS              6
#define KEYBOARD_INPUT_REPORT_LEN         8
#define KEYBOARD_OUTPUT_REPORT_LEN        1
#define KEYBOARD_FEATURE_REPORT_LEN       1

/* Keyboard TX path flags */
#define KEYBOARD_TX_FLAGS_READY           0x01
#define KEYBOARD_TX_FLAGS_PENDING         0x02

/* Keyboard LED Identifier bits */
#define KEYBOARD_LED_NUM_LOCK             (1<<0)
#define KEYBOARD_LED_CAPS_LOCK            (1<<1)
#define KEYBOARD_LED_SCROLL_LOCK          (1<<2)
#define KEYBOARD_LED_COMPOSE              (1<<3)
#define KEYBOARD_LED_KANA                 (1<<4)

/* Keyboard usage definitions */
#define KEYBOARD_USAGE_NONE               0x00
#define KEYBOARD_USAGE_CAPS_LOCK          0x39
#define KEYBOARD_USAGE_DOWN_ARROW         0x51
#define KEYBOARD_USAGE_UP_ARROW           0x52

/**************************************************************************************************
  Client Characteristic Configuration Descriptors
**************************************************************************************************/

/*! enumeration of client characteristic configuration descriptors */
enum
{
  KEYBOARD_GATT_SC_CCC_IDX,                    /*! GATT service, service changed characteristic */
  KEYBOARD_KBI_CCC_HDL,                        /*! Keyboard Boot Input characteristic */
  KEYBOARD_IR_CCC_HDL,                         /*! Input Report characteristic */
  KEYBOARD_BATT_LVL_CCC_IDX,                   /*! Battery service, battery level characteristic */
  KEYBOARD_NUM_CCC_IDX
};

/*! client characteristic configuration descriptors settings, indexed by above enumeration */
static const attsCccSet_t keyboardCccSet[KEYBOARD_NUM_CCC_IDX] =
{
  /* cccd handle                       value range               security level */
  {GATT_SC_CH_CCC_HDL,                 ATT_CLIENT_CFG_INDICATE,  DM_SEC_LEVEL_NONE},   /* KEYBOARD_GATT_SC_CCC_IDX */
  {HIDKB_KEYBOARD_BOOT_IN_CH_CCC_HDL,  ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE},   /* KEYBOARD_KBI_CCC_HDL */
  {HIDKB_INPUT_REPORT_CH_CCC_HDL,      ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE},   /* KEYBOARD_IR_CCC_HDL */
  {BATT_LVL_CH_CCC_HDL,                ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE}    /* KEYBOARD_BATT_LVL_CCC_IDX */
};

/*! HID Report Type/ID and attribute handle map */
static const hidReportIdMap_t keyboardReportIdSet[] =
{
  /* type                         ID             handle */
  {HID_REPORT_TYPE_INPUT,         0,             HIDKB_INPUT_REPORT_HDL},      /* Input Report */
  {HID_REPORT_TYPE_OUTPUT,        0,             HIDKB_OUTPUT_REPORT_HDL},     /* Output Report */
  {HID_REPORT_TYPE_FEATURE,       0,             HIDKB_FEATURE_REPORT_HDL},    /* Feature Report */
  {HID_REPORT_TYPE_INPUT,         HID_BOOT_ID,   HIDKB_KEYBOARD_BOOT_IN_HDL},  /* Boot Keyboard Input Report */
  {HID_REPORT_TYPE_OUTPUT,        HID_BOOT_ID,   HIDKB_KEYBOARD_BOOT_OUT_HDL}  /* Boot Keyboard Output Report */
};

/*! Keyboard Report Map (Descriptor) for generic keyboards */
static const uint8_t keyboardReportMap[] =
{
  0x05, 0x01,                         /* Usage Page (Generic Desktop) */
  0x09, 0x06,                         /* Usage (Keyboard) */
  0xA1, 0x01,                         /* Collection (Application) */
  0x75, 0x01,                         /*     Report Size (1) */
  0x95, 0x08,                         /*     Report Count (8) */
  0x05, 0x07,                         /*     Usage Page (Key Codes) */
  0x19, 0xe0,                         /*     Usage Minimum (224) */
  0x29, 0xe7,                         /*     Usage Maximum (231) */
  0x15, 0x00,                         /*     Logical Minimum (0) */
  0x25, 0x01,                         /*     Logical Maximum (1) */
  0x81, 0x02,                         /*     Input (Data, Variable, Absolute) */

  0x95, 0x01,                         /*     Report Count (1) */
  0x75, 0x08,                         /*     Report Size (8) */
  0x81, 0x01,                         /*     Input (Constant) reserved byte(1) */

  0x95, 0x05,                         /*     Report Count (5) */
  0x75, 0x01,                         /*     Report Size (1) */
  0x05, 0x08,                         /*     Usage Page (Page# for LEDs) */
  0x19, 0x01,                         /*     Usage Minimum (1) */
  0x29, 0x05,                         /*     Usage Maximum (5) */
  0x91, 0x02,                         /*     Output (Data, Variable, Absolute), Led report */
  0x95, 0x01,                         /*     Report Count (1) */
  0x75, 0x03,                         /*     Report Size (3) */
  0x91, 0x01,                         /*     Output (Constant), Led report padding */

  0x95, 0x06,                         /*     Report Count (6) */
  0x75, 0x08,                         /*     Report Size (8) */
  0x15, 0x00,                         /*     Logical Minimum (0) */
  0x25, 0x65,                         /*     Logical Maximum (101) */
  0x05, 0x07,                         /*     Usage Page (Key codes) */
  0x19, 0x00,                         /*     Usage Minimum (0) */
  0x29, 0x65,                         /*     Usage Maximum (101) */
  0x81, 0x00,                         /*     Input (Data, Array) Key array(6 bytes) */

  0xC0                                /* End Collection (Application) */
};

/*! HID Callback prototypes */
static void keyboardOutputReportCback(dmConnId_t connId, uint8_t id, uint16_t len, uint8_t *pReport);
static void keyboardInfoCback(dmConnId_t connId, uint8_t type, uint8_t value);

/*! HID Profile Configuration */
static const hidConfig_t keyboardHidConfig = 
{
  HID_DEVICE_TYPE_KEYBOARD,                               /* Type of HID device */
  (uint8_t*) keyboardReportMap,                           /* Report Map */
  sizeof(keyboardReportMap),                              /* Size of report map in bytes */
  (hidReportIdMap_t*) keyboardReportIdSet,                /* Report ID to Attribute Handle map */
  sizeof(keyboardReportIdSet)/sizeof(hidReportIdMap_t),   /* Size of Report ID to Attribute Handle map */
  &keyboardOutputReportCback,                             /* Output Report Callback */
  NULL,                                                   /* Feature Report Callback */
  &keyboardInfoCback                                      /* Info Callback */
};

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! application control block */
struct
{
  wsfHandlerId_t handlerId;             /* task handle */
  uint8_t modifier;                     /* pending key modifiers */
  uint8_t keys[KEYBOARD_IR_MAX_KEYS];   /* pending keys */
  uint8_t txFlags;                      /* transmit flags */
  uint8_t protocolMode;                 /* current protocol mode */
  uint8_t hostSuspended;                /* TRUE if host suspended */
} keyboardCb;

/*************************************************************************************************/
/*!
 *  \fn     keyboardDmCback
 *        
 *  \brief  Application DM callback.
 *
 *  \param  pDmEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void keyboardDmCback(dmEvt_t *pDmEvt)
{
  dmEvt_t   *pMsg;
  uint16_t  len;
  
  len = sizeof(dmEvt_t);

  if ((pMsg = WsfMsgAlloc(len)) != NULL)
  {
    memcpy(pMsg, pDmEvt, sizeof(dmEvt_t));
    WsfMsgSend(keyboardCb.handlerId, pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     keyboardAttCback
 *        
 *  \brief  Application  ATT callback.
 *
 *  \param  pEvt    ATT callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void keyboardAttCback(attEvt_t *pEvt)
{
  attEvt_t *pMsg;
  
  if ((pMsg = WsfMsgAlloc(sizeof(attEvt_t) + pEvt->valueLen)) != NULL)
  {
    memcpy(pMsg, pEvt, sizeof(attEvt_t));
    pMsg->pValue = (uint8_t *) (pMsg + 1);
    memcpy(pMsg->pValue, pEvt->pValue, pEvt->valueLen);
    WsfMsgSend(keyboardCb.handlerId, pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     keyboardCccCback
 *        
 *  \brief  Application ATTS client characteristic configuration callback.
 *
 *  \param  pDmEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void keyboardCccCback(attsCccEvt_t *pEvt)
{
  appDbHdl_t    dbHdl;
  
  /* if CCC not set from initialization and there's a device record */
  if ((pEvt->handle != ATT_HANDLE_NONE) &&
      ((dbHdl = AppDbGetHdl((dmConnId_t) pEvt->hdr.param)) != APP_DB_HDL_NONE))
  {
    /* store value in device database */  
    AppDbSetCccTblValue(dbHdl, pEvt->idx, pEvt->value);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     keyboardOpen
 *        
 *  \brief  Perform UI actions on connection open.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void keyboardOpen(dmEvt_t *pMsg)
{
  HidSetProtocolMode(HID_PROTOCOL_MODE_REPORT);
}

/*************************************************************************************************/
/*!
 *  \fn     keyboardSetup
 *        
 *  \brief  Set up procedures that need to be performed after device reset.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void keyboardSetup(dmEvt_t *pMsg)
{
  /* set advertising and scan response data for discoverable mode */
  AppAdvSetData(APP_ADV_DATA_DISCOVERABLE, sizeof(keyboardAdvDataDisc), (uint8_t *) keyboardAdvDataDisc);
  AppAdvSetData(APP_SCAN_DATA_DISCOVERABLE, sizeof(keyboardScanDataDisc), (uint8_t *) keyboardScanDataDisc);

  /* set advertising and scan response data for connectable mode */
  AppAdvSetData(APP_ADV_DATA_CONNECTABLE, sizeof(keyboardAdvDataDisc), (uint8_t *) keyboardAdvDataDisc);
  AppAdvSetData(APP_SCAN_DATA_CONNECTABLE, sizeof(keyboardScanDataDisc), (uint8_t *) keyboardScanDataDisc);
  
  /* start advertising; automatically set connectable/discoverable mode and bondable mode */
  AppAdvStart(APP_MODE_AUTO_INIT);
}

/*************************************************************************************************/
/*!
 *  \fn     keyboardSendData
 *        
 *  \brief  Send example data.
 *
 *  \param  connId    Connection identifier.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void keyboardSendData(dmConnId_t connId)
{
  uint8_t cccHandle = KEYBOARD_IR_CCC_HDL;
  uint8_t protocolMode;

  protocolMode = HidGetProtocolMode();

  if (protocolMode == HID_PROTOCOL_MODE_BOOT)
  {
    cccHandle = KEYBOARD_KBI_CCC_HDL;
  }

  if (AttsCccEnabled(connId, cccHandle))
  {
    if (keyboardCb.txFlags & KEYBOARD_TX_FLAGS_PENDING)
    {
      if (keyboardCb.txFlags & KEYBOARD_TX_FLAGS_READY)
      {
        uint8_t buffer[KEYBOARD_INPUT_REPORT_LEN];
        uint8_t reportId = 0;
        
        /* modifier, reserved, keys[6] */
        buffer[KEYBOARD_IR_MODIFIER_POS] = keyboardCb.modifier;
        buffer[KEYBOARD_IR_RESERVED_POS] = 0;
        memcpy(buffer + KEYBOARD_IR_KEY_POS, keyboardCb.keys, sizeof(keyboardCb.keys));

        keyboardCb.txFlags &= ~(KEYBOARD_TX_FLAGS_READY | KEYBOARD_TX_FLAGS_PENDING);

        if (protocolMode == HID_PROTOCOL_MODE_BOOT)
        {
          reportId = HID_BOOT_ID;
        }

        /* Send the message */
        HidSendInputReport(connId, reportId, KEYBOARD_INPUT_REPORT_LEN, buffer);
      }
    }
  }
}

/*************************************************************************************************/
/*!
 *  \fn     KeyboardReportEvent
 *        
 *  \brief  Send or queue a keyboard event to the host
 *
 *  \return None.
 */
/*************************************************************************************************/
static void KeyboardReportEvent(uint8_t modifiers, uint8_t keys[], uint8_t numKeys)
{
  dmConnId_t connId;

  if ((connId = AppConnIsOpen()) != DM_CONN_ID_NONE)
  {
    /* max 6 keys in report map */
    if (numKeys > KEYBOARD_IR_MAX_KEYS)
    {
      numKeys = KEYBOARD_IR_MAX_KEYS;
    }

    /* record key data */
    keyboardCb.modifier = modifiers;
    memset(keyboardCb.keys, 0, sizeof(keyboardCb.keys));
    memcpy(keyboardCb.keys, keys, numKeys);

    /* indicate new data is pending */
    keyboardCb.txFlags |= KEYBOARD_TX_FLAGS_PENDING;
    
    /* send the data */
    keyboardSendData(connId);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     keyboardBtnCback
 *        
 *  \brief  Button press callback.
 *
 *  \param  btn    Button press.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void keyboardBtnCback(uint8_t btn)
{
  dmConnId_t      connId;
  
  /* button actions when connected */
  if ((connId = AppConnIsOpen()) != DM_CONN_ID_NONE)
  {
    uint8_t key;

    switch (btn)
    {
      case APP_UI_BTN_2_DOWN:
        /* Send up arrow key */
        key = KEYBOARD_USAGE_DOWN_ARROW;
        KeyboardReportEvent(0, &key, 1);
        break;

      case APP_UI_BTN_1_DOWN:
        /* Send up arrow key */
        key = KEYBOARD_USAGE_UP_ARROW;
        KeyboardReportEvent(0, &key, 1);
        break;

      case APP_UI_BTN_1_SHORT:            
      case APP_UI_BTN_1_MED:
      case APP_UI_BTN_1_LONG:
      case APP_UI_BTN_1_EX_LONG:
      case APP_UI_BTN_2_SHORT:            
      case APP_UI_BTN_2_MED:
      case APP_UI_BTN_2_LONG:
      case APP_UI_BTN_2_EX_LONG:
        /* Send no key */
        key = KEYBOARD_USAGE_NONE;
        KeyboardReportEvent(0, &key, 1);        
        break;
        
      default:
        break;
    }    
  }
}

/*************************************************************************************************/
/*!
 *  \fn     keyboardProcMsg
 *        
 *  \brief  Process messages from the event handler.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void keyboardProcMsg(dmEvt_t *pMsg)
{
  uint8_t uiEvent = APP_UI_NONE;
  
  switch(pMsg->hdr.event)
  {
    case ATTS_HANDLE_VALUE_CNF:
      if (pMsg->hdr.status == ATT_SUCCESS)
      {
        keyboardCb.txFlags |= KEYBOARD_TX_FLAGS_READY;
        keyboardSendData((dmConnId_t) pMsg->hdr.param);
      }
      break;
     
    case KEYBOARD_BATT_TIMER_IND:
      BasProcMsg(&pMsg->hdr);
      break;

    case DM_RESET_CMPL_IND:
      DmSecGenerateEccKeyReq();
      keyboardSetup(pMsg);
      uiEvent = APP_UI_RESET_CMPL;
      break;
      
    case DM_CONN_OPEN_IND:
      keyboardOpen(pMsg);
      uiEvent = APP_UI_CONN_OPEN;
      break;
         
    case DM_CONN_CLOSE_IND:
      uiEvent = APP_UI_CONN_CLOSE;
      break;

    case DM_SEC_ECC_KEY_IND:
      DmSecSetEccKey(&pMsg->eccMsg.data.key);
      break;

    case DM_SEC_PAIR_CMPL_IND:
      uiEvent = APP_UI_SEC_PAIR_CMPL;
      break;
     
    case DM_SEC_PAIR_FAIL_IND:
      uiEvent = APP_UI_SEC_PAIR_FAIL;
      break;
     
    case DM_SEC_ENCRYPT_IND:
      uiEvent = APP_UI_SEC_ENCRYPT;
      break;
       
    case DM_SEC_ENCRYPT_FAIL_IND:
      uiEvent = APP_UI_SEC_ENCRYPT_FAIL;
      break;

    case DM_SEC_AUTH_REQ_IND:
      AppHandlePasskey(&pMsg->authReq);
      break;

    default:
      break;
  }
  
  if (uiEvent != APP_UI_NONE)
  {
    AppUiAction(uiEvent);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     keyboardInfoCback
 *        
 *  \brief  Callback to handle a change in protocol mode or control point from the host.
 *
 *  \param  connId    The connection identifier.
 *  \param  mode      The type of information (HID_INFO_CONTROL_POINT or HID_INFO_PROTOCOL_MODE)
 *  \param  value     The value of the information
 *
 *  \return None.
 */
/*************************************************************************************************/
static void keyboardInfoCback(dmConnId_t connId, uint8_t type, uint8_t value)
{
  /* Record information from the host in the control block */
  if (type == HID_INFO_PROTOCOL_MODE)
  {
    keyboardCb.protocolMode = value;
  }
  else if (type == HID_INFO_CONTROL_POINT)
  {
    keyboardCb.hostSuspended = (value == HID_CONTROL_POINT_SUSPEND) ? TRUE : FALSE;
  }
}

/*************************************************************************************************/
/*!
 *  \fn     keyboardOutputReportCback
 *        
 *  \brief  Callback that receives and processes HID output reports from the peer.
 *
 *  \param  connId    The connection identifier.
 *  \param  id        The report ID
 *  \param  len       The length of the report data in pReport.
 *  \param  pReport   A buffer containing the report.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void keyboardOutputReportCback(dmConnId_t connId, uint8_t id, uint16_t len, uint8_t *pReport)
{
  uint8_t ledMask = *pReport;

  APP_TRACE_INFO1(">>> Keyboard LED Change: %#x <<<", ledMask);

  /* TODO: Enable/disable keyboard LEDs */
}

/*************************************************************************************************/
/*!
 *  \fn     KeyboardHandlerInit
 *        
 *  \brief  Application handler init function called during system initialization.
 *
 *  \param  handlerID  WSF handler ID.
 *
 *  \return None.
 */
/*************************************************************************************************/
void KeyboardHandlerInit(wsfHandlerId_t handlerId)
{
  APP_TRACE_INFO0("KeyboardHandlerInit");

  /* Initialize the control block */
  memset(&keyboardCb, 0, sizeof(keyboardCb));
  keyboardCb.txFlags = KEYBOARD_TX_FLAGS_READY;

  /* store handler ID */
  keyboardCb.handlerId = handlerId;
    
  /* Set configuration pointers */
  pAppSlaveCfg = (appSlaveCfg_t *) &keyboardSlaveCfg;
  pAppAdvCfg = (appAdvCfg_t *) &keyboardAdvCfg;
  pAppSecCfg = (appSecCfg_t *) &keyboardSecCfg;
  pAppUpdateCfg = (appUpdateCfg_t *) &keyboardUpdateCfg;

  /* Set stack configuration pointers */
  pSmpCfg = (smpCfg_t *) &keyboardSmpCfg;

  /* Initialize application framework */
  AppSlaveInit();

  /* initialize battery service server */
  BasInit(handlerId, (basCfg_t *) &keyboardBasCfg);
}

/*************************************************************************************************/
/*!
 *  \fn     KeyboardHandler
 *        
 *  \brief  WSF event handler for application.
 *
 *  \param  event   WSF event mask.
 *  \param  pMsg    WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void KeyboardHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{ 
  if (pMsg != NULL)
  {
    APP_TRACE_INFO1("Keyboard got evt %d", pMsg->event);

    if (pMsg->event <= DM_CBACK_END)
    {
      /* process advertising and connection-related messages */
      AppSlaveProcDmMsg((dmEvt_t *) pMsg);
      
      /* process security-related messages */
      AppSlaveSecProcDmMsg((dmEvt_t *) pMsg);
    }
          
    /* perform profile and user interface-related operations */
    keyboardProcMsg((dmEvt_t *) pMsg);    
  }
}

/*************************************************************************************************/
/*!
 *  \fn     keyboardReportInit
 *        
 *  \brief  Initialize the report attributes to default values for the keyboard.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void keyboardReportInit()
{
  uint8_t iBuffer[KEYBOARD_INPUT_REPORT_LEN];
  uint8_t oBuffer[KEYBOARD_OUTPUT_REPORT_LEN];
  uint8_t fBuffer[KEYBOARD_FEATURE_REPORT_LEN];
  
  /* Input and boot input reports */
  memset(iBuffer, 0, KEYBOARD_INPUT_REPORT_LEN);
  AttsSetAttr(HIDKB_INPUT_REPORT_HDL, KEYBOARD_INPUT_REPORT_LEN, iBuffer);
  AttsSetAttr(HIDKB_KEYBOARD_BOOT_IN_HDL, KEYBOARD_INPUT_REPORT_LEN, iBuffer);

  /* Output and boot output reports */
  memset(oBuffer, 0, KEYBOARD_OUTPUT_REPORT_LEN);
  AttsSetAttr(HIDKB_OUTPUT_REPORT_HDL, KEYBOARD_OUTPUT_REPORT_LEN, oBuffer);
  AttsSetAttr(HIDKB_KEYBOARD_BOOT_OUT_HDL, KEYBOARD_OUTPUT_REPORT_LEN, oBuffer);

  /* feature report */
  memset(fBuffer, 0, KEYBOARD_FEATURE_REPORT_LEN);
  AttsSetAttr(HIDKB_FEATURE_REPORT_HDL, KEYBOARD_FEATURE_REPORT_LEN, fBuffer);
}

/*************************************************************************************************/
/*!
 *  \fn     KeyboardTest
 *        
 *  \brief  Test a keyboard button event.
 *
 *  \return None.
 */
/*************************************************************************************************/
void KeyboardTest()
{
  keyboardBtnCback(APP_UI_BTN_1_DOWN);
  keyboardBtnCback(APP_UI_BTN_1_SHORT);
}

/*************************************************************************************************/
/*!
 *  \fn     KeyboardStart
 *        
 *  \brief  Start the application.
 *
 *  \return None.
 */
/*************************************************************************************************/
void KeyboardStart(void)
{  
  /* Register for stack callbacks */
  DmRegister(keyboardDmCback);
  DmConnRegister(DM_CLIENT_ID_APP, keyboardDmCback);
  AttRegister(keyboardAttCback);
  AttConnRegister(AppServerConnCback);
  AttsCccRegister(KEYBOARD_NUM_CCC_IDX, (attsCccSet_t *) keyboardCccSet, keyboardCccCback);

  /* Register for app framework button callbacks */
  AppUiBtnRegister(keyboardBtnCback);
 
  /* Initialize the HID service */
  SvcHidKeyboardAddGroup();
  HidInit(&keyboardHidConfig);

  /* Initialize attribute server database */
  SvcCoreAddGroup();
  SvcDisAddGroup();
  SvcBattCbackRegister(BasReadCback, NULL);
  SvcBattAddGroup();

  /* Initialize the report attributes */
  keyboardReportInit();

  /* Reset the device */
  DmDevReset();  
}
