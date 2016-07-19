/*************************************************************************************************/
/*!
 *  \file   remote_main.c
 *
 *  \brief  HID Remote sample application.
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
#include "remote_api.h"

/**************************************************************************************************
  Configurable Parameters
**************************************************************************************************/

/*! configurable parameters for advertising */
static const appAdvCfg_t remoteAdvCfg =
{
  {60000,     0,     0},                  /*! Advertising durations in ms */
  {   64,  1600,     0}                   /*! Advertising intervals in 0.625 ms units */
};

/*! configurable parameters for slave */
static const appSlaveCfg_t remoteSlaveCfg =
{
  1,                                      /*! Maximum connections */
};

/*! configurable parameters for security */
static const appSecCfg_t remoteSecCfg =
{
  DM_AUTH_BOND_FLAG | DM_AUTH_SC_FLAG,    /*! Authentication and bonding flags */
  0,                                      /*! Initiator key distribution flags */
  DM_KEY_DIST_LTK,                        /*! Responder key distribution flags */
  FALSE,                                  /*! TRUE if Out-of-band pairing data is present */
  TRUE                                    /*! TRUE to initiate security upon connection */
};

/*! configurable parameters for connection parameter update */
static const appUpdateCfg_t remoteUpdateCfg =
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
static const basCfg_t remoteBasCfg =
{
  30,       /*! Battery measurement timer expiration period in seconds */
  1,        /*! Perform battery measurement after this many timer periods */
  100       /*! Send battery level notification to peer when below this level. */
};

/**************************************************************************************************
  Advertising Data
**************************************************************************************************/

/*! advertising data, discoverable mode */
static const uint8_t remoteAdvDataDisc[] =
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
static const uint8_t remoteScanDataDisc[] =
{
  /*! device name */
  11,                                     /*! length */
  DM_ADV_TYPE_LOCAL_NAME,                 /*! AD type */
  'H',
  'I',
  'D',
  ' ',
  'R',
  'e',
  'm',
  'o',
  't',
  'e'
};

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! WSF message event starting value */
#define REMOTE_MSG_START               0xA0

/*! WSF message event enumeration */
enum
{
  REMOTE_BATT_TIMER_IND = REMOTE_MSG_START,       /*! Battery timer expired */
};

/* Remote TX path flags */
#define REMOTE_TX_FLAGS_READY           0x01
#define REMOTE_TX_FLAGS_PENDING         0x02

/* Remote Button Identifier bits */
#define REMOTE_USAGE_NONE                   0
#define REMOTE_VOLUME_UP                (1<<0)
#define REMOTE_VOLUME_DOWN              (1<<1)
#define REMOTE_MUTE                     (1<<2)
#define REMOTE_PLAY                     (1<<3)
#define REMOTE_PAUSE                    (1<<4)
#define REMOTE_STOP                     (1<<5)
#define REMOTE_NEXT                     (1<<6)
#define REMOTE_PREVIOUS                 (1<<7)

/* The input report fits in one byte */
#define REMOTE_INPUT_REPORT_LEN         1
#define REMOTE_OUTPUT_REPORT_LEN        1
#define REMOTE_FEATURE_REPORT_LEN       1

/**************************************************************************************************
  Client Characteristic Configuration Descriptors
**************************************************************************************************/

/*! enumeration of client characteristic configuration descriptors */
enum
{
  REMOTE_GATT_SC_CCC_IDX,                    /*! GATT service, service changed characteristic */
  REMOTE_IR_CCC_HDL,                         /*! Input Report characteristic */
  REMOTE_BATT_LVL_CCC_IDX,                   /*! Battery service, battery level characteristic */
  REMOTE_NUM_CCC_IDX
};

/*! client characteristic configuration descriptors settings, indexed by above enumeration */
static const attsCccSet_t remoteCccSet[REMOTE_NUM_CCC_IDX] =
{
  /* cccd handle                       value range               security level */
  {GATT_SC_CH_CCC_HDL,                 ATT_CLIENT_CFG_INDICATE,  DM_SEC_LEVEL_NONE},   /* REMOTE_GATT_SC_CCC_IDX */
  {HIDG_INPUT_REPORT_CH_CCC_HDL,       ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE},   /* REMOTE_IR_CCC_HDL */
  {BATT_LVL_CH_CCC_HDL,                ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE}    /* REMOTE_BATT_LVL_CCC_IDX */
};

/*! HID Report Type/ID and attribute handle map */
static const hidReportIdMap_t remoteReportIdSet[] =
{
  /* type                         ID             handle */
  {HID_REPORT_TYPE_INPUT,         0,             HIDG_INPUT_REPORT_HDL},    /* Input Report */
  {HID_REPORT_TYPE_OUTPUT,        0,             HIDG_OUTPUT_REPORT_HDL},   /* Output Report */
  {HID_REPORT_TYPE_FEATURE,       0,             HIDG_FEATURE_REPORT_HDL},  /* Feature Report */
};

/*! Remote Report Map (Descriptor) for generic remotes */
static const uint8_t remoteReportMap[] =
{
    0x05, 0x0c,                    /*	Usage Page (Consumer Devices) */
    0x09, 0x01,                    /*	Usage (Consumer Control) */
    0xa1, 0x01,                    /*	Collection (Application) */
    0x15, 0x00,                    /*	  Logical Minimum (0) */
    0x25, 0x01,                    /*	  Logical Maximum (1) */
    0x09, 0xe9,                    /*	  Usage (Volume Up) */
    0x09, 0xea,                    /*	  Usage (Volume Down) */
    0x75, 0x01,                    /*	  Report Size (1) */
    0x95, 0x02,                    /*	  Report Count (2) */
    0x81, 0x06,                    /*	  Input (Data, Variable, Relative) */
    0x09, 0xe2,                    /*	  Usage (Mute) */
    0x95, 0x01,                    /*	  Report Count (1) */
    0x81, 0x06,                    /*	  Input (Data, Variable, Relative) */
    0x09, 0xb0,                    /*	  Usage (Play) */
    0x95, 0x01,                    /*	  Report Count (1) */
    0x81, 0x06,                    /*	  Input (Data, Variable, Relative) */
    0x09, 0xb1,                    /*	  Usage (Pause) */
    0x95, 0x01,                    /*	  Report Count (1) */
    0x81, 0x06,                    /*	  Input (Data, Variable, Relative) */
    0x09, 0xb7,                    /*	  Usage (Stop) */
    0x95, 0x01,                    /*	  Report Count (1) */
    0x81, 0x06,                    /*	  Input (Data, Variable, Relative) */
    0x09, 0xb5,                    /*	  Usage (Next) */
    0x95, 0x01,                    /*	  Report Count (1) */
    0x81, 0x06,                    /*	  Input (Data, Variable, Relative) */
    0x09, 0xb6,                    /*	  Usage (Previous) */
    0x95, 0x01,                    /*	  Report Count (1) */
    0x81, 0x06,                    /*	  Input (Data, Variable, Relative) */
    0xc0                           /*	End Collection */
};

/*! HID Callback prototypes */
void remoteInfoCback(dmConnId_t connId, uint8_t type, uint8_t value);

/*! HID Profile Configuration */
static const hidConfig_t remoteHidConfig = 
{
  HID_DEVICE_TYPE_GENERIC,                                /* Type of HID device */
  (uint8_t*) remoteReportMap,                             /* Report Map */
  sizeof(remoteReportMap),                                /* Size of report map in bytes */
  (hidReportIdMap_t*) remoteReportIdSet,                  /* Report ID to Attribute Handle map */
  sizeof(remoteReportIdSet)/sizeof(hidReportIdMap_t),     /* Size of Report ID to Attribute Handle map */
  NULL,                                                   /* Output Report Callback */
  NULL,                                                   /* Feature Report Callback */
  &remoteInfoCback                                        /* Info Callback */
};

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! application control block */
struct
{
  wsfHandlerId_t handlerId;             /* task handle */
  uint8_t btnData;                      /* pending remote button data */
  uint8_t txFlags;                      /* transmit flags */
  uint8_t protocolMode;                 /* current protocol mode */
  uint8_t hostSuspended;                /* TRUE if host suspended */
} remoteCb;

/*************************************************************************************************/
/*!
 *  \fn     remoteDmCback
 *        
 *  \brief  Application DM callback.
 *
 *  \param  pDmEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void remoteDmCback(dmEvt_t *pDmEvt)
{
  dmEvt_t   *pMsg;
  uint16_t  len;
  
  len = sizeof(dmEvt_t);

  if ((pMsg = WsfMsgAlloc(len)) != NULL)
  {
    memcpy(pMsg, pDmEvt, sizeof(dmEvt_t));
    WsfMsgSend(remoteCb.handlerId, pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     remoteAttCback
 *        
 *  \brief  Application  ATT callback.
 *
 *  \param  pEvt    ATT callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void remoteAttCback(attEvt_t *pEvt)
{
  attEvt_t *pMsg;
  
  if ((pMsg = WsfMsgAlloc(sizeof(attEvt_t) + pEvt->valueLen)) != NULL)
  {
    memcpy(pMsg, pEvt, sizeof(attEvt_t));
    pMsg->pValue = (uint8_t *) (pMsg + 1);
    memcpy(pMsg->pValue, pEvt->pValue, pEvt->valueLen);
    WsfMsgSend(remoteCb.handlerId, pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     remoteCccCback
 *        
 *  \brief  Application ATTS client characteristic configuration callback.
 *
 *  \param  pDmEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void remoteCccCback(attsCccEvt_t *pEvt)
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
 *  \fn     remoteOpen
 *        
 *  \brief  Perform UI actions on connection open.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void remoteOpen(dmEvt_t *pMsg)
{

}

/*************************************************************************************************/
/*!
 *  \fn     remoteSetup
 *        
 *  \brief  Set up procedures that need to be performed after device reset.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void remoteSetup(dmEvt_t *pMsg)
{
  /* set advertising and scan response data for discoverable mode */
  AppAdvSetData(APP_ADV_DATA_DISCOVERABLE, sizeof(remoteAdvDataDisc), (uint8_t *) remoteAdvDataDisc);
  AppAdvSetData(APP_SCAN_DATA_DISCOVERABLE, sizeof(remoteScanDataDisc), (uint8_t *) remoteScanDataDisc);

  /* set advertising and scan response data for connectable mode */
  AppAdvSetData(APP_ADV_DATA_CONNECTABLE, sizeof(remoteAdvDataDisc), (uint8_t *) remoteAdvDataDisc);
  AppAdvSetData(APP_SCAN_DATA_CONNECTABLE, sizeof(remoteScanDataDisc), (uint8_t *) remoteScanDataDisc);
  
  /* start advertising; automatically set connectable/discoverable mode and bondable mode */
  AppAdvStart(APP_MODE_AUTO_INIT);
}

/*************************************************************************************************/
/*!
 *  \fn     remoteSendData
 *        
 *  \brief  Send example data.
 *
 *  \param  connId    Connection identifier.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void remoteSendData(dmConnId_t connId)
{
  if (AttsCccEnabled(connId, REMOTE_IR_CCC_HDL))
  {
    if (remoteCb.txFlags & REMOTE_TX_FLAGS_PENDING)
    {
      if (remoteCb.txFlags & REMOTE_TX_FLAGS_READY)
      {
        uint8_t buffer;
        
        /* bitmask of remote buttons */
        buffer = remoteCb.btnData;

        remoteCb.txFlags &= ~(REMOTE_TX_FLAGS_READY | REMOTE_TX_FLAGS_PENDING);

        /* Send the message */
        HidSendInputReport(connId, 0, REMOTE_INPUT_REPORT_LEN, &buffer);
      }
    }
  }
}

/*************************************************************************************************/
/*!
 *  \fn     RemoteReportEvent
 *        
 *  \brief  Send or queue a remote event to the host
 *
 *  \return None.
 */
/*************************************************************************************************/
static void RemoteReportEvent(uint8_t remoteButton)
{
  dmConnId_t connId;

  if ((connId = AppConnIsOpen()) != DM_CONN_ID_NONE)
  {
    /* record key data */
    remoteCb.btnData = remoteButton;

    /* Indicate new data is pending */
    remoteCb.txFlags |= REMOTE_TX_FLAGS_PENDING;
    
    /* send the data */
    remoteSendData(connId);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     remoteBtnCback
 *        
 *  \brief  Button press callback.
 *
 *  \param  btn    Button press.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void remoteBtnCback(uint8_t btn)
{
  dmConnId_t      connId;
  
  /* button actions when connected */
  if ((connId = AppConnIsOpen()) != DM_CONN_ID_NONE)
  {
    uint8_t button;

    switch (btn)
    {
      case APP_UI_BTN_2_DOWN:
        /* Send pause command */
        button = REMOTE_PAUSE;
        RemoteReportEvent(button);
        break;

      case APP_UI_BTN_1_DOWN:
        /* Send play command */
        button = REMOTE_PLAY;
        RemoteReportEvent(button);
        break;


      case APP_UI_BTN_1_SHORT:            
      case APP_UI_BTN_1_MED:
      case APP_UI_BTN_1_LONG:
      case APP_UI_BTN_1_EX_LONG:
      case APP_UI_BTN_2_SHORT:            
      case APP_UI_BTN_2_MED:
      case APP_UI_BTN_2_LONG:
      case APP_UI_BTN_2_EX_LONG:
        /* Send no usage */
        button = REMOTE_USAGE_NONE;
        RemoteReportEvent(button);        
        break;
        
      default:
        break;
    }    
  }
}

/*************************************************************************************************/
/*!
 *  \fn     remoteProcMsg
 *        
 *  \brief  Process messages from the event handler.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void remoteProcMsg(dmEvt_t *pMsg)
{
  uint8_t uiEvent = APP_UI_NONE;
  
  switch(pMsg->hdr.event)
  {
    case ATTS_HANDLE_VALUE_CNF:
      if (pMsg->hdr.status == ATT_SUCCESS)
      {
        remoteCb.txFlags |= REMOTE_TX_FLAGS_READY;
        remoteSendData((dmConnId_t) pMsg->hdr.param);
      }
      break;
     
    case REMOTE_BATT_TIMER_IND:
      BasProcMsg(&pMsg->hdr);
      break;

    case DM_RESET_CMPL_IND:
      remoteSetup(pMsg);
      uiEvent = APP_UI_RESET_CMPL;
      break;
      
    case DM_CONN_OPEN_IND:
      remoteOpen(pMsg);
      uiEvent = APP_UI_CONN_OPEN;
      break;
         
    case DM_CONN_CLOSE_IND:
      uiEvent = APP_UI_CONN_CLOSE;
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
 *  \fn     remoteInfoCback
 *        
 *  \brief  Callback to handle a change in protocol mode or control point from the host.
 *
 *  \param  connId    The connection identifier.
 *  \param  mode      The type of information (HID_INFO_CONTROL_POINT or HID_INFO_PROTOCOL_MODE)
 *  \param  value     The value of the information
 *
 *  \return None.
 *
/*************************************************************************************************/
static void remoteInfoCback(dmConnId_t connId, uint8_t type, uint8_t value)
{
  if (type == HID_INFO_PROTOCOL_MODE)
  {
    remoteCb.protocolMode = value;
  }
  else if (type == HID_INFO_CONTROL_POINT)
  {
    remoteCb.hostSuspended = (value == HID_CONTROL_POINT_SUSPEND) ? TRUE : FALSE;
  }
}

/*************************************************************************************************/
/*!
 *  \fn     remoteReportInit
 *        
 *  \brief  Initialize the report attributes to default values for the remote.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void remoteReportInit()
{
  uint8_t iBuffer[REMOTE_INPUT_REPORT_LEN];
  uint8_t oBuffer[REMOTE_OUTPUT_REPORT_LEN];
  uint8_t fBuffer[REMOTE_FEATURE_REPORT_LEN];
  
  /* Input report */
  memset(iBuffer, 0, REMOTE_INPUT_REPORT_LEN);
  AttsSetAttr(HIDG_INPUT_REPORT_HDL, REMOTE_INPUT_REPORT_LEN, iBuffer);

  /* Output report */
  memset(oBuffer, 0, REMOTE_OUTPUT_REPORT_LEN);
  AttsSetAttr(HIDG_OUTPUT_REPORT_HDL, REMOTE_OUTPUT_REPORT_LEN, oBuffer);

  /* feature report */
  memset(fBuffer, 0, REMOTE_FEATURE_REPORT_LEN);
  AttsSetAttr(HIDG_FEATURE_REPORT_HDL, REMOTE_FEATURE_REPORT_LEN, fBuffer);
}

/*************************************************************************************************/
/*!
 *  \fn     RemoteHandlerInit
 *        
 *  \brief  Application handler init function called during system initialization.
 *
 *  \param  handlerID  WSF handler ID.
 *
 *  \return None.
 */
/*************************************************************************************************/
void RemoteHandlerInit(wsfHandlerId_t handlerId)
{
  APP_TRACE_INFO0("RemoteHandlerInit");

  /* Initialize the control block */
  memset(&remoteCb, 0, sizeof(remoteCb));
  remoteCb.txFlags = REMOTE_TX_FLAGS_READY;

  /* store handler ID */
  remoteCb.handlerId = handlerId;
    
  /* Set configuration pointers */
  pAppSlaveCfg = (appSlaveCfg_t *) &remoteSlaveCfg;
  pAppAdvCfg = (appAdvCfg_t *) &remoteAdvCfg;
  pAppSecCfg = (appSecCfg_t *) &remoteSecCfg;
  pAppUpdateCfg = (appUpdateCfg_t *) &remoteUpdateCfg;

  /* Initialize application framework */
  AppSlaveInit();

  /* initialize battery service server */
  BasInit(handlerId, (basCfg_t *) &remoteBasCfg);
}

/*************************************************************************************************/
/*!
 *  \fn     RemoteHandler
 *        
 *  \brief  WSF event handler for application.
 *
 *  \param  event   WSF event mask.
 *  \param  pMsg    WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void RemoteHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{ 
  if (pMsg != NULL)
  {
    APP_TRACE_INFO1("Remote got evt %d", pMsg->event);

    if (pMsg->event <= DM_CBACK_END)
    {
      /* process advertising and connection-related messages */
      AppSlaveProcDmMsg((dmEvt_t *) pMsg);
      
      /* process security-related messages */
      AppSlaveSecProcDmMsg((dmEvt_t *) pMsg);
    }
          
    /* perform profile and user interface-related operations */
    remoteProcMsg((dmEvt_t *) pMsg);    
  }
}

/*************************************************************************************************/
/*!
 *  \fn     RemoteTest
 *        
 *  \brief  Test a remote button event.
 *
 *  \return None.
 */
/*************************************************************************************************/
void RemoteTest()
{
  remoteBtnCback(APP_UI_BTN_1_DOWN);
  remoteBtnCback(APP_UI_BTN_1_SHORT);
}

/*************************************************************************************************/
/*!
 *  \fn     RemoteStart
 *        
 *  \brief  Start the application.
 *
 *  \return None.
 */
/*************************************************************************************************/
void RemoteStart(void)
{  
  /* Register for stack callbacks */
  DmRegister(remoteDmCback);
  DmConnRegister(DM_CLIENT_ID_APP, remoteDmCback);
  AttRegister(remoteAttCback);
  AttConnRegister(AppServerConnCback);
  AttsCccRegister(REMOTE_NUM_CCC_IDX, (attsCccSet_t *) remoteCccSet, remoteCccCback);

  /* Register for app framework button callbacks */
  AppUiBtnRegister(remoteBtnCback);
 
  /* Initialize the HID service */
  SvcHidGenericAddGroup();
  HidInit(&remoteHidConfig);

  /* Initialize attribute server database */
  SvcCoreAddGroup();
  SvcDisAddGroup();
  SvcBattCbackRegister(BasReadCback, NULL);
  SvcBattAddGroup();

  /* Initialize the report attributes */
  remoteReportInit();

  /* Reset the device */
  DmDevReset();  
}
