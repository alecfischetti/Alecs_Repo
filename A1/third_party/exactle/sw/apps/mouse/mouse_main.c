/*************************************************************************************************/
/*!
 *  \file   mouse_main.c
 *
 *  \brief  HID Mouse sample application.
 *
 *          $Date: 2015-10-01 12:19:36 -0700 (Thu, 01 Oct 2015) $
 *          $Revision: 4073 $
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
#include "mouse_api.h"

/**************************************************************************************************
  Configurable Parameters
**************************************************************************************************/

/*! configurable parameters for advertising */
static const appAdvCfg_t mouseAdvCfg =
{
  {60000,     0,     0},                  /*! Advertising durations in ms */
  {   96,  1600,     0}                   /*! Advertising intervals in 0.625 ms units */
};

/*! configurable parameters for slave */
static const appSlaveCfg_t mouseSlaveCfg =
{
  1,                                      /*! Maximum connections */
};

/*! configurable parameters for security */
static const appSecCfg_t mouseSecCfg =
{
  DM_AUTH_BOND_FLAG | DM_AUTH_SC_FLAG,    /*! Authentication and bonding flags */
  0,                                      /*! Initiator key distribution flags */
  DM_KEY_DIST_LTK,                        /*! Responder key distribution flags */
  FALSE,                                  /*! TRUE if Out-of-band pairing data is present */
  TRUE                                    /*! TRUE to initiate security upon connection */
};

/*! configurable parameters for connection parameter update */
static const appUpdateCfg_t mouseUpdateCfg =
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
static const basCfg_t mouseBasCfg =
{
  30,       /*! Battery measurement timer expiration period in seconds */
  1,        /*! Perform battery measurement after this many timer periods */
  100       /*! Send battery level notification to peer when below this level. */
};

/**************************************************************************************************
  Advertising Data
**************************************************************************************************/

/*! advertising data, discoverable mode */
static const uint8_t mouseAdvDataDisc[] =
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
static const uint8_t mouseScanDataDisc[] =
{
  /*! device name */
  10,                                     /*! length */
  DM_ADV_TYPE_LOCAL_NAME,                 /*! AD type */
  'H',
  'I',
  'D',
  ' ',
  'M',
  'o',
  'u',
  's',
  'e'
};

/**************************************************************************************************
  Macros
**************************************************************************************************/

/*! WSF message event starting value */
#define MOUSE_MSG_START               0xA0

/*! WSF message event enumeration */
enum
{
  MOUSE_BATT_TIMER_IND = MOUSE_MSG_START,       /*! Battery timer expired */
};

/* Mouse button bit mask */
#define MOUSE_BUTTON_LEFT             (1<<0)
#define MOUSE_BUTTON_RIGHT            (1<<1)
#define MOUSE_BUTTON_MIDDLE           (1<<2)

/* Mouse input record message format */
#define MOUSE_BUTTON_POS              0
#define MOUSE_X_POS                   1
#define MOUSE_Y_POS                   2
#define MOUSE_INPUT_REPORT_LEN        3

/* Mouse TX path flags */
#define MOUSE_TX_FLAGS_READY          0x01
#define MOUSE_TX_FLAGS_PENDING        0x02

/* Mouse usage definitions */
#define MOUSE_USAGE_NONE              0x00
#define MOUSE_USAGE_UP_ARROW          0x52

/**************************************************************************************************
  Client Characteristic Configuration Descriptors
**************************************************************************************************/

/*! enumeration of client characteristic configuration descriptors */
enum
{
  MOUSE_GATT_SC_CCC_IDX,                    /*! GATT service, service changed characteristic */
  MOUSE_MBI_CCC_HDL,                        /*! MOUSE Boot Mouse Input characteristic */
  MOUSE_IR_CCC_HDL,                         /*! Input Report characteristic */
  MOUSE_BATT_LVL_CCC_IDX,                   /*! Battery service, battery level characteristic */
  MOUSE_NUM_CCC_IDX
};

/*! client characteristic configuration descriptors settings, indexed by above enumeration */
static const attsCccSet_t mouseCccSet[MOUSE_NUM_CCC_IDX] =
{
  /* cccd handle                      value range               security level */
  {GATT_SC_CH_CCC_HDL,                ATT_CLIENT_CFG_INDICATE,  DM_SEC_LEVEL_NONE},   /* MOUSE_GATT_SC_CCC_IDX */
  {HIDM_MOUSE_BOOT_IN_CH_CCC_HDL,     ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE},   /* MOUSE_MBI_CCC_HDL */
  {HIDM_INPUT_REPORT_CH_CCC_HDL,      ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE},   /* MOUSE_IR_CCC_HDL */
  {BATT_LVL_CH_CCC_HDL,               ATT_CLIENT_CFG_NOTIFY,    DM_SEC_LEVEL_NONE}    /* MOUSE_BATT_LVL_CCC_IDX */
};

/*! HID Report Type/ID and attribute handle map */
static const hidReportIdMap_t mouseReportIdSet[] =
{
  /* type                         ID             handle */
  {HID_REPORT_TYPE_INPUT,         0,             HIDM_INPUT_REPORT_HDL},     /* Input Report */
  {HID_REPORT_TYPE_INPUT,         HID_BOOT_ID,   HIDM_MOUSE_BOOT_IN_HDL},    /* Boot Mouse Input Report */
};

/*! Mouse Report Map (Descriptor) for generic mouses */
static const uint8_t mouseReportMap[] =
{
  0x05, 0x01,                    /* USAGE_PAGE (Generic Desktop) */
  0x09, 0x02,                    /* USAGE (Mouse) */
  0xa1, 0x01,                    /* COLLECTION (Application) */
  0x09, 0x01,                    /*   USAGE (Pointer) */
  0xa1, 0x00,                    /*   COLLECTION (Physical) */
  0x95, 0x03,                    /*     REPORT_COUNT (3) */
  0x75, 0x01,                    /*     REPORT_SIZE (1) */
  0x05, 0x09,                    /*     USAGE_PAGE (Button) */
  0x19, 0x01,                    /*     USAGE_MINIMUM (Button 1) */
  0x29, 0x03,                    /*     USAGE_MAXIMUM (Button 3) */
  0x15, 0x00,                    /*     LOGICAL_MINIMUM (0) */
  0x25, 0x01,                    /*     LOGICAL_MAXIMUM (1) */
  0x81, 0x02,                    /*     INPUT (Data, Variable, Absolute) */
  0x95, 0x01,                    /*     REPORT_COUNT (1) */
  0x75, 0x05,                    /*     REPORT_SIZE (5) */
  0x81, 0x01,                    /*     INPUT (Constant) */
  0x75, 0x08,                    /*     REPORT_SIZE (8) */
  0x95, 0x02,                    /*     REPORT_COUNT (2) */
  0x05, 0x01,                    /*     USAGE_PAGE (Generic Desktop) */
  0x09, 0x30,                    /*     USAGE (X) */
  0x09, 0x31,                    /*     USAGE (Y) */
  0x15, 0x81,                    /*     LOGICAL_MINIMUM (-127) */
  0x25, 0x7f,                    /*     LOGICAL_MAXIMUM (127) */
  0x81, 0x06,                    /*     INPUT (Data, Variable, Relative) */
  0xc0,                          /*   End Collection (Physical) */
  0xc0                           /* End Collection (Application) */
};

/*! HID Callback prototypes */
void mouseInfoCback(dmConnId_t connId, uint8_t type, uint8_t value);

/*! HID Profile Configuration */
static const hidConfig_t mouseHidConfig = 
{
  HID_DEVICE_TYPE_MOUSE,                              /* Type of HID device */
  (uint8_t*) mouseReportMap,                          /* Report Map */
  sizeof(mouseReportMap),                             /* Size of report map in bytes */
  (hidReportIdMap_t*) mouseReportIdSet,               /* Report ID to Attribute Handle map */
  sizeof(mouseReportIdSet)/sizeof(hidReportIdMap_t),  /* Size of Report ID to Attribute Handle map */
  NULL,                                               /* Output Report Callback */
  NULL,                                               /* Feature Report Callback */
  mouseInfoCback                                      /* Info Callback */
};

/**************************************************************************************************
  Local Variables
**************************************************************************************************/

/*! application control block */
struct
{
  wsfHandlerId_t handlerId;             /* task handle */
  uint8_t buttonMask;                   /* pending button mask */
  uint8_t xDisplacement;                /* pending X Displacement */
  uint8_t yDisplacement;                /* pending Y Displacement */
  uint8_t devSpecific;                  /* pending Device Specific */
  uint8_t txFlags;                      /* transmit flags */
  uint8_t protocolMode;                 /* current protocol mode */
  uint8_t hostSuspended;                /* TRUE if host suspended */
} mouseCb;

/*************************************************************************************************/
/*!
 *  \fn     mouseDmCback
 *        
 *  \brief  Application DM callback.
 *
 *  \param  pDmEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void mouseDmCback(dmEvt_t *pDmEvt)
{
  dmEvt_t   *pMsg;
  uint16_t  len;
  
  len = sizeof(dmEvt_t);

  if ((pMsg = WsfMsgAlloc(len)) != NULL)
  {
    memcpy(pMsg, pDmEvt, sizeof(dmEvt_t));
    WsfMsgSend(mouseCb.handlerId, pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     mouseAttCback
 *        
 *  \brief  Application  ATT callback.
 *
 *  \param  pEvt    ATT callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void mouseAttCback(attEvt_t *pEvt)
{
  attEvt_t *pMsg;
  
  if ((pMsg = WsfMsgAlloc(sizeof(attEvt_t) + pEvt->valueLen)) != NULL)
  {
    memcpy(pMsg, pEvt, sizeof(attEvt_t));
    pMsg->pValue = (uint8_t *) (pMsg + 1);
    memcpy(pMsg->pValue, pEvt->pValue, pEvt->valueLen);
    WsfMsgSend(mouseCb.handlerId, pMsg);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     mouseCccCback
 *        
 *  \brief  Application ATTS client characteristic configuration callback.
 *
 *  \param  pDmEvt  DM callback event
 *
 *  \return None.
 */
/*************************************************************************************************/
static void mouseCccCback(attsCccEvt_t *pEvt)
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
 *  \fn     mouseOpen
 *        
 *  \brief  Perform UI actions on connection open.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void mouseOpen(dmEvt_t *pMsg)
{
  HidSetProtocolMode(HID_PROTOCOL_MODE_REPORT);
}

/*************************************************************************************************/
/*!
 *  \fn     mouseSetup
 *        
 *  \brief  Set up procedures that need to be performed after device reset.
 *
 *  \param  pMsg    Pointer to DM callback event message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void mouseSetup(dmEvt_t *pMsg)
{
  /* set advertising and scan response data for discoverable mode */
  AppAdvSetData(APP_ADV_DATA_DISCOVERABLE, sizeof(mouseAdvDataDisc), (uint8_t *) mouseAdvDataDisc);
  AppAdvSetData(APP_SCAN_DATA_DISCOVERABLE, sizeof(mouseScanDataDisc), (uint8_t *) mouseScanDataDisc);

  /* set advertising and scan response data for connectable mode */
  AppAdvSetData(APP_ADV_DATA_CONNECTABLE, sizeof(mouseAdvDataDisc), (uint8_t *) mouseAdvDataDisc);
  AppAdvSetData(APP_SCAN_DATA_CONNECTABLE, sizeof(mouseScanDataDisc), (uint8_t *) mouseScanDataDisc);
  
  /* start advertising; automatically set connectable/discoverable mode and bondable mode */
  AppAdvStart(APP_MODE_AUTO_INIT);
}

/*************************************************************************************************/
/*!
 *  \fn     mouseSendData
 *        
 *  \brief  Send example data.
 *
 *  \param  connId    Connection identifier.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void mouseSendData(dmConnId_t connId)
{
  uint8_t cccHandle = MOUSE_IR_CCC_HDL;
  uint8_t protocolMode;

  protocolMode = HidGetProtocolMode();

  if (protocolMode == HID_PROTOCOL_MODE_BOOT)
  {
    cccHandle = MOUSE_MBI_CCC_HDL;
  }

  if (AttsCccEnabled(connId, cccHandle))
  {
    if (mouseCb.txFlags & MOUSE_TX_FLAGS_PENDING)
    {
      if (mouseCb.txFlags & MOUSE_TX_FLAGS_READY)
      {
        uint8_t buffer[MOUSE_INPUT_REPORT_LEN];
        uint8_t reportId = 0;
        
        /* mouse record: button mask, x displacement, y displacement, device specific */
        buffer[MOUSE_BUTTON_POS] = mouseCb.buttonMask;
        buffer[MOUSE_X_POS] = mouseCb.xDisplacement;
        buffer[MOUSE_Y_POS] = mouseCb.yDisplacement;

        mouseCb.txFlags &= ~(MOUSE_TX_FLAGS_READY | MOUSE_TX_FLAGS_PENDING);

        if (protocolMode == HID_PROTOCOL_MODE_BOOT)
        {
          reportId = HID_BOOT_ID;
        }

        /* Send the message */
        HidSendInputReport(connId, reportId, MOUSE_INPUT_REPORT_LEN, buffer);
      }
    }
  }
}

/*************************************************************************************************/
/*!
 *  \fn     MouseReportEvent
 *        
 *  \brief  Report or queue a mouse event to the host.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void MouseReportEvent(uint8_t buttonMask, uint8_t xDisplacement, uint8_t yDisplacement)
{
  dmConnId_t connId;

  if ((connId = AppConnIsOpen()) != DM_CONN_ID_NONE)
  {
    /* record mouse data */
    mouseCb.buttonMask = buttonMask;
    mouseCb.xDisplacement = xDisplacement;
    mouseCb.yDisplacement = yDisplacement;
    mouseCb.devSpecific = 0;

    /* Indicate new data is pending */
    mouseCb.txFlags |= MOUSE_TX_FLAGS_PENDING;
    
    /* send the data */
    mouseSendData(connId);
  }
}

/*************************************************************************************************/
/*!
 *  \fn     mouseBtnCback
 *        
 *  \brief  Button press callback.
 *
 *  \param  btn    Button press.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void mouseBtnCback(uint8_t btn)
{
  dmConnId_t      connId;
  
  /* button actions when connected */
  if ((connId = AppConnIsOpen()) != DM_CONN_ID_NONE)
  {
    switch (btn)
    {
      case APP_UI_BTN_2_DOWN:
        /* Send right button down */
        MouseReportEvent(MOUSE_BUTTON_RIGHT, 0, 0);
        break;

      case APP_UI_BTN_1_DOWN:
        /* Send left button down */
        MouseReportEvent(MOUSE_BUTTON_LEFT, 0, 0);
        break;

      case APP_UI_BTN_1_SHORT:            
      case APP_UI_BTN_1_MED:
      case APP_UI_BTN_1_LONG:
      case APP_UI_BTN_1_EX_LONG:
      case APP_UI_BTN_2_SHORT:            
      case APP_UI_BTN_2_MED:
      case APP_UI_BTN_2_LONG:
      case APP_UI_BTN_2_EX_LONG:
        /* Send no buttons down */
        MouseReportEvent(0, 0, 0);
        break;
        
      default:
        break;
    }    
  }
}

/*************************************************************************************************/
/*!
 *  \fn     mouseProcMsg
 *        
 *  \brief  Process messages from the event handler.
 *
 *  \param  pMsg    Pointer to message.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void mouseProcMsg(dmEvt_t *pMsg)
{
  uint8_t uiEvent = APP_UI_NONE;
  
  switch(pMsg->hdr.event)
  {
    case ATTS_HANDLE_VALUE_CNF:
      if (pMsg->hdr.status == ATT_SUCCESS)
      {
        mouseCb.txFlags |= MOUSE_TX_FLAGS_READY;
        mouseSendData((dmConnId_t) pMsg->hdr.param);
      }
      break;
    
    case MOUSE_BATT_TIMER_IND:
      BasProcMsg(&pMsg->hdr);
      break;

    case DM_RESET_CMPL_IND:
      mouseSetup(pMsg);
      uiEvent = APP_UI_RESET_CMPL;
      break;
      
    case DM_CONN_OPEN_IND:
      mouseOpen(pMsg);
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
 *  \fn     mouseInfoCback
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
static void mouseInfoCback(dmConnId_t connId, uint8_t type, uint8_t value)
{
  /* Record information from the host in the control block */
  if (type == HID_INFO_PROTOCOL_MODE)
  {
    mouseCb.protocolMode = value;
  }
  else if (type == HID_INFO_CONTROL_POINT)
  {
    mouseCb.hostSuspended = (value == HID_CONTROL_POINT_SUSPEND) ? TRUE : FALSE;
  }
}

/*************************************************************************************************/
/*!
 *  \fn     mouseReportInit
 *        
 *  \brief  Initialize the report attributes to default values for the mouse.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void mouseReportInit()
{
  uint8_t buffer[MOUSE_INPUT_REPORT_LEN];
  
  /* Input and boot input reports */
  memset(buffer, 0, sizeof(buffer));
  AttsSetAttr(HIDM_INPUT_REPORT_HDL, MOUSE_INPUT_REPORT_LEN, buffer);
  AttsSetAttr(HIDM_MOUSE_BOOT_IN_HDL, MOUSE_INPUT_REPORT_LEN, buffer);
}

/*************************************************************************************************/
/*!
 *  \fn     MouseHandlerInit
 *        
 *  \brief  Application handler init function called during system initialization.
 *
 *  \param  handlerID  WSF handler ID.
 *
 *  \return None.
 */
/*************************************************************************************************/
void MouseHandlerInit(wsfHandlerId_t handlerId)
{
  APP_TRACE_INFO0("MouseHandlerInit");

  /* Initialize the control block */
  memset(&mouseCb, 0, sizeof(mouseCb));
  mouseCb.txFlags = MOUSE_TX_FLAGS_READY;

  /* store handler ID */
  mouseCb.handlerId = handlerId;
    
  /* Set configuration pointers */
  pAppSlaveCfg = (appSlaveCfg_t *) &mouseSlaveCfg;
  pAppAdvCfg = (appAdvCfg_t *) &mouseAdvCfg;
  pAppSecCfg = (appSecCfg_t *) &mouseSecCfg;
  pAppUpdateCfg = (appUpdateCfg_t *) &mouseUpdateCfg;

  /* Initialize application framework */
  AppSlaveInit();

  /* initialize battery service server */
  BasInit(handlerId, (basCfg_t *) &mouseBasCfg);
}

/*************************************************************************************************/
/*!
 *  \fn     MouseHandler
 *        
 *  \brief  WSF event handler for application.
 *
 *  \param  event   WSF event mask.
 *  \param  pMsg    WSF message.
 *
 *  \return None.
 */
/*************************************************************************************************/
void MouseHandler(wsfEventMask_t event, wsfMsgHdr_t *pMsg)
{ 
  if (pMsg != NULL)
  {
    APP_TRACE_INFO1("Mouse got evt %d", pMsg->event);

    if (pMsg->event <= DM_CBACK_END)
    {
      /* process advertising and connection-related messages */
      AppSlaveProcDmMsg((dmEvt_t *) pMsg);
      
      /* process security-related messages */
      AppSlaveSecProcDmMsg((dmEvt_t *) pMsg);
    }
          
    /* perform profile and user interface-related operations */
    mouseProcMsg((dmEvt_t *) pMsg);    
  }
}

/*************************************************************************************************/
/*!
 *  \fn     MouseTest
 *        
 *  \brief  Test a mouse button event.
 *
 *  \return None.
 */
/*************************************************************************************************/
void MouseTest()
{
  mouseBtnCback(APP_UI_BTN_1_DOWN);
  mouseBtnCback(APP_UI_BTN_1_SHORT);
}

/*************************************************************************************************/
/*!
 *  \fn     MouseStart
 *        
 *  \brief  Start the application.
 *
 *  \return None.
 */
/*************************************************************************************************/
void MouseStart(void)
{  
  /* Register for stack callbacks */
  DmRegister(mouseDmCback);
  DmConnRegister(DM_CLIENT_ID_APP, mouseDmCback);
  AttRegister(mouseAttCback);
  AttConnRegister(AppServerConnCback);
  AttsCccRegister(MOUSE_NUM_CCC_IDX, (attsCccSet_t *) mouseCccSet, mouseCccCback);

  /* Register for app framework button callbacks */
  AppUiBtnRegister(mouseBtnCback);
 
  /* Initialize the HID service */
  SvcHidMouseAddGroup();
  HidInit(&mouseHidConfig);

  /* Initialize attribute server database */
  SvcCoreAddGroup();
  SvcDisAddGroup();
  SvcBattCbackRegister(BasReadCback, NULL);
  SvcBattAddGroup();

  /* Initialize the report attributes */
  mouseReportInit();

  /* Reset the device */
  DmDevReset();  
}
