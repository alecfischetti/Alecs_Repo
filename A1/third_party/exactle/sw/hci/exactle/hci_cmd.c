/*************************************************************************************************/
/*!
 *  \file   hci_cmd.c
 *
 *  \brief  HCI command module.
 *
 *          $Date: 2015-06-17 19:13:24 -0700 (Wed, 17 Jun 2015) $
 *          $Revision: 3141 $
 *
 *  Copyright (c) 2009 Wicentric, Inc., all rights reserved.
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
#include "wsf_queue.h"
#include "wsf_timer.h"
#include "wsf_msg.h"
#include "wsf_trace.h"
#include "bstream.h"
#include "hci_cmd.h"
#include "hci_tr.h"
#include "hci_api.h"
#include "hci_main.h"

#include "ll_api.h"

/*************************************************************************************************/
/*!
 *  \fn     HciDisconnectCmd
 *
 *  \brief  HCI disconnect command.
 */
/*************************************************************************************************/
void HciDisconnectCmd(uint16_t handle, uint8_t reason)
{
  LlDisconnect(handle, reason);
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeAddDevWhiteListCmd
 *
 *  \brief  HCI LE add device white list command.
 */
/*************************************************************************************************/
void HciLeAddDevWhiteListCmd(uint8_t addrType, uint8_t *pAddr)
{
  LlAddDeviceToWhitelist(addrType, pAddr);
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeClearWhiteListCmd
 *
 *  \brief  HCI LE clear white list command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeClearWhiteListCmd(void)
{
  LlClearWhitelist();
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeConnUpdateCmd
 *
 *  \brief  HCI connection update command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeConnUpdateCmd(uint16_t handle, hciConnSpec_t *pConnSpec)
{
  LlConnUpdate(handle, (llConnSpec_t *)pConnSpec);
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeCreateConnCmd
 *
 *  \brief  HCI LE create connection command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeCreateConnCmd(uint16_t scanInterval, uint16_t scanWindow, uint8_t filterPolicy,
                        uint8_t peerAddrType, uint8_t *pPeerAddr, uint8_t ownAddrType,
                        hciConnSpec_t *pConnSpec)
{
  LlCreateConn(scanInterval,
               scanWindow,
               filterPolicy,
               peerAddrType,
               pPeerAddr,
               ownAddrType,
               (llConnSpec_t *)pConnSpec);
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeCreateConnCancelCmd
 *
 *  \brief  HCI LE create connection cancel command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeCreateConnCancelCmd(void)
{
  LlCreateConnCancel();
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeRandCmd
 *
 *  \brief  HCI LE random command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeRandCmd(void)
{
  hciLeRandCmdCmplEvt_t evt;

  evt.hdr.param = 0;
  evt.hdr.event = HCI_LE_RAND_CMD_CMPL_CBACK_EVT;
  evt.hdr.status = HCI_SUCCESS;

  evt.status = HCI_SUCCESS;

  LlGetRandNum(evt.randNum);

  hciCb.secCback((hciEvt_t *)&evt);
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeReadAdvTXPowerCmd
 *
 *  \brief  HCI LE read advertising TX power command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeReadAdvTXPowerCmd(void)
{
  /* unused */
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeReadBufSizeCmd
 *
 *  \brief  HCI LE read buffer size command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeReadBufSizeCmd(void)
{
  /* unused */
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeReadChanMapCmd
 *
 *  \brief  HCI LE read channel map command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeReadChanMapCmd(uint16_t handle)
{
  hciReadChanMapCmdCmplEvt_t evt;

  evt.hdr.param = handle;
  evt.hdr.event = HCI_LE_READ_CHAN_MAP_CMD_CMPL_CBACK_EVT;
  evt.hdr.status = LlGetChannelMap(handle, evt.chanMap);

  evt.handle = handle;
  evt.status = evt.hdr.status;

  hciCb.evtCback((hciEvt_t *)&evt);
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeReadLocalSupFeatCmd
 *
 *  \brief  HCI LE read local supported feautre command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeReadLocalSupFeatCmd(void)
{
  /* unused */
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeReadRemoteFeatCmd
 *
 *  \brief  HCI LE read remote feature command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeReadRemoteFeatCmd(uint16_t handle)
{
  LlReadRemoteFeat(handle);
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeReadSupStatesCmd
 *
 *  \brief  HCI LE read supported states command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeReadSupStatesCmd(void)
{
  /* unused */
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeReadWhiteListSizeCmd
 *
 *  \brief  HCI LE read white list size command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeReadWhiteListSizeCmd(void)
{
  /* unused */
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeRemoveDevWhiteListCmd
 *
 *  \brief  HCI LE remove device white list command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeRemoveDevWhiteListCmd(uint8_t addrType, uint8_t *pAddr)
{
  LlRemoveDeviceFromWhitelist(addrType, pAddr);
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeSetAdvEnableCmd
 *
 *  \brief  HCI LE set advanced enable command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeSetAdvEnableCmd(uint8_t enable)
{
  LlAdvEnable(enable);
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeSetAdvDataCmd
 *
 *  \brief  HCI LE set advertising data command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeSetAdvDataCmd(uint8_t len, uint8_t *pData)
{
  LlSetAdvData(len, pData);
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeSetAdvParamCmd
 *
 *  \brief  HCI LE set advertising parameters command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeSetAdvParamCmd(uint16_t advIntervalMin, uint16_t advIntervalMax, uint8_t advType,
                         uint8_t ownAddrType, uint8_t directAddrType, uint8_t *pDirectAddr,
                         uint8_t advChanMap, uint8_t advFiltPolicy)
{
  LlSetAdvParam(advIntervalMin,
                advIntervalMax,
                advType,
                ownAddrType,
                directAddrType,
                pDirectAddr,
                advChanMap,
                advFiltPolicy);
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeSetEventMaskCmd
 *
 *  \brief  HCI LE set event mask command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeSetEventMaskCmd(uint8_t *pLeEventMask)
{
  /* unused */
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeSetHostChanClassCmd
 *
 *  \brief  HCI set host channel class command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeSetHostChanClassCmd(uint8_t *pChanMap)
{
  /* unused */
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeSetRandAddrCmd
 *
 *  \brief  HCI LE set random address command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeSetRandAddrCmd(uint8_t *pAddr)
{
  LlSetRandAddr(pAddr);
}

/*************************************************************************************************/
/*!
 *  \fn     HciLeSetScanRespDataCmd
 *
 *  \brief  HCI LE set scan response data.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciLeSetScanRespDataCmd(uint8_t len, uint8_t *pData)
{
  LlSetScanRespData(len, pData);
}

/*************************************************************************************************/
/*!
 *  \fn     HciReadBdAddrCmd
 *
 *  \brief  HCI read BD address command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciReadBdAddrCmd(void)
{
  /* not used */
}

/*************************************************************************************************/
/*!
 *  \fn     HciReadBufSizeCmd
 *
 *  \brief  HCI read buffer size command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciReadBufSizeCmd(void)
{
  /* not used */
}

/*************************************************************************************************/
/*!
 *  \fn     HciReadLocalSupFeatCmd
 *
 *  \brief  HCI read local supported feature command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciReadLocalSupFeatCmd(void)
{
  /* not used */
}

/*************************************************************************************************/
/*!
 *  \fn     HciReadLocalVerInfoCmd
 *
 *  \brief  HCI read local version info command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciReadLocalVerInfoCmd(void)
{
  /* unused */
}

/*************************************************************************************************/
/*!
 *  \fn     HciReadRemoteVerInfoCmd
 *
 *  \brief  HCI read remote version info command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciReadRemoteVerInfoCmd(uint16_t handle)
{
  LlReadRemoteVerInfo(handle);
}

/*************************************************************************************************/
/*!
 *  \fn     HciReadRssiCmd
 *
 *  \brief  HCI read RSSI command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciReadRssiCmd(uint16_t handle)
{
  hciReadRssiCmdCmplEvt_t evt;

  evt.hdr.param = handle;
  evt.hdr.event = HCI_READ_RSSI_CMD_CMPL_CBACK_EVT;
  evt.hdr.status = LlGetRssi(handle, &evt.rssi);

  evt.handle = handle;
  evt.status = evt.hdr.status;

  hciCb.evtCback((hciEvt_t *)&evt);
}

/*************************************************************************************************/
/*!
 *  \fn     HciReadTxPwrLvlCmd
 *
 *  \brief  HCI read Tx power level command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciReadTxPwrLvlCmd(uint16_t handle, uint8_t type)
{
  hciReadTxPwrLvlCmdCmplEvt_t evt;

  evt.hdr.param = handle;
  evt.hdr.event = HCI_READ_TX_PWR_LVL_CMD_CMPL_CBACK_EVT;
  evt.hdr.status = LlGetTxPowerLevel(handle, type, &evt.pwrLvl);

  evt.handle = handle;
  evt.status = evt.hdr.status;

  hciCb.evtCback((hciEvt_t *)&evt);
}

/*************************************************************************************************/
/*!
 *  \fn     HciResetCmd
 *
 *  \brief  HCI reset command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciResetCmd(void)
{
  LlReset();
}

/*************************************************************************************************/
/*!
 *  \fn     HciSetEventMaskCmd
 *
 *  \brief  HCI set event mask command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciSetEventMaskCmd(uint8_t *pEventMask)
{
  /* unused */
}

/*************************************************************************************************/
/*!
 *  \fn     HciVendorSpecificCmd
 *
 *  \brief  HCI vencor specific command.
 *
 *  \return None.
 */
/*************************************************************************************************/
void HciVendorSpecificCmd(uint16_t opcode, uint8_t len, uint8_t *pData)
{
  /* not used */
}
