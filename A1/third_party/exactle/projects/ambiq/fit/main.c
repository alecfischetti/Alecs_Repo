/*************************************************************************************************/
/*!
 *  \file   main.c
 *
 *  \brief  Main module.
 *
 *          $Date: 2014-12-17 16:08:07 -0800 (Wed, 17 Dec 2014) $
 *          $Revision: 2125 $
 *
 *  Copyright (c) 2013 Wicentric, Inc., all rights reserved.
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

#include "wsf_types.h"
#include "wsf_trace.h"

#include "board_apollo.h"
#include "hci_handler.h"
#include "dm_handler.h"
#include "l2c_handler.h"
#include "att_handler.h"
#include "smp_handler.h"
#include "l2c_api.h"
#include "att_api.h"
#include "smp_api.h"
#include "app_api.h"
#include "fit_api.h"
#include "hci_core.h"
#include "hci_drv.h"

#include "apollo.h"
#include "am_bsp.h"

#include "fit_api.h"
#include "app_ui.h"

/*************************************************************************************************/
/*!
 *  \fn     mainStackInit
 *
 *  \brief  Initialize stack.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*************************************************************************************************/
static void mainStackInit(void)
{
  wsfHandlerId_t handlerId;

  handlerId = WsfOsSetNextHandler(HciHandler);
  HciHandlerInit(handlerId);

  handlerId = WsfOsSetNextHandler(DmHandler);
  DmAdvInit();
  DmConnInit();
  DmConnSlaveInit();
  DmSecInit();
  DmHandlerInit(handlerId);

  handlerId = WsfOsSetNextHandler(L2cSlaveHandler);
  L2cSlaveHandlerInit(handlerId);
  L2cInit();
  L2cSlaveInit();

  handlerId = WsfOsSetNextHandler(AttHandler);
  AttHandlerInit(handlerId);
  AttsInit();
  AttsIndInit();
  AttcInit();

  handlerId = WsfOsSetNextHandler(SmpHandler);
  SmpHandlerInit(handlerId);
  SmprInit();

  handlerId = WsfOsSetNextHandler(AppHandler);
  AppHandlerInit(handlerId);

  handlerId = WsfOsSetNextHandler(FitHandler);
  FitHandlerInit(handlerId);
}

/*************************************************************************************************/
/*!
 *  \fn     main
 *
 *  \brief  Entry point for demo software.
 *
 *  \param  None.
 *
 *  \return None.
 */
/*************************************************************************************************/
int main(void)
{
  BoardHwInit();
  BoardPlatformInit();
  mainStackInit();

  FitStart();

  while (TRUE)
  {
    BoardTimerService();
    wsfOsDispatcher();
    BoardTimerSetWakeup();

#if WSF_TOKEN_ENABLED == TRUE
    WsfTokenService();
#endif

    am_hal_interrupt_master_disable();

    if (wsfOsReadyToSleep())
    {
      am_bsp_led_off(0);
      __asm volatile ("wfi");
      am_bsp_led_on(0);
    }

    am_hal_ctimer_stop(0, AM_HAL_CTIMER_TIMERA);

    am_hal_interrupt_master_enable();
  }
}
