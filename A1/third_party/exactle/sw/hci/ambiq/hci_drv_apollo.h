//*****************************************************************************
//
//! @file hci_drv_apollo.h
//!
//! @brief Additional header information for the Apollo implementation of HCI.
//
//*****************************************************************************

#ifndef HCI_DRV_APOLLO_H
#define HCI_DRV_APOLLO_H

#ifdef __cplusplus
extern "C" 
{
#endif

//*****************************************************************************
//
// Function prototypes.
//
//*****************************************************************************
extern void HciDrvUartEnable(void);
extern void HciDrvUartDisable(void);
extern void HciDrvUartFlowOff(void);
extern void HciDrvUartFlowOn(void);
extern void HciDrvUartPause(void);
extern void HciDrvUartUnpause(void);
extern bool HciDrvUartSafeShutdown(void);
extern void HciDrvRadioBoot(void);

#ifdef __cplusplus
};
#endif

#endif // HCI_DRV_APOLLO_H
