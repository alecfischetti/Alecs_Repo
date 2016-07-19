//*****************************************************************************
//
//! @file hci_apollo_config.h
//!
//! @brief This file describes the physical aspects of the HCI conection.
//
//*****************************************************************************

#include <stdint.h>
#include "am_bsp.h"

#ifndef HCI_APOLLO_CONFIG_H
#define HCI_APOLLO_CONFIG_H

//*****************************************************************************
//
// Pin numbers and configuration.
//
// NOTE: RTS, CTS, and RESET are implemented as GPIOs, so no "CFG" field is
// needed.
//
//*****************************************************************************
#define HCI_APOLLO_UART_TX_PIN          AM_BSP_GPIO_UART_TX
#define HCI_APOLLO_UART_TX_CFG          AM_BSP_GPIO_CFG_UART_TX

#define HCI_APOLLO_UART_RX_PIN          AM_BSP_GPIO_UART_RX
#define HCI_APOLLO_UART_RX_CFG          AM_BSP_GPIO_CFG_UART_RX

#define HCI_APOLLO_UART_RTS_PIN         AM_BSP_GPIO_UART_RTS

#define HCI_APOLLO_UART_CTS_PIN         AM_BSP_GPIO_UART_CTS

#define HCI_APOLLO_POWER_PIN            AM_BSP_GPIO_DA14581_POWER
#define HCI_APOLLO_POWER_CFG            AM_BSP_GPIO_CFG_DA14581_POWER

#define HCI_APOLLO_RESET_PIN            AM_BSP_GPIO_DA14581_RESET

//*****************************************************************************
//
// Other options.
//
// These options are provided in case your board setup is a little more
// unusual. Most boards shouldn't need these features. If in doubt, leave all
// of these features disabled.
//
//*****************************************************************************
#define HCI_APOLLO_CFG_OVERRIDE_ISR         0 // Override the exactle UART ISR

#endif // HCI_APOLLO_CONFIG_H
