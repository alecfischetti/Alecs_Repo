//*****************************************************************************
//
//! @file hci_drv.c
//!
//! @brief HCI driver interface.
//
//*****************************************************************************

//*****************************************************************************
//
// Copyright (c) 2016, Ambiq Micro
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is part of revision 1.1.0 of the AmbiqSuite Development Package.
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>

#include "wsf_types.h"
#include "wsf_msg.h"
#include "wsf_cs.h"
#include "hci_drv.h"
#include "hci_drv_apollo.h"
#include "hci_tr_apollo.h"
#include "hci_core.h"

#include "am_mcu_apollo.h"
#include "am_util.h"
#include "am_devices_da14581.h"
#include "da14581_hci_image.h"

#include "hci_apollo_config.h"

#include <string.h>

volatile uint8_t g_ui8Paused = 0;

//*****************************************************************************
//
// Enable the UART for HCI.
//
//*****************************************************************************
void
HciDrvUartEnable(void)
{
    //
    // If the UART is already on, don't do anything.
    //
    if (AM_REG(CLKGEN, UARTEN))
    {
        return;
    }

    //
    // Enable the UART pins.
    //
    am_hal_gpio_pin_config(0, AM_HAL_PIN_0_UARTTX);
    am_hal_gpio_pin_config(1, AM_HAL_PIN_1_UARTRX);

    //
    // Enable the main clock to the UART module.
    //
    am_hal_uart_clock_enable();

    //
    // Enable the UART physical interface.
    //
    am_hal_uart_enable();
}

//*****************************************************************************
//
// Disable the UART for HCI
//
//*****************************************************************************
void
HciDrvUartDisable(void)
{
    //
    // If the UART is already off, don't do anything.
    //
    if (AM_REG(CLKGEN, UARTEN) == 0)
    {
        return;
    }

    //
    // Disable the UART clock in the UART module, then disable it in the main
    // clock tree register.
    //
    am_hal_uart_disable();
    am_hal_uart_clock_disable();

    //
    // Disable the UART pins.
    //
    am_hal_gpio_pin_config(0, AM_HAL_PIN_DISABLE);
    am_hal_gpio_pin_config(1, AM_HAL_PIN_DISABLE);
}

//*****************************************************************************
//
// Sets the RTS signal to the Dialog radio to stop incoming HCI traffic.
//
// This function also enables a GPIO interrupt on the Apollo's RTS pin. If the
// Dialog radio needs to send a message, it can toggle this pin to notify the
// Apollo that it should enable its UART.
//
//*****************************************************************************
void
HciDrvUartFlowOff(void)
{
    //
    // Raise RTS.
    //
    am_hal_gpio_out_bit_set(HCI_APOLLO_UART_RTS_PIN);

    //
    // Enable the CTS interrupt.
    //
    am_hal_gpio_int_polarity_bit_set(HCI_APOLLO_UART_CTS_PIN, AM_HAL_GPIO_FALLING);
    am_hal_gpio_int_clear(AM_HAL_GPIO_BIT(HCI_APOLLO_UART_CTS_PIN));
    am_hal_gpio_int_enable(AM_HAL_GPIO_BIT(HCI_APOLLO_UART_CTS_PIN));
}

//*****************************************************************************
//
// Clears the RTS signal to the Dialog radio to stop incoming HCI traffic.
//
// This function also disables the gpio "wakeup" interrupt on the RTS pin.
//
//*****************************************************************************
void
HciDrvUartFlowOn(void)
{
    //
    // Clear and disable the CTS interrupt.
    //
    am_hal_gpio_int_disable(AM_HAL_GPIO_BIT(HCI_APOLLO_UART_CTS_PIN));
    am_hal_gpio_int_clear(AM_HAL_GPIO_BIT(HCI_APOLLO_UART_CTS_PIN));

    //
    // Assert RTS, but only if we're not currently paused.
    //
    if (g_ui8Paused == 0)
    {
        am_hal_gpio_out_bit_clear(HCI_APOLLO_UART_RTS_PIN);
    }
}

//*****************************************************************************
//
// Raise RTS to stop the Dialog radio from sending more data.
//
//*****************************************************************************
void
HciDrvUartPause(void)
{
    //
    // Increment the pause count.
    //
    g_ui8Paused++;

    //
    // If we're the first call to "pause" we need to actually manipulate RTS.
    //
    if (g_ui8Paused == 1)
    {
        //
        // Raise RTS.
        //
        am_hal_gpio_out_bit_set(HCI_APOLLO_UART_RTS_PIN);
    }
}

//*****************************************************************************
//
// Restore the previous setting of RTS so that the Dialog may send data again.
//
//*****************************************************************************
void
HciDrvUartUnpause(void)
{
    //
    // Decrement the pause count.
    //
    g_ui8Paused = g_ui8Paused > 0 ? g_ui8Paused - 1 : 0;

    //
    // If no one else has the UART paused, we can restore RTS to its normal
    // state.
    //
    if (g_ui8Paused == 0)
    {
        //
        // If the UART is supposed to be enabled, assert RTS. Otherwise, we can
        // just leave it off.
        //
        if (AM_REG(CLKGEN, UARTEN))
        {
            am_hal_gpio_out_bit_clear(HCI_APOLLO_UART_RTS_PIN);
        }
    }
}

//*****************************************************************************
//
// Attempt to shut down the UART connection to the Dialog radio. If successful,
// return true. Otherwise, return false. If this function returns true, it
// should be safe to go to deep sleep.
//
//*****************************************************************************
bool
HciDrvUartSafeShutdown(void)
{
    uint64_t ui64Status;

    WSF_CS_INIT(cs);
    WSF_CS_ENTER(cs);
    //
    // If the UART is already off, we don't need to do anything. The UART is already off!
    //
    // Note: We don't manipulate RTS/CTS or the interrupt configuration here
    // because we're assuming that the UART was shut down by this function, and
    // that they should already be in the correct state.
    //
    // This means you should never shut down the UART except by calling this function.
    //
    if (AM_REG(CLKGEN, UARTEN) == 0)
    {
        WSF_CS_EXIT(cs);
        return true;
    }

    //
    // If we're still using the UART, either for sending or receiving, we should leave it on.
    //
    if (AM_BFR(UART, FR, BUSY) || hciTrReceivingPacket())
    {
        WSF_CS_EXIT(cs);
        return false;
    }

    //
    // Clear the interrupt status on the CTS pin so we can see if Dialog
    // attempts to assert it over the next byte time or so.
    //
    am_hal_gpio_int_polarity_bit_set(HCI_APOLLO_UART_CTS_PIN, AM_HAL_GPIO_FALLING);
    am_hal_gpio_int_clear(AM_HAL_GPIO_BIT(HCI_APOLLO_UART_CTS_PIN));
    am_hal_gpio_int_enable(AM_HAL_GPIO_BIT(HCI_APOLLO_UART_CTS_PIN));

    //
    // Delay here because the Dialog might be trying to send us a byte. If we
    // release RTS too early, we can catch a corner case where the Dialog has
    // already decided to send, but its hardware flow control prevents even the
    // first byte from coming out. This delay prevents that. Instead we'll just
    // get the byte here, and see it in our FIFO before this function ends.
    //
    am_util_delay_us(100);

    //
    // Check to see if Dialog asserted CTS while we were actually waiting for a
    // byte.
    //
    ui64Status = am_hal_gpio_int_status_get(false);

    //
    // If the fifo is not empty, then we need to keep the UART on. Also, we now
    // know that we're receiving a packet.
    //
    if (!AM_BFR(UART, FR, RXFE))
    {
        //
        // ... then we need to keep the UART on, and we know we're receiving a packet.
        //
        HciDrvUartEnable();
        HciDrvUartFlowOn();
        WSF_CS_EXIT(cs);
        return false;
    }

    //
    // If we saw an edge on CTS, we still need to keep the UART on, because
    // Dialog might be trying to send to us.
    //
    if ((ui64Status & AM_HAL_GPIO_BIT(HCI_APOLLO_UART_CTS_PIN)) &&
        !am_hal_gpio_input_bit_read(HCI_APOLLO_UART_CTS_PIN))
    {
        HciDrvUartEnable();
        HciDrvUartFlowOn();
        WSF_CS_EXIT(cs);
        return false;
    }

    //
    // Raise RTS to stop Dialog from talking to us.
    //
    am_hal_gpio_out_bit_set(HCI_APOLLO_UART_RTS_PIN);

    //
    // Delay again, because there's still a way to get a byte in the FIFO. If
    // Dialog asserts RTS and starts sending right before Apollo's RTS
    // deasserts, it'll end up in the FIFO right here. If we don't delay, we
    // will lose those bytes.
    //
    // Note: We could definitely miss a falling edge on CTS here. That's okay,
    // because the interrupt will still go to the NVIC, and we'll handle the
    // cleanup in the GPIO interrupt handler.
    //
    am_util_delay_us(100);

    //
    // Check for that byte we might have gotten.
    //
    if (!AM_BFR(UART, FR, RXFE))
    {
        //
        // If we got it, we have to leave the UART on, and declare that we have a new packet.
        //
        HciDrvUartEnable();
        HciDrvUartFlowOn();
        WSF_CS_EXIT(cs);
        return false;
    }

    //
    // If we manage to get here, we're done. Shut the UART down.
    //
    HciDrvUartDisable();
    WSF_CS_EXIT(cs);
    return true;
}

//*****************************************************************************
//
//! @brief Write data the driver.
//! 
//! @param type HCI packet type
//! @param len Number of bytes to write
//! @param pData Byte array to write
//!
//! @return Returns the number of bytes written.
//
//*****************************************************************************
uint16_t 
hciDrvWrite(uint8_t type, uint16_t len, uint8_t *pData)
{
    uint16_t ui16Count = 0;
    bool bTypeSent = false;
    bool bSuccess = false;

    //
    // Set RTS. If the UART wasn't on yet, this won't have any noticeable
    // effect on the line (because RTS will already be high). If the UART
    // was on though, this sets us up to generate a fresh falling edge on
    // RTS in case we need one. This will ensure that the Dialog device has
    // a reason to wake up.
    //
    am_hal_gpio_out_bit_set(HCI_APOLLO_UART_RTS_PIN);
    am_util_delay_us(150);

    while (1)
    {
        WSF_CS_INIT(cs);
        WSF_CS_ENTER(cs);

        //
        // Do a quick check to see if the Dialog is already awake. If it is,
        // there's a chance it could start sending us a message immediately.
        // That means that we'll need to have our UART ready before we lower
        // RTS again.
        //
        if (!am_hal_gpio_input_bit_read(HCI_APOLLO_UART_CTS_PIN))
        {
            //
            // Enable the UART before we lower RTS
            //
            HciDrvUartEnable();
            HciDrvUartFlowOn();
            bSuccess = true;
        }
        else
        {
            //
            // Briefly enable the UART and lower RTS. This will give the
            // Dialog a signal to wake up.
            //
            HciDrvUartEnable();
            HciDrvUartFlowOn();

            //
            // Immediately start a shutdown. If this shutdown succeeds, we'll go
            // directly to deep sleep. If it fails, the Dialog should be awake.
            //
            if (HciDrvUartSafeShutdown()) 
            {
                //
                // The dialog might have woken up during the shutdown. If not,
                // we should go to sleep now, and the interrupt will wake us
                // up.
                //
                if (am_hal_gpio_input_bit_read(HCI_APOLLO_UART_CTS_PIN))
                {
                    //
                    // Lowering RTS (even though our UART is off) because Dialog
                    // might miss the interrupt from the previous transition.
                    // We shouldn't have to do this. Also, it introduces a race
                    // condition.
                    //
                    am_hal_gpio_out_bit_clear(HCI_APOLLO_UART_RTS_PIN);

                    am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
                }
                else
                {
                    //
                    // If the dialog did wake up during the shutdown, we don't
                    // need to bother going to sleep.
                    //
                    bSuccess = true;
                }

                //
                // The Dialog is awake. Turn the UART back on.
                //
                HciDrvUartEnable();
                HciDrvUartFlowOn();

            }
            else
            {
                //
                // If the shutdown failed, then both our UART and the Dialog
                // UART are on. Go ahead and send the message.
                //
                bSuccess = true;
            }
        }

        //
        // If we succeeded, send the type packet before we exit the critical
        // section. Otherwise, just exit the critical section and continue the
        // loop.
        //
        if (bSuccess)
        {
            am_hal_uart_char_transmit_polled(type);
            WSF_CS_EXIT(cs);
            break;
        }
        else
        {
            WSF_CS_EXIT(cs);
        }
    }

    //
    // Start sending bytes.
    //
    while (ui16Count < len)
    {
        //
        // Check the fifo fullness.
        //
        if(AM_BFRn(UART, 0, FR, TXFF))
        {
            //
            // If it was full, sleep until we can reasonably expect it to be
            // empty.
            //
            am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);
        }
        else
        {
            //
            // If it wasn't full, we should put our data in it.
            //
            AM_REGn(UART, 0, DR) = *pData++;
            ui16Count++;
        }
    }

    return ui16Count;
}

//*****************************************************************************
//
// hciDrvReadyToSleep - Stub provided to allow other layers to run correctly.
//
//*****************************************************************************
bool_t
hciDrvReadyToSleep(void)
{
    return TRUE;
}

//*****************************************************************************
//
// Configure the necessary pins and start the Dialog radio.
//
//*****************************************************************************
void
HciDrvRadioBoot(void)
{
    //
    // Enable the radio pins.
    //
    am_hal_gpio_pin_config(HCI_APOLLO_POWER_PIN, HCI_APOLLO_POWER_CFG);
    am_hal_gpio_out_bit_set(HCI_APOLLO_POWER_PIN);

    //
    // Assert RESET to the Dialog device.
    //
    am_hal_gpio_pin_config(HCI_APOLLO_RESET_PIN, AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_out_bit_set(HCI_APOLLO_RESET_PIN);

    //
    // Configure the UART pins.
    //
    // CAJ: Adding a pullup to the RX pin for boot-up. This works around an
    // issue where the Apollo doesn't recognize the boot signal from the
    // Dialog, but I'm not sure why it works.
    //
    am_hal_gpio_pin_config(HCI_APOLLO_UART_TX_PIN, HCI_APOLLO_UART_TX_CFG);
    am_hal_gpio_pin_config(HCI_APOLLO_UART_RX_PIN, HCI_APOLLO_UART_RX_CFG | AM_HAL_GPIO_PULLUP);
    am_hal_gpio_pin_config(HCI_APOLLO_UART_RTS_PIN, AM_HAL_PIN_OUTPUT);
    am_hal_gpio_pin_config(HCI_APOLLO_UART_CTS_PIN, AM_HAL_PIN_INPUT);

    am_hal_uart_config_t sUartConfig;
    sUartConfig.ui32BaudRate = 112000;
    sUartConfig.ui32DataBits = AM_HAL_UART_DATA_BITS_8;
    sUartConfig.bTwoStopBits = false;
    sUartConfig.ui32Parity = AM_HAL_UART_PARITY_NONE;
    sUartConfig.ui32FlowCtrl = AM_HAL_UART_FLOW_CTRL_NONE;

    //
    // Enable UART FIFO operation.
    //
    am_hal_uart_clock_enable();
    am_hal_uart_disable();
    am_hal_uart_config(&sUartConfig);
    am_hal_uart_enable();

    am_hal_uart_fifo_config(AM_HAL_UART_TX_FIFO_1_2 | AM_HAL_UART_RX_FIFO_1_2);

    //
    // Force RTS low
    //
    am_hal_gpio_pin_config(HCI_APOLLO_UART_RTS_PIN, AM_HAL_PIN_OUTPUT);
    am_hal_gpio_out_bit_clear(HCI_APOLLO_UART_RTS_PIN);

    //
    // Delay a bit.
    //
    am_util_delay_ms(10);

    //
    // Deassert RESET to the Dialog device.
    //
    am_hal_gpio_out_bit_clear(HCI_APOLLO_RESET_PIN);
    am_hal_gpio_pin_config(HCI_APOLLO_RESET_PIN, AM_HAL_PIN_DISABLE);

    //
    // Transmit the Dialog firmware image across the PMOD UART port.
    //
    am_devices_da14581_uart_boot(g_pui8Da14581HciImage, DA14581_HCI_IMAGE_LENGTH);

    //
    // CAJ: Removing the pullup function from the RX pin. This is part of the
    // bootup workaround mentioned during pin configuration.
    //
    am_hal_gpio_pin_config(HCI_APOLLO_UART_RX_PIN, HCI_APOLLO_UART_RX_CFG);

    sUartConfig.ui32BaudRate = 250000;
    sUartConfig.ui32DataBits = AM_HAL_UART_DATA_BITS_8;
    sUartConfig.bTwoStopBits = false;
    sUartConfig.ui32Parity = AM_HAL_UART_PARITY_NONE;
    sUartConfig.ui32FlowCtrl = AM_HAL_UART_FLOW_CTRL_NONE;

    am_hal_uart_disable();
    am_hal_uart_config(&sUartConfig);
    am_hal_uart_enable();

    //
    // Clear any UART errors that may have come up in the reboot process.
    //
    uint32_t ui32Status = AM_REG(UART, IES);
    AM_REG(UART, IES) = ui32Status;

    //
    // Enable the UART and RX timeout interrupt.
    //
    AM_REGn(UART, 0, IER) |= (AM_REG_UART_IES_RTRIS_M |
                              AM_REG_UART_IES_RXRIS_M |
                              AM_REG_UART_IES_TXRIS_M);

    //
    // Enable the UART and GPIO interrupt handlers.
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_UART);
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_GPIO);

    //
    // Delay for 3 seconds to make sure the dialog goes to sleep.
    //
    am_util_delay_ms(20);
}


#if (HCI_APOLLO_CFG_OVERRIDE_ISR == 0)
//*****************************************************************************
//
// UART interrupt handler.
//
//*****************************************************************************
void
am_uart_isr(void)
{
    uint32_t ui32Status;

    //
    // Read and save the interrupt status, but clear out the status register.
    //
    ui32Status = AM_REGn(UART, 0, MIS);
    AM_REGn(UART, 0, IEC) = ui32Status;

    //
    // Check to see if we have filled the Rx FIFO past the configured limit, or
    // if we have an 'old' character or two sitting in the FIFO.
    //
    if (ui32Status & (AM_REG_UART_IES_RXRIS_M | AM_REG_UART_IES_RTRIS_M))
    {
        //
        // While there's stuff in the RX fifo....
        //
        while (!AM_BFRn(UART, 0, FR, RXFE))
        {
            //
            // Read a byte from the FIFO.
            //
            uint32_t ui32Char = AM_REGn(UART, 0, DR);

            //
            // If none of the error bits are set (stored in the bits just above
            // the data byte), send the byte along to the HCI layer.
            //
            if ((ui32Char & 0x700) == 0)
            {
                //
                // Pass it along to the HCI driver.
                //
                hciTrSerialRxIncoming((uint8_t *)(&ui32Char), 1);
            }
        }
    }
}
#endif
