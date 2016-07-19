//*****************************************************************************
//
//! @file ios_hub.c
//!
//! @brief An example showing the use of the IO Slave in a message protocol
//!
//! This is the main program. Refer to iosi2c_hub_driver for the details of the
//! actions an Android or Nucleus kernel driver has to do to talk to the Apollo
//! slave for sensor hub applications. Please see the iomi2c_host_side application
//! for a host example that can be used with this example.
//!
//
//  PIN Fly Lead Assumptions for the I/O Slave (IOS) Board (this one):
//         GPIO[0] == IOS I2C SCK
//         GPIO[1] == IOS I2C SDA
//         GPIO[4] == GPIO Interrupt from slave board
//
//  PIN Fly Lead Assumptions for the I/O Master (IOM) Board (the other one):
//         GPIO[5] == IOM0 I2C SCK
//         GPIO[6] == IOM0 I2C SDA
//         GPIO[2] == GPIO Interrupt from slave board
//
//
// We will use the Apollo I/O slave as a shared memory mail box. Apollo can
// signal the host of a message to read with a GPIO pin. The host can signal
// Apollo that it has received the message by writing anything to location 0x0.
// The access interrupt is enabled for this byte so that any host writes to
// this shared location will cause an interrupt to the Apollo.
//
// In a similar way, the host first writes a message in the provided buffer.
// The host then writes the length of the message to location 0x1.
//
//
// Layout of Apollo Slave Directly Addressable 120 Byte Register Space
//             ---------------------------------
//  0x77(119) |                                 |
//            |     unused                      |
//  0x6B(107) |                                 |
//             ---------------------------------
//  0x6A(106) |                                 |
//            |                                 |
//            |                                 |
//            |                                 |
//            |  Apollo to Host Message (64B)   |
//            |                                 |
//            |                                 |
//  0x26(38)  |                                 |
//             ---------------------------------
//  0x25(37)  |                                 |
//            |                                 |
//            |                                 |
//            |  Host to Apollo Message (32B)   |
//            |                                 |
//            |                                 |
//  0x2 (2)   |                                 |
//             ---------------------------------
//            |                                 |
//  0x1 (1)   | Host To Apollo Write Msg Length |  IOS Access Interrupt Enabled
//             ---------------------------------
//            |                                 |
//  0x0 (0)   | Host To Apollo Read Msg (H2A)   |  IOS Access Interrupt Enabled
//             ---------------------------------
//
// The 4 byte header attached to each message is completely arbitrary and is
// provided here as an example of one to convey the size and target of a message.
//
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
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "iosi2c_hub_driver.h"

//*****************************************************************************
//
// IO Slave configuration structure.
//
//*****************************************************************************
am_hal_ios_config_t g_sIOSConfig =
{
    //
    // Select IOS interface.
    //
    .ui32InterfaceSelect = (AM_HAL_IOS_USE_I2C | AM_HAL_IOS_I2C_ADDRESS(0x94)),

    //
    // Set up the Direct read section from 0x0 to 0x74, and use the rest for
    // FIFO space.
    //
    .ui32ROBase = IOSI2C_HUB_DRIVER_FIFO_BASE,
    .ui32FIFOBase = IOSI2C_HUB_DRIVER_FIFO_BASE,
    .ui32RAMBase = IOSI2C_HUB_DRIVER_FIFO_END,

    //
    // Set the FIFO threshold low, so we wake up when the read FIFO is drained.
    //
    .ui32FIFOThreshold = 1
};

//*****************************************************************************
//
// Global receive buffer for messages coming from the host.
//
//*****************************************************************************
#define H2S_BUFFER_SIZE (80)
uint8_t g_ui8H2SBuffer[H2S_BUFFER_SIZE];

//*****************************************************************************
//
// Global extracted message from receive buffer.
//
//*****************************************************************************
iosi2c_hub_driver_message_t g_sReceivedMessage;
uint32_t g_ui32MessageReceived = 0;

//*****************************************************************************
//
// Global slave to host message buffer.
//
//*****************************************************************************
iosi2c_hub_driver_message_t g_sS2HMessage;
#define S2H_BUFFER_SIZE (80)
uint8_t g_ui8S2HBuffer[S2H_BUFFER_SIZE];
uint32_t g_ui32MessageXmitBufferReleased = 0;

//*****************************************************************************
//
// IO Slave Register Access ISR.
//
//*****************************************************************************
void
am_ioslave_acc_isr(void)
{
    uint32_t ui32Status;

    //
    // Check to see what caused this interrupt, then clear the bit from the
    // interrupt register.
    //
    ui32Status = am_hal_ios_access_int_status_get(false);
    am_hal_ios_access_int_clear(ui32Status);

    //
    // Check for message from host.
    //
    if (ui32Status & AM_HAL_IOS_ACCESS_INT_01)
    {
        //
        //  Host has sent us a message.
        //

        //
        // Service any active register-access interrupts based on the STHP
        // protocol. Note this will cause an interrupt back to the HOST on
        // GPIO[4].
        //
        iosi2c_hub_driver_acc_service(g_ui8H2SBuffer, ui32Status);

        //
        // Extract the received message so the base level can use it.
        //
        iosi2c_hub_driver_message_extract(&g_sReceivedMessage, g_ui8H2SBuffer);

        //
        // Signal the base level that a message has been received.
        //
        g_ui32MessageReceived++;
    }

    //
    // Check for host releasing their inbound buffer.
    //
    if (ui32Status & AM_HAL_IOS_ACCESS_INT_00)
    {
        //
        // Host is releasing their inbound buffer so clear GPIO for next message
        // There is a race here with the arrival of a new host message.
        // This needs to be cleared before we try to echo the message back.
        //
        am_hal_gpio_out_bit_clear(4);

        //
        // Signal the base level that the S2H message buffer is released.
        //
        g_ui32MessageXmitBufferReleased++;
    }
}

//*****************************************************************************
//
// Configure GPIOs for this example.
//
//*****************************************************************************
void
configure_pins(void)
{
    //
    // Configure I/O Slave as I2C.
    //
    am_hal_gpio_pin_config(0, AM_HAL_PIN_0_SLSCL);
    am_hal_gpio_pin_config(1, AM_HAL_PIN_1_SLSDA);

    //
    // Configure The I/O Slave Interrupt Pin.
    //
    am_hal_gpio_pin_config(4, AM_HAL_PIN_OUTPUT);
    am_hal_gpio_out_bit_clear(4);

    //
    // Enable an LED so we can see whether we're awake or asleep.
    //
    am_hal_gpio_pin_config(AM_BSP_GPIO_LED0, AM_HAL_PIN_OUTPUT);
    am_hal_gpio_out_bit_clear(AM_BSP_GPIO_LED0);
}

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int
main(void)
{
    uint32_t ui32MessageReceived = 0;
    uint32_t ui32MessageXmitBufferReleased = 0;
    uint32_t ui32I;

    //
    // Set the system clock to 24 MHz.
    //
    am_hal_clkgen_sysclk_select(AM_HAL_CLKGEN_SYSCLK_24MHZ);

    //
    // Set up the board using the default configurations in the BSP.
    //
    am_bsp_low_power_init();

    //
    // Enable the RTC.
    //
    am_hal_rtc_osc_enable();

    //
    // Initialize the printf interface for polled ITM.
    //
    am_util_stdio_printf_init((am_util_stdio_print_char_t)
                              am_hal_itm_print);

    //
    // Enable the SWO pin.
    //
    am_bsp_pin_enable(ITM_SWO);

    //
    // Enable the ITM.
    //
    am_hal_itm_enable();
    am_bsp_debug_printf_enable();

    //
    // Print the banner.
    //
    am_util_stdio_printf("        \r\n");
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("I/O Slave I2C Hub Example\r\n");

    //
    // Setup the pins for IO Slave Example.
    //
    configure_pins();

    //
    // Initialize the I2C IOS protocol.
    //
    iosi2c_hub_driver_init(&g_sIOSConfig);

    //
    // Loop forever.
    //
    while (1)
    {
        //
        // Check for incoming messages to the slave.
        //
        if (g_ui32MessageReceived != ui32MessageReceived)
        {
            //
            // Note the arrival of the message.
            //
            ui32MessageReceived = g_ui32MessageReceived;

            //
            // Print the message.
            //
            am_util_stdio_printf("Message Received Length = 0x%x (%d) ",
                                 g_sReceivedMessage.Length,
                                 g_sReceivedMessage.Length);
            am_util_stdio_printf("Channel # = 0x%x (%d) ",
                                 g_sReceivedMessage.ChannelNumber,
                                 g_sReceivedMessage.ChannelNumber);
            am_util_stdio_printf("Sequence # = 0x%x (%d) ",
                                 g_sReceivedMessage.SequenceNumber,
                                 g_sReceivedMessage.SequenceNumber);
            //
            // Print the message payload.
            //
            {
                for (ui32I = 0; ui32I < g_sReceivedMessage.Length; ui32I++)
                {
                  am_util_stdio_printf("0x%x ", g_sReceivedMessage.Message[ui32I]);
                }
                am_util_stdio_printf("\r\n");
            }

            am_util_stdio_printf(" <%s>\r\n", &g_ui8H2SBuffer[4]);

            //
            // Now echo it back on channel + 1 with sequence number + 1.
            //
            g_sReceivedMessage.ChannelNumber++;
            g_sReceivedMessage.SequenceNumber++;

            //
            // Format it.
            //
            iosi2c_hub_driver_message_format(
                      &g_sReceivedMessage, g_ui8S2HBuffer);

            //
            // Copy to IOS LRAM in preparation for sending it to the host.
            //
            iosi2c_hub_driver_write(g_ui8S2HBuffer, g_sReceivedMessage.Length);

            //
            // Notify Host of Inbound Message by driving GPIO interrupt
            // source high.
            //
            am_hal_gpio_out_bit_set(4);
        }

        if (g_ui32MessageXmitBufferReleased != ui32MessageXmitBufferReleased)
        {
            //
            // Note the arrival of the message.
            //
            ui32MessageXmitBufferReleased = g_ui32MessageXmitBufferReleased;
        }

        //
        // Disable the TPIU so we can safely enter deep sleep.
        //
        am_bsp_debug_printf_disable();

        //
        // Sleep until the next interrupt comes along.
        //
        am_hal_gpio_out_bit_set(AM_BSP_GPIO_LED0);
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
        am_hal_gpio_out_bit_clear(AM_BSP_GPIO_LED0);

        //
        // Re-enable the TPIU so we can print messages over ITM.
        //
        am_bsp_debug_printf_enable();
    }
}
