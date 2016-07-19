//*****************************************************************************
//
//! @file iomi2c_host_side.c
//!
//! @brief An example to drive the IO Slave on a second board while mimicking
//! an android or nucleus driver.
//!
//! This is the main program. Refer to iomi2c_host_side_driver for the details
//! of the actions an Android or Nucleus kernel driver has to do to talk to the
//! Apollo slave for sensor hub applications.
//!
//! This application simply provides stimulus for the driver code in
//! iomi2c_host_side_driver.c. This examples passes messages to the slave board
//! where they are echoed back.
//!
//! See the i2cios_hub application as an example slave device to talk with.
//
//  PIN Fly Lead Assumptions for the I/O Master (IOM) Board (this one):
//         GPIO[4] == GPIO Interrupt from slave board
//         GPIO[5] == IOM0 I2C SCK
//         GPIO[6] == IOM0 I2C SDA
//
//  PIN Fly Lead Assumptions for the I/O Slave (IOS) Board (the other one):
//         GPIO[0] == IOS I2C SCK
//         GPIO[1] == IOS I2C SDA
//         GPIO[4] == GPIO Interrupt from slave board
//
// We will use the Apollo I/O slave as a shared memory mail box. Apollo can
// signal the host of a message to read with a GPIO pin.  The host can signal
// Apollo that it has received the message by writing anything to location 0x0.
// The access interrupt is enabled for this byte so that any host writes to
// this shared location will cause an interrupt to the Apollo.
//
// In a similar way, the host first writes a message in the provided buffer.
//
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
//            |  Apollo to Host Message (68B)   |
//            |  This is a 64 byte message      |
//            |  + a 4 byte header.             |
//            |                                 |
//  0x26(38)  |                                 |
//             ---------------------------------
//  0x25(37)  |                                 |
//            |                                 |
//            |                                 |
//            |  Host to Apollo Message (36B)   |
//            |  This is a 32 byte message      |
//            |  + a 4 byte header.             |
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
// provided here as an example of one to convey the size and target of a
// message.
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
// This is part of revision 0.0.36-r1.0 of the AmbiqSuite Development Package.
//
//*****************************************************************************
#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "iomi2c_host_side_driver.h"


#define MY_INTERRUPT_PIN 4

//*****************************************************************************
//
// Global message buffer for the IO master.
//
//*****************************************************************************
am_hal_iom_buffer(256) g_psTxBuffer;
am_hal_iom_buffer(256) g_psRxBuffer;

//*****************************************************************************
//
// Global message for slave (H2S).
//
//*****************************************************************************
iomi2c_host_side_driver_message_t g_sH2SMessage;
uint32_t g_ui32IntCount = 0;
uint32_t g_ui32IntCountAccepted = 0;

//*****************************************************************************
//
// Global slave to host message (S2H).
//
//*****************************************************************************
iomi2c_host_side_driver_message_t g_sS2HMessage;

//*****************************************************************************
//
// Configuration structure for the IO Master.
//
//*****************************************************************************
const am_hal_iom_config_t g_sIOMConfig =
{
    .ui32InterfaceMode = AM_HAL_IOM_I2CMODE,
    .ui32ClockFrequency = AM_HAL_IOM_100KHZ,
    .bSPHA = 0,
    .bSPOL = 0,
    .ui8WriteThreshold = 0,
    .ui8ReadThreshold = 60,
};

//*****************************************************************************
//
// Configure GPIOs for this example.
//
//*****************************************************************************
void
configure_pins(void)
{
    //
    // Configure I/O Master 0 as I2C.
    //
    am_bsp_iom_i2c_pins_enable(0);

    //
    // Configure The I/O Slave Interrupt Pin.
    //
    am_hal_gpio_pin_config(MY_INTERRUPT_PIN, AM_HAL_PIN_INPUT);


    //
    // Enable a GPIO interrupt for positive edges.
    //
    am_hal_gpio_int_clear(AM_HAL_GPIO_BIT(MY_INTERRUPT_PIN));
    am_hal_gpio_int_enable(AM_HAL_GPIO_BIT(MY_INTERRUPT_PIN));

    //
    // Enable an LED so we can see whether we're awake or asleep.
    //
    am_hal_gpio_pin_config(AM_BSP_GPIO_LED0, AM_HAL_PIN_OUTPUT);
    am_hal_gpio_out_bit_clear(AM_BSP_GPIO_LED0);
}

//*****************************************************************************
//
// Interrupt handler for the GPIO pins.
//
//*****************************************************************************
void
am_gpio_isr(void)
{
    uint64_t ui64Status;

    //
    // Read and clear the GPIO interrupt status.
    //
    ui64Status = am_hal_gpio_int_status_get(false);
    am_hal_gpio_int_clear(ui64Status);

    //
    // Check to make sure that this is the correct interrupt, and set a flag to
    // alert the base-level application that the ADXL has data.
    //
    if (ui64Status & AM_HAL_GPIO_BIT(MY_INTERRUPT_PIN))
    {
        //
        // Signal the base level on a single reader single writer semaphore.
        //
        g_ui32IntCount++;
    }
}

//*****************************************************************************
//
// Don't drag in C lib.
//
//*****************************************************************************
static void
my_strcpy(uint8_t *ui8pDst, uint8_t *ui8pSrc)
{
    while (*ui8pSrc)
    {
        *ui8pDst++ = *ui8pSrc++;
    }
}

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int
main(void)
{
    uint32_t ui32I;

    //
    // Initialize the globals.
    //
    g_ui32IntCount = 0;
    g_ui32IntCountAccepted = 0;

    g_sH2SMessage.Length = 27 + 4;      // length includes 4 bytes of header
    g_sH2SMessage.ChannelNumber = 45;   // starting
    g_sH2SMessage.SequenceNumber = 123; // starting

    //
    // 27 test characters final X will be over written by the length.
    //
    my_strcpy(g_sH2SMessage.Message,
           (uint8_t *)"The quick brown fox jumped.");

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
    // Initialize the printf interface for ITM/SWO output.
    //
    am_util_stdio_printf_init((am_util_stdio_print_char_t) am_hal_itm_print);

    //
    // Initialize the SWO GPIO pin
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
    am_util_stdio_printf("I/O Master side of I2C Connection Example\r\n");
    am_util_stdio_printf("MSG <%s>\n\r", g_sH2SMessage.Message);

    //
    // Setup the pins for IO Master Example.
    //
    configure_pins();

    //
    // Initialize IOM 0 in I2C mode at 100KHz.
    //
    am_hal_iom_config(0, &g_sIOMConfig);
    am_hal_iom_enable(0);

    //
    // Initialize the I2C IOM protocol.
    //
    iomi2c_host_side_driver_init();

    //
    // Enable the necessary interrupts. (IOM is completely polled).
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_GPIO);
    am_hal_interrupt_master_enable();

    //
    // Throw out the first ball by sending a message to the slave.
    //
    iomi2c_host_side_driver_xmit_packet(&g_sH2SMessage);

    //
    // Prepare to send next host message.
    //
    g_sH2SMessage.ChannelNumber++;
    g_sH2SMessage.SequenceNumber++;

    //
    // Loop forever.
    //
    while (1)
    {
        //
        // First check to see if we have a response available in the slave.
        // GPIO Interrupt detected go read the IRQ status register in the
        // slave.
        //
        if (g_ui32IntCount != g_ui32IntCountAccepted)
        {
            //
            // Get ready for next one (single reader single writer semaphore).
            //
            g_ui32IntCountAccepted++;

            //
            // Read the interrupt status register in the I/O slave.
            //
            iomi2c_host_side_driver_rcv_packet(&g_sS2HMessage);
            am_util_stdio_printf("\r\nSlave Response Length = %d Channel = %d"
                                 " Seq = %d\n\r",
                                 g_sS2HMessage.Length,
                                 g_sS2HMessage.ChannelNumber,
                                 g_sS2HMessage.SequenceNumber);
            am_util_stdio_printf("\t<%s>\r\n", g_sS2HMessage.Message);
            for (ui32I = 0; ui32I < g_sS2HMessage.Length; ui32I++)
            {
               am_util_stdio_printf("0x%x ", g_sS2HMessage.Message[ui32I]);
            }
            am_util_stdio_printf("\r\n");

            //
            // Tell the slave the buffer is free.
            //
            iomi2c_host_side_driver_release_slave_buffer();

            //
            // Send the previously prepared massage.
            //
            iomi2c_host_side_driver_xmit_packet(&g_sH2SMessage);

            //
            // Prepare to send next host message.
            //
            g_sH2SMessage.ChannelNumber++;
            g_sH2SMessage.SequenceNumber++;
        }
    }
}

