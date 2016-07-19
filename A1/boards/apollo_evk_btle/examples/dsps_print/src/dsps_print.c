//*****************************************************************************
//
//! @file dsps_print.c
//!
//! @brief Simple example to print UART data over BLE.
//!
//! This example uses the Dialog Serial Port Service Profile to send UART data
//! to a mobile device over a Bluetooth Low Energy connection.
//!
//! Note: No ITM debug printing takes place in this example.
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
#include "am_devices.h"
#include "am_bsp.h"
#include "am_util.h"

#include "dsps_image.h"

//*****************************************************************************
//
// UART buffers for the radio.
//
//*****************************************************************************
uint8_t g_pui8TxArray[1024];
uint8_t g_pui8RxArray[1024];

//*****************************************************************************
//
// Flags to alert the main application of various interrupt conditions.
//
//*****************************************************************************
volatile bool g_bUARTIdle = true;

//*****************************************************************************
//
// Configuration structure for the UART
//
//*****************************************************************************
am_hal_uart_config_t g_sUartConfig =
{
    .ui32BaudRate = 112000,
    .ui32DataBits = AM_HAL_UART_DATA_BITS_8,
    .bTwoStopBits = false,
    .ui32Parity = AM_HAL_UART_PARITY_NONE,
    .ui32FlowCtrl = AM_HAL_UART_FLOW_CTRL_RTS_CTS
};

//*****************************************************************************
//
// Configure the necessary pins for communication with the radio.
//
//*****************************************************************************
void
configure_radio_pins(void)
{
    //
    // Enable the necessary pins for talking to the radio.
    //
    am_bsp_pin_enable(UART_TX);
    am_bsp_pin_enable(UART_RX);
    am_bsp_pin_enable(UART_RTS);
    am_bsp_pin_enable(UART_CTS);
    am_bsp_pin_enable(DA14581_POWER);
    am_bsp_pin_enable(DA14581_RESET);
}

//*****************************************************************************
//
// Configure the necessary pins for communication with the radio.
//
//*****************************************************************************
void
configure_radio_uart(void)
{
    //
    // Enable the clock to the UART.
    //
    am_hal_uart_clock_enable();

    //
    // Disable and configure the UART.
    //
    am_hal_uart_disable();
    am_hal_uart_config(&g_sUartConfig);

    //
    // Enable the UART.
    //
    am_hal_uart_enable();
}

//*****************************************************************************
//
// Configure the necessary pins for communication with the radio.
//
//*****************************************************************************
void
radio_uart_enable(void)
{
    //
    // Set the global variable to indicate that the UART is on.
    //
    g_bUARTIdle = false;

    //
    // Enable the UART pins.
    //
    am_bsp_pin_enable(UART_TX);
    am_bsp_pin_enable(UART_RX);
    am_bsp_pin_enable(UART_RTS);
    am_bsp_pin_enable(UART_CTS);

    //
    // Enable the clock to the UART.
    //
    am_hal_uart_clock_enable();
    am_hal_uart_config(&g_sUartConfig);
    am_hal_uart_enable();
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_UART);
    am_hal_uart_int_enable(AM_HAL_UART_INT_TX);
}

//*****************************************************************************
//
// Configure the necessary pins for communication with the radio.
//
//*****************************************************************************
void
radio_uart_disable(void)
{
    //
    // Set the global variable to indicate that the UART is off.
    //
    g_bUARTIdle = true;

    //
    // Wait for the UART lines to stop toggling, and make sure we have
    // absolutely no interrupts pending.
    //
    while (am_hal_uart_flags_get() & AM_REG_UART_FR_BUSY_BUSY);
    am_hal_uart_int_clear(0xFFFFFFFF);

    //
    // Disable the UART pins.
    //
    am_bsp_pin_disable(UART_TX);
    am_bsp_pin_disable(UART_RX);
    am_bsp_pin_disable(UART_RTS);
    am_bsp_pin_disable(UART_CTS);

    //
    // Disable the clock to the UART.
    //
    am_hal_uart_disable();
    am_hal_uart_clock_disable();
}

//*****************************************************************************
//
// Run the dialog bootloader.
//
//*****************************************************************************
void
boot_radio(void)
{
    uint32_t ui32NumTries, ui32Status;

    //
    // Turn on the radio, and hold it in RESET.
    //
    am_hal_gpio_out_bit_set(AM_BSP_GPIO_DA14581_POWER);

    //
    // Try to boot the radio. If it doesn't work the first few times, give up.
    //
    for (ui32NumTries = 0; ui32NumTries < 10; ui32NumTries++)
    {
        //
        // Pulse the reset line.
        //
        am_hal_gpio_out_bit_set(AM_BSP_GPIO_DA14581_RESET);
        am_util_delay_us(100);
        am_hal_gpio_out_bit_clear(AM_BSP_GPIO_DA14581_RESET);
        am_bsp_pin_disable(DA14581_RESET);

        //
        // Run the boot algorithm.
        //
        ui32Status = am_devices_da14581_uart_boot(g_pui8DspsImage, DSPS_IMAGE_LENGTH);

        //
        // If we succeeded, we can stop.
        //
        if (ui32Status == 0)
        {
            return;
        }
    }

    //
    // If we weren't able to boot the radio, halt here for debugging.
    //
    while (1);
}

//*****************************************************************************
//
// Sets up a button interface.
//
//*****************************************************************************
void
setup_buttons(void)
{
    //
    // Enable the buttons for user interaction.
    //
    am_devices_button_array_init(am_bsp_psButtons, AM_BSP_NUM_BUTTONS);

    //
    // Enable the crystal oscillator.
    //
    am_hal_clkgen_osc_start(AM_HAL_CLKGEN_OSC_XT);

    //
    // Configure a timer to run the button debouncer from the crystal
    // oscillator.
    //
    am_hal_ctimer_config_single(0, AM_HAL_CTIMER_TIMERA,
                                (AM_HAL_CTIMER_FN_REPEAT |
                                 AM_HAL_CTIMER_XT_2_048KHZ |
                                 AM_HAL_CTIMER_INT_ENABLE));

    //
    // Set up initial timer period.
    //
    am_hal_ctimer_period_set(0, AM_HAL_CTIMER_TIMERA, 20, 0);

    //
    // Enable interrupts for Timer A0.
    //
    am_hal_ctimer_int_enable(AM_HAL_CTIMER_INT_TIMERA0);
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_CTIMER);

    //
    // Start the timer.
    //
    am_hal_ctimer_start(0, AM_HAL_CTIMER_TIMERA);
}

//*****************************************************************************
//
// Interrupt handler for the UART
//
//*****************************************************************************
void
am_uart_isr(void)
{
    uint32_t ui32Status;

    //
    // Read the masked interrupt status from the UART.
    //
    ui32Status = am_hal_uart_int_status_get(true);

    //
    // Service the buffered UART.
    //
    am_hal_uart_service_buffered(ui32Status);

    //
    // Clear the UART interrupts.
    //
    am_hal_uart_int_clear(ui32Status);

    //
    // If the transmit fifo is empty, wait until the UART isn't busy anymore,
    // and then shut it off.
    //
    if (am_hal_uart_flags_get() & AM_REG_UART_FR_TXFE_XMTFIFO_EMPTY)
    {
        radio_uart_disable();
    }
}

//*****************************************************************************
//
// Interrupt handler for the timers.
//
//*****************************************************************************
void
am_ctimer_isr(void)
{
    am_hal_ctimer_int_clear(AM_HAL_CTIMER_INT_TIMERA0);

    //
    // Run the sampling/debouncing routine on the buttons.
    //
    am_devices_button_array_tick(am_bsp_psButtons, AM_BSP_NUM_BUTTONS);

    //
    // If button zero was pressed, print a message to the DSPS interface.
    //
    if (am_devices_button_released(am_bsp_psButtons[0]))
    {
        //
        // Make sure the radio is on.
        //
        radio_uart_enable();

        //
        // Print a quick message.
        //
        am_util_stdio_printf("Hello World!!!");
    }
}

//*****************************************************************************
//
// Main.
//
//*****************************************************************************
int
main(void)
{
    //
    // Set the system clock to 24 MHz.
    //
    am_hal_clkgen_sysclk_select(AM_HAL_CLKGEN_SYSCLK_24MHZ);

    //
    // Enable internal buck converters and turn off the voltage comparator.
    //
    am_bsp_low_power_init();

    //
    // Enable the LEDs for debugging purposes, and turn on LED0.
    //
    am_devices_led_array_init(am_bsp_psLEDs, AM_BSP_NUM_LEDS);
    am_devices_led_on(am_bsp_psLEDs, 0);

    //
    // Configure peripherals for talking with the radio.
    //
    configure_radio_pins();
    configure_radio_uart();

    //
    // Boot the radio, and prepare for buffered UART operation.
    //
    boot_radio();

    am_hal_uart_init_buffered(g_pui8RxArray, 1024, g_pui8TxArray, 1024);
    am_util_stdio_printf_init((am_util_stdio_print_char_t)
                              am_hal_uart_string_transmit_buffered);

    //
    // Shut down the UART until we need it again.
    //
    radio_uart_disable();

    //
    // Enable the button interface.
    //
    setup_buttons();

    //
    // Loop forever, waiting for events.
    //
    while (1)
    {
        //
        // Disable interrupts temporarily. We need to make sure that the global
        // flags aren't changing while we are checking them.
        //
        am_hal_interrupt_master_disable();

        //
        // Turn off the indicator LED before we go to sleep mode.
        //
        am_devices_led_off(am_bsp_psLEDs, 0);

        //
        // Pick a sleep mode. If the UART is active, we will need to wake up
        // again very soon, so we will use normal sleep mode. Otherwise, we
        // should use deep sleep.
        //
        if (g_bUARTIdle)
        {
            am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
        }
        else
        {
            am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);
        }

        //
        // Turn the indicator LED on as we exit sleep.
        //
        am_devices_led_on(am_bsp_psLEDs, 0);

        //
        // Re-enable interrupts so we can handle any new events that have come
        // in.
        //
        am_hal_interrupt_master_enable();
    }
}
