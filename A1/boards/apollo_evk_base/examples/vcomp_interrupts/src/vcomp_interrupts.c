//*****************************************************************************
//
//! @file vcomp_interrupts.c
//!
//! @brief Example to initialize the voltage comparator, then track its state.
//!
//! This example initializes the voltage comparator, enables its interrupts
//! then captures transition interrupts and signal them to the base level
//! via a very simplistic method where it then reports the transitions on the
//! printf stream.
//!
//! When connected to a suitable RC network on the control pins, this example
//! creates a relaxation oscillator. The Apollo EVK base board contains one
//! example of a relaxation oscillator circuit. One can monitor the voltage
//! waveform on pin 18 to see the relaxation oscillator charge/discharge cycle.
//!
//! The RXO pins are defined in the BSP for this example.
//!
//! This example assumes the voltage comparator EXT2 on pin 18 is used for
//! the comparator input. If EXT1 is used, then make suitable changes below.
//!
//! This example turns on the following additional functions:
//!     1. The voltage comparator
//!     2. Voltage comparator RXO pin (GPIO output)
//!     3. Voltage comparator RXO_CMP pin (analog input pin)
//!     4. ITM/SWO output pin
//!     5. 3 LED pins
//!     6. The bandgap
//!
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
#include "am_devices.h"

//*****************************************************************************
//
// Global Variables
//
//*****************************************************************************
volatile bool g_bVCOMPState;                // < 1 --> comparator high
volatile uint32_t g_ui32VCOMPCount;         // < semaphore ISR --> BASE

//*****************************************************************************
//
// Voltage Comparator Configuration Structure
//
// These values will be passed to the voltage comparator configuration
// function.
//
//*****************************************************************************
const am_hal_vcomp_config_t g_sVCOMP_CfgA =
{
    // This macro selects the VCOMP DAC reference voltage.
    AM_HAL_VCOMP_DAC_LVLSEL_2_71V,

    // This macro select the VCOMP "positive" comparator input channel
    AM_HAL_VCOMP_PSEL_VEXT2,

    // This macro selects the VCOMP "negative" comparator input channel
    AM_HAL_VCOMP_NSEL_DAC_LEVEL
};

//*****************************************************************************
//
//  VCOMP Interrupt Service Routine (ISR)
//
//  Waits for the expected rising or falling slope to trigger a voltage
//  comparator interrupt. It signals the base level via the g_ui32VCOMPCount
//  single writer single reader semaphore. It tells the base level which
//  state was detected on the VCOMP input in the global variable g_bVCOMPState.
//
//*****************************************************************************
void
am_vcomp_isr(void)
{
    uint32_t ulVcompIntStatus;

    //
    // Publish the comparator state globally.
    //
    g_bVCOMPState = am_hal_vcomp_read();

    //
    // Signal the base level (single reader/single writer counting semaphore).
    // This test is unlikely to run long enough for this 32-bit counter to roll
    // over.
    //
    g_ui32VCOMPCount++;

    //
    // Toggle LED 1.
    //
    am_devices_led_toggle(am_bsp_psLEDs, 1);

    //
    // Get the VCOMP Interrupt Cause.
    //
    ulVcompIntStatus = am_hal_vcomp_int_status_get(false);

    //
    // Clear any interrupts (whatever they are).
    //
    am_hal_vcomp_int_clear(ulVcompIntStatus);

    //
    // Change expectation from going down to going up or vice versa.
    //
    if (g_bVCOMPState)
    {
        //
        // Set DAC target to low voltage range.
        //
        am_hal_vcomp_dac_level_set(AM_HAL_VCOMP_DAC_LVLSEL_0_97V);

        //
        // Set interrupt to wait for it to go below LOW threshold.
        //
        am_hal_vcomp_int_enable(AM_HAL_VCOMP_INT_OUTLO);

        //
        // Set GPIO driver for the relaxation oscillator LOW.
        //
        am_hal_gpio_out_bit_clear(AM_BSP_GPIO_VCOMP_RXO);

        //
        // Turn ON LED 3.
        //
        am_devices_led_on(am_bsp_psLEDs, 3);
    }
    else
    {
        //
        // Set DAC target to high voltage range.
        //
        am_hal_vcomp_dac_level_set(AM_HAL_VCOMP_DAC_LVLSEL_2_32V);

        //
        // Set interrupt to wait for it to go above HIGH threshold.
        //
        am_hal_vcomp_int_enable(AM_HAL_VCOMP_INT_OUTHI);

        //
        // Set GPIO driver for the relaxation oscillator HIGH.
        //
        am_hal_gpio_out_bit_set(AM_BSP_GPIO_VCOMP_RXO);

        //
        // Turn OFF LED 3.
        //
        am_devices_led_off(am_bsp_psLEDs, 3);
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
    uint32_t ui32Count;

    //
    // Set the clock frequency
    //
    am_hal_clkgen_sysclk_select(AM_HAL_CLKGEN_SYSCLK_24MHZ);

    //
    // Set up the board using the default configurations in the BSP.
    //
    am_bsp_low_power_init();

    //
    // Initialize the GPIO pins we need for this example.
    //
    am_bsp_pin_enable(VCOMP_RXO);
    am_bsp_pin_enable(VCOMP_RXO_CMP);

    //
    // Initialize the LED pins too.
    //
    am_devices_led_array_init(am_bsp_psLEDs, AM_BSP_NUM_LEDS);

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

    //
    // Enable debug printf messages using ITM on SWO pin
    //
    am_bsp_debug_printf_enable();

    //
    // Clear the terminal screen, and print a quick message to show that we're
    // alive.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("VCOMP Interrupt Example.\r\n");

    //
    // Start by driving the relaxation oscillator GPIO output pin low.
    //
    am_hal_gpio_out_bit_clear(AM_BSP_GPIO_VCOMP_RXO);

    //
    // Clear out the ISR signaling counters.
    //
    g_ui32VCOMPCount = 0;
    ui32Count = 0;

    //
    // Turn on the bandgap (used by VCOMP).
    //
    am_hal_mcuctrl_bandgap_enable();

    //
    // Configure the VCOMP.
    //
    am_hal_vcomp_config(&g_sVCOMP_CfgA);

    //
    // Turn on the analog power to the VCOMP.
    //
    am_hal_vcomp_enable();

    //
    // Set up for going to high level interrupt.
    //
    am_hal_vcomp_dac_level_set(AM_HAL_VCOMP_DAC_LVLSEL_0_77V);

    //
    // Wait for it to go low before starting the RXO.
    //
    while ( !am_hal_vcomp_read() );

    //
    // Announce that we initially pulled the GPIO low.
    //
    am_util_stdio_printf("VCOMP input below 0.77 volts.\r\n");

    //
    // Clear Interrupts in the VCOMP.
    //
    am_hal_vcomp_int_clear(0xFFFFFFFF);

    //
    // Start the RXO by driving the relaxation oscillator GPIO output pin high.
    //
    am_hal_gpio_out_bit_set(AM_BSP_GPIO_VCOMP_RXO);

    //
    // Set up for going to high level interrupt.
    //
    am_hal_vcomp_dac_level_set(AM_HAL_VCOMP_DAC_LVLSEL_2_32V);

    //
    // Enable Interrupts in the VCOMP.
    //
    am_hal_vcomp_int_enable(AM_HAL_VCOMP_INT_OUTHI);

    //
    // Enable the VCOMP interrupt in the NVIC.
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_VCOMP);
    am_hal_interrupt_master_enable();

    //
    // Announce that we are ready to start.
    //
    am_util_stdio_printf("VCOMP relaxation oscillator starting.\r\n\r\n");

    //
    // We are done printing. Disable debug printf messages on ITM.
    //
    am_bsp_debug_printf_disable();

    //
    // Stay here forever printing out the VCOMP state when it changes.
    //
    while (1)
    {
        //
        // Wait for a signal from the ISR.
        //
        if (g_ui32VCOMPCount > ui32Count)
        {
            //
            // Set up to wait for the next signal.
            //
            ui32Count = g_ui32VCOMPCount;

            //
            // The next part depends on the current state of the comparator.
            //
            if (g_bVCOMPState)
            {
                //
                // Print VCOMP is HIGH.
                // Enable the ITM, print a message, then disable the ITM.
                //
                am_bsp_debug_printf_enable();
                am_util_stdio_printf("VCOMP is HIGH\r\n");
                am_bsp_debug_printf_disable();
            }
            else
            {
                //
                // Print VCOMP is LOW.
                // Enable the ITM, print a message, then disable the ITM.
                //
                am_bsp_debug_printf_enable();
                am_util_stdio_printf("VCOMP is LOW\r\n");
                am_bsp_debug_printf_disable();
            }
        }

        //
        // Go to Deep Sleep here.
        //
        am_devices_led_off(am_bsp_psLEDs, 0);
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
        am_devices_led_on(am_bsp_psLEDs, 0);
    }
}
