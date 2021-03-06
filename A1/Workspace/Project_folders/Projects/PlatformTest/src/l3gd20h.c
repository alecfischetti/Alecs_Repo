//*****************************************************************************
//
//! @file l3gd20h.c
//!
//! @brief This module contains high level functions to sample l3dh20h.
//!
//! Configures the l3gd20h to sample at 100Hz and set its watermark based
//! on the L3GD20H_SAMPLE_SIZE define. When the l3gd20h FIFO hits
//! its watermark (data-ready if sample size equals 1), an interrupt line
//! rises causing the Apollo MCU to interrupt and begin draining the FIFO while
//! sleeping and periodically waking to empty the internal IOM. The samples are
//! plotted over the ITM. Use AM FLash to view the real-time plot.
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
#include "am_bsp.h"
#include "am_devices.h"
#include "am_util.h"

extern void uart_transmit_delay(void);

//*****************************************************************************
//
// Forward declaration for the data handler function.
//
//*****************************************************************************
void data_handler_l3gd20h(void);

//*****************************************************************************
//
// Data buffer for the most recent gyro sample.
//
//*****************************************************************************
#define L3GD20H_SAMPLE_SIZE                     5
am_devices_l3gd20h_sample(L3GD20H_SAMPLE_SIZE)  g_sGyroData_l3gd20h;

//*****************************************************************************
//
// Flags to alert the main application of various interrupt conditions.
//
//*****************************************************************************
//volatile bool g_bDataReady = false;
volatile bool g_bIOMIdle_l3gd20h = true;

//*****************************************************************************
//
// Configuration structure for the IO Master.
//
//*****************************************************************************
am_hal_iom_config_t g_sIOMConfig_l3gd20h =
{
    .ui32InterfaceMode = AM_HAL_IOM_SPIMODE,
    .ui32ClockFrequency = AM_HAL_IOM_1MHZ,
    .bSPHA = 0,
    .bSPOL = 0,
    .ui8WriteThreshold = 0,
    .ui8ReadThreshold = 60,
};

//*****************************************************************************
//
// Gyro structure.
//
//*****************************************************************************
am_devices_l3gd20h_t g_sL3GD20H_l3gd20h =
{
    .ui32IOMModule = AM_BSP_L3GD20H_IOM,
    .ui32ChipSelect = AM_BSP_L3GD20H_CS,
    .ui32Samples = L3GD20H_SAMPLE_SIZE,
};

//*****************************************************************************
//
// Interrupt handler for the GPIO pins.
//
//*****************************************************************************
void am_gpio_l3gd20h_isr(void)
{
    //
    // Alert the base level that the IOM is active.
    //
    g_bIOMIdle_l3gd20h = true;

    //
    // Start a SPI read command to retrieve the samples from the gyro
    //
    am_hal_iom_enable(AM_BSP_L3GD20H_IOM);
    am_hal_iom_int_clear(AM_BSP_L3GD20H_IOM,
                         AM_HAL_IOM_INT_CMDCMP | AM_HAL_IOM_INT_THR);
    am_hal_iom_int_enable(AM_BSP_L3GD20H_IOM,
                          AM_HAL_IOM_INT_CMDCMP | AM_HAL_IOM_INT_THR);
    //
    // Start a SPI read command to retrieve the samples from the
    // gyro.
    //
    am_devices_l3gd20h_sample_read(&g_sL3GD20H_l3gd20h, g_sGyroData_l3gd20h.words,
                                   data_handler_l3gd20h);
}

//*****************************************************************************
//
// Function to handle incoming data from the gyro.
//
//*****************************************************************************
void data_handler_l3gd20h(void)
{
//    uint32_t i;

    //
    // Now we have the gyro data, so we can disable the IOM.
    //
    am_hal_iom_disable(AM_BSP_L3GD20H_IOM);
    g_bIOMIdle_l3gd20h = false;

    am_util_stdio_printf(" l3gd20h gyro Data Rcvd \n");
    uart_transmit_delay();

#if 0
    //
    // Enable the ITM and plot the data
    //
    am_bsp_debug_printf_enable();

    for (i = 0; i < L3GD20H_SAMPLE_SIZE; i++)
    {
        am_util_plot_int(AM_UTIL_PLOT_0, g_sGyroData_l3gd20h.samples[3 * i + 0]);
        am_util_plot_int(AM_UTIL_PLOT_1, g_sGyroData_l3gd20h.samples[3 * i + 1]);
        am_util_plot_int(AM_UTIL_PLOT_2, g_sGyroData_l3gd20h.samples[3 * i + 2]);
    }
#endif
}

//*****************************************************************************
//
// Perform all of the tasks necessary to initialize the L3GD20H and put it in
// measurement mode.
//
//*****************************************************************************
void start_l3gd20h(void)
{
    //
    // Enable the IOM so we can talk to the gyro.
    //
    am_hal_iom_enable(AM_BSP_L3GD20H_IOM);

    //
    // Configure the gyro and start taking measurements.
    //
    am_devices_l3gd20h_config(&g_sL3GD20H_l3gd20h);

    //
    // Wait until the data has actually gone out over the SPI lines, and then
    // disable the IOM. We won't need it again until the L3GD20H actually has
    // data.
    //
    am_hal_iom_poll_complete(AM_BSP_L3GD20H_IOM);
    am_hal_iom_disable(AM_BSP_L3GD20H_IOM);
}

//*****************************************************************************
//
// unconfigure GPIOs for communicating with the gyro
//
//*****************************************************************************
void unconfigure_l3gd20h_pins(void)
{
    //
    // disable IOM pins needed for the L3GD20H.
    //
    am_bsp_iom_spi_pins_disable(AM_BSP_L3GD20H_IOM);

    //
    // disable gyro interrupt and chip select.
    //
    am_bsp_pin_disable(L3GD20H_CS);
    am_bsp_pin_disable(L3GD20H_INT2);

    //
    // disable ITM pin for plotting
    //
    //am_bsp_pin_disable(ITM_SWO);

    //
    // disable a GPIO interrupt for positive edges on the DRDY pin.
    //
    am_hal_gpio_int_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_L3GD20H_INT2));

}

//*****************************************************************************
//
// Configure GPIOs for communicating with the gyro
//
//*****************************************************************************
void configure_l3gd20h_pins(void)
{
    //
    // Set up IOM1 SPI pins.
    //
    am_bsp_iom_spi_pins_enable(AM_BSP_L3GD20H_IOM);
    am_bsp_pin_enable(L3GD20H_CS);

    //
    // Set up gyro data-ready pin.
    //
    am_bsp_pin_enable(L3GD20H_INT2);

    //
    // Setup ITM pin for plotting
    //
    //am_bsp_pin_enable(ITM_SWO);

    //
    // Enable a GPIO interrupt for positive edges on the DRDY pin.
    //
    am_hal_gpio_int_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_L3GD20H_INT2));
    am_hal_gpio_int_enable(AM_HAL_GPIO_BIT(AM_BSP_GPIO_L3GD20H_INT2));
}

//*****************************************************************************
//
// l3gd20h Test.
//
//*****************************************************************************
void l3gd20h_test(uint32_t TestCommand2)
{
    static uint32_t loopInteration;
    
    //
    // Configure IOM1 for talking with the L3GD20H.
    //
    am_hal_iom_config(AM_BSP_L3GD20H_IOM, &g_sIOMConfig_l3gd20h);

    //
    // Configure the GPIOs to work with the L3GD20H.
    //
    configure_l3gd20h_pins();

    //
    // Enable an LED so we can see whether we're awake or asleep
    //
    am_hal_gpio_pin_config(AM_BSP_GPIO_LED0, AM_HAL_PIN_OUTPUT);
    am_hal_gpio_out_bit_clear(AM_BSP_GPIO_LED0);

    //
    // Configure the L3GD20H, and start the data-taking process.
    //
    start_l3gd20h();

    //
    // Initialize the plotting interface.
    //
    //am_util_plot_init();

    //
    // Enable interrupts before entering applicaiton loop
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_IOMASTER1);
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_GPIO);
    am_hal_interrupt_master_enable();

    //
    // Loop forever, waiting for events.
    //
    loopInteration = TestCommand2;
    if(loopInteration < 0)
    {
        loopInteration = 1000;
    }

    while (loopInteration)
    {
       loopInteration--;

       am_util_stdio_printf(" %d \n",loopInteration);
       uart_transmit_delay();
        //
        // Disable interrupts temporarily. We need to make sure that the global
        // flags aren't changing while we are checking them.
        //
        am_hal_interrupt_master_disable();

        if( loopInteration == 0)
        {
            g_bIOMIdle_l3gd20h = false;
            unconfigure_l3gd20h_pins();
            am_hal_interrupt_master_enable();
            am_util_stdio_printf("1: %d \n",loopInteration);
            return;
        }
        //
        // Pick a sleep mode. If the IOM is active, we will need to wake up
        // again very soon, so we will use normal sleep mode. Otherwise, we
        // should use deep sleep.
        //
        if (g_bIOMIdle_l3gd20h)
        {
            am_hal_gpio_out_bit_set(AM_BSP_GPIO_LED0);
            am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);
            am_hal_gpio_out_bit_clear(AM_BSP_GPIO_LED0);
        }
        else
        {
            //
            // Diasable ITM before going to deep sleep
            //
            am_bsp_debug_printf_disable();

            //
            // Go to deep sleep
            //
            am_hal_gpio_out_bit_set(AM_BSP_GPIO_LED0);
            am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
            am_hal_gpio_out_bit_clear(AM_BSP_GPIO_LED0);
        }

        //
        // Re-enable interrupts so we can handle any new events that have come
        // in.
        //
        am_hal_interrupt_master_enable();
    }
}
