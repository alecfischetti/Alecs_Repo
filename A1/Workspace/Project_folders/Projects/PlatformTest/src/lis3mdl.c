//*****************************************************************************
//
//! @file lis3mdl.c
//!
//! @brief This module contains high level functions to sample lis3mdl.
//!
//! Configures the lis3mdl to sample at 100Hz and interrupt on data-ready
//! causing the Apollo MCU to interrupt and retrieve the sample. The samples
//! are plotted over the ITM.
//!
//! While moving the EVK board, use AM Flash (click "Show Plot Window") to
//! view the real-time plot.
//!
//! Note: 
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
void data_handler_lis3mdl(void);

//*****************************************************************************
//
// Data buffer for the most recent magnetometer sample.
// Note: The lis3mdl does not have an internal fifo so setting a sample
// size larger than one has no effect on sampling.
//
//*****************************************************************************
#define LIS3MDL_SAMPLE_SIZE                     1
am_devices_lis3mdl_sample(LIS3MDL_SAMPLE_SIZE)  g_sMagData_lis3mdl;

//*****************************************************************************
//
// Flags to alert the main application of various interrupt conditions.
//
//*****************************************************************************
//volatile bool g_bMagDataReady = false;
volatile bool g_bIOMIdle_lis3mdl = true;

//*****************************************************************************
//
// Configuration structure for the IO Master.
//
//*****************************************************************************
am_hal_iom_config_t g_sIOMConfig_lis3mdl =
{
    .ui32InterfaceMode = AM_HAL_IOM_SPIMODE,
    .ui32ClockFrequency = AM_HAL_IOM_8MHZ,
    .bSPHA = 0,
    .bSPOL = 0,
    .ui8WriteThreshold = 0,
    .ui8ReadThreshold = 60,
};

//*****************************************************************************
//
// Device structure for the ADXL
//
//*****************************************************************************
am_devices_lis3mdl_t g_sMAG_lis3mdl =
{
    .ui32IOMModule = AM_BSP_LIS3MDL_IOM,
    .ui32ChipSelect = AM_BSP_LIS3MDL_CS,
};

//*****************************************************************************
//
// Interrupt handler for the GPIO pins.
//
//*****************************************************************************
void am_gpio_lis3mdl_isr(void)
{
    //
    // Alert the base level that the IOM is active.
    //
    g_bIOMIdle_lis3mdl = true;

    //
    // Start a SPI read command to retrieve the samples from the ADXL
    //
    am_hal_iom_enable(AM_BSP_LIS3MDL_IOM);
    am_hal_iom_int_clear(AM_BSP_LIS3MDL_IOM,
                         AM_HAL_IOM_INT_CMDCMP | AM_HAL_IOM_INT_THR);
    am_hal_iom_int_enable(AM_BSP_LIS3MDL_IOM,
                          AM_HAL_IOM_INT_CMDCMP | AM_HAL_IOM_INT_THR);
    //
    // Start a SPI read command to retrieve the samples from the
    // gyro.
    //
    am_devices_lis3mdl_sample_get(&g_sMAG_lis3mdl, g_sMagData_lis3mdl.words,
                                  data_handler_lis3mdl);
}

//*****************************************************************************
//
// Function to handle incoming data from the magnetometer.
//
//*****************************************************************************
void data_handler_lis3mdl(void)
{
    //
    // Now we have the magnetometer data, so we can disable the IOM.
    //
    am_hal_iom_disable(AM_BSP_LIS3MDL_IOM);
    g_bIOMIdle_lis3mdl = false;
    am_util_stdio_printf(" lis3mdl Magne Data Rcvd \n");
    uart_transmit_delay();
#if 0

    //
    // Enable the ITM and plot the data
    //
    am_bsp_debug_printf_enable();

    am_util_plot_int(AM_UTIL_PLOT_0, g_sMagData_lis3mdl.samples[0]);
    am_util_plot_int(AM_UTIL_PLOT_1, g_sMagData_lis3mdl.samples[1]);
    am_util_plot_int(AM_UTIL_PLOT_2, g_sMagData_lis3mdl.samples[2]);
#endif
}

//*****************************************************************************
//
// Perform all of the tasks necessary to initialize the LIS3MDL and put it in
// measurement mode.
//
//*****************************************************************************
void start_lis3mdl(void)
{
    //
    // Enable the IOM so we can talk to the LIS.
    //
    am_hal_iom_enable(AM_BSP_LIS3MDL_IOM);

    //
    // Initialize the ADXL362 driver.
    //
    am_devices_lis3mdl_config(&g_sMAG_lis3mdl);

    //
    // Wait until the data has actually gone out over the SPI lines, and then
    // disable the IOM. We won't need it again until the LIS actually has data.
    //
    am_hal_iom_poll_complete(AM_BSP_LIS3MDL_IOM);
    am_hal_iom_disable(AM_BSP_LIS3MDL_IOM);
}

//*****************************************************************************
//
// unconfigure GPIOs for communicating with the magnetometer
//
//*****************************************************************************
void unconfigure_lis3mdl_pins(void)
{
    //
    // disable IOM pins needed for the LIS2DH12.
    //
    am_bsp_iom_spi_pins_disable(AM_BSP_LIS3MDL_IOM);

    //
    // disable gyro interrupt and chip select.
    //
    am_bsp_pin_disable(LIS3MDL_CS);
    am_bsp_pin_disable(LIS3MDL_INT1);

    //
    // disable ITM pin for plotting
    //
    //am_bsp_pin_disable(ITM_SWO);

    //
    // disable the chip-select and data-ready pins for the LIS3MDL.
    //
    am_hal_gpio_int_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_LIS3MDL_DRDY));

}

//*****************************************************************************
//
// Configure GPIOs for communicating with the magnetometer
//
//*****************************************************************************
void configure_lis3mdl_pins(void)
{
    //
    // Set up IOM1 SPI pins.
    //
    am_bsp_iom_spi_pins_enable(AM_BSP_LIS3MDL_IOM);

    //
    // Enable the chip-select and data-ready pins for the LIS3MDL
    //
    am_bsp_pin_enable(LIS3MDL_CS);
    am_bsp_pin_enable(LIS3MDL_DRDY);

    //
    // Setup ITM pin for plotting
    //
    //am_bsp_pin_enable(ITM_SWO);

    //
    // Enable a GPIO interrupt for positive edges on the DRDY pin.
    //
    am_hal_gpio_int_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_LIS3MDL_DRDY));
    am_hal_gpio_int_enable(AM_HAL_GPIO_BIT(AM_BSP_GPIO_LIS3MDL_DRDY));
}

//*****************************************************************************
//
// lis3mdl Test.
//
//*****************************************************************************
void lis3mdl_test(uint32_t TestCommand2)
{
    static uint32_t loopInteration;

    //
    // Configure IOM1 for talking with the LIS3MDL.
    //
    am_hal_iom_config(AM_BSP_LIS3MDL_IOM, &g_sIOMConfig_lis3mdl);

    //
    // Configure the GPIOs to work with the LIS3MDL.
    //
    configure_lis3mdl_pins();

    //
    // Enable an LED so we can see whether we're awake or asleep
    //
    am_hal_gpio_pin_config(AM_BSP_GPIO_LED0, AM_HAL_PIN_OUTPUT);
    am_hal_gpio_out_bit_clear(AM_BSP_GPIO_LED0);

    //
    // Configure the LIS3MDL, and start the data-taking process.
    //
    start_lis3mdl();

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
            g_bIOMIdle_lis3mdl = false;
            unconfigure_lis3mdl_pins();
            am_hal_interrupt_master_enable();
            am_util_stdio_printf("1: %d \n",loopInteration);
            return;
        }
        //
        // Pick a sleep mode. If the IOM is active, we will need to wake up
        // again very soon, so we will use normal sleep mode. Otherwise, we
        // should use deep sleep.
        //
        if (g_bIOMIdle_lis3mdl)
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
