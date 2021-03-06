//*****************************************************************************
//
//! @file lis2dh12.c
//!
//! @brief This module contains high level functions to sample lis2dh12.
//!
//! Configures the lis2dh12 to sample at 100Hz and set its watermark based
//! on the LIS2DH12_SAMPLE_SIZE define. When the lis2dh12 FIFO hits
//! its watermark (data-ready if sample size equals 1), an interrupt line
//! rises causing the Apollo MCU to interrupt and begin draining the FIFO while
//! sleeping and periodically waking to empty the internal IOM. The samples are
//! plotted over the ITM. Use AM FLash to view the real-time plot.
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
void data_handler_lis2dh12(void);

//*****************************************************************************
//
// Data buffer for the most recent accelerometer sample.
//
//*****************************************************************************
#define LIS2DH12_SAMPLE_SIZE                        5
am_devices_lis2dh12_sample(LIS2DH12_SAMPLE_SIZE)    g_sAccelData_lis2dh12;

//*****************************************************************************
//
// Flags to alert the main application of various interrupt conditions.
//
//*****************************************************************************
//volatile bool g_bMagDataReady = false;
volatile bool g_bIOMIdle_lis2dh12 = true;

//*****************************************************************************
//
// Configuration structure for the IO Master.
//
//*****************************************************************************
am_hal_iom_config_t g_sIOMConfig_lis2dh12 =
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
// Device structure for the Accelerometer
//
//*****************************************************************************
am_devices_lis2dh12_t g_sACCEL_lis2dh12 =
{
    .ui32IOMModule = AM_BSP_LIS2DH12_IOM,
    .ui32ChipSelect = AM_BSP_LIS2DH12_CS,
    .ui32Samples = LIS2DH12_SAMPLE_SIZE,
};

//*****************************************************************************
//
// Interrupt handler for the GPIO pins.
//
//*****************************************************************************
void am_gpio_lis2dh12_isr(void)
{
    //
    // Alert the base level that the IOM is active.
    //
    g_bIOMIdle_lis2dh12 = true;

    //
    // Start a SPI read command to retrieve the samples from the LIS2DH12
    //
    am_hal_iom_enable(AM_BSP_LIS2DH12_IOM);
    am_hal_iom_int_clear(AM_BSP_LIS2DH12_IOM,
                         AM_HAL_IOM_INT_CMDCMP | AM_HAL_IOM_INT_THR);
    am_hal_iom_int_enable(AM_BSP_LIS2DH12_IOM,
                          AM_HAL_IOM_INT_CMDCMP | AM_HAL_IOM_INT_THR);
    //
    // Start a SPI read command to retrieve the samples from the
    // gyro.
    //
    am_devices_lis2dh12_sample_get(&g_sACCEL_lis2dh12, g_sAccelData_lis2dh12.words,
                                   data_handler_lis2dh12);
}

//*****************************************************************************
//
// Function to handle incoming data from the accelerometer.
//
//*****************************************************************************
void data_handler_lis2dh12(void)
{
//    uint32_t i;
    //
    // Now we have the accelerometer data, so we can disable the IOM.
    //
    am_hal_iom_disable(AM_BSP_LIS2DH12_IOM);
    g_bIOMIdle_lis2dh12 = false;

    am_util_stdio_printf(" lis2dh12 gyro Data Rcvd \n");
    uart_transmit_delay();

#if 0

    //
    // Enable the ITM and plot the data
    //
    am_bsp_debug_printf_enable();

    //
    // Plot accelerometer data. LIS2DH12 accelerometer is configured by
    // am_devices_lis2dh12_config hal function in normal mode which
    // generates 10 bit, left-justified data.  Samples are shifted prior
    // to being transfered to the plot util.
    //
    for (i = 0; i < LIS2DH12_SAMPLE_SIZE; i++)
    {
        am_util_plot_int(AM_UTIL_PLOT_0, g_sAccelData_lis2dh12.samples[3 * i + 0] >> 6);
        am_util_plot_int(AM_UTIL_PLOT_1, g_sAccelData_lis2dh12.samples[3 * i + 1] >> 6);
        am_util_plot_int(AM_UTIL_PLOT_2, g_sAccelData_lis2dh12.samples[3 * i + 2] >> 6);
    }
#endif
}

//*****************************************************************************
//
// Perform all of the tasks necessary to initialize the LIS2DH12 and put it in
// measurement mode.
//
//*****************************************************************************
void start_lis2dh12(void)
{
    //
    // Enable the IOM so we can talk to the LIS.
    //
    am_hal_iom_enable(AM_BSP_LIS2DH12_IOM);

    //
    // Initialize the LIS2DH12 driver.
    //
    am_devices_lis2dh12_config(&g_sACCEL_lis2dh12);

    //
    // Wait until the data has actually gone out over the SPI lines, and then
    // disable the IOM. We won't need it again until the LIS actually has data.
    //
    am_hal_iom_poll_complete(AM_BSP_LIS2DH12_IOM);
    am_hal_iom_disable(AM_BSP_LIS2DH12_IOM);
}
//*****************************************************************************
//
// unconfigure GPIOs for communicating with the accelerometer
//
//*****************************************************************************
void unconfigure_lis2dh12_pins(void)
{
    //
    // disable IOM pins needed for the LIS2DH12.
    //
    am_bsp_iom_spi_pins_disable(AM_BSP_LIS2DH12_IOM);

    //
    // disable gyro interrupt and chip select.
    //
    am_bsp_pin_disable(LIS2DH12_CS);
    am_bsp_pin_disable(LIS2DH12_INT1);

    //
    // disable ITM pin for plotting
    //
    //am_bsp_pin_disable(ITM_SWO);

    //
    // disable the chip-select and data-ready pins for the LIS2DH12.
    //
    am_hal_gpio_int_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_LIS2DH12_INT2));

}

//*****************************************************************************
//
// Configure GPIOs for communicating with the accelerometer
//
//*****************************************************************************
void configure_lis2dh12_pins(void)
{
    //
    // Set up IOM1 SPI pins.
    //
    am_bsp_iom_spi_pins_enable(AM_BSP_LIS2DH12_IOM);

    //
    // Enable the chip-select and data-ready pins for the LIS2DH12
    //
    am_bsp_pin_enable(LIS2DH12_CS);
    am_bsp_pin_enable(LIS2DH12_INT1);

    //
    // Setup ITM pin for plotting
    //
    //am_bsp_pin_enable(ITM_SWO);

    //
    // Enable a GPIO interrupt for positive edges on the DRDY pin.
    //
    am_hal_gpio_int_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_LIS2DH12_INT1));
    am_hal_gpio_int_enable(AM_HAL_GPIO_BIT(AM_BSP_GPIO_LIS2DH12_INT1));
}

//*****************************************************************************
//
// lis2dh12 Test.
//
//*****************************************************************************
void lis2dh12_test(uint32_t TestCommand2)
{
    static uint32_t loopInteration;

    //
    // Configure IOM1 for talking with the LIS2DH12.
    //
    am_hal_iom_config(AM_BSP_LIS2DH12_IOM, &g_sIOMConfig_lis2dh12);

    //
    // Configure the GPIOs to work with the LIS2DH12.
    //
    configure_lis2dh12_pins();

    //
    // Enable an LED so we can see whether we're awake or asleep
    //
    am_hal_gpio_pin_config(AM_BSP_GPIO_LED0, AM_HAL_PIN_OUTPUT);
    am_hal_gpio_out_bit_clear(AM_BSP_GPIO_LED0);

    //
    // Configure the LIS2DH12, and start the data-taking process.
    //
    start_lis2dh12();

    //
    // Initialize the plotting interface.
    //
    //am_util_plot_init();

    //
    // Enable interrupts before entering application loop
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
            g_bIOMIdle_lis2dh12 = false;
            unconfigure_lis2dh12_pins();
            am_hal_interrupt_master_enable();
            am_util_stdio_printf("1: %d \n",loopInteration);
            return;
        }
        //
        // Pick a sleep mode. If the IOM is active, we will need to wake up
        // again very soon, so we will use normal sleep mode. Otherwise, we
        // should use deep sleep.
        //
        if (g_bIOMIdle_lis2dh12)
        {
            am_hal_gpio_out_bit_set(AM_BSP_GPIO_LED0);
            am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);
            am_hal_gpio_out_bit_clear(AM_BSP_GPIO_LED0);
        }
        else
        {
            //
            // Disable ITM before going to deep sleep
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
