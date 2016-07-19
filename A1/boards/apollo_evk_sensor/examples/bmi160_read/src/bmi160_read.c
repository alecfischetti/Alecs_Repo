//*****************************************************************************
//
//! @file bmi160_read.c
//!
//! @brief This example samples the bmi160 at 100Hz.
//!
//! Configures the bmi160 to sample at 100Hz and set its watermark based
//! on the BMI160_SAMPLE_SIZE define. When the bmi160 FIFO hits
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

//*****************************************************************************
//
// Forward declaration for the data handler function.
//
//*****************************************************************************
void data_handler(void);

//*****************************************************************************
//
// Buffer for holding BMI samples. 1 sample consists of 12 bytes (Gyro/Accel)
//
//*****************************************************************************
#define BMI160_SAMPLE_SIZE                      5
static bmi160_sample_buffer(BMI160_SAMPLE_SIZE) g_psSampleBuffer;

//*****************************************************************************
//
// Flags to alert the main application of various interrupt conditions.
//
//*****************************************************************************
volatile bool g_bIOMIdle = true;

//*****************************************************************************
//
// Configuration structure for the IO Master.
//
//*****************************************************************************
const am_hal_iom_config_t g_sIOMConfig =
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
// Device structure for the BMI.
//
//*****************************************************************************
const am_devices_bmi160_t g_sBMI160 =
{
    .bMode = AM_DEVICES_BMI160_MODE_SPI,
    .ui32IOMModule = AM_BSP_BMI160_IOM,
    .ui32ChipSelect = AM_BSP_BMI160_CS,
    .ui32Address = 0x00,                            // unused in SPI mode
    .ui32Samples = BMI160_SAMPLE_SIZE,
};

//*****************************************************************************
//
// Interrupt handler for IOM0
//
//*****************************************************************************
void
am_iomaster0_isr(void)
{
    uint32_t ui32IntStatus;

    //
    // Read and clear the interrupt status.
    //
    ui32IntStatus = am_hal_iom_int_status_get(0, false);
    am_hal_iom_int_clear(0, ui32IntStatus);

    //
    // Service FIFO interrupts as necessary, and call IOM callbacks as
    // transfers are completed.
    //
    am_hal_iom_int_service(0, ui32IntStatus);
}

//*****************************************************************************
//
// Interrupt handler for IOM1
//
//*****************************************************************************
void
am_iomaster1_isr(void)
{
    uint32_t ui32IntStatus;

    //
    // Read and clear the interrupt status.
    //
    ui32IntStatus = am_hal_iom_int_status_get(1, false);
    am_hal_iom_int_clear(1, ui32IntStatus);

    //
    // Service FIFO interrupts as necessary, and call IOM callbacks as
    // transfers are completed.
    //
    am_hal_iom_int_service(1, ui32IntStatus);
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
    // alert the base-level application that the BMI has data.
    //
    if (ui64Status & AM_HAL_GPIO_BIT(AM_BSP_GPIO_BMI160_INT1))
    {
        //
        // Alert the base level that the IOM is active.
        //
        g_bIOMIdle = true;

        //
        // Make sure the IOM is enabled, and that interrupts are configured.
        //
        am_hal_iom_enable(AM_BSP_BMI160_IOM);
        am_hal_iom_int_clear(AM_BSP_BMI160_IOM,
                             AM_HAL_IOM_INT_CMDCMP | AM_HAL_IOM_INT_THR);
        am_hal_iom_int_enable(AM_BSP_BMI160_IOM,
                              AM_HAL_IOM_INT_CMDCMP | AM_HAL_IOM_INT_THR);

        //
        // We expect to get six samples (gryo[xyz] & accel[xyz]). Call the
        // bmi160_data_handler when the samples finally arrive.
        //
        am_devices_bmi160_sample_get(&g_sBMI160, BMI160_SAMPLE_SIZE,
                                     g_psSampleBuffer.words,
                                     data_handler);
    }
}

//*****************************************************************************
//
// Function to handle incoming data from the BMI.
//
//*****************************************************************************
void
data_handler(void)
{
    uint32_t i;

    //
    // Now we have the data, so we can disable the IOM.
    //
    am_hal_iom_disable(AM_BSP_BMI160_IOM);
    g_bIOMIdle = false;

    //
    // Enable the ITM and plot the data
    //
    am_bsp_debug_printf_enable();

    //
    // Plot the gyro data.
    // Note: The accel data is in the "next" three slots of the buffer
    // g_psSampleBuffer.samples[3] - accel-X
    // g_psSampleBuffer.samples[4] - accel-Y
    // g_psSampleBuffer.samples[5] - accel-Z
    //
    for (i = 0; i < BMI160_SAMPLE_SIZE; i++)
    {
        am_util_plot_int(AM_UTIL_PLOT_0, g_psSampleBuffer.samples[6 * i + 0]);
        am_util_plot_int(AM_UTIL_PLOT_1, g_psSampleBuffer.samples[6 * i + 1]);
        am_util_plot_int(AM_UTIL_PLOT_2, g_psSampleBuffer.samples[6 * i + 2]);
    }
}

//*****************************************************************************
//
// Perform all of the tasks necessary to initialize the BMI and put it in
// measurement mode.
//
//*****************************************************************************
void
start_bmi160(void)
{
    //
    // Enable the IOM so we can talk to the BMI.
    //
    am_hal_iom_enable(AM_BSP_BMI160_IOM);

    //
    // Set BMI registers to the default settings from the driver (asynchronous
    // mode) and set "measurement mode" to get the BMI to start taking
    // samples.
    //
    am_devices_bmi160_config(&g_sBMI160);

    //
    // Wait until the data has actually gone out over the SPI lines.
    //
    am_hal_iom_poll_complete(AM_BSP_BMI160_IOM);

    //
    // Disable the IOM to save power. We won't need it again until the BMI
    // actually has data.
    //
    am_hal_iom_disable(AM_BSP_BMI160_IOM);
}

//*****************************************************************************
//
// Configure GPIOs for communicating with the BMI
//
//*****************************************************************************
void
configure_bmi160_pins(void)
{
    //
    // Set up the IOM pins needed for the BMI160.
    //
    am_bsp_iom_spi_pins_enable(AM_BSP_BMI160_IOM);

    //
    // Set up BMI interrupt and chip select.
    //
    am_bsp_pin_enable(BMI160_CS);
    am_bsp_pin_enable(BMI160_INT1);

    //
    // Setup ITM pin for plotting
    //
    am_bsp_pin_enable(ITM_SWO);

    //
    // Enable a GPIO interrupt for positive edges on BMI's INT1 pin.
    //
    am_hal_gpio_int_clear(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BMI160_INT1));
    am_hal_gpio_int_enable(AM_HAL_GPIO_BIT(AM_BSP_GPIO_BMI160_INT1));
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
    // Set the system clock to maximum frequency.
    //
    am_hal_clkgen_sysclk_select(AM_HAL_CLKGEN_SYSCLK_MAX);

    //
    // Set up the board using the default configurations in the BSP.
    //
    am_bsp_low_power_init();

    //
    // Configure the IOM for talking with the BMI.
    //
    am_hal_iom_config(AM_BSP_BMI160_IOM, &g_sIOMConfig);

    //
    // Configure the GPIOs to work with the BMI.
    //
    configure_bmi160_pins();

    //
    // Enable an LED so we can see whether we're awake or asleep
    //
    am_hal_gpio_pin_config(AM_BSP_GPIO_LED0, AM_HAL_PIN_OUTPUT);
    am_hal_gpio_out_bit_clear(AM_BSP_GPIO_LED0);

    //
    // Configure the BMI, and start the data-taking process.
    //
    start_bmi160();

    //
    // Initialize the plotting interface.
    //
    am_util_plot_init();

    //
    // Enable interrupts before entering application loop
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_IOMASTER1);
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_GPIO);
    am_hal_interrupt_master_enable();

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
        // Pick a sleep mode. If the IOM is active, we will need to wake up
        // again very soon, so we will use normal sleep mode. Otherwise, we
        // should use deep sleep.
        //
        if (g_bIOMIdle)
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
