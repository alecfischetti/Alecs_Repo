//*****************************************************************************
//
//! @file adxl362_task.c
//!
//! @brief ADXL362 Linear Accelerometer Read Task
//!
//! DESCRIPTION: Once initialized, this continuously reads 3 axis accelerometer
//! samples from the ADXL362 over the SPI bus. These samples are written to an
//! Ambiq standard ring buffer to the data_logger application task.
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

#include <stdint.h>
#include <stdbool.h>

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "am_devices.h"

#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"
#include "portable.h"
#include "semphr.h"
#include "event_groups.h"

#include "iom_ctl.h"
#include "gpio_ctl.h"
#include "adxl362_task.h"

//*****************************************************************************
//
// Task handle.
//
//*****************************************************************************
TaskHandle_t xADXL362Task;

//*****************************************************************************
//
// Event handles
//
//*****************************************************************************
EventGroupHandle_t xADXL362Events = 0;

//*****************************************************************************
//
// Private (static) Variables
//
//*****************************************************************************
#define ADXL362_SAMPLE_SIZE                             21
static am_devices_adxl362_sample(ADXL362_SAMPLE_SIZE)   g_psSampleBuffer;

//*****************************************************************************
//
// Device structure for the ADXL.
//
//*****************************************************************************
am_devices_adxl362_t g_sADXL =
{
    .ui32IOMModule = AM_BSP_ADXL362_IOM,
    .ui32ChipSelect = AM_BSP_ADXL362_CS,
    .ui32Samples = ADXL362_SAMPLE_SIZE,
    .ui32SampleRate = AM_DEVICES_ADXL362_400HZ,
    .ui32Range = AM_DEVICES_ADXL362_2G,
    .bHalfBandwidth = false,
    .bSyncMode = false,
};

//*****************************************************************************
//
// GPIO Handler for ADXL INT1
//
//*****************************************************************************
void
adxl_handler(void)
{
    BaseType_t xHigherPriorityTaskWoken, xResult;

    //
    // Signal event to adxl362 task
    //
    if (xADXL362Events)
    {
        //
        // Set the event bits, and check to see if this will cause another
        // higher-priority event to be awoken.
        //
        xHigherPriorityTaskWoken = pdFALSE;

        xResult = xEventGroupSetBitsFromISR(xADXL362Events,
                                            ADXL362_DATA_READY_EVENT,
                                            &xHigherPriorityTaskWoken);

        //
        // If we need to transfer to a higher priority task, make sure that there
        // is an opportunity to do that here.
        //
        if (xResult != pdFAIL)
        {
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

//*****************************************************************************
//
// ADXL362 task function.
//
//*****************************************************************************
void
adxl362_task(void *pvParameters)
{
    uint32_t ui32NumSamples, ui32Index;
    int16_t i16XAccel, i16YAccel, i16ZAccel;

    EventBits_t xEvent;

    //
    // Create an event group to catch ADXL362 GPIO interrupts.
    //
    xADXL362Events = xEventGroupCreate();

    //
    // Add the ADXL interrupt handler to the GPIO interrupt handler list, and
    // enable the GPIO interrupt for the ADXL data ready signal.
    //
    am_hal_gpio_int_register(AM_BSP_GPIO_ADXL362_INT1, adxl_handler);
    am_hal_gpio_int_enable(1UL << AM_BSP_GPIO_ADXL362_INT1);
    am_bsp_pin_enable(ADXL362_INT1);

    //
    // Enable the IOM for the ADXL362 connection and the corresponding CS pin.
    //
    am_hal_iom_enable(AM_BSP_ADXL362_IOM);
    am_bsp_pin_enable(ADXL362_CS);

    am_devices_adxl362_config(&g_sADXL);

    am_devices_adxl362_measurement_mode_set(&g_sADXL);
    am_devices_adxl362_fifo_depth_get(&g_sADXL,
                                      &ui32NumSamples);

    //
    // Start with the assumption that we don't have any ADXL samples yet, and
    // that we're not accelerating at all.
    //
    xEvent = 0;
    i16XAccel = 0;
    i16YAccel = 0;
    i16ZAccel = 0;

    while (1)
    {
        //
        // Loop forever, waiting for either an RTOS event from the ADXL's
        // interrupt handler, or for the ADXL's gpio interrupt line to be high.
        // Checking for both cases here ensures that we can recover even if we
        // were to somehow miss the initial interrupt.
        //
        if ((xEvent & 0x1) || am_hal_gpio_input_bit_read(AM_BSP_GPIO_ADXL362_INT1))
        {
            //
            // Clear our event.
            //
            xEvent = 0;

            //
            // If we received an event because the ADXL has another batch of
            // 210 samples to send us, initiate a 210-samples read to capture
            // this data.
            //
            am_devices_adxl362_sample_get(&g_sADXL, ADXL362_SAMPLE_SIZE,
                                          g_psSampleBuffer.words, 0);
            //
            // Re-enable the TPIU and SWO pin to allow for debug messages.
            //
            am_bsp_debug_printf_enable();

            for (ui32Index = 0; ui32Index < ADXL362_SAMPLE_SIZE; ui32Index += 3)
            {
                //
                // Match up the next triplet of samples with the correct axis
                // name, and read the acceleration values.
                //
                i16XAccel = AM_DEVICES_ADXL362_VALUE(g_psSampleBuffer.samples[ui32Index + 0]);
                i16YAccel = AM_DEVICES_ADXL362_VALUE(g_psSampleBuffer.samples[ui32Index + 1]);
                i16ZAccel = AM_DEVICES_ADXL362_VALUE(g_psSampleBuffer.samples[ui32Index + 2]);

                //
                // Plot acceleration data to AM Flash.
                //
                am_util_plot_int(AM_UTIL_PLOT_0, i16XAccel);
                am_util_plot_int(AM_UTIL_PLOT_1, i16YAccel);
                am_util_plot_int(AM_UTIL_PLOT_2, i16ZAccel);
            }

            //
            // Disable the SWO pin and the TPIU to save power.
            //
            am_bsp_debug_printf_disable();
        }
        else
        {
            //
            // If there isn't any indication of data from the ADXL, we can put
            // this task to sleep. The task will resume when the 100-tick
            // timeout expires, or when we receive an interrupt from the ADXL.
            //
            xEvent = xEventGroupWaitBits(xADXL362Events, 0x1, pdTRUE, pdFALSE,
                                         100);
        }
    }
}
