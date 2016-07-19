//*****************************************************************************
//
//! @file iom_ctl.c
//!
//! @brief Functions for controlling the IOM in an RTOS compatible way.
//!
//! These functions help the low-level operation of the IOM in an RTOS context.
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
#include "am_devices.h"
#include "am_bsp.h"

#include "am_util.h"

#include "FreeRTOS.h"
#include "task.h"
#include "portmacro.h"
#include "portable.h"
#include "semphr.h"
#include "event_groups.h"

#include "iom_ctl.h"

//*****************************************************************************
//
// Mutex locks for the iom
//
//*****************************************************************************
SemaphoreHandle_t xIOM0Semaphore;
SemaphoreHandle_t xIOM1Semaphore;

//*****************************************************************************
//
// Configuration structure for the IOM
//
//*****************************************************************************
am_hal_iom_config_t g_sIOMConfig =
{
    .ui32ClockFrequency = AM_HAL_IOM_8MHZ,
    .ui32InterfaceMode = AM_HAL_IOM_SPIMODE,
    .ui8WriteThreshold = 4,
    .ui8ReadThreshold = 60,
    .bSPHA = 0,
    .bSPOL = 0,
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
// IOM setup function.
//
//*****************************************************************************
void
iom_ctl_init(void)
{
    //
    // There's only two iom modles, so create mutex locks for them
    //
    xIOM0Semaphore = xSemaphoreCreateMutex();
    xIOM1Semaphore = xSemaphoreCreateMutex();

    //
    // Enable the IOM pins.
    //
    am_bsp_iom_spi_pins_enable(AM_BSP_ADXL362_IOM);

    //
    // Set the required configuration settings for the IOM.
    //
    am_hal_iom_config(AM_BSP_ADXL362_IOM, &g_sIOMConfig);

    //
    // Enable interrupts for command complete and fifo threshold events, which
    // are needed for the iom_ctrl write and read functions.
    //
    am_hal_iom_int_enable(0, AM_HAL_IOM_INT_CMDCMP | AM_HAL_IOM_INT_THR);
    am_hal_iom_int_enable(1, AM_HAL_IOM_INT_CMDCMP | AM_HAL_IOM_INT_THR);

    //
    // Set the iom interrupt priority to a level that allows interacting with
    // FreeRTOS API calls.
    //
    am_hal_interrupt_priority_set(AM_HAL_INTERRUPT_IOMASTER0, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    am_hal_interrupt_priority_set(AM_HAL_INTERRUPT_IOMASTER1, configMAX_SYSCALL_INTERRUPT_PRIORITY);

    //
    // Enable interrupts to IOM0 and IOM1
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_IOMASTER0);
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_IOMASTER1);
}
