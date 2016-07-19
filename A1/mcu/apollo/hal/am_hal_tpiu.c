//*****************************************************************************
//
//! @file am_hal_tpiu.c
//!
//! @brief Support functions for the ARM TPIU module
//!
//! Provides support functions for configuring the ARM TPIU module
//!
//! @addtogroup hal Hardware Abstraction Layer (HAL)
//! @addtogroup tpiu Trace Port Interface Unit (TPIU)
//! @ingroup hal
//! @{
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

//*****************************************************************************
//
//! @brief Enable the clock to the TPIU module.
//!
//! This function enables the clock to the TPIU module.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_tpiu_clock_enable(void)
{
    //
    // Enable the TPIU clock
    //
    AM_REG(MCUCTRL, TPIUCTRL) |= AM_REG_MCUCTRL_TPIUCTRL_ENABLE_M;
}

//*****************************************************************************
//
//! @brief Disable the clock to the TPIU module.
//!
//! This function disables the clock to the TPIU module.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_tpiu_clock_disable(void)
{
    //
    // Disable the TPIU clock
    //
    AM_REG(MCUCTRL, TPIUCTRL) &= ~AM_REG_MCUCTRL_TPIUCTRL_ENABLE_M;
}

//*****************************************************************************
//
//! @brief Set the output port width of the TPIU
//!
//! @param ui32PortWidth - The desired port width (in bits)
//!
//! This function uses the TPIU_CSPSR register to set the desired output port
//! width of the TPIU.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_tpiu_port_width_set(uint32_t ui32PortWidth)
{
   AM_REG(TPIU, CSPSR) = 1 << (ui32PortWidth - 1);
}

//*****************************************************************************
//
//! @brief Read the supported_output port width of the TPIU
//!
//! This function uses the \e TPIU_SSPSR register to set the supported output
//! port widths of the TPIU.
//!
//! @return Current width of the TPIU output port
//
//*****************************************************************************
uint32_t
am_hal_tpiu_supported_port_width_get(void)
{
    uint32_t i, ui32WidthValue;

    //
    // Read the supported width register.
    //
    ui32WidthValue = AM_REG(TPIU, SSPSR);

    //
    // The register value is encoded in a one-hot format, so the position of
    // the single set bit determines the actual width of the port.
    //
    for (i = 1; i < 32; i++)
    {
        //
        // Check each bit for a '1'. When we find it, our current loop index
        // will be equal to the port width.
        //
        if (ui32WidthValue == (0x1 << (i - 1)))
        {
            return i;
        }
    }

    //
    // We should never get here, but if we do, just return the smallest
    // possible value for a supported trace port width.
    //
    return 1;
}

//*****************************************************************************
//
//! @brief Read the output port width of the TPIU
//!
//! This function uses the \e TPIU_CSPSR register to set the desired output
//! port width of the TPIU.
//!
//! @return Current width of the TPIU output port
//
//*****************************************************************************
uint32_t
am_hal_tpiu_port_width_get(void)
{
    uint32_t ui32Temp;
    uint32_t ui32Width;

    ui32Width = 1;
    ui32Temp = AM_REG(TPIU, CSPSR);

    while ( !(ui32Temp & 1) )
    {
        ui32Temp = ui32Temp >> 1;
        ui32Width++;

        if (ui32Width > 32)
        {
            ui32Width = 0;
            break;
        }
    }

    //
    // Current width of the TPIU output port.
    //
    return ui32Width;
}

//*****************************************************************************
//
//! @brief Configure the TPIU based on the values in the configuration struct.
//!
//! @param psConfig - pointer to an tTPIUConfig structure containing the
//! desired configuration information.
//!
//! This function reads the provided configuration structure, and sets the
//! relevant TPIU registers to achieve the desired configuration.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_tpiu_configure(tTPIUConfig *psConfig)
{
    //
    // Set the clock freq in the MCUCTRL register.
    //
    AM_REG(MCUCTRL, TPIUCTRL) |= psConfig->ui32TraceClkIn;

    //
    // Set the desired protocol.
    //
    AM_REG(TPIU, SPPR) = psConfig->ui32PinProtocol;

    //
    // Set the parallel port width. This may be redundant if the user has
    // selected a serial protocol, but we'll set it anyway.
    //
    AM_REG(TPIU, CSPSR) = (1 << (psConfig->ui32ParallelPortSize - 1));

    //
    // Set the clock prescaler.
    //
    AM_REG(TPIU, ACPR) = psConfig->ui32ClockPrescaler;
}

//*****************************************************************************
//
//! @brief Enables the TPIU
//!
//! This function enables the ARM TPIU by setting the TPIU registers and then
//! enabling the TPIU clock source in MCU control register.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_tpiu_enable(void)
{

    //
    // TPIU formatter & flush control register.
    //
    AM_REG(TPIU, FFCR) = 0;

    //
    // Now turn on the TPIU in UART mode at 1.0 MHz based on the HFRC.
    //
    AM_REG(TPIU, CSPSR) = 1;

    //
    // Divide TPIU clock by 3
    //
    AM_REG(TPIU, ACPR) = AM_REG_TPIU_ACPR_SWOSCALER(2);
    AM_REG(TPIU, SPPR) = AM_REG_TPIU_SPPR_TXMODE_UART;

    //
    // Make sure we are not in test mode.
    //
    AM_REG(TPIU, ITCTRL) = AM_REG_TPIU_ITCTRL_MODE_NORMAL;
    //
    // Enable the TPIU clock source in MCU control.
    //
    AM_REGn(MCUCTRL, 0, TPIUCTRL) = AM_REG_MCUCTRL_TPIUCTRL_CLKSEL_3MHZ |
                                    AM_REG_MCUCTRL_TPIUCTRL_ENABLE_EN;

    // wait for 50us for the data to flush out
    am_hal_itm_delay_us(50);
}

//*****************************************************************************
//
//! @brief Disables the TPIU
//!
//! This function disables the ARM TPIU by disabling the TPIU clock source
//! in MCU control register.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_tpiu_disable(void)
{
    //
    // Disable the TPIU clock source in MCU control.
    //
    AM_REG(MCUCTRL, TPIUCTRL) = AM_REG_MCUCTRL_TPIUCTRL_CLKSEL_0MHz |
                                AM_REG_MCUCTRL_TPIUCTRL_ENABLE_DIS;

}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
