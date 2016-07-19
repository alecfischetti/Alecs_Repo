//*****************************************************************************
//
//! @file am_hal_tpiu.h
//!
//! @brief Definitions and structures for working with the TPIU.
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
#ifndef AM_HAL_TPIU_H
#define AM_HAL_TPIU_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// TPIU register defines.
//
//*****************************************************************************
#define AM_HAL_TPIU_SSPSR       0xE0040000  //! Supported Parallel Port Sizes
#define AM_HAL_TPIU_CSPSR       0xE0040004  //! Current Parallel Port Size
#define AM_HAL_TPIU_ACPR        0xE0040010  //! Asynchronous Clock Prescaler
#define AM_HAL_TPIU_SPPR        0xE00400F0  //! Selected Pin Protocol
#define AM_HAL_TPIU_TYPE        0xE0040FC8  //! TPIU Type

//*****************************************************************************
//
// TPIU ACPR defines.
//
//*****************************************************************************
#define AM_HAL_TPIU_ACPR_SWOSCALER_M    0x0000FFFF  //! SWO baud rate prescalar

//*****************************************************************************
//
// TPIU_SPPR TXMODE defines.
//
//*****************************************************************************
#define AM_HAL_TPIU_SPPR_PARALLEL       0x00000000
#define AM_HAL_TPIU_SPPR_MANCHESTER     0x00000001
#define AM_HAL_TPIU_SPPR_NRZ            0x00000002

//*****************************************************************************
//
// TPIU Type defines
//
//*****************************************************************************
#define AM_HAL_TPIU_TYPE_NRZVALID       0x00000800
#define AM_HAL_TPIU_TYPE_MANCVALID      0x00000400
#define AM_HAL_TPIU_TYPE_PTINVALID      0x00000200
#define AM_HAL_TPIU_TYPE_FIFOSZ_M       0x000001C0

//*****************************************************************************
//
// TPIU Clock defines
//
//*****************************************************************************
#define AM_HAL_TPIU_TRACECLKIN_6MHZ     AM_REG_MCUCTRL_TPIUCTRL_TPIUCLKSEL(0)
#define AM_HAL_TPIU_TRACECLKIN_3MHZ     AM_REG_MCUCTRL_TPIUCTRL_TPIUCLKSEL(1)
#define AM_HAL_TPIU_TRACECLKIN_1_5MHZ   AM_REG_MCUCTRL_TPIUCTRL_TPIUCLKSEL(2)
#define AM_HAL_TPIU_TRACECLKIN_750KHZ   AM_REG_MCUCTRL_TPIUCTRL_TPIUCLKSEL(3)

//*****************************************************************************
//
//! @brief Structure used for configuring the TPIU
//
//*****************************************************************************
typedef struct
{
    //
    //! MCU Control TRACECLKIN clock freq.
    //!
    //! Valid values for ui32TraceClkIn are:
    //!
    //!     AM_HAL_TPIU_TRACECLKIN_6MHZ
    //!     AM_HAL_TPIU_TRACECLKIN_3MHZ
    //!     AM_HAL_TPIU_TRACECLKIN_1_5MHZ
    //!     AM_HAL_TPIU_TRACECLKIN_750KHZ
    //
    uint32_t ui32TraceClkIn;

    //
    //! Protocol to use for the TPIU
    //!
    //! Valid values for ui32PinProtocol are:
    //!
    //!     AM_HAL_TPIU_SPPR_PARALLEL
    //!     AM_HAL_TPIU_SPPR_MANCHESTER
    //!     AM_HAL_TPIU_SPPR_NRZ
    //
    uint32_t ui32PinProtocol;

    //
    //! Desired width of the TPIU parallel port
    //
    uint32_t ui32ParallelPortSize;

    //
    //! Desired Clock prescaler value
    //
    uint32_t ui32ClockPrescaler;
}
tTPIUConfig;

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************
extern void am_hal_tpiu_clock_enable(void);
extern void am_hal_tpiu_clock_disable(void);
extern void am_hal_tpiu_port_width_set(uint32_t ui32PortWidth);
extern uint32_t am_hal_tpiu_supported_port_width_get(void);
extern uint32_t am_hal_tpiu_port_width_get(void);
extern void am_hal_tpiu_configure(tTPIUConfig *psConfig);
extern void am_hal_tpiu_enable(void);
extern void am_hal_tpiu_disable(void);

#ifdef __cplusplus
}
#endif

#endif // AM_HAL_TPIU_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
