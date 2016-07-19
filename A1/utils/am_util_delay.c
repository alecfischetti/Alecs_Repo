//*****************************************************************************
//
//! @file am_util_delay.c
//!
//! @brief A few useful delay functions.
//!
//! Functions for fixed delays.
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
#include "hal/am_hal_clkgen.h"
#include "am_util_delay.h"

//*****************************************************************************
//
//! @brief Delays for a desired amount of cycles.
//!
//! @param ui32Cycles - Desired number of cycles to delay for.
//!
//! This function will delay for a number of cycles.
//!
//! @note - the number of cycles this function takes to execute is
//! approximately ~3. Therefore, ui32Cycles will be ~3x what is passed.
//!
//! For example, a ui32Cycles value of 100 will delay for 300 cycles.
//!
//! @returns None
//
//*****************************************************************************
#ifdef gcc
void __attribute__((naked))
am_util_delay_cycles(uint32_t ui32Cycles)
{
    __asm("    subs    r0, #1\n"
          "    bne     am_util_delay_cycles\n"
          "    bx      lr");
}
#endif
#ifdef keil
__asm void
am_util_delay_cycles(uint32_t ui32Cycles)
{
    SUBS    R0, #1
    BNE     am_util_delay_cycles
    BX      LR
}
#endif
#ifdef iar
void
am_util_delay_cycles(uint32_t ui32Cycles)
{
    asm("SUBS    R0, #1");
    asm("BNE.N     am_util_delay_cycles");
    asm("BX      LR");
}
#endif

//*****************************************************************************
//
//! @brief Delays for a desired amount of milliseconds.
//!
//! @param ui32MilliSeconds - number of milliseconds to delay for.
//!
//! This function will delay for a number of milliseconds.
//!
//! @returns None
//
//*****************************************************************************
void
am_util_delay_ms(uint32_t ui32MilliSeconds)
{
    uint32_t ui32Cycles = ui32MilliSeconds * (am_hal_clkgen_sysclk_get() / 3000);

    //
    // Call the cycle delay
    //
    am_util_delay_cycles(ui32Cycles);
}

//*****************************************************************************
//
//! @brief Delays for a desired amount of microseconds.
//!
//! @param ui32MicroSeconds - number of microseconds to delay for.
//!
//! This function will delay for a number of microseconds.
//!
//! @returns None
//
//*****************************************************************************
void
am_util_delay_us(uint32_t ui32MicroSeconds)
{
    uint32_t ui32Cycles = ui32MicroSeconds * (am_hal_clkgen_sysclk_get() / 3000000);

    //
    // Call the cycle delay
    //
    am_util_delay_cycles(ui32Cycles);
}
