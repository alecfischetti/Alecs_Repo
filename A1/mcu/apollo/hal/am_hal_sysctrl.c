//*****************************************************************************
//
//! @file am_hal_sysctrl.c
//!
//! @brief Functions for interfacing with the M4F system control registers
//!
//! @addtogroup hal Hardware Abstraction Layer (HAL)
//! @addtogroup sysctrl System Control (SYSCTRL)
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
//! @brief Place the core into sleep or deepsleep.
//!
//! @param bSleepDeep - False for Normal or True Deep sleep.
//!
//! This function puts the MCU to sleep or deepsleep depending on bSleepDeep.
//!
//! Valid values for bSleepDeep are:
//!
//!     AM_HAL_SYSCTRL_SLEEP_NORMAL
//!     AM_HAL_SYSCTRL_SLEEP_DEEP
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_sysctrl_sleep(bool bSleepDeep)
{
    //
    // Sleep or Deepsleep?
    //
    if(bSleepDeep)
    {
       //
       // If TPIU is not enabled, go to deepsleep.
       // else go to normal sleep.
       //
       if ( (AM_REG(MCUCTRL, TPIUCTRL) & ~AM_REG_MCUCTRL_TPIUCTRL_ENABLE_EN) ==
             AM_REG_MCUCTRL_TPIUCTRL_ENABLE_DIS )
       {
          AM_REG(SYSCTRL, SCR) |= AM_REG_SYSCTRL_SCR_SLEEPDEEP_M;
       }
       else
       {
          AM_REG(SYSCTRL, SCR) &= ~AM_REG_SYSCTRL_SCR_SLEEPDEEP_M;
       }
    }
    else
    {
       AM_REG(SYSCTRL, SCR) &= ~AM_REG_SYSCTRL_SCR_SLEEPDEEP_M;
    }

    //
    // Go to sleep.
    //
#if defined(__ARMCC_VERSION)
    __wfi();            // ARM/Keil intrinsic
#else
    __asm("    wfi");   // GCC version
#endif
}

//*****************************************************************************
//
//! @brief Enable the floating point module.
//!
//! Call this function to enable the ARM hardware floating point module.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_sysctrl_fpu_enable(void)
{
    //
    // Enable access to the FPU in both privileged and user modes.
    // NOTE: Write 0s to all reserved fields in this register.
    //
    AM_REG(SYSCTRL, CPACR) = (AM_REG_SYSCTRL_CPACR_CP11(0x3) |
                             AM_REG_SYSCTRL_CPACR_CP10(0x3));
}

//*****************************************************************************
//
//! @brief Disable the floating point module.
//!
//! Call this function to disable the ARM hardware floating point module.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_sysctrl_fpu_disable(void)
{
    //
    // Disable access to the FPU in both privileged and user modes.
    // NOTE: Write 0s to all reserved fields in this register.
    //
    AM_REG(SYSCTRL, CPACR) = 0x00000000                     &
                          ~(AM_REG_SYSCTRL_CPACR_CP11(0x3) |
                            AM_REG_SYSCTRL_CPACR_CP10(0x3));
}

//*****************************************************************************
//
//! @brief Enable stacking of FPU registers on exception entry.
//!
//! @param bLazy - Set to "true" to enable "lazy stacking".
//!
//! This function allows the core to save floating-point information to the
//! stack on exception entry. Setting the bLazy option enables "lazy stacking"
//! for interrupt handlers.  Normally, mixing floating-point code and interrupt
//! driven routines causes increased interrupt latency, because the core must
//! save extra information to the stack upon exception entry. With the lazy
//! stacking option enabled, the core will skip the saving of floating-point
//! registers when possible, reducing average interrupt latency.
//!
//! @note This function should be called before the floating-point module is
//! used in interrupt-driven code. If it is not called, the core will not have
//! any way to save context information for floating-point variables on
//! exception entry.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_sysctrl_fpu_stacking_enable(bool bLazy)
{
    if(bLazy)
    {
        //
        // Enable automatic saving of FPU registers on exception entry, using lazy
        // context saving.
        //
        AM_REG(SYSCTRL, FPCCR) |= (AM_REG_SYSCTRL_FPCCR_ASPEN(0x1) |
                                   AM_REG_SYSCTRL_FPCCR_LSPEN(0x1));
    }
    else
    {
        //
        // Enable automatic saving of FPU registers on exception entry.
        //
        AM_REG(SYSCTRL, FPCCR) |= AM_REG_SYSCTRL_FPCCR_ASPEN(0x1);
    }
}

//*****************************************************************************
//
//! @brief Disable FPU register stacking on exception entry.
//!
//! This function disables all stacking of floating point registers for
//! interrupt handlers.
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_sysctrl_fpu_stacking_disable(void)
{
    //
    // Enable automatic saving of FPU registers on exception entry, using lazy
    // context saving.
    //
    AM_REG(SYSCTRL, FPCCR) &= ~(AM_REG_SYSCTRL_FPCCR_ASPEN(0x1) |
                                AM_REG_SYSCTRL_FPCCR_LSPEN(0x1));
}

//*****************************************************************************
//
//! @brief Issue a system wide reset using the AIRCR bit in the M4 system ctrl.
//!
//! This function issues a system wide reset (Apollo POR level reset).
//!
//! @return None.
//
//*****************************************************************************
void
am_hal_sysctrl_reset(void)
{
    //
    // Set the system reset bit in the AIRCR register
    //
    AM_REG(SYSCTRL, AIRCR) = AM_REG_SYSCTRL_AIRCR_VECTKEY(0x5FA) |
                             AM_REG_SYSCTRL_AIRCR_SYSRESETREQ(1);
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
