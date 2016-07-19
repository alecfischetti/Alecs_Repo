//*****************************************************************************
//
//! @file am_hal_mcuctrl.c
//!
//! @brief Functions for interfacing with the MCUCTRL.
//!
//! @addtogroup hal Hardware Abstraction Layer (HAL)
//! @addtogroup mcuctrl MCU Control (MCUCTRL)
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
// Global Variables.
//
//*****************************************************************************
const uint32_t am_hal_mcuctrl_flash_size[] =
{
    1 << 15,
    1 << 16,
    1 << 17,
    1 << 18,
    1 << 19,
    1 << 20,
    1 << 21
};

const uint32_t am_hal_mcuctrl_sram_size[] =
{
    1 << 15,
    1 << 16,
    1 << 17,
    1 << 18,
    1 << 19,
    1 << 20,
    1 << 21
};

//*****************************************************************************
//
//! @brief Gets all relevant device information.
//!
//! @param psDevice is a pointer to a structure that will be used to store all
//! device info.
//!
//! This function gets the device part number, chip IDs, and revision and
//! stores them in the passed structure.
//!
//! @return None
//
//*****************************************************************************
void
am_hal_mcuctrl_device_info_get(am_hal_mcuctrl_device_t *psDevice)
{
    //
    // Read the Part Number.
    //
    psDevice->ui32ChipPN = AM_REG(MCUCTRL, CHIP_INFO);

    //
    // Read the Chip ID0.
    //
    psDevice->ui32ChipID0 = AM_REG(MCUCTRL, CHIPID0);

    //
    // Read the Chip ID1.
    //
    psDevice->ui32ChipID1 = AM_REG(MCUCTRL, CHIPID1);

    //
    // Read the Chip Revision.
    //
    psDevice->ui32ChipRev = AM_REG(MCUCTRL, CHIPREV);

    //
    // Flash size from Part Number.
    //
    psDevice->ui32FlashSize =
        am_hal_mcuctrl_flash_size[AM_BFR(MCUCTRL, CHIP_INFO, FLASH)];

    //
    // SRAM size from Part Number.
    //
    psDevice->ui32SRAMSize =
        am_hal_mcuctrl_sram_size[AM_BFR(MCUCTRL, CHIP_INFO, RAM)];
}

//*****************************************************************************
//
//! @brief Enables the fault capture registers.
//!
//! This function enables the DCODEFAULTADDR and ICODEFAULTADDR registers.
//!
//! @return None
//
//*****************************************************************************
void
am_hal_mcuctrl_fault_capture_enable(void)
{
    //
    // Enable the Fault Capture registers.
    //
    AM_BFW(MCUCTRL, FAULTCAPTUREEN, ENABLE, 1);
}

//*****************************************************************************
//
//! @brief Disables the fault capture registers.
//!
//! This function disables the DCODEFAULTADDR and ICODEFAULTADDR registers.
//!
//! @return None
//
//*****************************************************************************
void
am_hal_mcuctrl_fault_capture_disable(void)
{
    //
    // Disable the Fault Capture registers.
    //
    AM_BFW(MCUCTRL, FAULTCAPTUREEN, ENABLE, 0);
}

//*****************************************************************************
//
//! @brief Gets the fault status and capture registers.
//!
//! @param psFault is a pointer to a structure that will be used to store all
//! fault info.
//!
//! This function gets the status of the ICODE, DCODE, and SYS bus faults and
//! the addresses associated with the fault.
//!
//! @return None
//
//*****************************************************************************
void
am_hal_mcuctrl_fault_status(am_hal_mcuctrl_fault_t *psFault)
{
    uint32_t ui32FaultStat;

    //
    // Read the Fault Status Register.
    //
    ui32FaultStat = AM_REG(MCUCTRL, FAULTSTATUS);
    psFault->bICODE = (ui32FaultStat & AM_REG_MCUCTRL_FAULTSTATUS_ICODE_M);
    psFault->bDCODE = (ui32FaultStat & AM_REG_MCUCTRL_FAULTSTATUS_DCODE_M);
    psFault->bSYS = (ui32FaultStat & AM_REG_MCUCTRL_FAULTSTATUS_SYS_M);

    //
    // Read the DCODE fault capture address register.
    //
    psFault->ui32DCODE = AM_REG(MCUCTRL, DCODEFAULTADDR);

    //
    // Read the ICODE fault capture address register.
    //
    psFault->ui32ICODE |= AM_REG(MCUCTRL, ICODEFAULTADDR);

    //
    // Read the ICODE fault capture address register.
    //
    psFault->ui32SYS |= AM_REG(MCUCTRL, SYSFAULTADDR);
}

//*****************************************************************************
//
//! @brief Set power state of the flash.
//!
//! @param ui32FlashPower is the desired flash power configuration.
//!
//! This function sets the device power state for the flash banks.
//!
//! Valid values for ui32FlashPower are:
//!
//!     AM_HAL_MCUCTRL_FLASH_POWER_DOWN_NONE
//!     AM_HAL_MCUCTRL_FLASH_POWER_DOWN_0
//!     AM_HAL_MCUCTRL_FLASH_POWER_DOWN_1
//!     AM_HAL_MCUCTRL_FLASH_POWER_DOWN_ALL
//!
//! @return None
//
//*****************************************************************************
void
am_hal_mcuctrl_flash_power_set(uint32_t ui32FlashPower)
{
    //
    // Write desired flash power state.
    //
    AM_REG(MCUCTRL, FLASHPWRDIS) = ui32FlashPower;
}

//*****************************************************************************
//
//! @brief Set power state of the SRAM.
//!
//! @param ui32SRAMPower is the desired SRAM power configuration.
//! @param ui32SRAMPowerDeepSleep is the desired SRAM power configuration in
//! deep sleep.
//!
//! This function sets the device power state for the SRAM banks.
//!
//! Valid values for ui32SRAMPower and ui32SRAMPowerDeepSleep are:
//!
//!     AM_HAL_MCUCTRL_SRAM_POWER_DOWN_NONE
//!     AM_HAL_MCUCTRL_SRAM_POWER_DOWN_1
//!     AM_HAL_MCUCTRL_SRAM_POWER_DOWN_2
//!     AM_HAL_MCUCTRL_SRAM_POWER_DOWN_3
//!     AM_HAL_MCUCTRL_SRAM_POWER_DOWN_4
//!     AM_HAL_MCUCTRL_SRAM_POWER_DOWN_5
//!     AM_HAL_MCUCTRL_SRAM_POWER_DOWN_6
//!     AM_HAL_MCUCTRL_SRAM_POWER_DOWN_7
//!     AM_HAL_MCUCTRL_SRAM_POWER_DOWN_ALL
//!
//! @return None
//
//*****************************************************************************
void
am_hal_mcuctrl_sram_power_set(uint32_t ui32SRAMPower,
                              uint32_t ui32SRAMPowerDeepSleep)
{
    //
    // Write desired SRAM power state.
    //
    AM_REG(MCUCTRL, SRAMPWRDIS) = ui32SRAMPower;

    //
    // Write desired SRAM deep sleep power state.
    //
    AM_REG(MCUCTRL, SRAMPWDINSLEEP) = ui32SRAMPowerDeepSleep;
}

//*****************************************************************************
//
//! @brief Enable the Bandgap.
//!
//! This function enables the Bandgap.
//!
//! @return None
//
//*****************************************************************************
void
am_hal_mcuctrl_bandgap_enable(void)
{
    //
    // Enable the Bandgap in the MCUCTRL.
    //
    AM_REG(MCUCTRL, BANDGAPEN) = AM_REG_MCUCTRL_BANDGAPEN_BGPEN_M;
}

//*****************************************************************************
//
//! @brief Disable the Bandgap.
//!
//! This function disables the Bandgap.
//!
//! @return None
//
//*****************************************************************************
void
am_hal_mcuctrl_bandgap_disable(void)
{
    //
    // Disable the Bandgap in the MCUCTRL.
    //
    AM_REG(MCUCTRL, BANDGAPEN) = ~AM_REG_MCUCTRL_BANDGAPEN_BGPEN_M;
}

//*****************************************************************************
//
//! @brief Enable the core and memory buck converters.
//!
//! This function enables the core and memory buck converters.
//!
//! @return None
//
//*****************************************************************************
void
am_hal_mcuctrl_bucks_enable(void)
{
    //
    // Enable the core buck converter in the MCUCTRL.
    //
    AM_BFW(MCUCTRL, SUPPLYSRC, COREBUCKEN, 1);

    //
    // Enable the SRAM buck converter in the MCUCTRL.
    //
    AM_BFW(MCUCTRL, SUPPLYSRC, MEMBUCKEN, 1);

    //
    // Poll until core buck is enabled.
    //
    while( !AM_BFR(MCUCTRL, SUPPLYSTATUS, COREBUCKON) );

    //
    // Poll until SRAM buck is enabled.
    //
    while( !AM_BFR(MCUCTRL, SUPPLYSTATUS, MEMBUCKON) );
}

//*****************************************************************************
//
//! @brief Disable the core and memory buck converters.
//!
//! This function disables the core and memory buck converters.
//!
//! @return None
//
//*****************************************************************************
void
am_hal_mcuctrl_bucks_disable(void)
{
    //
    // Disable the core buck converter in the MCUCTRL.
    //
    AM_BFW(MCUCTRL, SUPPLYSRC, COREBUCKEN, 0);

    //
    // Disable the SRAM buck converter in the MCUCTRL.
    //
    AM_BFW(MCUCTRL, SUPPLYSRC, MEMBUCKEN, 0);
}


//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
