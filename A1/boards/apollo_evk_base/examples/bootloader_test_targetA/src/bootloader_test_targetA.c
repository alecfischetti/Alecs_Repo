//*****************************************************************************
//
//! @file bootloader_targetA.c
//!
//! @brief A simple "Hello World" style example to be downloaded.
//!
//! bootloader_targetA is an example that is intended to be downloaded by a
//! boot host into an Apollo chip running either ios_boot or the secure
//! boot loader. It is set to build at 0x8000 instead of 0x0.
//!
//! This example prints a "Hello World" style message marked TARGET A 
//! over SWO at 1M baud. To see the output of this program, run AMFlash,
//! and configure the console for SWO. The example sleeps after it is done
//! printing.
//!
//! It generates a different repeating output message than targetB.
//!
//! bootloader_targetA is an example that is intended to be downloaded by a
//! boot host into an Apollo chip running either ios_boot or the secure
//! boot loader. It is set to build at 0x8000 instead of 0x0.
//! bootloader_targetA is an example that is intended to be downloaded by a
//! boot host into an Apollo chip running either ios_boot or the secure
//! boot loader. It is set to build at 0x8000 instead of 0x0.
//!
//! Use the bash script generate_boot_image.sh to get an OTA file for download.
//! 
//! bootloader_targetA and bootloader_targetB are nearly identical programs.
//! The host program should download targetA into a fresh Apollo MCU with a 
//! brand new boot loader in it. After a reset it should be continuously
//! printing "TARGET A: I was downloaded via the boot loader". In order to 
//! test the host ability to over ride an existing program, one should
//! download bootloader_targetB to get "TARGET B: I was downloaded via the 
//! boot loader" in order to confirm a successful update operation.
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
#include "am_util.h"

//*****************************************************************************
//
// Main
//
//*****************************************************************************
int
main(void)
{
    uint32_t ui32LoopCount = 0;
    //
    // Set the clock frequency.
    //
    am_hal_clkgen_sysclk_select(AM_HAL_CLKGEN_SYSCLK_24MHZ);

    //
    // Initialize the BSP.
    //
    am_bsp_low_power_init();

    //
    // Initialize the printf interface for ITM/SWO output.
    //
    am_util_stdio_printf_init((am_util_stdio_print_char_t) am_hal_itm_print);

    //
    // Initialize the SWO GPIO pin
    //
    am_bsp_pin_enable(ITM_SWO);

    //
    // Enable the ITM.
    //
    am_hal_itm_enable();

    //
    // Enable debug printf messages using ITM on SWO pin
    //
    am_bsp_debug_printf_enable();

    //
    // Print the banner.
    //
    am_util_stdio_terminal_clear();

    //
    // Loop forever
    //
    while (1)
    {
        am_util_stdio_printf("TARGET A: I was downloaded via the boot loader %d\n\n",
                             ui32LoopCount++);
    }
}
