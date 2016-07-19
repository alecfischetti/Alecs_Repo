//*****************************************************************************
//
//! @file spiflash_example.c
//!
//! @brief This example demonstrates basic operation with the spiflash module.
//!
//! This example attempts to perform a series of read and write operations to
//! an external spiflash. Success or failure of each operation is reported over
//! ITM.
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

// Number of addresses in the spiflash to write
#define NUM_ADDR               128

unsigned char g_pucOutBuffer[NUM_ADDR];
unsigned char g_pucInBuffer[NUM_ADDR];

//*****************************************************************************
//
// Configuration structure for the IO Master.
//
//*****************************************************************************
am_hal_iom_config_t g_sIOMConfig =
{
    .ui32InterfaceMode = AM_HAL_IOM_SPIMODE,
    .ui32ClockFrequency = AM_HAL_IOM_4MHZ,
    .bSPHA = 0,
    .bSPOL = 0,
    .ui8WriteThreshold = 0,
    .ui8ReadThreshold = 60,
};

//*****************************************************************************
//
// Device structure for the SPI flash.
//
//*****************************************************************************
am_devices_spiflash_t g_sSpiFlash =
{
    .ui32IOMModule = AM_BSP_FLASH_IOM,
    .ui32ChipSelect = AM_BSP_FLASH_CS,
};

//*****************************************************************************
//
// Configure GPIOs for communicating with a SPI flash
//
//*****************************************************************************
void
configure_spiflash_pins(void)
{
    //
    // Set up IOM1 SPI pins.
    //
    am_bsp_iom_spi_pins_enable(AM_BSP_FLASH_IOM);

    //
    // Enable the chip-select and data-ready pins for the SPI FLASH
    //
    am_bsp_pin_enable(FLASH_CS);

    //
    // Setup ITM pin for plotting
    //
    am_bsp_pin_enable(ITM_SWO);
}

//*****************************************************************************
//
// Short function for running examples and printing results
//
//*****************************************************************************
void
check_return(char *pcExampleName, bool bAssertion)
{
    am_util_stdio_printf(pcExampleName);
    am_util_stdio_printf(": ");

    if (bAssertion)
    {
        am_util_stdio_printf("PASS\r\n");
    }
    else
    {
        am_util_stdio_printf("FAIL\r\n");
    }
}

//*****************************************************************************
//
// Main function.
//
//*****************************************************************************
int
main(void)
{
    static volatile uint32_t ui32Status;
    uint8_t i;
    bool bAssertion;

    //
    // Set the clock frequency
    //
    am_hal_clkgen_sysclk_select(AM_HAL_CLKGEN_SYSCLK_MAX);

    //
    // Initialize the board.
    //
    am_bsp_low_power_init();

    //
    // Configure IOM1 for talking with the SPI FLASH.
    //
    am_hal_iom_config(AM_BSP_FLASH_IOM, &g_sIOMConfig);


    // Configure pins for this example
    configure_spiflash_pins();

    //
    // Initialize the spiflash driver with the IOM information for the second
    // flash device.
    //
    am_devices_spiflash_init(&g_sSpiFlash);

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
    // Clear the terminal, and print a message to show that we're up and
    // running.
    //
    am_util_stdio_terminal_clear();
    am_util_stdio_printf("SPI Flash Device Example:\n\r\n");

    //
    // Turn on the IOM for this operation.
    //
    am_hal_iom_enable(AM_BSP_FLASH_IOM);

    //
    // Flash ID
    //
    ui32Status = am_devices_spiflash_id();
    check_return("Checking ID", ui32Status == 0x0018BA20);

    //
    // Flash ui32Status
    //
    ui32Status = am_devices_spiflash_status();
    check_return("Checking Status", ui32Status == 0x0);

    //
    // Program (and read)
    //
    for (i = 0; i < NUM_ADDR; i++)
    {
        g_pucOutBuffer[i] = i ^ 0x55;
    }
    am_util_stdio_printf("Erasing..... \r\n");
    am_devices_spiflash_mass_erase();
    am_devices_spiflash_write(g_pucOutBuffer, 0, NUM_ADDR);
    am_devices_spiflash_read(g_pucInBuffer, 0, NUM_ADDR);
    bAssertion = true;
    for (i = 0; i < NUM_ADDR; i++)
    {
        if (g_pucInBuffer[i] != (i ^ 0x55))
        {
            bAssertion = false;
        }
    }
    check_return("Program example", bAssertion);

    //
    // Page Erase (and read)
    //
    am_devices_spiflash_sector_erase(0);
    am_devices_spiflash_read(g_pucInBuffer, 0, NUM_ADDR);
    bAssertion = true;
    for (i = 0; i < NUM_ADDR; i++)
    {
        if (g_pucInBuffer[i] != 0xFF)
        {
            bAssertion = false;
        }
    }
    check_return("Page erase example", bAssertion);

    //
    // Mass erase (and read)
    //
    am_util_stdio_printf("Mass erase example...\r");
    am_devices_spiflash_mass_erase();
    am_devices_spiflash_read(g_pucInBuffer, 0, NUM_ADDR);
    bAssertion = true;
    for (i = 0; i < NUM_ADDR; i++)
    {
        if (g_pucInBuffer[i] != 0xFF)
        {
            bAssertion = false;
        }
    }
    check_return("Mass erase example", bAssertion);

    am_util_stdio_printf("Operations complete. \n");

    //
    // Endless loop.
    //
    while(1)
    {
    }
}
