//*****************************************************************************
//
//! @file ios_boot.c
//!
//! @brief I/O Slave (I2C or SPI) based boot loader for sensor hub like devices.
//!
//! This bootloader is intended to reside permanently in the beginning of flash
//! on an Apollo MCU used as a sensor hub like device, i.e. attached to a host
//! application processor.  The AP can download new applications to the flash
//! in the Apollo processor whenever it desires.
//!
//! This bootloader implementation support the I/O slave and be conditionally
//! compiled to use either I2C mode or SPI mode of the I/O slave.
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
#include "am_bootloader.h"


//*****************************************************************************
//
// Use a -D define to select compilation for I2C mode connection.
//
//*****************************************************************************
#ifdef IOS_BOOT_USES_I2C
#define USE_SPI_MODE 0
#else
#define USE_SPI_MODE 1
#endif

//*****************************************************************************
//
// I2C Address for the I/O Slave (this will not be used in SPI mode).
//
//*****************************************************************************
#define MY_I2C_ADDRESS 0x20

//*****************************************************************************
//
// Location of the flag page.
//
//*****************************************************************************
#define FLAG_PAGE_LOCATION                 0x00003C00
am_bootloader_image_t *g_psBootImage = (am_bootloader_image_t *) FLAG_PAGE_LOCATION;

//*****************************************************************************
//
// Default settings.
//
//*****************************************************************************
#define DEFAULT_OVERRIDE_GPIO               39
#define DEFAULT_OVERRIDE_POLARITY           AM_BOOTLOADER_OVERRIDE_LOW

#define INTERRUPT_PIN                       4

//*****************************************************************************
//
// Boot Loader Version Number
//
//*****************************************************************************
#define AM_BOOTLOADER_VERSION_NUM           0x03282016

//*****************************************************************************
//
// Declaring an image structure for use later.
//
//*****************************************************************************
am_bootloader_image_t g_sImage = {0, 0, 0, 0, 0, 0};

//*****************************************************************************
//
// Boot messages.
//
//*****************************************************************************
#define AM_BOOTLOADER_ACK                   0x00000000
#define AM_BOOTLOADER_NAK                   0x00000001
#define AM_BOOTLOADER_READY                 0x00000002
#define AM_BOOTLOADER_IMAGE_COMPLETE        0x00000003
#define AM_BOOTLOADER_BAD_CRC               0x00000004
#define AM_BOOTLOADER_ERROR                 0x00000005
#define AM_BOOTLOADER_BL_VERSION            0x00000006
#define AM_BOOTLOADER_FW_VERSION            0x00000007
#define AM_BOOTLOADER_DBG_READ              0x00000010
#define AM_BOOTLOADER_DBG_ECHO              0x00000011

//*****************************************************************************
//
// Boot Commands.
//
//*****************************************************************************
#define AM_BOOTLOADER_ACK_CMD               0x00000000
#define AM_BOOTLOADER_NAK_CMD               0x00000001
#define AM_BOOTLOADER_NEW_IMAGE             0x00000002
#define AM_BOOTLOADER_NEW_PACKET            0x00000003
#define AM_BOOTLOADER_RESET                 0x00000004
#define AM_BOOTLOADER_SET_OVERRIDE_CMD      0x00000005
#define AM_BOOTLOADER_BL_VERSION_CMD        0x00000006
#define AM_BOOTLOADER_FW_VERSION_CMD        0x00000007
#define AM_BOOTLOADER_DBG_READ_CMD          0x00000010
#define AM_BOOTLOADER_DBG_ECHO_CMD          0x00000011

//*****************************************************************************
//
// Message buffers.
//
// Note: The RX buffer needs to be 32-bit aligned to be compatible with the
// flash helper functions, but we also need an 8-bit pointer to it for copying
// data from the IOS interface, which is only 8 bits wide.
//
//*****************************************************************************
uint8_t g_pui8TxBuffer[8];
uint32_t g_pui32RxBuffer[512];
uint8_t *g_pui8RxBuffer = (uint8_t *) g_pui32RxBuffer;
uint32_t g_ui32BytesInBuffer;

//*****************************************************************************
//
// Globals to keep track of the image write state.
//
//*****************************************************************************
uint32_t *g_pui32WriteAddress = 0;
uint32_t g_ui32BytesReceived = 0;

bool g_bImageValid = false;

uint32_t g_ui32CRC = 0;

//*****************************************************************************
//
// SPI Slave Configuration
//
//*****************************************************************************
am_hal_ios_config_t g_sIOSConfigSPI =
{
    // Configure the IOS in SPI mode.
    .ui32InterfaceSelect = AM_HAL_IOS_USE_SPI | AM_HAL_IOS_SPIMODE_0,

    // Eliminate the "read-only" section, so an external host can use the
    // entire "direct write" section.
    .ui32ROBase = 0x78,

    // Set the FIFO base to the maximum value, making the "direct write"
    // section as big as possible.
    .ui32FIFOBase = 0x78,

    // We don't need any RAM space, so extend the FIFO all the way to the end
    // of the LRAM.
    .ui32RAMBase = 0x100,
};

//*****************************************************************************
//
// I2C Slave Configuration
//
//*****************************************************************************
am_hal_ios_config_t g_sIOSConfigI2C =
{
    // Configure the IOS in I2C mode.
    .ui32InterfaceSelect = (AM_HAL_IOS_USE_I2C | AM_HAL_IOS_I2C_ADDRESS(MY_I2C_ADDRESS)),

    // Eliminate the "read-only" section, so an external host can use the
    // entire "direct write" section.
    .ui32ROBase = 0x78,

    // Set the FIFO base to the maximum value, making the "direct write"
    // section as big as possible.
    .ui32FIFOBase = 0x78,

    // We don't need any RAM space, so extend the FIFO all the way to the end
    // of the LRAM.
    .ui32RAMBase = 0x100,
};

//*****************************************************************************
//
// Configure the I/O slave for either SPI or I2C mode for booting.
//
//*****************************************************************************
void
setup_the_slave(uint32_t SPIMode)
{
    if (SPIMode)
    {
        //
        // Set up SPI pins.
        //
        am_hal_gpio_pin_config(0, AM_HAL_PIN_0_SLSCK);
        am_hal_gpio_pin_config(1, AM_HAL_PIN_1_SLMISO);
        am_hal_gpio_pin_config(2, AM_HAL_PIN_2_SLMOSI);
        am_hal_gpio_pin_config(3, AM_HAL_PIN_3_SLnCE);
        am_hal_gpio_out_bit_set(INTERRUPT_PIN); //
        am_hal_gpio_pin_config(INTERRUPT_PIN, AM_HAL_PIN_OUTPUT);

        //
        // Configure the IOS interface and LRAM structure.
        //
        am_hal_ios_config(&g_sIOSConfigSPI);
    }
    else
    {
        //
        // Set up I2C pins.
        //
        am_hal_gpio_pin_config(0, AM_HAL_PIN_0_SLSCL);
        am_hal_gpio_pin_config(1, AM_HAL_PIN_1_SLSDA);
        am_hal_gpio_out_bit_set(INTERRUPT_PIN); //
        am_hal_gpio_pin_config(INTERRUPT_PIN, AM_HAL_PIN_OUTPUT);

        //
        // Configure the IOS interface and LRAM structure.
        //
        am_hal_ios_config(&g_sIOSConfigI2C);
    }

    //
    // Clear out any IOS register-access interrupts that may be active, and
    // enable interrupts for the registers we're interested in.
    //
    am_hal_ios_access_int_clear(AM_HAL_IOS_ACCESS_INT_ALL);
    am_hal_ios_access_int_enable(AM_HAL_IOS_ACCESS_INT_03);

    am_hal_ios_int_clear(AM_HAL_IOS_INT_ALL);
    am_hal_ios_int_enable(AM_HAL_IOS_INT_FSIZE);

    //
    // Set the bit in the NVIC to accept access interrupts from the IO Slave.
    //
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_IOSACC);
    am_hal_interrupt_enable(AM_HAL_INTERRUPT_IOSLAVE);
}


//*****************************************************************************
//
//! @brief Read an image start packet from the IOS LRAM
//!
//! @param psImage is the image structure to read the packet into.
//!
//! This function reads the IOS LRAM as a "new image" packet, and uses that
//! packet to fill in a bootloader image structure. The caller is responsible
//! for verifying the packet type before calling this function.
//!
//! @return true if the image parameters are acceptable.
//
//*****************************************************************************
bool
image_start_packet_read(am_bootloader_image_t *psImage)
{
    uint32_t *pui32Packet;

    pui32Packet = (uint32_t *) am_hal_ios_pui8LRAM;

    //
    // Set the image structure parameters based on the information in the
    // packet.
    //
    psImage->pui32LinkAddress = (uint32_t *)(pui32Packet[1]);
    psImage->ui32NumBytes = pui32Packet[2];
    psImage->ui32CRC = pui32Packet[3];
    psImage->ui32OverrideGPIO = DEFAULT_OVERRIDE_GPIO;
    psImage->ui32OverridePolarity = DEFAULT_OVERRIDE_POLARITY;

    //
    // Check to make sure we're not overwriting the bootloader.
    //
    if ((uint32_t) psImage->pui32LinkAddress < 0x4000)
    {
        return false;
    }

    //
    // Otherwise, the image is presumed to be reasonable. Set our global
    // variables based on the new image structure.
    //
    g_pui32WriteAddress = psImage->pui32LinkAddress;
    g_ui32BytesReceived = 0;
    return true;
}

//*****************************************************************************
//
//! @brief Read an image data packet from the IOS LRAM
//!
//! This function reads the IOS LRAM as a continuing data packet, and uses that
//! packet to write to the image area the flash. The caller is responsible for
//! verifying the packet type before calling this function.
//!
//! @return None.
//
//*****************************************************************************
void
image_data_packet_read(void)
{
    uint32_t i;
    uint8_t *pui8Src;
    uint32_t ui32Size, ui32CurrentPage, ui32CurrentBlock;

    //
    // Get the size information from the packet header, and set the src pointer
    // to the beginning of the actual data.
    //
    ui32Size = *((uint32_t *) (am_hal_ios_pui8LRAM + 4));
    pui8Src = (uint8_t *)(am_hal_ios_pui8LRAM + 8);

    //
    // Run a quick CRC on the received bytes, holding on to the result in a
    // global variable, so we can pick up where we left off on the next pass.
    //
    am_bootloader_partial_crc32(pui8Src, ui32Size, &g_ui32CRC);

    //
    // Loop through the data, copying it into the global buffer.
    //
    for (i = 0; i < ui32Size; i++)
    {
        g_pui8RxBuffer[g_ui32BytesInBuffer] = *pui8Src++;

        //
        // Keep track of how much data we've copied into the SRAM buffer.
        //
        g_ui32BytesInBuffer++;
        g_ui32BytesReceived++;

        //
        // Whenever we hit a page boundary or the end of the image, we should
        // write to flash.
        //
        if (g_ui32BytesInBuffer == 2048 ||
           g_ui32BytesReceived == g_sImage.ui32NumBytes)
        {
            //
            // Erase the next page in flash.
            //
            ui32CurrentPage = ((uint32_t) g_pui32WriteAddress / 2048) % 128;
            ui32CurrentBlock = (uint32_t) g_pui32WriteAddress / (128 * 2048);

            am_hal_flash_page_erase(AM_HAL_FLASH_PROGRAM_KEY,
                                    ui32CurrentBlock, ui32CurrentPage);

            //
            // Program the flash page with the data we just received.
            //
            am_hal_flash_program_main(AM_HAL_FLASH_PROGRAM_KEY, g_pui32RxBuffer,
                                      g_pui32WriteAddress, g_ui32BytesInBuffer);

            //
            // Adjust the global variables.
            //
            g_pui32WriteAddress += (g_ui32BytesInBuffer / 4);
            g_ui32BytesInBuffer = 0;
        }
    }
}

//*****************************************************************************
//
// IO Slave Main ISR.
//
//*****************************************************************************
void
am_ioslave_ios_isr(void)
{
    uint32_t ui32Status;

    //
    // Check to see what caused this interrupt, then clear the bit from the
    // interrupt register.
    //
    ui32Status = am_hal_ios_int_status_get(false);
    am_hal_ios_int_clear(ui32Status);

    //
    // Service the SPI slave FIFO if necessary.
    //
    am_hal_ios_fifo_service(ui32Status);
}

//*****************************************************************************
//
// IO Slave Register Access ISR.
//
//*****************************************************************************
void
am_ioslave_acc_isr(void)
{
    uint32_t ui32Status;
    uint32_t *pui32Packet;

    //
    // Set up a pointer for writing 32-bit aligned packets through the IO slave
    // interface.
    //
    pui32Packet = (uint32_t *) am_hal_ios_pui8LRAM;

    //
    // Check to see what caused this interrupt, then clear the bit from the
    // interrupt register.
    //
    ui32Status = am_hal_ios_access_int_status_get(false);
    am_hal_ios_access_int_clear(ui32Status);

    if (ui32Status & AM_HAL_IOS_ACCESS_INT_03)
    {
        am_hal_gpio_out_bit_set(INTERRUPT_PIN); //

        //
        // Figure out what to do next based on the packet header.
        //
        switch (pui32Packet[0])
        {
            case AM_BOOTLOADER_NEW_IMAGE:
                //
                // Parse the image packet, and store the result to the global
                // image structure.
                //
                g_bImageValid = image_start_packet_read(&g_sImage);

                //
                // Make sure the image packet had reasonable contents. If it
                // didn't, we need to let the host know.
                //
                if (g_bImageValid)
                {
                    //
                    // Good image; Send back a "READY" packet.
                    //
                    pui32Packet[0] = AM_BOOTLOADER_READY;
                    am_hal_gpio_out_bit_clear(INTERRUPT_PIN); //
                }
                else
                {
                    //
                    // Bad image; Send back an error.
                    //
                    pui32Packet[0] = AM_BOOTLOADER_ERROR;
                    am_hal_gpio_out_bit_clear(INTERRUPT_PIN); //
                }

                break;

            case AM_BOOTLOADER_SET_OVERRIDE_CMD:
                //
                // Set the override GPIO settings based on the packet
                // information.
                //
                g_sImage.ui32OverrideGPIO = pui32Packet[1];
                g_sImage.ui32OverridePolarity = pui32Packet[2];

                //
                // Send back a "READY" packet.
                //
                pui32Packet[0] = AM_BOOTLOADER_READY;
                am_hal_gpio_out_bit_clear(INTERRUPT_PIN); //
                break;

            case AM_BOOTLOADER_NEW_PACKET:
                //
                // Only take new packets if our image structure is valid.
                //
                if (!g_bImageValid)
                {
                    pui32Packet[0] = AM_BOOTLOADER_ERROR;
                    am_hal_gpio_out_bit_clear(INTERRUPT_PIN); //
                    break;
                }

                //
                // Parse the rest of the packet sitting in the IOS LRAM.
                //
                image_data_packet_read();

                //
                // If this packet completed the image...
                //
                if (g_ui32BytesReceived == g_sImage.ui32NumBytes)
                {
                    //
                    // Check this against the CRC we received from the host
                    // earlier. Report the status (either good or bad) back to
                    // the host.
                    //
                    pui32Packet[0] = ((g_ui32CRC == g_sImage.ui32CRC) ?
                                      AM_BOOTLOADER_IMAGE_COMPLETE :
                                      AM_BOOTLOADER_BAD_CRC);

                    //
                    // Assert the interrupt line so the host knows we have a
                    // message.
                    //
                    am_hal_gpio_out_bit_clear(INTERRUPT_PIN); //
                }
                else
                {
                    //
                    // If this wasn't the end of the image, just send back a
                    // "READY" packet.
                    //
                    pui32Packet[0] = AM_BOOTLOADER_READY;
                    am_hal_gpio_out_bit_clear(INTERRUPT_PIN); //
                }

                break;

            case AM_BOOTLOADER_RESET:
                //
                // Write the flag page.
                //
                am_bootloader_flag_page_update(&g_sImage, (uint32_t *)FLAG_PAGE_LOCATION);

                //
                // Perform a software reset.
                //
                //AM_REG(SYSCTRL, AIRCR) = (AM_REG_SYSCTRL_AIRCR_VECTKEY(0x5FA) |
                //                          AM_REG_SYSCTRL_AIRCR_SYSRESETREQ_M);
		// am_hal_reset_por(); // Normal system wide reset (minus shadow regs)
                am_hal_reset_poi();    // reset that also reloads shadow registers.

                //
                // Wait for the reset to take effect.
                //
                while (1);
                // never reach this break;

            case AM_BOOTLOADER_BL_VERSION_CMD:
                //
                // Respond with the version number.
                //
                pui32Packet[0] = AM_BOOTLOADER_BL_VERSION;
                pui32Packet[1] = AM_BOOTLOADER_VERSION_NUM;
                am_hal_gpio_out_bit_clear(INTERRUPT_PIN); //
                break;

#define BOOTLOADER_DEBUG
#ifdef BOOTLOADER_DEBUG
            case AM_BOOTLOADER_DBG_READ_CMD:
                am_hal_gpio_out_bit_clear(AM_BSP_GPIO_LED1);
                //
                // Respond with a well known debug string
                //
                pui32Packet[0]  = AM_BOOTLOADER_DBG_READ;
                {
                    int i;
                    char *echo = "The quick brown fox";
                    uint8_t *pSrc = (uint8_t *) echo;
                    uint8_t *pDst = (uint8_t *) &pui32Packet[1];
                    for (i = 0; i < 20; i++)
                    {
                        *pDst++  = *pSrc++;
                    }
                }
                am_hal_gpio_out_bit_clear(INTERRUPT_PIN);
                am_hal_gpio_out_bit_set(AM_BSP_GPIO_LED1);
                break;

            case AM_BOOTLOADER_DBG_ECHO_CMD:
                am_hal_gpio_out_bit_clear(AM_BSP_GPIO_LED2);
                //
                // Complement the bits in the message received and return it.
                //
                pui32Packet[0]  = AM_BOOTLOADER_DBG_ECHO;
                pui32Packet[1]  ^=  0xFFFFFFFF;
                pui32Packet[2]  ^=  0xFFFFFFFF;
                pui32Packet[3]  ^=  0xFFFFFFFF;
                pui32Packet[4]  ^=  0xFFFFFFFF;
                pui32Packet[5]  ^=  0xFFFFFFFF;
                pui32Packet[6]  ^=  0xFFFFFFFF;
                pui32Packet[7]  ^=  0xFFFFFFFF;
                pui32Packet[8]  ^=  0xFFFFFFFF;
                am_hal_gpio_out_bit_clear(INTERRUPT_PIN); //
                am_hal_gpio_out_bit_set(AM_BSP_GPIO_LED2);
                break;
#endif

            case AM_BOOTLOADER_ACK_CMD:
            case AM_BOOTLOADER_NAK_CMD:
                break;

            default:
                break;
        }
    }
}
//*****************************************************************************
//
// Main
//
//*****************************************************************************
int
main(void)
{

    //
    // Make sure we drive the INTERRUPT_PIN HIGH for synchronization purposes
    //
    am_hal_gpio_out_bit_set(INTERRUPT_PIN); //
    am_hal_gpio_pin_config(INTERRUPT_PIN, AM_HAL_PIN_OUTPUT);

    am_hal_gpio_out_bit_set(AM_BSP_GPIO_LED0);
    am_hal_gpio_pin_config(AM_BSP_GPIO_LED0, AM_HAL_PIN_OUTPUT);

    am_hal_gpio_out_bit_set(AM_BSP_GPIO_LED1);
    am_hal_gpio_pin_config(AM_BSP_GPIO_LED1, AM_HAL_PIN_OUTPUT);

    am_hal_gpio_out_bit_set(AM_BSP_GPIO_LED2);
    am_hal_gpio_pin_config(AM_BSP_GPIO_LED2, AM_HAL_PIN_OUTPUT);

    //
    // Check the flag page to see if there's a valid image ready.
    //
    if (am_bootloader_image_check(g_psBootImage))
    {
        am_hal_gpio_pin_config(AM_BSP_GPIO_LED0, AM_HAL_PIN_DISABLE);
        //
        // We are going to a program that was already successfully downloaded.
        // Turn off the INTERRUPT_PIN and restore things to reset state
        //
        am_hal_gpio_pin_config(INTERRUPT_PIN, AM_HAL_PIN_DISABLE);

        //
        // The preloaded image in flash looks valid so lets go there and never
        // come back here again.
        //
        am_bootloader_image_run(g_psBootImage);

        // should never return to this point.
    }

    //
    // If we get here, we didn't have a valid image in the flag page, so we
    // need to go get one. Speed up the clocks and start turning on
    // peripherals.
    //
    am_hal_clkgen_sysclk_select(AM_HAL_CLKGEN_SYSCLK_24MHZ);

    //
    // Configure the board for low power.
    //
    am_bsp_low_power_init();

    //
    // Start the the Slave interface in either I2C or SPI mode as previously
    // selected from conditional compilation.
    //
    setup_the_slave(USE_SPI_MODE);

    //
    // Signal to the host that we're ready to continue.
    //
    *((uint32_t *) am_hal_ios_pui8LRAM) = AM_BOOTLOADER_READY;
    am_hal_gpio_out_bit_clear(INTERRUPT_PIN); //

    //
    // Enable interrupts.
    //
    am_hal_interrupt_master_enable();

    //
    // Loop forever. All of the boot loader action happens in the access ISR.
    //
    while (1);
}
