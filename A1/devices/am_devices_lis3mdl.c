//*****************************************************************************
//
//! @file am_devices_lis3mdl.c
//!
//! @brief Generic lis3mdl driver.
//!
//! @addtogroup devices External Device Control Library
//! @addtogroup lis3mdl SPI Device Control for the LIS3MDL External Magnetometer 
//! @ingroup devices
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

#include "am_mcu_apollo.h"
#include "am_devices_lis3mdl.h"

//*****************************************************************************
//
//! @brief Reset the LIS3MDL mag
//!
//! @param psDevice is a pointer to a device structure describing the LIS3MDL
//!
//! This function resets the LIS3MDL gryo.
//!
//! @return None.
//
//*****************************************************************************
void 
am_devices_lis3mdl_reset(am_devices_lis3mdl_t *psDevice)
{
    //
    // Reset the LIS3MDL Mag
    //
    am_devices_lis3mdl_reg_write(psDevice, AM_DEVICES_LIS3MDL_CTRL_REG2, 0x0C);
}

//*****************************************************************************
//
//! @brief Get the device ID
//!
//! @param psDevice is a pointer to a device structure describing the LIS3MDL
//!
//! This function reads the device ID register and returns the result. The
//! LIS3MDL should return 0x3D when functioning correctly.
//!
//! @return ID value.
//
//*****************************************************************************
uint8_t 
am_devices_lis3mdl_device_id_get(am_devices_lis3mdl_t *psDevice)
{
    //
    // Read the WHO_AM_I register and return the result.
    //
    return am_devices_lis3mdl_reg_read(psDevice, AM_DEVICES_LIS3MDL_WHO_AM_I);
}

//*****************************************************************************
//
//! @brief Configures the LIS3MDL for operation.
//!
//! @param psDevice is a pointer to a device structure describing the LIS3MDL.
//!
//! This function performs a basic, default configuration for the LIS3MDL.
//!
//! @return None.
//
//*****************************************************************************
void
am_devices_lis3mdl_config(am_devices_lis3mdl_t *psDevice)
{
    am_hal_lis3mdl_regs(6) sRegBuffer;

    //
    // Reset the device
    //
    am_devices_lis3mdl_reset(psDevice);

    sRegBuffer.bytes[0] = 0x10;     // CTRL_REG1
    sRegBuffer.bytes[1] = 0x00;     // CTRL_REG2
    sRegBuffer.bytes[2] = 0x00;     // CTRL_REG3
    sRegBuffer.bytes[3] = 0x00;     // CTRL_REG4
    sRegBuffer.bytes[4] = 0x00;     // CTRL_REG5

    //
    // Write the control registers to the device as one solid block write.
    //
    am_devices_lis3mdl_reg_block_write(psDevice, AM_DEVICES_LIS3MDL_CTRL_REG1,
                                       sRegBuffer.words, 5, 0);

    //
    // Clear out any old data that might be in the mag.
    //
    am_devices_lis3mdl_sample_get(psDevice, sRegBuffer.words, 0);
}

//*****************************************************************************
//
//! @brief Retrieves the most recent sample from the LIS3MDL.
//!
//! @param psDevice is a pointer to a device structure describing the LIS3MDL.
//! @param psData is the location where this function will place the data.
//! @param pfnCallback is an optional callback function.
//!
//! This function reads the Magnetometer sample registers in the LIS3MDL, and
//! places the resulting samples into the caller-supplied buffer. If a callback
//! function is supplied, this function will use the am_hal_iom_spi_write_nb()
//! call to perform the transfer, and the caller's callback function will be
//! called upon completion.
//!
//! @note This function will write exactly 8 bytes of data to the location
//! pointed to by psData. The caller must make sure that psData is large enough
//! to hold this data.
//!
//! @return None.
//
//*****************************************************************************
void
am_devices_lis3mdl_sample_get(am_devices_lis3mdl_t *psDevice, uint32_t *psData,
                              am_hal_iom_callback_t pfnCallback)
{
    //
    // Read the magnetometer registers as a single block.
    //
    am_devices_lis3mdl_reg_block_read(psDevice, AM_DEVICES_LIS3MDL_OUT_X_L,
                                      psData, 6, pfnCallback);
}

//*****************************************************************************
//
//! @brief Reads an internal register in the LIS3MDL.
//!
//! @param psDevice is a pointer to a device structure describing the LIS3MDL.
//! @param ui32Register is the address of the register to read.
//! @param ui32Value is the value to read to the register.
//!
//! This function performs a read to an LIS3MDL register over the serial bus.
//!
//! @return
//
//*****************************************************************************
uint8_t
am_devices_lis3mdl_reg_read(am_devices_lis3mdl_t *psDevice,
                            uint8_t ui8Register)
{
    uint8_t ui8Offset;
    am_hal_iom_buffer(1) sData;

    //
    // Build the SPI offset and the data buffer.
    //
    ui8Offset = 0x80 | ui8Register;

    //
    // Send the read to the bus using the polled API.
    //
    am_hal_iom_spi_read(psDevice->ui32IOMModule, psDevice->ui32ChipSelect,
                        sData.words, 1, AM_HAL_IOM_OFFSET(ui8Offset));

    //
    // Return the retrieved data.
    //
    return sData.bytes[0];
}

//*****************************************************************************
//
//! @brief Reads a block of internal registers in the LIS3MDL.
//!
//! @param psDevice is a pointer to a device structure describing the LIS3MDL.
//! @param ui32StartRegister is the address of the first register to read.
//! @param pui32Values is the byte-packed array where the read data will go.
//! @param ui32NumBytes is the total number of 8-bit registers to read.
//! @param pfnCallback is an optional callback function pointer.
//!
//! This function performs a read to a block of LIS3MDL registers over the
//! serial bus. If the \e pfnCallback parameter is nonzero, this function will
//! use the am_hal_iom_spi_read_nb() function as the underlying interface, and
//! \e pfnCallback will be provided to the HAL as the IOM callback function.
//! Otherwise, the SPI read will be polled.
//!
//! @return
//
//*****************************************************************************
void
am_devices_lis3mdl_reg_block_read(am_devices_lis3mdl_t *psDevice,
                                  uint8_t ui8StartRegister,
                                  uint32_t *pui32Values,
                                  uint32_t ui32NumBytes,
                                  am_hal_iom_callback_t pfnCallback)
{
    uint8_t ui8Offset;

    //
    // Build the SPI offset for writing a block of registers from the
    // user-supplied start point.
    //
    ui8Offset = 0xC0 | ui8StartRegister;

    //
    // Check to see if the callback pointer is valid.
    //
    if(pfnCallback)
    {
        //
        // If so, use a non-blocking call with a callback.
        //
        am_hal_iom_spi_read_nb(psDevice->ui32IOMModule,
                               psDevice->ui32ChipSelect,
                               pui32Values, ui32NumBytes,
                               AM_HAL_IOM_OFFSET(ui8Offset),
                               pfnCallback);
    }
    else
    {
        //
        // Otherwise, use a polled call.
        //
        am_hal_iom_spi_read(psDevice->ui32IOMModule,
                            psDevice->ui32ChipSelect,
                            pui32Values, ui32NumBytes,
                            AM_HAL_IOM_OFFSET(ui8Offset));
    }
}

//*****************************************************************************
//
//! @brief Writes an internal register in the LIS3MDL.
//!
//! @param psDevice is a pointer to a device structure describing the LIS3MDL.
//! @param ui32Register is the address of the register to write.
//! @param ui32Value is the value to write to the register.
//!
//! This function performs a write to an LIS3MDL register over the serial bus.
//!
//! @return
//
//*****************************************************************************
void
am_devices_lis3mdl_reg_write(am_devices_lis3mdl_t *psDevice,
                             uint8_t ui8Register, uint8_t ui8Value)
{
    uint8_t ui8Offset;
    am_hal_iom_buffer(1) sData;

    //
    // Build the SPI offset and the data buffer.
    //
    ui8Offset = ui8Register;
    sData.bytes[0] = ui8Value;

    //
    // Send the write to the bus using the polled API.
    //
    am_hal_iom_spi_write(psDevice->ui32IOMModule, psDevice->ui32ChipSelect,
                         sData.words, 1, AM_HAL_IOM_OFFSET(ui8Offset));
}

//*****************************************************************************
//
//! @brief Writes a block of internal registers in the LIS3MDL.
//!
//! @param psDevice is a pointer to a device structure describing the LIS3MDL.
//! @param ui32StartRegister is the address of the first register to write.
//! @param pui32Values is the byte-packed array of data to write.
//! @param ui32NumBytes is the total number of registers to write.
//! @param pfnCallback is an optional callback function pointer.
//!
//! This function performs a write to a block of LIS3MDL registers over the
//! serial bus. If the \e pfnCallback parameter is nonzero, this function will
//! use the am_hal_iom_spi_write_nb() function as the underlying interface, and
//! \e pfnCallback will be provided to the HAL as the IOM callback function.
//! Otherwise, the spi write will be polled.
//!
//! @return
//
//*****************************************************************************
void
am_devices_lis3mdl_reg_block_write(am_devices_lis3mdl_t *psDevice,
                                   uint8_t ui8StartRegister,
                                   uint32_t *pui32Values,
                                   uint32_t ui32NumBytes,
                                   am_hal_iom_callback_t pfnCallback)
{
    uint8_t ui8Offset;

    //
    // Build the SPI offset for writing a block of registers from the
    // user-supplied start point.
    //
    ui8Offset = 0x40 | ui8StartRegister;

    //
    // Check to see if the callback pointer is valid.
    //
    if(pfnCallback)
    {
        //
        // If so, use a non-blocking call with a callback.
        //
        am_hal_iom_spi_write_nb(psDevice->ui32IOMModule,
                                psDevice->ui32ChipSelect,
                                pui32Values, ui32NumBytes,
                                AM_HAL_IOM_OFFSET(ui8Offset),
                                pfnCallback);
    }
    else
    {
        //
        // Otherwise, use a polled call.
        //
        am_hal_iom_spi_write(psDevice->ui32IOMModule,
                             psDevice->ui32ChipSelect,
                             pui32Values, ui32NumBytes,
                             AM_HAL_IOM_OFFSET(ui8Offset));
    }
}

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
