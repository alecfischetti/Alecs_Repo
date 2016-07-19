//*****************************************************************************
//
//! @file am_devices_lis2dh12.c
//!
//! @brief Driver to interface with the LIS2DH12
//!
//! These functions implement the LIS2DH12 support routines for use on Ambiq
//! Micro MCUs.
//!
//! @addtogroup devices External Device Control Library
//! @addtogroup lis2dh12 SPI Device Control for the LIS2DH12 External Accelerometer
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
#include "am_bsp.h"
#include "am_util.h"

#include "am_devices_lis2dh12.h"

//*****************************************************************************
//
//! @brief Reset the LIS2DH12 accel
//!
//! @param psDevice is a pointer to a device structure describing the LIS2DH12
//!
//! This function resetes the LIS2DH12 accel.
//!
//! @return None.
//
//*****************************************************************************
void
am_devices_lis2dh12_reset(am_devices_lis2dh12_t *psDevice)
{
    am_hal_lis2dh12_regs(6) sRegBuffer;
    uint8_t ui8FifoCnt;
    uint32_t ui8Buffer[6];

    //
    // Reset the LIS2DH12 Accel
    //
    sRegBuffer.bytes[0] = 0x00; // CTRL_REG1 (default)
    sRegBuffer.bytes[1] = 0x00; // CTRL_REG2 (default)
    sRegBuffer.bytes[2] = 0x00; // CTRL_REG3 (default)
    sRegBuffer.bytes[3] = 0x00; // CTRL_REG4 (default)
    sRegBuffer.bytes[4] = 0x00; // CTRL_REG5 (default)
    sRegBuffer.bytes[5] = 0x00; // CTRL_REG6 (default)

    am_devices_lis2dh12_reg_block_write(psDevice, AM_DEVICES_LIS2DH12_CTRL_REG1,
                                       sRegBuffer.words, 6, 0);

    //
    // Clear any pending interrupts
    //
    am_devices_lis2dh12_reg_read(psDevice, AM_DEVICES_LIS2DH12_INT1_SRC);
    am_devices_lis2dh12_reg_read(psDevice, AM_DEVICES_LIS2DH12_INT2_SRC);

    //
    // Get FIFO count
    //
    ui8FifoCnt = am_devices_lis2dh12_reg_read(psDevice, AM_DEVICES_LIS2DH12_SRC_REG) & 0x1F;

    //
    // Clear the FIFO
    //
    while (ui8FifoCnt--)
    {
        am_devices_lis2dh12_reg_block_read(psDevice,
                                  AM_DEVICES_LIS2DH12_OUT_X_L,
                                  ui8Buffer,
                                  6,
                                  NULL);
    }
}

//*****************************************************************************
//
//! @brief Get the device ID
//!
//! @param psDevice is a pointer to a device structure describing the LIS2DH12
//!
//! This function reads the device ID register and returns the result. The
//! LIS2DH12 should return 0x33 when functioning correctly.
//!
//! @return ID value.
//
//*****************************************************************************
uint8_t
am_devices_lis2dh12_device_id_get(am_devices_lis2dh12_t *psDevice)
{
    //
    // Read the WHO_AM_I register and return the result.
    //
    return am_devices_lis2dh12_reg_read(psDevice, AM_DEVICES_LIS3MDL_WHO_AM_I);
}

//*****************************************************************************
//
//! @brief Configures the LIS2DH12 for operation.
//!
//! @param psDevice is a pointer to a device structure describing the LIS2DH12.
//!
//! This function performs a basic, default configuration for the LIS2DH12.
//!
//! @return None.
//
//*****************************************************************************
void
am_devices_lis2dh12_config(am_devices_lis2dh12_t *psDevice)
{
    am_hal_lis2dh12_regs(6) sRegBuffer;

    //
    // Reset the device
    //
    am_devices_lis2dh12_reset(psDevice);

    //
    // Default initialization:
    // (100Hz,Enable XYZ axis, Normal Mode)
    // (Default filters)
    // (Little-Endian, +/-2g, Normal Mode)
    // (FIFO Disabled)
    //
    sRegBuffer.bytes[0] = 0x57;     // CTRL_REG1
    sRegBuffer.bytes[1] = 0x00;     // CTRL_REG2
    sRegBuffer.bytes[2] = 0x00;     // CTRL_REG3
    sRegBuffer.bytes[3] = 0x00;     // CTRL_REG4
    sRegBuffer.bytes[4] = 0x00;     // CTRL_REG5
    sRegBuffer.bytes[5] = 0x00;     // CTRL_REG6

    //
    // Configure the LIS2DH12 sensor according to samples to collect
    // before interrupt.  If only greater than one sample is
    // requested, configure sensor for FIFO operation.  If only one
    // sample, configure device to interrupt on every sample ready.
    //
    if (psDevice->ui32Samples > 1)
    {
        //
        // Configure the fifo watermark.
        //
        am_devices_lis2dh12_reg_write(psDevice,
                                     AM_DEVICES_LIS2DH12_FIFO_CTRL_REG,
                                     (0x80 | (psDevice->ui32Samples & 0x1F)));

        sRegBuffer.bytes[2] = 0x04;     // INT1 as watermark
        sRegBuffer.bytes[4] = 0x40;     // Enable FIFO
    }
    else
    {
        sRegBuffer.bytes[2] = 0x10;     // INT1 as data-ready
        sRegBuffer.bytes[4] = 0x00;     // Disable FIFO
    }

    //
    // Write the control registers to the device as one solid block write.
    //
    am_devices_lis2dh12_reg_block_write(psDevice,
                                        AM_DEVICES_LIS2DH12_CTRL_REG1,
                                        sRegBuffer.words, 6, 0);

}

//*****************************************************************************
//
//! @brief Retrieves the most recent sample from the LIS2DH12.
//!
//! @param psDevice is a pointer to a device structure describing the LIS2DH12.
//! @param psData is the location where this function will place the data.
//! @param pfnCallback is an optional callback function.
//!
//! This function reads the Magnetometer sample registers in the LIS2DH12, and
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
am_devices_lis2dh12_sample_get(am_devices_lis2dh12_t *psDevice,
                               uint32_t *psData,
                               am_hal_iom_callback_t pfnCallback)
{
    //
    // Read the lis2dh12 registers as a single block.
    //
    am_devices_lis2dh12_reg_block_read(psDevice,
                                       AM_DEVICES_LIS2DH12_OUT_X_L,
                                       psData,
                                       (psDevice->ui32Samples * 6),
                                       pfnCallback);
}

//*****************************************************************************
//
//! @brief Reads an internal register in the LIS2DH12.
//!
//! @param psDevice is a pointer to a device structure describing the LIS2DH12.
//! @param ui32Register is the address of the register to read.
//! @param ui32Value is the value to read to the register.
//!
//! This function performs a read to an LIS2DH12 register over the serial bus.
//!
//! @return
//
//*****************************************************************************
uint8_t
am_devices_lis2dh12_reg_read(am_devices_lis2dh12_t *psDevice,
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
//! @brief Reads a block of internal registers in the LIS2DH12.
//!
//! @param psDevice is a pointer to a device structure describing the LIS2DH12.
//! @param ui32StartRegister is the address of the first register to read.
//! @param pui32Values is the byte-packed array where the read data will go.
//! @param ui32NumBytes is the total number of 8-bit registers to read.
//! @param pfnCallback is an optional callback function pointer.
//!
//! This function performs a read to a block of LIS2DH12 registers over the
//! serial bus. If the \e pfnCallback parameter is nonzero, this function will
//! use the am_hal_iom_spi_read_nb() function as the underlying interface, and
//! \e pfnCallback will be provided to the HAL as the IOM callback function.
//! Otherwise, the spi read will be polled.
//!
//! @return
//
//*****************************************************************************
void
am_devices_lis2dh12_reg_block_read(am_devices_lis2dh12_t *psDevice,
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
    if (pfnCallback)
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
//! @brief Writes an internal register in the LIS2DH12.
//!
//! @param psDevice is a pointer to a device structure describing the LIS2DH12.
//! @param ui32Register is the address of the register to write.
//! @param ui32Value is the value to write to the register.
//!
//! This function performs a write to an LIS2DH12 register over the serial bus.
//!
//! @return
//
//*****************************************************************************
void
am_devices_lis2dh12_reg_write(am_devices_lis2dh12_t *psDevice,
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
//! @brief Writes a block of internal registers in the LIS2DH12.
//!
//! @param psDevice is a pointer to a device structure describing the LIS2DH12.
//! @param ui32StartRegister is the address of the first register to write.
//! @param pui32Values is the byte-packed array of data to write.
//! @param ui32NumBytes is the total number of registers to write.
//! @param pfnCallback is an optional callback function pointer.
//!
//! This function performs a write to a block of LIS2DH12 registers over the
//! serial bus. If the \e pfnCallback parameter is nonzero, this function will
//! use the am_hal_iom_spi_write_nb() function as the underlying interface, and
//! \e pfnCallback will be provided to the HAL as the IOM callback function.
//! Otherwise, the spi write will be polled.
//!
//! @return
//
//*****************************************************************************
void
am_devices_lis2dh12_reg_block_write(am_devices_lis2dh12_t *psDevice,
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
    if (pfnCallback)
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
