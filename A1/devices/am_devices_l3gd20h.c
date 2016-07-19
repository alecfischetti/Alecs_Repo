//*****************************************************************************
//
//! @file am_devices_l3gd20h.c
//!
//! @brief Driver to interface with the L3GD20H
//!
//! These functions implement the L3GD20H support routines for use on Ambiq
//! Micro MCUs.
//!
//! @addtogroup devices External Device Control Library
//! @addtogroup L3GD20H SPI Device Control for the L3GD20H External Gyro
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

#include "am_bsp.h"
#include "am_devices_l3gd20h.h"

//*****************************************************************************
//
//! @brief Reset the L3GD20H gyro
//!
//! @param psDevice is a pointer to a device structure describing the L3GD20H
//!
//! This function resets the L3GD20H gryo.
//!
//! @return None.
//
//*****************************************************************************
void
am_devices_l3gd20h_reset(am_devices_l3gd20h_t *psDevice)
{
    //
    // Reset the device.
    //
    am_devices_l3gd20h_reg_write(psDevice, AM_DEVICES_L3GD20H_LOW_ODR, 0x04);
}

//*****************************************************************************
//
//! @brief Get the device ID
//!
//! @param psDevice is a pointer to a device structure describing the L3GD20H
//!
//! This function reads the device ID register and returns the result. The
//! L3GD20H should return 0xD7 when functioning correctly.
//!
//! @return ID value.
//
//*****************************************************************************
uint8_t
am_devices_l3gd20h_device_id_get(am_devices_l3gd20h_t *psDevice)
{
    //
    // Read the WHO_AM_I register and return the result.
    //
    return am_devices_l3gd20h_reg_read(psDevice, AM_DEVICES_L3GD20H_WHO_AM_I);
}

//*****************************************************************************
//
//! @brief Initialize the L3GD20H
//!
//! @param psDevice is a pointer to a device structure describing the L3GD20H
//!
//! This function sends a few commands to initialize the GYRO for normal usage
//! depending on the number of sample to be collected.  If samples is greater
//! than one, the GYRO is configured to use the FIFO in data-stream mode where
//! the interrupts occurs on the threshold level. If only one sample is
//! requested, the GYRO is configured to use the interrupt as a data-ready
//! line.
//!
//! @note You can call this any time to reinitialize the device.
//!
//! @return None.
//
//*****************************************************************************
void
am_devices_l3gd20h_config(am_devices_l3gd20h_t *psDevice)
{
    am_hal_iom_buffer(4) sCommand;

    //
    // Reset the device.
    //
    am_devices_l3gd20h_reset(psDevice);

    //
    // Default initialization:
    // (Default filters)
    // (Little-Endian, 245dps, 4-wire SPI)
    // (FIFO Disabled)
    //
    sCommand.bytes[0] = 0x00;   // CTRL2
    sCommand.bytes[1] = 0x00;   // CTRL3
    sCommand.bytes[2] = 0x00;   // CTRL4
    sCommand.bytes[3] = 0x00;   // CTRL5

    //
    // If fifo size is greater than one configure fifo
    //
    if (psDevice->ui32Samples > 1)
    {
        //
        // Calculate the correct FIFO control word. The lower 5 bits are the fifo
        // threshold, and the upper three are the fifo operating mode. Take the
        // user-supplied threshold, and put the fifo in "dynamic stream" mode.
        //
        am_devices_l3gd20h_reg_write(psDevice,
                                     AM_DEVICES_L3GD20H_FIFO_CTRL,
                                     (0xC0 | (psDevice->ui32Samples & 0x1F)));

        sCommand.bytes[1] = 0x04;   // INT2 on FIFO Threshold
        sCommand.bytes[3] = 0x40;   // Enable FIFO
    }
    else
    {
        sCommand.bytes[1] = 0x08;   // INT2 on data-ready
    }

    //
    // Write the register values to the block of registers starting with CTRL2.
    //
    am_devices_l3gd20h_reg_block_write(psDevice, AM_DEVICES_L3GD20H_CTRL2,
                                       sCommand.words, 4, 0);

    //
    // Write CTRL1 to enable axis and start the sampling process.
    //
    am_devices_l3gd20h_reg_write(psDevice, AM_DEVICES_L3GD20H_CTRL1, 0x0F);
}

//*****************************************************************************
//
//! @brief Read a series of gyroscope samples from the L3GD20H.
//!
//! @param psDevice is a pointer to a device structure describing the L3GD20H
//! @param pui32Data is the buffer where samples will be placed
//! @param pfnCallback is an optional callback to be called on completion.
//!
//! This sample retrieves a set of gyroscope data from the L3GD20H. Each
//! triplet is 6 bytes long (2 bytes per axis). They are placed into the
//! caller-supplied array in a byte-packed format.
//!
//! If the \e pfnCallback parameter is non-null, this function will use the
//! am_hal_iom_spi_read_nb() function as the transport mechanism. The
//! caller-supplied callback function will be passed to the IOM HAL, and it
//! will be called from the IOM interrupt handler when the last byte is
//! received. If \e pfnCallback is not provided, this function will use a
//! polled API instead.
//!
//! @return None.
//
//*****************************************************************************
void
am_devices_l3gd20h_sample_read(am_devices_l3gd20h_t *psDevice,
                               uint32_t *pui32Data,
                               am_hal_iom_callback_t pfnCallback)
{
    //
    // Start a block read from the beginning of the L3GD20H's FIFO read
    // location. The L3GD20H should handle the address increment and wrap
    // automatically.
    //
    am_devices_l3gd20h_reg_block_read(psDevice, AM_DEVICES_L3GD20H_OUT_X_L,
                                      pui32Data,
                                      (psDevice->ui32Samples * 6),
                                      pfnCallback);
}

//*****************************************************************************
//
//! @brief Get the state of the control registers from the L3GD20H Gyro.
//!
//! @param psDevice is a pointer to a device structure describing the L3GD20H
//! @param pui32Return is a buffer where the control register values will be
//! written.
//!
//! Gets the state of the control registers from the L3GD20H Gyro and returns
//! the 5 bytes.
//!
//! @note The register values will be retrieved in a byte-packed format. The
//! caller must ensure that the memory location references by the
//! \e pui32Return pointer has enough space for all 5 bytes.
//!
//! @return None.
//
//*****************************************************************************
void
am_devices_l3gd20h_ctrl_reg_state_get(am_devices_l3gd20h_t *psDevice,
                                      uint32_t *pui32Return)
{
    //
    // Read the control registers from the device as one solid block.
    //
    am_devices_l3gd20h_reg_block_read(psDevice, AM_DEVICES_L3GD20H_CTRL1,
                                      pui32Return, 5, 0);
}

//*****************************************************************************
//
//! @brief Reads an internal register in the L3GD20H.
//!
//! @param psDevice is a pointer to a device structure describing the L3GD20H.
//! @param ui8Register is the address of the register to read.
//!
//! This function performs a read to an L3GD20H register over the serial bus.
//!
//! @return Retrieved Data
//
//*****************************************************************************
uint8_t
am_devices_l3gd20h_reg_read(am_devices_l3gd20h_t *psDevice,
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
//! @brief Reads a block of internal registers in the L3GD20H.
//!
//! @param psDevice is a pointer to a device structure describing the L3GD20H.
//! @param ui8StartRegister is the address of the first register to read.
//! @param pui32Values is the byte-packed array where the read data will go.
//! @param ui32NumBytes is the total number of registers to read.
//! @param pfnCallback is an optional callback function pointer.
//!
//! This function performs a read to a block of L3GD20H registers over the
//! serial bus. If the \e pfnCallback parameter is nonzero, this function will
//! use the am_hal_iom_spi_read_nb() function as the underlying interface, and
//! \e pfnCallback will be provided to the HAL as the IOM callback function.
//! Otherwise, the spi read will be polled.
//!
//! @return None.
//
//*****************************************************************************
void
am_devices_l3gd20h_reg_block_read(am_devices_l3gd20h_t *psDevice,
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
//! @brief Writes an internal register in the L3GD20H.
//!
//! @param psDevice is a pointer to a device structure describing the L3GD20H.
//! @param ui8Register is the address of the register to write.
//! @param ui8Value is the value to write to the register.
//!
//! This function performs a write to an L3GD20H register over the serial bus.
//!
//! @return None.
//
//*****************************************************************************
void
am_devices_l3gd20h_reg_write(am_devices_l3gd20h_t *psDevice,
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
//! @brief Writes a block of internal registers in the L3GD20H.
//!
//! @param psDevice is a pointer to a device structure describing the L3GD20H.
//! @param ui8StartRegister is the address of the first register to write.
//! @param pui32Values is the byte-packed array of data to write.
//! @param ui32NumBytes is the total number of registers to write.
//! @param pfnCallback is an optional callback function pointer.
//!
//! This function performs a write to a block of L3GD20H registers over the
//! serial bus. If the \e pfnCallback parameter is nonzero, this function will
//! use the am_hal_iom_spi_write_nb() function as the underlying interface, and
//! \e pfnCallback will be provided to the HAL as the IOM callback function.
//! Otherwise, the spi write will be polled.
//!
//! @return None.
//
//*****************************************************************************
void
am_devices_l3gd20h_reg_block_write(am_devices_l3gd20h_t *psDevice,
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
