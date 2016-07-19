//*****************************************************************************
//
//! @file am_devices_ak09911.c
//!
//! @brief Generic ak09911 driver.
//!
//! @addtogroup devices External Device Control Library
//! @addtogroup ak09911 SPI Device Control for the AK09911 External Magnetometer
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
#include "am_devices_ak09911.h"
#include "am_util.h"

//*****************************************************************************
//
//! @brief Reset the AK09911 mag
//!
//! @param psDevice is a pointer to a device structure describing the AK09911
//!
//! This function resets the AK09911 mag.
//!
//! @return None.
//
//*****************************************************************************
void
am_devices_ak09911_reset(am_devices_ak09911_t *psDevice)
{
    //
    // Reset the AK09911 Mag.
    //
    am_devices_ak09911_reg_write(psDevice, AM_DEVICES_AK09911_CNTL3, 0x01);
}

//*****************************************************************************
//
//! @brief Get the device ID
//!
//! @param psDevice is a pointer to a device structure describing the AK09911
//!
//! This function reads the device ID register and returns the result. The
//! AK09911 should return 0x3D when functioning correctly.
//!
//! @return ID value.
//
//*****************************************************************************
uint16_t
am_devices_ak09911_device_id_get(am_devices_ak09911_t *psDevice)
{
    //
    // Read the WHO_AM_I register and return the result.
    //
    return (am_devices_ak09911_reg_read(psDevice, AM_DEVICES_AK09911_WIA2) << 8) |
                  am_devices_ak09911_reg_read(psDevice, AM_DEVICES_AK09911_WIA1);
}

//*****************************************************************************
//
//! @brief Configures the AK09911 for operation.
//!
//! @param psDevice is a pointer to a device structure describing the AK09911.
//!
//! This function performs a basic, default configuration for the AK09911.
//!
//! @return None.
//
//*****************************************************************************
void
am_devices_ak09911_config(am_devices_ak09911_t *psDevice)
{
    am_hal_ak09911_regs(6) sRegBuffer;

    //
    // Reset the device.
    //
    am_devices_ak09911_reset(psDevice);
    am_util_delay_ms(1);

    sRegBuffer.bytes[0] = 0x1;     // CTRL_REG2 (continuous mode 0)

    //
    // Write the control registers to the device as one solid block write.
    //
    am_devices_ak09911_reg_write(psDevice, AM_DEVICES_AK09911_CNTL2,
                                 sRegBuffer.bytes[0]);

    //
    // Clear out any old data that might be in the mag.
    //
    am_devices_ak09911_sample_get(psDevice, sRegBuffer.words, 0);
}

//*****************************************************************************
//
//! @brief Triggers the next sample from the AK09911
//!
//! @param psDevice is a pointer to a device structure describing the AK09911.
//!
//! @return None.
//
//*****************************************************************************
void
am_devices_ak09911_sample_trigger(am_devices_ak09911_t *psDevice)
{
    am_devices_ak09911_reg_write(psDevice, AM_DEVICES_AK09911_CNTL2, 0x1);
}

//*****************************************************************************
//
//! @brief Retrieves the most recent sample from the AK09911.
//!
//! @param psDevice is a pointer to a device structure describing the AK09911.
//! @param psData is the location where this function will place the data.
//! @param pfnCallback is an optional callback function.
//!
//! This function reads the Magnetometer sample registers in the AK09911, and
//! places the resulting samples into the caller-supplied buffer. If a callback
//! function is supplied, this function will use the am_hal_iom_i2c_write_nb()
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
am_devices_ak09911_sample_get(am_devices_ak09911_t *psDevice, uint32_t *psData,
                              am_hal_iom_callback_t pfnCallback)
{
    uint8_t ui8Value;
    
    am_devices_ak09911_reg_write(psDevice, AM_DEVICES_AK09911_CNTL2, 0x1);
    
    while(1)
    {
        ui8Value = am_devices_ak09911_reg_read(psDevice, AM_DEVICES_AK09911_ST1);
        
        if (ui8Value & 0x1) break;
        if (ui8Value & 0x2) am_devices_ak09911_reg_read(psDevice, AM_DEVICES_AK09911_ST2);
    }
    
    //
    // Read the magnetometer registers as a single block.
    //
    am_devices_ak09911_reg_block_read(psDevice, AM_DEVICES_AK09911_OUT_X_L,
                                      psData, 6, pfnCallback);
}

//*****************************************************************************
//
//! @brief Reads an internal register in the AK09911.
//!
//! @param psDevice is a pointer to a device structure describing the AK09911.
//! @param ui32Register is the address of the register to read.
//! @param ui32Value is the value to read to the register.
//!
//! This function performs a read to an AK09911 register over the serial bus.
//!
//! @return
//
//*****************************************************************************
uint8_t
am_devices_ak09911_reg_read(am_devices_ak09911_t *psDevice,
                            uint8_t ui8Register)
{
    uint8_t ui8Offset;
    am_hal_iom_buffer(1) sData;

    //
    // Build the data buffer.
    //
    ui8Offset = ui8Register;

    //
    // Send the read to the bus using the polled API.
    //
    am_hal_iom_i2c_read(psDevice->ui32IOMModule, psDevice->ui32Address,
                        sData.words, 1, AM_HAL_IOM_OFFSET(ui8Offset));

    //
    // Return the retrieved data.
    //
    return sData.bytes[0];
}

//*****************************************************************************
//
//! @brief Reads a block of internal registers in the AK09911.
//!
//! @param psDevice is a pointer to a device structure describing the AK09911.
//! @param ui32StartRegister is the address of the first register to read.
//! @param pui32Values is the byte-packed array where the read data will go.
//! @param ui32NumBytes is the total number of 8-bit registers to read.
//! @param pfnCallback is an optional callback function pointer.
//!
//! This function performs a read to a block of AK09911 registers over the
//! serial bus. If the \e pfnCallback parameter is nonzero, this function will
//! use the am_hal_iom_i2c_read_nb() function as the underlying interface, and
//! \e pfnCallback will be provided to the HAL as the IOM callback function.
//! Otherwise, the SPI read will be polled.
//!
//! @return
//
//*****************************************************************************
void
am_devices_ak09911_reg_block_read(am_devices_ak09911_t *psDevice,
                                  uint8_t ui8StartRegister,
                                  uint32_t *pui32Values,
                                  uint32_t ui32NumBytes,
                                  am_hal_iom_callback_t pfnCallback)
{
    uint8_t ui8Offset;
    ui8Offset = ui8StartRegister;

    //
    // Check to see if the callback pointer is valid.
    //
    if(pfnCallback)
    {
        //
        // If so, use a non-blocking call with a callback.
        //
        am_hal_iom_i2c_read_nb(psDevice->ui32IOMModule,
                               psDevice->ui32Address,
                               pui32Values, ui32NumBytes,
                               AM_HAL_IOM_OFFSET(ui8Offset),
                               pfnCallback);
    }
    else
    {
        //
        // Otherwise, use a polled call.
        //
        am_hal_iom_i2c_read(psDevice->ui32IOMModule,
                            psDevice->ui32Address,
                            pui32Values, ui32NumBytes,
                            AM_HAL_IOM_OFFSET(ui8Offset));
    }
}

//*****************************************************************************
//
//! @brief Writes an internal register in the AK09911.
//!
//! @param psDevice is a pointer to a device structure describing the AK09911.
//! @param ui32Register is the address of the register to write.
//! @param ui32Value is the value to write to the register.
//!
//! This function performs a write to an AK09911 register over the serial bus.
//!
//! @return
//
//*****************************************************************************
void
am_devices_ak09911_reg_write(am_devices_ak09911_t *psDevice,
                             uint8_t ui8Register, uint8_t ui8Value)
{
    uint8_t ui8Offset;
    am_hal_iom_buffer(1) sData;

    //
    // Build the data buffer.
    //
    ui8Offset = ui8Register;
    sData.bytes[0] = ui8Value;

    //
    // Send the write to the bus using the polled API.
    //
    am_hal_iom_i2c_write(psDevice->ui32IOMModule, psDevice->ui32Address,
                         sData.words, 1, AM_HAL_IOM_OFFSET(ui8Offset));
}

//*****************************************************************************
//
//! @brief Writes a block of internal registers in the AK09911.
//!
//! @param psDevice is a pointer to a device structure describing the AK09911.
//! @param ui32StartRegister is the address of the first register to write.
//! @param pui32Values is the byte-packed array of data to write.
//! @param ui32NumBytes is the total number of registers to write.
//! @param pfnCallback is an optional callback function pointer.
//!
//! This function performs a write to a block of AK09911 registers over the
//! serial bus. If the \e pfnCallback parameter is nonzero, this function will
//! use the am_hal_iom_i2c_write_nb() function as the underlying interface, and
//! \e pfnCallback will be provided to the HAL as the IOM callback function.
//! Otherwise, the spi write will be polled.
//!
//! @return
//
//*****************************************************************************
void
am_devices_ak09911_reg_block_write(am_devices_ak09911_t *psDevice,
                                   uint8_t ui8StartRegister,
                                   uint32_t *pui32Values,
                                   uint32_t ui32NumBytes,
                                   am_hal_iom_callback_t pfnCallback)
{
    uint8_t ui8Offset;
    ui8Offset = ui8StartRegister;

    //
    // Check to see if the callback pointer is valid.
    //
    if(pfnCallback)
    {
        //
        // If so, use a non-blocking call with a callback.
        //
        am_hal_iom_i2c_write_nb(psDevice->ui32IOMModule,
                                psDevice->ui32Address,
                                pui32Values, ui32NumBytes,
                                AM_HAL_IOM_OFFSET(ui8Offset),
                                pfnCallback);
    }
    else
    {
        //
        // Otherwise, use a polled call.
        //
        am_hal_iom_i2c_write(psDevice->ui32IOMModule,
                             psDevice->ui32Address,
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
