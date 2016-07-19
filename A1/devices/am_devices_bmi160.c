//*****************************************************************************
//
//! @file am_devices_bmi160.c
//!
//! @brief Driver to interface with the BMI160.
//!
//! These functions implement the BMI160 support routines for use on Ambiq
//! Micro MCUs.
//!
//! @addtogroup devices External Device Control Library
//! @addtogroup BMI160 SPI Device Control for the BMI160 External Accel/Gryo
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
#include "am_devices_bmi160.h"
#include "am_util_delay.h"

//*****************************************************************************
//
//! @brief Clear one or more bits.
//!
//! @param psDevice is a pointer to a device structure describing the BMI160.
//! @param ui8Address - Reg address.
//! @param ui8Mask - Bits to clear.
//!
//! This function clears one or more bits in the selected register, selected by
//! 1's in the mask.
//!
//! @return None.
//
//*****************************************************************************
void
am_devices_bmi160_reg_clear(const am_devices_bmi160_t *psDevice,
                            uint8_t ui8Address,
                            uint8_t ui8Mask)
{
    uint8_t ui8Temp;

    ui8Temp = am_devices_bmi160_reg_read(psDevice, ui8Address);
    ui8Temp &= ~ui8Mask;
    am_devices_bmi160_reg_write(psDevice, ui8Address, ui8Temp);
}

//*****************************************************************************
//
//! @brief Set one or more bits.
//!
//! @param psDevice is a pointer to a device structure describing the BMI160.
//! @param ui8Address - RTC address.
//! @param ui8Mask - Bits to set.
//!
//! This function sets one or more bits in the selected register, selected by
//! 1's in the mask.
//!
//! @return None.
//
//*****************************************************************************
void
am_devices_bmi160_reg_set(const am_devices_bmi160_t *psDevice,
                          uint8_t ui8Address,
                          uint8_t ui8Mask)
{
    uint8_t ui8Temp;

    ui8Temp = am_devices_bmi160_reg_read(psDevice, ui8Address);
    ui8Temp |= ui8Mask;
    am_devices_bmi160_reg_write(psDevice, ui8Address, ui8Temp);
}

//*****************************************************************************
//
//! @brief Reads an internal register in the BMI160.
//!
//! @param psDevice is a pointer to a device structure describing the BMI160.
//! @param ui32Register is the address of the register to read.
//! @param ui32Value is the value to read to the register.
//!
//! This function performs a read to an BMI160 register over the serial bus.
//!
//! @return
//
//*****************************************************************************
uint8_t
am_devices_bmi160_reg_read(const am_devices_bmi160_t *psDevice,
                           uint8_t ui8Register)
{
    uint8_t ui8Offset;
    am_hal_iom_buffer(1) sData;

    //
    // Build the SPI offset and the data buffer.
    //
    ui8Offset = ui8Register;

    //
    // If configured for SPI mode, use SPI read, otherwise use I2C read.
    //
    if (psDevice->bMode == AM_DEVICES_BMI160_MODE_SPI)
    {
        //
        // turn on the read bit in the offset value
        //
        ui8Offset = ui8Register | 0x80;

        //
        // Send the read to the bus using the polled API.
        //
        am_hal_iom_spi_read(psDevice->ui32IOMModule, psDevice->ui32ChipSelect,
                            sData.words, 1, AM_HAL_IOM_OFFSET(ui8Offset));
    }
    else
    {

        //
        // Send the write to the bus using the polled API.
        //
        am_hal_iom_i2c_read(psDevice->ui32IOMModule, psDevice->ui32Address,
                             sData.words, 1, AM_HAL_IOM_OFFSET(ui8Offset));
    }

    //
    // Return the retrieved data.
    //
    return sData.bytes[0];
}

//*****************************************************************************
//
//! @brief Writes an internal register in the BMI160.
//!
//! @param psDevice is a pointer to a device structure describing the BMI160.
//! @param ui32Register is the address of the register to write.
//! @param ui32Value is the value to write to the register.
//!
//! This function performs a write to an BMI160 register over the serial bus.
//!
//! @return
//
//*****************************************************************************
void
am_devices_bmi160_reg_write(const am_devices_bmi160_t *psDevice,
                            uint8_t ui8Register, uint8_t ui8Value)
{
    uint8_t ui8Offset;
    am_hal_iom_buffer(1) sData;

    //
    // Build the offset and the data buffer.
    //
    ui8Offset = ui8Register;
    sData.bytes[0] = ui8Value;

    //
    // If configured for SPI mode, use SPI read, otherwise use I2C read.
    //
    if (psDevice->bMode == AM_DEVICES_BMI160_MODE_SPI)
    {

        //
        // Send the write to the bus using the polled API.
        //
        am_hal_iom_spi_write(psDevice->ui32IOMModule, psDevice->ui32ChipSelect,
                             sData.words, 1, AM_HAL_IOM_OFFSET(ui8Offset));
    }
    else
    {

        //
        // Send the write to the bus using the polled API.
        //
        am_hal_iom_i2c_write(psDevice->ui32IOMModule, psDevice->ui32Address,
                             sData.words, 1, AM_HAL_IOM_OFFSET(ui8Offset));
    }

    //
    // Delay after write to guarantee we exceed the ~400us delay needed
    // when the device is in low power mode (default out of POR).
    //
    am_util_delay_ms(1);
}

//*****************************************************************************
//
//! @brief Reads a block of internal registers in the BMI160.
//!
//! @param psDevice is a pointer to a device structure describing the BMI160.
//! @param ui32StartRegister is the address of the first register to read.
//! @param pui32Values is the byte-packed array where the read data will go.
//! @param ui32NumBytes is the total number of registers to read.
//! @param pfnCallback is an optional callback function pointer.
//!
//! This function performs a read to a block of BMI160 registers over the
//! serial bus. If the \e pfnCallback parameter is nonzero, this function will
//! use the am_hal_iom_spi_read_nb() function as the underlying interface, and
//! \e pfnCallback will be provided to the HAL as the IOM callback function.
//! Otherwise, the SPI read will be polled.
//!
//! @return None
//
//*****************************************************************************
void
am_devices_bmi160_reg_block_read(const am_devices_bmi160_t *psDevice,
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
    ui8Offset = ui8StartRegister;

    //
    // If configured for SPI mode, use SPI read, otherwise use I2C read.
    //
    if (psDevice->bMode == AM_DEVICES_BMI160_MODE_SPI)
    {
        //
        // Make sure R/W bit is set for reading
        //
        ui8Offset = ui8StartRegister | 0x80;

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
    else
    {
        //
        // Check to see if the callback pointer is valid.
        //
        if (pfnCallback)
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
}

//*****************************************************************************
//
//! @brief Writes a block of internal registers in the BMI160.
//!
//! @param psDevice is a pointer to a device structure describing the BMI160.
//! @param ui32StartRegister is the address of the first register to write.
//! @param pui32Values is the byte-packed array of data to write.
//! @param ui32NumBytes is the total number of registers to write.
//! @param pfnCallback is an optional callback function pointer.
//!
//! This function performs a write to a block of BMI160 registers over the
//! serial bus. If the \e pfnCallback parameter is nonzero, this function will
//! use the am_hal_iom_spi_write_nb() function as the underlying interface, and
//! \e pfnCallback will be provided to the HAL as the IOM callback function.
//! Otherwise, the SPI write will be polled.
//!
//! @return None
//
//*****************************************************************************
void
am_devices_bmi160_reg_block_write(const am_devices_bmi160_t *psDevice,
                                  uint8_t ui8StartRegister,
                                  uint32_t *pui32Values,
                                  uint32_t ui32NumBytes,
                                  am_hal_iom_callback_t pfnCallback)
{
    uint8_t ui8Offset;

    //
    // Build the offset for writing a block of registers from the
    // user-supplied start point.
    //
    ui8Offset = ui8StartRegister;

    //
    // If configured for SPI mode, use SPI read, otherwise use I2C read.
    //
    if (psDevice->bMode == AM_DEVICES_BMI160_MODE_SPI)
    {
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
    else
    {
        //
        // Check to see if the callback pointer is valid.
        //
        if (pfnCallback)
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
}


//*****************************************************************************
//
//! @brief Soft Reset the BMI160
//!
//! @param psDevice is a pointer to a device structure describing the BMI160.
//!
//! This function forces a soft reset command to the BMI160.
//!
//! @return None
//
//*****************************************************************************
void
am_devices_bmi160_reset(const am_devices_bmi160_t *psDevice)
{
    //
    // Issue a soft reset.
    //
    am_devices_bmi160_reg_write(psDevice, AM_DEVICES_BMI160_CMD, 0xB6);
}


//*****************************************************************************
//
//! @brief Initializes the BMI160.
//!
//! @param psDevice is a pointer to a device structure describing the BMI160.
//!
//! This function initializes the BMI160.
//!
//! @return None
//
//*****************************************************************************
void
am_devices_bmi160_config(const am_devices_bmi160_t *psDevice)
{
    uint8_t ui8Status = 0;
    uint8_t ui8Attempts = 50;

    //
    // Reset the BMI160 sensor
    //
    am_devices_bmi160_reset(psDevice);

    //
    // Put accel and gyro in normal mode.
    //
    while (ui8Status != 0x14 && ui8Attempts--)
    {
        //
        // Place accel & gyro in normal mode and verify status
        //
        am_devices_bmi160_reg_write(psDevice, AM_DEVICES_BMI160_CMD, 0x11);
        am_devices_bmi160_reg_write(psDevice, AM_DEVICES_BMI160_CMD, 0x15);
        am_util_delay_ms(1);
        ui8Status = am_devices_bmi160_reg_read(psDevice,
                                               AM_DEVICES_BMI160_PMU_STATUS);
    }

    //
    // BMI160 not in correct power mode
    //
    if (!ui8Attempts)
    {
        return;
    }

    //
    // Read status register to clear it.
    //
    ui8Status = am_devices_bmi160_reg_read(psDevice,
                                           AM_DEVICES_BMI160_ERR_REG);

    //
    // Enable INT 1 output as active high
    //
    am_devices_bmi160_reg_write(psDevice,
                                AM_DEVICES_BMI160_INT_OUT_CTRL, 0x0A);
    //
    // Latch interrupts
    //
    am_devices_bmi160_reg_write(psDevice,
                                AM_DEVICES_BMI160_INT_LATCH, 0x0F);

    //
    // Map data-ready to INT 1
    //
    if (psDevice->ui32Samples > 1)
    {
        //
        // Enable accel & gyro axis in the FIFO
        //
        am_devices_bmi160_reg_write(psDevice,
                                    AM_DEVICES_BMI160_FIFO_CFG_1, 0xC0);

        //
        // Map INT1 to the fifo watermark interrupt
        am_devices_bmi160_reg_write(psDevice,
                                    AM_DEVICES_BMI160_INT_MAP_1, 0x40);

        //
        // Set the FIFO watermark
        //
        am_devices_bmi160_reg_write(psDevice,
                                    AM_DEVICES_BMI160_FIFO_CFG_0,
                                    (psDevice->ui32Samples & 0xFF) * 3);

        ui8Status = am_devices_bmi160_reg_read(psDevice,
                                           AM_DEVICES_BMI160_FIFO_CFG_0);


        //
        // Enable INT 1 as FIFO watermark
        //
        am_devices_bmi160_reg_write(psDevice,
                                    AM_DEVICES_BMI160_INT_EN_1, 0x40);

        //
        // FIFO Flush
        //
        am_devices_bmi160_reg_write(psDevice, AM_DEVICES_BMI160_CMD, 0xB0);


    }
    else
    {
        //
        // Map INT1 to the data-ready interrupt
        //
        am_devices_bmi160_reg_write(psDevice,
                                    AM_DEVICES_BMI160_INT_MAP_1, 0x80);

        //
        // Enable INT 1 as data-ready
        //
        am_devices_bmi160_reg_write(psDevice,
                                    AM_DEVICES_BMI160_INT_EN_1, 0x10);
    }
}

//*****************************************************************************
//
//! @brief Gets the BMI160 samples.
//!
//! @param psDevice is a pointer to a device structure describing the BMI160.
//! @param ui32NumSamples is the number of samples to get.
//! @param pui32ReturnBuffer is a pointer to the buffer to store return data.
//! @param am_devices_bmi160_callback_t optional callback.
//!
//! This function gets the BMI160 samples.
//!
//! @return None
//
//*****************************************************************************
void
am_devices_bmi160_sample_get(const am_devices_bmi160_t *psDevice,
                             uint32_t ui32NumSamples,
                             uint32_t *pui32ReturnBuffer,
                             am_devices_bmi160_callback_t pfnCallback)
{
    //
    // Read the registers as a single block.
    //
    if (psDevice->ui32Samples > 1)
    {
        am_devices_bmi160_reg_block_read(psDevice, AM_DEVICES_BMI160_FIFO_DATA,
                                        pui32ReturnBuffer,
                                        (ui32NumSamples * 12),
                                        pfnCallback);
    }
    else
    {
        am_devices_bmi160_reg_block_read(psDevice,
                                        AM_DEVICES_BMI160_DATA_GYR_X_L,
                                        pui32ReturnBuffer,
                                        12,
                                        pfnCallback);
    }
}


//*****************************************************************************
//
//! @brief Get the device ID.
//!
//! @param psDevice - Pointer to BMI160 device structure
//!
//! Gets the device ID and returns the 4 bytes. This function blocks.
//!
//! @return Device ID byte.
//
//*****************************************************************************
uint8_t
am_devices_bmi160_device_id_get(const am_devices_bmi160_t *psDevice)
{
    uint8_t ui8DeviceID;

    //
    // Enable the IOM for this operation.
    //
    am_hal_iom_enable(AM_BSP_BMI160_IOM);

    //
    // Go read the device ID.
    //
    ui8DeviceID = am_devices_bmi160_reg_read(psDevice,
                                            AM_DEVICES_BMI160_CHIPID);

    //
    // Disable the IOM to save power.
    //
    am_hal_iom_disable(AM_BSP_BMI160_IOM);

    //
    // Return the device ID.
    //
    return ui8DeviceID;
}

