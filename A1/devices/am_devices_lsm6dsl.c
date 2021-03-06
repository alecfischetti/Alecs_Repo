//*****************************************************************************
//
//! @file am_devices_lsm6dsl.c
//!
//! @brief Driver for the ST Microelectronics LSM6DSL (iNEMO inertial module:
//!        6 AXIS INERTIAL MODULE, always-on 3D accelerometer and 3D gyroscope)
//!
//! @addtogroup devices External Device Control Library
//! @addtogroup lsm6dsl I2C Device Control for the LSM6DSL 
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
#include "am_devices_lsm6dsl.h"

//*****************************************************************************
//
//! @brief Configure the gpios to bit bang to lsm6dsl
//!
//! @param 
//!
//! This function configures gpio 11 and 12 to communicate with lsm6dsl.
//!
//! @return None.
//
//*****************************************************************************
void am_devices_lsm6dsl_config(void)
{
    am_hal_i2c_bit_bang_init(LSM6DSL_I2CSCL_GPIO,
                             LSM6DSL_I2CSDA_GPIO);
}


//*****************************************************************************
//
//! @brief Enable/disable the LSM3DSL, 3D accelerometer and 3D gyroscope
//!
//! @param 
//!
//! This function sets or clears the LSM3DSL sensor.
//!
//! @return None.
//
//*****************************************************************************

void am_devices_lsm6dsl_set_clear(uint8_t setClear)
{
    if(setClear)
    {
        //
        // Sensor Enable -- set gpio 35 power high
        //
        am_hal_gpio_out_bit_set(LSM6DSL_SENSOR_ENABLE_GPIO);
    }
    else
    {
        //
        // Sensor disable -- clear gpio 35 
        //
        am_hal_gpio_out_bit_clear(LSM6DSL_SENSOR_ENABLE_GPIO);
    }
}

//*****************************************************************************
//
//! @brief Reads an internal register in the LSM6DSL.
//!
//! @param ui8Register is the address of the register to read.
//! @param ui8Value is the value to read to the register.
//! @param ui32NumBytes is the value to read to the register.
//!
//! This function performs a read of an LSM6DSL register over the serial bus.
//!
//! @return
//
//*****************************************************************************
void am_devices_lsm6dsl_reg_read(uint8_t ui8Register, uint32_t ui32NumBytes, uint8_t *ui8Value)
{
     uint32_t lsm6dslAdd = (AM_DEVICES_LSM6DSL_I2C_ADDRESS << 1) | 1;
     am_hal_i2c_bit_bang_receive((uint8_t)lsm6dslAdd, ui32NumBytes, ui8Value, ui8Register, true);
}

//*****************************************************************************
//
//! @brief Writes to an internal register in the LSM6DSL.
//!
//! @param ui8Register is the address of the register to write.
//! @param ui8Value is the value to write to the register.
//! @param ui32NumBytes is the value to read to the register.
//!
//! This function performs a write to an LSM6DSL register over the serial bus.
//!
//! @return
//
//*****************************************************************************
void am_devices_lsm6dsl_reg_write(uint8_t ui8Register, 
                                  uint32_t ui32NumBytes, 
                                  uint8_t *ui8Value)
{
    uint32_t lsm6dslAdd = AM_DEVICES_LSM6DSL_I2C_ADDRESS << 1;
    am_hal_i2c_bit_bang_send((uint8_t)lsm6dslAdd, ui32NumBytes, ui8Value, ui8Register, true);
}

