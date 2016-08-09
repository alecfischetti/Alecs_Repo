//*****************************************************************************
//
//! @file am_devices_lsm6dsl.h
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

#ifndef AM_DEVICES_LSM6DSL_H
#define AM_DEVICES_LSM6DSL_H

#ifdef __cplusplus
extern "C"
{
#endif

/***************** I2C Address *****************/
#define AM_DEVICES_LSM6DSL_I2C_ADDRESS    0xD5

/**************** Who am I  *******************/
#define LSM6DSL_ACC_GYRO_WHO_AM_I         0x6A

/*********** Sensor LSM6DSL Enable  ***********/
#define LSM6DSL_SENSOR_ENABLE_GPIO        35

/*********** LSM6DSL Bit Bang GPIOs  **********/
#define LSM6DSL_I2CSCL_GPIO               11
#define LSM6DSL_I2CSDA_GPIO               12

//*****************************************************************************
//
// LSM6DSL Registers.
//
//*****************************************************************************
#define AM_DEVICES_LSM6DSL_WHO_AM_I_REG     0x0F
#define AM_DEVICES_LSM6DSL_CTRL_REG1        0x20
#define AM_DEVICES_LSM6DSL_CTRL_REG2        0x21
#define AM_DEVICES_LSM6DSL_CTRL_REG3        0x22
#define AM_DEVICES_LSM6DSL_CTRL_REG4        0x23
#define AM_DEVICES_LSM6DSL_CTRL_REG5        0x24
#define AM_DEVICES_LSM6DSL_STATUS_REG       0x27
#define AM_DEVICES_LSM6DSL_OUT_X_L          0x28
#define AM_DEVICES_LSM6DSL_OUT_X_H          0x29
#define AM_DEVICES_LSM6DSL_OUT_Y_L          0x2A
#define AM_DEVICES_LSM6DSL_OUT_Y_H          0x2B
#define AM_DEVICES_LSM6DSL_OUT_Z_L          0x2C
#define AM_DEVICES_LSM6DSL_OUT_Z_H          0x2D
#define AM_DEVICES_LSM6DSL_TEMP_OUT_L       0x2E
#define AM_DEVICES_LSM6DSL_TEMP_OUT_H       0x2F
#define AM_DEVICES_LSM6DSL_INT_CFG          0x30
#define AM_DEVICES_LSM6DSL_INT_SRC          0x31
#define AM_DEVICES_LSM6DSL_INT_THS_L        0x32
#define AM_DEVICES_LSM6DSL_INT_THS_H        0x33
#define AM_DEVICES_LSM6DSL_WAKEUP_DUR       0x5C
#define AM_DEVICES_LSM6DSL_MASTER_CMD_CODE  0x60


//*****************************************************************************
//
// External function definitions.
//
//*****************************************************************************
extern void am_devices_lsm6dsl_config(void);
extern void am_devices_lsm6dsl_set_clear(uint8_t setClear);
void am_devices_lsm6dsl_reg_read(uint8_t ui8Register, uint32_t ui32NumBytes, uint8_t *ui8Value);

extern void am_devices_lsm6dsl_reg_write(uint8_t ui8Register, uint32_t ui32NumBytes, uint8_t *ui8Value);


#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_LSM6DSL_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
