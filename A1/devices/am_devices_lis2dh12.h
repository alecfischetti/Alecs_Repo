//*****************************************************************************
//
//! @file am_devices_lis2dh12.h
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

#ifndef AM_DEVICES_LIS2DH12_H
#define AM_DEVICES_LIS2DH12_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// LIS2DH12 Registers.
//
//*****************************************************************************
#define AM_DEVICES_LIS2DH12_MAX_SAMPLE_SIZE   32
#define AM_DEVICES_LIS2DH12_MAX_SAMPLE_BYTES  192

//*****************************************************************************
//
// LIS2DH12 Registers.
//
//*****************************************************************************
#define AM_DEVICES_LIS2DH12_STATUS_REG_AUX    0x07
#define AM_DEVICES_LIS2DH12_OUT_TEMP_L        0x0C
#define AM_DEVICES_LIS2DH12_OUT_TEMP_H        0x0D
#define AM_DEVICES_LIS2DH12_INT_COUNTER_REG   0x0E
#define AM_DEVICES_LIS2DH12_WHO_AM_I          0x0F
#define AM_DEVICES_LIS2DH12_CTRL_REG1         0x20
#define AM_DEVICES_LIS2DH12_CTRL_REG2         0x21
#define AM_DEVICES_LIS2DH12_CTRL_REG3         0x22
#define AM_DEVICES_LIS2DH12_CTRL_REG4         0x23
#define AM_DEVICES_LIS2DH12_CTRL_REG5         0x24
#define AM_DEVICES_LIS2DH12_CTRL_REG6         0x25
#define AM_DEVICES_LIS2DH12_CTRL_REG6         0x25
#define AM_DEVICES_LIS2DH12_REF_DCAPTURE      0x26
#define AM_DEVICES_LIS2DH12_STATUS_REG        0x27
#define AM_DEVICES_LIS2DH12_OUT_X_L           0x28
#define AM_DEVICES_LIS2DH12_OUT_X_H           0x29
#define AM_DEVICES_LIS2DH12_OUT_Y_L           0x2A
#define AM_DEVICES_LIS2DH12_OUT_Y_H           0x2B
#define AM_DEVICES_LIS2DH12_OUT_Z_L           0x2C
#define AM_DEVICES_LIS2DH12_OUT_Z_H           0x2D
#define AM_DEVICES_LIS2DH12_FIFO_CTRL_REG     0x2E
#define AM_DEVICES_LIS2DH12_SRC_REG           0x2F
#define AM_DEVICES_LIS2DH12_INT1_CFG          0x30
#define AM_DEVICES_LIS2DH12_INT1_SRC          0x31
#define AM_DEVICES_LIS2DH12_INT1_THS          0x32
#define AM_DEVICES_LIS2DH12_INT1_DURATION     0x33
#define AM_DEVICES_LIS2DH12_INT2_CFG          0x34
#define AM_DEVICES_LIS2DH12_INT2_SRC          0x35
#define AM_DEVICES_LIS2DH12_INT2_THS          0x36
#define AM_DEVICES_LIS2DH12_INT2_DURATION     0x37
#define AM_DEVICES_LIS2DH12_CLICK_CFG         0x38
#define AM_DEVICES_LIS2DH12_CLICK_SRC         0x39
#define AM_DEVICES_LIS2DH12_CLICK_THS         0x3A
#define AM_DEVICES_LIS2DH12_TIME_LIMIT        0x3B
#define AM_DEVICES_LIS2DH12_TIME_LATENCY      0x3C
#define AM_DEVICES_LIS2DH12_TIME_WINDOW       0x3D
#define AM_DEVICES_LIS2DH12_ACT_THS           0x3E
#define AM_DEVICES_LIS2DH12_ACT_DUR           0x3F

//*****************************************************************************
//
// Structure for holding information about the LIS2DH12
//
//*****************************************************************************
typedef struct
{
    //
    // Module number to use for IOM access.
    //
    uint32_t ui32IOMModule;

    //
    // Chip Select number to use for IOM access.
    //
    uint32_t ui32ChipSelect;

    //
    // Number of samples to collect before interrupt.  1 sample consists of
    // 6 bytes (2 bytes/axis)
    uint32_t ui32Samples;
}
am_devices_lis2dh12_t;

//*****************************************************************************
//
// Buffer type to make reading lis2dh12 samples easier.
//
//*****************************************************************************
#define am_devices_lis2dh12_sample(n)                                         \
    union                                                                     \
    {                                                                         \
        uint32_t words[((3 * n) + 1) >> 1];                                   \
        int16_t samples[3 * n];                                               \
    }

//*****************************************************************************
//
// Useful types for handling data transfers to and from the LIS2DH12
//
//*****************************************************************************
#define am_hal_lis2dh12_regs(n)              am_hal_iom_buffer(n)

//*****************************************************************************
//
// External function definitions.
//
//*****************************************************************************
extern void am_devices_lis2dh12_config(am_devices_lis2dh12_t *psDevice);

extern void am_devices_lis2dh12_sample_get(am_devices_lis2dh12_t *psDevice,
                                          uint32_t *psData,
                                          am_hal_iom_callback_t pfnCallback);

extern uint8_t am_devices_lis2dh12_reg_read(am_devices_lis2dh12_t *psDevice,
                                           uint8_t ui8Register);

extern void am_devices_lis2dh12_reg_block_read(am_devices_lis2dh12_t *psDevice,
                                              uint8_t ui8StartRegister,
                                              uint32_t *pui32Values, uint32_t
                                              ui32NumBytes,
                                              am_hal_iom_callback_t
                                              pfnCallback);

extern void am_devices_lis2dh12_reg_write(am_devices_lis2dh12_t *psDevice,
                                         uint8_t ui8Register, uint8_t ui8Value);

extern void am_devices_lis2dh12_reg_block_write(
                                         am_devices_lis2dh12_t *psDevice,
                                         uint8_t ui8StartRegister,
                                         uint32_t *pui32Values,
                                         uint32_t ui32NumBytes,
                                         am_hal_iom_callback_t pfnCallback);

extern uint8_t am_devices_lis2dh12_device_id_get(
                                         am_devices_lis2dh12_t *psDevice);

#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_LIS2DH12_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
