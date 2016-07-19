//*****************************************************************************
//
//! @file am_devices_l3gd20h.h
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

#ifndef AM_DEVICES_L3GD20H_H
#define AM_DEVICES_L3GD20H_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Defines
//
//*****************************************************************************
#define AM_DEVICES_L3GD20H_WHO_AM_I         0x0F
#define AM_DEVICES_L3GD20H_CTRL1            0x20
#define AM_DEVICES_L3GD20H_CTRL2            0x21
#define AM_DEVICES_L3GD20H_CTRL3            0x22
#define AM_DEVICES_L3GD20H_CTRL4            0x23
#define AM_DEVICES_L3GD20H_CTRL5            0x24
#define AM_DEVICES_L3GD20H_REFERENCE        0x25
#define AM_DEVICES_L3GD20H_OUT_TEMP         0x26
#define AM_DEVICES_L3GD20H_STATUS           0x27
#define AM_DEVICES_L3GD20H_OUT_X_L          0x28
#define AM_DEVICES_L3GD20H_OUT_X_H          0x29
#define AM_DEVICES_L3GD20H_OUT_Y_L          0x2A
#define AM_DEVICES_L3GD20H_OUT_Y_H          0x2B
#define AM_DEVICES_L3GD20H_OUT_Z_L          0x2C
#define AM_DEVICES_L3GD20H_OUT_Z_H          0x2D
#define AM_DEVICES_L3GD20H_FIFO_CTRL        0x2E
#define AM_DEVICES_L3GD20H_FIFO_SRC         0x2F
#define AM_DEVICES_L3GD20H_IG_CFG           0x30
#define AM_DEVICES_L3GD20H_IG_SRC           0x31
#define AM_DEVICES_L3GD20H_IG_THS_XH        0x32
#define AM_DEVICES_L3GD20H_IG_THS_XL        0x33
#define AM_DEVICES_L3GD20H_IG_THS_YH        0x34
#define AM_DEVICES_L3GD20H_IG_THS_YL        0x35
#define AM_DEVICES_L3GD20H_IG_THS_ZH        0x36
#define AM_DEVICES_L3GD20H_IG_THS_ZL        0x37
#define AM_DEVICES_L3GD20H_IG_DURATION      0x38
#define AM_DEVICES_L3GD20H_LOW_ODR          0x39

//*****************************************************************************
//
// Device structure used for communication.
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
am_devices_l3gd20h_t;

//*****************************************************************************
//
// Buffer type to make reading l3gd20h samples easier.
//
//*****************************************************************************
#define am_devices_l3gd20h_sample(n)                                          \
    union                                                                     \
    {                                                                         \
        uint32_t words[((3 * n) + 1) >> 1];                                   \
        int16_t samples[3 * n];                                               \
    }

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************
extern void am_devices_l3gd20h_reg_block_write(
                                         am_devices_l3gd20h_t *psDevice,
                                         uint8_t ui8StartRegister,
                                         uint32_t *pui32Values,
                                         uint32_t ui32NumBytes,
                                         am_hal_iom_callback_t pfnCallback);

extern void am_devices_l3gd20h_reg_write(am_devices_l3gd20h_t *psDevice,
                                         uint8_t ui8Register, uint8_t ui8Value);

extern void am_devices_l3gd20h_reg_block_read(
                                         am_devices_l3gd20h_t *psDevice,
                                         uint8_t ui8StartRegister,
                                         uint32_t *pui32Values,
                                         uint32_t ui32NumBytes,
                                         am_hal_iom_callback_t pfnCallback);

extern uint8_t am_devices_l3gd20h_reg_read(
                                         am_devices_l3gd20h_t *psDevice,
                                         uint8_t ui8Register);

extern void am_devices_l3gd20h_sample_read(
                                         am_devices_l3gd20h_t *psDevice,
                                         uint32_t *pui32Data,
                                         am_hal_iom_callback_t pfnCallback);

extern void am_devices_l3gd20h_ctrl_reg_state_get(
                                         am_devices_l3gd20h_t *psDevice,
                                         uint32_t *pui32Return);

extern void am_devices_l3gd20h_config(am_devices_l3gd20h_t *psDevice);

extern uint8_t am_devices_l3gd20h_device_id_get(
                                         am_devices_l3gd20h_t *psDevice);

extern void am_devices_l3gd20h_reset(am_devices_l3gd20h_t *psDevice);

#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_L3GD20H_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
