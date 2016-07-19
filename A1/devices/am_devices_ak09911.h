//*****************************************************************************
//
//! @file am_devices_ak09911.h
//!
//! @brief Driver for the ST Microelectronics AK09911 magnetometer
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

#ifndef AM_DEVICES_AK09911_H
#define AM_DEVICES_AK09911_H

#ifdef __cpluspluso
extern "C"
{
#endif

//*****************************************************************************
//
// AK09911 Registers.
//
//*****************************************************************************
#define AM_DEVICES_AK09911_WIA1             0x00
#define AM_DEVICES_AK09911_WIA2             0x01
#define AM_DEVICES_AK09911_INFO             0x02
#define AM_DEVICES_AK09911_ST1              0x10
#define AM_DEVICES_AK09911_OUT_X_L          0x11
#define AM_DEVICES_AK09911_OUT_X_H          0x12
#define AM_DEVICES_AK09911_OUT_Y_L          0x13
#define AM_DEVICES_AK09911_OUT_Y_H          0x14
#define AM_DEVICES_AK09911_OUT_Z_L          0x15
#define AM_DEVICES_AK09911_OUT_Z_H          0x16
#define AM_DEVICES_AK09911_CNTL0            0x17 // Dummy
#define AM_DEVICES_AK09911_ST2              0x18
#define AM_DEVICES_AK09911_CNTL1            0x30 //Dummy
#define AM_DEVICES_AK09911_CNTL2            0x31
#define AM_DEVICES_AK09911_CNTL3            0x32
#define AM_DEVICES_AK09911_TEST             0x33
#define AM_DEVICES_AK09911_ASAX             0x60
#define AM_DEVICES_AK09911_ASAY             0x61
#define AM_DEVICES_AK09911_ASAZ             0x62

//*****************************************************************************
//
// Structure for holding information about the AK09911
//
//*****************************************************************************
typedef struct
{
    uint32_t ui32IOMModule;
    uint32_t ui32Address;
}
am_devices_ak09911_t;

//*****************************************************************************
//
// Buffer type to make reading ak09911 samples easier.
//
//*****************************************************************************
#define am_devices_ak09911_sample(n)                                          \
    union                                                                     \
    {                                                                         \
        uint32_t words[((3 * n) + 1) >> 1];                                   \
        int16_t samples[3 * n];                                               \
    }

//*****************************************************************************
//
// Useful types for handling data transfers to and from the AK09911
//
//*****************************************************************************
#define am_hal_ak09911_regs(n)              am_hal_iom_buffer(n)

//*****************************************************************************
//
// External function definitions.
//
//*****************************************************************************
extern void am_devices_ak09911_config(am_devices_ak09911_t *psDevice);

extern void am_devices_ak09911_sample_trigger(am_devices_ak09911_t *psDevice);
extern void am_devices_ak09911_sample_get(
                                         am_devices_ak09911_t *psDevice,
                                         uint32_t *psData,
                                         am_hal_iom_callback_t pfnCallback);

extern uint8_t am_devices_ak09911_reg_read(am_devices_ak09911_t *psDevice,
                                         uint8_t ui8Register);

extern void am_devices_ak09911_reg_block_read(
                                         am_devices_ak09911_t *psDevice,
                                         uint8_t ui8StartRegister,
                                         uint32_t *pui32Values, uint32_t
                                         ui32NumBytes,
                                         am_hal_iom_callback_t
                                         pfnCallback);

extern void am_devices_ak09911_reg_write(am_devices_ak09911_t *psDevice,
                                         uint8_t ui8Register, uint8_t ui8Value);

extern void am_devices_ak09911_reg_block_write(
                                         am_devices_ak09911_t *psDevice,
                                         uint8_t ui8StartRegister,
                                         uint32_t *pui32Values,
                                         uint32_t ui32NumBytes,
                                         am_hal_iom_callback_t pfnCallback);

extern uint16_t am_devices_ak09911_device_id_get(am_devices_ak09911_t *psDevice);

extern void am_devices_ak09911_reset(am_devices_ak09911_t *psDevice);

#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_AK09911_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
