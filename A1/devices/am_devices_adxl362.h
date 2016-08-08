//*****************************************************************************
//
//! @file am_devices_adxl362.h
//!
//! @brief Driver to interface with the ADXL362
//!
//! These functions implement the ADXL362 support routines for use on Ambiq
//! Micro MCUs.
//!
//! @addtogroup devices External Device Control Library
//! @addtogroup ADXL362 SPI Device Control for the ADXL362 External Accelerometer
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

#ifndef AM_DEVICES_ADXL362_H
#define AM_DEVICES_ADXL362_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Defines
//
//*****************************************************************************
#define AM_DEVICES_ADXL_IS_X_AXIS(sample)   ((sample & 0x00000001) != 0)

typedef enum
{
    AM_DEVICES_ADXL362_400HZ,
    AM_DEVICES_ADXL362_200HZ,
    AM_DEVICES_ADXL362_100HZ,
}eADXL362SampleRate;

typedef enum
{
    AM_DEVICES_ADXL362_2G,
    AM_DEVICES_ADXL362_4G,
    AM_DEVICES_ADXL362_8G,
}eADXL362Range;

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

    //
    // Sensor sample rate.
    // Valid values are:
    //    AM_DEVICES_ADXL362_100HZ
    //    AM_DEVICES_ADXL362_200HZ
    //    AM_DEVICES_ADXL362_400HZ
    //
    eADXL362SampleRate ui32SampleRate;

    //
    // Accelerometer measurement range.
    // Valid values are:
    //    AM_DEVICES_ADXL362_2G
    //    AM_DEVICES_ADXL362_4G
    //    AM_DEVICES_ADXL362_8G
    //
    eADXL362Range ui32Range;

    //
    // Half bandwidth mode?
    //
    bool bHalfBandwidth;

    //
    // Sync mode?
    //
    bool bSyncMode;
}
am_devices_adxl362_t;

//*****************************************************************************
//
// Typedef to make sample/word conversion easier to deal with.
//
//*****************************************************************************
#define am_devices_adxl362_sample(n)                                          \
    union                                                                     \
    {                                                                         \
        uint32_t words[((3 * n) + 1) >> 1];                                   \
        int16_t samples[3 * n];                                               \
    }

//*****************************************************************************
//
// Macro for retrieving the sign-extended magnitude of a 16-bit ADXL sample.
//
//*****************************************************************************
#define AM_DEVICES_ADXL362_VALUE(x)    (x ? (((int16_t) ((x) << 2)) / 4) : 0)

//*****************************************************************************
//
// Function pointer used for callbacks.
//
//*****************************************************************************
typedef void (*am_devices_adxl362_callback_t)(void);

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************
extern void am_devices_adxl362_config(const am_devices_adxl362_t *psDevice);

extern void am_devices_adxl362_reset(const am_devices_adxl362_t *psDevice);

extern void am_devices_adxl362_measurement_mode_set(const am_devices_adxl362_t *psDevice);

extern void am_devices_adxl362_fifo_depth_get(const am_devices_adxl362_t *psDevice,
                                              uint32_t *pui32Return);

extern void am_devices_adxl362_sample_get(const am_devices_adxl362_t *psDevice,
                                          uint32_t ui32NumSamples,
                                          uint32_t *pui32ReturnBuffer,
                                          am_devices_adxl362_callback_t pfnCallback);

extern void am_devices_adxl362_ctrl_reg_state_get(const am_devices_adxl362_t *psDevice,
                                                  uint32_t *pui32Return);

extern void am_devices_adxl362_device_id_get(const am_devices_adxl362_t *psDevice,
                                             uint32_t *pui32Return);

extern void am_devices_adxl362_reg_write(const am_devices_adxl362_t *psDevice,
                                         uint8_t *pui8RegValues,
                                         uint8_t ui8StartReg,
                                         uint32_t ui32Number);

extern void am_devices_adxl362_ctrl_reg_read(const am_devices_adxl362_t *psDevice,
                                             uint8_t *pui8Return,
                                             uint8_t ui8StartReg,
                                             uint32_t ui32Number);

#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_ADXL362_H

//*****************************************************************************
//
// End Doxygen group.
//! @}
//
//*****************************************************************************
