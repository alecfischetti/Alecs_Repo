//*****************************************************************************
//
//! @file am_devices_bmi160.h
//!
//! @brief Driver to interface with the BMI160
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

#ifndef AM_DEVICES_BMI160_H
#define AM_DEVICES_BMI160_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// BMI160 Registers.
//
//*****************************************************************************
#define AM_DEVICES_BMI160_CHIPID                    0x00
#define AM_DEVICES_BMI160_ERR_REG                   0x02
#define AM_DEVICES_BMI160_PMU_STATUS                0x03
#define AM_DEVICES_BMI160_DATA_MAG_X_L              0x04
#define AM_DEVICES_BMI160_DATA_MAG_X_H              0x05
#define AM_DEVICES_BMI160_DATA_MAG_Y_L              0x06
#define AM_DEVICES_BMI160_DATA_MAG_Y_H              0x07
#define AM_DEVICES_BMI160_DATA_MAG_Z_L              0x08
#define AM_DEVICES_BMI160_DATA_MAG_Z_H              0x09
#define AM_DEVICES_BMI160_DATA_RHALL_L              0x0A
#define AM_DEVICES_BMI160_DATA_RHALL_H              0x0B
#define AM_DEVICES_BMI160_DATA_GYR_X_L              0x0C
#define AM_DEVICES_BMI160_DATA_GYR_X_H              0x0D
#define AM_DEVICES_BMI160_DATA_GYR_Y_L              0x0E
#define AM_DEVICES_BMI160_DATA_GYR_Y_H              0x0F
#define AM_DEVICES_BMI160_DATA_GYR_Z_L              0x10
#define AM_DEVICES_BMI160_DATA_GYR_Z_H              0x11
#define AM_DEVICES_BMI160_DATA_ACC_X_L              0x12
#define AM_DEVICES_BMI160_DATA_ACC_X_H              0x13
#define AM_DEVICES_BMI160_DATA_ACC_Y_L              0x14
#define AM_DEVICES_BMI160_DATA_ACC_Y_H              0x15
#define AM_DEVICES_BMI160_DATA_ACC_Z_L              0x16
#define AM_DEVICES_BMI160_DATA_ACC_Z_H              0x17
#define AM_DEVICES_BMI160_SENSORTIME_0              0x18
#define AM_DEVICES_BMI160_SENSORTIME_1              0x19
#define AM_DEVICES_BMI160_SENSORTIME_2              0x1A
#define AM_DEVICES_BMI160_STATUS                    0x1B
#define AM_DEVICES_BMI160_INT_STATUS_0              0x1C
#define AM_DEVICES_BMI160_INT_STATUS_1              0x1D
#define AM_DEVICES_BMI160_INT_STATUS_2              0x1E
#define AM_DEVICES_BMI160_INT_STATUS_3              0x1F
#define AM_DEVICES_BMI160_TEMP_L                    0x20
#define AM_DEVICES_BMI160_TEMP_H                    0x21
#define AM_DEVICES_BMI160_FIFO_LEN_L                0x22
#define AM_DEVICES_BMI160_FIFO_LEN_H                0x23
#define AM_DEVICES_BMI160_FIFO_DATA                 0x24
#define AM_DEVICES_BMI160_ACC_CONF                  0x40
#define AM_DEVICES_BMI160_ACC_RANGE                 0x41
#define AM_DEVICES_BMI160_GYR_CONF                  0x42
#define AM_DEVICES_BMI160_GYR_RANGE                 0x43
#define AM_DEVICES_BMI160_MAG_CONF                  0x44
#define AM_DEVICES_BMI160_FIFO_DOWNS                0x45
#define AM_DEVICES_BMI160_FIFO_CFG_0                0x46
#define AM_DEVICES_BMI160_FIFO_CFG_1                0x47
#define AM_DEVICES_BMI160_MAG_IF_0                  0x4B
#define AM_DEVICES_BMI160_MAG_IF_1                  0x4C
#define AM_DEVICES_BMI160_MAG_IF_2                  0x4D
#define AM_DEVICES_BMI160_MAG_IF_3                  0x4E
#define AM_DEVICES_BMI160_MAG_IF_4                  0x4F
#define AM_DEVICES_BMI160_INT_EN_0                  0x50
#define AM_DEVICES_BMI160_INT_EN_1                  0x51
#define AM_DEVICES_BMI160_INT_EN_2                  0x52
#define AM_DEVICES_BMI160_INT_OUT_CTRL              0x53
#define AM_DEVICES_BMI160_INT_LATCH                 0x54
#define AM_DEVICES_BMI160_INT_MAP_0                 0x55
#define AM_DEVICES_BMI160_INT_MAP_1                 0x56
#define AM_DEVICES_BMI160_INT_MAP_2                 0x57
#define AM_DEVICES_BMI160_INT_DATA_0                0x58
#define AM_DEVICES_BMI160_INT_DATA_1                0x59
#define AM_DEVICES_BMI160_INT_LOWHIGH_0             0x5A
#define AM_DEVICES_BMI160_INT_LOWHIGH_1             0x5B
#define AM_DEVICES_BMI160_INT_LOWHIGH_2             0x5C
#define AM_DEVICES_BMI160_INT_LOWHIGH_3             0x5D
#define AM_DEVICES_BMI160_INT_LOWHIGH_4             0x5E
#define AM_DEVICES_BMI160_INT_MOTION_0              0x5F
#define AM_DEVICES_BMI160_INT_MOTION_1              0x60
#define AM_DEVICES_BMI160_INT_MOTION_2              0x61
#define AM_DEVICES_BMI160_INT_MOTION_3              0x62
#define AM_DEVICES_BMI160_INT_TAP_0                 0x63
#define AM_DEVICES_BMI160_INT_TAP_1                 0x64
#define AM_DEVICES_BMI160_INT_ORIENT_0              0x65
#define AM_DEVICES_BMI160_INT_ORIENT_1              0x66
#define AM_DEVICES_BMI160_INT_FLAT_0                0x67
#define AM_DEVICES_BMI160_INT_FLAT_1                0x68
#define AM_DEVICES_BMI160_FOC_CONF                  0x69
#define AM_DEVICES_BMI160_CONF                      0x6A
#define AM_DEVICES_BMI160_IF_CONF                   0x6B
#define AM_DEVICES_BMI160_PMU_TRIG                  0x6C
#define AM_DEVICES_BMI160_SELF_TEST                 0x6D
#define AM_DEVICES_BMI160_NV_CONF                   0x70
#define AM_DEVICES_BMI160_OFFSET_0                  0x71
#define AM_DEVICES_BMI160_OFFSET_1                  0x72
#define AM_DEVICES_BMI160_OFFSET_2                  0x73
#define AM_DEVICES_BMI160_OFFSET_3                  0x74
#define AM_DEVICES_BMI160_OFFSET_4                  0x75
#define AM_DEVICES_BMI160_OFFSET_5                  0x76
#define AM_DEVICES_BMI160_OFFSET_6                  0x77
#define AM_DEVICES_BMI160_STEP_CNT_0                0x78
#define AM_DEVICES_BMI160_STEP_CNT_1                0x79
#define AM_DEVICES_BMI160_STEP_CONF_0               0x7A
#define AM_DEVICES_BMI160_STEP_CONF_1               0x7B
#define AM_DEVICES_BMI160_CMD                       0x7E


//*****************************************************************************
//
// Device structure used for communication.
//
//*****************************************************************************

// defines to be used in am_devices_bmi160_t structure.
#define AM_DEVICES_BMI160_MODE_SPI                  true
#define AM_DEVICES_BMI160_MODE_I2C                  false

typedef struct
{
    //
    // SPI or I2C mode.
    //
    bool bMode;

    //
    // Module number to use for IOM access.
    //
    uint32_t ui32IOMModule;

    //
    // Chip Select number to use for IOM access. (unused in I2C mode).
    //
    uint32_t ui32ChipSelect;

    //
    // Address for I2C communication. (unused in SPI mode)
    //
    uint32_t ui32Address;

    //
    // Number of samples to collect before interrupt.
    //
    uint32_t ui32Samples;
}
am_devices_bmi160_t;

//*****************************************************************************
//
// Function pointer used for callbacks.
//
//*****************************************************************************
typedef void (*am_devices_bmi160_callback_t)(void);

//*****************************************************************************
//
// Typedef to make sample/word conversion easier to deal with.
//
//*****************************************************************************
#define bmi160_sample_buffer(n)                                               \
    union                                                                     \
    {                                                                         \
        uint32_t words[((6 * n) + 1) >> 1];                                   \
        int16_t samples[6 * n];                                               \
    }

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************
extern void am_devices_bmi160_reg_clear(const am_devices_bmi160_t *psDevice,
                                        uint8_t ui8Address, uint8_t ui8Mask);

extern void am_devices_bmi160_reg_set(const am_devices_bmi160_t *psDevice,
                                      uint8_t ui8Address, uint8_t ui8Mask);

extern uint8_t am_devices_bmi160_reg_read(const am_devices_bmi160_t *psDevice,
                            uint8_t ui8Register);

extern void am_devices_bmi160_reg_block_read(const am_devices_bmi160_t *psDevice,
                                  uint8_t ui8StartRegister,
                                  uint32_t *pui32Values,
                                  uint32_t ui32NumBytes,
                                  am_hal_iom_callback_t pfnCallback);

extern void am_devices_bmi160_reg_write(const am_devices_bmi160_t *psDevice,
                             uint8_t ui8Register, uint8_t ui8Value);

extern void am_devices_bmi160_reg_block_write(const am_devices_bmi160_t *psDevice,
                                   uint8_t ui8StartRegister,
                                   uint32_t *pui32Values,
                                   uint32_t ui32NumBytes,
                                   am_hal_iom_callback_t pfnCallback);

extern void am_devices_bmi160_sample_get(const am_devices_bmi160_t *psDevice,
                                          uint32_t ui32NumSamples,
                                          uint32_t *pui32ReturnBuffer,
                                          am_devices_bmi160_callback_t pfnCallback);

extern void am_devices_bmi160_reset(const am_devices_bmi160_t *psDevice);

extern void am_devices_bmi160_config(const am_devices_bmi160_t *psDevice);

extern uint8_t am_devices_bmi160_device_id_get(const am_devices_bmi160_t *psDevice);



#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_BMI160_H
