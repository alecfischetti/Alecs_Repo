//*****************************************************************************
//
//! @file am_devices_amx8x5.h
//!
//! @brief Driver to interface with the AMx8x5 RTC.
//!
//! These functions implement the AMx8x5 support routines for use on Ambiq
//! Micro MCUs.
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

#ifndef AM_DEVICES_AMX8X5_H
#define AM_DEVICES_AMX8X5_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Register Defines.
//
//*****************************************************************************
#define AM_DEVICES_AMX8X5_HUNDREDTHS                0x00
#define AM_DEVICES_AMX8X5_YEARS                     0x06
#define AM_DEVICES_AMX8X5_ALARM_HUNDRS              0x08
#define AM_DEVICES_AMX8X5_STATUS                    0x0F
#define AM_DEVICES_AMX8X5_CONTROL_1                 0x10
#define AM_DEVICES_AMX8X5_CONTROL_2                 0x11
#define AM_DEVICES_AMX8X5_INT_MASK                  0x12
#define AM_DEVICES_AMX8X5_SQW                       0x13
#define AM_DEVICES_AMX8X5_CAL_XT                    0x14
#define AM_DEVICES_AMX8X5_CAL_RC_HI                 0x15
#define AM_DEVICES_AMX8X5_CAL_RC_LOW                0x16
#define AM_DEVICES_AMX8X5_SLEEP_CTRL                0x17
#define AM_DEVICES_AMX8X5_TIMER_CTRL                0x18
#define AM_DEVICES_AMX8X5_TIMER                     0x19
#define AM_DEVICES_AMX8X5_TIMER_INITIAL             0x1A
#define AM_DEVICES_AMX8X5_WDT                       0x1B
#define AM_DEVICES_AMX8X5_OSC_CONTROL               0x1C
#define AM_DEVICES_AMX8X5_OSC_STATUS                0x1D
#define AM_DEVICES_AMX8X5_CONFIG_KEY                0x1F
#define AM_DEVICES_AMX8X5_ACAL_FLT                  0x26
#define AM_DEVICES_AMX8X5_EXTENDED_ADDR             0x3F
#define AM_DEVICES_AMX8X5_RAM_START                 0x40

// Keys.
#define AM_DEVICES_AMX8X5_CONFIG_KEY_VAL            0xA1

//*****************************************************************************
//
// Device structure used for communication.
//
//*****************************************************************************

// defines to be used in am_devices_amx8x5_t structure.
#define AM_DEVICES_AMX8X5_MODE_SPI                  true
#define AM_DEVICES_AMX8X5_MODE_I2C                  false

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
}
am_devices_amx8x5_t;

//*****************************************************************************
//
// Structure used for time keeping.
//
//*****************************************************************************
typedef struct
{
    uint8_t ui8Hundredth;
    uint8_t ui8Second;
    uint8_t ui8Minute;
    uint8_t ui8Hour;
    uint8_t ui8Date;
    uint8_t ui8Weekday;
    uint8_t ui8Month;
    uint8_t ui8Year;
    uint8_t ui8Century;
    uint8_t ui8Mode;
} am_devices_amx8x5_time_t;

extern am_devices_amx8x5_time_t g_psTimeRegs;

//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************
extern void am_devices_amx8x5_reg_clear(am_devices_amx8x5_t *psDevice,
                                        uint8_t ui8Address, uint8_t ui8Mask);

extern void am_devices_amx8x5_reg_set(am_devices_amx8x5_t *psDevice,
                                      uint8_t ui8Address, uint8_t ui8Mask);

extern uint8_t am_devices_amx8x5_reg_read(am_devices_amx8x5_t *psDevice,
                            uint8_t ui8Register);

extern void am_devices_amx8x5_reg_block_read(am_devices_amx8x5_t *psDevice,
                                  uint8_t ui8StartRegister,
                                  uint32_t *pui32Values,
                                  uint32_t ui32NumBytes,
                                  am_hal_iom_callback_t pfnCallback);

extern void am_devices_amx8x5_reg_write(am_devices_amx8x5_t *psDevice,
                             uint8_t ui8Register, uint8_t ui8Value);

extern void am_devices_amx8x5_reg_block_write(am_devices_amx8x5_t *psDevice,
                                   uint8_t ui8StartRegister,
                                   uint32_t *pui32Values,
                                   uint32_t ui32NumBytes,
                                   am_hal_iom_callback_t pfnCallback);
extern void am_devices_amx8x5_reset(am_devices_amx8x5_t *psDevice);
extern void am_devices_amx8x5_time_get(am_devices_amx8x5_t *psDevice);
extern void am_devices_amx8x5_time_set(am_devices_amx8x5_t *psDevice,
                                       uint8_t ui8Protect);

extern void am_devices_amx8x5_cal_set(am_devices_amx8x5_t *psDevice,
                                      uint8_t ui8Mode, int32_t iAdjust);

extern void am_devices_amx8x5_alarm_set(am_devices_amx8x5_t *psDevice,
                                        uint8_t ui8Repeat, uint8_t ui8IntMode,
                                        uint8_t ui8Pin);

extern void am_devices_amx8x5_countdown_set(am_devices_amx8x5_t *psDevice,
                                            uint8_t ui8Range, int32_t iPeriod,
                                            uint8_t ui8Repeat, uint8_t ui8Pin);

extern uint32_t am_devices_amx8x5_osc_sel(am_devices_amx8x5_t *psDevice,
                                          uint8_t ui8OSC);

extern void am_devices_amx8x5_sqw_set(am_devices_amx8x5_t *psDevice,
                                      uint8_t ui8SQFS, uint8_t ui8Pin);

extern uint32_t am_devices_amx8x5_sleep_set(am_devices_amx8x5_t *psDevice,
                                           uint8_t ui8Timeout, uint8_t ui8Mode);

extern void am_devices_amx8x5_watchdog_set(am_devices_amx8x5_t *psDevice,
                                           uint32_t ui8Period, uint8_t ui8Pin);

extern void am_devices_amx8x5_autocal_set(am_devices_amx8x5_t *psDevice,
                                          uint8_t ui8Period);

extern uint8_t am_devices_amx8x5_ext_address_get(am_devices_amx8x5_t *psDevice,
                                                 uint8_t ui8Address);

extern uint8_t am_devices_amx8x5_ram_read(am_devices_amx8x5_t *psDevice,
                                          uint8_t ui8Address);

extern void am_devices_amx8x5_ram_write(am_devices_amx8x5_t *psDevice,
                                        uint8_t ui8Address, uint8_t ui8Data);

#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_AMX8X5_H
