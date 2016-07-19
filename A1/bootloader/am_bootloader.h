//*****************************************************************************
//
//! @file am_bootloader.h
//!
//! @brief
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
#include <stdint.h>
#include <stdbool.h>

#ifndef AM_BOOTLOADER_H
#define AM_BOOTLOADER_H

#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Macros
//
//*****************************************************************************
#define AM_BOOTLOADER_DISABLE_OVERRIDE_PIN  0xFFFFFFFF
#define AM_BOOTLOADER_OVERRIDE_HIGH         0x1
#define AM_BOOTLOADER_OVERRIDE_LOW          0x0

//*****************************************************************************
//
// Structure to keep track of boot image information.
//
//*****************************************************************************
typedef struct
{
    // Starting address where the image was linked to run.
    uint32_t *pui32LinkAddress;

    // Length of the executable image in bytes.
    uint32_t ui32NumBytes;

    // CRC-32 Value for the full image.
    uint32_t ui32CRC;

    // Override GPIO number. (Can be used to force a new image load)
    uint32_t ui32OverrideGPIO;

    // Polarity for the override pin.
    uint32_t ui32OverridePolarity;

    // Stack pointer location.
    uint32_t *pui32StackPointer;

    // Reset vector location.
    uint32_t *pui32ResetVector;

    // Is the image encrypted?
    bool bEncrypted;
}
am_bootloader_image_t;

//*****************************************************************************
//
// External function declarations.
//
//*****************************************************************************
extern uint32_t am_bootloader_crc32(const void *pvData, uint32_t ui32Length);
extern uint32_t am_bootloader_fast_crc32(const void *pvData, uint32_t ui32NumBytes);
extern void am_bootloader_partial_crc32(const void *pvData, uint32_t ui32NumBytes, uint32_t *pui32CRC);
extern bool am_bootloader_image_check(am_bootloader_image_t *psImage);
extern void am_bootloader_flag_page_update(am_bootloader_image_t *psImage, uint32_t *pui32FlagPage);

#ifdef gcc
extern void __attribute__((naked)) am_bootloader_image_run(am_bootloader_image_t *psImage);
#endif

#ifdef keil
extern __asm void am_bootloader_image_run(am_bootloader_image_t *psImage);
#endif

#ifdef iar
extern void am_bootloader_image_run(am_bootloader_image_t *psImage);
#endif

#ifdef __cplusplus
}
#endif

#endif // AM_BOOTLOADER_H
