//*****************************************************************************
//
//! @file am_devices_da14581.h
//!
//! @brief Support functions for the Dialog Semiconductor DA14581 BTLE radio.
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
#ifndef AM_DEVICES_DA14581_H
#define AM_DEVICES_DA14581_H

#ifdef __cplusplus
extern "C"
{
#endif


//*****************************************************************************
//
// Defines
//
//*****************************************************************************
#define AM_DEVICES_DA14581_UART_MODE         (0)
#define AM_DEVICES_DA14581_SPI_MODE          (1)

#define AM_DEVICES_DA14581_SPI_XPORT_CTS     (0x06)
#define AM_DEVICES_DA14581_SPI_XPORT_NOT_CTS (0x07)

//*****************************************************************************
//
// DA14581 device structure
//
//*****************************************************************************
typedef struct
{
    //
    // MODE UART vs IOM SPI
    //
    uint32_t ui32Mode;

    //
    // IOM Module #
    //
    uint32_t ui32IOMModule;

    //
    // IOM Chip Select NOTE: this driver uses GPIO for chip selects
    //
    uint32_t ui32IOMChipSelect;

    //
    // GPIO # for DA14581 DREADY signal
    //
    uint32_t ui32DREADY;
}
am_devices_da14581_t;


//*****************************************************************************
//
// External function definitions
//
//*****************************************************************************
extern bool am_devices_da14581_init(am_devices_da14581_t *psDevice);
extern bool am_devices_da14581_uart_boot(const uint8_t *pui8BinData,
                                         uint32_t ui32NumBytes);
extern void am_devices_da14581_spi_send(am_devices_da14581_t *psDevice,
                                        uint32_t options,
                                        uint8_t type, uint8_t *pData,
                                        uint16_t len);
extern void am_devices_da14581_spi_receive_bytes(
                                        am_devices_da14581_t *psDevice,
                                        uint32_t options,
                                        uint8_t *pData, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif // AM_DEVICES_DA14581_H

